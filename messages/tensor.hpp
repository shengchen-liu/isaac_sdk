/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/buffers.hpp"
#include "engine/core/buffers/shared_buffer.hpp"
#include "engine/core/logger.hpp"
#include "engine/core/tensor/tensor.hpp"
#include "engine/gems/math/float16.hpp"
#include "messages/element_type.hpp"
#include "messages/tensor.capnp.h"

namespace details {

// Helper struct for  generateing views based upon storage type.
// Partial template selection is used to make the choice.
template <typename K, size_t Order, size_t StrideIndex, typename BufferType,
          isaac::BufferStorageMode mode>
struct generate_tensor_view;

template <typename K, size_t Order, size_t StrideIndex, typename BufferType>
struct generate_tensor_view<K, Order, StrideIndex, BufferType, isaac::BufferStorageMode::Host> {
  isaac::TensorBase<K, Order, StrideIndex, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols,
      const std::array<size_t, Order>& sizes) {
    ASSERT(buffer.host_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CpuAlignedBufferConstView(buffer.host_buffer().const_view().begin(),
                                                      rows, cols * sizeof(K));
    return isaac::TensorBase<K, Order, StrideIndex, BufferType>(data_view, sizes);
  }
};

template <typename K, size_t Order, size_t StrideIndex, typename BufferType>
struct generate_tensor_view<K, Order, StrideIndex, BufferType, isaac::BufferStorageMode::Cuda> {
  isaac::TensorBase<K, Order, StrideIndex, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols,
      const std::array<size_t, Order>& sizes) {
    ASSERT(buffer.cuda_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CudaAlignedBufferConstView(buffer.cuda_buffer().const_view().begin(),
                                                       rows, cols * sizeof(K));
    return isaac::TensorBase<K, Order, StrideIndex, BufferType>(data_view, sizes);
  }
};

}  // namespace details

// Creates a tensor from a proto. Will print errors and return false if the tensor type is not
// compatible with the proto.
template <typename K, size_t Order, size_t StrideIndex, typename BufferType>
bool FromProto(::TensorProto::Reader reader, const std::vector<isaac::SharedAlignedBuffer>& buffers,
               isaac::TensorBase<K, Order, StrideIndex, BufferType>& tensor_view) {
  static_assert(
      !isaac::BufferTraits<BufferType>::kIsMutable && !isaac::BufferTraits<BufferType>::kIsOwning,
      "Invalid buffer type. Buffer cannot own or mutate data");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");

  // Check element type
  if (isaac::details::ElementType<K>::value != reader.getElementType()) {
    LOG_ERROR("Tensor element type does not match: actual=%d, expected=%d", reader.getElementType(),
              isaac::details::ElementType<K>::value);
    return false;
  }
  // resize the tensor
  auto proto_sizes = reader.getSizes();
  size_t expected = 1;
  std::array<size_t, Order> sizes;
  for (size_t i = 0; i < sizes.size(); i++) {
    expected *= proto_sizes[i];
    sizes[i] = proto_sizes[i];
  }
  auto stride_index = reader.getStrideIndex();

  // Get the buffer
  const uint32_t buffer_index = reader.getDataBufferIndex();
  if (buffer_index >= buffers.size()) {
    LOG_ERROR("Buffer index %u out of range (%zu): ", buffer_index, buffers.size());
    return false;
  }
  const isaac::SharedAlignedBuffer& buffer = buffers[buffer_index];
  // Check data storage mode.
  const isaac::BufferStorageMode mode = isaac::BufferTraits<BufferType>::kStorageMode;
  size_t size_provided = 0;
  if (mode == isaac::BufferStorageMode::Host) {
    size_provided = buffer.host_buffer().size();
  }
  if (mode == isaac::BufferStorageMode::Cuda) {
    size_provided = buffer.cuda_buffer().size();
  }
  const size_t size_expected = expected * sizeof(K);
  if (size_provided != size_expected) {
    LOG_ERROR(
        "Tensor data size does not match. Proto provides %zu bytes while image expected "
        "%zu bytes.",
        size_provided, size_expected);
    return false;
  }
  if (stride_index != StrideIndex) {
    LOG_ERROR(
        "Tensor data layouts do not match. Proto has stride index %d, while requested tensor has "
        "stride index %d",
        StrideIndex, stride_index);
    return false;
  }

  auto buffer_layout = isaac::ConvertTensorDimensionsToBufferDimensions(Order, StrideIndex, sizes);
  auto generator = details::generate_tensor_view<K, Order, StrideIndex, BufferType,
                                                 isaac::BufferTraits<BufferType>::kStorageMode>();
  isaac::TensorBase<K, Order, StrideIndex, BufferType> view =
      generator(buffer, buffer_layout[0], buffer_layout[1], sizes);
  if (view.scanline_stride() != reader.getScanlineStride()) {
    LOG_ERROR(
        "Data alignment mismatch. Data has scanline stride of %d but allocated buffer believes the "
        "stride to be %d",
        reader.getScanlineStride(), view.scanline_stride());
    return false;
  }
  tensor_view = view;
  return true;
}

// Writes an tensor to a TensorProto
template <typename K, size_t Order, size_t StrideIndex, typename BufferType>
void ToProto(isaac::TensorBase<K, Order, StrideIndex, BufferType> tensor,
             ::TensorProto::Builder builder, std::vector<isaac::SharedAlignedBuffer>& buffers) {
  static_assert(isaac::BufferTraits<BufferType>::kIsOwning, "Must own the data to send it.");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Set the elemenet ntype
  builder.setElementType(isaac::details::ElementType<K>::value);
  // Set the tensor sizes
  const auto sizes = tensor.dimensions();
  auto dims = builder.initSizes(sizes.size());
  for (size_t i = 0; i < sizes.size(); i++) {
    dims.set(i, sizes[i]);
  }

  // Add the tensor buffer to the list of buffers and make a copy of the tensor.
  builder.setDataBufferIndex(buffers.size());
  builder.setStrideIndex(StrideIndex);
  builder.setScanlineStride(tensor.scanline_stride());
  buffers.emplace_back(std::move(tensor.data()));
}
