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
#include "engine/core/image/image.hpp"
#include "engine/core/logger.hpp"
#include "engine/gems/math/float16.hpp"
#include "messages/element_type.hpp"
#include "messages/image.capnp.h"

// Creates an image from a proto. Will print errors and return false if the image type is not
// compatible with the received image.
namespace details {

// Helper struct for  generateing views based upon storage type.
// Partial template selection is used to make the choice.
template <typename K, int N, typename BufferType, isaac::BufferStorageMode mode>
struct generate_image_view;

template <typename K, int N, typename BufferType>
struct generate_image_view<K, N, BufferType, isaac::BufferStorageMode::Host> {
  isaac::ImageBase<K, N, BufferType> operator()(const isaac::SharedAlignedBuffer& buffer,
                                                size_t rows, size_t cols) {
    ASSERT(buffer.host_buffer().size() >= rows * cols * sizeof(K) * N,
           "Buffer does not have enough storage");
    auto data_view = isaac::CpuAlignedBufferConstView(buffer.host_buffer().const_view().begin(),
                                                      rows, cols * N * sizeof(K));
    return isaac::ImageBase<K, N, BufferType>(data_view, rows, cols);
  }
};

template <typename K, int N, typename BufferType>
struct generate_image_view<K, N, BufferType, isaac::BufferStorageMode::Cuda> {
  isaac::ImageBase<K, N, BufferType> operator()(const isaac::SharedAlignedBuffer& buffer,
                                                size_t rows, size_t cols) {
    ASSERT(buffer.cuda_buffer().size() >= rows * cols * sizeof(K) * N,
           "Buffer does not have enough storage");
    auto data_view = isaac::CudaAlignedBufferConstView(buffer.cuda_buffer().const_view().begin(),
                                                       rows, cols * N * sizeof(K));
    return isaac::ImageBase<K, N, BufferType>(data_view, rows, cols);
  }
};

}  // namespace details
template <typename K, int N, typename BufferType>
bool FromProto(::ImageProto::Reader reader, const std::vector<isaac::SharedAlignedBuffer>& buffers,
               isaac::ImageBase<K, N, BufferType>& image_view) {
  static_assert(
      !isaac::BufferTraits<BufferType>::kIsMutable && !isaac::BufferTraits<BufferType>::kIsOwning,
      "Invalid buffer type. Buffer cannot own or mutate data");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");

  // Check element type
  if (isaac::details::ElementType<K>::value != reader.getElementType()) {
    LOG_ERROR("Image element type does not match: actual=%d, expected=%d", reader.getElementType(),
              isaac::details::ElementType<K>::value);
    return false;
  }
  // Validate the received proto and make sure that it is compatible with the desired image type
  const size_t rows = reader.getRows();
  const size_t cols = reader.getCols();
  const size_t channels = reader.getChannels();
  if (static_cast<int>(channels) != N) {
    LOG_ERROR(
        "Number of channels does not match. Proto provides %zu channels while image "
        "expected %d channels.",
        channels, N);
    return false;
  }
  // Get the buffer
  const uint32_t buffer_index = reader.getDataBufferIndex();
  if (buffer_index >= buffers.size()) {
    LOG_ERROR("Buffer index %u out of range (%zu): ", buffer_index, buffers.size());
    return false;
  }
  const isaac::SharedAlignedBuffer& buffer = buffers[buffer_index];
  // Check that the length of the received memory chunk matches

  const size_t size_provided =
      isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host
          ? buffer.host_buffer().size()
          : buffer.cuda_buffer().size();
  const size_t size_expected = rows * cols * channels * sizeof(K);
  if (size_provided != size_expected) {
    PANIC(
        "Image data size does not match. Buffer provides %zu bytes while image expected "
        "%zu bytes.",
        size_provided, size_expected);
    return false;
  }

  auto generator = details::generate_image_view<K, N, BufferType,
                                                isaac::BufferTraits<BufferType>::kStorageMode>();
  isaac::ImageBase<K, N, BufferType> view = generator(buffer, rows, cols);

  // Create an image view based on the buffer view
  image_view = view;
  return true;
}

// Writes an image to a proto. We take ownership of the image in this function and will add the
// buffer which is underlying the image to the buffer list.
template <typename K, int N, typename BufferType>
void ToProto(isaac::ImageBase<K, N, BufferType> image, ::ImageProto::Builder builder,
             std::vector<isaac::SharedAlignedBuffer>& buffers) {
  static_assert(isaac::BufferTraits<BufferType>::kIsOwning, "Must own the data to send it.");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Set the elemenet type
  builder.setElementType(isaac::details::ElementType<K>::value);
  // Set the image sizes and channels
  builder.setRows(image.rows());
  builder.setCols(image.cols());
  builder.setChannels(image.channels());
  // Add the image buffer to the list of buffers and disown the buffer from the image
  builder.setDataBufferIndex(buffers.size());
  ASSERT(image.hasTrivialStride(), "Image with stride not yet supported");
  // Check data storage mode.
  builder.setDataBufferIndex(buffers.size());
  buffers.emplace_back(std::move(image.data()));
}
