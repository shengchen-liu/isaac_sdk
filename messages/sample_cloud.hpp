/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

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
#include "engine/core/sample_cloud/sample_cloud.hpp"
#include "engine/gems/math/float16.hpp"
#include "messages/element_type.hpp"
#include "messages/sample_cloud.capnp.h"

namespace details {

// Helper struct for  generateing views based upon storage type.
// Partial template selection is used to make the choice.
template <typename K, size_t Channels, typename BufferType, isaac::BufferStorageMode mode>
struct generate_sample_cloud_view;

template <typename K, size_t Channels, typename BufferType>
struct generate_sample_cloud_view<K, Channels, BufferType, isaac::BufferStorageMode::Host> {
  isaac::SampleCloudBase<K, Channels, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols, size_t samples) {
    ASSERT(buffer.host_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CpuAlignedBufferConstView(buffer.host_buffer().const_view().begin(),
                                                      rows, cols * sizeof(K));
    return isaac::SampleCloudBase<K, Channels, BufferType>(data_view, samples);
  }
};

template <typename K, size_t Channels, typename BufferType>
struct generate_sample_cloud_view<K, Channels, BufferType, isaac::BufferStorageMode::Cuda> {
  isaac::SampleCloudBase<K, Channels, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols, size_t samples) {
    ASSERT(buffer.cuda_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CudaAlignedBufferConstView(buffer.cuda_buffer().const_view().begin(),
                                                       rows, cols * sizeof(K));
    return isaac::SampleCloudBase<K, Channels, BufferType>(data_view, samples);
  }
};

}  // namespace details

template <typename K, size_t Channels, typename BufferType>
bool FromProto(::SampleCloudProto::Reader reader,
               const std::vector<isaac::SharedAlignedBuffer>& buffers,
               isaac::SampleCloudBase<K, Channels, BufferType>& sample_cloud_view) {
  static_assert(
      !isaac::BufferTraits<BufferType>::kIsMutable && !isaac::BufferTraits<BufferType>::kIsOwning,
      "Invalid buffer type. Buffer cannot own or mutate data");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Check element type
  if (isaac::details::ElementType<K>::value != reader.getElementType()) {
    LOG_ERROR("Sample Cloud element type does not match: actual=%d, expected=%d",
              reader.getElementType(), isaac::details::ElementType<K>::value);
    return false;
  }
  if (Channels != reader.getChannels()) {
    LOG_ERROR("Sample Cloud Channel counts do not match: actual = %d, expected = %d",
              reader.getChannels(), Channels);
    return false;
  }
  auto storageOrder = reader.getStorageOrder();
  if (storageOrder != ::StorageOrder::INTERLEAVED) {
    LOG_ERROR("Storage Orders do not match");
    return false;
  }

  size_t sample_count = reader.getSampleCount();
  size_t buffer_index = reader.getDataBufferIndex();
  const isaac::SharedAlignedBuffer& buffer = buffers[buffer_index];

  auto generator =
      details::generate_sample_cloud_view<K, Channels, BufferType,
                                          isaac::BufferTraits<BufferType>::kStorageMode>();
  isaac::SampleCloudBase<K, Channels, BufferType> view =
      generator(buffer, 1, Channels * sample_count, sample_count);

  sample_cloud_view = view;
  return true;
}

template <typename K, size_t Channels, typename BufferType>
void ToProto(isaac::SampleCloudBase<K, Channels, BufferType> sample_cloud,
             ::SampleCloudProto::Builder builder,
             std::vector<isaac::SharedAlignedBuffer>& buffers) {
  static_assert(isaac::BufferTraits<BufferType>::kIsOwning, "Must own the data to send it.");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Set the element type
  builder.setElementType(isaac::details::ElementType<K>::value);
  // Set the sample cloud meta data
  builder.setChannels(Channels);
  builder.setSampleCount(sample_cloud.size());
  builder.setStorageOrder(::StorageOrder::INTERLEAVED);
  // Store the sample cloud data as a buffer
  builder.setDataBufferIndex(buffers.size());
  buffers.emplace_back(std::move(sample_cloud.data()));
}

namespace details {

// Helper struct for  generateing views based upon storage type.
// Partial template selection is used to make the choice.
template <typename K, size_t Channels, typename BufferType, isaac::BufferStorageMode mode>
struct generate_sample_cloud_planar_view;

template <typename K, size_t Channels, typename BufferType>
struct generate_sample_cloud_planar_view<K, Channels, BufferType, isaac::BufferStorageMode::Host> {
  isaac::SampleCloudPlanarBase<K, Channels, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols, size_t samples) {
    ASSERT(buffer.host_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CpuAlignedBufferConstView(buffer.host_buffer().const_view().begin(),
                                                      rows, cols * sizeof(K));
    return isaac::SampleCloudPlanarBase<K, Channels, BufferType>(data_view, samples);
  }
};

template <typename K, size_t Channels, typename BufferType>
struct generate_sample_cloud_planar_view<K, Channels, BufferType, isaac::BufferStorageMode::Cuda> {
  isaac::SampleCloudPlanarBase<K, Channels, BufferType> operator()(
      const isaac::SharedAlignedBuffer& buffer, size_t rows, size_t cols, size_t samples) {
    ASSERT(buffer.cuda_buffer().size() >= rows * cols * sizeof(K),
           "Buffer does not have enough storage");
    auto data_view = isaac::CpuAlignedBufferConstView(buffer.cuda_buffer().const_view().begin(),
                                                      rows, cols * sizeof(K));
    return isaac::SampleCloudPlanarBase<K, Channels, BufferType>(data_view, samples);
  }
};

}  // namespace details
template <typename K, size_t Channels, typename BufferType>
bool FromProto(::SampleCloudProto::Reader reader,
               const std::vector<isaac::SharedAlignedBuffer>& buffers,
               isaac::SampleCloudPlanarBase<K, Channels, BufferType>& sample_cloud_view) {
  static_assert(
      !isaac::BufferTraits<BufferType>::kIsMutable && !isaac::BufferTraits<BufferType>::kIsOwning,
      "Invalid buffer type. Buffer cannot own or mutate data");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");

  // Check element type
  if (isaac::details::ElementType<K>::value != reader.getElementType()) {
    LOG_ERROR("Sample Cloud element type does not match: actual=%d, expected=%d",
              reader.getElementType(), isaac::details::ElementType<K>::value);
    return false;
  }
  if (Channels != reader.getChannels()) {
    LOG_ERROR("Sample Cloud Channel counts do not match: actual = %d, expected = %d",
              reader.getChannels(), Channels);
    return false;
  }
  auto storageOrder = reader.getStorageOrder();
  if (::StorageOrder::PLANAR != storageOrder) {
    LOG_ERROR("Storage Orders do not match");
    return false;
  }

  size_t sample_count = reader.getSampleCount();
  size_t buffer_index = reader.getDataBufferIndex();
  const isaac::SharedAlignedBuffer& buffer = buffers[buffer_index];
  auto generator =
      details::generate_sample_cloud_planar_view<K, Channels, BufferType,
                                                 isaac::BufferTraits<BufferType>::kStorageMode>();
  isaac::SampleCloudBase<K, Channels, BufferType> view =
      generator(buffer, Channels, sample_count, sample_count);

  sample_cloud_view = view;
  return true;
}

template <typename K, size_t Channels, typename BufferType>
void ToProto(isaac::SampleCloudPlanarBase<K, Channels, BufferType> sample_cloud,
             ::SampleCloudProto::Builder builder,
             std::vector<isaac::SharedAlignedBuffer>& buffers) {
  static_assert(isaac::BufferTraits<BufferType>::kIsOwning, "Must own the data to send it.");
  static_assert(isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Host ||
                    isaac::BufferTraits<BufferType>::kStorageMode == isaac::BufferStorageMode::Cuda,
                "Unknown buffer storage mode.");
  // Set the element type
  builder.setElementType(isaac::details::ElementType<K>::value);
  // Set the sample cloud meta data
  builder.setChannels(Channels);
  builder.setSampleCount(sample_cloud.size());
  builder.setStorageOrder(::StorageOrder::PLANAR);
  // Store the sample cloud data as a buffer
  builder.setDataBufferIndex(buffers.size());
  buffers.emplace_back(std::move(sample_cloud.data()));
}
