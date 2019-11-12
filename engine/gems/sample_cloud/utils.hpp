/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <array>
#include <initializer_list>
#include <type_traits>
#include <utility>

#include "engine/core/buffers.hpp"
#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/sample_cloud/sample_cloud.hpp"

namespace isaac {

// Copy tensors with compatibale memory layouts
template <typename K, size_t Channels, typename SourceContainer, typename TargetContainer>
void Copy(const SampleCloudBase<K, Channels, SourceContainer>& source,
          SampleCloudBase<K, Channels, TargetContainer>& target) {
  // Asserts that images have the same shape
  ASSERT(source.size() == target.size(), "Sample Clouds do not have the same number of points");

  const size_t rows = 1;
  const size_t row_width = Channels * source.size() * sizeof(K);
  CopyMatrixRaw(reinterpret_cast<const void*>(source.data().pointer().get()),
                source.channel_data_pitch(), BufferTraits<SourceContainer>::kStorageMode,
                reinterpret_cast<void*>(target.data().pointer().get()), target.channel_data_pitch(),
                BufferTraits<TargetContainer>::kStorageMode, rows, row_width);
}

// Copy tensors with compatibale memory layouts
template <typename K, size_t Channels, typename SourceContainer, typename TargetContainer>
void Copy(const SampleCloudPlanarBase<K, Channels, SourceContainer>& source,
          SampleCloudPlanarBase<K, Channels, TargetContainer>& target) {
  // Asserts that images have the same shape
  ASSERT(source.size() == target.size(), "Sample Clouds do not have the same number of points");

  const size_t rows = Channels;
  const size_t row_width = source.size() * sizeof(K);
  CopyMatrixRaw(reinterpret_cast<const void*>(source.data().pointer().get()),
                source.channel_data_pitch(), BufferTraits<SourceContainer>::kStorageMode,
                reinterpret_cast<void*>(target.data().pointer().get()), target.channel_data_pitch(),
                BufferTraits<TargetContainer>::kStorageMode, rows, row_width);
}

}  // namespace isaac
