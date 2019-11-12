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
#include "engine/core/tensor/tensor.hpp"

namespace isaac {

// Copy tensors with compatibale memory layouts
template <typename K, size_t Order, size_t StrideIndex, typename SourceContainer,
          typename TargetContainer>
void Copy(const TensorBase<K, Order, StrideIndex, SourceContainer>& source,
          TensorBase<K, Order, StrideIndex, TargetContainer>& target) {
  // Asserts that images have the same shape
  ASSERT(source.dimensions() == target.dimensions(), "Tensor Dimension Mismatch");
  // Copy the bytes
  CopyMatrixRaw(reinterpret_cast<const void*>(source.element_wise_begin()),
                source.scanline_stride(), BufferTraits<SourceContainer>::kStorageMode,
                reinterpret_cast<void*>(target.element_wise_begin()), target.scanline_stride(),
                BufferTraits<TargetContainer>::kStorageMode, source.scanline_count(),
                source.scanline_length() * sizeof(K));
}

// Fills a tensor with the given value.
template <typename K, size_t Order, size_t StrideIndex, typename Container>
void Fill(TensorBase<K, Order, StrideIndex, Container>& tensor, K value) {
  static_assert(TensorBase<K, Order, StrideIndex, Container>::kIsMutable,
                "Cannot Fill const buffer");
  size_t length = tensor.scanline_length();
  size_t count = tensor.scanline_count();

  for (size_t i = 0; i < count; i++) {
    auto scanline = tensor.scanline_pointer(i);
    std::fill(scanline, scanline + length, value);
  }
}

// flattens the tensor into a buffer with all stride pitch removed.
template <typename K, size_t Order, size_t StrideIndex, typename Container>
void FlattenData(const TensorBase<K, Order, StrideIndex, Container>& tensor, ByteBuffer* out) {
  ASSERT(out->size() == tensor.element_count() * sizeof(K),
         "Buffer must be large enough to hold results");
  auto* data = out->begin();
  size_t length = tensor.scanline_length() * sizeof(K);
  size_t count = tensor.scanline_count();
  for (size_t i = 0; i < count; i++) {
    auto scanline = tensor.scanline_pointer(i);
    std::memcpy(data, scanline, length);
    data += length;
  }
}
}  // namespace isaac
