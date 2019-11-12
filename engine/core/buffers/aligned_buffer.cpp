/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/buffers/aligned_buffer.hpp"

#include <stdlib.h>

#include <cstdlib>

#include "cuda_runtime.h"  // NOLINT
#include "engine/core/assert.hpp"

namespace isaac {

// TODO Enable the following code once CpuImage fully supports stride
// namespace {
// // We choose a default alignment which should work well for up to Image4f.
// constexpr size_t kDefaultHostAlignment = 4 * sizeof(float);
// }  // namespace

byte* MallocAlignedAllocator::Allocate(size_t rows, size_t& stride) {
  // TODO Enable the following code once CpuImage fully supports stride
  // stride = std::max(kDefaultHostAlignment, stride);
  // const size_t num_bytes = rows * stride;
  // ASSERT(num_bytes >= kDefaultHostAlignment, "Not yet supported: %zd / %zd",
  //        num_bytes, kDefaultHostAlignment);
  // // TODO Switch to std::aligned_alloc once C++17 is supported
  // byte* pointer = aligned_alloc(kDefaultHostAlignment, num_bytes);
  byte* pointer = static_cast<byte*>(std::malloc(rows * stride));
  ASSERT(pointer != nullptr, "Could not allocate memory");
  return pointer;
}

void MallocAlignedAllocator::Deallocate(byte* pointer) {
  if (pointer != nullptr) {
    std::free(pointer);
  }
}

byte* CudaMallocAlignedAllocator::Allocate(size_t rows, size_t& stride) {
  byte* raw_pointer;
  // const cudaError_t error = cudaMallocPitch(&raw_pointer, &stride, stride, rows);
  const cudaError_t error = cudaMalloc(&raw_pointer, stride * rows);
  ASSERT(error == cudaSuccess, "Could not allocate memory. Error: %d", error);
  return raw_pointer;
}

void CudaMallocAlignedAllocator::Deallocate(byte* pointer) {
  if (pointer != nullptr) {
    const cudaError_t error = cudaFree(pointer);
    ASSERT(error == cudaSuccess, "Could not free memory. Error: %d", error);
  }
}

byte* CudaMallocAllocator::Allocate(size_t rows, size_t& stride) {
  byte* raw_pointer;
  const cudaError_t error = cudaMalloc(&raw_pointer, stride * rows);
  ASSERT(error == cudaSuccess, "Could not allocate memory. Error: %d", error);
  return raw_pointer;
}

void CudaMallocAllocator::Deallocate(byte* pointer) {
  if (pointer != nullptr) {
    const cudaError_t error = cudaFree(pointer);
    ASSERT(error == cudaSuccess, "Could not free memory. Error: %d", error);
  }
}

}  // namespace isaac
