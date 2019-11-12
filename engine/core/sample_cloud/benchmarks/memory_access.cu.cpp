/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "memory_access.cu.hpp"

#include <cuda_runtime.h>

namespace isaac {

namespace {

// Dummy variable to prevent compiler code stripping.
__device__ float dummy;
// This performs a trivial 7 tap memory access across interleaved data for each channel
__global__ void WindowSumInterleavedImpl(const float* data, int channels, size_t num_points) {
  int point = (3 + blockIdx.x * blockDim.x + threadIdx.x) * channels;
  float out = 0.0;
  while (point < (num_points - 3) * channels) {
    for (int i = 0; i < channels; ++i) {
      const float* val = data + point;
      out += val[-3 * channels + i] + val[-2 * channels + i] + val[-1 * channels + i] + val[i] +
             val[1 * channels + i] + val[2 * channels + i] + val[3 * channels + i];
    }
    if (point > 1000000000) {
      dummy = out;
    }
    point += (blockDim.x * gridDim.x) * channels;
  }
}

// This performs a trivial 7 tap memory access across planar data for each channel
__global__ void WindowedSumPlanarImpl(const float* data, size_t num_points, int channels, size_t pitch) {
  int point_x = 3 + blockIdx.x * blockDim.x + threadIdx.x;
  volatile float out = 0.0;
  while (point_x < (num_points - 3)) {
    for (int i = 0; i < channels; ++i) {
      const float* val = data + (i * pitch + point_x);
      out += val[-3] + val[-2] + val[-1] + val[0] + val[1] + val[2] + val[3];
    }
    point_x += blockDim.x * gridDim.x;
    if (point_x > 1000000000) {
      dummy = out;
    }
  }
}

}  // namespace

float WindowedSumInterleaved(const float* data, size_t channels, size_t num_points) {
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start);
  WindowSumInterleavedImpl<<<256, 256>>>(data, channels, num_points);
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);
  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  return milliseconds;
}

float WindowedSumPlanar(const float* data, size_t channels, size_t num_points, size_t pitch) {
  cudaEvent_t start, stop;
  cudaEventCreate(&start);
  cudaEventCreate(&stop);
  cudaEventRecord(start);
  WindowedSumPlanarImpl<<<256, 256>>>(data, num_points, channels, pitch);
  cudaEventRecord(stop);
  cudaEventSynchronize(stop);
  float milliseconds = 0;
  cudaEventElapsedTime(&milliseconds, start, stop);
  return milliseconds;
}
}  // namespace isaac
