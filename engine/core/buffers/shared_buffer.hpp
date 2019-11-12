/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <mutex>
#include <utility>

#include "engine/core/buffers/algorithm.hpp"
#include "engine/core/buffers/aligned_buffer.hpp"

namespace isaac {

// A wrapper class that supports dual access to cpu and gpu memory buffers.
// Memory copies will happen on demand.
// The primary use case for this class is for use with the message system
// to elide the need to constantly copy from gpu to cpu before sending a message.
class SharedAlignedBuffer {
 public:
  SharedAlignedBuffer() = delete;

  template <typename BufferType>
  SharedAlignedBuffer(BufferType&& buffer) {
    MoveImpl(buffer, typename BufferTraits<BufferType>::memory_t());
  }
  // Need a move constructor for use with vectors and other containers.
  SharedAlignedBuffer(SharedAlignedBuffer&& rhs);
  // Access the data from the cpu.
  const CpuAlignedBuffer& host_buffer() const;
  // Access the data from the gpu.
  const CudaAlignedBuffer& cuda_buffer() const;

 private:
  template <typename BufferType>
  void MoveImpl(BufferType&& buffer, const MemoryType<BufferStorageMode::Host>::memory_t&) {
    cpu_buffer_ = std::move(buffer);
    has_cpu_data_ = true;
  }

  template <typename BufferType>
  void MoveImpl(BufferType&& buffer, const MemoryType<BufferStorageMode::Cuda>::memory_t&) {
    cuda_buffer_ = std::move(buffer);
    has_cuda_data_ = true;
  }
  // Storage for gpu memory.
  mutable CudaAlignedBuffer cuda_buffer_;
  // Storage for cpu memory.
  mutable CpuAlignedBuffer cpu_buffer_;
  // Lock to protect against concurent memory creation.
  mutable std::mutex data_access_lock_;

  mutable bool has_cpu_data_ = false;
  mutable bool has_cuda_data_ = false;
};

}  // namespace isaac
