/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

#include "shared_buffer.hpp"

#include <mutex>
#include <utility>

namespace isaac {

SharedAlignedBuffer::SharedAlignedBuffer(SharedAlignedBuffer&& buffer) {
  std::lock_guard<std::mutex> lock(buffer.data_access_lock_);
  has_cpu_data_ = buffer.has_cpu_data_;
  has_cuda_data_ = buffer.has_cuda_data_;
  cpu_buffer_ = std::move(buffer.cpu_buffer_);
  cuda_buffer_ = std::move(buffer.cuda_buffer_);
}

const CpuAlignedBuffer& SharedAlignedBuffer::host_buffer() const {
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (!has_cpu_data_) {
    ASSERT(has_cuda_data_, "Buffer has no data available");
    cpu_buffer_ = CpuAlignedBuffer(cuda_buffer_.rows(), cuda_buffer_.stride());
    CopyMatrixRaw(reinterpret_cast<const byte*>(cuda_buffer_.begin()), cuda_buffer_.stride(),
                  BufferStorageMode::Cuda, reinterpret_cast<byte*>(cpu_buffer_.begin()),
                  cpu_buffer_.stride(), BufferStorageMode::Host, cuda_buffer_.rows(),
                  cuda_buffer_.stride());
    has_cpu_data_ = true;
  }
  return cpu_buffer_;
}

const CudaAlignedBuffer& SharedAlignedBuffer::cuda_buffer() const {
  std::lock_guard<std::mutex> lock(data_access_lock_);
  if (!has_cuda_data_) {
    ASSERT(has_cpu_data_, "Buffer has no data available");
    cuda_buffer_ = CudaAlignedBuffer(cpu_buffer_.rows(), cpu_buffer_.stride());
    CopyMatrixRaw(reinterpret_cast<const byte*>(cpu_buffer_.begin()), cpu_buffer_.stride(),
                  BufferStorageMode::Host, reinterpret_cast<byte*>(cuda_buffer_.begin()),
                  cuda_buffer_.stride(), BufferStorageMode::Cuda, cpu_buffer_.rows(),
                  cpu_buffer_.stride());
    has_cuda_data_ = true;
  }
  return cuda_buffer_;
}
}  // namespace isaac
