/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <type_traits>
#include <utility>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/traits.hpp"
#include "engine/core/byte.hpp"

namespace isaac {

// -------------------------------------------------------------------------------------------------

namespace detail {

// A simple pointer which can be tagged. This is used for example to differentiate between a pointer
// to host memory and a pointer to device memory.
template <typename K, typename Tag = byte>
class TaggedPointer {
 public:
  using value_t = std::remove_cv_t<K>;
  using const_pointer_t = TaggedPointer<const value_t, Tag>;
  using pointer_t = TaggedPointer<value_t, Tag>;
  using tag_t = Tag;
  // Standard constructor
  TaggedPointer(K* data = nullptr) : data_(data) {}

  // Default copy
  TaggedPointer(const TaggedPointer& other) = default;
  TaggedPointer& operator=(const TaggedPointer& other) = default;

  // Move will set source to nullptr
  TaggedPointer(TaggedPointer&& other) { *this = std::move(other); }
  TaggedPointer& operator=(TaggedPointer&& other) {
    data_ = other.data_;
    other.data_ = nullptr;
    return *this;
  }

  // Sets the actual pointer
  TaggedPointer& operator=(K* other) {
    data_ = other;
    return *this;
  }
  // Gets the actual pointer
  K* get() const { return data_; }
  operator K*() const { return data_; }

 private:
  K* data_;
};

}  // namespace detail

// -------------------------------------------------------------------------------------------------

namespace detail {

// Base class for AlignedBuffer and AlignedBufferView which provides storage for the memory pointer
// and corresponding dimensions.
template <typename Pointer>
class AlignedBufferBase {
 public:
  AlignedBufferBase() : pointer_(nullptr), rows_(0), stride_(0) {}

  AlignedBufferBase(Pointer pointer, size_t rows, size_t stride)
      : pointer_(std::move(pointer)), rows_(rows), stride_(stride) {}

  // pointer to the first row
  const Pointer& pointer() const { return pointer_; }
  // number of rows
  size_t rows() const { return rows_; }
  // number of bytes allocated per row
  size_t stride() const { return stride_; }
  // The total size of the buffer in bytes
  size_t size() const { return rows() * stride(); }
  // A pointer to the first byte of the buffer.
  auto begin() const { return pointer_.get(); }
  // A pointer behind the last byte of the buffer.
  auto end() const { return begin() + size(); }

 protected:
  Pointer pointer_;
  size_t rows_;
  size_t stride_;
};

}  // namespace detail

// -------------------------------------------------------------------------------------------------

// A buffer with stride which owns its memory
template <typename Pointer, typename StrideAllocator>
class AlignedBuffer : public detail::AlignedBufferBase<Pointer> {
 public:
  using mutable_view_t = detail::AlignedBufferBase<typename Pointer::pointer_t>;
  using const_view_t = detail::AlignedBufferBase<typename Pointer::const_pointer_t>;

  AlignedBuffer() : handle_(nullptr, StrideAllocator::Deallocate) {}

  // Allocates memory for `rows` with at least `min_stride` of bytes per row. The chosen stride
  // might be larger than the requested one depending on the allocator.
  AlignedBuffer(size_t rows, size_t min_stride)
      : detail::AlignedBufferBase<Pointer>(nullptr, rows, min_stride),
        handle_(StrideAllocator::Allocate(rows, this->stride_), StrideAllocator::Deallocate) {
    // 1) Initialize the base class with a nullptr, the number of rows, and the desired stride.
    // 2) Allocate memory and store it in the unique pointer used as handle. This will also change
    //    the stride stored in the base class to the actual stride chosen by the allocator.
    // 3) Get a pointer to the allocated memory and store it in the base class so that calls to
    //    pointer() actually work.
    this->pointer_ = handle_.get();
  }

  AlignedBuffer(AlignedBuffer&& buffer)
      : detail::AlignedBufferBase<Pointer>(nullptr, buffer.rows_, buffer.stride_),
        handle_(std::move(buffer.handle_)) {
    this->pointer_ = handle_.get();
    buffer.pointer_ = nullptr;
  }

  AlignedBuffer& operator=(AlignedBuffer&& buffer) {
    this->handle_ = std::move(buffer.handle_);
    buffer.pointer_ = nullptr;
    this->pointer_ = this->handle_.get();
    this->rows_ = buffer.rows_;
    this->stride_ = buffer.stride_;
    return *this;
  }

  // Disowns the pointer from the buffer. The user is now responsible for deallocation.
  // WARNING: This is dangerous as the wrong allocator might be called.
  byte* release() {
    byte* pointer = handle_.release();
    this->pointer_ = nullptr;
    return pointer;
  }

  // Creates a view which provides read and write access from this buffer object.
  mutable_view_t view() { return mutable_view_t(this->pointer_, this->rows_, this->stride_); }
  const_view_t view() const {
    return const_view_t(typename Pointer::const_pointer_t(this->pointer_.get()), this->rows_,
                        this->stride_);
  }
  // Creates a view which only provides read access from this buffer object.
  const_view_t const_view() const {
    return const_view_t(typename Pointer::const_pointer_t(this->pointer_.get()), this->rows_,
                        this->stride_);
  }

 private:
  // A unique pointer is used to handle the allocator and to make this object non-copyable.
  using handle_t = std::unique_ptr<byte, void (*)(byte*)>;

  // Memory handle used to automatically deallocate memory and to disallow copy semantics.
  handle_t handle_;
};

// -------------------------------------------------------------------------------------------------

// Allocates host memory for a table with std::malloc
struct MallocAlignedAllocator {
  static constexpr BufferStorageMode Storage = BufferStorageMode::Host;
  static byte* Allocate(size_t rows, size_t& stride);
  static void Deallocate(byte* pointer);
};

// A pointer to host memory
template <typename K>
using HostPointer =
    detail::TaggedPointer<K, std::integral_constant<BufferStorageMode, BufferStorageMode::Host>>;

// A host buffer with stride which owns its memory
using CpuAlignedBuffer = AlignedBuffer<HostPointer<byte>, MallocAlignedAllocator>;

// A host buffer with stride which does not own its memory
using CpuAlignedBufferView = detail::AlignedBufferBase<HostPointer<byte>>;

// A host buffer with stride which does not own its memory and provides only read access
using CpuAlignedBufferConstView = detail::AlignedBufferBase<HostPointer<const byte>>;

// Empty types to help differentiate buffer traits
namespace detail {

struct Host {};
struct Cuda {};

}  // namespace detail

template <BufferStorageMode>
struct MemoryType;

template <>
struct MemoryType<BufferStorageMode::Host> {
  using memory_t = detail::Host;
};

template <>
struct MemoryType<BufferStorageMode::Cuda> {
  using memory_t = detail::Cuda;
};

template <>
struct BufferTraits<AlignedBuffer<HostPointer<byte>, MallocAlignedAllocator>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = CpuAlignedBufferView;
  using buffer_const_view_t = CpuAlignedBufferConstView;
  using memory_t = MemoryType<BufferStorageMode::Host>::memory_t;
};

template <typename K>
struct BufferTraits<detail::AlignedBufferBase<HostPointer<K>>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Host;
  static constexpr bool kIsMutable = !std::is_const<K>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CpuAlignedBufferView;
  using buffer_const_view_t = CpuAlignedBufferConstView;
  using memory_t = MemoryType<BufferStorageMode::Host>::memory_t;
};

// -------------------------------------------------------------------------------------------------

// Allocates CUDA device memory for a table with cudaMallocPitch
struct CudaMallocAlignedAllocator {
  static constexpr BufferStorageMode Storage = BufferStorageMode::Cuda;
  static byte* Allocate(size_t rows, size_t& stride);
  static void Deallocate(byte* pointer);
};

// Allocates CUDA device memory for a table with cudaMalloc
struct CudaMallocAllocator {
  static constexpr BufferStorageMode Storage = BufferStorageMode::Cuda;
  static byte* Allocate(size_t rows, size_t& stride);
  static void Deallocate(byte* pointer);
};

// A pointer to CUDA memory
template <typename K>
using CudaPointer =
    detail::TaggedPointer<K, std::integral_constant<BufferStorageMode, BufferStorageMode::Cuda>>;

// A CUDA buffer with stride which owns its memory
using CudaAlignedBuffer = AlignedBuffer<CudaPointer<byte>, CudaMallocAlignedAllocator>;

// A CUDA buffer with stride which does not own its memory
using CudaAlignedBufferView = detail::AlignedBufferBase<CudaPointer<byte>>;

// A CUDA buffer with stride which does not own its memory and provides only read access
using CudaAlignedBufferConstView = detail::AlignedBufferBase<CudaPointer<const byte>>;

// A CUDA buffer with trivial stride which owns its memory
using CudaBuffer = AlignedBuffer<CudaPointer<byte>, CudaMallocAllocator>;

// A CUDA buffer with trivial stride which does not own its memory
using CudaBufferView = CudaAlignedBufferView;

// A CUDA buffer with trivial stride which does not own its memory and provides only read access
using CudaBufferConstView = CudaAlignedBufferConstView;

template <>
struct BufferTraits<AlignedBuffer<CudaPointer<byte>, CudaMallocAlignedAllocator>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = CudaAlignedBufferView;
  using buffer_const_view_t = CudaAlignedBufferConstView;
  using memory_t = MemoryType<BufferStorageMode::Cuda>::memory_t;
};

template <>
struct BufferTraits<AlignedBuffer<CudaPointer<byte>, CudaMallocAllocator>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = true;
  static constexpr bool kIsOwning = true;

  using buffer_view_t = CudaBufferView;
  using buffer_const_view_t = CudaBufferConstView;
  using memory_t = MemoryType<BufferStorageMode::Cuda>::memory_t;
};

template <typename K>
struct BufferTraits<detail::AlignedBufferBase<CudaPointer<K>>> {
  static constexpr BufferStorageMode kStorageMode = BufferStorageMode::Cuda;
  static constexpr bool kIsMutable = !std::is_const<K>::value;
  static constexpr bool kIsOwning = false;

  using buffer_view_t = CudaAlignedBufferView;
  using buffer_const_view_t = CudaAlignedBufferConstView;
  using memory_t = MemoryType<BufferStorageMode::Cuda>::memory_t;
};

// -------------------------------------------------------------------------------------------------

}  // namespace isaac
