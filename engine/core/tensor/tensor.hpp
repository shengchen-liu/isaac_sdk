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
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/buffers/aligned_buffer.hpp"
#include "engine/core/byte.hpp"
#include "engine/gems/math/float16.hpp"

namespace isaac {
// Free Function to convert the size values for a Tensor of a given order with a given stride index
// into the row and column counts for a buffer. Useful for computing the offsets to convert a buffer
// into a tensor without first constructing a tensor of a given size.
inline std::array<size_t, 2> ConvertTensorDimensionsToBufferDimensions(
    size_t order, size_t stride_index, const std::vector<size_t>& dimensions) {
  ASSERT(stride_index <= order && order > 0, "Invalid tensor configuration");
  ASSERT(dimensions.size() == order, "Invalid dimension size.");
  std::array<size_t, 2> out = {1, 1};
  for (size_t i = 0; i < order - stride_index; ++i) {
    out[0] *= dimensions[i];
  }
  for (size_t i = order - stride_index; i < order; ++i) {
    out[1] *= dimensions[i];
  }
  return out;
}

// Same function as the previous  but for an array list of dimensions
template <size_t SIZE>
inline std::array<size_t, 2> ConvertTensorDimensionsToBufferDimensions(
    size_t order, size_t stride_index, const std::array<size_t, SIZE>& dimensions) {
  ASSERT(stride_index <= order && order > 0, "Invalid tensor configuration");
  ASSERT(dimensions.size() == order, "Invalid dimension size.");
  std::array<size_t, 2> out = {1, 1};
  for (size_t i = 0; i < order - stride_index; ++i) {
    out[0] *= dimensions[i];
  }
  for (size_t i = order - stride_index; i < order; ++i) {
    out[1] *= dimensions[i];
  }
  return out;
}

// Tensor of type K with order Order and specified storage type
// This class is primarily intended to be used as a container
// and does not support math operations.
// Free functions to handle math or other tensor operations may exist as utilities.
//
// K is the data type of the tensor.
// Order is fixed at compile time, but dimensions are dynamic
// StrideIndex represents the index starting from the innermost(right most)
// dimension that will be packed onto scanlines.
// buffer_t represents the storage type of the data.
//
// Example: an Order 3 Tensor with StrideIndex of 2 and dimensions 480 x 640 x 3
// In this case there would be 480 scanlines of lengnth 3*640 elements.
// Example: an Order 3 Tensor with StrideIndex of 1 and dimensions 480 x 640 x 3
// In this case there would be 480*640 scanlines of length 3 elements.
//
template <typename K, size_t Order, size_t StrideIndex, typename BufferType>
class TensorBase {
  static_assert(StrideIndex <= Order, "StrideIndex must be less than or equal to the Order");
  static_assert(Order > 0, "Isaac does not support order 0 tensors");

 public:
  using buffer_t = BufferType;

  static constexpr bool kIsMutable = BufferTraits<buffer_t>::kIsMutable;
  static constexpr bool kIsOwning = BufferTraits<buffer_t>::kIsOwning;

  using element_t = std::remove_cv_t<K>;
  using element_const_ptr_t = std::add_const_t<element_t>*;
  using element_ptr_t = std::conditional_t<kIsMutable, element_t*, element_const_ptr_t>;

  using element_const_ref_t = std::add_const_t<element_t>&;
  using element_ref_t = std::conditional_t<kIsMutable, element_t&, element_const_ref_t>;

  using raw_const_ptr_t = std::add_const_t<byte>*;
  using raw_ptr_t = std::conditional_t<kIsMutable, byte*, raw_const_ptr_t>;

  using buffer_view_t = typename BufferTraits<buffer_t>::buffer_view_t;
  using buffer_const_view_t = typename BufferTraits<buffer_t>::buffer_const_view_t;

  using tensor_view_t = TensorBase<K, Order, StrideIndex, buffer_view_t>;
  using tensor_const_view_t = TensorBase<K, Order, StrideIndex, buffer_const_view_t>;

  using dimension_t = size_t;
  using dimension_array_t = std::array<dimension_t, Order>;

  using index_t = size_t;
  using index_array_t = std::array<index_t, Order>;

  using scanline_array_t = std::array<size_t, 2>;

  TensorBase() {
    // If no dimensions are provided set to zero.
    std::fill(dimensions_.begin(), dimensions_.end(), 0);
    calculate_offsets();
  }

  // Constructs a tensor with the given dimensions. Allocates storage.
  TensorBase(const dimension_array_t& dimensions) { resize(dimensions); }
  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 1, bool> = true>
  TensorBase(dimension_t dim1) : TensorBase(dimension_array_t{dim1}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 2, bool> = true>
  TensorBase(dimension_t dim1, dimension_t dim2) : TensorBase(dimension_array_t{dim1, dim2}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 3, bool> = true>
  TensorBase(dimension_t dim1, dimension_t dim2, dimension_t dim3)
      : TensorBase(dimension_array_t{dim1, dim2, dim3}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 4, bool> = true>
  TensorBase(dimension_t dim1, dimension_t dim2, dimension_t dim3, dimension_t dim4)
      : TensorBase(dimension_array_t{dim1, dim2, dim3, dim4}) {}

  // Constructor of tensor around existing data. Does not allocate storage.
  TensorBase(buffer_t data, const dimension_array_t& dimensions)
      : data_(std::move(data)), dimensions_(dimensions) {
    ASSERT(this->validate_dimensions(), "Trying to create tensor with invalid dimensions");
    ASSERT(data.rows() >= this->scanline_count() &&
               data.stride() >= this->scanline_length() * sizeof(element_t),
           "Provided data and dimensions do not match");
    calculate_offsets();
  }

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 1, bool> = true>
  TensorBase(buffer_t data, dimension_t dim1)
      : TensorBase(std::move(data), dimension_array_t{dim1}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 2, bool> = true>
  TensorBase(buffer_t data, dimension_t dim1, dimension_t dim2)
      : TensorBase(std::move(data), dimension_array_t{dim1, dim2}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 3, bool> = true>
  TensorBase(buffer_t data, dimension_t dim1, dimension_t dim2, dimension_t dim3)
      : TensorBase(std::move(data), dimension_array_t{dim1, dim2, dim3}) {}

  // Helper constructor of the appropriate order.
  template <size_t Dummy = Order, typename std::enable_if_t<Dummy == 4, bool> = true>
  TensorBase(buffer_t data, dimension_t dim1, dimension_t dim2, dimension_t dim3, dimension_t dim4)
      : TensorBase(std::move(data), dimension_array_t{dim1, dim2, dim3, dim4}) {}

  // Copy construction uses the default behavior
  TensorBase(const TensorBase& other) = default;
  // Copy assignment uses the default behavior1
  TensorBase& operator=(const TensorBase& other) = default;
  // Move construction uses the default behavior
  TensorBase(TensorBase&& other) = default;
  // Move assignment uses the default behavior
  TensorBase& operator=(TensorBase&& other) = default;

  // Create a view if the data is mutable
  template <bool X = kIsMutable>
  std::enable_if_t<X, tensor_view_t> view() {
    return tensor_view_t(
        {this->data().pointer().get(), this->scanline_count(), this->scanline_stride()},
        this->dimensions());
  }

  // create a const view
  tensor_const_view_t const_view() const {
    return tensor_const_view_t(
        {this->data().pointer().get(), this->scanline_count(), this->scanline_stride()},
        this->dimensions());
  }

  // Allow conversion from owning to mutable view
  template <bool X = kIsMutable>
  operator std::enable_if_t<X, tensor_view_t>() {
    return view();
  }
  // Allow conversion to const view
  operator tensor_const_view_t() const { return const_view(); }

  // Returns a pointer to the specified scanline
  template <bool X = kIsMutable>
  std::enable_if_t<X, element_ptr_t> scanline_pointer(scanline_array_t indicies) {
    ASSERT(indicies[0] < this->scanline_count(), "Invalid scanline index");
    return reinterpret_cast<element_ptr_t>(reinterpret_cast<raw_ptr_t>(this->element_wise_begin()) +
                                           indicies[0] * this->scanline_stride() +
                                           indicies[1] * sizeof(element_t));
  }
  // Returns a const pointer to the specified scanline
  element_const_ptr_t scanline_pointer(scanline_array_t indicies) const {
    ASSERT(indicies[0] < this->scanline_count(), "Invalid scanline index");
    return reinterpret_cast<element_const_ptr_t>(
        reinterpret_cast<raw_const_ptr_t>(this->element_wise_begin()) +
        indicies[0] * this->scanline_stride() + indicies[1] * sizeof(element_t));
  }

  // Returns a pointer to the specified scanline and offset
  template <bool X = kIsMutable>
  std::enable_if_t<X, element_ptr_t> scanline_pointer(size_t scanline_index,
                                                      size_t scanline_offset = 0) {
    return scanline_pointer(scanline_array_t{scanline_index, scanline_offset});
  }
  // Returns a const pointer to the specified scanline and offset
  element_const_ptr_t scanline_pointer(size_t scanline_index, size_t scanline_offset = 0) const {
    return scanline_pointer(scanline_array_t{scanline_index, scanline_offset});
  }

  // Convert tensor indicies into corresponding scanline indicies.
  // The first element of the return value is the scanline and the second
  // is the scanline offset.
  scanline_array_t tensor_to_scanline_index(index_array_t indicies) const {
    scanline_array_t out = {0, 0};
    for (size_t i = 0; i < Order - StrideIndex; ++i) {
      ASSERT(indicies[i] < dimensions_[i], "Invalid index %d for dimension %d ", indicies[i], i);
      out[0] += indicies[i] * offsets_[i];
    }
    for (size_t i = Order - StrideIndex; i < Order; ++i) {
      ASSERT(indicies[i] < dimensions_[i], "Invalid index %d for dimension %d ", indicies[i], i);
      out[1] += indicies[i] * offsets_[i];
    }
    return out;
  }

  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 1)>
  std::enable_if_t<X, scanline_array_t> tensor_to_scanline_index(index_t index1) {
    return tensor_to_scanline_index(index_array_t{index1});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 2)>
  std::enable_if_t<X, scanline_array_t> tensor_to_scanline_index(index_t index1, index_t index2) {
    return tensor_to_scanline_index(index_array_t{index1, index2});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 3)>
  std::enable_if_t<X, scanline_array_t> tensor_to_scanline_index(index_t index1, index_t index2,
                                                                 index_t index3) {
    return tensor_to_scanline_index(index_array_t{index1, index2, index3});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 4)>
  std::enable_if_t<X, scanline_array_t> tensor_to_scanline_index(index_t index1, index_t index2,
                                                                 index_t index3, size_t index4) {
    return tensor_to_scanline_index(index_array_t{index1, index2, index3, index4});
  }

  // Scanline stride in bytes
  size_t scanline_stride() const { return data_.stride(); }
  // Returns the number of scanlines backing the tensor
  size_t scanline_count() const {
    size_t length = 1;
    for (size_t i = 0; i < Order - StrideIndex; ++i) {
      length *= dimensions_[i];
    }
    return length;
  }
  // Returns the number of elements per scanline
  size_t scanline_length() const {
    size_t length = 1;
    for (size_t i = Order - StrideIndex; i < Order; ++i) {
      length *= dimensions_[i];
    }
    return length;
  }

  // Const Pointer to the beginning of the data block
  element_const_ptr_t element_wise_begin() const {
    return reinterpret_cast<element_const_ptr_t>(data_.pointer().get());
  }
  // Pointer to the beginning of the data block
  element_ptr_t element_wise_begin() {
    return reinterpret_cast<element_ptr_t>(data_.pointer().get());
  }

  // const access to the underlying buffer object
  const buffer_t& data() const { return data_; }
  // access to the underlying buffer object
  template <bool X = kIsMutable>
  std::enable_if_t<X, buffer_t&> data() {
    return data_;
  }

  // The total number of elements in the tensor
  size_t element_count() const {
    size_t total = 1;
    for (size_t i = 0; i < Order; ++i) {
      total *= dimensions_[i];
    }
    return total;
  }
  // Returns the order of the tensor
  constexpr size_t order() const { return Order; }
  // Returns the dimensions of the tensor
  dimension_array_t dimensions() const { return dimensions_; }
  // Returns the index order of the tensor
  constexpr size_t stride_index() const { return StrideIndex; }

  // Resizes the tensor memory, This operation is destructive.
  template <bool X = kIsOwning>
  std::enable_if_t<X, void> resize(const dimension_array_t& dimensions) {
    dimensions_ = dimensions;
    ASSERT(this->validate_dimensions(), "Invalid dimensions");
    offsets_.fill(1);
    data_ = buffer_t(this->scanline_count(), this->scanline_length() * sizeof(element_t));
    calculate_offsets();
  }

  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 1)>
  std::enable_if_t<X, void> resize(dimension_t dimension1) {
    return resize(dimension_array_t{dimension1});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 2)>
  std::enable_if_t<X, void> resize(dimension_t dimension1, dimension_t dimension2) {
    return resize(dimension_array_t{dimension1, dimension2});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 3)>
  std::enable_if_t<X, void> resize(dimension_t dimension1, dimension_t dimension2,
                                   dimension_t dimension3) {
    return resize(dimension_array_t{dimension1, dimension2, dimension3});
  }
  // Helper function of the approrpriate order for syntatic nicety.
  template <bool X = (Order == 4)>
  std::enable_if_t<X, void> resize(dimension_t dimension1, dimension_t dimension2,
                                   dimension_t dimension3, size_t dimension4) {
    return resize(dimension_array_t{dimension1, dimension2, dimension3, dimension4});
  }

  // Accesses the given index in the tensor with respect to memory ordering
  element_t operator()(index_array_t indicies) const {
    ASSERT(data_.size() > 0, "Attempting to access empty data");
    auto index = tensor_to_scanline_index(indicies);
    return *(scanline_pointer(index));
  }

  // Accesses the given index in the tensor with respect to memory ordering
  template <bool X = kIsMutable>
  std::enable_if_t<X, element_ref_t> operator()(index_array_t indicies) {
    ASSERT(data_.size() > 0, "Attempting to access empty data");
    return *(scanline_pointer(tensor_to_scanline_index(indicies)));
  }

  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 1)>
  std::enable_if_t<X, element_t> operator()(index_t index1) const {
    return operator()(index_array_t{index1});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 1), bool Y = kIsMutable>
  std::enable_if_t<X && Y, element_ref_t> operator()(index_t index1) {
    return operator()(index_array_t{index1});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 2)>
  std::enable_if_t<X, element_t> operator()(index_t index1, index_t index2) const {
    return operator()(index_array_t{index1, index2});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 2), bool Y = kIsMutable>
  std::enable_if_t<X && Y, element_ref_t> operator()(index_t index1, index_t index2) {
    return operator()(index_array_t{index1, index2});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 3)>
  std::enable_if_t<X, element_t> operator()(index_t index1, index_t index2, index_t index3) const {
    return operator()(index_array_t{index1, index2, index3});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 3), bool Y = kIsMutable>
  std::enable_if_t<X && Y, element_ref_t> operator()(index_t index1, index_t index2,
                                                     index_t index3) {
    return operator()(index_array_t{index1, index2, index3});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 4)>
  std::enable_if_t<X, element_t> operator()(index_t index1, index_t index2, index_t index3,
                                            index_t index4) const {
    return operator()(index_array_t{index1, index2, index3, index4});
  }
  // Helper wrappers for operator access for the most common orders of tensors
  template <bool X = (Order == 4), bool Y = kIsMutable>
  std::enable_if_t<X && Y, element_ref_t> operator()(index_t index1, index_t index2, index_t index3,
                                                     index_t index4) {
    return operator()(index_array_t{index1, index2, index3, index4});
  }

 private:
  // Compute memory offsets based upon index order and dimensions of the tensor
  void calculate_offsets() {
    offsets_.fill(1);
    for (size_t i = 0; i < Order - StrideIndex; ++i) {
      for (size_t j = i + 1; j < Order - StrideIndex; ++j) {
        offsets_[i] *= dimensions_[j];
      }
    }
    for (size_t i = Order - StrideIndex; i < Order; ++i) {
      for (size_t j = i + 1; j < Order; ++j) {
        offsets_[i] *= dimensions_[j];
      }
    }
  }
  // Make sure the dimensions are greater than 0
  bool validate_dimensions() const {
    bool valid = true;
    for (size_t i = 0; i < Order; ++i) {
      valid = valid && dimensions_[i] > 0;
    }
    return valid;
  }

  // storage for the tensor
  buffer_t data_;
  // The dimensions of the tensor.
  dimension_array_t dimensions_;
  // Dimension offsets for indexing rows of storage
  dimension_array_t offsets_;
};  // namespace isaac

template <typename K, size_t Order, size_t StrideIndex = 1>
using Tensor = TensorBase<K, Order, StrideIndex, CpuAlignedBuffer>;

template <typename K, size_t Order, size_t StrideIndex = 1>
using TensorView = TensorBase<K, Order, StrideIndex, CpuAlignedBufferView>;

template <typename K, size_t Order, size_t StrideIndex = 1>
using TensorConstView = TensorBase<K, Order, StrideIndex, CpuAlignedBufferConstView>;

#define ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, T, S) \
  using Tensor##N##S = Tensor<T, N>;             \
  using TensorView##N##S = TensorView<T, N>;     \
  using TensorConstView##N##S = TensorConstView<T, N>;

#define ISAAC_DECLARE_TENSOR_TYPES(N)                \
  template <class K>                                 \
  using Tensor##N = Tensor<K, N>;                    \
  template <class K>                                 \
  using TensorView##N = TensorView<K, N>;            \
  template <class K>                                 \
  using TensorConstView##N = TensorConstView<K, N>;  \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, uint8_t, ub)    \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, uint16_t, ui16) \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, int, i)         \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, double, d)      \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, float, f)       \
  ISAAC_DECLARE_TENSOR_TYPES_IMPL(N, float16, f16)

ISAAC_DECLARE_TENSOR_TYPES(1)
ISAAC_DECLARE_TENSOR_TYPES(2)
ISAAC_DECLARE_TENSOR_TYPES(3)
ISAAC_DECLARE_TENSOR_TYPES(4)

// -------------------------------------------------------------------------------------------------

// An Tensor stored in device memory which owns it's memory
template <class K, size_t Order, size_t StrideIndex = 1>
using CudaTensor = TensorBase<K, Order, StrideIndex, CudaAlignedBuffer>;

// A mutable view on an Tensor which is stored on GPU device memory, does not own memory, but can
// be used to read and write the data of the underlying Tensor.
template <class K, size_t Order, size_t StrideIndex = 1>
using CudaTensorView = TensorBase<K, Order, StrideIndex, CudaAlignedBufferView>;

// A non-mutable view on an Tensor which is stored on GPU device memory, does not own its memory,
// and can only be used to read the data of the underlying Tensor.
template <class K, size_t Order, size_t StrideIndex = 1>
using CudaTensorConstView = TensorBase<K, Order, StrideIndex, CudaAlignedBufferConstView>;

// Helper macro for ISAAC_DECLARE_CUDA_TENSOR_TYPES
#define ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, K, S) \
  using CudaTensor##N##S = CudaTensor<K, N>;          \
  using CudaTensorView##N##S = CudaTensorView<K, N>;  \
  using CudaTensorConstView##N##S = CudaTensorConstView<K, N>;

// Helper macro to define various CudaTensor types
#define ISAAC_DECLARE_CUDA_TENSOR_TYPES(N)                  \
  template <class K>                                        \
  using CudaTensor##N = CudaTensor<K, N>;                   \
  template <class K>                                        \
  using CudaTensorView##N = CudaTensorView<K, N>;           \
  template <class K>                                        \
  using CudaTensorConstView##N = CudaTensorConstView<K, N>; \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, uint8_t, ub)      \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, uint16_t, ui16)   \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, int, i)           \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, double, d)        \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, float, f)         \
  ISAAC_DECLARE_CUDA_TENSOR_TYPES_IMPL(N, float16, f16)

ISAAC_DECLARE_CUDA_TENSOR_TYPES(1)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(2)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(3)
ISAAC_DECLARE_CUDA_TENSOR_TYPES(4)

}  // namespace isaac
