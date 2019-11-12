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
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/gems/math/float16.hpp"
#include "messages/element_type.capnp.h"

namespace isaac {
namespace details {

template <typename T>
struct ElementType;

template <>
struct ElementType<uint8_t> {
  static constexpr ::ElementType value = ::ElementType::UINT8;
};

template <>
struct ElementType<uint16_t> {
  static constexpr ::ElementType value = ::ElementType::UINT16;
};
template <>
struct ElementType<uint32_t> {
  static constexpr ::ElementType value = ::ElementType::UINT32;
};

template <>
struct ElementType<uint64_t> {
  static constexpr ::ElementType value = ::ElementType::UINT64;
};
template <>
struct ElementType<int8_t> {
  static constexpr ::ElementType value = ::ElementType::INT8;
};

template <>
struct ElementType<int16_t> {
  static constexpr ::ElementType value = ::ElementType::INT16;
};
template <>
struct ElementType<int32_t> {
  static constexpr ::ElementType value = ::ElementType::INT32;
};

template <>
struct ElementType<int64_t> {
  static constexpr ::ElementType value = ::ElementType::INT64;
};

template <>
struct ElementType<float16> {
  static constexpr ::ElementType value = ::ElementType::FLOAT16;
};

template <>
struct ElementType<float> {
  static constexpr ::ElementType value = ::ElementType::FLOAT32;
};

template <>
struct ElementType<double> {
  static constexpr ::ElementType value = ::ElementType::FLOAT64;
};

}  // namespace details
}  // namespace isaac
