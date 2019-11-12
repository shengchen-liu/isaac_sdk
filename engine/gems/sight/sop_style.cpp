/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "sop_style.hpp"

#include <utility>

namespace isaac {
namespace sight {

const Json& ToJson(const SopStyle& style) {
  return style.json_;
}

Json ToJson(SopStyle&& style) {
  return std::move(style.json_);
}

}  // namespace sight
}  // namespace isaac
