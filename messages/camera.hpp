/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/gems/geometry/pinhole.hpp"
#include "messages/camera.capnp.h"
#include "messages/image.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace geometry {

// Reads a pinhole model from PinholeProto
template<typename K>
bool FromProto(::PinholeProto::Reader reader, Pinhole<K>& pinhole) {
  const int rows = reader.getRows();
  const int cols = reader.getCols();
  if (rows < 0) {
    LOG_ERROR("Number of rows must not be negative: %d", rows);
    return false;
  }
  if (cols < 0) {
    LOG_ERROR("Number of cols must not be negative: %d", cols);
    return false;
  }
  pinhole.rows = rows;
  pinhole.cols = cols;
  pinhole.focal = isaac::FromProto(reader.getFocal());
  pinhole.center = isaac::FromProto(reader.getCenter());
  return true;
}

// Writes a pinhole model to PinholeProto
template<typename K>
void ToProto(const Pinhole<K>& pinhole, ::PinholeProto::Builder builder) {
  builder.setRows(pinhole.rows);
  builder.setCols(pinhole.cols);
  isaac::ToProto(pinhole.focal, builder.initFocal());
  isaac::ToProto(pinhole.center, builder.initCenter());
}

}  // namespace geometry
}  // namespace isaac
