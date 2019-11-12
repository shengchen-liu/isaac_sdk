/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Pose.hpp"

#include <string>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/pose_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

void Pose::initialize() {
  backend_ = node()->app()->backend()->getBackend<PoseBackend>();
}

void Pose::start() {
  reportSuccess();  // do not participate in status updates TODO solver differently
}

void Pose::deinitialize() {
  backend_ = nullptr;
}

Pose3d Pose::get(const FrameId& lhs, const FrameId& rhs, double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  ASSERT(maybe, "Could not get the transformation %s_T_%s.", lhs.c_str(), rhs.c_str());
  return *maybe;
}

Pose2d Pose::getPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const {
  return get(lhs, rhs, time).toPose2XY();
}

std::optional<Pose3d> Pose::tryGet(const FrameId& lhs, const FrameId& rhs, double time) const {
  const Uuid lhs_uuid = Uuid::FromAsciiString(lhs);
  const Uuid rhs_uuid = Uuid::FromAsciiString(rhs);
  if (use_versioning_) {
    return local_pose_tree_.get(lhs_uuid, rhs_uuid, time);
  } else {
    return backend_->get(lhs_uuid, rhs_uuid, time);
  }
}

std::optional<Pose2d> Pose::tryGetPose2XY(const FrameId& lhs, const FrameId& rhs,
                                          double time) const {
  auto maybe = tryGet(lhs, rhs, time);
  if (maybe) {
    return maybe->toPose2XY();
  } else {
    return std::nullopt;
  }
}

bool Pose::set(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time) {
  const Uuid lhs_uuid = Uuid::FromAsciiString(lhs);
  const Uuid rhs_uuid = Uuid::FromAsciiString(rhs);
  const bool ok1 = backend_->set(lhs_uuid, rhs_uuid, time, lhs_T_rhs);
  if (use_versioning_) {
    const bool ok2 = local_pose_tree_.set(lhs_uuid, rhs_uuid, time, lhs_T_rhs);
    ASSERT(ok1 == ok2, "Setting the transformation %s_T_%s succeeded only in either master or the "
           "current version. This created an inconsistent pose tree and currently is an "
           "unrecoverable error.", lhs.c_str(), rhs.c_str());
  }
  return ok1;
}

bool Pose::set(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time) {
  return set(lhs, rhs, Pose3d::FromPose2XY(lhs_T_rhs), time);
}

void Pose::enableVersioning() {
  if (use_versioning_) {
    return;
  }
  use_versioning_ = true;
  onBeforeExec();
}

void Pose::disableVersioning() {
  if (!use_versioning_) {
    return;
  }
  onAfterExec();
  use_versioning_ = false;
}

void Pose::onBeforeExec() {
  if (use_versioning_) {
    local_pose_tree_ = backend_->clonePoseTree();
  }
}

void Pose::onAfterExec() {
  if (use_versioning_) {
    local_pose_tree_ = pose_tree::PoseTree{};
  }
}

}  // namespace alice
}  // namespace isaac
