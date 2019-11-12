/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pose_backend.hpp"

#include "engine/alice/components/Pose.hpp"
#include "engine/gems/sight/sight.hpp"

namespace isaac {
namespace alice {

std::optional<Pose3d> PoseBackend::get(const Uuid& lhs, const Uuid& rhs, double stamp) const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.get(lhs, rhs, stamp);
}

bool PoseBackend::set(const Uuid& lhs, const Uuid& rhs, double stamp, const Pose3d& lhs_T_rhs) {
  std::unique_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.set(lhs, rhs, stamp, lhs_T_rhs);
}

pose_tree::PoseTree PoseBackend::clonePoseTree() const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_;
}

pose_tree::PoseTree PoseBackend::cloneLatestPoseTree() const {
  std::shared_lock<std::shared_timed_mutex> lock(pose_tree_mutex_);
  return pose_tree_.latest();
}

}  // namespace alice
}  // namespace isaac
