/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <shared_mutex>

#include "engine/alice/backend/component_backend.hpp"
#include "engine/alice/components/Pose.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"
#include "engine/gems/uuid/uuid.hpp"

namespace isaac {
namespace alice {

// Manages a pose graph for pose components
class PoseBackend : public ComponentBackend<Pose> {
 public:
  // Gets the latest 3D pose lhs_T_rhs for the given time
  std::optional<Pose3d> get(const Uuid& lhs, const Uuid& rhs, double stamp) const;
  // Sets 3D pose lhs_T_rhs (and rhs_T_lhs) for the given time
  bool set(const Uuid& lhs, const Uuid& rhs, double stamp, const Pose3d& lhs_T_rhs);

  // Creates a full copy of the pose tree
  pose_tree::PoseTree clonePoseTree() const;

  // Creates a copy of the pose tree where each edge contains only the latest pose
  pose_tree::PoseTree cloneLatestPoseTree() const;

 private:
  // TODO Switch to std::shared_mutex (C++17) once available
  mutable std::shared_timed_mutex pose_tree_mutex_;
  pose_tree::PoseTree pose_tree_;
};

}  // namespace alice
}  // namespace isaac
