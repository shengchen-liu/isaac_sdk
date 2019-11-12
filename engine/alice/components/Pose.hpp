/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <map>
#include <string>

#include "engine/alice/component.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/pose_tree/pose_tree.hpp"

namespace isaac {
namespace alice {

class PoseBackend;

// Provides convenience functions to access 3D transformations from the application wide pose tree.
//
// This component is added to every node by default and does not have to be added manually.
//
// Poses use 64-bit floating point types and are 3-dimensional. All coordinate frames for the whole
// application are stored in a single central pose tree.
//
// All functions below accept two coordinate frames: `lhs` and `rhs`. This refers to the pose
// lhs_T_rhs which is the relative transformations between these two coordinate frames. In
// particular the following equations hold:
//   p_lhs = lhs_T_rhs * p_rhs
//   a_T_c = a_T_b * b_T_c
//
// Not all coordinate frames are connected. If this is the case or either of the two coordinate
// frames does not exist the pose is said to be "invalid".
class Pose : public Component {
 public:
  void initialize() override;
  void start() override;
  void deinitialize() override;

  using FrameId = std::string;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs`. This function will assert
  // if the pose is invalid.
  Pose3d get(const FrameId& lhs, const FrameId& rhs, double time) const;
  // Similar to `get`, but also converts the 3D pose to a 2D pose relative to the plane Z = 0.
  Pose2d getPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const;

  // Gives the relative pose `lhs_T_rhs` between frame `lhs` and `rhs`. This function will return
  // nullopt if the pose is invalid.
  std::optional<Pose3d> tryGet(const FrameId& lhs, const FrameId& rhs, double time) const;
  // Similar to `tryGet`, but also converts the 3D pose to a 2D pose relative to the plane Z = 0.
  std::optional<Pose2d> tryGetPose2XY(const FrameId& lhs, const FrameId& rhs, double time) const;

  // Sets the relative pose between two coordinate frames.  If the parameter is not specified and
  // the pose could not be set the function will return false.
  bool set(const FrameId& lhs, const FrameId& rhs, const Pose3d& lhs_T_rhs, double time);
  // Similar to `set` but for setting a pose in the Z = 0 plane.
  bool set(const FrameId& lhs, const FrameId& rhs, const Pose2d& lhs_T_rhs, double time);

  // If versioning is enabled the pose will create a local version of the pose tree for the
  // duration of the tick. By default versioning is disabled. Versioning can be enabled or disabled
  // at any time and will go into effect immediately.
  void enableVersioning();
  void disableVersioning();

  // TODO Hide these functions
  void onBeforeExec();
  void onAfterExec();

 private:
  friend class PoseBackend;

  PoseBackend* backend_;

  // If versioning is enabled a local copy of the pose tree will be used during a tick which
  // can not be changed by other threads. All write operations are mirrored into the global pose
  // tree immediately.
  bool use_versioning_ = false;
  // A local copy of the pose tree used when versioning is enabled
  pose_tree::PoseTree local_pose_tree_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Pose)
