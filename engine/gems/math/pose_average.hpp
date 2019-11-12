/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/epsilon.hpp"
#include "engine/core/math/pose2.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace math {

// Compute the weighted average for a stream of Pose2
template <typename K>
class Pose2Average {
 public:
  // Adds another pose to the average
  void add(const Pose2<K>& pose, K weight) {
    average_translation_ += weight * pose.translation;
    average_rotation_ += weight * pose.rotation.asDirection();
    total_weight_ += weight;
  }
  void add(const Pose2<K>& pose) {
    average_translation_ += pose.translation;
    average_rotation_ += pose.rotation.asDirection();
    total_weight_ += K(1);
  }

  // Gets the current average. Will return nullopt in case the average could not be determined
  // uniquely either because it is ambiguous or all weights so far were 0.
  std::optional<Pose2<K>> computeAverage() const {
    // Check that we got any elements with non-zero weights
    if (IsAlmostZero(total_weight_)) {
      return std::nullopt;
    }
    // Check that rotations are not degenerated. We choose a larger epsilon to compensate for
    // inaccuracies with sin/cos computations. Note that this should really only be relevant for
    // truely degenerate cases like average between -90 and +90 degree and should not matter for
    // any real example.
    constexpr K kZeroTolerance = K(10.0) * MachineEpsilon<K>;
    const K average_rotation_norm = average_rotation_.norm();
    if (std::abs(average_rotation_norm) <= kZeroTolerance) {
      return std::nullopt;
    }
    return Pose2<K>{SO2<K>::FromNormalized(average_rotation_ / average_rotation_norm),
                    average_translation_ / total_weight_};
  }

  // Returns the sum of all weights so far
  K total_weight() const {
    return total_weight_;
  }

 private:
  Vector2<K> average_translation_ = Vector2<K>::Zero();
  Vector2<K> average_rotation_ = Vector2<K>::Zero();
  K total_weight_ = K(0);
};

using Pose2AverageD = Pose2Average<double>;
using Pose2AverageF = Pose2Average<float>;

}  // namespace math
}  // namespace isaac
