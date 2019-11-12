/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <random>

#include "engine/core/math/pose2.hpp"
#include "engine/core/math/pose3.hpp"
#include "engine/core/math/so2.hpp"
#include "engine/core/math/so3.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {

// A random vector using normal distributions with zero mean for each axis
template <typename K, int N, typename RandomEngine>
auto VectorNormalDistribution(RandomEngine& rng, size_t n = N) {
  std::normal_distribution<K> normal;
  Vector<K, N> result(n);
  for (int i = 0; i < result.size(); i++) {
    result[i] = normal(rng);
  }
  return result;
}
template <typename Derived, typename RandomEngine>
auto VectorNormalDistribution(const Eigen::MatrixBase<Derived>& sigma, RandomEngine& rng) {
  std::normal_distribution<typename Derived::Scalar> normal;
  auto result = sigma.eval();
  for (int i = 0; i < result.size(); i++) {
    result[i] *= normal(rng);
  }
  return result;
}
template <typename Derived1, typename Derived2, typename RandomEngine>
auto VectorNormalDistribution(const Eigen::MatrixBase<Derived1>& mean,
    const Eigen::MatrixBase<Derived2>& sigma, RandomEngine& rng) {
  return (mean + VectorNormalDistribution(sigma, rng)).eval();
}

// Picks a random point on a N-dimensional sphere (uniformly distributed)
template <typename K, int N, typename RandomEngine>
Vector<K, N> RandomPointOnSphere(RandomEngine& rng, size_t n = N) {
  // TODO This theoretically has problems because we could sample all zeros many times, but most
  // likely this will happen extremly rarely.
  while (true) {
    auto result = VectorNormalDistribution(Vector<K, N>::Constant(n), rng);
    K length = result.norm();
    if (IsAlmostZero(length)) {
      continue;
    }
    return result / length;
  }
}

// A random pose with normal distribution
// This will only give normally distributed rotations for small angle standard deviations.
template <typename K, typename RandomEngine>
Pose2<K> PoseNormalDistribution(const Vector<K, 3>& sigma, RandomEngine& rng) {
  std::normal_distribution<K> normal;
  Pose2<K> delta{
      SO2<K>::FromAngle(sigma[2] * normal(rng)),
      VectorNormalDistribution(sigma.template head<2>(), rng)};
  return delta;
}

template <typename K, typename Derived, typename RandomEngine>
Pose2<K> PoseNormalDistribution(const Pose2<K>& mean, const Eigen::MatrixBase<Derived>& sigma,
                                RandomEngine& rng) {
  return mean * PoseNormalDistribution(sigma.eval(), rng);
}

template <typename K, typename RandomEngine>
Pose3<K> PoseNormalDistribution(const Vector<K, 4>& sigma, RandomEngine& rng) {
  std::normal_distribution<K> normal;
  Pose3<K> delta{
      SO3<K>::FromAxisAngle(RandomPointOnSphere<double, 3>(rng), sigma[3] * normal(rng)),
      VectorNormalDistribution(sigma.template head<3>(), rng)};
  return delta;
}

}  // namespace isaac
