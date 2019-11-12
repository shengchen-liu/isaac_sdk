/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>

#include "engine/core/assert.hpp"
#include "engine/core/epsilon.hpp"
#include "engine/core/math/types.hpp"
#include "engine/core/math/utils.hpp"

namespace isaac {

// Evaluates a Catmull-Rom spline at given curve time `time`. This version is for a basic curve with
// four control points. Note that the values in `times` must be in strictly increasing order.
// If `time` falls out of the given series `times` extrapolation will be used.
template <typename K, int N>
Vector<K, N> CatmullRomSpline4(const Vector<K, N>* points, const K* times, K time) {
  const K dt0 = time - times[0];
  const K dt1 = time - times[1];
  const K dt2 = time - times[2];
  const K dt3 = time - times[3];
  const K d10 = times[1] - times[0];
  const K d20 = times[2] - times[0];
  const K d21 = times[2] - times[1];
  const K d31 = times[3] - times[1];
  const K d32 = times[3] - times[2];
  const K d_21_31 = d21*d31;
  const K d_20_21 = d20*d21;
  const K dt1d2 = dt1*dt2;
  const K dt0t2t2 = dt0*dt2*dt2;
  const K dt1t1t3 = dt1*dt1*dt3;
  const K w0 = - dt1d2*dt2 / (d10*d_20_21);
  const K w1 = dt0t2t2 / (d_20_21*d21) + dt0t2t2 / (d10*d_20_21) + dt1d2*dt3 / (d21*d_21_31);
  const K w2 = - (dt0*dt1d2 / (d_20_21*d21) + dt1t1t3 / (d21*d_21_31) + dt1t1t3 / (d_21_31*d32));
  const K w3 = dt1*dt1d2 / (d_21_31*d32);
  return w0*points[0] + w1*points[1] + w2*points[2] + w3*points[3];
}

// Evaluates a Catmull-Rom spline at given curve time `time`. This version is for a curve with
// many sample points. Note that the values in `times` must be in strictly increasing order.
// If `time` falls out of the given series `times` extrapolation will be used.
template <typename K, int N>
Vector<K, N> CatmullRomSpline(const Vector<K, N>* points, const K* times, size_t count, K time) {
  ASSERT(count >= 4, "Not enough sample points. Need at least 4, but got %zd", count);
  // Find the correct sample points. We would like to find `index` such that:
  //   times[index] < times[index + 1] <= time < times[index + 2] < times[index + 3]
  // This will assure that we have one sample point before and after the interval in which `time`
  // is falling.
  const int index = Clamp<int>(std::lower_bound(times, times + count, time) - times - 1,
                               0, count - 4);
  // Now we can do basic 4-point interpolation
  return CatmullRomSpline4(points + index, times + index, time);
}

// Computes curve times based on control points. The `knot` parameter defines the shape of the
// curve. For a centripetal spline choose 0.5 (default), for a uniform spline choose 0, for a
// chordal spline choose 1. Memory for `times` must be allocated be the caller.
// NOTE: The first element of `times` *must be initialized* to the desired start curve time.
template <typename K, int N>
bool CatmullRomComputeCurveTimes(const Vector<K, N>* points, K* times, size_t count,
                                 K knot = K(0.5)) {
  ASSERT(K(0) <= knot && knot <= K(1), "`knot` parameter (%f) must be in range [0, 1]", knot);
  // We would like to compute ||d||^knot = (d*d)^(1/2)^knot = (d*d)^(knot/2).
  const K exponent = knot / K(2);
  for (size_t i = 1; i < count; i++) {
    const K delta = std::pow((points[i] - points[i - 1]).squaredNorm(), exponent);
    if (delta < 0 || IsAlmostZero(delta)) return false;
    times[i] = times[i - 1] + delta;
  }
  return true;
}

}  // namespace isaac
