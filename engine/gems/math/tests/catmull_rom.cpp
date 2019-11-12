/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <iostream>

#include "engine/gems/math/catmull_rom.hpp"
#include "engine/gems/math/test_utils.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(CatmullRomSpline, CurveTimes) {
  // Take delta steps of 0.25, 1.0 and 2.25 along the vector {2,1}. This will result in delta curve
  // time steps of 0.5, 1.0 and 1.5.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{0.5, 0.25}, Vector2d{2.5, 1.25}, Vector2d{7.0, 3.5}};
  double times[4];
  times[0] = 1.3;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  const double expected_step = std::pow(5.0, 0.25);
  EXPECT_NEAR(times[0],                      1.3, 1e-9);
  EXPECT_NEAR(times[1], 0.50*expected_step + 1.3, 1e-9);
  EXPECT_NEAR(times[2], 1.50*expected_step + 1.3, 1e-9);
  EXPECT_NEAR(times[3], 3.00*expected_step + 1.3, 1e-9);
}

TEST(CatmullRomSpline, CurveTimesRepeating) {
  // Check that computing curve times for list with duplicated points is invalid.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{0.5, 0.25}, Vector2d{0.5, 0.25}, Vector2d{3.0, 4.5}};
  double times[4];
  times[0] = 1.3;
  ASSERT_FALSE(CatmullRomComputeCurveTimes(points, times, 4));
}

TEST(CatmullRomSpline, InterpolateLinearSpline) {
  // Check interpolation for a short linear curve.
  const Vector2d points[4] = {
      Vector2d{0, 0}, Vector2d{2, 1}, Vector2d{4, 2}, Vector2d{6, 3}};
  double times[4];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  for (double t = 0.0; t <= 1.0; t += 0.1) {
    ISAAC_EXPECT_VEC_NEAR((1.0 - t)*points[1] + t*points[2],
                          CatmullRomSpline4(points, times, (1.0 - t)*times[1] + t*times[2]), 1e-9);
  }
}

TEST(CatmullRomSpline, EvaluateAtPoints4) {
  // Check interpolation at sample points using 4-point version. The spline is guaranteed to go
  // through the second and third sample point.
  const Vector2d points[4] = {
      Vector2d{0.303656, 0.316798}, Vector2d{0.677207, 1.17211},
      Vector2d{0.896317, 1.72843}, Vector2d{1.0771, 1.91572}};
  double times[4];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, 4));
  ISAAC_EXPECT_VEC_NEAR(points[1], CatmullRomSpline4(points, times, times[1]), 1e-9);
  ISAAC_EXPECT_VEC_NEAR(points[2], CatmullRomSpline4(points, times, times[2]), 1e-9);
}

TEST(CatmullRomSpline, EvaluateAtPointsN) {
  constexpr int kNumPoints = 12;
  // Check interpolation at sample points using N-point version. The spline is guaranteed to go
  // through all sample points except for the first and the last point.
  const Vector2d points[kNumPoints] = {
      Vector2d{0.234099, 0.893742}, Vector2d{1.14441, 1.3151}, Vector2d{1.62298, 1.70618},
      Vector2d{2.40708, 2.38953}, Vector2d{2.97525, 2.50614}, Vector2d{3.90171, 2.62645},
      Vector2d{4.3033, 3.38146}, Vector2d{4.73817, 3.80319}, Vector2d{5.42781, 4.4778},
      Vector2d{6.22677, 5.42769}, Vector2d{6.47075, 6.25194}, Vector2d{7.22324, 6.84748}};
  double times[kNumPoints];
  times[0] = 0.0;
  ASSERT_TRUE(CatmullRomComputeCurveTimes(points, times, kNumPoints));
  for (size_t i = 1; i + 1 < kNumPoints; i++) {
    ISAAC_EXPECT_VEC_NEAR(points[i], CatmullRomSpline(points, times, kNumPoints, times[i]), 1e-9);
  }
}

}  // namespace isaac
