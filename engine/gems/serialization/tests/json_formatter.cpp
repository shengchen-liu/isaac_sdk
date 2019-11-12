/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <string>

#include "engine/gems/serialization/json_formatter.hpp"
#include "gtest/gtest.h"

namespace isaac {
namespace serialization {

TEST(Serialization, Double) {
  const double actual = -1.2;
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), actual);
}

TEST(Serialization, DoubleTryGetFailure1) {
  Json json;
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, DoubleTryGetFailure2) {
  Json json;
  Set(json, false);
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, DoubleFromInt) {
  const int actual = -1;
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), actual);
}

TEST(Serialization, DoubleFromString) {
  const std::string actual = "this is a string";
  Json json;
  Set(json, actual);
  auto maybe = TryGet<double>(json);
  ASSERT_FALSE(maybe);
}

TEST(Serialization, Pair) {
  using T = std::pair<double, int>;
  T val{2.2, -1};

  Json json;
  Set(json, val);

  auto maybe = TryGet<T>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

TEST(Serialization, Map) {
  std::map<std::string, std::vector<std::string>> val;
  val["a"] = {"1", "2", "3"};
  val["b"];

  Json json;
  Set(json, val);

  auto maybe = TryGet<decltype(val)>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

TEST(Serialization, Circled) {
  geometry::Circled val{{4.4, 5.5}, 6.6};

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 5.5}, 6.6};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::Circled>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, CircledVerbose) {
  geometry::Circled val{{4.4, 5.5}, 6.6};

  Json json;
  json["center"] = {4.4, 5.5};
  json["radius"] = 6.6;

  auto maybe = TryGet<geometry::Circled>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, Spheref) {
  geometry::Spheref val{{-2.2, 4.4, 5.5}, 6.6};

  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::Spheref>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, SpherefVerbose) {
  geometry::Spheref val{{-2.2, 4.4, 5.5}, 6.6};

  Json json;
  json["center"] = {-2.2, 4.4, 5.5};
  json["radius"] = 6.6;

  auto maybe = TryGet<geometry::Spheref>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->center, val.center);
  EXPECT_EQ(maybe->radius, val.radius);
}

TEST(Serialization, Rectangled) {
  geometry::Rectangled val = geometry::Rectangled::FromOppositeCorners({4.4, 5.5}, {6.6, 7.7});

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 6.6}, {5.5, 7.7}};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::Rectangled>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->min(), val.min());
  EXPECT_EQ(maybe->max(), val.max());
}

TEST(Serialization, RectangledVerbose) {
  geometry::Rectangled val = geometry::Rectangled::FromOppositeCorners({4.4, 7.7}, {6.6, 5.5});

  // JSON defined by bounding box
  Json json_from_bounds;
  json_from_bounds["bounds"] = {{4.4, 6.6}, {5.5, 7.7}};

  auto maybe_from_bounds = TryGet<geometry::Rectangled>(json_from_bounds);
  ASSERT_TRUE(maybe_from_bounds);
  EXPECT_EQ(maybe_from_bounds->min(), val.min());
  EXPECT_EQ(maybe_from_bounds->max(), val.max());

  // JSON defined by opposite corners
  Json json_from_corners;
  json_from_corners["corner_1"] = {4.4, 7.7};
  json_from_corners["corner_2"] = {6.6, 5.5};

  auto maybe_from_corners = TryGet<geometry::Rectangled>(json_from_corners);
  ASSERT_TRUE(maybe_from_corners);
  EXPECT_EQ(maybe_from_corners->min(), val.min());
  EXPECT_EQ(maybe_from_corners->max(), val.max());

  // JSON defined by center and dimensions
  Json json_from_dimensions;
  json_from_dimensions["center"] = {5.5, 6.6};
  json_from_dimensions["dimensions"] = {2.2, 2.2};

  auto maybe_from_dimensions = TryGet<geometry::Rectangled>(json_from_dimensions);
  ASSERT_TRUE(maybe_from_dimensions);
  EXPECT_EQ(maybe_from_dimensions->min(), val.min());
  EXPECT_NEAR(maybe_from_dimensions->max()[0], val.max()[0], 1e-15);
  EXPECT_EQ(maybe_from_dimensions->max()[1], val.max()[1]);
}

TEST(Serialization, Boxf) {
  geometry::Boxf val = geometry::Boxf::FromOppositeCorners({4.4, 7.7, 2.2}, {6.6, 5.5, 3.3});

  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::Boxf>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->min(), val.min());
  EXPECT_EQ(maybe->max(), val.max());
}

TEST(Serialization, BoxfVerbose) {
  geometry::Boxf val = geometry::Boxf::FromOppositeCorners({4.4, 7.7, 2.2}, {6.6, 5.5, 3.3});

  // JSON defined by bounding box
  Json json_from_bounds;
  json_from_bounds["bounds"] = {{4.4, 6.6}, {5.5, 7.7}, {2.2, 3.3}};

  auto maybe_from_bounds = TryGet<geometry::Boxf>(json_from_bounds);
  ASSERT_TRUE(maybe_from_bounds);
  EXPECT_EQ(maybe_from_bounds->min(), val.min());
  EXPECT_EQ(maybe_from_bounds->max(), val.max());

  // JSON defined by opposite corners
  Json json_from_corners;
  json_from_corners["corner_1"] = {4.4, 7.7, 2.2};
  json_from_corners["corner_2"] = {6.6, 5.5, 3.3};

  auto maybe_from_corners = TryGet<geometry::Boxf>(json_from_corners);
  ASSERT_TRUE(maybe_from_corners);
  EXPECT_EQ(maybe_from_corners->min(), val.min());
  EXPECT_EQ(maybe_from_corners->max(), val.max());

  // JSON defined by center and dimensions
  Json json_from_dimensions;
  json_from_dimensions["center"] = {5.5, 6.6, 2.75};
  json_from_dimensions["dimensions"] = {2.2, 2.2, 1.1};

  auto maybe_from_dimensions = TryGet<geometry::Boxf>(json_from_dimensions);
  ASSERT_TRUE(maybe_from_dimensions);
  EXPECT_EQ(maybe_from_dimensions->min(), val.min());
  EXPECT_NEAR(maybe_from_dimensions->max()[0], val.max()[0], 1e-5);
  EXPECT_EQ(maybe_from_dimensions->max()[1], val.max()[1]);
}

TEST(Serialization, LineSegment2d) {
  geometry::LineSegment2d val = geometry::LineSegment2d({4.4, 5.5}, {6.6, 7.7});

  Json json;
  Set(json, val);

  Json json_direct = {{4.4, 5.5}, {6.6, 7.7}};
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<geometry::LineSegment2d>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->a(), val.a());
  EXPECT_EQ(maybe->b(), val.b());
}

TEST(Serialization, LineSegment2dVerbose) {
  geometry::LineSegment2d val = geometry::LineSegment2d({4.4, 5.5}, {6.6, 7.7});

  // JSON defined by points
  Json json_from_points;
  json_from_points["point_1"] = {4.4, 5.5};
  json_from_points["point_2"] = {6.6, 7.7};

  auto maybe_from_points = TryGet<geometry::LineSegment2d>(json_from_points);
  ASSERT_TRUE(maybe_from_points);
  EXPECT_EQ(maybe_from_points->a(), val.a());
  EXPECT_EQ(maybe_from_points->b(), val.b());
}

TEST(Serialization, LineSegment3f) {
  geometry::LineSegment3f val = geometry::LineSegment3f({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});
  Json json;
  Set(json, val);

  auto maybe = TryGet<geometry::LineSegment3f>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->a(), val.a());
  EXPECT_EQ(maybe->b(), val.b());
}

TEST(Serialization, LineSegment3fVerbose) {
  geometry::LineSegment3f val = geometry::LineSegment3f({1.0, 2.0, 3.0}, {4.0, 5.0, 6.0});

  // JSON defined by points
  Json json_from_points;
  json_from_points["point_1"] = {1.0, 2.0, 3.0};
  json_from_points["point_2"] = {4.0, 5.0, 6.0};

  auto maybe_from_points = TryGet<geometry::LineSegment3f>(json_from_points);
  ASSERT_TRUE(maybe_from_points);
  EXPECT_EQ(maybe_from_points->a(), val.a());
  EXPECT_EQ(maybe_from_points->b(), val.b());
}

TEST(Serialization, SopAsset) {
  sight::SopAsset val = sight::SopAsset::FromName("my_favorite_robot");

  Json json;
  Set(json, sight::SopAsset::FromName("my_favorite_robot"));

  Json json_direct = "my_favorite_robot";
  EXPECT_TRUE(json == json_direct);

  auto maybe = TryGet<sight::SopAsset>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe->name(), val.name());
}

TEST(Serialization, SopAssetVerbose) {
  sight::SopAsset val = sight::SopAsset::FromName("my_favorite_robot");

  // JSON defined by name
  Json json_from_name;
  json_from_name["name"] = "my_favorite_robot";

  auto maybe_from_name = TryGet<sight::SopAsset>(json_from_name);
  ASSERT_TRUE(maybe_from_name);
  EXPECT_EQ(maybe_from_name->name(), val.name());
}

TEST(Serialization, Pixel3ub) {
  Pixel3ub val{120, 150, 200};

  Json json;
  Set(json, val);

  auto maybe_rgb = TryGet<Pixel3ub>(json);
  ASSERT_TRUE(maybe_rgb);
  EXPECT_EQ(maybe_rgb.value().size(), 3);
  EXPECT_EQ(maybe_rgb.value()[0], val[0]);
  EXPECT_EQ(maybe_rgb.value()[1], val[1]);
  EXPECT_EQ(maybe_rgb.value()[2], val[2]);

  // Add alpha channel if not supplied in Json, default value 255 (opaque)
  auto maybe_rgba = TryGet<Pixel4ub>(json);
  ASSERT_TRUE(maybe_rgba);
  EXPECT_EQ(maybe_rgba.value().size(), 4);
  EXPECT_EQ(maybe_rgba.value()[0], val[0]);
  EXPECT_EQ(maybe_rgba.value()[1], val[1]);
  EXPECT_EQ(maybe_rgba.value()[2], val[2]);
  EXPECT_EQ(maybe_rgba.value()[3], 255);
}

TEST(Serialization, Pixel4ub) {
  Pixel4ub val{120, 150, 200, 100};

  Json json;
  Set(json, val);

  // Drop alpha channel when reading Pixel3ub
  auto maybe_rgb = TryGet<Pixel3ub>(json);
  ASSERT_TRUE(maybe_rgb);
  EXPECT_EQ(maybe_rgb.value().size(), 3);
  EXPECT_EQ(maybe_rgb.value()[0], val[0]);
  EXPECT_EQ(maybe_rgb.value()[1], val[1]);
  EXPECT_EQ(maybe_rgb.value()[2], val[2]);

  auto maybe_rgba = TryGet<Pixel4ub>(json);
  ASSERT_TRUE(maybe_rgba);
  EXPECT_EQ(maybe_rgba.value().size(), 4);
  EXPECT_EQ(maybe_rgba.value()[0], val[0]);
  EXPECT_EQ(maybe_rgba.value()[1], val[1]);
  EXPECT_EQ(maybe_rgba.value()[2], val[2]);
  EXPECT_EQ(maybe_rgba.value()[3], val[3]);
}

TEST(Serialization, Vector) {
  using T = Vector<double, 2>;
  T val{12.3, -45.6};

  Json json;
  Set(json, val);

  auto maybe = TryGet<T>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), val);
}

enum class TestEnum {
  kNone,
  kOne,
  kTwo,
  kThree
};

NLOHMANN_JSON_SERIALIZE_ENUM(TestEnum, {
  {TestEnum::kNone, nullptr},
  {TestEnum::kOne, "one cake"},
  {TestEnum::kTwo, "two flake"},
  {TestEnum::kThree, "and three"},
});

TEST(Serialization, Enum) {
  TestEnum x = TestEnum::kOne;

  Json json;
  Set(json, x);

  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(maybe.value(), x);
}

TEST(Serialization, EnumFailure1) {
  Json json;
  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(*maybe, TestEnum::kNone);
}

TEST(Serialization, EnumFailure2) {
  Json json;
  Set(json, 1.3);
  auto maybe = TryGet<TestEnum>(json);
  ASSERT_TRUE(maybe);
  EXPECT_EQ(*maybe, TestEnum::kNone);
}

}  // namespace serialization
}  // namespace isaac
