/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/sample_cloud/sample_cloud.hpp"

#include <vector>

#include "engine/gems/math/float16.hpp"
#include "gtest/gtest.h"

namespace isaac {

TEST(SampleCloud, Creation) {
  SampleCloud2d pc1;
  EXPECT_EQ(pc1.size(), 0);
  SampleCloud2f pc2(10);
  EXPECT_EQ(pc2.size(), 10);
  SampleCloud4i pc3(100);
  EXPECT_EQ(pc3.size(), 100);
  SampleCloudPlanar<float, 15> pc4(100);
  EXPECT_EQ(pc4.size(), 100);
}

TEST(SampleCloud, Views) {
  SampleCloud2d pc1(100);
  SampleCloud2f pc2(10);

  SampleCloudView2d pcv1 = pc1.view();
  EXPECT_EQ(pcv1.size(), 100);
  SampleCloudConstView2f pccv2 = pc2.const_view();
  EXPECT_EQ(pccv2.size(), 10);
}

TEST(SampleCloud, ChannelsInterleaved) {
  SampleCloud3d pc(10);
  auto* channel0 = pc.channel(0);
  auto* channel1 = pc.channel(1);
  auto* channel2 = pc.channel(2);
  auto channel_stride = pc.channel_stride();
  EXPECT_EQ(channel_stride, 3);
  for (size_t i = 0; i < pc.size(); ++i) {
    channel0[i * channel_stride] = static_cast<double>(i);
    channel1[i * channel_stride] = static_cast<double>(i);
    channel2[i * channel_stride] = static_cast<double>(i);
  }
  for (size_t i = 0; i < pc.size(); ++i) {
    auto data = pc[i];
    EXPECT_EQ(data[0], static_cast<double>(i));
    EXPECT_EQ(data[1], static_cast<double>(i));
    EXPECT_EQ(data[2], static_cast<double>(i));
  }
}

TEST(SampleCloud, ChannelsPlanar) {
  SampleCloudPlanar3d pc(10);
  auto* channel0 = pc.channel(0);
  auto* channel1 = pc.channel(1);
  auto* channel2 = pc.channel(2);
  auto channel_stride = pc.channel_stride();
  EXPECT_EQ(channel_stride, 1);
  for (size_t i = 0; i < pc.size(); ++i) {
    channel0[i * channel_stride] = static_cast<double>(i);
    channel1[i * channel_stride] = static_cast<double>(i);
    channel2[i * channel_stride] = static_cast<double>(i);
  }
  auto pcv = pc.const_view();
  for (size_t i = 0; i < pc.size(); ++i) {
    auto data = pcv[i];
    EXPECT_EQ(data[0], static_cast<double>(i));
    EXPECT_EQ(data[1], static_cast<double>(i));
    EXPECT_EQ(data[2], static_cast<double>(i));
  }
}

TEST(SampleCloud, SetAndGetInterleaved) {
  SampleCloud3d pc(10);
  for (size_t i = 0; i < pc.size(); ++i) {
    double val = static_cast<double>(i);
    Vector3d sample = {val, val, val};
    pc[i] = sample;
  }
  for (size_t i = 0; i < pc.size(); ++i) {
    auto data = pc[i];
    EXPECT_EQ(data[0], static_cast<double>(i));
    EXPECT_EQ(data[1], static_cast<double>(i));
    EXPECT_EQ(data[2], static_cast<double>(i));
  }
}

TEST(SampleCloud, SetAndGetPlanar) {
  SampleCloudPlanar3d pc(10);
  for (size_t i = 0; i < pc.size(); ++i) {
    double val = static_cast<double>(i);
    Vector3d sample = {val, val, val};
    pc[i] = sample;
  }
  auto pcv = pc.const_view();
  for (size_t i = 0; i < pc.size(); ++i) {
    auto data = pcv[i];
    EXPECT_EQ(data[0], static_cast<double>(i));
    EXPECT_EQ(data[1], static_cast<double>(i));
    EXPECT_EQ(data[2], static_cast<double>(i));
  }
}

TEST(SampleCloud, EigenView) {
  {
    SampleCloud3d pc(200);
    for (size_t i = 0; i < pc.size(); ++i) {
      double val = static_cast<double>(i);
      Vector3d sample = {val, val, val};
      pc[i] = sample;
    }
    auto data = pc.eigen_const_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      EXPECT_EQ(data(0, i), static_cast<double>(i));
      EXPECT_EQ(data(1, i), static_cast<double>(i));
      EXPECT_EQ(data(2, i), static_cast<double>(i));
    }
  }

  {
    SampleCloudPlanar3d pc(200);
    for (size_t i = 0; i < pc.size(); ++i) {
      double val = static_cast<double>(i);
      Vector3d sample = {val, val, val};
      pc[i] = sample;
    }
    auto data = pc.eigen_const_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      EXPECT_EQ(data(0, i), static_cast<double>(i));
      EXPECT_EQ(data(1, i), static_cast<double>(i));
      EXPECT_EQ(data(2, i), static_cast<double>(i));
    }
  }

  {
    SampleCloud3d pc(200);
    auto view = pc.eigen_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      view(0, i) = static_cast<double>(i);
      view(1, i) = static_cast<double>(i);
      view(2, i) = static_cast<double>(i);
    }
    auto data = pc.eigen_const_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      EXPECT_EQ(data(0, i), static_cast<double>(i));
      EXPECT_EQ(data(1, i), static_cast<double>(i));
      EXPECT_EQ(data(2, i), static_cast<double>(i));
    }
  }

  {
    SampleCloudPlanar3d pc(200);
    auto view = pc.eigen_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      view(0, i) = static_cast<double>(i);
      view(1, i) = static_cast<double>(i);
      view(2, i) = static_cast<double>(i);
    }
    auto data = pc.eigen_const_view();
    for (size_t i = 0; i < pc.size(); ++i) {
      EXPECT_EQ(data(0, i), static_cast<double>(i));
      EXPECT_EQ(data(1, i), static_cast<double>(i));
      EXPECT_EQ(data(2, i), static_cast<double>(i));
    }
  }
}

}  // namespace isaac
