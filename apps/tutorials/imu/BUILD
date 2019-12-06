"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "imu_components1",
    srcs = ["ImuDriver.cpp"],
    hdrs = ["ImuDriver.hpp"],
    deps = [
        "//engine/gems/state:io",
        "//messages/state:differential_base",
        "//apps/tutorials/imu/gems:segway",
        "//apps/tutorials/imu/gems:RTIMUHal",
        "//apps/tutorials/imu/gems:RTIMUSettings",
        "//apps/tutorials/imu/gems:RTMath",
        "//apps/tutorials/imu/gems:RTFusion",
        "//apps/tutorials/imu/gems:RTIMU"
    ]
)

isaac_app(
    name = "imu",
    data = ["fast_ping.json"],
    modules = [
        "//apps/tutorials/imu:imu_components1",
        "//apps/tutorials/webinar_1:go_to_mockup",
    ],
)

