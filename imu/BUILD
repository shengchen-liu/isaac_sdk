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
    name = "imu_lsm6ds33",
    srcs = ["ImuDriver.cpp"],
    hdrs = ["ImuDriver.hpp"],
    deps = [
        "//engine/gems/state:io",
        "//apps/segway_rmp_lawson/imu/gems:segway",
        "//apps/segway_rmp_lawson/imu/gems:RTIMUHal",
        "//apps/segway_rmp_lawson/imu/gems:RTIMUSettings",
        "//apps/segway_rmp_lawson/imu/gems:RTMath",
        "//apps/segway_rmp_lawson/imu/gems:RTFusion",
        "//apps/segway_rmp_lawson/imu/gems:RTIMU",
        "//apps/segway_rmp_lawson/imu/gems:RTFusionKalman4",

    ]
)

isaac_app(
    name = "imu",
    data = [],
    modules = [
        "//apps/segway_rmp_lawson/imu:imu_lsm6ds33",
        "imu",
    ],
)

isaac_app(
    name = "lsm6ds33",
    modules = [
        "//apps/segway_rmp_lawson/imu:imu_lsm6ds33",
        "imu"
    ],
)

