"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "imu_lsm6ds33",
    deps = [
        ":imu_lsm6ds33_driver",
    ],
)

isaac_component(
    name = "imu_lsm6ds33_driver",
    deps = [
        "//engine/gems/state:io",
        "//messages/state:differential_base",
        "//packages/imu_lsm6ds33/gems:imu_lsm6ds33",
    ],
)
