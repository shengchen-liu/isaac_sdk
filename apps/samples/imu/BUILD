"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app")

isaac_app(
    name = "imu",
    app_json_file = "imu.app.json",
    modules = [
        "imu",
    ],
)

isaac_app(
    name = "imu_lsm6ds33",
    app_json_file = "imu_lsm6ds33.app.json",
    modules = [
        "imu",
        "//apps/tutorials/imu:imu_lsm6ds33",
    ],
)
