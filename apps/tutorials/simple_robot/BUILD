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
    name = "simple_robot",
    data = [
        "//apps/carter:2d_carter_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
    ],
    modules = [
        "imu",
        "//apps/tutorials/imu:imu_lsm6ds33",
        "navigation",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
    ],
)
isaac_app(
    name = "simple_robot_imu_lsm6ds33",
    data = [
        "//apps/carter:2d_carter_subgraph",
        "//packages/navigation/apps:differential_base_commander_subgraph",
    ],
    modules = [
        "imu",
        "//apps/tutorials/imu:imu_lsm6ds33",
        "navigation",
        "segway",
        "sensors:joystick",
        "velodyne_lidar",
    ],
)