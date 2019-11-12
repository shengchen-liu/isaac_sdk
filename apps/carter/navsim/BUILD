"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_app(
    name = "navsim_navigate",
    data = [
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navsim/apps:minisight_subgraph",
        "//packages/navsim/apps:navsim_subgraph",
        "//packages/navsim/maps",
        "//packages/navsim/robots",
    ],
    modules = [
        "map",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_gmapping",
    data = [
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_imu_odometry_subgraph",
        "//packages/navigation/apps:local_map_subgraph",
        "//packages/navsim/apps:navsim_subgraph",
        "//packages/navsim/robots",
    ],
    modules = [
        "navigation",
    ],
    visibility = ["//visibility:public"],
)
