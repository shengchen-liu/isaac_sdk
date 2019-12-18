"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_doc_dep", "isaac_subgraph")

isaac_subgraph(
    name = "lawson_hardware_lsm6ds33__subgraph",
    modules = [
        "//apps/tutorials/imu:imu_lsm6ds33",
        "segway",
        "velodyne_lidar",
    ],
    subgraph = "lawson_hardware_lsm6ds33.subgraph.json",
    visibility = ["//visibility:public"],
)


isaac_subgraph(
    name = "2d_lawson_subgraph",
    data = [
        ":lawson_hardware_lsm6ds33__subgraph",
        "//packages/navigation/apps:scan_flattener_subgraph",
    ],
    modules = [],
    subgraph = "2d_lawson.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "lawson",
    data = [
        ":2d_lawson_subgraph",
        "//apps/assets/maps",
        "//apps/lawson/robots:robots",
        "//packages/navigation/apps:differential_base_commander_subgraph",
        "//packages/navigation/apps:differential_base_navigation_subgraph",
        "//packages/navigation/apps:goal_generators_subgraph",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "sensors:joystick",
        "viewers",
    ],
    script = "select_json.sh",
)

