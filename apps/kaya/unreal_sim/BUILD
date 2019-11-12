"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_app", "isaac_subgraph")

isaac_subgraph(
    name = "kaya_sim_subgraph",
    modules = [
        "flatsim",
    ],
    subgraph = "kaya_sim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "joystick",
    app_json_file = "joystick.app.json",
    data = [
        ":kaya_sim_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "flatsim",
        "navigation",
        "planner",
        "sensors:joystick",
        "viewers",
    ],
)

isaac_app(
    name = "object_detection",
    app_json_file = "object_detection.app.json",
    data = [
        ":kaya_sim_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "//packages/yolo/apps:yolo_detection_subgraph",
        "@yolo_tensorrt_test_data_gtc2019_demo",
    ],
    modules = [
        "flatsim",
        "ml",
        "navigation",
        "planner",
        "sensors:joystick",
        "viewers",
        "yolo",
    ],
)

isaac_app(
    name = "follow_me",
    data = [
        ":kaya_sim_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_control_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "flatsim",
        "imu",
        "navigation",
        "perception:april_tags",
        "perception",
        "planner",
        "sensors:joystick",
        "viewers",
    ],
)
