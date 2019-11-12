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
    name = "joystick",
    data = [
        "2d_kaya_subgraph",
        "kaya_hardware_subgraph",
        "//apps/kaya/robots",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
    ],
)

isaac_app(
    name = "navigate",
    data = [
        "2d_kaya_subgraph",
        "kaya_hardware_subgraph",
        "//apps/assets/maps",
        "//apps/kaya/robots",
        "//packages/navigation/apps:goal_generators_subgraph",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_navigation_subgraph",
    ],
    modules = [
        "map",
        "navigation",
        "perception",
        "planner",
        "viewers",
    ],
)

isaac_app(
    name = "waste_sorting",
    data = [
        "2d_kaya_subgraph",
        "behavior_waste_sorting",
        "kaya_hardware_subgraph",
        "//apps/assets/maps",
        "//apps/kaya/robots",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "//packages/navigation/apps:local_map_subgraph",
        "//packages/navigation/apps:localization_subgraph",
        "//packages/yolo/apps:yolo_detection_subgraph",
        "@yolo_tensorrt_test_data_gtc2019_demo",
    ],
    modules = [
        "//packages/kaya_misc:kaya",
        "map",
        "ml",
        "navigation",
        "perception:april_tags",
        "perception",
        "planner",
        "utils",
        "viewers",
        "yolo",
    ],
)

isaac_app(
    name = "object_detection",
    data = [
        "2d_kaya_subgraph",
        "kaya_hardware_subgraph",
        "//apps/kaya/robots",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "//packages/yolo/apps:yolo_detection_subgraph",
        "@yolo_tensorrt_test_data_gtc2019_demo",
    ],
    modules = [
        "ml",
        "navigation",
        "viewers",
        "yolo",
    ],
)

isaac_app(
    name = "follow_me",
    data = [
        "2d_kaya_subgraph",
        "kaya_hardware_subgraph",
        "//apps/kaya/robots",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_control_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
    ],
    modules = [
        "navigation",
        "perception:april_tags",
        "perception",
        "planner",
        "viewers",
    ],
)

isaac_app(
    name = "gmapping_distributed_host",
    modules = [
        "navigation",
    ],
)

isaac_app(
    name = "gmapping_distributed_kaya",
    data = [
        "2d_kaya_subgraph",
        "kaya_hardware_subgraph",
        "//apps/kaya/robots",
        "//packages/navigation/apps:holonomic_base_commander_subgraph",
        "//packages/navigation/apps:holonomic_base_imu_odometry_subgraph",
        "//packages/navigation/apps:local_map_subgraph",
    ],
    modules = [
        "navigation",
        "perception",
        "planner",
        "viewers",
    ],
)

isaac_subgraph(
    name = "behavior_waste_sorting",
    modules = [
        "//packages/kaya_misc:kaya",
        "navigation",
        "planner",
        "utils",
    ],
    subgraph = "behavior_waste_sorting.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "kaya_hardware_subgraph",
    modules = [
        "imu",
        "kaya_base",
        "realsense",
        "viewers",
    ],
    subgraph = "kaya_hardware.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "2d_kaya_subgraph",
    modules = [
        "rgbd_processing",
    ],
    subgraph = "2d_kaya.subgraph.json",
    visibility = ["//visibility:public"],
)
