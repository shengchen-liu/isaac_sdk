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
    name = "carter_navsim_subgraph",
    modules = [
        "imu",
    ],
    subgraph = "carter.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim_viewer_tcp",
    data = [
        "navsim_subgraph",
    ],
    modules = [
        "ml",
        "viewers",
    ],
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "minisight_subgraph",
    modules = [
        "utils",
    ],
    subgraph = "minisight.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_subgraph(
    name = "navsim_subgraph",
    modules = [
        "navsim",
        "ml",
    ],
    subgraph = "navsim.subgraph.json",
    visibility = ["//visibility:public"],
)

isaac_app(
    name = "navsim",
    data = [
        "//packages/c_api:libisaac_c_api.so",
        "//packages/navsim/scenarios",
    ],
    modules = [
        "imu",
        "map",
    ],
    visibility = ["//visibility:public"],
)
