"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:pybind.bzl", "pybind_library")

py_binary(
    name = "training",
    srcs = ["training.py"],
    data = glob(["*.json"]) + [
        "//packages/ml:libml_module.so",
        "//packages/navsim/apps:navsim_subgraph",
        "//packages/viewers:libviewers_module.so",
    ],
    deps = [
        "//engine/pyalice",
        "//packages/ml:pyml"
    ],
)
