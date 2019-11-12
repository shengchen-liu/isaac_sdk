"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_test_group")

cc_test(
    name = "object_detection_utils_tests",
    size = "small",
    srcs = ["object_detection_utils.cpp"],
    deps = [
        "//engine/gems/geometry",
        "//engine/gems/ml:object_detection_utils",
        "@gtest//:main",
    ],
)
