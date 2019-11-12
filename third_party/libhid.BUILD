"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

exports_files(["license.txt"])

cc_library(
    name = "libhid",
    srcs = [
        "linux/hid.c",
    ],
    hdrs = [
        "hidapi/hidapi.h",
    ],
    includes = [
        "hidapi",
    ],
    visibility = ["//visibility:public"],
    deps = select({
        "@com_nvidia_isaac//engine/build:platform_x86_64": [
            "@libudev",
        ],
        "@com_nvidia_isaac//engine/build:platform_aarch64": [
            "@libudev_aarch64",
        ],
    }),
)
