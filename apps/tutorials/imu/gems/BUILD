"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_binary", "isaac_cc_library", "isaac_cc_module")

isaac_cc_library(
    name = "segway",
    srcs = [
        "segway.cpp",
    ],
    hdrs = [
        "rmp_types.hpp",
        "segway.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_library(
    name = "RTIMUHal",
    srcs = [
        "RTIMUHal.cpp",
    ],
    hdrs = [
        "RTIMUHal.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_library(
    name = "RTIMUSettings",
    srcs = [
        "RTIMUSettings.cpp",
    ],
    hdrs = [
        "RTIMUSettings.hpp",
        "RTMath.hpp",
        "RTIMUHal.hpp",
        "RTIMU.hpp",
        "RTFusion.hpp",
        "RTIMULibDefs.hpp",
        "RTIMUDefs.hpp",
        "RTIMULSM6DS33LIS3MDL.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_library(
    name = "RTIMU",
    srcs = [
        "RTIMU.cpp",
    ],
    hdrs = [
        "RTIMUSettings.hpp",
        "RTMath.hpp",
        "RTIMUHal.hpp",
        "RTIMU.hpp",
        "RTFusion.hpp",
        "RTIMULibDefs.hpp",
        "RTIMUDefs.hpp",
        "RTFusionKalman4.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_library(
    name = "RTFusion",
    srcs = [
        "RTFusion.cpp",
    ],
    hdrs = [
        "RTFusion.hpp",
        "RTMath.hpp",
        "RTIMUHal.hpp",
        "RTIMULibDefs.hpp",
        "RTIMUDefs.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_library(
    name = "RTMath",
    srcs = [
        "RTMath.cpp",
    ],
    hdrs = [
        "RTMath.hpp",
        "RTIMUHal.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//engine/core",
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)

isaac_cc_module(
    name = "segway",
    srcs = ["segway.cpp"],
    hdrs = [
        "rmp_types.hpp",
        "segway.hpp",
    ],
    deps = [
        "//engine/gems/coms:socket",
        "//engine/gems/math",
        "//engine/gems/serialization:crc16",
        "//engine/gems/sight",
    ],
)
