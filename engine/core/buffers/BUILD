"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "algorithm",
    srcs = select({
        "//conditions:default": ["algorithm_cuda.cpp"],
        "//engine/build:isaac_gpu_disabled": ["algorithm_cpu.cpp"],
    }),
    hdrs = [
        "algorithm.hpp",
    ],
    deps = [
        ":utils",
        "//engine/core",
    ] + select({
        "//conditions:default": ["//third_party:cudart"],
        "//engine/build:isaac_gpu_disabled": [],
    }),
)

isaac_cc_library(
    name = "buffers",
    hdrs = ["//engine/core:buffers.hpp"],
    visibility = ["//visibility:public"],
    deps = [
        ":algorithm",
        ":cpu",
        ":cuda",
        ":shared_buffer",
        ":stride_buffer",
        ":utils",
    ],
)

isaac_cc_library(
    name = "cpu",
    hdrs = [
        "cpu_array.hpp",
        "cpu_array_view.hpp",
    ],
    deps = [
        ":stride_buffer",
        ":utils",
        "//engine/core",
    ],
)

isaac_cc_library(
    name = "stride_buffer",
    srcs = [
        "aligned_buffer.cpp",
    ],
    hdrs = [
        "aligned_buffer.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":utils",
        "//engine/core",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "shared_buffer",
    srcs = [
        "shared_buffer.cpp",
    ],
    hdrs = [
        "shared_buffer.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":algorithm",
        ":stride_buffer",
        ":utils",
        "//engine/core",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "cuda",
    srcs = [
        "cuda_array.cpp",
    ],
    hdrs = [
        "cuda_array.hpp",
        "cuda_array_view.hpp",
    ],
    deps = [
        ":cpu",
        ":utils",
        "//engine/core",
        "//third_party:cudart",
    ],
)

isaac_cc_library(
    name = "utils",
    hdrs = [
        "traits.hpp",
    ],
)
