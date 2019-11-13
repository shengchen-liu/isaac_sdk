"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

# tensorrt is necessary for running inference in the nvstereonet codelet
# Follow the installation steps here
# https://docs.nvidia.com/deeplearning/sdk/tensorrt-install-guide/index.html

cc_library(
    name = "tensorrt_x86_64",
    srcs = [
        "lib/x86_64-linux-gnu/libnvinfer.so",
        "lib/x86_64-linux-gnu/libnvinfer.so.5",
        "lib/x86_64-linux-gnu/libnvinfer.so.5.0.2",
        "lib/x86_64-linux-gnu/libnvinfer_plugin.so",
        "lib/x86_64-linux-gnu/libnvinfer_plugin.so.5",
        "lib/x86_64-linux-gnu/libnvinfer_plugin.so.5.0.2",
        "lib/x86_64-linux-gnu/libnvonnxparser.so",
        "lib/x86_64-linux-gnu/libnvonnxparser.so.0",
        "lib/x86_64-linux-gnu/libnvonnxparser.so.0.1.0",
        "lib/x86_64-linux-gnu/libnvonnxparser_runtime.so",
        "lib/x86_64-linux-gnu/libnvonnxparser_runtime.so.0",
        "lib/x86_64-linux-gnu/libnvonnxparser_runtime.so.0.1.0",
        "lib/x86_64-linux-gnu/libnvparsers.so",
        "lib/x86_64-linux-gnu/libnvparsers.so.5",
        "lib/x86_64-linux-gnu/libnvparsers.so.5.0.2",
    ],
    hdrs = [
        "include/x86_64-linux-gnu/NvInfer.h",
        "include/x86_64-linux-gnu/NvInferPlugin.h",
        "include/x86_64-linux-gnu/NvOnnxConfig.h",
        "include/x86_64-linux-gnu/NvOnnxParser.h",
        "include/x86_64-linux-gnu/NvOnnxParserRuntime.h",
        "include/x86_64-linux-gnu/NvUffParser.h",
    ],
    includes = ["include"],
    strip_include_prefix = "include/x86_64-linux-gnu",
    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac//third_party:cudnn",
    ],
)