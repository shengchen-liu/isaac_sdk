"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

cc_library(
    name = "asio",
    srcs = [
        "asio/src/asio.cpp",
        #    "asio/src/asio_ssl.cpp",
    ],
    hdrs = glob([
        "asio/include/asio.hpp",
        "asio/include/asio/*.hpp",
        "asio/include/asio/**/*.hpp",
        "asio/include/asio/**/*.ipp",
    ]),
    defines = [
        "ASIO_STANDALONE",
        "ASIO_SEPARATE_COMPILATION",
    ],
    includes = [
        "asio/include",
    ],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "blocking_tcp_echo_client",
    srcs = ["asio/src/examples/cpp11/echo/blocking_tcp_echo_client.cpp"],
    deps = [":asio"],
)

cc_binary(
    name = "blocking_tcp_echo_server",
    srcs = ["asio/src/examples/cpp11/echo/blocking_tcp_echo_server.cpp"],
    deps = [":asio"],
)
