"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_binary", "isaac_cc_library", "isaac_cc_test_group")
load("//engine/build:cc_capnp_library.bzl", "cc_capnp_library")

cc_capnp_library(
    name = "message_protos",
    protos = ["messages.capnp"],
)

cc_capnp_library(
    name = "foo_bar_protos",
    protos = [
        "bar.capnp",
        "foo.capnp",
    ],
)

isaac_cc_library(
    name = "messages",
    srcs = [
        "bulky_transmission.cpp",
        "foo_transmission.cpp",
    ],
    hdrs = [
        "bulky_transmission.hpp",
        "foo_transmission.hpp",
        "messages.hpp",
    ],
    deps = [
        ":message_protos",
        "//engine/alice",
        "@gtest//:main",
    ],
)

isaac_cc_test_group(
    srcs = glob(
        ["*.cpp"],
        exclude = [
            "crashdump.cpp",
            "bulky_transmission.*",
            "foo_transmission.*",
            "image_transmission.*",
            # Tests which have extra dependencies
            "message_images.cpp",
            "modules.cpp",
            "network_images_tcp.cpp",
            "network_images_udp.cpp",
            "record_replay_images.cpp",
            "subgraphs.cpp",
            # Temporarily disabled tests
            "massively_ticking.cpp",
            "network_ping_pong.cpp",
            "ping_pong.cpp",
        ],
    ),
    deps = [
        ":foo_bar_protos",
        ":messages",
        "//engine/alice",
    ],
)

cc_test(
    name = "subgraphs",
    srcs = ["subgraphs.cpp"],
    data = glob([
        "subgraphs*.app.json",
        "transmission*.group.json",
    ]),
    deps = [
        ":messages",
        "//engine/alice",
        "@gtest//:main",
    ],
)

cc_test(
    name = "modules",
    srcs = ["modules.cpp"],
    data = [
        "//packages/dummies:libdummies_module.so",
        "//packages/flatsim:libflatsim_module.so",
    ],
    deps = [
        "//engine/alice",
        "@gtest//:main",
    ],
)

isaac_cc_library(
    name = "image_transmission",
    srcs = [
        "image_transmission.cpp",
    ],
    hdrs = [
        "image_transmission.hpp",
    ],
    deps = [
        "//engine/alice",
        "//messages:core_messages",
        "@gtest//:main",
    ],
)

isaac_cc_test_group(
    srcs = [
        "message_images.cpp",
        "network_images_tcp.cpp",
        # "network_images_udp.cpp",  // UDP-based message passing not yet supported for images
        "record_replay_images.cpp",
    ],
    deps = [
        ":image_transmission",
        "//engine/alice",
    ],
)

isaac_cc_binary(
    name = "crashdump_binary",
    srcs = [
        "crashdump.cpp",
    ],
    deps = [
        "//engine/alice",
        "//engine/alice/tools:parse_command_line",
        "//external:gflags",
        "//messages",
    ],
)

sh_test(
    name = "crashdump_sh",
    size = "small",
    srcs = ["crashdump.sh"],
    data = [
        ":crashdump_binary",
        "@breakpad//:dump_syms",
        "@breakpad//:minidump_stackwalk",
    ],
    deps = [
    ],
)
