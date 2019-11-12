"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_library")
load("//engine/build:cc_capnp_library.bzl", "cc_capnp_library")

filegroup(
    name = "capnp",
    srcs = glob(["*.capnp"]),
    visibility = ["//visibility:public"],
)

cc_capnp_library(
    name = "core_protos",
    protos = [
        "element_type.capnp",
        "image.capnp",
        "json.capnp",
        "math.capnp",
        "ping.capnp",
        "sample_cloud.capnp",
        "tensor.capnp",
        "uuid.capnp",
    ],
)

cc_capnp_library(
    name = "state_proto",
    protos = ["state.capnp"],
    deps = [":core_protos"],
)

cc_capnp_library(
    name = "protos",
    protos = [
        "actuators.capnp",
        "audio.capnp",
        "audio_file_playback.capnp",
        "bodies.capnp",
        "camera.capnp",
        "chat_message.capnp",
        "collision.capnp",
        "detections.capnp",
        "dynamixel.capnp",
        "fiducials.capnp",
        "geometry.capnp",
        "heatmap.capnp",
        "imu.capnp",
        "joystick.capnp",
        "led_strip.capnp",
        "lidar.capnp",
        "ml.capnp",
        "mocap.capnp",
        "navigation.capnp",
        "obstacles.capnp",
        "point_cloud.capnp",
        "posetree.capnp",
        "pwm_controller.capnp",
        "robot_state.capnp",
        "superpixels.capnp",
        "voice_command_detection.capnp",
    ],
    deps = [
        ":core_protos",
        ":state_proto",
    ],
)

isaac_cc_library(
    name = "core_messages",
    hdrs = [
        "element_type.hpp",
        "image.hpp",
        "json.hpp",
        "math.hpp",
        "sample_cloud.hpp",
        "tensor.hpp",
        "uuid.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core_protos",
        "//engine/core/buffers",
        "//engine/gems/image",
        "//engine/gems/math",
        "//engine/gems/serialization",
        "//engine/gems/uuid",
    ],
)

isaac_cc_library(
    name = "state_messages",
    visibility = ["//visibility:public"],
    deps = [
        ":core_messages",
        ":state_proto",
    ],
)

isaac_cc_library(
    name = "messages",
    hdrs = [
        "camera.hpp",
        "geometry.hpp",
        "messages.hpp",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":core_messages",
        ":protos",
        ":state_messages",
        "//engine/gems/geometry",
    ],
)

cc_capnp_library(
    name = "alice_protos",
    protos = ["alice.capnp"],
    deps = [":core_protos"],
)

isaac_cc_library(
    name = "alice",
    visibility = ["//visibility:public"],
    deps = [
        ":alice_protos",
        ":core_messages",
    ],
)
