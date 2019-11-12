"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_new_http_archive")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies required for ROS
def isaac_ros_workspace():
    isaac_new_http_archive(
        name = "isaac_ros_bridge_x86_64",
        build_file = clean_dep("//third_party:ros.BUILD"),
        sha256 = "9795b242c2636c2545c2c581037e9a27091e922faec9617784ae5705989f6e17",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros-melodic-x86_64-20190327-tar-gz",
        type = "tar.gz",
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )

    isaac_new_http_archive(
        name = "isaac_ros_bridge_aarch64_xavier",
        build_file = clean_dep("//third_party:ros_xavier.BUILD"),
        sha256 = "5130ba3576687f7a6d85b0f1513024c490e69b64214b7fa50b1c0a072b754233",
        url = "https://developer.nvidia.com/isaac/download/third_party/ros-melodic-aarch64_xavier-20180315-tgz",
        type = "tgz",
        # We only use packages under BSD licenses from this list.
        licenses = ["https://docs.ros.org/diamondback/api/licenses.html"],
    )
