"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module", "isaac_component")

isaac_cc_module(
    name = "components",
    deps = [":dummy_pose2d_codelets"],
)

isaac_component(
    name = "dummy_pose2d_codelets",
    filename = "dummy_pose2d_codelets",
)

py_test(
    name = "pycodelet_test",
    srcs = ["pycodelet_test.py"],
    data = [
        "pycodelet_test_cpp2py.app.json",
        "pycodelet_test_img.app.json",
        "pycodelet_test_py2cpp.app.json",
        "pycodelet_test_py2py.app.json",
        "pycodelet_test_sync.app.json",
        "//engine/pyalice/tests:libcomponents_module.so",
        "//messages:core_messages",
        "//packages/dummies:libdummies_module.so",
    ],
    deps = [
        "//engine/pyalice",
    ],
)
