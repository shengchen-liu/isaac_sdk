#!/bin/bash
#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
set +e

WORK_DIR=/tmp/minidump

# Creates working folder, builds tools from breakpad and copies them to working folder
if [ ! -d "$WORK_DIR" ]; then
    mkdir -p "$WORK_DIR"
    bazel build @breakpad//:minidump_stackwalk @breakpad//:dump_syms
    cp bazel-bin/external/breakpad/dump_syms bazel-bin/external/breakpad/minidump_stackwalk $WORK_DIR
fi
