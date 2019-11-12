#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xecd282d812d9c328;

using import "geometry.capnp".RectangleProto;
using import "math.capnp".Pose3dProto;
using import "math.capnp".Vector2iProto;

# A prediction gives a class name with a confidence level from 0 to 1
struct PredictionProto {
  # Name or label for the detection
  label @0: Text;
  # A general value to indicate how confident we are about the detection.
  # This could for example be provided by a perception algorithm like a
  # neural network bounding box detector.
  confidence @1: Float64;
}

# A message containing detections made by sensor(s)
# Each detection has a bounding box, label, and confidence
struct Detections2Proto {
  # List of predictions made
  predictions @0: List(PredictionProto);
  # List of bounding boxes where we detected objects
  boundingBoxes @1: List(RectangleProto);
}

# A message containing detections made by sensor(s)
# Each detection has a 3D pose, label, and confidence
struct Detections3Proto {
  # List of predictions made
  predictions @0: List(PredictionProto);
  # List of poses of objects detected relative to the sensor frame
  poses @1: List(Pose3dProto);
}
