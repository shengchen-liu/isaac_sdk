#####################################################################################
# Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#####################################################################################
@0xad79d0e15fcdd8a7;

using import "math.capnp".Vector3fProto;

# A message about laser range scans published for example from a LIDAR sensor
# The message does not have to capture a full 360 degree scan, as most sensors publish partial
# slices at high frequency instead of waiting to complete a full scan.
struct RangeScanProto {
  struct Ray {
    # Normalized distance to target (see rangeScale)
    range @0: UInt16;
    # Normalized ray return intensity (see intensityScale)
    intensity @1: UInt8;
  }
  # The range and intensity of the scanned rays. For each angle theta there is a ray
  # for every phi angle. So the total number of rays is length(theta) * length(size).
  rays @0: List(Ray);
  # table of theta (horizontal) angles
  theta @1: List(Float32);
  # table of phi (vertical) angles
  phi @2: List(Float32);
  # Scale factor which can be used to convert a range value from a 16-bit integer to meters. The
  # conversion formula is: range[meters] = range[normalized] / 0xFFFF * rangeScale
  rangeDenormalizer @3: Float32;
  # Scale factor which can be used to convert an intensity value from an 8-bit integer to meters.
  # The conversion formula is: intensity = intensity[normalized] / 0xFF * rangeScale
  intensityDenormalizer @4: Float32;
  # Delay in microseconds between firing
  deltaTime @5: UInt16;
  # Beams with a range smaller than or equal to this distance (in meters) are considered to have
  # returned an invalid measurement.
  invalidRangeThreshold @6: Float64;
  # Beams with a range larger than or equal to this distance (in meters) are considered to not have
  # hit an obstacle within the maximum possible range of the sensor.
  outOfRangeThreshold @7: Float64;
}

# A 2D range scan which is essentially a flat version of the 3D RangeScanProto
struct FlatscanProto {
  # Angles (in radians) under which rays are shot
  angles @0: List(Float32);
  # Return distance of the ray
  ranges @1: List(Float32);
  # Beams with a range smaller than or equal to this distance are considered to have returned an
  # invalid measurement.
  invalidRangeThreshold @2: Float64;
  # Beams with a range larger than or equal to this distance are considered to not have hit an
  # obstacle within the maximum possible range of the sensor.
  outOfRangeThreshold @3: Float64;
}
