/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "engine/gems/geometry/pinhole.hpp"
#include "messages/messages.hpp"
#include "sl/Camera.hpp"

namespace sl {

NLOHMANN_JSON_SERIALIZE_ENUM(RESOLUTION, {
    {RESOLUTION_HD2K, "2208x1242"},
    {RESOLUTION_HD1080, "1920x1080"},
    {RESOLUTION_HD720, "1280x720"},
    {RESOLUTION_VGA, "672x376"},
    {RESOLUTION_LAST, nullptr},
});

}  // namespace sl

namespace isaac {

// Provides stereo image pairs and calibration information from a ZED camera
class ZedCamera : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // left rgb image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, left_camera_rgb);
  // right rgb image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, right_camera_rgb);
  // left gray image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, left_camera_gray);
  // right gray rgb image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, right_camera_gray);
  // camera pair extrinsics (right-to-left)
  ISAAC_PROTO_TX(Pose3dProto, extrinsics);

  // The resolution to use for the ZED camera. The following values can be set:
  //  RESOLUTION_HD2K:   2208x1242
  //  RESOLUTION_HD1080: 1920x1080
  //  RESOLUTION_HD720:  1280x720
  //  RESOLUTION_VGA:    672x376
  ISAAC_PARAM(sl::RESOLUTION, resolution, sl::RESOLUTION_VGA);
  // The numeral of the system video device of the ZED camera. For example for /dev/video0 choose 0.
  ISAAC_PARAM(int, device_id, 0);
  // Turns on gray scale images
  ISAAC_PARAM(bool, gray_scale, false);
  // Turns on RGB color images
  ISAAC_PARAM(bool, rgb, true);
  // The folder path to the settings file (SN#####.conf) for the zed camera.
  // This file contains the calibration parameters for the camera.
  ISAAC_PARAM(std::string, settings_folder_path, "./");
  // The GPU device to be used for ZED CUDA operations
  ISAAC_PARAM(int, gpu_id, 0);

 private:
  // Initialize the ZED camera. Called once at startup
  void initializeZedCamera();
  // Retrieve the camera calibration parameters from the ZED firmware
  // 'raw' specifies if we want the parameters for unrectified images (true)
  // or rectified images (false)
  sl::CalibrationParameters getCameraParameters(bool raw = false);
  // Retrieve the camera extrinsics
  Pose3d getCameraExtrinsics();
  // Publish the stereo data (images, camera intrinsics and extrinsics)
  void publish(int64_t time);

  // ZED camera data
  bool zed_initialized_;
  std::unique_ptr<sl::Camera> zed_;
  sl::Resolution zed_resolution_;
  sl::Mat left_image_rgb_;
  sl::Mat right_image_rgb_;
  sl::Mat left_image_gray_;
  sl::Mat right_image_gray_;
  sl::CalibrationParameters camera_parameters_;
  sl::CameraInformation zed_info_ = {};
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ZedCamera);
