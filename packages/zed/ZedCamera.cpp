/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ZedCamera.hpp"

#include <algorithm>
#include <utility>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"

namespace isaac {
namespace {

// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 4;

// Helper function to copy camera intrinsics to ColorCameraProto
void SetCameraProtoParameters(const sl::CameraParameters& in, ::ColorCameraProto::Builder& out) {
  // Color space
  out.setColorSpace(ColorCameraProto::ColorSpace::RGB);

  // Pinhole camera model parameters
  auto pinhole = out.initPinhole();
  ToProto(Vector2d(in.fy, in.fx), pinhole.getFocal());
  ToProto(Vector2d(in.cy, in.cx), pinhole.getCenter());
  pinhole.setCols(in.image_size.width);
  pinhole.setRows(in.image_size.height);

  // Distortion parameters
  auto distortion = out.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  // We have all zero distortion coefficients because we are retrieving rectified images
  ToProto(Vector5f::Zero(), distortion.getCoefficients());
}

// Gets a view on a ZED image
ImageConstView1ub GrayZedImageView(const sl::Mat& mat) {
  ASSERT(mat.getStepBytes() == mat.getWidth(), "Not yet supported");
  return CreateImageView<uint8_t, 1>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

// Gets a view on a ZED image
ImageConstView4ub BgraZedImageView(const sl::Mat& mat) {
  return CreateImageView<uint8_t, 4>(mat.getPtr<sl::uchar1>(), mat.getHeight(), mat.getWidth());
}

}  // namespace

void ZedCamera::start() {
  // sanity-check parameters
  zed_initialized_ = false;
  initializeZedCamera();
  tickPeriodically();
}

void ZedCamera::tick() {
  if (!zed_initialized_) {
    initializeZedCamera();
  }

  sl::RuntimeParameters run_params;
  run_params.sensing_mode = sl::SENSING_MODE_STANDARD;
  run_params.enable_depth = false;
  run_params.enable_point_cloud = false;

  if (sl::SUCCESS == zed_->grab(run_params)) {
    // Timestamp in ns
    const int64_t zed_current_time = zed_->getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_CURRENT);
    const int64_t zed_acqtime = zed_->getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);

    publish(this->node()->clock()->timestamp() - zed_current_time + zed_acqtime);
  }
}

void ZedCamera::stop() {
  if (zed_ != nullptr) {
    zed_->close();
    zed_.reset(nullptr);
  }
}

void ZedCamera::initializeZedCamera() {
  sl::InitParameters params = {};
  params.camera_resolution = static_cast<sl::RESOLUTION>(get_resolution());
  auto frequency = getTickPeriodAsFrequency();
  if (!frequency) {
    LOG_ERROR("Invalid frame rate");
    return;
  }
  params.camera_fps = static_cast<int>(frequency.value());
  params.camera_linux_id = get_device_id();
  params.coordinate_units = sl::UNIT_METER;
  params.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
  params.depth_mode = sl::DEPTH_MODE::DEPTH_MODE_NONE;
  params.sdk_verbose = true;
  params.optional_settings_path = sl::String(get_settings_folder_path().c_str());
  params.sdk_gpu_id = get_gpu_id();
  params.sdk_cuda_ctx = isaac::cuda::GetOrCreateCudaContext(get_gpu_id());

  // Create the camera object and open the camera
  zed_.reset(new sl::Camera());
  sl::ERROR_CODE err = zed_->open(params);
  if (err == sl::ERROR_CODE_CALIBRATION_FILE_NOT_AVAILABLE ||
      err == sl::ERROR_CODE_INVALID_CALIBRATION_FILE) {
    zed_->close();
    // Get the device serial number
    const std::vector<sl::DeviceProperties> device_list = zed_->getDeviceList();
    const int selected_device_id = get_device_id();
    const auto selected_device = std::find_if(
        device_list.begin(), device_list.end(),
        [selected_device_id](const auto& device) { return device.id == selected_device_id; });

    ASSERT(selected_device != device_list.end(), "Invalid selected zed camera %d", selected_device);
    // The camera is not calibrated or we couldn't find the calibration file.
    // We cannot proceed without calibration therefore we fail this component.
    reportFailure(
        "Calibration file was not found for your zed camera.\n"
        " Please download the factory calibration or calibrate your camera using the zed"
        " calibration utility.\n"
        " The serial number of your zed camera is %d\n"
        " In order to download factory calibration:\n"
        " 1. call engine/build/scripts/download_zed_calibration.sh -s %d\n"
        " 2. copy the downloaded file, SN%d.conf, to the target device and specify the path to the"
        " containing folder using settings_folder_path setting\n",
        selected_device->serial_number, selected_device->serial_number,
        selected_device->serial_number);
    return;
  } else if (err != sl::SUCCESS) {
    // The camera is unresponsive. We cannot proceed without the camera
    // therefore we fail this component.
    reportFailure("[ZedCamera] Error initializing Zed camera: %s", sl::toString(err).c_str());
    return;
  }

  zed_resolution_ = zed_->getResolution();
  zed_info_ = zed_->getCameraInformation(zed_resolution_);
  LOG_INFO("ZED Camera Serial Number : %d", zed_info_.serial_number);
  LOG_INFO("ZED Camera Firmware version : %d", zed_info_.firmware_version);

  zed_->disableTracking();
  zed_->disableSpatialMapping();

  zed_initialized_ = true;
}

sl::CalibrationParameters ZedCamera::getCameraParameters(bool raw) {
  if (raw) {
    return zed_->getCameraInformation().calibration_parameters_raw;
  } else {
    return zed_->getCameraInformation().calibration_parameters;
  }
}

Pose3d ZedCamera::getCameraExtrinsics() {
  Pose3d out;
  zed_info_ = zed_->getCameraInformation();
  sl::Rotation rot;
  rot.setRotationVector(zed_info_.calibration_parameters.R);
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
  const float* rr = rot.getOrientation().ptr();
  double qq[4] = {rr[0], rr[1], rr[2], rr[3]};
#pragma GCC diagnostic pop
  out.rotation = SO3<double>::FromQuaternion(Quaterniond(qq));
  sl::float3 trs = zed_info_.calibration_parameters.T;
  out.translation = Vector3d(trs[0], trs[1], trs[2]);
  return out;
}

void ZedCamera::publish(int64_t acq_time) {
  const bool gray_scale = get_gray_scale();
  const bool rgb = get_rgb();
  // Retrieve left and right images
  if (gray_scale) {
    zed_->retrieveImage(left_image_gray_, sl::VIEW_LEFT_GRAY);
    zed_->retrieveImage(right_image_gray_, sl::VIEW_RIGHT_GRAY);
  }
  if (rgb) {
    zed_->retrieveImage(left_image_rgb_, sl::VIEW_LEFT);
    zed_->retrieveImage(right_image_rgb_, sl::VIEW_RIGHT);
  }

  // Retrieve camera parameters
  const sl::CalibrationParameters camera_parameters = getCameraParameters();

  if (gray_scale) {
    auto l_camera = tx_left_camera_gray().initProto();
    auto r_camera = tx_right_camera_gray().initProto();
    SetCameraProtoParameters(camera_parameters.left_cam, l_camera);
    SetCameraProtoParameters(camera_parameters.right_cam, r_camera);

    Image1ub buffer_left_gray(zed_resolution_.height, zed_resolution_.width);
    Copy(GrayZedImageView(left_image_gray_), buffer_left_gray);
    show("left_gray_thumbnail",
         [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray)); });
    ToProto(std::move(buffer_left_gray), l_camera.getImage(), tx_left_camera_gray().buffers());

    Image1ub buffer_right_gray(zed_resolution_.height, zed_resolution_.width);
    Copy(GrayZedImageView(right_image_gray_), buffer_right_gray);
    show("right_gray_thumbnail",
         [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray)); });
    ToProto(std::move(buffer_right_gray), r_camera.getImage(), tx_right_camera_gray().buffers());

    tx_left_camera_gray().publish(acq_time);
    tx_right_camera_gray().publish(acq_time);
  }
  if (rgb) {
    auto l_camera = tx_left_camera_rgb().initProto();
    auto r_camera = tx_right_camera_rgb().initProto();
    SetCameraProtoParameters(camera_parameters.left_cam, l_camera);
    SetCameraProtoParameters(camera_parameters.right_cam, r_camera);

    Image3ub buffer_left_rgb(zed_resolution_.height, zed_resolution_.width);
    ConvertBgraToRgb(BgraZedImageView(left_image_rgb_), buffer_left_rgb);
    show("left_rgb_thumbnail",
         [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_rgb)); });
    show("left_rgb", [&](sight::Sop& sop) { sop.add(buffer_left_rgb); });
    ToProto(std::move(buffer_left_rgb), l_camera.getImage(), tx_left_camera_rgb().buffers());

    Image3ub buffer_right_rgb(zed_resolution_.height, zed_resolution_.width);
    ConvertBgraToRgb(BgraZedImageView(right_image_rgb_), buffer_right_rgb);
    show("right_rgb_thumbnail",
         [&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_rgb)); });
    ToProto(std::move(buffer_right_rgb), r_camera.getImage(), tx_right_camera_rgb().buffers());

    tx_left_camera_rgb().publish(acq_time);
    tx_right_camera_rgb().publish(acq_time);
  }

  // Camera extrinsic parameters
  auto ext = tx_extrinsics().initProto();
  ToProto(getCameraExtrinsics(), ext);
  tx_extrinsics().publish(acq_time);

  show("Zed Serial Number", zed_info_.serial_number);
  show("Zed Firmware Version", zed_info_.firmware_version);
}

}  // namespace isaac
