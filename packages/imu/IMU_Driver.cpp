/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "IMU_Driver.hpp"

#include <cmath>

#include "engine/core/constants.hpp"
#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/messages.hpp"
#include "packages/imu/gems/LSM6DS33.hpp"

namespace isaac {

namespace {

constexpr double kMaxSafeSpeedLimit = 2.0;
constexpr double kMaxSafeTurningRateLimit = 2.0;

}  // namespace

IMU_Driver::IMU_Driver() {}

IMU_Driver::~IMU_Driver() {}

void IMU_Driver::start() {
  tickBlocking();
  imu_.reset(new drivers::LSM6DS33());
  imu_->start();
  // beep
  // hexadecimal
  //  What is 0xDD? 
  LOG_INFO("Start LSM6DS33............");
}

void IMU_Driver::tick() {
  auto state = imu_->getImuState();
  LOG_INFO("tick");

  ::capnp::MallocMessageBuilder message;
  auto imu_list = message.getRoot<ImuProto>();
  auto A_x = imu_list.getLinearAccelerationX();
  // LOG_INFO("A_x: %s", A_x.f_str());
  std::cout<<A_x<<std::endl;
  
  // // publish current state of segway
  // ::capnp::List<ImuProto>::Builder people = ImuProto::Builder.i

  messages::ImuProto imu_state;
  imu_state.linear_speed() = A_x;
  
  // // publish to edges
  ToProto(imu_state, tx_segway_state().initProto(), tx_segway_state().buffers());

  // tx_segway_state().publish();

  // // visualization
  // show("linearAccelerationX", state.linearAccelerationX);
}

void IMU_Driver::stop() {
  // 31: RMP_CMD_SET_AUDIO_COMMAND
  // 2: MOTOR_AUDIO_PLAY_POWER_OFF_SONG
  // 32: RMP_CMD_SET_OPERATIONAL_MODE
  // 4: STANDBY_REQUEST 
  imu_->stop();
}

}  // namespace isaac
