/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"
#include "engine/core/math/pose2.hpp"

#include "apps/segway_rmp_lawson/imu/gems/RTIMUSettings.hpp"
#include "apps/segway_rmp_lawson/imu/gems/RTIMU.hpp"
#include "apps/segway_rmp_lawson/imu/gems/RTIMULibDefs.hpp"

namespace isaac
{
namespace drivers
{
class Segway;
}
} // namespace isaac

namespace isaac
{
namespace drivers
{
class RTIMUHal;
}
} // namespace isaac

namespace isaac
{

// A simple C++ codelet that prints periodically
class ImuDriver : public alice::Codelet
{
public:
  // Has whatever needs to be run in the beginning of the program
  void start() override;
  // Has whatever needs to be run repeatedly
  void tick() override;

  // Output IMU data
  ISAAC_PROTO_TX(ImuProto, imu_raw);

  // Isaac will use this device_id to talk to imu
  ISAAC_PARAM(int, device_id, 1);

private:
  void publishIMU_raw(const drivers::RTIMU_DATA &imu_data);

  // Location of the last goal that is transmitted
  Vector2d goal_position_;
  // Timestamp of the last goal that is transmitted
  int64_t goal_timestamp_;

  // Timestamp of the last imu_raw that is transmitted
  int64_t imu_raw_timestamp_;

  std::unique_ptr<drivers::Segway> segway_;
  std::unique_ptr<drivers::RTIMUHal> rtimuhal_;
  std::unique_ptr<drivers::RTIMUSettings> settings_;

  drivers::RTIMUSettings *settings = new drivers::RTIMUSettings("RTIMULib");

  drivers::RTIMU *imu = new drivers::RTIMU(settings);

  int sampleCount = 0;
  int sampleRate = 0;
  uint64_t displayTimer;
  uint64_t now;
  uint64_t rateTimer;
};

} // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ImuDriver);
