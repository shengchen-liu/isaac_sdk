/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/gems/coms/socket.hpp"
#include "rmp_types.hpp"

namespace isaac {
namespace drivers {

// The state of the Segway
struct LSM6DS33State {
  // The linear acceleration of the body frame along the primary axis
  float linearAccelerationX;
  float linearAccelerationY;
  float linearAccelerationZ;
  // The angular velocity of the body frame around the primary axes 
  float angularVelocityX;
  float angularVelocityY;
  float angularVelocityZ;
  // Optional angles as integrated by a potential internal estimator 
  float angleYaw;
  float anglePitch;
  float angleRoll;
};

// Segway RMP class that interface with the robot base over Ethernet
class LSM6DS33 {
 public:
  // Instantiates a LSM6DS33
  LSM6DS33();

  // Starts listening
  void start();

  // Stops listening
  void stop();

  // Sends a speed command to the segway (linear speed and angular speed)
  bool sendSpeedCommand(float linear_speed, float angular_speed);

  // Sends a Float to the robot
  bool sendCommandF(uint16_t command_id, float value1, float value2);

  // Sends a Unsigned int to the robot
  bool sendCommandU(uint16_t command_id, uint32_t value1, uint32_t value2);

  // Sends a command to the Robor
  bool sendCommandPacketToRmp(const rmp_set_packet_t& packet);

  // Configures which responses we would like to receive
  void configureResponses();

  // Returns current IMU state
  const LSM6DS33State& getImuState() {
    return LSM6DS33_state_;
  }

 private:
  std::unique_ptr<Socket> socket_;
  std::map<std::string, uint32_t> rmp_data_;
  std::vector<std::vector<std::tuple<std::string, int>>> answers_per_frame_;
  LSM6DS33State LSM6DS33_state_;
};

}  // namespace drivers
}  // namespace isaac
