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

namespace isaac {

// A simple C++ codelet that prints periodically
class ImuDriver : public alice::Codelet {
  public:
    ImuDriver();
    ~ImuDriver();
    // Has whatever needs to be run in the beginning of the program
    void start() override;
    // Has whatever needs to be run repeatedly
    void tick() override;


    // Output goal for the robot
    ISAAC_PROTO_TX(Goal2Proto, goal);
    // Feedback about imu
    ISAAC_PROTO_RX(ImuProto, feedback);

    // Desired x and y position of the robot on map in meters
    ISAAC_PARAM(Vector2d, desired_position, Vector2d(9.0, 25.0));

    // Message to be printed at every tick
    ISAAC_PARAM(std::string, message, "Hello World!");
  private:
  // Publishes a goal message with given target position.
    void publishGoal(const Vector2d& position);

    // Location of the last goal that is transmitted
    Vector2d goal_position_;
    // Timestamp of the last goal that is transmitted
    int64_t goal_timestamp_;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::ImuDriver);
