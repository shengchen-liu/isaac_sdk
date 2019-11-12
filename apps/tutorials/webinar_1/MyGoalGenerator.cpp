/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "MyGoalGenerator.hpp"
#include "messages/math.hpp"

namespace isaac {
namespace tutorials {

void MyGoalGenerator::start() {
  LOG_INFO("Hello World!");
  tickPeriodically();
}

void MyGoalGenerator::tick() {
  LOG_WARNING("Tick");

  auto goal_proto = tx_my_goal().initProto();
  //
  goal_proto.setStopRobot(false);
  goal_proto.setTolerance(0.1);
  goal_proto.setGoalFrame("world");
  ToProto(Pose2d::Translation(1.1, 2.2), goal_proto.initGoal());
  tx_my_goal().publish();
}

}  // namespace tutorials
}  // namespace isaac
