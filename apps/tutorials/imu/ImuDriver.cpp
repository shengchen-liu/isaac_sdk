/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "ImuDriver.hpp"

#include "messages/math.hpp"

#include <cmath>
#include "engine/alice/components/Failsafe.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/sight/sight.hpp"
#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"
#include "apps/tutorials/imu/gems/segway.hpp"
#include "apps/tutorials/imu/gems/RTIMUHal.hpp"
// #include "apps/tutorials/imu/gems/RTIMUSettings.hpp"
// #include "apps/tutorials/imu/gems/RTIMU.hpp"
#include "apps/tutorials/imu/gems/RTMath.hpp"
#include "apps/tutorials/imu/gems/RTFusion.hpp"
#include "apps/tutorials/imu/gems/RTFusionKalman4.hpp"

namespace isaac
{

using drivers::RTFusion;
using drivers::RTIMU;
using drivers::RTIMU_DATA;
using drivers::RTIMUSettings;
using drivers::RTMath;
using drivers::RTVector3;

// Goal tolerance in meters
constexpr double kGoalTolerance = 0.1;

void ImuDriver::start()
{
  // This part will be run once in the beginning of the program

  // We can tick periodically, on every message, or blocking. The tick period is set in the
  // json ping.app.json file. You can for example change the value there to change the tick
  // frequency of the node.
  // Alternatively you can also overwrite configuration with an existing configuration file like
  // in the example file fast_ping.json. Run the application like this to use an additional config
  // file:
  //   bazel run //apps/tutorials/ping -- --config apps/tutorials/ping/fast_ping.json

  // Read device_id parameter
  const int device_id = get_device_id();
  std::cout << device_id << std::endl;

  goal_timestamp_ = 0;
  goal_position_ = Vector2d::Zero();
  tickBlocking();

  std::cout << imu->IMUType() << std::endl;

  if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
  {
    printf("No IMU found\n");
    exit(1);
  }

  //  This is an opportunity to manually override any settings before the call IMUInit

  //  set up IMU

  imu->IMUInit();

  //  this is a convenient place to change fusion parameters

  imu->setSlerpPower(0.02);
  imu->setGyroEnable(true);
  imu->setAccelEnable(true);
  imu->setCompassEnable(false);

  // //  set up for rate timer

  // rateTimer = displayTimer = RTMath::currentUSecsSinceEpoch();
}

void ImuDriver::publishGoal(const Vector2d &position)
{
  // Save the timestamp to later check it against the feedback timestamp
  goal_timestamp_ = node()->clock()->timestamp();
  // Update the last goal information to avoid transmitting repeated messages
  goal_position_ = position;
  // Show the new goal on WebSight
  show("goal_timestamp", goal_timestamp_);
  show("goal_position_x", goal_position_.x());
  show("goal_position_y", goal_position_.y());
  // Publish a goal with the new goal location
  auto goal_proto = tx_goal().initProto();
  goal_proto.setStopRobot(false);
  ToProto(Pose2d::Translation(position), goal_proto.initGoal());
  // Use a goal tolerance of kGoalTolerance meters
  goal_proto.setTolerance(kGoalTolerance);
  // The goal we publish is with respect to the global "world" coordinate frame
  goal_proto.setGoalFrame("world");
  tx_goal().publish(goal_timestamp_);
}

void ImuDriver::publishIMU_raw(const RTIMU_DATA &imu_data)
{
  // Save the timestamp to later check it against the feedback timestamp
  imu_raw_timestamp_ = node()->clock()->timestamp();

  float a_x = imu_data.gyro.x();
  float a_y = imu_data.gyro.y();
  float a_z = imu_data.gyro.z();

  float v_x = imu_data.accel.x();
  float v_y = imu_data.accel.y();
  float v_z = imu_data.accel.z();

  float roll = imu_data.fusionPose.x() * RTMATH_RAD_TO_DEGREE;
  float pitch = imu_data.fusionPose.y() * RTMATH_RAD_TO_DEGREE;
  float yaw = imu_data.fusionPose.z() * RTMATH_RAD_TO_DEGREE;

  // Publish a imu_data
  auto imu_proto = tx_imu_raw().initProto();
  // assign values
  imu_proto.setLinearAccelerationX(a_x);
  imu_proto.setLinearAccelerationY(a_y);
  imu_proto.setLinearAccelerationZ(a_z);

  imu_proto.setAngularVelocityX(v_x);
  imu_proto.setAngularVelocityY(v_y);
  imu_proto.setAngularVelocityZ(v_z);

  imu_proto.setAngleRoll(roll);
  imu_proto.setAnglePitch(pitch);
  imu_proto.setAngleYaw(yaw);

  tx_imu_raw().publish(imu_raw_timestamp_);

  // Show the new goal on WebSight
  show("imu_raw_timestamp_", imu_raw_timestamp_);
  show("LinearAccelerationX", a_x);
  show("LinearAccelerationY", a_y);
  show("LinearAccelerationZ", a_z);

  show("AngularVelocityX", v_x);
  show("AngularVelocityY", v_y);
  show("AngularVelocityZ", v_z);

  show("Roll", roll);
  show("Pitch", pitch);
  show("Yaw", yaw);

  // std::cout<<roll<<std::endl;
}

void ImuDriver::tick()
{
  // This part will be run at every tick. We are ticking periodically in this example.
  // usleep(imu->IMUGetPollInterval() * 1000); // 4000 micro second = 4 ms
  if (imu->IMURead())
  {
    RTIMU_DATA imuData = imu->getIMUData();
    publishIMU_raw(imuData);

    sampleCount++;

    now = RTMath::currentUSecsSinceEpoch();
    //  display 10 times per second

    if ((now - displayTimer) > 100000)
    {
      printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
      fflush(stdout);
      displayTimer = now;
    }
    //  update rate every second

    if ((now - rateTimer) > 1000000)
    {
      sampleRate = sampleCount;
      sampleCount = 0;
      rateTimer = now;
    }
  }
  // if (imu->IMURead()){
  //   std::cout<<"true"<<std::endl;
  // }
  // else
  // {
  //   std::cout<<"false"<<std::endl;
  // }

  // while (imu->IMURead())
  // {
  //   RTIMU_DATA imuData = imu->getIMUData();
  //   sampleCount++;

  //   now = RTMath::currentUSecsSinceEpoch();

  //   //  display 10 times per second

  //   if ((now - displayTimer) > 100000)
  //   {
  //     printf("Sample rate %d: %s\r", sampleRate, RTMath::displayDegrees("", imuData.fusionPose));
  //     fflush(stdout);
  //     displayTimer = now;
  //   }
  //   //  update rate every second

  //   if ((now - rateTimer) > 1000000)
  //   {
  //     sampleRate = sampleCount;
  //     sampleCount = 0;
  //     rateTimer = now;
  //   }
  // }

  // // This part will be run at every tick. We are ticking periodically in this example.
  // // Read desired position parameter <- ISAAC_PARAM(Vector2d, desired_position, Vector2d(9.0, 25.0));
  // const Vector2d position = get_desired_position();
  // // Publish goal, if there has been a location change
  // if (isFirstTick() || (position - goal_position_).norm() > kGoalTolerance)
  // {
  //   publishGoal(position);
  // }

  // // auto state = segway_->getSegwayState();
  // // std::cout<<state.linear_accel_msp2<<std::endl;

  // // Process feedback
  // rx_feedback().processLatestNewMessage(
  //     [this](auto feedback_proto, int64_t pubtime, int64_t acqtime) {
  //       // Check if this feedback is associated with the last goal we transmitted
  //       if (goal_timestamp_ != acqtime)
  //       {
  //         return;
  //       }
  //       // const float A_x = feedback_proto.getLinearAccelerationX();
  //       // std::cout<<A_x<<std::endl;
  //       // // Show information on WebSight
  //       // show("arrived", arrived ? 1.0 : 0.0);
  //     });
  // // // Print the desired message to the console <- ISAAC_PARAM(std::string, message, "Hello World!");
  // // LOG_INFO(get_message().c_str());
}

} // namespace isaac
