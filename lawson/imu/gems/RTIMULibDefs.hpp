#pragma once

#include "RTMath.hpp"
#include "RTIMUDefs.hpp"
#include <stdint.h>

//  these defines describe the various fusion filter options

#define RTFUSION_TYPE_NULL 0         // just a dummy to keep things happy if not needed
#define RTFUSION_TYPE_KALMANSTATE4 1 // kalman state is the quaternion pose
#define RTFUSION_TYPE_RTQF 2         // RT quaternion fusion
#define RTFUSION_TYPE_COUNT 3        // number of fusion algorithm types

//  This is a convenience structure that can be used to pass IMU data around
namespace isaac
{
namespace drivers
{
typedef struct
{
    uint64_t timestamp;
    bool fusionPoseValid;
    RTVector3 fusionPose;
    bool fusionQPoseValid;
    RTQuaternion fusionQPose;
    bool gyroValid;
    RTVector3 gyro;
    bool accelValid;
    RTVector3 accel;
    bool compassValid;
    RTVector3 compass;
    bool pressureValid;
    RTFLOAT pressure;
    bool temperatureValid;
    RTFLOAT temperature;
    bool humidityValid;
    RTFLOAT humidity;
} RTIMU_DATA;

} // namespace drivers
} // namespace isaac
