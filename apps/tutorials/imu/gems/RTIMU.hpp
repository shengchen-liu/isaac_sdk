#pragma once

#include "RTMath.hpp"
#include "RTFusion.hpp"
#include "RTIMUSettings.hpp"
#include "RTIMULibDefs.hpp"

namespace isaac
{
namespace drivers
{
class RTIMU
{
public:
    //  Constructor/destructor
    RTIMU(RTIMUSettings *settings);

    const char *IMUName() { return "LSM6DS33 + LIS3MDL"; } // the name of the IMU
    int IMUType() { return RTIMU_TYPE_LSM6DS33LIS3MDL; }   // the type code of the IMU
    bool IMUInit();                                        // set up the IMU
    int IMUGetPollInterval();                              // returns the recommended poll interval in mS
    bool IMURead();                                        // get a sample

    //  the following function can be called to set the SLERP power

    void setSlerpPower(RTFLOAT power) { m_fusion->setSlerpPower(power); }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_fusion->setGyroEnable(enable); }
    void setAccelEnable(bool enable) { m_fusion->setAccelEnable(enable); }
    void setCompassEnable(bool enable) { m_fusion->setCompassEnable(enable); }

    //  setCalibrationData configures the cal data from settings and also enables use if valid

    void setCalibrationData();

protected:
    void gyroBiasInit(); // sets up gyro bias calculation

    bool m_compassCalibrationMode; // true if cal mode so don't use cal data!
    bool m_accelCalibrationMode;   // true if cal mode so don't use cal data!

    RTIMU_DATA m_imuData; // the data from the IMU

    RTIMUSettings *m_settings; // the settings object pointer

    RTFusion *m_fusion; // the fusion algorithm

    int m_sampleRate = 100;    // samples per second
    uint64_t m_sampleInterval; // interval between samples in microseonds

    RTFLOAT m_gyroLearningAlpha;   // gyro bias rapid learning rate
    RTFLOAT m_gyroContinuousAlpha; // gyro bias continuous (slow) learning rate
    int m_gyroSampleCount;         // number of gyro samples used

    float m_compassCalOffset[3];
    float m_compassCalScale[3];

    bool m_runtimeMagCalValid;   // true if the runtime mag calibration has valid data
    float m_runtimeMagCalMax[3]; // runtime max mag values seen
    float m_runtimeMagCalMin[3]; // runtime min mag values seen

private:
    unsigned char m_gyroAccelSlaveAddr; // I2C address of LSM6DS33
    bool setGyro();
    bool setAccel();

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

} // namespace drivers
} // namespace isaac