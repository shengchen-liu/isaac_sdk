#pragma once

#include "RTMath.hpp"
#include "RTFusion.hpp"
#include "RTIMUSettings.hpp"
#include "RTIMULibDefs.hpp"

#define RTIMU_XNORTH_YEAST 0 // this is the default identity matrix
#define RTIMU_XEAST_YSOUTH 1
#define RTIMU_XSOUTH_YWEST 2
#define RTIMU_XWEST_YNORTH 3
#define RTIMU_XNORTH_YWEST 4
#define RTIMU_XEAST_YNORTH 5
#define RTIMU_XSOUTH_YEAST 6
#define RTIMU_XWEST_YSOUTH 7
#define RTIMU_XUP_YNORTH 8
#define RTIMU_XUP_YEAST 9
#define RTIMU_XUP_YSOUTH 10
#define RTIMU_XUP_YWEST 11
#define RTIMU_XDOWN_YNORTH 12
#define RTIMU_XDOWN_YEAST 13
#define RTIMU_XDOWN_YSOUTH 14
#define RTIMU_XDOWN_YWEST 15
#define RTIMU_XNORTH_YUP 16
#define RTIMU_XEAST_YUP 17
#define RTIMU_XSOUTH_YUP 18
#define RTIMU_XWEST_YUP 19
#define RTIMU_XNORTH_YDOWN 20
#define RTIMU_XEAST_YDOWN 21
#define RTIMU_XSOUTH_YDOWN 22
#define RTIMU_XWEST_YDOWN 23

#define RTIMU_AXIS_ROTATION_COUNT 24

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

    //  getIMUData returns the standard outputs of the IMU and fusion filter

    const RTIMU_DATA &getIMUData() { return m_imuData; }

    //  setCalibrationData configures the cal data from settings and also enables use if valid

    void setCalibrationData();

    //  getCompassCalibrationValid() returns true if the compass min/max calibration data is being used

    bool getCompassCalibrationValid() { return !m_compassCalibrationMode && m_settings->m_compassCalValid; }

    //  getRuntimeCompassCalibrationValid() returns true if the runtime compass min/max calibration data is being used

    bool getRuntimeCompassCalibrationValid() { return !m_compassCalibrationMode && m_runtimeMagCalValid; }

    //  getAccelCalibrationValid() returns true if the accel calibration data is being used

    bool getAccelCalibrationValid() { return !m_accelCalibrationMode && m_settings->m_accelCalValid; }

protected:
    void gyroBiasInit();            // sets up gyro bias calculation
    void handleGyroBias();          // adjust gyro for bias
    void calibrateAverageCompass(); // calibrate and smooth compass
    void calibrateAccel();          // calibrate the accelerometers
    void updateFusion();            // call when new data to update fusion state

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

    RTVector3 m_previousAccel; // previous step accel for gyro learning

    float m_compassCalOffset[3];
    float m_compassCalScale[3];
    RTVector3 m_compassAverage; // a running average to smooth the mag outputs

    bool m_runtimeMagCalValid;   // true if the runtime mag calibration has valid data
    float m_runtimeMagCalMax[3]; // runtime max mag values seen
    float m_runtimeMagCalMin[3]; // runtime min mag values seen

    float m_axisRotation[RTIMU_AXIS_ROTATION_COUNT][9] = {
        {1, 0, 0, 0, 1, 0, 0, 0, 1},   // RTIMU_XNORTH_YEAST
        {0, -1, 0, 1, 0, 0, 0, 0, 1},  // RTIMU_XEAST_YSOUTH
        {-1, 0, 0, 0, -1, 0, 0, 0, 1}, // RTIMU_XSOUTH_YWEST
        {0, 1, 0, -1, 0, 0, 0, 0, 1},  // RTIMU_XWEST_YNORTH

        {1, 0, 0, 0, -1, 0, 0, 0, -1},  // RTIMU_XNORTH_YWEST
        {0, 1, 0, 1, 0, 0, 0, 0, -1},   // RTIMU_XEAST_YNORTH
        {-1, 0, 0, 0, 1, 0, 0, 0, -1},  // RTIMU_XSOUTH_YEAST
        {0, -1, 0, -1, 0, 0, 0, 0, -1}, // RTIMU_XWEST_YSOUTH

        {0, 1, 0, 0, 0, -1, -1, 0, 0},  // RTIMU_XUP_YNORTH
        {0, 0, 1, 0, 1, 0, -1, 0, 0},   // RTIMU_XUP_YEAST
        {0, -1, 0, 0, 0, 1, -1, 0, 0},  // RTIMU_XUP_YSOUTH
        {0, 0, -1, 0, -1, 0, -1, 0, 0}, // RTIMU_XUP_YWEST

        {0, 1, 0, 0, 0, 1, 1, 0, 0},   // RTIMU_XDOWN_YNORTH
        {0, 0, -1, 0, 1, 0, 1, 0, 0},  // RTIMU_XDOWN_YEAST
        {0, -1, 0, 0, 0, -1, 1, 0, 0}, // RTIMU_XDOWN_YSOUTH
        {0, 0, 1, 0, -1, 0, 1, 0, 0},  // RTIMU_XDOWN_YWEST

        {1, 0, 0, 0, 0, 1, 0, -1, 0},   // RTIMU_XNORTH_YUP
        {0, 0, -1, 1, 0, 0, 0, -1, 0},  // RTIMU_XEAST_YUP
        {-1, 0, 0, 0, 0, -1, 0, -1, 0}, // RTIMU_XSOUTH_YUP
        {0, 0, 1, -1, 0, 0, 0, -1, 0},  // RTIMU_XWEST_YUP

        {1, 0, 0, 0, 0, -1, 0, 1, 0}, // RTIMU_XNORTH_YDOWN
        {0, 0, 1, 1, 0, 0, 0, 1, 0},  // RTIMU_XEAST_YDOWN
        {-1, 0, 0, 0, 0, 1, 0, 1, 0}, // RTIMU_XSOUTH_YDOWN
        {0, 0, -1, -1, 0, 0, 0, 1, 0} // RTIMU_XWEST_YDOWN
    };                                // array of rotation matrices

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