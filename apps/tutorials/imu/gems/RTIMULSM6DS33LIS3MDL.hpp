#pragma once

#include "RTIMU.hpp"

namespace isaac
{
namespace drivers
{

class RTIMULSM6DS33LIS3MDL : public RTIMU
{
public:
    RTIMULSM6DS33LIS3MDL(RTIMUSettings *settings);
    ~RTIMULSM6DS33LIS3MDL();

    virtual const char *IMUName() { return "LSM6DS33 + LIS3MDL"; }
    virtual int IMUType() { return RTIMU_TYPE_LSM6DS33LIS3MDL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();

private:
    bool setGyroSampleRate();
    bool setGyro();
    bool setAccel();
    bool setCompass();

    unsigned char m_gyroAccelSlaveAddr; // I2C address of LSM6DS33
    unsigned char m_compassSlaveAddr;   // I2C address of LIS3MDL

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;
    RTFLOAT m_compassScale;
};

} // namespace drivers
} // namespace isaac