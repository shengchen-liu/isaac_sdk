#pragma once

#include "RTIMU.hpp"
#include "RTIMUSettings.hpp"

namespace isaac
{
namespace drivers
{
class RTIMUNull : public RTIMU
{
public:
    RTIMUNull(RTIMUSettings *settings);
    ~RTIMUNull();

    // The timestamp parameter is assumed to be from RTMath::currentUSecsSinceEpoch()

    void setIMUData(const RTIMU_DATA &data);

    virtual const char *IMUName() { return "Null IMU"; }
    virtual int IMUType() { return RTIMU_TYPE_NULL; }
    virtual bool IMUInit();
    virtual int IMUGetPollInterval();
    virtual bool IMURead();
    virtual bool IMUGyroBiasValid() { return true; }

private:
    uint64_t m_timestamp;
};
} // namespace drivers
} // namespace isaac