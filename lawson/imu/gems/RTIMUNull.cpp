#include "RTIMUNull.hpp"

namespace isaac
{
namespace drivers
{
RTIMUNull::RTIMUNull(RTIMUSettings *settings) : RTIMU(settings)
{
}

RTIMUNull::~RTIMUNull()
{
}

bool RTIMUNull::IMUInit()
{
    return true;
}

int RTIMUNull::IMUGetPollInterval()
{
    return (100); // just a dummy value really
}

bool RTIMUNull::IMURead()
{
    updateFusion();
    return true;
}

void RTIMUNull::setIMUData(const RTIMU_DATA &data)
{
    m_imuData = data;
}

} // namespace drivers
} // namespace isaac