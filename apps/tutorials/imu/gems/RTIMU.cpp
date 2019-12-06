#include "RTIMU.hpp"

namespace isaac
{
namespace drivers
{
RTIMU::RTIMU(RTIMUSettings *settings)
{
    m_settings = settings;

    m_compassCalibrationMode = false;
    m_accelCalibrationMode = false;

    m_runtimeMagCalValid = false;
}
} // namespace drivers
} // namespace isaac
