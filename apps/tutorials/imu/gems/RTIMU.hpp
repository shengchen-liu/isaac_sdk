#pragma once

#include "RTIMUSettings.hpp"
// #include "RTIMULibDefs.hpp"

namespace isaac
{
namespace drivers
{
class RTIMU
{
public:
    //  Constructor/destructor
    RTIMU(RTIMUSettings *settings);

    int IMUType() { return 11; } // the type code of the IMU
protected:
    bool m_compassCalibrationMode; // true if cal mode so don't use cal data!
    bool m_accelCalibrationMode;   // true if cal mode so don't use cal data!

    // RTIMU_DATA m_imuData; // the data from the IMU

    RTIMUSettings *m_settings; // the settings object pointer

    bool m_runtimeMagCalValid; // true if the runtime mag calibration has valid data
};

} // namespace drivers
} // namespace isaac