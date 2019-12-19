#pragma once

#include "RTIMULibDefs.hpp"

namespace isaac
{
namespace drivers
{
class RTIMUSettings;
}
} // namespace isaac

namespace isaac
{
namespace drivers
{
class RTFusion
{
public:
    RTFusion();
    virtual ~RTFusion();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_NULL; }

    //  the following function can be called to set the SLERP power

    void setSlerpPower(RTFLOAT power) { m_slerpPower = power; }

    //  reset() resets the fusion state but keeps any setting changes (such as enables)

    virtual void reset() {}

    //  newIMUData() should be called for subsequent updates
    //  the fusion fields are updated with the results

    virtual void newIMUData(RTIMU_DATA & /* data */, const RTIMUSettings * /* settings */) {}

    //  This static function returns performs the type to name mapping

    static const char *fusionName(int fusionType) { return m_fusionNameMap[fusionType]; }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_enableGyro = enable; }
    void setAccelEnable(bool enable) { m_enableAccel = enable; }
    void setCompassEnable(bool enable) { m_enableCompass = enable; }

    inline const RTVector3 &getMeasuredPose() { return m_measuredPose; }
    inline const RTQuaternion &getMeasuredQPose() { return m_measuredQPose; }

    // //  getAccelResiduals() gets the residual after subtracting gravity

    RTVector3 getAccelResiduals();

    void setDebugEnable(bool enable) { m_debug = enable; }

protected:
    void calculatePose(const RTVector3 &accel, const RTVector3 &mag, float magDeclination); // generates pose from accels and mag

    RTVector3 m_gyro;    // current gyro sample
    RTVector3 m_accel;   // current accel sample
    RTVector3 m_compass; // current compass sample

    RTQuaternion m_measuredQPose; // quaternion form of pose from measurement
    RTVector3 m_measuredPose;     // vector form of pose from measurement
    RTQuaternion m_fusionQPose;   // quaternion form of pose from fusion
    RTVector3 m_fusionPose;       // vector form of pose from fusion

    RTQuaternion m_gravity; // the gravity vector as a quaternion

    float m_slerpPower;           // a value 0 to 1 that controls measured state influence
    RTQuaternion m_rotationDelta;   // amount by which measured state differs from predicted
    RTQuaternion m_rotationPower;   // delta raised to the appopriate power
    RTVector3 m_rotationUnitVector; // the vector part of the rotation delta

    bool m_debug;
    bool m_enableGyro;    // enables gyro as input
    bool m_enableAccel;   // enables accel as input
    bool m_enableCompass; // enables compass a input
    bool m_compassValid;  // true if compass data valid

    bool m_firstTime;          // if first time after reset
    uint64_t m_lastFusionTime; // for delta time calculation

    static const char *m_fusionNameMap[]; // the fusion name array
};
} // namespace drivers
} // namespace isaac
