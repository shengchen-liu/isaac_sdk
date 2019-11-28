#pragma once

#include "RTIMUHal.hpp"

//  Useful constants

#define RTMATH_PI 3.1415926535
#define RTMATH_DEGREE_TO_RAD (RTMATH_PI / 180.0)
#define RTMATH_RAD_TO_DEGREE (180.0 / RTMATH_PI)

namespace isaac
{
namespace drivers
{
class RTVector3
{
public:
    RTVector3();
    RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z);

    // const RTVector3 &operator+=(RTVector3 &vec);
    // const RTVector3 &operator-=(RTVector3 &vec);

    // RTVector3 &operator=(const RTVector3 &vec);

    // RTFLOAT length();
    // void normalize();
    void zero();
    // const char *display();
    // const char *displayDegrees();

    // static float dotProduct(const RTVector3 &a, const RTVector3 &b);
    // static void crossProduct(const RTVector3 &a, const RTVector3 &b, RTVector3 &d);

    // void accelToEuler(RTVector3 &rollPitchYaw) const;
    // void accelToQuaternion(RTQuaternion &qPose) const;

    // inline RTFLOAT x() const { return m_data[0]; }
    // inline RTFLOAT y() const { return m_data[1]; }
    // inline RTFLOAT z() const { return m_data[2]; }
    // inline RTFLOAT data(const int i) const { return m_data[i]; }

    // inline void setX(const RTFLOAT val) { m_data[0] = val; }
    // inline void setY(const RTFLOAT val) { m_data[1] = val; }
    // inline void setZ(const RTFLOAT val) { m_data[2] = val; }
    // inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }
    // inline void fromArray(RTFLOAT *val) { memcpy(m_data, val, 3 * sizeof(RTFLOAT)); }
    // inline void toArray(RTFLOAT *val) const { memcpy(val, m_data, 3 * sizeof(RTFLOAT)); }

private:
    float m_data[3];
};
} // namespace drivers
} // namespace isaac