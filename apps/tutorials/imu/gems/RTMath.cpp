#include "RTMath.hpp"

//  The RTVector3 class
namespace isaac
{
namespace drivers
{
RTVector3::RTVector3()
{
    zero();
}

RTVector3::RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z)
{
    m_data[0] = x;
    m_data[1] = y;
    m_data[2] = z;
}

// RTVector3 &RTVector3::operator=(const RTVector3 &vec)
// {
//     if (this == &vec)
//         return *this;

//     m_data[0] = vec.m_data[0];
//     m_data[1] = vec.m_data[1];
//     m_data[2] = vec.m_data[2];

//     return *this;
// }

// const RTVector3 &RTVector3::operator+=(RTVector3 &vec)
// {
//     for (int i = 0; i < 3; i++)
//         m_data[i] += vec.m_data[i];
//     return *this;
// }

// const RTVector3 &RTVector3::operator-=(RTVector3 &vec)
// {
//     for (int i = 0; i < 3; i++)
//         m_data[i] -= vec.m_data[i];
//     return *this;
// }

void RTVector3::zero()
{
    for (int i = 0; i < 3; i++)
        m_data[i] = 0;
}

// RTFLOAT RTVector3::dotProduct(const RTVector3 &a, const RTVector3 &b)
// {
//     return a.x() * b.x() + a.y() * b.y() + a.z() * b.z();
// }

// void RTVector3::crossProduct(const RTVector3 &a, const RTVector3 &b, RTVector3 &d)
// {
//     d.setX(a.y() * b.z() - a.z() * b.y());
//     d.setY(a.z() * b.x() - a.x() * b.z());
//     d.setZ(a.x() * b.y() - a.y() * b.x());
// }

// void RTVector3::accelToEuler(RTVector3 &rollPitchYaw) const
// {
//     RTVector3 normAccel = *this;

//     normAccel.normalize();

//     rollPitchYaw.setX(atan2(normAccel.y(), normAccel.z()));
//     rollPitchYaw.setY(-atan2(normAccel.x(), sqrt(normAccel.y() * normAccel.y() + normAccel.z() * normAccel.z())));
//     rollPitchYaw.setZ(0);
// }

// void RTVector3::accelToQuaternion(RTQuaternion &qPose) const
// {
//     RTVector3 normAccel = *this;
//     RTVector3 vec;
//     RTVector3 z(0, 0, 1.0);

//     normAccel.normalize();

//     RTFLOAT angle = acos(RTVector3::dotProduct(z, normAccel));
//     RTVector3::crossProduct(normAccel, z, vec);
//     vec.normalize();

//     qPose.fromAngleVector(angle, vec);
// }

// void RTVector3::normalize()
// {
//     RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
//                           m_data[2] * m_data[2]);

//     if (length == 0)
//         return;

//     m_data[0] /= length;
//     m_data[1] /= length;
//     m_data[2] /= length;
// }

// RTFLOAT RTVector3::length()
// {
//     return sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
//                 m_data[2] * m_data[2]);
// }
} // namespace drivers
} // namespace isaac