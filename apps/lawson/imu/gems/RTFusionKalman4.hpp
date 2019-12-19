#pragma once
#ifndef _RTFUSIONKALMAN4_H
#define _RTFUSIONKALMAN4_H

#include "RTFusion.hpp"

namespace isaac
{
namespace drivers
{
class RTFusionKalman4 : public RTFusion
{
public:
    RTFusionKalman4();
    ~RTFusionKalman4();

    //  fusionType returns the type code of the fusion algorithm

    virtual int fusionType() { return RTFUSION_TYPE_KALMANSTATE4; }

    //  reset() resets the kalman state but keeps any setting changes (such as enables)

    void reset();

    //  newIMUData() should be called for subsequent updates
    //  deltaTime is in units of seconds

    void newIMUData(RTIMU_DATA &data, const RTIMUSettings *settings);

    //  the following two functions can be called to customize the covariance matrices

    void setQMatrix(RTMatrix4x4 Q)
    {
        m_Q = Q;
        reset();
    }
    void setRkMatrix(RTMatrix4x4 Rk)
    {
        m_Rk = Rk;
        reset();
    }

private:
    void predict();
    void update();

    RTVector3 m_gyro;    // unbiased gyro data
    RTFLOAT m_timeDelta; // time between predictions

    RTQuaternion m_stateQ;      // quaternion state vector
    RTQuaternion m_stateQError; // difference between stateQ and measuredQ

    RTMatrix4x4 m_Kk;          // the Kalman gain matrix
    RTMatrix4x4 m_Pkk_1;       // the predicted estimated covariance matrix
    RTMatrix4x4 m_Pkk;         // the updated estimated covariance matrix
    RTMatrix4x4 m_PDot;        // the derivative of the covariance matrix
    RTMatrix4x4 m_Q;           // process noise covariance
    RTMatrix4x4 m_Fk;          // the state transition matrix
    RTMatrix4x4 m_FkTranspose; // the state transition matrix transposed
    RTMatrix4x4 m_Rk;          // the measurement noise covariance

    //  Note: SInce Hk ends up being the identity matrix, these are omitted

    //    RTMatrix4x4 m_Hk;                                     // map from state to measurement
    //    RTMatrix4x4> m_HkTranspose;                           // transpose of map
};

} // namespace drivers
} // namespace isaac

#endif // _RTFUSIONKALMAN4_H