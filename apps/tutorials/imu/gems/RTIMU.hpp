#pragma once
// #include "RTMath.hpp"
// #include "RTFusion.hpp"
#include "RTIMULibDefs.hpp"
#include "RTIMUSettings.hpp"

//  Axis rotation defs
//
//  These allow the IMU to be virtually repositioned if it is in a non-standard configuration
//  Standard configuration is X pointing at north, Y pointing east and Z pointing down
//  with the IMU horizontal. There are 24 different possible orientations as defined
//  below. Setting the axis rotation code to non-zero values performs the repositioning.

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
class RTIMUSettings;
}
} // namespace isaac

namespace isaac
{
namespace drivers
{
class RTIMU
{
public:
    //  IMUs should always be created with the following call
    //it's just a declaration, not a definition!
    //The * belongs to the return type (RTIMU), and not to the function name, i.e.:
    static RTIMU *createIMU();

    //  Constructor/destructor

    RTIMU(RTIMUSettings *settings);
    virtual ~RTIMU();

    //  These functions must be provided by sub classes

    virtual const char *IMUName() = 0;    // the name of the IMU
    virtual int IMUType() = 0;            // the type code of the IMU
    virtual bool IMUInit() = 0;           // set up the IMU
    virtual int IMUGetPollInterval() = 0; // returns the recommended poll interval in mS
    virtual bool IMURead() = 0;           // get a sample

    // setGyroContinuousALearninglpha allows the continuous learning rate to be over-ridden
    // The value must be between 0.0 and 1.0 and will generally be close to 0

    bool setGyroContinuousLearningAlpha(RTFLOAT alpha);

    // returns true if enough samples for valid data

    virtual bool IMUGyroBiasValid();

    //  the following function can be called to set the SLERP power

    void setSlerpPower(RTFLOAT power) { m_fusion->setSlerpPower(power); }

    //  call the following to reset the fusion algorithm

    void resetFusion() { m_fusion->reset(); }

    //  the following three functions control the influence of the gyro, accel and compass sensors

    void setGyroEnable(bool enable) { m_fusion->setGyroEnable(enable); }
    void setAccelEnable(bool enable) { m_fusion->setAccelEnable(enable); }
    void setCompassEnable(bool enable) { m_fusion->setCompassEnable(enable); }

    //  call the following to enable debug messages

    void setDebugEnable(bool enable) { m_fusion->setDebugEnable(enable); }

    //  getIMUData returns the standard outputs of the IMU and fusion filter

    const RTIMU_DATA &getIMUData() { return m_imuData; }

    //  setExtIMUData allows data from some external IMU to be injected to the fusion algorithm

    void setExtIMUData(RTFLOAT gx, RTFLOAT gy, RTFLOAT gz, RTFLOAT ax, RTFLOAT ay, RTFLOAT az,
                       RTFLOAT mx, RTFLOAT my, RTFLOAT mz, uint64_t timestamp);

    //  the following two functions get access to the measured pose (accel and compass)

    const RTVector3 &getMeasuredPose() { return m_fusion->getMeasuredPose(); }
    const RTQuaternion &getMeasuredQPose() { return m_fusion->getMeasuredQPose(); }

    //  setCompassCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setCompassCalibrationMode(bool enable) { m_compassCalibrationMode = enable; }

    //  setAccelCalibrationMode() turns off use of cal data so that raw data can be accumulated
    //  to derive calibration data

    void setAccelCalibrationMode(bool enable) { m_accelCalibrationMode = enable; }

    //  setCalibrationData configures the cal data from settings and also enables use if valid

    void setCalibrationData();

    //  getCompassCalibrationValid() returns true if the compass min/max calibration data is being used

    bool getCompassCalibrationValid() { return !m_compassCalibrationMode && m_settings->m_compassCalValid; }

    //  getRuntimeCompassCalibrationValid() returns true if the runtime compass min/max calibration data is being used

    bool getRuntimeCompassCalibrationValid() { return !m_compassCalibrationMode && m_runtimeMagCalValid; }

    //  getCompassCalibrationEllipsoidValid() returns true if the compass ellipsoid calibration data is being used

    bool getCompassCalibrationEllipsoidValid() { return !m_compassCalibrationMode && m_settings->m_compassCalEllipsoidValid; }

    //  getAccelCalibrationValid() returns true if the accel calibration data is being used

    bool getAccelCalibrationValid() { return !m_accelCalibrationMode && m_settings->m_accelCalValid; }

    const RTVector3 &getGyro() { return m_imuData.gyro; }       // gets gyro rates in radians/sec
    const RTVector3 &getAccel() { return m_imuData.accel; }     // get accel data in gs
    const RTVector3 &getCompass() { return m_imuData.compass; } // gets compass data in uT

    RTVector3 getAccelResiduals() { return m_fusion->getAccelResiduals(); }

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

    int m_sampleRate;          // samples per second
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

    static float m_axisRotation[RTIMU_AXIS_ROTATION_COUNT][9]; // array of rotation matrices
};

} // namespace drivers
} // namespace isaac

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

namespace isaac
{
namespace drivers
{
class RTIMULSM6DS33LIS3MDL : public RTIMU
{
public:
    RTIMULSM6DS33LIS3MDL();
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

#ifdef LSM6DS33LIS3MDL_CACHE_MODE
    bool m_firstTime; // if first sample

    LSM6DS33LIS3MDL_CACHE_BLOCK m_cache[LSM6DS33LIS3MDL_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                                          // the in index
    int m_cacheOut;                                                         // the out index
    int m_cacheCount;                                                       // number of used cache blocks

#endif
};
} // namespace drivers
} // namespace isaac