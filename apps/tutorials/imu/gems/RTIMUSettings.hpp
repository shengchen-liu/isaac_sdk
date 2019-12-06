#pragma once

// #include "RTMath.hpp"
#include "RTIMUHal.hpp"

//  Settings keys

#define RTIMULIB_IMU_TYPE "IMUType"
#define RTIMULIB_FUSION_TYPE "FusionType"
#define RTIMULIB_BUS_IS_I2C "BusIsI2C"
#define RTIMULIB_I2C_SLAVEADDRESS "I2CSlaveAddress"
#define RTIMULIB_I2C_BUS "I2CBus"
#define RTIMULIB_SPI_BUS "SPIBus"
#define RTIMULIB_SPI_SELECT "SPISelect"
#define RTIMULIB_SPI_SPEED "SPISpeed"
#define RTIMULIB_AXIS_ROTATION "AxisRotation"
#define RTIMULIB_PRESSURE_TYPE "PressureType"
#define RTIMULIB_I2C_PRESSUREADDRESS "I2CPressureAddress"
#define RTIMULIB_HUMIDITY_TYPE "HumidityType"
#define RTIMULIB_I2C_HUMIDITYADDRESS "I2CHumidityAddress"

//  LSM6DS33LIS3MDL defaults

#define RTIMULIB_LSM6DS33LIS3MDL_GYRO_SAMPLERATE "LSM6DS33LIS3MDLGyroSampleRate"
#define RTIMULIB_LSM6DS33LIS3MDL_GYRO_HPF "LSM6DS33LIS3MDLGyroHpf"
#define RTIMULIB_LSM6DS33LIS3MDL_GYRO_FSR "LSM6DS33LIS3MDLGyroFsr"

#define RTIMULIB_LSM6DS33LIS3MDL_ACCEL_SAMPLERATE "LSM6DS33LIS3MDLAccelSampleRate"
#define RTIMULIB_LSM6DS33LIS3MDL_ACCEL_HPF "LSM6DS33LIS3MDLAccelHpf"
#define RTIMULIB_LSM6DS33LIS3MDL_ACCEL_FSR "LSM6DS33LIS3MDLAccelFsr"
#define RTIMULIB_LSM6DS33LIS3MDL_ACCEL_LPF "LSM6DS33LIS3MDLAccelLpf"

#define RTIMULIB_LSM6DS33LIS3MDL_COMPASS_SAMPLERATE "LSM6DS33LIS3MDLCompassSampleRate"
#define RTIMULIB_LSM6DS33LIS3MDL_COMPASS_FSR "LSM6DS33LIS3MDLCompassFsr"

//  Gyro bias keys

#define RTIMULIB_GYRO_BIAS_VALID "GyroBiasValid"
#define RTIMULIB_GYRO_BIAS_X "GyroBiasX"
#define RTIMULIB_GYRO_BIAS_Y "GyroBiasY"
#define RTIMULIB_GYRO_BIAS_Z "GyroBiasZ"

//  Compass calibration and adjustment settings keys

#define RTIMULIB_COMPASSCAL_VALID "CompassCalValid"
#define RTIMULIB_COMPASSCAL_MINX "CompassCalMinX"
#define RTIMULIB_COMPASSCAL_MAXX "CompassCalMaxX"
#define RTIMULIB_COMPASSCAL_MINY "CompassCalMinY"
#define RTIMULIB_COMPASSCAL_MAXY "CompassCalMaxY"
#define RTIMULIB_COMPASSCAL_MINZ "CompassCalMinZ"
#define RTIMULIB_COMPASSCAL_MAXZ "CompassCalMaxZ"

#define RTIMULIB_COMPASSCAL_ELLIPSOID_VALID "compassCalEllipsoidValid"
#define RTIMULIB_COMPASSCAL_OFFSET_X "compassCalOffsetX"
#define RTIMULIB_COMPASSCAL_OFFSET_Y "compassCalOffsetY"
#define RTIMULIB_COMPASSCAL_OFFSET_Z "compassCalOffsetZ"
#define RTIMULIB_COMPASSCAL_CORR11 "compassCalCorr11"
#define RTIMULIB_COMPASSCAL_CORR12 "compassCalCorr12"
#define RTIMULIB_COMPASSCAL_CORR13 "compassCalCorr13"
#define RTIMULIB_COMPASSCAL_CORR21 "compassCalCorr21"
#define RTIMULIB_COMPASSCAL_CORR22 "compassCalCorr22"
#define RTIMULIB_COMPASSCAL_CORR23 "compassCalCorr23"
#define RTIMULIB_COMPASSCAL_CORR31 "compassCalCorr31"
#define RTIMULIB_COMPASSCAL_CORR32 "compassCalCorr32"
#define RTIMULIB_COMPASSCAL_CORR33 "compassCalCorr33"

#define RTIMULIB_COMPASSADJ_DECLINATION "compassAdjDeclination"

//  Accel calibration settings keys

#define RTIMULIB_ACCELCAL_VALID "AccelCalValid"
#define RTIMULIB_ACCELCAL_MINX "AccelCalMinX"
#define RTIMULIB_ACCELCAL_MAXX "AccelCalMaxX"
#define RTIMULIB_ACCELCAL_MINY "AccelCalMinY"
#define RTIMULIB_ACCELCAL_MAXY "AccelCalMaxY"
#define RTIMULIB_ACCELCAL_MINZ "AccelCalMinZ"
#define RTIMULIB_ACCELCAL_MAXZ "AccelCalMaxZ"

# define RTFLOAT float

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

    RTVector3 &operator=(const RTVector3 &vec);
    const RTVector3 &operator+=(RTVector3 &vec);
    const RTVector3 &operator-=(RTVector3 &vec);

    RTFLOAT length();
    void normalize();
    void zero();
    const char *display();
    const char *displayDegrees();

    static float dotProduct(const RTVector3 &a, const RTVector3 &b);
    static void crossProduct(const RTVector3 &a, const RTVector3 &b, RTVector3 &d);

    void accelToEuler(RTVector3 &rollPitchYaw) const;
    // void accelToQuaternion(RTQuaternion &qPose) const;

    inline RTFLOAT x() const { return m_data[0]; }
    inline RTFLOAT y() const { return m_data[1]; }
    inline RTFLOAT z() const { return m_data[2]; }
    inline RTFLOAT data(const int i) const { return m_data[i]; }

    inline void setX(const RTFLOAT val) { m_data[0] = val; }
    inline void setY(const RTFLOAT val) { m_data[1] = val; }
    inline void setZ(const RTFLOAT val) { m_data[2] = val; }
    inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }
    inline void fromArray(RTFLOAT *val) { memcpy(m_data, val, 3 * sizeof(RTFLOAT)); }
    inline void toArray(RTFLOAT *val) const { memcpy(val, m_data, 3 * sizeof(RTFLOAT)); }

private:
    float m_data[3];
};
} // namespace drivers
} // namespace isaac

namespace isaac
{
namespace drivers
{
class RTIMUSettings : public RTIMUHal
{
public:
    //  Standard constructor sets up for ini file in working directory

    RTIMUSettings(const char *productType = "RTIMULib");

    //  Alternate constructor allow ini file to be in any directory

    RTIMUSettings(const char *settingsDirectory, const char *productType);

    //  This function tries to find an IMU. It stops at the first valid one
    //  and returns true or else false

    bool discoverIMU(int &imuType, bool &busIsI2C, unsigned char &slaveAddress);

    //  This function sets the settings to default values.

    void setDefaults();

    // //  This function loads the local variables from the settings file or uses defaults

    virtual bool loadSettings();

    //  This function saves the local variables to the settings file

    virtual bool saveSettings();

    //  These are the local variables

    int m_imuType;                   // type code of imu in use
    int m_fusionType;                // fusion algorithm type code
    unsigned char m_I2CSlaveAddress; // I2C slave address of the imu
    int m_axisRotation;              // axis rotation code

    bool m_compassCalValid;    // true if there is valid compass calibration data
    RTVector3 m_compassCalMin;     // the minimum values
    RTVector3 m_compassCalMax; // the maximum values

    bool m_compassCalEllipsoidValid;       // true if the ellipsoid calibration data is valid
    RTVector3 m_compassCalEllipsoidOffset; // the ellipsoid offset
    float m_compassCalEllipsoidCorr[3][3]; // the correction matrix

    float m_compassAdjDeclination; // magnetic declination adjustment - subtracted from measured

    bool m_accelCalValid;    // true if there is valid accel calibration data
    RTVector3 m_accelCalMin; // the minimum values
    RTVector3 m_accelCalMax; // the maximum values

    bool m_gyroBiasValid; // true if the recorded gyro bias is valid
    RTVector3 m_gyroBias; // the recorded gyro bias

    // //  IMU-specific vars

    // //  LSM6DS33LIS3MDL defaults

    int m_LSM6DS33LIS3MDLGyroSampleRate; // the gyro sample rate
    int m_LSM6DS33LIS3MDLGyroHpf;        // the gyro high pass filter cutoff code
    int m_LSM6DS33LIS3MDLGyroFsr;        // the gyro full scale range

    int m_LSM6DS33LIS3MDLAccelSampleRate; // the accel sample rate
    int m_LSM6DS33LIS3MDLAccelHpf;        // the accel high pass filter cutoff code
    int m_LSM6DS33LIS3MDLAccelFsr;        // the accel full scale range
    int m_LSM6DS33LIS3MDLAccelLpf;        // the accel low pass filter

    int m_LSM6DS33LIS3MDLCompassSampleRate; // the compass sample rate
    int m_LSM6DS33LIS3MDLCompassFsr;        // the compass full scale range

private:
    void setBlank();
    void setComment(const char *comment);
    void setValue(const char *key, const bool val);
    void setValue(const char *key, const int val);
    void setValue(const char *key, const RTFLOAT val);

    char m_filename[256]; // the settings file name

    FILE *m_fd;
};

class RTQuaternion
{
public:
    RTQuaternion();
    RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z);

    RTQuaternion &operator+=(const RTQuaternion &quat);
    RTQuaternion &operator-=(const RTQuaternion &quat);
    RTQuaternion &operator*=(const RTQuaternion &qb);
    RTQuaternion &operator*=(const RTFLOAT val);
    RTQuaternion &operator-=(const RTFLOAT val);

    RTQuaternion &operator=(const RTQuaternion &quat);
    const RTQuaternion operator*(const RTQuaternion &qb) const;
    const RTQuaternion operator*(const RTFLOAT val) const;
    const RTQuaternion operator-(const RTQuaternion &qb) const;
    const RTQuaternion operator-(const RTFLOAT val) const;

    void normalize();
    void toEuler(RTVector3 &vec);
    void fromEuler(RTVector3 &vec);
    RTQuaternion conjugate() const;
    void toAngleVector(RTFLOAT &angle, RTVector3 &vec);
    void fromAngleVector(const RTFLOAT &angle, const RTVector3 &vec);

    void zero();
    const char *display();

    inline RTFLOAT scalar() const { return m_data[0]; }
    inline RTFLOAT x() const { return m_data[1]; }
    inline RTFLOAT y() const { return m_data[2]; }
    inline RTFLOAT z() const { return m_data[3]; }
    inline RTFLOAT data(const int i) const { return m_data[i]; }

    inline void setScalar(const RTFLOAT val) { m_data[0] = val; }
    inline void setX(const RTFLOAT val) { m_data[1] = val; }
    inline void setY(const RTFLOAT val) { m_data[2] = val; }
    inline void setZ(const RTFLOAT val) { m_data[3] = val; }
    inline void setData(const int i, RTFLOAT val) { m_data[i] = val; }
    inline void fromArray(RTFLOAT *val) { memcpy(m_data, val, 4 * sizeof(RTFLOAT)); }
    inline void toArray(RTFLOAT *val) const { memcpy(val, m_data, 4 * sizeof(RTFLOAT)); }

private:
    RTFLOAT m_data[4];
};
} // namespace drivers
} // namespace isaac

