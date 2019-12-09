#include "RTIMU.hpp"
#include "RTFusionKalman4.hpp"
#include <iostream>


//  this sets the learning rate for compass running average calculation

#define COMPASS_ALPHA 0.2f

//  this sets the min range (max - min) of values to trigger runtime mag calibration

#define RTIMU_RUNTIME_MAGCAL_RANGE  30

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_GYRO_ZERO      0.20

//  this defines the accelerometer noise level

#define RTIMU_FUZZY_ACCEL_ZERO      0.05

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

    for (int i = 0; i < 3; i++)
    {
        m_runtimeMagCalMax[i] = -1000;
        m_runtimeMagCalMin[i] = 1000;
    }

    switch (m_settings->m_fusionType)
    {
    case RTFUSION_TYPE_KALMANSTATE4:
        m_fusion = new RTFusionKalman4();
        break;

        // case RTFUSION_TYPE_RTQF:
        //     m_fusion = new RTFusionRTQF();
        //     break;

    default:
        m_fusion = new RTFusion();
        break;
    }
    HAL_INFO1("Using fusion algorithm %s\n", RTFusion::fusionName(m_settings->m_fusionType));
};

bool RTIMU::IMUInit()
{
    unsigned char result;
    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = false;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_gyroAccelSlaveAddr = m_settings->m_I2CSlaveAddress;

    // work out gyro/accel address

    if (!m_settings->HALRead(LSM6DS33_ADDRESS0, LSM6DS33_WHO_AM_I, 1, &result, "Reading WHO AM I on Addr0"))
    {
        if (!m_settings->HALRead(LSM6DS33_ADDRESS1, LSM6DS33_WHO_AM_I, 1, &result, "Reading WHO AM I on Addr1"))
        {
            return false;
        }
        else
        {
            if (result == LSM6DS33_ID)
            {
                m_gyroAccelSlaveAddr = LSM6DS33_ADDRESS1;
                std::cout << "Addr1 " << LSM6DS33_ADDRESS1 << std::endl;
            }
        }
    }
    else
    {
        if (result == LSM6DS33_ID)
        {
            m_gyroAccelSlaveAddr = LSM6DS33_ADDRESS0;
            std::cout << "Addr0 " << LSM6DS33_ADDRESS0 << std::endl;
        }
    }

    setCalibrationData();

    //  enable the I2C bus

    if (!m_settings->HALOpen())
        return false;

    HAL_INFO("LSM6DS33LIS3MDL init complete\n");
    return true;

    //  Set up the gyro/accel

    // IF_INC = 1 (automatically increment address register)
    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL3_C, 0x04, "Failed to set LSM6DS33 automatic register address increment"))
    {
        std::cout << "Error setting LSM6DS33 gyroscope" << std::endl;
        return false;
    }
    else
    {
        std::cout << "Set LSM6DS33 automatic register address increment" << std::endl;
    }

    if (!setGyro())
        return false;

    if (!setAccel())
    {
        std::cout << "Error setting LSM6DS33 accelerometer" << std::endl;
        return false;
    }
    else
    {
        std::cout << "LSM6DS33 accelerometer set" << std::endl;
    }

    gyroBiasInit();
}

void RTIMU::setCalibrationData()
{
    float maxDelta = -1;
    float delta;

    if (m_settings->m_compassCalValid)
    {
        //  find biggest range

        for (int i = 0; i < 3; i++)
        {
            if ((m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) > maxDelta)
                maxDelta = m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i);
        }
        if (maxDelta < 0)
        {
            HAL_ERROR("Error in compass calibration data\n");
            return;
        }
        maxDelta /= 2.0f; // this is the max +/- range

        for (int i = 0; i < 3; i++)
        {
            delta = (m_settings->m_compassCalMax.data(i) - m_settings->m_compassCalMin.data(i)) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta; // makes everything the same range
            m_compassCalOffset[i] = (m_settings->m_compassCalMax.data(i) + m_settings->m_compassCalMin.data(i)) / 2.0f;
        }
    }

    if (m_settings->m_compassCalValid)
    {
        HAL_INFO("Using min/max compass calibration\n");
    }
    else
    {
        HAL_INFO("min/max compass calibration not in use\n");
    }

    if (m_settings->m_compassCalEllipsoidValid)
    {
        HAL_INFO("Using ellipsoid compass calibration\n");
    }
    else
    {
        HAL_INFO("Ellipsoid compass calibration not in use\n");
    }

    if (m_settings->m_accelCalValid)
    {
        HAL_INFO("Using accel calibration\n");
    }
    else
    {
        HAL_INFO("Accel calibration not in use\n");
    }
}

bool RTIMU::setGyro()
{
    unsigned char ctrl2_g;
    unsigned char powerMode = 0; // high speed enabled -> powerMode = 0, low power and normal mode -> powerMode = 1

    switch (m_settings->m_LSM6DS33LIS3MDLGyroSampleRate)
    {
    case LSM6DS33_SAMPLERATE_0:
        ctrl2_g = 0x00;
        powerMode = 1;
        m_sampleRate = 0;
        break;

    case LSM6DS33_SAMPLERATE_13:
        ctrl2_g = 0x10;
        powerMode = 1;
        m_sampleRate = 13;
        break;

    case LSM6DS33_SAMPLERATE_26:
        ctrl2_g = 0x20;
        powerMode = 1;
        m_sampleRate = 26;
        break;

    case LSM6DS33_SAMPLERATE_52:
        ctrl2_g = 0x30;
        powerMode = 1;
        m_sampleRate = 52;
        break;

    case LSM6DS33_SAMPLERATE_104:
        ctrl2_g = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_SAMPLERATE_208:
        ctrl2_g = 0x50;
        m_sampleRate = 208;
        break;

    case LSM6DS33_SAMPLERATE_416:
        ctrl2_g = 0x60;
        m_sampleRate = 416;
        break;

    case LSM6DS33_SAMPLERATE_833:
        ctrl2_g = 0x70;
        m_sampleRate = 833;
        break;

    case LSM6DS33_SAMPLERATE_1660:
        ctrl2_g = 0x80;
        m_sampleRate = 1660;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM6DS33LIS3MDLGyroFsr)
    {
    case LSM6DS33_FSR_125:
        ctrl2_g |= 0x02;
        m_gyroScale = (RTFLOAT)0.004375 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_245:
        ctrl2_g |= 0x00;
        m_gyroScale = (RTFLOAT)0.00875 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_500:
        ctrl2_g |= 0x04;
        m_gyroScale = (RTFLOAT)0.0175 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_1000:
        ctrl2_g |= 0x08;
        m_gyroScale = (RTFLOAT)0.035 * RTMATH_DEGREE_TO_RAD;
        break;

    case LSM6DS33_FSR_2000:
        ctrl2_g |= 0x0c;
        m_gyroScale = (RTFLOAT)0.07 * RTMATH_DEGREE_TO_RAD;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroFsr);
        return false;
    }

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G, ctrl2_g, "Failed to set LSM6DS33 CTRL2_G"))
        return false;

    //std::cout << "Writing CTRL2_G " << static_cast<unsigned>(ctrl2_g) << std::endl;

    unsigned char regData;
    m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_CTRL2_G, 1, &regData, "Failed to read LSM6DS33 CTRL2_G");

    if ((m_settings->m_GD20HM303DGyroHpf < LSM6DS33_HPF_0) || (m_settings->m_LSM6DS33LIS3MDLGyroHpf > LSM6DS33_HPF_3))
    {
        HAL_ERROR1("Illegal LSM6DS33 high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLGyroHpf);
        return false;
    }

    unsigned char hpgEn = 1;

    unsigned char gyroHpf = m_settings->m_LSM6DS33LIS3MDLGyroHpf;

    // G_HM_MODE, HP_G_EN, HPCF_G1, HPCF_G0, HP_G_RST, ROUNDING_STATUS, 0, 0
    unsigned char ctrl7_g = (powerMode << 7) | (hpgEn << 6) | (gyroHpf << 4) | 0x00;

    //std::cout << "Writing CTRL7_G " << static_cast<unsigned>(ctrl7_g) << std::endl;

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL7_G, ctrl7_g, "Failed to set LSM6DS33 CTRL7_G"))
        return false;

    std::cout << "LSM6DS33 Gyro set" << std::endl;
    return true;
}

bool RTIMU::setAccel()
{
    unsigned char ctrl1_xl;

    if ((m_settings->m_GD20HM303DAccelSampleRate < 0) || (m_settings->m_GD20HM303DAccelSampleRate > 10))
    {
        HAL_ERROR1("Illegal LSM303D accel sample rate code %d\n", m_settings->m_GD20HM303DAccelSampleRate);
        return false;
    }

    switch (m_settings->m_LSM6DS33LIS3MDLAccelSampleRate)
    {
    case LSM6DS33_ACCEL_SAMPLERATE_0:
        ctrl1_xl = 0x00;
        m_sampleRate = 0;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_13:
        ctrl1_xl = 0x10;
        m_sampleRate = 13;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_26:
        ctrl1_xl = 0x20;
        m_sampleRate = 26;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_52:
        ctrl1_xl = 0x30;
        m_sampleRate = 52;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_104:
        ctrl1_xl = 0x40;
        m_sampleRate = 104;
        break;

    case LSM6DS33_SAMPLERATE_208:
        ctrl1_xl = 0x50;
        m_sampleRate = 208;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_416:
        ctrl1_xl = 0x60;
        m_sampleRate = 416;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_833:
        ctrl1_xl = 0x70;
        m_sampleRate = 833;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_1660:
        ctrl1_xl = 0x80;
        m_sampleRate = 1660;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_3330:
        ctrl1_xl = 0x90;
        m_sampleRate = 3330;
        break;

    case LSM6DS33_ACCEL_SAMPLERATE_6660:
        ctrl1_xl = 0xa0;
        m_sampleRate = 6660;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 sample rate code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelSampleRate);
        return false;
    }

    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;

    switch (m_settings->m_LSM6DS33LIS3MDLAccelFsr)
    {
    case LSM6DS33_ACCEL_FSR_2:
        ctrl1_xl |= (0x00 << 2);
        m_accelScale = (RTFLOAT)0.000061;
        break;

    case LSM6DS33_ACCEL_FSR_16:
        ctrl1_xl |= (0x01 << 2);
        m_accelScale = (RTFLOAT)0.000488;
        break;

    case LSM6DS33_ACCEL_FSR_4:
        ctrl1_xl |= (0x10 << 2);
        m_accelScale = (RTFLOAT)0.000122;
        break;

    case LSM6DS33_ACCEL_FSR_8:
        ctrl1_xl |= (0x11 << 2);
        m_accelScale = (RTFLOAT)0.000244;
        break;

    default:
        HAL_ERROR1("Illegal LSM6DS33 FSR code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelFsr);
        return false;
    }

    switch (m_settings->m_LSM6DS33LIS3MDLAccelLpf)
    {
    case LSM6DS33_ACCEL_LPF_400:
        ctrl1_xl |= 0x00;
        break;

    case LSM6DS33_ACCEL_LPF_200:
        ctrl1_xl |= 0x01;
        break;

    case LSM6DS33_ACCEL_LPF_100:
        ctrl1_xl |= 0x02;
        break;

    case LSM6DS33_ACCEL_LPF_50:
        ctrl1_xl |= 0x03;
        break;
    }

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL1_XL, ctrl1_xl, "Failed to set LSM6DS33 CTRL1_XL"))
        return false;

    //std::cout << "Writing LSM6DS33 CTRL1_XL: " << static_cast<unsigned>(ctrl1_xl) << std::endl;

    if ((m_settings->m_LSM6DS33LIS3MDLAccelHpf < LSM6DS33_ACCEL_HPF_0) || (m_settings->m_LSM6DS33LIS3MDLAccelHpf > LSM6DS33_ACCEL_HPF_3))
    {
        HAL_ERROR1("Illegal LSM6DS33 high pass filter code %d\n", m_settings->m_LSM6DS33LIS3MDLAccelHpf);
        return false;
    }

    unsigned char LPF2_XL_EN = 0;
    unsigned char HP_SLOPE_XL_EN = 0;

    unsigned char accelHpf = m_settings->m_LSM6DS33LIS3MDLAccelHpf;

    // LPF2_XL_EN, HPCF_XL1, HPCF_XL0, 0, 0, HP_SLOPE_XL_EN, 0, LOW_PASS_ON_6D
    unsigned char ctrl8_xl = (LPF2_XL_EN << 7) | (accelHpf << 5) | (HP_SLOPE_XL_EN << 2) | (HP_SLOPE_XL_EN << 0) | 0x00;

    //std::cout << "Writing LSM6DS33 CTRL8_XL: " << static_cast<unsigned>(ctrl8_xl) << std::endl;

    if (!m_settings->HALWrite(m_gyroAccelSlaveAddr, LSM6DS33_CTRL8_XL, ctrl8_xl, "Failed to set LSM6DS33 CTRL8_XL"))
        return false;

    std::cout << "LSM6DS33 Accel set" << std::endl;
    return true;
}

void RTIMU::gyroBiasInit()
{
    m_gyroLearningAlpha = 2.0f / m_sampleRate;
    m_gyroContinuousAlpha = 0.01f / m_sampleRate;
    m_gyroSampleCount = 0;
}

bool RTIMU::IMURead()
{
    unsigned char status;
    unsigned char gyroData[6];
    unsigned char accelData[6];
    unsigned char compassData[6];

    // STATUS_REG
    // -, -, -, -, EV_BOOT, TDA, GDA, XLDA
    // EV_BOOT: Boot running flag signal. Default value: 0
    //          0: no boot running; 1: boot running
    // TDA:     Temperature new data available. Default value: 0
    //          0: no set of data is available at temperature sensor output;
    //          1: a new set of data is available at temperature sensor output
    // GDA:     Gyroscope new data available. Default value: 0
    //          0: no set of data available at gyroscope output
    //          1: a new set of data is available at gyroscope sensor output
    // XLDA:    Accelerometer new data available. Default value: 0
    //          0: no set of data available at accelerometer output
    //          1: a new set of data is available at accelerometer output
    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_STATUS_REG, 1, &status, "Failed to read LSM6DS33 status"))
        return false;
    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_OUTX_L_G, 6, gyroData, "Failed to read LSM6DS33 data"))
        return false;

    std::cout << "Gyro Data: "
              << "  X " << static_cast<int16_t>(gyroData[0] | gyroData[1] << 8) * m_gyroScale
              << "; Y " << static_cast<int16_t>(gyroData[2] | gyroData[3] << 8) * m_gyroScale
              << "; Z " << static_cast<int16_t>(gyroData[4] | gyroData[5] << 8) * m_gyroScale << std::endl;

    m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();

    if (!m_settings->HALRead(m_gyroAccelSlaveAddr, LSM6DS33_OUTX_L_XL, 6, accelData, "Failed to read LSM6DS33 accel data"))
        return false;

    std::cout << "Accel Data: "
              << "  X " << static_cast<int16_t>(accelData[0] | accelData[1] << 8)
              << "; Y " << static_cast<int16_t>(accelData[2] | accelData[3] << 8)
              << "; Z " << static_cast<int16_t>(accelData[4] | accelData[5] << 8) << std::endl;

    RTMath::convertToVector(gyroData, m_imuData.gyro, m_gyroScale, false);
    RTMath::convertToVector(accelData, m_imuData.accel, m_accelScale, false);
    RTMath::convertToVector(compassData, m_imuData.compass, m_compassScale, false);

    //  sort out gyro axes
    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(m_imuData.gyro.y());
    m_imuData.gyro.setZ(m_imuData.gyro.z());

    //  sort out accel data;
    m_imuData.accel.setX(m_imuData.accel.x());
    m_imuData.accel.setY(-m_imuData.accel.y());
    m_imuData.accel.setZ(-m_imuData.accel.z());

    //  sort out compass axes
    m_imuData.compass.setX(m_imuData.compass.x());
    m_imuData.compass.setY(-m_imuData.compass.y());
    m_imuData.compass.setZ(-m_imuData.compass.z());
    
    return true;
}
} // namespace drivers
} // namespace isaac
