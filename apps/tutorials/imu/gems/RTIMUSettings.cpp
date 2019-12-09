#include "RTIMUSettings.hpp"
#include "RTIMUDefs.hpp"
#include "RTIMULibDefs.hpp"
#include "RTIMU.hpp"

// #include "RTIMULSM6DS33LIS3MDL.hpp"

#include <iostream>

#define RATE_TIMER_INTERVAL 2

namespace isaac
{
namespace drivers
{
RTIMUSettings::RTIMUSettings(const char *productType)
{
    if ((strlen(productType) > 200) || (strlen(productType) == 0))
    {
        HAL_ERROR("Product name too long or null - using default\n");
        strcpy(m_filename, "RTIMULib.ini");
    }
    else
    {
        sprintf(m_filename, "%s.ini", productType);
    }
    loadSettings();
}

RTIMUSettings::RTIMUSettings(const char *settingsDirectory, const char *productType)
{
    if (((strlen(productType) + strlen(settingsDirectory)) > 200) || (strlen(productType) == 0))
    {
        HAL_ERROR("Product name too long or null - using default\n");
        strcpy(m_filename, "RTIMULib.ini");
    }
    else
    {
        sprintf(m_filename, "%s/%s.ini", settingsDirectory, productType);
    }
    loadSettings();
}

bool RTIMUSettings::discoverIMU(int &imuType, bool &busIsI2C, unsigned char &slaveAddress)
{
    unsigned char result;
    unsigned char altResult;

    //  auto detect on I2C bus

    m_busIsI2C = true;

    if (HALOpen())
    {
        if (HALRead(LSM6DS33_ADDRESS0, LSM6DS33_WHO_AM_I, 1, &result, ""))
        {
            if (result == LSM6DS33_ID)
            {
                imuType = RTIMU_TYPE_LSM6DS33LIS3MDL;
                slaveAddress = LSM6DS33_ADDRESS0;
                busIsI2C = true;
                HAL_INFO("Detected LSM6DS33 at standard/standard address\n");
                return true;
            }
        }

        if (HALRead(LSM6DS33_ADDRESS1, LSM6DS33_WHO_AM_I, 1, &result, ""))
        {
            if (result == LSM6DS33_ID)
            {
                if (HALRead(LIS3MDL_ADDRESS1, LIS3MDL_WHO_AM_I, 1, &altResult, ""))
                {
                    if (altResult == LIS3MDL_ID)
                    {
                        imuType = RTIMU_TYPE_LSM6DS33LIS3MDL;
                        slaveAddress = LSM6DS33_ADDRESS1;
                        busIsI2C = true;
                        HAL_INFO("Detected LSM6DS33/LIS3MDL at option/option address\n");
                        std::cout << "FOUND IMU LSM6DS33 and Result was " << static_cast<unsigned>(result) << " alt result was " << static_cast<unsigned>(altResult) << std::endl;
                        return true;
                    }
                }
                if (HALRead(LIS3MDL_ADDRESS0, LIS3MDL_WHO_AM_I, 1, &altResult, ""))
                {
                    if (altResult == LIS3MDL_ID)
                    {
                        imuType = RTIMU_TYPE_LSM6DS33LIS3MDL;
                        slaveAddress = LSM6DS33_ADDRESS1;
                        busIsI2C = true;
                        HAL_INFO("Detected LSM6DS33/LIS3MDL at option/standard address\n");
                        std::cout << "FOUND IMU LIS3MDL and Result was " << static_cast<unsigned>(altResult) << std::endl;
                        return true;
                    }
                }
            }
        }
        HALClose();
    }

    HAL_ERROR("No IMU detected\n");
    return false;
}


void RTIMUSettings::setDefaults()
{
    //  preset general defaults

    m_imuType = RTIMU_TYPE_AUTODISCOVER;
    m_I2CSlaveAddress = 0;
    m_busIsI2C = true;
    m_I2CBus = 1;
    m_fusionType = RTFUSION_TYPE_KALMANSTATE4;
    m_axisRotation = RTIMU_XNORTH_YEAST;
    m_compassCalValid = false;
    m_compassCalEllipsoidValid = false;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            m_compassCalEllipsoidCorr[i][j] = 0;
        }
    }
    m_compassCalEllipsoidCorr[0][0] = 1;
    m_compassCalEllipsoidCorr[1][1] = 1;
    m_compassCalEllipsoidCorr[2][2] = 1;

    m_compassAdjDeclination = 0;

    m_accelCalValid = false;
    m_gyroBiasValid = false;

    //  LSM6DS33LIS3MDL defaults

    m_LSM6DS33LIS3MDLGyroSampleRate = LSM6DS33_SAMPLERATE_1660;
    m_LSM6DS33LIS3MDLGyroHpf = LSM6DS33_HPF_0;
    m_LSM6DS33LIS3MDLGyroFsr = LSM6DS33_FSR_2000;

    m_LSM6DS33LIS3MDLAccelSampleRate = LSM6DS33_ACCEL_SAMPLERATE_6660;
    m_LSM6DS33LIS3MDLAccelHpf = LSM6DS33_ACCEL_HPF_0;
    m_LSM6DS33LIS3MDLAccelFsr = LSM6DS33_ACCEL_FSR_16;
    m_LSM6DS33LIS3MDLAccelLpf = LSM6DS33_ACCEL_LPF_50;

    m_LSM6DS33LIS3MDLCompassSampleRate = LIS3MDL_COMPASS_SAMPLERATE_80;
    m_LSM6DS33LIS3MDLCompassFsr = LIS3MDL_COMPASS_FSR_4;
}

bool RTIMUSettings::loadSettings()
{
    setDefaults();

    char buf[200];
    char key[200];
    char val[200];
    RTFLOAT ftemp;
    //  check to see if settings file exists

    if (!(m_fd = fopen(m_filename, "r")))
    {
        HAL_INFO("Settings file not found. Using defaults and creating settings file\n");
        return saveSettings();
    }

    while (fgets(buf, 200, m_fd))
    {
        if ((buf[0] == '#') || (buf[0] == ' ') || (buf[0] == '\n'))
            // just a comment
            continue;

        if (sscanf(buf, "%[^=]=%s", key, val) != 2)
        {
            HAL_ERROR1("Bad line in settings file: %s\n", buf);
            fclose(m_fd);
            return false;
        }

        //  now decode keys

        //  general config

        if (strcmp(key, RTIMULIB_IMU_TYPE) == 0)
        {
            m_imuType = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_FUSION_TYPE) == 0)
        {
            m_fusionType = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_BUS_IS_I2C) == 0)
        {
            m_busIsI2C = strcmp(val, "true") == 0;
        }
        else if (strcmp(key, RTIMULIB_I2C_BUS) == 0)
        {
            m_I2CBus = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_I2C_SLAVEADDRESS) == 0)
        {
            m_I2CSlaveAddress = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_AXIS_ROTATION) == 0)
        {
            m_axisRotation = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_VALID) == 0)
        {
            m_compassCalValid = strcmp(val, "true") == 0;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MINX) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MINY) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MINZ) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMin.setZ(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXX) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXY) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_MAXZ) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalMax.setZ(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSADJ_DECLINATION) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassAdjDeclination = ftemp;

            // compass ellipsoid calibration
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_ELLIPSOID_VALID) == 0)
        {
            m_compassCalEllipsoidValid = strcmp(val, "true") == 0;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_OFFSET_X) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidOffset.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_OFFSET_Y) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidOffset.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_OFFSET_Z) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidOffset.setZ(ftemp);
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR11) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[0][0] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR12) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[0][1] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR13) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[0][2] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR21) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[1][0] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR22) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[1][1] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR23) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[1][2] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR31) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[2][0] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR32) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[2][1] = ftemp;
        }
        else if (strcmp(key, RTIMULIB_COMPASSCAL_CORR33) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_compassCalEllipsoidCorr[2][2] = ftemp;

            // accel calibration
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_VALID) == 0)
        {
            m_accelCalValid = strcmp(val, "true") == 0;
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MINX) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MINY) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MINZ) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMin.setZ(ftemp);
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MAXX) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MAXY) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_ACCELCAL_MAXZ) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_accelCalMax.setZ(ftemp);

            // gyro bias
        }
        else if (strcmp(key, RTIMULIB_GYRO_BIAS_VALID) == 0)
        {
            m_gyroBiasValid = strcmp(val, "true") == 0;
        }
        else if (strcmp(key, RTIMULIB_GYRO_BIAS_X) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setX(ftemp);
        }
        else if (strcmp(key, RTIMULIB_GYRO_BIAS_Y) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setY(ftemp);
        }
        else if (strcmp(key, RTIMULIB_GYRO_BIAS_Z) == 0)
        {
            sscanf(val, "%f", &ftemp);
            m_gyroBias.setZ(ftemp);
            //  LSM6DS33LIS3MDL settings
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_GYRO_SAMPLERATE) == 0)
        {
            m_LSM6DS33LIS3MDLGyroSampleRate = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_GYRO_FSR) == 0)
        {
            m_LSM6DS33LIS3MDLGyroFsr = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_GYRO_HPF) == 0)
        {
            m_LSM6DS33LIS3MDLGyroHpf = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_ACCEL_HPF) == 0)
        {
            m_LSM6DS33LIS3MDLAccelHpf = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_ACCEL_SAMPLERATE) == 0)
        {
            m_LSM6DS33LIS3MDLAccelSampleRate = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_ACCEL_FSR) == 0)
        {
            m_LSM6DS33LIS3MDLAccelFsr = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_ACCEL_LPF) == 0)
        {
            m_LSM6DS33LIS3MDLAccelLpf = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_COMPASS_SAMPLERATE) == 0)
        {
            m_LSM6DS33LIS3MDLCompassSampleRate = atoi(val);
        }
        else if (strcmp(key, RTIMULIB_LSM6DS33LIS3MDL_COMPASS_FSR) == 0)
        {
            m_LSM6DS33LIS3MDLCompassFsr = atoi(val);
            //  Handle unrecognized key
        }
        else
        {
            HAL_ERROR1("Unrecognized key in settings file: %s\n", buf);
        }
    }
    HAL_INFO1("Settings file %s loaded\n", m_filename);
    fclose(m_fd);
    return saveSettings(); // make sure settings file is correct and complete
}

bool RTIMUSettings::saveSettings()
{
    if (!(m_fd = fopen(m_filename, "w")))
    {
        HAL_ERROR("Failed to open settings file for save");
        return false;
    }

    //  General settings

    setComment("#####################################################################");
    setComment("");
    setComment("RTIMULib settings file");
    setBlank();
    setComment("General settings");
    setComment("");

    setBlank();
    setComment("IMU type - ");
    setComment("  0 = Auto discover");
    setComment("  1 = Null (used when data is provided from a remote IMU");
    setComment("  2 = InvenSense MPU-9150");
    setComment("  3 = STM L3GD20H + LSM303D");
    setComment("  4 = STM L3GD20 + LSM303DLHC");
    setComment("  5 = STM LSM9DS0");
    setComment("  6 = STM LSM9DS1");
    setComment("  7 = InvenSense MPU-9250");
    setComment("  8 = STM L3GD20H + LSM303DLHC");
    setComment("  9 = Bosch BMX055");
    setComment("  10 = Bosch BNX055");
    setComment("  11 = STM LSM6DS33 + LIS3MDL");
    setValue(RTIMULIB_IMU_TYPE, m_imuType);

    setBlank();
    setComment("");
    setComment("Fusion type type - ");
    setComment("  0 - Null. Use if only sensor data required without fusion");
    setComment("  1 - Kalman STATE4");
    setComment("  2 - RTQF");
    setValue(RTIMULIB_FUSION_TYPE, m_fusionType);

    setBlank();
    setComment("");
    setComment("Is bus I2C: 'true' for I2C, 'false' for SPI");
    setValue(RTIMULIB_BUS_IS_I2C, m_busIsI2C);

    setBlank();
    setComment("");
    setComment("I2C Bus (between 0 and 7) ");
    setValue(RTIMULIB_I2C_BUS, m_I2CBus);
    
    setBlank();
    setComment("");
    setComment("I2C slave address (filled in automatically by auto discover) ");
    setValue(RTIMULIB_I2C_SLAVEADDRESS, m_I2CSlaveAddress);

    setBlank();
    setComment("");
    setComment("IMU axis rotation - see RTIMU.h for details");
    setValue(RTIMULIB_AXIS_ROTATION, m_axisRotation);

    //  Compass settings

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
    setComment("Compass calibration settings");
    setValue(RTIMULIB_COMPASSCAL_VALID, m_compassCalValid);
    setValue(RTIMULIB_COMPASSCAL_MINX, m_compassCalMin.x());
    setValue(RTIMULIB_COMPASSCAL_MINY, m_compassCalMin.y());
    setValue(RTIMULIB_COMPASSCAL_MINZ, m_compassCalMin.z());
    setValue(RTIMULIB_COMPASSCAL_MAXX, m_compassCalMax.x());
    setValue(RTIMULIB_COMPASSCAL_MAXY, m_compassCalMax.y());
    setValue(RTIMULIB_COMPASSCAL_MAXZ, m_compassCalMax.z());

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
    setComment("Compass adjustment settings");
    setComment("Compass declination is in radians and is subtracted from calculated heading");
    setValue(RTIMULIB_COMPASSADJ_DECLINATION, m_compassAdjDeclination);

    //  Compass ellipsoid calibration settings

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
    setComment("Compass ellipsoid calibration");
    setValue(RTIMULIB_COMPASSCAL_ELLIPSOID_VALID, m_compassCalEllipsoidValid);
    setValue(RTIMULIB_COMPASSCAL_OFFSET_X, m_compassCalEllipsoidOffset.x());
    setValue(RTIMULIB_COMPASSCAL_OFFSET_Y, m_compassCalEllipsoidOffset.y());
    setValue(RTIMULIB_COMPASSCAL_OFFSET_Z, m_compassCalEllipsoidOffset.z());
    setValue(RTIMULIB_COMPASSCAL_CORR11, m_compassCalEllipsoidCorr[0][0]);
    setValue(RTIMULIB_COMPASSCAL_CORR12, m_compassCalEllipsoidCorr[0][1]);
    setValue(RTIMULIB_COMPASSCAL_CORR13, m_compassCalEllipsoidCorr[0][2]);
    setValue(RTIMULIB_COMPASSCAL_CORR21, m_compassCalEllipsoidCorr[1][0]);
    setValue(RTIMULIB_COMPASSCAL_CORR22, m_compassCalEllipsoidCorr[1][1]);
    setValue(RTIMULIB_COMPASSCAL_CORR23, m_compassCalEllipsoidCorr[1][2]);
    setValue(RTIMULIB_COMPASSCAL_CORR31, m_compassCalEllipsoidCorr[2][0]);
    setValue(RTIMULIB_COMPASSCAL_CORR32, m_compassCalEllipsoidCorr[2][1]);
    setValue(RTIMULIB_COMPASSCAL_CORR33, m_compassCalEllipsoidCorr[2][2]);

    //  Accel calibration settings

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
    setComment("Accel calibration");
    setValue(RTIMULIB_ACCELCAL_VALID, m_accelCalValid);
    setValue(RTIMULIB_ACCELCAL_MINX, m_accelCalMin.x());
    setValue(RTIMULIB_ACCELCAL_MINY, m_accelCalMin.y());
    setValue(RTIMULIB_ACCELCAL_MINZ, m_accelCalMin.z());
    setValue(RTIMULIB_ACCELCAL_MAXX, m_accelCalMax.x());
    setValue(RTIMULIB_ACCELCAL_MAXY, m_accelCalMax.y());
    setValue(RTIMULIB_ACCELCAL_MAXZ, m_accelCalMax.z());

    //  Gyro bias settings

    setBlank();
    setComment("#####################################################################");
    setComment("");

    setBlank();
    setComment("Saved gyro bias data");
    setValue(RTIMULIB_GYRO_BIAS_VALID, m_gyroBiasValid);
    setValue(RTIMULIB_GYRO_BIAS_X, m_gyroBias.x());
    setValue(RTIMULIB_GYRO_BIAS_Y, m_gyroBias.y());
    setValue(RTIMULIB_GYRO_BIAS_Z, m_gyroBias.z());

    //  LSM6DS33 LIS3MDL settings

    setBlank();
    setComment("#####################################################################");
    setComment("");
    setComment("LSM6DS33 + LIS3MDL settings");

    setBlank();
    setComment("");
    setComment("Gyro sample rate - ");
    setComment("  0 = 0Hz ");
    setComment("  1 = 13Hz ");
    setComment("  2 = 26Hz ");
    setComment("  3 = 52Hz ");
    setComment("  4 = 104Hz ");
    setComment("  5 = 208Hz ");
    setComment("  6 = 416Hz ");
    setComment("  7 = 833Hz ");
    setComment("  8 = 1660Hz ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_GYRO_SAMPLERATE, m_LSM6DS33LIS3MDLGyroSampleRate);

    setBlank();
    setComment("");
    setComment("Gyro full scale range - ");
    setComment("  0 = 125 degrees per second ");
    setComment("  1 = 245 degrees per second ");
    setComment("  2 = 500 degrees per second ");
    setComment("  3 = 1000 degrees per second ");
    setComment("  4 = 2000 degrees per second ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_GYRO_FSR, m_LSM6DS33LIS3MDLGyroFsr);

    setBlank();
    setComment("");
    setComment("Gyro high pass filter - ");
    setComment("  0 - 9 but see the LSM6DS33 manual for details");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_GYRO_HPF, m_LSM6DS33LIS3MDLGyroHpf);

    setBlank();
    setComment("Accel sample rate - ");
    setComment("  0 = 0Hz ");
    setComment("  1 = 13Hz ");
    setComment("  2 = 26Hz ");
    setComment("  3 = 52Hz ");
    setComment("  4 = 104Hz ");
    setComment("  5 = 208Hz ");
    setComment("  6 = 416Hz ");
    setComment("  7 = 833Hz ");
    setComment("  8 = 1660Hz ");
    setComment("  9 = 3330Hz ");
    setComment("  10 = 6660Hz ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_ACCEL_SAMPLERATE, m_LSM6DS33LIS3MDLAccelSampleRate);

    setBlank();
    setComment("");
    setComment("Accel full scale range - ");
    setComment("  0 = +/- 2g ");
    setComment("  1 = +/- 4g ");
    setComment("  2 = +/- 8g ");
    setComment("  3 = +/- 16g ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_ACCEL_FSR, m_LSM6DS33LIS3MDLAccelFsr);

    setBlank();
    setComment("");
    setComment("Accel high pass filter - ");
    setComment("  0 = ODR_XL/50Hz");
    setComment("  1 = ODR_XL/100Hz");
    setComment("  2 = ODR_XL/9Hz");
    setComment("  3 = ODR_XL/400Hz");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_ACCEL_HPF, m_LSM6DS33LIS3MDLAccelHpf);

    setBlank();
    setComment("");
    setComment("Accel low pass filter - ");
    setComment("  0 = 400Hz");
    setComment("  1 = 200Hz");
    setComment("  2 = 100Hz");
    setComment("  3 = 50Hz");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_ACCEL_LPF, m_LSM6DS33LIS3MDLAccelLpf);

    setBlank();
    setComment("");
    setComment("Compass sample rate - ");
    setComment("  0 = 3.125Hz ");
    setComment("  1 = 6.25Hz ");
    setComment("  2 = 12.5Hz ");
    setComment("  3 = 25Hz ");
    setComment("  4 = 50Hz ");
    setComment("  5 = 100Hz ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_COMPASS_SAMPLERATE, m_LSM6DS33LIS3MDLCompassSampleRate);

    setBlank();
    setComment("");
    setComment("Compass full scale range - ");
    setComment("  0 = +/- 200 uT ");
    setComment("  1 = +/- 400 uT ");
    setComment("  2 = +/- 800 uT ");
    setComment("  3 = +/- 1200 uT ");
    setValue(RTIMULIB_LSM6DS33LIS3MDL_COMPASS_FSR, m_LSM6DS33LIS3MDLCompassFsr);

    fclose(m_fd);
    return true;
}

void RTIMUSettings::setBlank()
{
    fprintf(m_fd, "\n");
}

void RTIMUSettings::setComment(const char *comment)
{
    fprintf(m_fd, "# %s\n", comment);
}

void RTIMUSettings::setValue(const char *key, const bool val)
{
    fprintf(m_fd, "%s=%s\n", key, val ? "true" : "false");
}

void RTIMUSettings::setValue(const char *key, const int val)
{
    fprintf(m_fd, "%s=%d\n", key, val);
}

void RTIMUSettings::setValue(const char *key, const RTFLOAT val)
{
    fprintf(m_fd, "%s=%f\n", key, val);
}

} // namespace drivers
} // namespace isaac

// // // //----------------------------------------------------------
// // //
// //  The RTVector3 class
// namespace isaac
// {
// namespace drivers
// {
// RTVector3::RTVector3()
// {
//     zero();
// }

// RTVector3::RTVector3(RTFLOAT x, RTFLOAT y, RTFLOAT z)
// {
//     m_data[0] = x;
//     m_data[1] = y;
//     m_data[2] = z;
// }

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

// void RTVector3::zero()
// {
//     for (int i = 0; i < 3; i++)
//         m_data[i] = 0;
// }

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

// // void RTVector3::accelToQuaternion(RTQuaternion &qPose) const
// // {
// //     RTVector3 normAccel = *this;
// //     RTVector3 vec;
// //     RTVector3 z(0, 0, 1.0);

// //     normAccel.normalize();

// //     RTFLOAT angle = acos(RTVector3::dotProduct(z, normAccel));
// //     RTVector3::crossProduct(normAccel, z, vec);
// //     vec.normalize();

// //     qPose.fromAngleVector(angle, vec);
// // }

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


// // //----------------------------------------------------------
// // //
// // //  The RTQuaternion class

// // RTQuaternion::RTQuaternion()
// // {
// //     zero();
// // }

// // RTQuaternion::RTQuaternion(RTFLOAT scalar, RTFLOAT x, RTFLOAT y, RTFLOAT z)
// // {
// //     m_data[0] = scalar;
// //     m_data[1] = x;
// //     m_data[2] = y;
// //     m_data[3] = z;
// // }

// // RTQuaternion &RTQuaternion::operator=(const RTQuaternion &quat)
// // {
// //     if (this == &quat)
// //         return *this;

// //     m_data[0] = quat.m_data[0];
// //     m_data[1] = quat.m_data[1];
// //     m_data[2] = quat.m_data[2];
// //     m_data[3] = quat.m_data[3];

// //     return *this;
// // }

// // RTQuaternion &RTQuaternion::operator+=(const RTQuaternion &quat)
// // {
// //     for (int i = 0; i < 4; i++)
// //         m_data[i] += quat.m_data[i];
// //     return *this;
// // }

// // RTQuaternion &RTQuaternion::operator-=(const RTQuaternion &quat)
// // {
// //     for (int i = 0; i < 4; i++)
// //         m_data[i] -= quat.m_data[i];
// //     return *this;
// // }

// // RTQuaternion &RTQuaternion::operator-=(const RTFLOAT val)
// // {
// //     for (int i = 0; i < 4; i++)
// //         m_data[i] -= val;
// //     return *this;
// // }

// // RTQuaternion &RTQuaternion::operator*=(const RTQuaternion &qb)
// // {
// //     RTQuaternion qa;

// //     qa = *this;

// //     m_data[0] = qa.scalar() * qb.scalar() - qa.x() * qb.x() - qa.y() * qb.y() - qa.z() * qb.z();
// //     m_data[1] = qa.scalar() * qb.x() + qa.x() * qb.scalar() + qa.y() * qb.z() - qa.z() * qb.y();
// //     m_data[2] = qa.scalar() * qb.y() - qa.x() * qb.z() + qa.y() * qb.scalar() + qa.z() * qb.x();
// //     m_data[3] = qa.scalar() * qb.z() + qa.x() * qb.y() - qa.y() * qb.x() + qa.z() * qb.scalar();

// //     return *this;
// // }

// // RTQuaternion &RTQuaternion::operator*=(const RTFLOAT val)
// // {
// //     m_data[0] *= val;
// //     m_data[1] *= val;
// //     m_data[2] *= val;
// //     m_data[3] *= val;

// //     return *this;
// // }

// // const RTQuaternion RTQuaternion::operator*(const RTQuaternion &qb) const
// // {
// //     RTQuaternion result = *this;
// //     result *= qb;
// //     return result;
// // }

// // const RTQuaternion RTQuaternion::operator*(const RTFLOAT val) const
// // {
// //     RTQuaternion result = *this;
// //     result *= val;
// //     return result;
// // }

// // const RTQuaternion RTQuaternion::operator-(const RTQuaternion &qb) const
// // {
// //     RTQuaternion result = *this;
// //     result -= qb;
// //     return result;
// // }

// // const RTQuaternion RTQuaternion::operator-(const RTFLOAT val) const
// // {
// //     RTQuaternion result = *this;
// //     result -= val;
// //     return result;
// // }

// // void RTQuaternion::zero()
// // {
// //     for (int i = 0; i < 4; i++)
// //         m_data[i] = 0;
// // }

// // void RTQuaternion::normalize()
// // {
// //     RTFLOAT length = sqrt(m_data[0] * m_data[0] + m_data[1] * m_data[1] +
// //                           m_data[2] * m_data[2] + m_data[3] * m_data[3]);

// //     if ((length == 0) || (length == 1))
// //         return;

// //     m_data[0] /= length;
// //     m_data[1] /= length;
// //     m_data[2] /= length;
// //     m_data[3] /= length;
// // }

// // void RTQuaternion::toEuler(RTVector3 &vec)
// // {
// //     vec.setX(atan2(2.0 * (m_data[2] * m_data[3] + m_data[0] * m_data[1]),
// //                    1 - 2.0 * (m_data[1] * m_data[1] + m_data[2] * m_data[2])));

// //     vec.setY(asin(2.0 * (m_data[0] * m_data[2] - m_data[1] * m_data[3])));

// //     vec.setZ(atan2(2.0 * (m_data[1] * m_data[2] + m_data[0] * m_data[3]),
// //                    1 - 2.0 * (m_data[2] * m_data[2] + m_data[3] * m_data[3])));
// // }

// // void RTQuaternion::fromEuler(RTVector3 &vec)
// // {
// //     RTFLOAT cosX2 = cos(vec.x() / 2.0f);
// //     RTFLOAT sinX2 = sin(vec.x() / 2.0f);
// //     RTFLOAT cosY2 = cos(vec.y() / 2.0f);
// //     RTFLOAT sinY2 = sin(vec.y() / 2.0f);
// //     RTFLOAT cosZ2 = cos(vec.z() / 2.0f);
// //     RTFLOAT sinZ2 = sin(vec.z() / 2.0f);

// //     m_data[0] = cosX2 * cosY2 * cosZ2 + sinX2 * sinY2 * sinZ2;
// //     m_data[1] = sinX2 * cosY2 * cosZ2 - cosX2 * sinY2 * sinZ2;
// //     m_data[2] = cosX2 * sinY2 * cosZ2 + sinX2 * cosY2 * sinZ2;
// //     m_data[3] = cosX2 * cosY2 * sinZ2 - sinX2 * sinY2 * cosZ2;
// //     normalize();
// // }

// // RTQuaternion RTQuaternion::conjugate() const
// // {
// //     RTQuaternion q;
// //     q.setScalar(m_data[0]);
// //     q.setX(-m_data[1]);
// //     q.setY(-m_data[2]);
// //     q.setZ(-m_data[3]);
// //     return q;
// // }

// // void RTQuaternion::toAngleVector(RTFLOAT &angle, RTVector3 &vec)
// // {
// //     RTFLOAT halfTheta;
// //     RTFLOAT sinHalfTheta;

// //     halfTheta = acos(m_data[0]);
// //     sinHalfTheta = sin(halfTheta);

// //     if (sinHalfTheta == 0)
// //     {
// //         vec.setX(1.0);
// //         vec.setY(0);
// //         vec.setZ(0);
// //     }
// //     else
// //     {
// //         vec.setX(m_data[1] / sinHalfTheta);
// //         vec.setY(m_data[1] / sinHalfTheta);
// //         vec.setZ(m_data[1] / sinHalfTheta);
// //     }
// //     angle = 2.0 * halfTheta;
// // }

// // void RTQuaternion::fromAngleVector(const RTFLOAT &angle, const RTVector3 &vec)
// // {
// //     RTFLOAT sinHalfTheta = sin(angle / 2.0);
// //     m_data[0] = cos(angle / 2.0);
// //     m_data[1] = vec.x() * sinHalfTheta;
// //     m_data[2] = vec.y() * sinHalfTheta;
// //     m_data[3] = vec.z() * sinHalfTheta;
// // }

// } // namespace drivers
// } // namespace isaac