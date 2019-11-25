/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "RTIMUHal.hpp"
// #include "RTIMU.hpp"
#include <linux/spi/spidev.h>

#include <algorithm>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/gems/serialization/bytes_convert.hpp"
#include "engine/gems/serialization/crc16.hpp"

namespace isaac
{
namespace drivers
{

namespace
{
const std::set<std::string> kDefaultSensorReading = {
    "aux_batt_soc", "min_propulsion_batt_soc", "linear_accel_msp2",
    "linear_vel_mps", "differential_wheel_vel_rps", "right_front_vel_mps",
    "left_front_vel_mps"};
}

RTIMUHal::RTIMUHal()
{
    m_I2CBus = 255;
    m_currentSlave = 255;
    m_I2C = -1;
}

bool RTIMUHal::HALOpen()
{
    char buf[32];

    if (m_I2C >= 0)
        return true;

    m_I2CBus = 1;
    if (m_I2CBus == 255)
    {
        HAL_ERROR("No I2C bus has been set\n");
        return false;
    }
    sprintf(buf, "/dev/i2c-%d", m_I2CBus);
    m_I2C = open(buf, O_RDWR);
    if (m_I2C < 0)
    {
        HAL_ERROR1("Failed to open I2C bus %d\n", m_I2CBus);
        m_I2C = -1;
        return false;
    }

    HAL_INFO("HALOpened!!");
    return true;
}

void RTIMUHal::HALClose()
{
    I2CClose();
}

void RTIMUHal::I2CClose()
{
    if (m_I2C >= 0)
    {
        close(m_I2C);
        m_I2C = -1;
        m_currentSlave = 255;
    }
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                        unsigned char const data, const char *errorMsg)
{
    return HALWrite(slaveAddr, regAddr, 1, &data, errorMsg);
}

bool RTIMUHal::HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                        unsigned char length, unsigned char const *data, const char *errorMsg)
{
    int result;
    unsigned char txBuff[MAX_WRITE_LEN + 1];
    char *ifType;

    if (m_busIsI2C)
    {
        if (!I2CSelectSlave(slaveAddr, errorMsg))
            return false;
        ifType = (char *)"I2C";
    }
    else
    {
        ifType = (char *)"SPI";
    }

    if (length == 0)
    {
        result = ifWrite(&regAddr, 1);

        if (result < 0)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed - %s\n", ifType, errorMsg);
            return false;
        }
        else if (result != 1)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("%s write of regAddr failed (nothing written) - %s\n", ifType, errorMsg);
            return false;
        }
    }
    else
    {
        txBuff[0] = regAddr;
        memcpy(txBuff + 1, data, length);

        result = ifWrite(txBuff, length + 1);

        if (result < 0)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("%s data write of %d bytes failed - %s\n", ifType, length, errorMsg);
            return false;
        }
        else if (result < (int)length)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR4("%s data write of %d bytes failed, only %d written - %s\n", ifType, length, result, errorMsg);
            return false;
        }
    }
    return true;
}

bool RTIMUHal::ifWrite(unsigned char *data, unsigned char length)
{
    return write(m_I2C, data, length);
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                       unsigned char *data, const char *errorMsg)
{
    int tries, result, total;

    if (!HALWrite(slaveAddr, regAddr, 0, NULL, errorMsg))
        return false;

    total = 0;
    tries = 0;

    while ((total < length) && (tries < 5))
    {
        result = read(m_I2C, data + total, length - total);

        if (result < 0)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR3("I2C read error from %d, %d - %s\n", slaveAddr, regAddr, errorMsg);
            return false;
        }

        total += result;

        if (total == length)
            break;

        delayMs(10);
        tries++;
    }

    if (total < length)
    {
        if (strlen(errorMsg) > 0)
            HAL_ERROR3("I2C read from %d, %d failed - %s\n", slaveAddr, regAddr, errorMsg);
        return false;
    }
    return true;
}

bool RTIMUHal::HALRead(unsigned char slaveAddr, unsigned char length,
                       unsigned char *data, const char *errorMsg)
{
    int tries, result, total;
    if (!I2CSelectSlave(slaveAddr, errorMsg))
        return false;

    total = 0;
    tries = 0;

    while ((total < length) && (tries < 5))
    {
        result = read(m_I2C, data + total, length - total);

        if (result < 0)
        {
            if (strlen(errorMsg) > 0)
                HAL_ERROR2("I2C read error from %d - %s\n", slaveAddr, errorMsg);
            return false;
        }

        total += result;

        if (total == length)
            break;

        delayMs(10);
        tries++;
    }

    if (total < length)
    {
        if (strlen(errorMsg) > 0)
            HAL_ERROR2("I2C read from %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }
    return true;
}

bool RTIMUHal::I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg)
{
    if (m_currentSlave == slaveAddr)
        return true;

    if (!HALOpen())
    {
        HAL_ERROR1("Failed to open I2C port - %s\n", errorMsg);
        return false;
    }

    if (ioctl(m_I2C, I2C_SLAVE, slaveAddr) < 0)
    {
        HAL_ERROR2("I2C slave select %d failed - %s\n", slaveAddr, errorMsg);
        return false;
    }

    m_currentSlave = slaveAddr;

    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
    usleep(1000 * milliSeconds);
}

} // namespace drivers
} // namespace isaac
