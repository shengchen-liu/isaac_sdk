#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "engine/core/constants.hpp"
#include "engine/gems/coms/socket.hpp"

#include <unistd.h>
#include <fcntl.h>

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/time.h>

#define MAX_WRITE_LEN 255
#define MAX_READ_LEN 255



namespace isaac
{
namespace drivers
{

#define HAL_INFO(m)      \
    {                    \
        printf("%s", m); \
        fflush(stdout);  \
    }
#define HAL_INFO1(m, x) \
    {                   \
        printf(m, x);   \
        fflush(stdout); \
    }
#define HAL_INFO2(m, x, y) \
    {                      \
        printf(m, x, y);   \
        fflush(stdout);    \
    }
#define HAL_INFO3(m, x, y, z) \
    {                         \
        printf(m, x, y, z);   \
        fflush(stdout);       \
    }
#define HAL_INFO4(m, x, y, z, a) \
    {                            \
        printf(m, x, y, z, a);   \
        fflush(stdout);          \
    }
#define HAL_INFO5(m, x, y, z, a, b) \
    {                               \
        printf(m, x, y, z, a, b);   \
        fflush(stdout);             \
    }
#define HAL_ERROR(m) fprintf(stderr, m);
#define HAL_ERROR1(m, x) fprintf(stderr, m, x);
#define HAL_ERROR2(m, x, y) fprintf(stderr, m, x, y);
#define HAL_ERROR3(m, x, y, z) fprintf(stderr, m, x, y, z);
#define HAL_ERROR4(m, x, y, z, a) fprintf(stderr, m, x, y, z, a);

// The state of the Segway
struct IMUState
{
    // Linear acceleration derived from wheel velocities
    float linear_accel_msp2;
    // Linear velocity
    float linear_vel_mps;
    // Differential wheel speed (yaw rate) of the RMP derived using wheel velocities
    float differential_wheel_vel_rps;
    // Right front wheel velocity
    float right_front_vel_mps;
    // Left front wheel velocity
    float left_front_vel_mps;
    // The minimum of all propulsion battery states of charge (Percentage)
    float min_propulsion_batt_soc;
    // The auxiliary battery state of charge (Percentage)
    float aux_batt_soc;
};

// Segway RMP class that interface with the robot base over Ethernet
class RTIMUHal
{
public:
    // Instantiates a RTIMUHal class
    RTIMUHal();

    bool m_busIsI2C;           // true if I2C bus in use, false if SPI in use
    unsigned char m_I2CBus;    // I2C bus of the imu (eg 1 for Raspberry Pi usually)

    bool HALOpen();
    void HALClose();
    bool HALRead(unsigned char slaveAddr, unsigned char regAddr, unsigned char length,
                 unsigned char *data, const char *errorMsg);    // normal read with register select
    bool HALRead(unsigned char slaveAddr, unsigned char length,
                 unsigned char *data, const char *errorMsg);    // read without register select
    bool HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char length, unsigned char const *data, const char *errorMsg);
    bool HALWrite(unsigned char slaveAddr, unsigned char regAddr,
                  unsigned char const data, const char *errorMsg);

    void delayMs(int milliSeconds);

protected:
    void I2CClose();
    bool I2CSelectSlave(unsigned char slaveAddr, const char *errorMsg);
    bool ifWrite(unsigned char *data, unsigned char length);

private:
    int m_I2C;
    unsigned char m_currentSlave;
};

} // namespace drivers
} // namespace isaac
