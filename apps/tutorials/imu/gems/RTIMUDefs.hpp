//  IMU type codes
//
//  For compatibility, only add new codes at the end to avoid renumbering

#define RTIMU_TYPE_AUTODISCOVER             0                   // audodiscover the IMU
#define RTIMU_TYPE_NULL                     1                   // if no physical hardware
#define RTIMU_TYPE_MPU9150                  2                   // InvenSense MPU9150
#define RTIMU_TYPE_GD20HM303D               3                   // STM L3GD20H/LSM303D (Pololu Altimu)
#define RTIMU_TYPE_GD20M303DLHC             4                   // STM L3GD20/LSM303DHLC (old Adafruit IMU)
#define RTIMU_TYPE_LSM9DS0                  5                   // STM LSM9DS0 (eg Sparkfun IMU)
#define RTIMU_TYPE_LSM9DS1                  6                   // STM LSM9DS1
#define RTIMU_TYPE_MPU9250                  7                   // InvenSense MPU9250
#define RTIMU_TYPE_GD20HM303DLHC            8                   // STM L3GD20H/LSM303DHLC (new Adafruit IMU)
#define RTIMU_TYPE_BMX055                   9                   // Bosch BMX055
#define RTIMU_TYPE_BNO055                   10                  // Bosch BNO055
#define RTIMU_TYPE_LSM6DS33LIS3MDL          11                  // STM LSM6DS33/LIS3MDL (Pololu minimu v5)

//----------------------------------------------------------
//
//  LSM6DS33


//  I2C Slave Addresses

#define LSM6DS33_ADDRESS0         0x6a
#define LSM6DS33_ADDRESS1         0x6b
#define LSM6DS33_ID               0x69

//  LSM6DS33 Register map

#define LSM6DS33_FUNC_CFG_ACCESS  0x01

#define LSM6DS33_FIFO_CTRL1       0x06
#define LSM6DS33_FIFO_CTRL2       0x07
#define LSM6DS33_FIFO_CTRL3       0x08
#define LSM6DS33_FIFO_CTRL4       0x09
#define LSM6DS33_FIFO_CTRL5       0x0A
#define LSM6DS33_ORIENT_CFG_G     0x0B

#define LSM6DS33_INT1_CTRL        0x0D
#define LSM6DS33_INT2_CTRL        0x0E
#define LSM6DS33_WHO_AM_I         0x0F
#define LSM6DS33_CTRL1_XL         0x10
#define LSM6DS33_CTRL2_G          0x11
#define LSM6DS33_CTRL3_C          0x12
#define LSM6DS33_CTRL4_C          0x13
#define LSM6DS33_CTRL5_C          0x14
#define LSM6DS33_CTRL6_C          0x15
#define LSM6DS33_CTRL7_G          0x16
#define LSM6DS33_CTRL8_XL         0x17
#define LSM6DS33_CTRL9_XL         0x18
#define LSM6DS33_CTRL10_C         0x19

#define LSM6DS33_WAKE_UP_SRC      0x1B
#define LSM6DS33_TAP_SRC          0x1C
#define LSM6DS33_D6D_SRC          0x1D
#define LSM6DS33_STATUS_REG       0x1E

#define LSM6DS33_OUT_TEMP_L       0x20
#define LSM6DS33_OUT_TEMP_H       0x21
#define LSM6DS33_OUTX_L_G         0x22
#define LSM6DS33_OUTX_H_G         0x23
#define LSM6DS33_OUTY_L_G         0x24
#define LSM6DS33_OUTY_H_G         0x25
#define LSM6DS33_OUTZ_L_G         0x26
#define LSM6DS33_OUTZ_H_G         0x27
#define LSM6DS33_OUTX_L_XL        0x28
#define LSM6DS33_OUTX_H_XL        0x29
#define LSM6DS33_OUTY_L_XL        0x2A
#define LSM6DS33_OUTY_H_XL        0x2B
#define LSM6DS33_OUTZ_L_XL        0x2C
#define LSM6DS33_OUTZ_H_XL        0x2D

#define LSM6DS33_FIFO_STATUS1     0x3A
#define LSM6DS33_FIFO_STATUS2     0x3B
#define LSM6DS33_FIFO_STATUS3     0x3C
#define LSM6DS33_FIFO_STATUS4     0x3D
#define LSM6DS33_FIFO_DATA_OUT_L  0x3E
#define LSM6DS33_FIFO_DATA_OUT_H  0x3F
#define LSM6DS33_TIMESTAMP0_REG   0x40
#define LSM6DS33_TIMESTAMP1_REG   0x41
#define LSM6DS33_TIMESTAMP2_REG   0x42

#define LSM6DS33_STEP_TIMESTAMP_L 0x49
#define LSM6DS33_STEP_TIMESTAMP_H 0x4A
#define LSM6DS33_STEP_COUNTER_L   0x4B
#define LSM6DS33_STEP_COUNTER_H   0x4C

#define LSM6DS33_FUNC_SRC         0x53

#define LSM6DS33_TAP_CFG          0x58
#define LSM6DS33_TAP_THS_6D       0x59
#define LSM6DS33_INT_DUR2         0x5A
#define LSM6DS33_WAKE_UP_THS      0x5B
#define LSM6DS33_WAKE_UP_DUR      0x5C
#define LSM6DS33_FREE_FALL        0x5D
#define LSM6DS33_MD1_CFG          0x5E
#define LSM6DS33_MD2_CFG          0x5F

//  Gyro sample rate defines

#define LSM6DS33_SAMPLERATE_0     0
#define LSM6DS33_SAMPLERATE_13    1
#define LSM6DS33_SAMPLERATE_26    2
#define LSM6DS33_SAMPLERATE_52    3
#define LSM6DS33_SAMPLERATE_104   4
#define LSM6DS33_SAMPLERATE_208   5
#define LSM6DS33_SAMPLERATE_416   6
#define LSM6DS33_SAMPLERATE_833   7
#define LSM6DS33_SAMPLERATE_1660  8

//  Gyro FSR (Full scale Range) defines

#define LSM6DS33_FSR_125         0
#define LSM6DS33_FSR_245         1
#define LSM6DS33_FSR_500         2
#define LSM6DS33_FSR_1000        3
#define LSM6DS33_FSR_2000        4

//  Gyro high pass filter defines

#define LSM6DS33_HPF_0           0
#define LSM6DS33_HPF_1           1
#define LSM6DS33_HPF_2           2
#define LSM6DS33_HPF_3           3
#define LSM6DS33_HPF_4           4
#define LSM6DS33_HPF_5           5
#define LSM6DS33_HPF_6           6
#define LSM6DS33_HPF_7           7
#define LSM6DS33_HPF_8           8
#define LSM6DS33_HPF_9           9


//  Accel sample rate defines

#define LSM6DS33_ACCEL_SAMPLERATE_0     0
#define LSM6DS33_ACCEL_SAMPLERATE_13    1
#define LSM6DS33_ACCEL_SAMPLERATE_26    2
#define LSM6DS33_ACCEL_SAMPLERATE_52    3
#define LSM6DS33_ACCEL_SAMPLERATE_104   4
#define LSM6DS33_ACCEL_SAMPLERATE_208   5
#define LSM6DS33_ACCEL_SAMPLERATE_416   6
#define LSM6DS33_ACCEL_SAMPLERATE_833   7
#define LSM6DS33_ACCEL_SAMPLERATE_1660  8
#define LSM6DS33_ACCEL_SAMPLERATE_3330  9
#define LSM6DS33_ACCEL_SAMPLERATE_6660  10

//  Accel highpass defines

#define LSM6DS33_ACCEL_HPF_0     0
#define LSM6DS33_ACCEL_HPF_1     1
#define LSM6DS33_ACCEL_HPF_2     2
#define LSM6DS33_ACCEL_HPF_3     3

//  Accel lowpass filter bandwidth

#define LSM6DS33_ACCEL_LPF_400   0
#define LSM6DS33_ACCEL_LPF_200   1
#define LSM6DS33_ACCEL_LPF_100   2
#define LSM6DS33_ACCEL_LPF_50    3

//  Accel FSR

#define LSM6DS33_ACCEL_FSR_2     0
#define LSM6DS33_ACCEL_FSR_4     1
#define LSM6DS33_ACCEL_FSR_8     2
#define LSM6DS33_ACCEL_FSR_16    3


//----------------------------------------------------------
//
//  LIS3MDL

#define LIS3MDL_ADDRESS0        0x1c
#define LIS3MDL_ADDRESS1        0x1e
#define LIS3MDL_ID              0x3d

//  LIS3MDL Register Map

#define LIS3MDL_WHO_AM_I        0x0f

#define LIS3MDL_CTRL1           0x20
#define LIS3MDL_CTRL2           0x21
#define LIS3MDL_CTRL3           0x22
#define LIS3MDL_CTRL4           0x23
#define LIS3MDL_CTRL5           0x24

#define LIS3MDL_STATUS_REG      0x27

#define LIS3MDL_OUT_X_L         0x28
#define LIS3MDL_OUT_X_H         0x29
#define LIS3MDL_OUT_Y_L         0x2a
#define LIS3MDL_OUT_Y_H         0x2b
#define LIS3MDL_OUT_Z_L         0x2c
#define LIS3MDL_OUT_Z_H         0x2d

#define LIS3MDL_TEMP_OUT_L      0x2e
#define LIS3MDL_TEMP_OUT_H      0x2f

#define LIS3MDL_INT_CFG         0x30
#define LIS3MDL_INT_SRC         0x31
#define LIS3MDL_INT_THS_L       0x32
#define LIS3MDL_INT_THS_H       0x33


//  Compass sample rate defines

#define LIS3MDL_COMPASS_SAMPLERATE_0_625    0
#define LIS3MDL_COMPASS_SAMPLERATE_1_25     1
#define LIS3MDL_COMPASS_SAMPLERATE_2_5      2
#define LIS3MDL_COMPASS_SAMPLERATE_5        3
#define LIS3MDL_COMPASS_SAMPLERATE_10       4
#define LIS3MDL_COMPASS_SAMPLERATE_20       5
#define LIS3MDL_COMPASS_SAMPLERATE_40       6
#define LIS3MDL_COMPASS_SAMPLERATE_80       7

//  Compass FSR

#define LIS3MDL_COMPASS_FSR_4   0
#define LIS3MDL_COMPASS_FSR_8   1
#define LIS3MDL_COMPASS_FSR_12  2
#define LIS3MDL_COMPASS_FSR_16  3



