/********************************************************************************
 *   File Name:
 *       lsm9ds0_types.hpp
 *
 *   Description:
 *       Implements the driver for the LSM9DS0
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef LSM9DS0_TYPES_HPP
#define LSM9DS0_TYPES_HPP

/* C++ Includes */
#include <cstdint>

namespace LSM9DS0
{
  static constexpr uint8_t LSM_READ_BIT      = 1u << 7;
  static constexpr uint8_t LSM_AUTO_INCR_BIT = 1u << 6;

  static constexpr size_t MAX_SPI_CLOCK  = 10000000;
  static constexpr size_t FAST_SPI_CLOCK = 8000000;
  static constexpr size_t MED_SPI_CLOCK  = 4000000;
  static constexpr size_t SLOW_SPI_CLOCK = 1000000;

  /*------------------------------------------------
  LSM9DS0 Gyro Registers
  ------------------------------------------------*/
  static constexpr uint8_t WHO_AM_I_G      = 0x0F;
  static constexpr uint8_t CTRL_REG1_G     = 0x20;
  static constexpr uint8_t CTRL_REG2_G     = 0x21;
  static constexpr uint8_t CTRL_REG3_G     = 0x22;
  static constexpr uint8_t CTRL_REG4_G     = 0x23;
  static constexpr uint8_t CTRL_REG5_G     = 0x24;
  static constexpr uint8_t REFERENCE_G     = 0x25;
  static constexpr uint8_t STATUS_REG_G    = 0x27;
  static constexpr uint8_t OUT_X_L_G       = 0x28;
  static constexpr uint8_t OUT_X_H_G       = 0x29;
  static constexpr uint8_t OUT_Y_L_G       = 0x2A;
  static constexpr uint8_t OUT_Y_H_G       = 0x2B;
  static constexpr uint8_t OUT_Z_L_G       = 0x2C;
  static constexpr uint8_t OUT_Z_H_G       = 0x2D;
  static constexpr uint8_t FIFO_CTRL_REG_G = 0x2E;
  static constexpr uint8_t FIFO_SRC_REG_G  = 0x2F;
  static constexpr uint8_t INT1_CFG_G      = 0x30;
  static constexpr uint8_t INT1_SRC_G      = 0x31;
  static constexpr uint8_t INT1_THS_XH_G   = 0x32;
  static constexpr uint8_t INT1_THS_XL_G   = 0x33;
  static constexpr uint8_t INT1_THS_YH_G   = 0x34;
  static constexpr uint8_t INT1_THS_YL_G   = 0x35;
  static constexpr uint8_t INT1_THS_ZH_G   = 0x36;
  static constexpr uint8_t INT1_THS_ZL_G   = 0x37;
  static constexpr uint8_t INT1_DURATION_G = 0x38;

  /*------------------------------------------------
  LSM9DS0 Accelerometer/Magnetometer (XM) Registers
  ------------------------------------------------*/
  static constexpr uint8_t OUT_TEMP_L_XM      = 0x05;
  static constexpr uint8_t OUT_TEMP_H_XM      = 0x06;
  static constexpr uint8_t STATUS_REG_M       = 0x07;
  static constexpr uint8_t OUT_X_L_M          = 0x08;
  static constexpr uint8_t OUT_X_H_M          = 0x09;
  static constexpr uint8_t OUT_Y_L_M          = 0x0A;
  static constexpr uint8_t OUT_Y_H_M          = 0x0B;
  static constexpr uint8_t OUT_Z_L_M          = 0x0C;
  static constexpr uint8_t OUT_Z_H_M          = 0x0D;
  static constexpr uint8_t WHO_AM_I_XM        = 0x0F;
  static constexpr uint8_t INT_CTRL_REG_M     = 0x12;
  static constexpr uint8_t INT_SRC_REG_M      = 0x13;
  static constexpr uint8_t INT_THS_L_M        = 0x14;
  static constexpr uint8_t INT_THS_H_M        = 0x15;
  static constexpr uint8_t OFFSET_X_L_M       = 0x16;
  static constexpr uint8_t OFFSET_X_H_M       = 0x17;
  static constexpr uint8_t OFFSET_Y_L_M       = 0x18;
  static constexpr uint8_t OFFSET_Y_H_M       = 0x19;
  static constexpr uint8_t OFFSET_Z_L_M       = 0x1A;
  static constexpr uint8_t OFFSET_Z_H_M       = 0x1B;
  static constexpr uint8_t REFERENCE_X        = 0x1C;
  static constexpr uint8_t REFERENCE_Y        = 0x1D;
  static constexpr uint8_t REFERENCE_Z        = 0x1E;
  static constexpr uint8_t CTRL_REG0_XM       = 0x1F;
  static constexpr uint8_t CTRL_REG1_XM       = 0x20;
  static constexpr uint8_t CTRL_REG2_XM       = 0x21;
  static constexpr uint8_t CTRL_REG3_XM       = 0x22;
  static constexpr uint8_t CTRL_REG4_XM       = 0x23;
  static constexpr uint8_t CTRL_REG5_XM       = 0x24;
  static constexpr uint8_t CTRL_REG6_XM       = 0x25;
  static constexpr uint8_t CTRL_REG7_XM       = 0x26;
  static constexpr uint8_t STATUS_REG_A       = 0x27;
  static constexpr uint8_t OUT_X_L_A          = 0x28;
  static constexpr uint8_t OUT_X_H_A          = 0x29;
  static constexpr uint8_t OUT_Y_L_A          = 0x2A;
  static constexpr uint8_t OUT_Y_H_A          = 0x2B;
  static constexpr uint8_t OUT_Z_L_A          = 0x2C;
  static constexpr uint8_t OUT_Z_H_A          = 0x2D;
  static constexpr uint8_t FIFO_CTRL_REG      = 0x2E;
  static constexpr uint8_t FIFO_SRC_REG       = 0x2F;
  static constexpr uint8_t INT_GEN_1_REG      = 0x30;
  static constexpr uint8_t INT_GEN_1_SRC      = 0x31;
  static constexpr uint8_t INT_GEN_1_THS      = 0x32;
  static constexpr uint8_t INT_GEN_1_DURATION = 0x33;
  static constexpr uint8_t INT_GEN_2_REG      = 0x34;
  static constexpr uint8_t INT_GEN_2_SRC      = 0x35;
  static constexpr uint8_t INT_GEN_2_THS      = 0x36;
  static constexpr uint8_t INT_GEN_2_DURATION = 0x37;
  static constexpr uint8_t CLICK_CFG          = 0x38;
  static constexpr uint8_t CLICK_SRC          = 0x39;
  static constexpr uint8_t CLICK_THS          = 0x3A;
  static constexpr uint8_t TIME_LIMIT         = 0x3B;
  static constexpr uint8_t TIME_LATENCY       = 0x3C;
  static constexpr uint8_t TIME_WINDOW        = 0x3D;
  static constexpr uint8_t ACT_THS            = 0x3E;
  static constexpr uint8_t ACT_DUR            = 0x3F;

  /*------------------------------------------------
   LSM9DS0 Sensor Configuration Options
   ------------------------------------------------*/
  enum AccelerometerScale : uint8_t
  {
    A_SCALE_2G  = 0x00, /**< 2g */
    A_SCALE_4G  = 0x01, /**< 4g */
    A_SCALE_6G  = 0x02, /**< 6g */
    A_SCALE_8G  = 0x03, /**< 8g */
    A_SCALE_16G = 0x04  /**< 16g */
  };

  enum GyroscopeScale : uint8_t
  {
    G_SCALE_245DPS  = 0x00, /**< 245 degrees per second */
    G_SCALE_500DPS  = 0x01, /**< 500 degrees per second */
    G_SCALE_2000DPS = 0x02, /**< 2000 degrees per second */
  };

  enum MagnetometerScale : uint8_t
  {
    M_SCALE_2GS  = 0x00, /**< 2Gs */
    M_SCALE_4GS  = 0x01, /**< 4Gs */
    M_SCALE_8GS  = 0x02, /**< 8Gs */
    M_SCALE_12GS = 0x03, /**< 12Gs */
  };

  enum AccelerometerOuputDataRate : uint8_t
  {
    A_POWER_DOWN = 0x00, /**< Power-down mode */
    A_ODR_3125   = 0x01, /**< 3.125 Hz */
    A_ODR_625    = 0x02, /**< 6.25 Hz */
    A_ODR_125    = 0x03, /**< 12.5 Hz */
    A_ODR_25     = 0x04, /**< 25 Hz */
    A_ODR_50     = 0x05, /**< 50 Hz */
    A_ODR_100    = 0x06, /**< 100 Hz */
    A_ODR_200    = 0x07, /**< 200 Hz */
    A_ODR_400    = 0x08, /**< 400 Hz */
    A_ODR_800    = 0x09, /**< 800 Hz */
    A_ODR_1600   = 0x0A  /**< 1600 Hz */
  } accel_odr;

  enum GyroscopeOutputDataRate : uint8_t
  {                         /**< ODR (Hz) --- Cutoff*/
    G_ODR_95_BW_125  = 0x0, /**<   95      |  12.5  */
    G_ODR_95_BW_25   = 0x1, /**<   95      |   25   */
    G_ODR_190_BW_125 = 0x4, /**<   190     |  12.5  */
    G_ODR_190_BW_25  = 0x5, /**<   190     |   25   */
    G_ODR_190_BW_50  = 0x6, /**<   190     |   50   */
    G_ODR_190_BW_70  = 0x7, /**<   190     |   70   */
    G_ODR_380_BW_20  = 0x8, /**<   380     |   20   */
    G_ODR_380_BW_25  = 0x9, /**<   380     |   25   */
    G_ODR_380_BW_50  = 0xA, /**<   380     |   50   */
    G_ODR_380_BW_100 = 0xB, /**<   380     |   100  */
    G_ODR_760_BW_30  = 0xC, /**<   760     |   30   */
    G_ODR_760_BW_35  = 0xD, /**<   760     |   35   */
    G_ODR_760_BW_50  = 0xE, /**<   760     |   50   */
    G_ODR_760_BW_100 = 0xF, /**<   760     |   100  */
  };

  enum MagnetometerOutputDataRate : uint8_t
  {
    M_ODR_3125 = 0x00, /**< 3.125 Hz */
    M_ODR_625  = 0x01, /**< 6.25 Hz */
    M_ODR_125  = 0x02, /**< 12.5 Hz */
    M_ODR_25   = 0x03, /**< 25 Hz */
    M_ODR_50   = 0x04, /**< 50 */
    M_ODR_100  = 0x05, /**< 100 Hz */
  };

  enum AccelerometerAntiAliasFilterBandwidth : uint8_t
  {
    A_ABW_773 = 0x00, /**< 773 Hz */
    A_ABW_194 = 0x01, /**< 194 Hz */
    A_ABW_362 = 0x02, /**< 362 Hz */
    A_ABW_50  = 0x03, /**<  50 Hz */
  };

  struct Settings
  {
    struct OutputDataRate
    {
      AccelerometerOuputDataRate accel;
      GyroscopeOutputDataRate gyro;
      MagnetometerOutputDataRate mag;
    } outputDataRate;

    struct Scale
    {
      GyroscopeScale gyro;
      AccelerometerScale accel;
      MagnetometerScale mag;
    } scale;

    struct Resolution
    {
      float gyro;
      float accel;
      float mag;
    } resolution;

    Chimera::Modules::IMU::Measurement9DOF<float> bias;

    AccelerometerAntiAliasFilterBandwidth aaFilterBW;

    uint8_t xmAddress = 0x1D;
    uint8_t gAddress  = 0x6B;
  };

}    // namespace LSM9DS0

#endif /* !LSM9DS0_TYPES_HPP */