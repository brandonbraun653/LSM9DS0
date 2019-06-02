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

/* Eigen Includes */
#include <Eigen/Core>

namespace LSM9DS0
{
  static constexpr size_t MAX_SPI_CLOCK  = 10000000;
  static constexpr size_t FAST_SPI_CLOCK = 8000000;
  static constexpr size_t MED_SPI_CLOCK  = 4000000;
  static constexpr size_t SLOW_SPI_CLOCK = 1000000;

  /*------------------------------------------------
  LSM9DS0 Gyro Registers
  ------------------------------------------------*/
  static constexpr uint8_t WHO_AM_I_G = 0x0F;

  /*-------------------------------------------------
  Control Register 1 Gyroscope
    - Output data rate
    - Output bandwidth
    - Power down mode enable/disable
    - Gyroscope XYZ axis sense enable/disable
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG1_G         = 0x20;
  
  static constexpr uint8_t CTRL_REG1_G_ODR_MSK = 0xF0;
  static constexpr uint8_t CTRL_REG1_G_ODR_POS = 4;

  static constexpr uint8_t CTRL_REG1_G_PD_POS = 3u;
  static constexpr uint8_t CTRL_REG1_G_PD     = 1u << CTRL_REG1_G_PD_POS;
  
  static constexpr uint8_t CTRL_REG1_G_Z_EN_POS = 2u;
  static constexpr uint8_t CTRL_REG1_G_Z_EN     = 1u << CTRL_REG1_G_Z_EN_POS;

  static constexpr uint8_t CTRL_REG1_G_Y_EN_POS = 1u;
  static constexpr uint8_t CTRL_REG1_G_Y_EN     = 1u << CTRL_REG1_G_Y_EN_POS;
  
  static constexpr uint8_t CTRL_REG1_G_X_EN_POS = 0u;
  static constexpr uint8_t CTRL_REG1_G_X_EN     = 1u << CTRL_REG1_G_X_EN_POS;
  
  /*-------------------------------------------------
  Control Register 2 Gyroscope
    - High pass filter mode and configuration 
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG2_G = 0x21;
  static constexpr uint8_t CTRL_REG2_G_MSK = 0x3F;

  static constexpr uint8_t CTRL_REG2_G_HPF_MODE_MSK    = 0x30;
  static constexpr uint8_t CTRL_REG2_G_HPF_MODE_POS    = 4u;
  static constexpr uint8_t CTRL_REG2_G_HPF_MODE_REF    = 0x01;
  static constexpr uint8_t CTRL_REG2_G_HPF_MODE_NORMAL = 0x2;
  static constexpr uint8_t CTRL_REG2_G_HPF_MODE_IT     = 0x03;

  static constexpr uint8_t CTRL_REG2_G_HPCF_MSK = 0x0F;
  static constexpr uint8_t CTRL_REG2_G_HPCF_POS = 0u;

  /*-------------------------------------------------
  Control Register 3 Gyroscope
    - Interrupt behavioral configuration settings
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG3_G = 0x22;

  static constexpr uint8_t CTRL_REG3_G_PP_OD_POS = 4u;
  static constexpr uint8_t CTRL_REG3_G_PP_OD     = 1u << CTRL_REG3_G_PP_OD_POS;

  /*-------------------------------------------------
  Control Register 4 Gyroscope
    - Block data update enable/disable
    - Big/little endian data reporting
    - Full scale selection
    - Self test enable
    - SPI interface mode (4-wire vs 3-wire)
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG4_G         = 0x23;

  static constexpr uint8_t CTRL_REG4_G_BDU_POS = 7u;
  static constexpr uint8_t CTRL_REG4_G_BDU     = 1u << CTRL_REG4_G_BDU_POS;

  static constexpr uint8_t CTRL_REG4_G_BLE_POS = 6u;
  static constexpr uint8_t CTRL_REG4_G_BLE     = 1u << CTRL_REG4_G_BLE_POS;

  static constexpr uint8_t CTRL_REG4_G_SCL_MSK = 0x30;
  static constexpr uint8_t CTRL_REG4_G_SCL_POS = 4u;
  
  static constexpr uint8_t CTRL_REG4_G_STE_MSK = 0x30;
  static constexpr uint8_t CTRL_REG4_G_STE_POS = 1u;

  static constexpr uint8_t CTRL_REG4_G_SIM_POS = 0u;
  static constexpr uint8_t CTRL_REG4_G_SIM     = 1u << CTRL_REG4_G_SIM_POS;

  /*-------------------------------------------------
  Control Register 5 Gyroscope
    - Reboot memory content
    - FIFO enable/disable
    - High pass filter enable/disable
    - INT1 selection config
    - OUT_sel1/0 config
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG5_G     = 0x24;
  
  static constexpr uint8_t CTRL_REG5_G_BOOT_POS = 7u;
  static constexpr uint8_t CTRL_REG5_G_BOOT     = 1u << CTRL_REG5_G_BOOT_POS;
  
  static constexpr uint8_t CTRL_REG5_G_FIFO_EN_POS = 6u;
  static constexpr uint8_t CTRL_REG5_G_FIFO_EN     = 1u << CTRL_REG5_G_FIFO_EN_POS;
  
  static constexpr uint8_t CTRL_REG5_G_HPF_EN_POS = 4u;
  static constexpr uint8_t CTRL_REG5_G_HPF_EN     = 1u << CTRL_REG5_G_HPF_EN_POS;


  static constexpr uint8_t REFERENCE_G     = 0x25;
  static constexpr uint8_t STATUS_REG_G    = 0x27;
  
  /*-------------------------------------------------
  Data Output Registers Gyroscope
  -------------------------------------------------*/
  static constexpr uint8_t OUT_X_L_G       = 0x28;
  static constexpr uint8_t OUT_X_H_G       = 0x29;
  static constexpr uint8_t OUT_Y_L_G       = 0x2A;
  static constexpr uint8_t OUT_Y_H_G       = 0x2B;
  static constexpr uint8_t OUT_Z_L_G       = 0x2C;
  static constexpr uint8_t OUT_Z_H_G       = 0x2D;
  
  /*-------------------------------------------------
  FIFO Control Register Gyroscope
    - FIFO Mode Selection
    - FIFO Threshold/Watermark setting
  -------------------------------------------------*/
  static constexpr uint8_t FIFO_CTRL_REG_G = 0x2E;

  static constexpr uint8_t FIFO_CTRL_REG_G_FM_MSK           = 0xE0;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_POS           = 5u;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_BYPASS        = 0x00 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_FIFO          = 0x01 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_STREAM        = 0x02 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_STREAM2FIFO   = 0x03 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_G_FM_BYPASS2STREAM = 0x04 << FIFO_CTRL_REG_G_FM_POS;

  static constexpr uint8_t FIFO_CTRL_REG_G_THRSH_MSK = 0x1F;

  /*-------------------------------------------------
  FIFO Source Register Gyroscope
    - Watermark status
    - Overrun status
    - Empty status
    - FIFO stored data level
  -------------------------------------------------*/
  static constexpr uint8_t FIFO_SRC_REG_G  = 0x2F;

  static constexpr uint8_t FIFO_SRC_REG_G_WTM_POS = 7u;
  static constexpr uint8_t FIFO_SRC_REG_G_WTM     = 1u << FIFO_SRC_REG_G_WTM_POS;

  static constexpr uint8_t FIFO_SRC_REG_G_OVRN_POS = 6u;
  static constexpr uint8_t FIFO_SRC_REG_G_OVRN     = 1u << FIFO_SRC_REG_G_OVRN_POS;

  static constexpr uint8_t FIFO_SRC_REG_G_EMPTY_POS = 5u;
  static constexpr uint8_t FIFO_SRC_REG_G_EMPTY     = 1u << FIFO_SRC_REG_G_EMPTY_POS;

  static constexpr uint8_t FIFO_SRC_REG_G_FSS_MSK = 0x1F;
  static constexpr uint8_t FIFO_SRC_REG_G_FSS_POS = 0u;


  /*-------------------------------------------------
  Interrupt Configuration Registers Gyroscope
    - Currently not used/supported
  -------------------------------------------------*/
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
  static constexpr uint8_t OUT_TEMP_L_XM  = 0x05;
  static constexpr uint8_t OUT_TEMP_H_XM  = 0x06;
  static constexpr uint8_t STATUS_REG_M   = 0x07;
  static constexpr uint8_t OUT_X_L_M      = 0x08;
  static constexpr uint8_t OUT_X_H_M      = 0x09;
  static constexpr uint8_t OUT_Y_L_M      = 0x0A;
  static constexpr uint8_t OUT_Y_H_M      = 0x0B;
  static constexpr uint8_t OUT_Z_L_M      = 0x0C;
  static constexpr uint8_t OUT_Z_H_M      = 0x0D;
  static constexpr uint8_t WHO_AM_I_XM    = 0x0F;
  static constexpr uint8_t INT_CTRL_REG_M = 0x12;
  static constexpr uint8_t INT_SRC_REG_M  = 0x13;
  static constexpr uint8_t INT_THS_L_M    = 0x14;
  static constexpr uint8_t INT_THS_H_M    = 0x15;
  static constexpr uint8_t OFFSET_X_L_M   = 0x16;
  static constexpr uint8_t OFFSET_X_H_M   = 0x17;
  static constexpr uint8_t OFFSET_Y_L_M   = 0x18;
  static constexpr uint8_t OFFSET_Y_H_M   = 0x19;
  static constexpr uint8_t OFFSET_Z_L_M   = 0x1A;
  static constexpr uint8_t OFFSET_Z_H_M   = 0x1B;
  static constexpr uint8_t REFERENCE_X    = 0x1C;
  static constexpr uint8_t REFERENCE_Y    = 0x1D;
  static constexpr uint8_t REFERENCE_Z    = 0x1E;
  
  /*-------------------------------------------------
  Control Register 0 Accel/Mag
    - Boot enable/disable
    - FIFO enable/disable
    - FIFO watermark enable/disable
    - High pass filter for click function enable/disable
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG0_XM   = 0x1F;
  static constexpr uint8_t CTRL_REG0_XM_MSK = 0xE7;

  static constexpr uint8_t CTRL_REG0_XM_BOOT_POS = 7u;
  static constexpr uint8_t CTRL_REG0_XM_BOOT     = 1u << CTRL_REG0_XM_BOOT_POS;
  
  static constexpr uint8_t CTRL_REG0_XM_FIFO_EN_POS = 6u;
  static constexpr uint8_t CTRL_REG0_XM_FIFO_EN     = 1u << CTRL_REG0_XM_FIFO_EN_POS;
  
  static constexpr uint8_t CTRL_REG0_XM_WTMN_EN_POS = 5u;
  static constexpr uint8_t CTRL_REG0_XM_WTMN_EN     = 1u << CTRL_REG0_XM_WTMN_EN_POS;
  
  static constexpr uint8_t CTRL_REG0_XM_HPF_CLICK_EN_POS = 2u;
  static constexpr uint8_t CTRL_REG0_XM_HPF_CLICK_EN     = 1u << CTRL_REG0_XM_HPF_CLICK_EN_POS;
;

  /*-------------------------------------------------
  Control Register 1 Accel/Mag
    - Accelerometer output data rate
    - Block data update enable/disable
    - Accelerometer XYZ axis sense enable/disable
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG1_XM            = 0x20;
  
  static constexpr uint8_t CTRL_REG1_XM_ODR_MSK    = 0xF0;
  static constexpr uint8_t CTRL_REG1_XM_ODR_POS    = 4u;

  static constexpr uint8_t CTRL_REG1_XM_BDU_EN_POS = 3u;
  static constexpr uint8_t CTRL_REG1_XM_BDU_EN     = 1u << CTRL_REG1_XM_BDU_EN_POS;

  static constexpr uint8_t CTRL_REG1_XM_XYZ_EN_MSK = 0x07;
  static constexpr uint8_t CTRL_REG1_XM_AX_EN      = 1u << 0;
  static constexpr uint8_t CTRL_REG1_XM_AY_EN      = 1u << 1;
  static constexpr uint8_t CTRL_REG1_XM_AZ_EN      = 1u << 2;

  /*-------------------------------------------------
  Control Register 2 Accel/Mag
    - Accelerometer anti-alias filter bandwidth
    - Acceleration full scale selection
    - Acceleration self-test enable/disable
    - SPI transaction mode (4-wire vs 3-wire)
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG2_XM         = 0x21;
  static constexpr uint8_t CTRL_REG2_XM_ABW_MSK = 0xC0;
  static constexpr uint8_t CTRL_REG2_XM_ABW_POS = 6u;
  static constexpr uint8_t CTRL_REG2_XM_SCL_MSK = 0x38;
  static constexpr uint8_t CTRL_REG2_XM_SCL_POS = 3u;

  static constexpr uint8_t CTRL_REG3_XM = 0x22;
  static constexpr uint8_t CTRL_REG4_XM = 0x23;

  /*-------------------------------------------------
  Control Register 5 Accel/Mag
    - Temperature sensor enable/disable
    - Magnetic resolution
    - Magnetic data rate
    - Latch interrupt request status
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG5_XM         = 0x24;

  static constexpr uint8_t CTRL_REG5_XM_TEMP_EN_POS = 7u;
  static constexpr uint8_t CTRL_REG5_XM_TEMP_EN     = 1u << CTRL_REG5_XM_TEMP_EN_POS;

  static constexpr uint8_t CTRL_REG5_XM_ODR_MSK = 0x07;
  static constexpr uint8_t CTRL_REG5_XM_ODR_POS = 2u;

  /*-------------------------------------------------
  Control Register 6 Accel/Mag
    - Magnetometer full scale selection
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG6_XM         = 0x25;
  
  static constexpr uint8_t CTRL_REG6_XM_SCL_MSK = 0x60;
  static constexpr uint8_t CTRL_REG6_XM_SCL_POS = 5u;

  /*-------------------------------------------------
  Control Register 7 Accel/Mag
    - Accelerometer high pass filter mode selection
    - Accelerometer data source selection
    - Magnetometer low power mode
    - Magnetometer sensor mode
  -------------------------------------------------*/
  static constexpr uint8_t CTRL_REG7_XM       = 0x26;
  static constexpr uint8_t CTRL_REG7_XM_MSK   = 0xE7;

  static constexpr uint8_t CTRL_REG7_XM_AHPM_MSK = 0xC0;
  static constexpr uint8_t CTRL_REG7_XM_AHPM_POS = 6u;

  static constexpr uint8_t CTRL_REG7_XM_AFDS_POS = 5u;
  static constexpr uint8_t CTRL_REG7_XM_AFDS     = 1u << CTRL_REG7_XM_AFDS_POS;

  static constexpr uint8_t CTRL_REG7_XM_MLP_POS = 2u;
  static constexpr uint8_t CTRL_REG7_XM_MLP     = 1u << CTRL_REG7_XM_MLP_POS;

  static constexpr uint8_t CTRL_REG7_XM_MD_MSK = 0x03;
  static constexpr uint8_t CTRL_REG7_XM_MD_POS = 0u;

  
  static constexpr uint8_t STATUS_REG_A       = 0x27;
  static constexpr uint8_t OUT_X_L_A          = 0x28;
  static constexpr uint8_t OUT_X_H_A          = 0x29;
  static constexpr uint8_t OUT_Y_L_A          = 0x2A;
  static constexpr uint8_t OUT_Y_H_A          = 0x2B;
  static constexpr uint8_t OUT_Z_L_A          = 0x2C;
  static constexpr uint8_t OUT_Z_H_A          = 0x2D;
  
  /*-------------------------------------------------
  FIFO Control Register Accel/Mag
    - FIFO Mode Selection
    - FIFO Threshold/Watermark setting
  -------------------------------------------------*/
  static constexpr uint8_t FIFO_CTRL_REG_XM      = 0x2E;
  
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_MSK           = 0xE0;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_POS           = 5u;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_BYPASS        = 0x00 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_FIFO          = 0x01 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_STREAM        = 0x02 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_STREAM2FIFO   = 0x03 << FIFO_CTRL_REG_G_FM_POS;
  static constexpr uint8_t FIFO_CTRL_REG_XM_FM_BYPASS2STREAM = 0x04 << FIFO_CTRL_REG_G_FM_POS;

  static constexpr uint8_t FIFO_CTRL_REG_XM_THRSH_MSK = 0x1F;
  
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
  };

  enum AccelerometerAntiAliasFilterBandwidth : uint8_t
  {
    A_ABW_773 = 0x00, /**< 773 Hz */
    A_ABW_194 = 0x01, /**< 194 Hz */
    A_ABW_362 = 0x02, /**< 362 Hz */
    A_ABW_50  = 0x03, /**<  50 Hz */
  };

  enum AccelerometerHighPassFilterMode : uint8_t
  {
    A_HPFM_NORMAL0,
    A_HPFM_REF,
    A_HPFM_NORMAL1,
    A_HPFM_IT
  };

  enum GyroscopeScale : uint8_t
  {
    G_SCALE_245DPS  = 0x00, /**< 245 degrees per second */
    G_SCALE_500DPS  = 0x01, /**< 500 degrees per second */
    G_SCALE_2000DPS = 0x02, /**< 2000 degrees per second */
  };

  enum GyroscopeOutputDataRate : uint8_t
  {
    G_ODR_95_BW_125  = 0x0, /**<   95  (ODR,Hz)    |   12.5 (BW,Hz)  */
    G_ODR_95_BW_25   = 0x1, /**<   95  (ODR,Hz)    |   25   (BW,Hz)  */
    G_ODR_190_BW_125 = 0x4, /**<   190 (ODR,Hz)    |   12.5 (BW,Hz)  */
    G_ODR_190_BW_25  = 0x5, /**<   190 (ODR,Hz)    |   25   (BW,Hz)  */
    G_ODR_190_BW_50  = 0x6, /**<   190 (ODR,Hz)    |   50   (BW,Hz)  */
    G_ODR_190_BW_70  = 0x7, /**<   190 (ODR,Hz)    |   70   (BW,Hz)  */
    G_ODR_380_BW_20  = 0x8, /**<   380 (ODR,Hz)    |   20   (BW,Hz)  */
    G_ODR_380_BW_25  = 0x9, /**<   380 (ODR,Hz)    |   25   (BW,Hz)  */
    G_ODR_380_BW_50  = 0xA, /**<   380 (ODR,Hz)    |   50   (BW,Hz)  */
    G_ODR_380_BW_100 = 0xB, /**<   380 (ODR,Hz)    |   100  (BW,Hz)  */
    G_ODR_760_BW_30  = 0xC, /**<   760 (ODR,Hz)    |   30   (BW,Hz)  */
    G_ODR_760_BW_35  = 0xD, /**<   760 (ODR,Hz)    |   35   (BW,Hz)  */
    G_ODR_760_BW_50  = 0xE, /**<   760 (ODR,Hz)    |   50   (BW,Hz)  */
    G_ODR_760_BW_100 = 0xF, /**<   760 (ODR,Hz)    |   100  (BW,Hz)  */
  };

  /**
   * High pass filter cutoff frequency configuration (Hz) for
   * a given output data rate setting. See Table 26 in the device
   * datasheet (Pg.43)
   * 
   * |    Enum     | 95Hz  | 190Hz | 380Hz | 760Hz |
   * |-------------|-------|-------|-------|-------|
   * | G_HPF_LVL_0 | 7.2   | 13.5  | 27 5  | 1.4   |
   * | G_HPF_LVL_1 | 3.5   | 7.2   | 13.5  | 27    |
   * | G_HPF_LVL_2 | 1.8   | 3.5   | 7.2   | 13.5  |
   * | G_HPF_LVL_3 | 0.9   | 1.8   | 3.5   | 7.2   |
   * | G_HPF_LVL_4 | 0.45  | 0.9   | 1.8   | 3.5   |
   * | G_HPF_LVL_5 | 0.18  | 0.45  | 0.9   | 1.8   |
   * | G_HPF_LVL_6 | 0.09  | 0.18  | 0.45  | 0.9   |
   * | G_HPF_LVL_7 | 0.045 | 0.09  | 0.18  | 0.45  |
   * | G_HPF_LVL_8 | 0.018 | 0.045 | 0.09  | 0.18  |
   * | G_HPF_LVL_9 | 0.009 | 0.018 | 0.045 | 0.09  |
   */
  enum GyroscopeHighPassFilter : uint8_t
  {
    G_HPF_LVL_0,  
    G_HPF_LVL_1,  
    G_HPF_LVL_2,  
    G_HPF_LVL_3,  
    G_HPF_LVL_4,  
    G_HPF_LVL_5,  
    G_HPF_LVL_6,  
    G_HPF_LVL_7,  
    G_HPF_LVL_8,  
    G_HPF_LVL_9
  };

  enum MagnetometerScale : uint8_t
  {
    M_SCALE_2GS  = 0x00, /**< 2Gs */
    M_SCALE_4GS  = 0x01, /**< 4Gs */
    M_SCALE_8GS  = 0x02, /**< 8Gs */
    M_SCALE_12GS = 0x03, /**< 12Gs */
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

  enum MagnetometerSensorMode : uint8_t
  {
    M_SM_CONTINUOUS,
    M_SM_SINGLE,
    M_SM_PWR_DN
  };


  struct Settings
  {
    /**
     *  Set the sensor output data rate.
     */
    struct OutputDataRate
    {
      AccelerometerOuputDataRate accel = A_ODR_100;
      GyroscopeOutputDataRate gyro     = G_ODR_95_BW_125;
      MagnetometerOutputDataRate mag   = M_ODR_3125;
    } outputDataRate;

    /**
     *  Set the sensor output range scale.
     */
    struct Scale
    {
      AccelerometerScale accel = A_SCALE_2G;
      GyroscopeScale gyro      = G_SCALE_245DPS;
      MagnetometerScale mag    = M_SCALE_2GS;
    } scale;
    
    /**
     *  Set the sensor anti-aliasing filter bandwidth
     */
    uint8_t aaFilterBW = A_ABW_773;
    
    /**
     *  Set the sensor address. Defaulted to SPI configuration.
     */
    uint8_t xmAddress = 0x1D;
    uint8_t gAddress  = 0x6B;

    /**
     *  Specifies the resolution per sensor of each bit in the 
     *  measured 16-bit ADC data. Do not specify in setup.
     */
    struct Resolution
    {
      float gyro;   /**< mdps/ADC tick */
      float accel;  /**< mg/ADC tick */
      float mag;    /**< mgauss/ADC tick*/
    } resolution;

    /**
     *  The current calculated bias for each sensor & axis.
     *  Do not specify in setup.
     */
    Chimera::Modules::IMU::Measurement9DOF<float> bias;
  };

}    // namespace LSM9DS0

#endif /* !LSM9DS0_TYPES_HPP */