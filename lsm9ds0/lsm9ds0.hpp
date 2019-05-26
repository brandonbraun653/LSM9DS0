/********************************************************************************
 *   File Name:
 *       lsm9ds0.hpp
 *
 *   Description:
 *       Implements the driver for the LSM9DS0
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#pragma once
#ifndef DRIVER_LSM9DS0_HPP
#define DRIVER_LSM9DS0_HPP

/* C++ Includes */
#include <cstdlib>
#include <cstdint>
#include <array>

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/modules/sensors/imu/imu_intf.hpp>
#include <Chimera/modules/sensors/imu/imu_types.hpp>

/* Driver Includes */
#include <lsm9ds0/lsm9ds0_types.hpp>

namespace LSM9DS0
{
  class Driver : public Chimera::Modules::IMU::Interface9DOF
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t initialize() final override;

    Chimera::Status_t calibrate( const Chimera::Modules::IMU::Sensor_t sensor ) final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t assignDOF( const Chimera::Modules::IMU::Sensor_t sensor0, const Chimera::Modules::IMU::Sensor_t sensor1,
                                 const Chimera::Modules::IMU::Sensor_t sensor2 ) final override;

    Chimera::Status_t measure( const Chimera::Modules::IMU::Sensor_t sensor, void *const measurement,
                               const size_t size ) final override;

    Chimera::Status_t attachHW( Chimera::SPI::SPIClass_sPtr &spi, Chimera::GPIO::GPIOClass_sPtr &accelMagCS,
                                Chimera::GPIO::GPIOClass_sPtr &gyroCS );

    Chimera::Status_t attachSettings( const Settings &settings );

    Chimera::Status_t setScaling( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value );

    Chimera::Status_t setODR( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value );

    Chimera::Status_t setABW( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value );

  private:
    Settings settings;
    Chimera::SPI::SPIClass_sPtr spi;
    Chimera::GPIO::GPIOClass_sPtr accelMagCS;
    Chimera::GPIO::GPIOClass_sPtr gyroCS;

    Chimera::Modules::IMU::Measurement9DOF<float> data;
    Chimera::Modules::IMU::Measurement9DOF<int16_t> rawData;

    std::array<uint8_t, 32> cmd_pkt;
    std::array<uint8_t, 32> rcv_pkt;

    bool hardwareAttached = false;
    bool settingsAttached = false;

    /* Reads sensor data */
    void readDevice( const Chimera::Modules::IMU::Sensor_t chip );

    /* Resolution Calculations */
    void calc_gRes();
    void calc_mRes();
    void calc_aRes();

    /* Converts from raw data to float */
    void calc_gyro();
    void calc_accel();
    void calc_mag();

    /* Initializes sensors */
    void init_gyro();
    void init_accel();
    void init_mag();

    /* Hardware Level Interfacing */
    void set_CS( const Chimera::Modules::IMU::Sensor_t chip );
    void clr_CS( const Chimera::Modules::IMU::Sensor_t chip );
    void write_pkt( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, size_t length );
    void read_pkt( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length );

    /* Buffer Management */
    void clr_cmd_pkt();
    void clr_rcv_pkt();
  };
}    // namespace LSM9DS0


#endif /* !DRIVER_LSM9DS0_HPP */
