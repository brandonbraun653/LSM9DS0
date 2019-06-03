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

/* Eigen Includes */
#include <Eigen/Core>

/* Chimera Includes */
#include <Chimera/gpio.hpp>
#include <Chimera/spi.hpp>
#include <Chimera/extensions/spi_ext.hpp>
#include <Chimera/modules/sensors/imu/imu_intf.hpp>
#include <Chimera/modules/sensors/imu/imu_types.hpp>

/* Driver Includes */
#include <lsm9ds0/lsm9ds0_types.hpp>

namespace LSM9DS0
{
  class Driver : public Chimera::Modules::IMU::Interface9DOF, public Chimera::SPI::SPIAcceptor
  {
  public:
    Driver();
    ~Driver();

    Chimera::Status_t initialize() final override;

    Chimera::Status_t attachSPI( const Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::Setup &setup ) final override;

    Chimera::Status_t attachCS( const Chimera::GPIO::GPIOClass_sPtr &accelMagCS, const Chimera::GPIO::PinInit &accelMagSetup,
                                const Chimera::GPIO::GPIOClass_sPtr &gyroCS, const Chimera::GPIO::PinInit &gyroSetup );

    Chimera::Status_t calibrate( const Chimera::Modules::IMU::Sensor_t sensor ) final override;

    Chimera::Status_t reset() final override;

    Chimera::Status_t assignDOF( const Chimera::Modules::IMU::Sensor_t sensor0, const Chimera::Modules::IMU::Sensor_t sensor1,
                                 const Chimera::Modules::IMU::Sensor_t sensor2 ) final override;

    Chimera::Status_t measure( const Chimera::Modules::IMU::Sensor_t sensor, void *const measurement,
                               const size_t size ) final override;

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

    static constexpr size_t transferPacketSize = 32;
    std::array<uint8_t, transferPacketSize> cmd_pkt;
    std::array<uint8_t, transferPacketSize> rcv_pkt;

    bool spiDriverAttached   = false;
    bool chipSelectsAttached = false;
    bool imuSettingsAttached = false;

    /**
     *  Performs zero offset calibration of the accelerometer. Functionally this
     *  is accounting for slight reported offsets under zero acceleration conditions.
     *  In order for this to work, the accelerometer must be planar in X/Y, with 1G 
     *  perpendicular to the surface of the chip. (ie make it flat)
     *  
     *  After the calculations are complete, the result is the accelerometer will report
     *  the real acceleration in each axis relative to the calibration conditions.
     */
    void calibrateAccelZeroOffset();
    
    /**
     *  Performs zero offset calibration of the gyroscope. These sensors have a
     *  natural static value that is reported when the system is perfectly still and
     *  it must be subtracted from each measurement to get a close approximation of 
     *  the true rotation rate. This requires that the system is as still as possible
     *  while the calibration is in progress.
     *  
     *  After the calculations are complete, the sensor should report the rotation 
     *  rate relative to a perfectly still system.
     */
    void calibrateGyroZeroOffset();
    
    /**
     *  Currently this does nothing as it would require the user pointing the system
     *  at true north, or moving the device in a sphere. Deemed unnecessary for the
     *  moment and is simply just a placeholder for future implementations.
     */
    void calibrateMag();

    void updateGyroResolution();
    void updateMagResolution();
    void updateAccelResolution();

    Eigen::Matrix<float, 3, 1> convertRawAccel( const Eigen::Ref<const Eigen::Matrix<int16_t, 3, 1>> &data );
    Eigen::Matrix<float, 3, 1> convertRawGyro( const Eigen::Matrix<int16_t, 3, 1> &data );
    Eigen::Matrix<float, 3, 1> convertRawMag( const Eigen::Ref<const Eigen::Matrix<int16_t, 3, 1>> &data );

    void initGyro();
    void initAccel();
    void initMag();

    void setChipSelect( const Chimera::Modules::IMU::Sensor_t chip );
    void clearChipSelect( const Chimera::Modules::IMU::Sensor_t chip );
    void writeCmd( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, size_t length );
    void readCmd( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length );

    void writeRegister( const Chimera::Modules::IMU::Sensor_t chip, const uint8_t reg, const uint8_t value );
    uint8_t readRegister( const Chimera::Modules::IMU::Sensor_t chip, const uint8_t reg );
  };
}    // namespace LSM9DS0


#endif /* !DRIVER_LSM9DS0_HPP */
