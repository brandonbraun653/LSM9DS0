/********************************************************************************
 *   File Name:
 *       lsm9ds0.cpp
 *
 *   Description:
 *       Implements the driver for the LSM9DS0
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

/* C++ Includes */
#include <limits>

/* Chimera Includes */
#include <Chimera/chimera.hpp>
#include <Chimera/constants/physics.hpp>
#include <Chimera/constants/units.hpp>

/* Driver Includes */
#include <lsm9ds0/lsm9ds0.hpp>


namespace LSM9DS0
{
  static constexpr uint8_t LSM_READ_BIT      = 1u << 7;
  static constexpr uint8_t LSM_WRITE_BIT     = static_cast<uint8_t>( ~LSM_READ_BIT );
  static constexpr uint8_t LSM_AUTO_INCR_BIT = 1u << 6;
  static constexpr size_t IMU_READ_LEN       = 7;
  static constexpr size_t TEMP_READ_LEN      = 3;

  Driver::Driver() : spiDriverAttached( false ), chipSelectsAttached( false ), imuSettingsAttached( false )
  {
  }

  Driver::~Driver()
  {
  }

  Chimera::Status_t Driver::initialize()
  {
    using namespace Chimera::GPIO;
    using namespace Chimera::Hardware;
    using namespace Chimera::SPI;
    using namespace Chimera::Modules::IMU;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;
    bool sensorPresent       = true;

    if ( !spiDriverAttached || !chipSelectsAttached || !imuSettingsAttached )
    {
      result = Chimera::CommonStatusCodes::NOT_INITIALIZED;
    }
    else
    {
      Chimera::SPI::Setup setup = spi->getInit();

      /*------------------------------------------------
      Initialize the low level hardware resources
      ------------------------------------------------*/
      accelMagCS->setMode( Drive::OUTPUT_PUSH_PULL, true );
      accelMagCS->setState( State::HIGH );

      gyroCS->setMode( Drive::OUTPUT_PUSH_PULL, true );
      gyroCS->setState( State::HIGH );

      spi->setChipSelectControlMode( ChipSelectMode::MANUAL );
      spi->setPeripheralMode( SubPeripheral::TXRX, setup.transferMode );


      reset();

      /*------------------------------------------------
      Attempt to find the gyroscope sensor
      ------------------------------------------------*/
      if ( readRegister( Sensor_t::GYRO, WHO_AM_I_G ) != 0xD4 )
      {
        sensorPresent = false;
        result        = Chimera::CommonStatusCodes::FAIL;
      }

      /*------------------------------------------------
      Attempt to find the accelerometer/magnetometer sensor
      ------------------------------------------------*/
      if ( readRegister( Sensor_t::ACCEL, WHO_AM_I_XM ) != 0x49 )
      {
        sensorPresent = false;
        result        = Chimera::CommonStatusCodes::FAIL;
      }

      /*------------------------------------------------
      Initialize everything properly
      ------------------------------------------------*/
      Chimera::Status_t initFailDetect = Chimera::CommonStatusCodes::OK;

      initAccel();
      initFailDetect |= setODR( Sensor_t::ACCEL, settings.outputDataRate.accel );
      initFailDetect |= setScaling( Sensor_t::ACCEL, settings.scale.accel );
      updateAccelResolution();

      initGyro();
      initFailDetect |= setODR( Sensor_t::GYRO, settings.outputDataRate.gyro );
      initFailDetect |= setScaling( Sensor_t::GYRO, settings.scale.gyro );
      updateGyroResolution();

      initMag();
      initFailDetect |= setODR( Sensor_t::MAG, settings.outputDataRate.mag );
      initFailDetect |= setScaling( Sensor_t::MAG, settings.scale.mag );
      updateMagResolution();

      if ( initFailDetect != Chimera::CommonStatusCodes::OK )
      {
        result = Chimera::CommonStatusCodes::FAIL;
      }
    }

    return result;
  }

  Chimera::Status_t Driver::attachSPI( const Chimera::SPI::SPIClass_sPtr &spi, Chimera::SPI::Setup &setup )
  {
    using namespace Chimera::SPI;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    setup.mode      = Mode::MASTER;
    setup.bitOrder  = BitOrder::MSB_FIRST;
    setup.dataSize  = DataSize::SZ_8BIT;
    setup.clockMode = ClockMode::MODE0;

    if ( setup.clockFrequency > MAX_SPI_CLOCK )
    {
      setup.clockFrequency = SLOW_SPI_CLOCK;
    }

    if ( spi->init( setup ) != Chimera::CommonStatusCodes::OK )
    {
      result = Chimera::CommonStatusCodes::FAIL;
    }
    else
    {
      this->spi         = spi;
      spiDriverAttached = true;
    }

    return result;
  }

  Chimera::Status_t Driver::attachCS( const Chimera::GPIO::GPIOClass_sPtr &accelMagCS,
                                      const Chimera::GPIO::PinInit &accelMagSetup, const Chimera::GPIO::GPIOClass_sPtr &gyroCS,
                                      const Chimera::GPIO::PinInit &gyroSetup )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( !accelMagCS || !gyroCS )
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      accelMagCS->init( accelMagSetup.port, accelMagSetup.pin );
      gyroCS->init( gyroSetup.port, gyroSetup.pin );

      this->accelMagCS    = accelMagCS;
      this->gyroCS        = gyroCS;
      chipSelectsAttached = true;
    }


    return result;
  }

  Chimera::Status_t Driver::calibrate( const Chimera::Modules::IMU::Sensor_t chip )
  {
    using namespace Chimera::Modules::IMU;
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    switch ( chip )
    {
      case Sensor_t::ACCEL:
        calibrateAccelZeroOffset();
        break;

      case Sensor_t::GYRO:
        calibrateGyroZeroOffset();
        break;

      case Sensor_t::MAG:
        calibrateMag();
        break;

      case Sensor_t::TEMP:
      default:
        result = Chimera::CommonStatusCodes::FAIL;
        break;
    };

    return result;
  }

  Chimera::Status_t Driver::reset()
  {
    using namespace Chimera::Modules::IMU;
    static constexpr size_t rebootDelay = 50;
    uint8_t regCache                    = 0;

    /*------------------------------------------------
    Reload accelerometer factory calibration
    ------------------------------------------------*/
    regCache = readRegister( Sensor_t::ACCEL, CTRL_REG0_XM );

    regCache |= CTRL_REG0_XM_BOOT;
    writeRegister( Sensor_t::ACCEL, CTRL_REG0_XM, regCache & CTRL_REG0_XM_MSK );
    Chimera::delayMilliseconds( rebootDelay );

    regCache &= ~CTRL_REG0_XM_BOOT;
    writeRegister( Sensor_t::ACCEL, CTRL_REG0_XM, regCache & CTRL_REG0_XM_MSK );
    Chimera::delayMilliseconds( rebootDelay );

    /*------------------------------------------------
    Reload gyro factory calibration
    ------------------------------------------------*/
    regCache = readRegister( Sensor_t::GYRO, CTRL_REG5_G );

    regCache |= CTRL_REG5_G_BOOT;
    writeRegister( Sensor_t::GYRO, CTRL_REG5_G, regCache );
    Chimera::delayMilliseconds( rebootDelay );

    regCache &= ~CTRL_REG5_G_BOOT;
    writeRegister( Sensor_t::GYRO, CTRL_REG5_G, regCache );
    Chimera::delayMilliseconds( rebootDelay );

    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::assignDOF( const Chimera::Modules::IMU::Sensor_t sensor0,
                                       const Chimera::Modules::IMU::Sensor_t sensor1,
                                       const Chimera::Modules::IMU::Sensor_t sensor2 )
  {
    /*-------------------------------------------------
    By default the sensors will be configured as:
      sensor0: Accelerometer
      sensor1: Gyroscope
      sensor2: Magnetometer
    -------------------------------------------------*/
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t Driver::measure( const Chimera::Modules::IMU::Sensor_t chip, void *const measurement, const size_t size )
  {
    using namespace Chimera::Modules::IMU;
    constexpr size_t SHIFT = std::numeric_limits<uint8_t>::digits;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    if ( size != sizeof( data ) )
    {
      result = Chimera::CommonStatusCodes::INVAL_FUNC_PARAM;
    }
    else
    {
      Eigen::Matrix<int16_t, 3, 1> rawData;
      cmd_pkt.fill( 0 );
      rcv_pkt.fill( 0 );

      switch ( chip )
      {
        case Sensor_t::ACCEL:
          cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
          readCmd( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

          rawData[ 0 ] = static_cast<int16_t>( ( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ] );
          rawData[ 1 ] = static_cast<int16_t>( ( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ] );
          rawData[ 2 ] = static_cast<int16_t>( ( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ] );

          data.sensor0 = convertRawAccel( rawData ) - settings.zeroOffset.sensor0;
          break;

        case Sensor_t::GYRO:
          cmd_pkt[ 0 ] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
          readCmd( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

          rawData[ 0 ] = static_cast<int16_t>( ( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ] );
          rawData[ 1 ] = static_cast<int16_t>( ( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ] );
          rawData[ 2 ] = static_cast<int16_t>( ( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ] );

          data.sensor1 = convertRawGyro( rawData ) - settings.zeroOffset.sensor1;
          break;

        case Sensor_t::MAG:
          cmd_pkt[ 0 ] = OUT_X_L_M | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
          readCmd( Sensor_t::MAG, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

          rawData[ 0 ] = static_cast<int16_t>( ( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ] );
          rawData[ 1 ] = static_cast<int16_t>( ( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ] );
          rawData[ 2 ] = static_cast<int16_t>( ( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ] );

          data.sensor2 = convertRawMag( rawData ) - settings.zeroOffset.sensor2;
          break;

        case Sensor_t::TEMP:
          cmd_pkt[ 0 ] = OUT_TEMP_L_XM | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
          readCmd( Sensor_t::TEMP, cmd_pkt.data(), rcv_pkt.data(), TEMP_READ_LEN );

          // temperature_data = (((int16_t)rcv_pkt[2] << 12) | rcv_pkt[1] << 4) >> 4; // Temperature is a 12-bit signed integer
          break;

        default:
          break;
      }

      memcpy( measurement, &data, sizeof( data ) );
    }

    return result;
  }

  Chimera::Status_t Driver::attachSettings( const Settings &settings )
  {
    this->settings      = settings;
    imuSettingsAttached = true;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setScaling( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    using namespace Chimera::Modules::IMU;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    uint8_t cachedSettings = 0u;

    switch ( sensor )
    {
      case Sensor_t::ACCEL:
        cachedSettings = readRegister( Sensor_t::ACCEL, CTRL_REG2_XM );
        cachedSettings &= ~CTRL_REG2_XM_SCL_MSK;
        cachedSettings |= ( value << CTRL_REG2_XM_SCL_POS ) & CTRL_REG2_XM_SCL_MSK;

        writeRegister( Sensor_t::ACCEL, CTRL_REG2_XM, cachedSettings );

        settings.scale.accel = static_cast<AccelerometerScale>( value );
        updateAccelResolution();
        break;

      case Sensor_t::GYRO:
        cachedSettings = readRegister( Sensor_t::GYRO, CTRL_REG4_G );
        cachedSettings &= ~CTRL_REG4_G_SCL_MSK;
        cachedSettings |= ( value << CTRL_REG4_G_SCL_POS ) & CTRL_REG4_G_SCL_MSK;

        writeRegister( Sensor_t::GYRO, CTRL_REG4_G, cachedSettings );

        settings.scale.gyro = static_cast<GyroscopeScale>( value );
        updateGyroResolution();
        break;

      case Sensor_t::MAG:
        cachedSettings = readRegister( Sensor_t::MAG, CTRL_REG6_XM );
        cachedSettings &= ~CTRL_REG6_XM_SCL_MSK;
        cachedSettings |= ( value << CTRL_REG6_XM_SCL_POS ) & CTRL_REG6_XM_SCL_MSK;

        writeRegister( Sensor_t::MAG, CTRL_REG6_XM, cachedSettings );

        settings.scale.mag = static_cast<MagnetometerScale>( value );
        updateMagResolution();
        break;

      case Sensor_t::TEMP:
      default:
        result = Chimera::CommonStatusCodes::FAIL;
        break;
    };

    return result;
  }

  Chimera::Status_t Driver::setODR( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    using namespace Chimera::Modules::IMU;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    uint8_t cachedSettings = 0u;

    switch ( sensor )
    {
      case Sensor_t::ACCEL:
        cachedSettings = readRegister( Sensor_t::ACCEL, CTRL_REG1_XM );
        cachedSettings &= ~CTRL_REG1_XM_ODR_MSK;
        cachedSettings |= ( value << CTRL_REG1_XM_ODR_POS ) & CTRL_REG1_XM_ODR_MSK;

        writeRegister( Sensor_t::ACCEL, CTRL_REG1_XM, cachedSettings );
        settings.outputDataRate.accel = static_cast<AccelerometerOuputDataRate>( value );
        break;

      case Sensor_t::GYRO:
        cachedSettings = readRegister( Sensor_t::GYRO, CTRL_REG1_G );
        cachedSettings &= ~CTRL_REG1_G_ODR_MSK;
        cachedSettings |= ( value << CTRL_REG1_G_ODR_POS ) & CTRL_REG1_G_ODR_MSK;

        writeRegister( Sensor_t::GYRO, CTRL_REG1_G, cachedSettings );
        settings.outputDataRate.gyro = static_cast<GyroscopeOutputDataRate>( value );
        break;

      case Sensor_t::MAG:
        cachedSettings = readRegister( Sensor_t::MAG, CTRL_REG5_XM );
        cachedSettings &= ~CTRL_REG5_XM_MODR_MSK;
        cachedSettings |= ( value << CTRL_REG5_XM_MODR_POS ) & CTRL_REG5_XM_MODR_MSK;

        writeRegister( Sensor_t::MAG, CTRL_REG5_XM, cachedSettings );
        settings.outputDataRate.mag = static_cast<MagnetometerOutputDataRate>( value );
        break;

      case Sensor_t::TEMP:
      default:
        result = Chimera::CommonStatusCodes::FAIL;
        break;
    };

    return result;
  }

  Chimera::Status_t Driver::setABW( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    using namespace Chimera::Modules::IMU;

    Chimera::Status_t result = Chimera::CommonStatusCodes::FAIL;

    if ( sensor == Sensor_t::ACCEL )
    {
      cmd_pkt.fill( 0 );
      rcv_pkt.fill( 0 );

      uint8_t cachedSettings = readRegister( Sensor_t::ACCEL, CTRL_REG2_XM );

      cachedSettings &= ~CTRL_REG2_XM_ABW_MSK;
      cachedSettings |= ( value << CTRL_REG2_XM_ABW_POS ) & CTRL_REG2_XM_ABW_MSK;

      writeRegister( Sensor_t::ACCEL, CTRL_REG2_XM, cachedSettings );
      settings.aaFilterBW = value;
      result              = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  void Driver::calibrateAccelZeroOffset()
  {
    using namespace Chimera::Modules::IMU;
    static constexpr size_t calibrationDelay = 20;

    /*------------------------------------------------
    Initialize everything properly
    ------------------------------------------------*/
    cmd_pkt.fill( 0 );
    Eigen::Matrix<int16_t, 3, 1> rawAccelBias;
    uint8_t samples       = 0;
    uint8_t registerCache = 0;

    /*------------------------------------------------
    Enable FIFO and wait for it to take effect
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::ACCEL, CTRL_REG0_XM );
    registerCache |= CTRL_REG0_XM_FIFO_EN;
    
    writeRegister( Sensor_t::ACCEL, CTRL_REG0_XM, registerCache & CTRL_REG0_XM_MSK );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Enable FIFO stream mode and set watermark at 32 samples
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::ACCEL, FIFO_CTRL_REG_XM );
    registerCache |= FIFO_CTRL_REG_XM_FM_STREAM | FIFO_CTRL_REG_XM_THRSH_MSK;
    
    writeRegister( Sensor_t::ACCEL, FIFO_CTRL_REG_XM, registerCache );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Create a running average of the bias over time
    ------------------------------------------------*/
    for ( uint8_t x = 0; x < 5; x++ )
    {
      samples = readRegister( Sensor_t::ACCEL, FIFO_SRC_REG ) & FIFO_CTRL_REG_XM_THRSH_MSK;

      /*------------------------------------------------
      Read the data stored in the FIFO
      ------------------------------------------------*/
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

      for ( uint32_t i = 0; i < samples; i++ )
      {
        readCmd( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );
        rawAccelBias[ X_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 2 ] << 8 ) ) | rcv_pkt[ 1 ] );
        rawAccelBias[ Y_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 4 ] << 8 ) ) | rcv_pkt[ 3 ] );
        rawAccelBias[ Z_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 6 ] << 8 ) ) | rcv_pkt[ 5 ] );
      }
      
      /*------------------------------------------------
      Average all the data up to this point
      ------------------------------------------------*/
      if ( samples )
      {
        rawAccelBias[ X_Axis ] /= samples + 1;
        rawAccelBias[ Y_Axis ] /= samples + 1;
        rawAccelBias[ Z_Axis ] /= samples + 1;
      }

      Chimera::delayMilliseconds( calibrationDelay );
    }

    /*------------------------------------------------
    Cache the converted bias data for later use
    ------------------------------------------------*/
    settings.zeroOffset.sensor0 = convertRawAccel( rawAccelBias );
    settings.zeroOffset.sensor0[ Z_Axis ] += Chimera::Physics::GRAVITY;

    /*------------------------------------------------
    Disable the FIFO and then wait for it to take effect
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::ACCEL, CTRL_REG0_XM );
    registerCache &= ~CTRL_REG0_XM_FIFO_EN;
    
    writeRegister( Sensor_t::ACCEL, CTRL_REG0_XM, registerCache & CTRL_REG0_XM_MSK );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Set the FIFO back to bypass mode and clear the watermark
    ------------------------------------------------*/
    writeRegister( Sensor_t::ACCEL, FIFO_CTRL_REG_XM, 0x00 );
  }

  void Driver::calibrateGyroZeroOffset()
  {
    using namespace Chimera::Modules::IMU;
    static constexpr size_t calibrationDelay = 20;

    /*------------------------------------------------
    Initialize everything properly
    ------------------------------------------------*/
    cmd_pkt.fill( 0 );
    Eigen::Matrix<int16_t, 3, 1> rawGyroBias;
    uint8_t samples       = 0;
    uint8_t registerCache = 0;

    /*------------------------------------------------
    Enable the FIFO and then wait for it to take effect
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::GYRO, CTRL_REG5_G );
    registerCache |= CTRL_REG5_G_FIFO_EN;
    
    writeRegister( Sensor_t::GYRO, CTRL_REG5_G, registerCache );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Enable FIFO stream mode and set watermark at 32 samples (max)
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::GYRO, FIFO_CTRL_REG_G );
    registerCache |= FIFO_CTRL_REG_G_FM_STREAM | FIFO_CTRL_REG_G_THRSH_MSK;
    
    writeRegister( Sensor_t::GYRO, FIFO_CTRL_REG_G, registerCache );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Create a running average of the bias over time
    ------------------------------------------------*/
    for ( uint8_t x = 0; x < 5; x++ )
    {
      samples = readRegister( Sensor_t::GYRO, FIFO_SRC_REG_G ) & FIFO_SRC_REG_G_FSS_MSK;

      /*------------------------------------------------
      Read the data stored in the FIFO
      ------------------------------------------------*/
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

      for ( uint32_t i = 0; i < samples; i++ )
      {
        readCmd( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 7 );
        rawGyroBias[ X_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 2 ] ) << 8 ) | rcv_pkt[ 1 ] );
        rawGyroBias[ Y_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 4 ] ) << 8 ) | rcv_pkt[ 3 ] );
        rawGyroBias[ Z_Axis ] += ( ( static_cast<int16_t>( rcv_pkt[ 6 ] ) << 8 ) | rcv_pkt[ 5 ] );
      }

      /*------------------------------------------------
      Average all the data up to this point.
      ------------------------------------------------*/
      if ( samples )
      {
        rawGyroBias[ X_Axis ] /= samples + 1;
        rawGyroBias[ Y_Axis ] /= samples + 1;
        rawGyroBias[ Z_Axis ] /= samples + 1;
      }

      Chimera::delayMilliseconds( calibrationDelay );
    }

    /*------------------------------------------------
    Cache the converted bias data for later use
    ------------------------------------------------*/
    settings.zeroOffset.sensor1 = convertRawGyro( rawGyroBias );

    /*------------------------------------------------
    Disable the FIFO and then wait for it to take effect
    ------------------------------------------------*/
    registerCache = readRegister( Sensor_t::GYRO, CTRL_REG5_G );
    registerCache &= ~CTRL_REG5_G_FIFO_EN;
    
    writeRegister( Sensor_t::GYRO, CTRL_REG5_G, registerCache );
    Chimera::delayMilliseconds( calibrationDelay );

    /*------------------------------------------------
    Set the FIFO back to bypass mode and clear the watermark
    ------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, FIFO_CTRL_REG_G, 0x00 );
  }

  void Driver::calibrateMag()
  {
    // Currently not supported
    settings.zeroOffset.sensor2[ 0 ] = 0.0f;
    settings.zeroOffset.sensor2[ 1 ] = 0.0f;
    settings.zeroOffset.sensor2[ 2 ] = 0.0f;
  }

  void Driver::updateAccelResolution()
  {
    /*------------------------------------------------
    Resolution in milli-g's per ADC tick; Datasheet Pg.13
    ------------------------------------------------*/
    switch ( settings.scale.accel )
    {
      case A_SCALE_2G:
        settings.resolution.accel = 0.061f;
        break;

      case A_SCALE_4G:
        settings.resolution.accel = 0.122f;
        break;

      case A_SCALE_6G:
        settings.resolution.accel = 0.183f;
        break;

      case A_SCALE_8G:
        settings.resolution.accel = 0.244f;
        break;

      case A_SCALE_16G:
        settings.resolution.accel = 0.732f;
        break;
    }
  }

  void Driver::updateMagResolution()
  {
    /*------------------------------------------------
    Resolution in milli-gauss's per ADC tick; Datasheet Pg.13
    ------------------------------------------------*/
    switch ( settings.scale.mag )
    {
      case MagnetometerScale::M_SCALE_2GS:
        settings.resolution.mag = 0.08f;
        break;

      case MagnetometerScale::M_SCALE_4GS:
        settings.resolution.mag = 0.16f;
        break;

      case MagnetometerScale::M_SCALE_8GS:
        settings.resolution.mag = 0.32f;
        break;

      case MagnetometerScale::M_SCALE_12GS:
        settings.resolution.mag = 0.48f;
        break;
    }
  }

  void Driver::updateGyroResolution()
  {
    /*------------------------------------------------
    Resolution in milli-dps's per ADC tick; Datasheet Pg.13
    ------------------------------------------------*/
    switch ( settings.scale.gyro )
    {
      case GyroscopeScale::G_SCALE_245DPS:
        settings.resolution.gyro = 8.75f;
        break;

      case GyroscopeScale::G_SCALE_500DPS:
        settings.resolution.gyro = 17.50f;
        break;

      case GyroscopeScale::G_SCALE_2000DPS:
        settings.resolution.gyro = 70.00f;
        break;
    }
  }

  Eigen::Matrix<float, 3, 1> Driver::convertRawAccel( const Eigen::Ref<const Eigen::Matrix<int16_t, 3, 1>> &data )
  {
    using namespace Chimera::Physics;
    using namespace Chimera::Units;

    Eigen::Matrix<float, 3, 1> output;

    /* Acceleration (m/s2) = ( ( milli_g/ADCTick ) * ADCTick * g ) / 1000.0f */
    output[ 0 ] = ( ( settings.resolution.accel * static_cast<float>( data[ 0 ] ) * GRAVITY_ABS ) / MilliToKilo<float>::val );
    output[ 1 ] = ( ( settings.resolution.accel * static_cast<float>( data[ 1 ] ) * GRAVITY_ABS ) / MilliToKilo<float>::val );
    output[ 2 ] = ( ( settings.resolution.accel * static_cast<float>( data[ 2 ] ) * GRAVITY ) / MilliToKilo<float>::val );

    return output;
  }

  Eigen::Matrix<float, 3, 1> Driver::convertRawGyro( const Eigen::Matrix<int16_t, 3, 1> &data )
  {
    using namespace Chimera::Units;

    Eigen::Matrix<float, 3, 1> output;

    /* Rotation Rate (deg/s) = ( milli_dps per ADCTick ) * ADCTick / 1000.0f */
    output[ 0 ] = ( ( settings.resolution.gyro * static_cast<float>( data[ 0 ] ) ) / MilliToKilo<float>::val );
    output[ 1 ] = ( ( settings.resolution.gyro * static_cast<float>( data[ 1 ] ) ) / MilliToKilo<float>::val );
    output[ 2 ] = ( ( settings.resolution.gyro * static_cast<float>( data[ 2 ] ) ) / MilliToKilo<float>::val );

    return output;
  }

  Eigen::Matrix<float, 3, 1> Driver::convertRawMag( const Eigen::Ref<const Eigen::Matrix<int16_t, 3, 1>> &data )
  {
    using namespace Chimera::Units;

    Eigen::Matrix<float, 3, 1> output;

    /* Gauss (Mx/cm2) = ( milli_gauss per ADCTick ) * ADCTick / 1000.0f */
    output[ 0 ] = ( ( settings.resolution.mag * static_cast<float>( data[ 0 ] ) ) / MilliToKilo<float>::val );
    output[ 1 ] = ( ( settings.resolution.mag * static_cast<float>( data[ 1 ] ) ) / MilliToKilo<float>::val );
    output[ 2 ] = ( ( settings.resolution.mag * static_cast<float>( data[ 2 ] ) ) / MilliToKilo<float>::val );

    return output;
  }

  void Driver::writeCmd( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, size_t length )
  {
    clearChipSelect( chip );
    spi->writeBytes( cmd_buffer, length, 500 );
    setChipSelect( chip );
  }

  void Driver::readCmd( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length )
  {
    clearChipSelect( chip );
    spi->readWriteBytes( cmd_buffer, rcv_buffer, length, 500 );
    setChipSelect( chip );
  }

  void Driver::writeRegister( const Chimera::Modules::IMU::Sensor_t chip, const uint8_t reg, const uint8_t value )
  {
    cmd_pkt[ 0 ] = reg; /* By default the write bit will be zero */
    cmd_pkt[ 1 ] = value;
    writeCmd( chip, cmd_pkt.data(), 2 );
  }

  uint8_t Driver::readRegister( const Chimera::Modules::IMU::Sensor_t chip, const uint8_t reg )
  {
    cmd_pkt[ 0 ] = reg | LSM_READ_BIT;
    cmd_pkt[ 1 ] = 0x00;
    readCmd( chip, cmd_pkt.data(), rcv_pkt.data(), 2 );

    return rcv_pkt[ 1 ];
  }

  void Driver::setChipSelect( const Chimera::Modules::IMU::Sensor_t chip )
  {
    using namespace Chimera::GPIO;
    using namespace Chimera::Modules::IMU;

    switch ( chip )
    {
      case Sensor_t::ACCEL:
      case Sensor_t::MAG:
        accelMagCS->setState( State::HIGH );
        break;

      case Sensor_t::GYRO:
        gyroCS->setState( State::HIGH );

      case Sensor_t::TEMP:
      default:
        break;
    }
  }

  void Driver::clearChipSelect( const Chimera::Modules::IMU::Sensor_t chip )
  {
    using namespace Chimera::GPIO;
    using namespace Chimera::Modules::IMU;

    switch ( chip )
    {
      case Sensor_t::ACCEL:
      case Sensor_t::MAG:
        accelMagCS->setState( State::LOW );
        break;

      case Sensor_t::GYRO:
        gyroCS->setState( State::LOW );

      case Sensor_t::TEMP:
      default:
        break;
    }
  }

  void Driver::initGyro()
  {
    using namespace Chimera::Modules::IMU;

    /*-------------------------------------------------
    Enable all axis and switch into normal mode
    -------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, CTRL_REG1_G, CTRL_REG1_G_PD | CTRL_REG1_G_X_EN | CTRL_REG1_G_Y_EN | CTRL_REG1_G_Z_EN );

    /*-------------------------------------------------
    Set the high pass filter to be a medium ranged value
    -------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, CTRL_REG2_G, ( CTRL_REG2_G_HPF_MODE_NORMAL | G_HPF_LVL_4 ) & CTRL_REG2_G_MSK );

    /*-------------------------------------------------
    We aren't using interrupts, so this register is pointless
    -------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, CTRL_REG3_G, 0x00 );

    /*-------------------------------------------------
    Turn on the block data update functionality
    -------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, CTRL_REG4_G, CTRL_REG4_G_BDU );

    /*-------------------------------------------------
    Enable the high pass filter configured in REG2
    -------------------------------------------------*/
    writeRegister( Sensor_t::GYRO, CTRL_REG5_G, CTRL_REG5_G_HPF_EN );
  }

  void Driver::initAccel()
  {
    using namespace Chimera::Modules::IMU;

    /*-------------------------------------------------
    No functionality from this register is needed
    -------------------------------------------------*/
    writeRegister( Sensor_t::ACCEL, CTRL_REG0_XM, 0x00 );

    /*-------------------------------------------------
    Enable all axis
    -------------------------------------------------*/
    writeRegister( Sensor_t::ACCEL, CTRL_REG1_XM, CTRL_REG1_XM_AX_EN | CTRL_REG1_XM_AY_EN | CTRL_REG1_XM_AZ_EN );

    /*-------------------------------------------------
    Functionality will be configured by the user. Defaults
    to 4-wire SPI mode, which is pretty typical.
    -------------------------------------------------*/
    writeRegister( Sensor_t::ACCEL, CTRL_REG2_XM, 0x00 );

    /*-------------------------------------------------
    Interrupts are unsupported in this software
    -------------------------------------------------*/
    writeRegister( Sensor_t::ACCEL, CTRL_REG3_XM, 0x00 );
  }

  void Driver::initMag()
  {
    using namespace Chimera::Modules::IMU;

    /*-------------------------------------------------
    Only enable the temperature sensor. Everything else will
    be configured by the user.
    -------------------------------------------------*/
    writeRegister( Sensor_t::MAG, CTRL_REG5_XM, CTRL_REG5_XM_TEMP_EN | ( M_RES_HI << CTRL_REG5_XM_MRES_POS ) );

    /*-------------------------------------------------
    Only contains Mag scale settings, which is user configured
    -------------------------------------------------*/
    writeRegister( Sensor_t::MAG, CTRL_REG6_XM, 0x00 );

    /*-------------------------------------------------
    Interrupt features not supported yet
    -------------------------------------------------*/
    writeRegister( Sensor_t::MAG, CTRL_REG4_XM, 0x00 );

    /*-------------------------------------------------
    Set the sensor mode
    -------------------------------------------------*/
    uint8_t reg7Cache = readRegister( Sensor_t::MAG, CTRL_REG7_XM );
    reg7Cache &= ~CTRL_REG7_XM_MD_MSK;
    reg7Cache |= ( M_MODE_NORMAL0 << CTRL_REG7_XM_MD_POS );

    writeRegister( Sensor_t::MAG, CTRL_REG7_XM, reg7Cache & CTRL_REG7_XM_MSK );

    /*-------------------------------------------------
    Interrupt features not supported yet
    -------------------------------------------------*/
    writeRegister( Sensor_t::MAG, INT_CTRL_REG_M, 0x00 );
  }

}    // namespace LSM9DS0
