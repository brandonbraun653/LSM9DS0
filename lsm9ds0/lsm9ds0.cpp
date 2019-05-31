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

/* Driver Includes */
#include <lsm9ds0/lsm9ds0.hpp>


namespace LSM9DS0
{
  static constexpr uint8_t LSM_READ_BIT      = 1u << 7;
  static constexpr uint8_t LSM_WRITE_BIT     = static_cast<uint8_t>( ~LSM_READ_BIT );
  static constexpr uint8_t LSM_AUTO_INCR_BIT = 1u << 6;

  Driver::Driver() : hardwareAttached( false ), settingsAttached( false )
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

    if ( !hardwareAttached || !settingsAttached )
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

      setup.mode      = Mode::MASTER;
      setup.bitOrder  = BitOrder::MSB_FIRST;
      setup.dataSize  = DataSize::SZ_8BIT;
      setup.clockMode = ClockMode::MODE0;

      if ( setup.clockFrequency > MAX_SPI_CLOCK )
      {
        setup.clockFrequency = SLOW_SPI_CLOCK;
      }

      spi->init( setup );
      spi->setChipSelectControlMode( ChipSelectMode::MANUAL );
      spi->setPeripheralMode( SubPeripheral::TXRX, setup.transferMode );

      /*------------------------------------------------
      Attempt to find the Sensor_t::GYROscope sensor
      ------------------------------------------------*/
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = WHO_AM_I_G | LSM_READ_BIT;
      read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

      if ( rcv_pkt[ 1 ] != 0xD4 )
      {
        sensorPresent = false;
        result        = Chimera::CommonStatusCodes::FAIL;
      }

      /*------------------------------------------------
      Attempt to find the accelerometer/magnetometer sensor
      ------------------------------------------------*/
      cmd_pkt[ 0 ] = WHO_AM_I_XM | LSM_READ_BIT;
      read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      if ( rcv_pkt[ 1 ] != 0x49 )
      {
        sensorPresent = false;
        result        = Chimera::CommonStatusCodes::FAIL;
      }
      
      calc_gRes();    // Calculate DPS / ADC tick
      calc_mRes();    // Calculate  Gs / ADC tick
      calc_aRes();    // Calculate   g / ADC tick

      init_gyro();
      setODR( Sensor_t::GYRO, settings.outputDataRate.gyro );
      setScaling( Sensor_t::GYRO, settings.scale.gyro );

      init_accel();
      setODR( Sensor_t::ACCEL, settings.outputDataRate.accel );
      setScaling( Sensor_t::ACCEL, settings.scale.accel );

      init_mag();
      setODR( Sensor_t::MAG, settings.outputDataRate.mag );
      setScaling( Sensor_t::MAG, settings.scale.mag );
    }

    return result;
  }


  Chimera::Status_t Driver::calibrate( const Chimera::Modules::IMU::Sensor_t chip )
  {
    using namespace Chimera::Modules::IMU;

    if ( chip == Sensor_t::GYRO )
    {
      cmd_pkt.fill( 0 );
      int16_t _gyro_bias[ 3 ] = { 0, 0, 0 };
      uint8_t samples;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG5_G | LSM_READ_BIT;
      read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Enable the FIFO and then wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG5_G;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] | 0x40;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 20 );

      // Enable Sensor_t::GYRO FIFO stream mode and set watermark at 32 samples
      cmd_pkt[ 0 ] = FIFO_CTRL_REG_G;
      cmd_pkt[ 1 ] = 0x20 | 0x1F;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 1000 );

      // Read number of stored samples
      cmd_pkt[ 0 ] = FIFO_SRC_REG_G | LSM_READ_BIT;
      cmd_pkt[ 1 ] = 0x00;
      read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );
      samples = ( rcv_pkt[ 1 ] & 0x1F );

      // Read the Sensor_t::GYRO data stored in the FIFO
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

      for ( uint32_t i = 0; i < samples; i++ )
      {
        read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 7 );
        _gyro_bias[ 0 ] += ( ( ( int16_t )rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ] );
        _gyro_bias[ 1 ] += ( ( ( int16_t )rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ] );
        _gyro_bias[ 2 ] += ( ( ( int16_t )rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ] );
      }

      // average the data
      _gyro_bias[ 0 ] /= samples;
      _gyro_bias[ 1 ] /= samples;
      _gyro_bias[ 2 ] /= samples;

      // Properly scale the data to get deg/s
      settings.bias.sensor1.x = ( float )_gyro_bias[ 0 ] * settings.resolution.gyro;
      settings.bias.sensor1.y = ( float )_gyro_bias[ 1 ] * settings.resolution.gyro;
      settings.bias.sensor1.z = ( float )_gyro_bias[ 2 ] * settings.resolution.gyro;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG5_G | LSM_READ_BIT;
      read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Disable the FIFO and then wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG5_G;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] & ~0x40;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 20 );

      // Enable Sensor_t::GYRO bypass mode
      cmd_pkt[ 0 ] = FIFO_CTRL_REG_G;
      cmd_pkt[ 1 ] = 0x00;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
    }

    if ( chip == Sensor_t::ACCEL )
    {
      cmd_pkt.fill( 0 );
      uint16_t _accel_bias[ 3 ] = { 0, 0, 0 };
      uint8_t samples;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG0_XM | LSM_READ_BIT;
      read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Enable  Sensor_t::ACCELerometer FIFO and wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG0_XM;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] | 0x40;
      write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 20 );

      // Enable  Sensor_t::ACCELerometer FIFO stream mode and set watermark at 32 samples
      cmd_pkt[ 0 ] = FIFO_CTRL_REG;
      cmd_pkt[ 1 ] = 0x20 | 0x1F;
      write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 1000 );

      // Read number of stored samples
      cmd_pkt[ 0 ] = FIFO_SRC_REG | LSM_READ_BIT;
      cmd_pkt[ 1 ] = 0x00;
      read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );
      samples = ( rcv_pkt[ 1 ] & 0x1F );

      // Read the  Sensor_t::ACCELerometer data stored in the FIFO
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

      for ( uint32_t i = 0; i < samples; i++ )
      {
        read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 7 );
        _accel_bias[ 0 ] += ( ( ( int16_t )rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ] );
        _accel_bias[ 1 ] += ( ( ( int16_t )rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ] );
        _accel_bias[ 2 ] += ( ( ( int16_t )rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ] ) -
                            ( int16_t )( 1.0f / settings.resolution.accel );    // Assumes sensor facing up!
      }

      // average the data
      _accel_bias[ 0 ] /= samples;
      _accel_bias[ 1 ] /= samples;
      _accel_bias[ 2 ] /= samples;

      // Properly scale data to get gs
      settings.bias.sensor0.x = ( float )_accel_bias[ 0 ] * settings.resolution.accel;
      settings.bias.sensor0.y = ( float )_accel_bias[ 1 ] * settings.resolution.accel;
      settings.bias.sensor0.z = ( float )_accel_bias[ 2 ] * settings.resolution.accel;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG0_XM | LSM_READ_BIT;
      read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Disable the FIFO and then wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG0_XM;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] & ~0x40;
      write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      Chimera::delayMilliseconds( 20 );

      // Enable  Sensor_t::ACCELerometer bypass mode
      cmd_pkt[ 0 ] = FIFO_CTRL_REG;
      cmd_pkt[ 1 ] = 0x00;
      write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );
    }
  }

  Chimera::Status_t Driver::reset()
  {
    return Chimera::CommonStatusCodes::NOT_SUPPORTED;
  }

  Chimera::Status_t assignDOF( const Chimera::Modules::IMU::Sensor_t sensor0, const Chimera::Modules::IMU::Sensor_t sensor1,
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
    constexpr size_t IMU_READ_LEN = 7;
    constexpr size_t TEMP_READ_LEN = 3;
    constexpr size_t SHIFT = std::numeric_limits<uint8_t>::digits;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    switch ( chip )
    {
      case Sensor_t::ACCEL:
        cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
        read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

        rawData.sensor0.x = static_cast<int16_t>(( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ]);
        rawData.sensor0.y = static_cast<int16_t>(( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ]);
        rawData.sensor0.z = static_cast<int16_t>(( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ]);
        break;

      case Sensor_t::GYRO:
        cmd_pkt[ 0 ] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
        read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

        rawData.sensor1.x = static_cast<int16_t>(( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ]);
        rawData.sensor1.y = static_cast<int16_t>(( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ]);
        rawData.sensor1.z = static_cast<int16_t>(( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ]);
        break;

      case Sensor_t::MAG:
        cmd_pkt[ 0 ] = OUT_X_L_M | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
        read_pkt( Sensor_t::MAG, cmd_pkt.data(), rcv_pkt.data(), IMU_READ_LEN );

        rawData.sensor2.x = static_cast<int16_t>(( rcv_pkt[ 2 ] << SHIFT ) | rcv_pkt[ 1 ]);
        rawData.sensor2.y = static_cast<int16_t>(( rcv_pkt[ 4 ] << SHIFT ) | rcv_pkt[ 3 ]);
        rawData.sensor2.z = static_cast<int16_t>(( rcv_pkt[ 6 ] << SHIFT ) | rcv_pkt[ 5 ]);
        break;

      case Sensor_t::TEMP:
        cmd_pkt[ 0 ] = OUT_TEMP_L_XM | LSM_READ_BIT | LSM_AUTO_INCR_BIT;
        read_pkt( Sensor_t::TEMP, cmd_pkt.data(), rcv_pkt.data(), TEMP_READ_LEN );

        // temperature_data = (((int16_t)rcv_pkt[2] << 12) | rcv_pkt[1] << 4) >> 4; // Temperature is a 12-bit signed integer
        break;

      default:
        break;
    }

    return result;
  }

  Chimera::Status_t Driver::attachHW( Chimera::SPI::SPIClass_sPtr &spi, Chimera::GPIO::GPIOClass_sPtr &accelMagCS,
                                      Chimera::GPIO::GPIOClass_sPtr &gyroCS )
  {
    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    this->spi = spi;
    this->accelMagCS = accelMagCS;
    this->gyroCS = gyroCS;
    hardwareAttached = true;

    return result;
  }

  Chimera::Status_t Driver::attachSettings( const Settings &settings )
  {
    this->settings = settings;
    settingsAttached = true;
    return Chimera::CommonStatusCodes::OK;
  }

  Chimera::Status_t Driver::setScaling( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    using namespace Chimera::Modules::IMU;

    Chimera::Status_t result = Chimera::CommonStatusCodes::OK;

    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    switch ( sensor )
    {
      case Sensor_t::ACCEL:
        cmd_pkt[ 0 ] = CTRL_REG2_XM | LSM_READ_BIT;
        read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG2_XM_SCL_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG2_XM_SCL_POS ) & CTRL_REG2_XM_SCL_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );

        settings.scale.accel = static_cast<AccelerometerScale>( value );
        calc_aRes();
        break;

      case Sensor_t::GYRO:
        cmd_pkt[ 0 ] = CTRL_REG4_G | LSM_READ_BIT;
        read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG4_G_SCL_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG4_G_SCL_POS ) & CTRL_REG4_G_SCL_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );

        settings.scale.gyro = static_cast<GyroscopeScale>( value );
        calc_gRes();
        break;

      case Sensor_t::MAG:
        cmd_pkt[ 0 ] = CTRL_REG6_XM | LSM_READ_BIT;
        read_pkt( Sensor_t::MAG, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG6_XM_SCL_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG6_XM_SCL_POS ) & CTRL_REG6_XM_SCL_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::MAG, cmd_pkt.data(), 2 );

        settings.scale.mag = static_cast<MagnetometerScale>( value );
        calc_mRes();
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

    switch ( sensor )
    {
      case Sensor_t::ACCEL:
        cmd_pkt[ 0 ] = CTRL_REG1_XM | LSM_READ_BIT;
        read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG1_XM_ODR_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG1_XM_ODR_POS ) & CTRL_REG1_XM_ODR_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );

        settings.outputDataRate.accel = static_cast<AccelerometerOuputDataRate>( value );
        break;

      case Sensor_t::GYRO:
        cmd_pkt[ 0 ] = CTRL_REG1_G | LSM_READ_BIT;
        read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG1_G_ODR_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG1_G_ODR_POS ) & CTRL_REG1_G_ODR_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );

        settings.outputDataRate.gyro = static_cast<GyroscopeOutputDataRate>( value );
        break;

      case Sensor_t::MAG:
        cmd_pkt[ 0 ] = CTRL_REG5_XM | LSM_READ_BIT;
        read_pkt( Sensor_t::MAG, cmd_pkt.data(), rcv_pkt.data(), 2 );

        rcv_pkt[ 1 ] &= ~CTRL_REG5_XM_ODR_MSK;
        rcv_pkt[ 1 ] |= ( value << CTRL_REG5_XM_ODR_POS ) & CTRL_REG5_XM_ODR_MSK;

        cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
        cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
        write_pkt( Sensor_t::MAG, cmd_pkt.data(), 2 );

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

      cmd_pkt[ 0 ] = CTRL_REG2_XM | LSM_READ_BIT;
      read_pkt( Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      rcv_pkt[ 1 ] &= ~CTRL_REG2_XM_ABW_MSK;
      rcv_pkt[ 1 ] |= ( value << CTRL_REG2_XM_ABW_POS ) & CTRL_REG2_XM_ABW_MSK;

      cmd_pkt[ 0 ] &= LSM_WRITE_BIT;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
      write_pkt( Sensor_t::ACCEL, cmd_pkt.data(), 2 );

      settings.aaFilterBW = value;
      result              = Chimera::CommonStatusCodes::OK;
    }

    return result;
  }

  void Driver::calc_aRes()
  {
    settings.resolution.accel =
        settings.scale.accel == A_SCALE_16G ? 16.0f / 32768.0f : ( ( ( float )settings.scale.accel + 1.0f ) * 2.0f ) / 32768.0f;
  }

  void Driver::calc_mRes()
  {
    settings.resolution.mag =
        settings.scale.mag == M_SCALE_2GS ? 2.0f / 32768.0f : ( float )( settings.scale.mag << 2 ) / 32768.0f;
  }

  void Driver::calc_gRes()
  {
    switch ( settings.scale.gyro )
    {
      case G_SCALE_245DPS:
        settings.resolution.gyro = 245.0f / 32768.0f;
        break;
      case G_SCALE_500DPS:
        settings.resolution.gyro = 500.0f / 32768.0f;
        break;
      case G_SCALE_2000DPS:
        settings.resolution.gyro = 2000.0f / 32768.0f;
        break;
    }
  }

  void Driver::calc_gyro()
  {
    data.sensor1.x = ( settings.resolution.gyro * ( float )rawData.sensor1.x );
    data.sensor1.y = ( settings.resolution.gyro * ( float )rawData.sensor1.y );
    data.sensor1.z = ( settings.resolution.gyro * ( float )rawData.sensor1.z );
  }

  void Driver::calc_accel()
  {
    data.sensor0.x = ( settings.resolution.accel * ( float )rawData.sensor0.x * 9.8f );
    data.sensor0.y = ( settings.resolution.accel * ( float )rawData.sensor0.y * 9.8f );
    data.sensor0.z = ( settings.resolution.accel * ( float )rawData.sensor0.z * 9.8f );
  }

  void Driver::calc_mag()
  {
    data.sensor2.x = ( settings.resolution.mag * ( float )rawData.sensor2.x );
    data.sensor2.y = ( settings.resolution.mag * ( float )rawData.sensor2.y );
    data.sensor2.z = ( settings.resolution.mag * ( float )rawData.sensor2.z );
  }

  void Driver::write_pkt( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, size_t length )
  {
    clr_CS( chip );
    spi->writeBytes( cmd_buffer, length, 500 );
    set_CS( chip );
  }

  void Driver::read_pkt( const Chimera::Modules::IMU::Sensor_t chip, uint8_t *cmd_buffer, uint8_t *rcv_buffer, size_t length )
  {
    clr_CS( chip );
    spi->readWriteBytes( cmd_buffer, rcv_buffer, length, 500 );
    set_CS( chip );
  }

  void Driver::set_CS( const Chimera::Modules::IMU::Sensor_t chip )
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

  void Driver::clr_CS( const Chimera::Modules::IMU::Sensor_t chip )
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

  void Driver::init_gyro()
  {
    using namespace Chimera::Modules::IMU;

    cmd_pkt.fill( 0 );

    cmd_pkt[ 0 ] = CTRL_REG1_G;
    cmd_pkt[ 1 ] = 0x0F;
    cmd_pkt[ 2 ] = CTRL_REG2_G;
    cmd_pkt[ 3 ] = 0x00;
    cmd_pkt[ 4 ] = CTRL_REG3_G;
    cmd_pkt[ 5 ] = 0x88;
    cmd_pkt[ 6 ] = CTRL_REG4_G;
    cmd_pkt[ 7 ] = 0x00;
    cmd_pkt[ 8 ] = CTRL_REG5_G;
    cmd_pkt[ 9 ] = 0x00;

    write_pkt( Sensor_t::GYRO, &cmd_pkt[ 0 ], 2 );
    write_pkt( Sensor_t::GYRO, &cmd_pkt[ 2 ], 2 );
    write_pkt( Sensor_t::GYRO, &cmd_pkt[ 4 ], 2 );
    write_pkt( Sensor_t::GYRO, &cmd_pkt[ 6 ], 2 );
    write_pkt( Sensor_t::GYRO, &cmd_pkt[ 8 ], 2 );
  }

  void Driver::init_accel()
  {
    using namespace Chimera::Modules::IMU;

    cmd_pkt.fill( 0 );

    cmd_pkt[ 0 ] = CTRL_REG0_XM;
    cmd_pkt[ 1 ] = 0x00;
    cmd_pkt[ 2 ] = CTRL_REG1_XM;
    cmd_pkt[ 3 ] = 0x57;
    cmd_pkt[ 4 ] = CTRL_REG2_XM;
    cmd_pkt[ 5 ] = 0x00;
    cmd_pkt[ 6 ] = CTRL_REG3_XM;
    cmd_pkt[ 7 ] = 0x04;

    write_pkt( Sensor_t::ACCEL, &cmd_pkt[ 0 ], 2 );
    write_pkt( Sensor_t::ACCEL, &cmd_pkt[ 2 ], 2 );
    write_pkt( Sensor_t::ACCEL, &cmd_pkt[ 4 ], 2 );
    write_pkt( Sensor_t::ACCEL, &cmd_pkt[ 6 ], 2 );
  }

  void Driver::init_mag()
  {
    using namespace Chimera::Modules::IMU;

    cmd_pkt.fill( 0 );

    cmd_pkt[ 0 ] = CTRL_REG5_XM;
    cmd_pkt[ 1 ] = 0x94;
    cmd_pkt[ 2 ] = CTRL_REG6_XM;
    cmd_pkt[ 3 ] = 0x00;
    cmd_pkt[ 4 ] = CTRL_REG7_XM;
    cmd_pkt[ 5 ] = 0x00;
    cmd_pkt[ 6 ] = CTRL_REG4_XM;
    cmd_pkt[ 7 ] = 0x04;
    cmd_pkt[ 8 ] = INT_CTRL_REG_M;
    cmd_pkt[ 9 ] = 0x09;

    write_pkt( Sensor_t::MAG, &cmd_pkt[ 0 ], 2 );
    write_pkt( Sensor_t::MAG, &cmd_pkt[ 2 ], 2 );
    write_pkt( Sensor_t::MAG, &cmd_pkt[ 4 ], 2 );
    write_pkt( Sensor_t::MAG, &cmd_pkt[ 6 ], 2 );
    write_pkt( Sensor_t::MAG, &cmd_pkt[ 8 ], 2 );
  }

}    // namespace LSM9DS0
