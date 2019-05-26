/********************************************************************************
 *   File Name:
 *       lsm9ds0.cpp
 *
 *   Description:
 *       Implements the driver for the LSM9DS0
 *
 *   2019 | Brandon Braun | brandonbraun653@gmail.com
 ********************************************************************************/

#include <lsm9ds0/lsm9ds0.hpp>


namespace LSM9DS0
{
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
      
      gyroCS->setMode(Drive::OUTPUT_PUSH_PULL, true );
      gyroCS->setState( State::HIGH );

      setup.mode     = Mode::MASTER;
      setup.bitOrder = BitOrder::MSB_FIRST;
      setup.dataSize = DataSize::SZ_8BIT;
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
      Attempt to find the  Sensor_t::ACCELerometer/magnetometer sensor
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
      setODR( Sensor_t::GYRO, settings.odr.gyro );
      setScaling( Sensor_t::GYRO, settings.scale.gyro );

      init_accel();
      setODR( Sensor_t::ACCEL, settings.odr. Sensor_t::ACCEL );
      setScaling( Sensor_t::ACCEL, settings.scale. Sensor_t::ACCEL );

      init_mag();
      setODR( Sensor_t::MAG, settings.odr.mag );
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
      HAL_Delay( 20 );

      // Enable Sensor_t::GYRO FIFO stream mode and set watermark at 32 samples
      cmd_pkt[ 0 ] = FIFO_CTRL_REG_G;
      cmd_pkt[ 1 ] = 0x20 | 0x1F;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
      HAL_Delay( 1000 );

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
      settings.bias.gyro.x = ( float )_gyro_bias[ 0 ] * settings.res.gyro;
      settings.bias.gyro.y = ( float )_gyro_bias[ 1 ] * settings.res.gyro;
      settings.bias.gyro.z = ( float )_gyro_bias[ 2 ] * settings.res.gyro;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG5_G | LSM_READ_BIT;
      read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Disable the FIFO and then wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG5_G;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] & ~0x40;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
      HAL_Delay( 20 );

      // Enable Sensor_t::GYRO bypass mode
      cmd_pkt[ 0 ] = FIFO_CTRL_REG_G;
      cmd_pkt[ 1 ] = 0x00;
      write_pkt( Sensor_t::GYRO, cmd_pkt.data(), 2 );
    }

    if ( chip ==  Sensor_t::ACCEL )
    {
      cmd_pkt.fill( 0 );
      uint16_t _accel_bias[ 3 ] = { 0, 0, 0 };
      uint8_t samples;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG0_XM | LSM_READ_BIT;
      read_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Enable  Sensor_t::ACCELerometer FIFO and wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG0_XM;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] | 0x40;
      write_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      HAL_Delay( 20 );

      // Enable  Sensor_t::ACCELerometer FIFO stream mode and set watermark at 32 samples
      cmd_pkt[ 0 ] = FIFO_CTRL_REG;
      cmd_pkt[ 1 ] = 0x20 | 0x1F;
      write_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      HAL_Delay( 1000 );

      // Read number of stored samples
      cmd_pkt[ 0 ] = FIFO_SRC_REG | LSM_READ_BIT;
      cmd_pkt[ 1 ] = 0x00;
      read_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );
      samples = ( rcv_pkt[ 1 ] & 0x1F );

      // Read the  Sensor_t::ACCELerometer data stored in the FIFO
      cmd_pkt.fill( 0 );
      cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;

      for ( uint32_t i = 0; i < samples; i++ )
      {
        read_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 7 );
        _accel_bias[ 0 ] += ( ( ( int16_t )rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ] );
        _accel_bias[ 1 ] += ( ( ( int16_t )rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ] );
        _accel_bias[ 2 ] += ( ( ( int16_t )rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ] ) -
                            ( int16_t )( 1.0f / settings.res. Sensor_t::ACCEL );    // Assumes sensor facing up!
      }

      // average the data
      _accel_bias[ 0 ] /= samples;
      _accel_bias[ 1 ] /= samples;
      _accel_bias[ 2 ] /= samples;

      // Properly scale data to get gs
      settings.bias. Sensor_t::ACCEL.x = ( float )_accel_bias[ 0 ] * settings.res. Sensor_t::ACCEL;
      settings.bias. Sensor_t::ACCEL.y = ( float )_accel_bias[ 1 ] * settings.res. Sensor_t::ACCEL;
      settings.bias. Sensor_t::ACCEL.z = ( float )_accel_bias[ 2 ] * settings.res. Sensor_t::ACCEL;

      // Grab the current register settings
      cmd_pkt[ 0 ] = CTRL_REG0_XM | LSM_READ_BIT;
      read_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 2 );

      // Disable the FIFO and then wait for it to take effect
      cmd_pkt[ 0 ] = CTRL_REG0_XM;
      cmd_pkt[ 1 ] = rcv_pkt[ 1 ] & ~0x40;
      write_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), 2 );
      HAL_Delay( 20 );

      // Enable  Sensor_t::ACCELerometer bypass mode
      cmd_pkt[ 0 ] = FIFO_CTRL_REG;
      cmd_pkt[ 1 ] = 0x00;
      write_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), 2 );
    }
  }

  Chimera::Status_t Driver::measure( const Chimera::Modules::IMU::Sensor_t chip, void *const measurement, const size_t size )
  {
    using namespace Chimera::Modules::IMU;
    
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    switch ( chip )
    {
      case  Sensor_t::ACCEL:
        // Read out 6 bytes of data, starting from OUT_X_L_A
        cmd_pkt[ 0 ] = OUT_X_L_A | LSM_READ_BIT | LSM_AUTO_INCR_BIT;    //
        read_pkt(  Sensor_t::ACCEL, cmd_pkt.data(), rcv_pkt.data(), 7 );                         // First byte received is garbage. Ignore.

        rawData.sensor0.x = ( rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ];
        rawData.sensor0.y = ( rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ];
        rawData.sensor0.z = ( rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ];
        break;

      case Sensor_t::GYRO:
        // Read out 6 bytes of data, starting from OUT_X_L_G
        cmd_pkt[ 0 ] = OUT_X_L_G | LSM_READ_BIT | LSM_AUTO_INCR_BIT;    //
        read_pkt( Sensor_t::GYRO, cmd_pkt.data(), rcv_pkt.data(), 7 );                          // First byte received is garbage. Ignore.

        rawData.sensor1.x = ( rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ];
        rawData.sensor1.y = ( rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ];
        rawData.sensor1.z = ( rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ];
        break;

      case Sensor_t::MAG:
        // Read out 6 bytes of data, starting from OUT_X_L_M
        cmd_pkt[ 0 ] = OUT_X_L_M | LSM_READ_BIT | LSM_AUTO_INCR_BIT;    //
        read_pkt( Sensor_t::MAG, cmd_pkt.data(), rcv_pkt.data(), 7 );                           // First byte received is garbage. Ignore.

        rawData.sensor2.x = ( rcv_pkt[ 2 ] << 8 ) | rcv_pkt[ 1 ];
        rawData.sensor2.y = ( rcv_pkt[ 4 ] << 8 ) | rcv_pkt[ 3 ];
        rawData.sensor2.z = ( rcv_pkt[ 6 ] << 8 ) | rcv_pkt[ 5 ];
        break;

      case Sensor_t::TEMP:
        // Read out 2 bytes of data, starting from OUT_TEMP_L_XM
        cmd_pkt[ 0 ] = OUT_TEMP_L_XM | LSM_READ_BIT | LSM_AUTO_INCR_BIT;    //
        read_pkt( Sensor_t::TEMP, cmd_pkt.data(), rcv_pkt.data(), 3 );                              // First byte received is garbage. Ignore.

        // temperature_data = (((int16_t)rcv_pkt[2] << 12) | rcv_pkt[1] << 4) >> 4; // Temperature is a 12-bit signed integer
        break;

      default:
        break;
    }
  }

  Chimera::Status_t Driver::attachHW( Chimera::SPI::SPIClass_sPtr &spi, Chimera::GPIO::GPIOClass_sPtr &accelMagCS,
                                      Chimera::GPIO::GPIOClass_sPtr &gyroCS )
  {
    
  }

  Chimera::Status_t Driver::attachSettings( const Settings &settings )
  {
    
  }

  Chimera::Status_t Driver::setScaling( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    void Driver::setScale_gyro( Sensor_t::GYRO_scale gScl )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG4_G | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt( Sensor_t::GYRO, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the Sensor_t::GYRO scale bits and shift in new scale bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0x3 << 4 );
    rcv_pkt[ 1 ] |= gScl << 4;

    // And write the new register value back into CTRL_REG4_G:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt( Sensor_t::GYRO, cmd_pkt, 2 );

    // Update class variables and calculate a new resolution
    settings.scale.gyro = gScl;
    calc_gRes();
  }

  void Driver::setScale_ Sensor_t::ACCEL(  Sensor_t::ACCEL_scale aScl )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG2_XM | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt(  Sensor_t::ACCEL, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the  Sensor_t::ACCEL scale bits and shift in new scale bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0x3 << 3 );
    rcv_pkt[ 1 ] |= aScl << 3;

    // And write the new register value back into CTRL_REG2_XM:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt(  Sensor_t::ACCEL, cmd_pkt, 2 );

    // Update class variable and calculate a new resolution
    settings.scale. Sensor_t::ACCEL = aScl;
    calc_aRes();
  }

  void Driver::setScale_mag( mag_scale mScl )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG6_XM | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt( MAG, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the  Sensor_t::ACCEL scale bits and shift in new scale bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0x3 << 5 );
    rcv_pkt[ 1 ] |= mScl << 5;

    // And write the new register value back into CTRL_REG6_XM:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt( MAG, cmd_pkt, 2 );

    // Update class variable and calculate a new resolution
    settings.scale.mag = mScl;
    calc_mRes();
  }
  }

  Chimera::Status_t Driver::setODR( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
    void Driver::setODR_gyro( Sensor_t::GYRO_odr gODR )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG1_G | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt( Sensor_t::GYRO, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the Sensor_t::GYRO ODR bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0xF << 4 );
    rcv_pkt[ 1 ] |= ( gODR << 4 );

    // And write the new register value back into CTRL_REG1_G:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt( Sensor_t::GYRO, cmd_pkt, 2 );

    // Update class variable
    settings.odr.gyro = gODR;
  }

  void Driver::setODR_ Sensor_t::ACCEL(  Sensor_t::ACCEL_odr aODR )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG1_XM | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt(  Sensor_t::ACCEL, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the  Sensor_t::ACCEL ODR bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0xF << 4 );
    rcv_pkt[ 1 ] |= ( aODR << 4 );

    // And write the new register value back into CTRL_REG1_XM:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt(  Sensor_t::ACCEL, cmd_pkt, 2 );

    // Update class variable
    settings.odr. Sensor_t::ACCEL = aODR;
  }

  void Driver::setODR_mag( mag_odr mODR )
  {
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG5_XM | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt( MAG, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the mag ODR bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0x7 << 2 );
    rcv_pkt[ 1 ] |= ( mODR << 2 );

    // And write the new register value back into CTRL_REG5_XM:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt( MAG, cmd_pkt, 2 );

    // Update class variable
    settings.odr.mag = mODR;
  }
  }

  Chimera::Status_t Driver::setABW( const Chimera::Modules::IMU::Sensor_t sensor, const uint8_t value )
  {
     
    cmd_pkt.fill( 0 );
    rcv_pkt.fill( 0 );

    // We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
    cmd_pkt[ 0 ] = CTRL_REG2_XM | LSM_READ_BIT;    // Ensure we ask for a read
    read_pkt(  Sensor_t::ACCEL, cmd_pkt, rcv_pkt, 2 );

    // Then mask out the  Sensor_t::ACCEL ABW bits:
    rcv_pkt[ 1 ] &= 0xFF ^ ( 0x3 << 7 );
    rcv_pkt[ 1 ] |= ( aBW << 7 );

    // And write the new register value back into CTRL_REG2_XM:
    cmd_pkt[ 0 ] &= ~LSM_READ_BIT;    // Ensure we are writing
    cmd_pkt[ 1 ] = rcv_pkt[ 1 ];
    write_pkt(  Sensor_t::ACCEL, cmd_pkt, 2 );

    // Update class variable
    settings.abw_ Sensor_t::ACCEL = aBW;

  }
  
  
  
  void Driver::calc_aRes()
  {
    // Possible  Sensor_t::ACCELerometer scales (and their register bit settings) are:
    // 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
    // algorithm to calculate g/(ADC tick) based on that 3-bit value:
    settings.res. Sensor_t::ACCEL =
        settings.scale. Sensor_t::ACCEL == A_SCALE_16G ? 16.0f / 32768.0f : ( ( ( float )settings.scale. Sensor_t::ACCEL + 1.0f ) * 2.0f ) / 32768.0f;
  }

  void Driver::calc_mRes()
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
    // to calculate Gs/(ADC tick) based on that 2-bit value:
    settings.res.mag = settings.scale.mag == M_SCALE_2GS ? 2.0f / 32768.0f : ( float )( settings.scale.mag << 2 ) / 32768.0f;
  }

  void Driver::calc_gRes()
  {
    // Possible Sensor_t::GYRO scales (and their register bit settings) are:
    // 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
    // to calculate DPS/(ADC tick) based on that 2-bit value:
    switch ( settings.scale.gyro )
    {
      case G_SCALE_245DPS:
        settings.res.gyro = 245.0f / 32768.0f;
        break;
      case G_SCALE_500DPS:
        settings.res.gyro = 500.0f / 32768.0f;
        break;
      case G_SCALE_2000DPS:
        settings.res.gyro = 2000.0f / 32768.0f;
        break;
    }
  }

  void Driver::calc_gyro()
  {
    data.sensor1.x = ( settings.res.gyro * ( float )rawData.sensor1.x );
    data.sensor1.y = ( settings.res.gyro * ( float )rawData.sensor1.y );
    data.sensor1.z = ( settings.res.gyro * ( float )rawData.sensor1.z );
  }

  void Driver::calc_accel()
  {
    data.sensor0.x = ( settings.res. Sensor_t::ACCEL * ( float )rawData.sensor0.x * 9.8f );
    data.sensor0.y = ( settings.res. Sensor_t::ACCEL * ( float )rawData.sensor0.y * 9.8f );
    data.sensor0.z = ( settings.res. Sensor_t::ACCEL * ( float )rawData.sensor0.z * 9.8f );
  }

  void Driver::calc_mag()
  {
    data.sensor2.x = ( settings.res.mag * ( float )rawData.sensor2.x );
    data.sensor2.y = ( settings.res.mag * ( float )rawData.sensor2.y );
    data.sensor2.z = ( settings.res.mag * ( float )rawData.sensor2.z );
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

    write_pkt(  Sensor_t::ACCEL, &cmd_pkt[ 0 ], 2 );
    write_pkt(  Sensor_t::ACCEL, &cmd_pkt[ 2 ], 2 );
    write_pkt(  Sensor_t::ACCEL, &cmd_pkt[ 4 ], 2 );
    write_pkt(  Sensor_t::ACCEL, &cmd_pkt[ 6 ], 2 );
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
