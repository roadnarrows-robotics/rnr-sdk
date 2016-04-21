////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Firmware:  laeToFMux
//
// File:      Vl6180.cpp
//
/*! \file
 *
 * $LastChangedDate: 2016-02-09 15:08:50 -0700 (Tue, 09 Feb 2016) $
 * $Rev: 4308 $
 *
 * \brief The VL6180 Time-of-Flight sensor class implementation.
 *
 * This code is based on the Arduino library freely available from Sparkfun.
 * See original comment block below.
 *
 * \sa https://github.com/sparkfun/ToF_Range_Finder_Sensor-VL6180/tree/master/Libraries/Arduino
 * \sa https://www.sparkfun.com/products/12785
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * SparkFun_VL6180x.cpp
 * Library for VL6180x time of flight range finder.
 * Casey Kuhns @ SparkFun Electronics
 * 10/29/2014
 * https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
 * 
 * The VL6180x by ST micro is a time of flight range finder that
 * uses pulsed IR light to determine distances from object at close
 * range.  The average range of a sensor is between 0-200mm
 * 
 * In this file are the functions in the VL6180x class
 * 
 * Resources:
 * This library uses the Arduino Wire.h to complete I2C transactions.
 * 
 * Development environment specifics:
 * 	IDE: Arduino 1.0.5
 * 	Hardware Platform: Arduino Pro 3.3V/8MHz
 * 	VL6180x Breakout Version: 1.0
 * **Updated for Arduino 1.6.4 5/2015**
 * 
 * This code is beerware. If you see me (or any other SparkFun employee) at the
 * local pub, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ******************************************************************************/

#include <SoftwareWire.h>

#include "VL6180X.h"

VL6180x::VL6180x(SoftwareWire &wire, uint8_t address) :
    m_wire(wire), m_addr(address)
{
  m_bBlackListed  = false;
  m_bBusy         = false;

  memset(&m_ident, 0, sizeof(VL6180xIdentification));

  m_range         = 0;
  m_lux           = 0.0;

  m_bRangeNeedsTuning = false;
  m_bAlsNeedsTuning   = false;

  m_regRangeOffset    = 0;        // default
  m_regRangeCrossTalk = 0;        // default
  m_regAlsGain        = GAIN_1;   // 1.0
  m_regAlsIntPeriod   = 100;      // 100 msec
}

boolean VL6180x::ping(int tries)
{
  int   i;
  byte  d;

  m_bBusy = true;

  for(i = 0; i < tries; ++i)
  {
    d = readReg8(VL6180X_IDENTIFICATION_MODEL_ID);

    if( (d != 0) && (d != 0xff) )
    {
      m_bBusy = false;
      return true;
    }

    delay(10);
  }

  m_bBusy = false;

  return false;
}

int VL6180x::initSensor()
{
  byte data; //for temp data storage

  m_bBusy = true;

  data = readReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);

  if( data != 1 )
  {
    m_bBusy = false;
    return VL6180X_FAILURE_RESET;
  }

  //
  // Required by datasheet
  // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  //
  writeReg8(0x0207, 0x01);
  writeReg8(0x0208, 0x01);
  writeReg8(0x0096, 0x00);
  writeReg8(0x0097, 0xfd);
  writeReg8(0x00e3, 0x00);
  writeReg8(0x00e4, 0x04);
  writeReg8(0x00e5, 0x02);
  writeReg8(0x00e6, 0x01);
  writeReg8(0x00e7, 0x03);
  writeReg8(0x00f5, 0x02);
  writeReg8(0x00d9, 0x05);
  writeReg8(0x00db, 0xce);
  writeReg8(0x00dc, 0x03);
  writeReg8(0x00dd, 0xf8);
  writeReg8(0x009f, 0x00);
  writeReg8(0x00a3, 0x3c);
  writeReg8(0x00b7, 0x00);
  writeReg8(0x00bb, 0x3c);
  writeReg8(0x00b2, 0x09);
  writeReg8(0x00ca, 0x09);  
  writeReg8(0x0198, 0x01);
  writeReg8(0x01b0, 0x17);
  writeReg8(0x01ad, 0x00);
  writeReg8(0x00ff, 0x05);
  writeReg8(0x0100, 0x05);
  writeReg8(0x0199, 0x05);
  writeReg8(0x01a6, 0x1b);
  writeReg8(0x01ac, 0x3e);
  writeReg8(0x01a7, 0x1f);
  writeReg8(0x0030, 0x00);

  m_bBusy = false;

  return 0;
}

void VL6180x::writeSensorDefaults()
{
  m_bBusy = true;

  //Recommended settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

  //Enable Interrupts on Conversion Complete (any source)
  writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4) ); // Set GPIO1 high when sample complete


  writeReg8(VL6180X_SYSTEM_MODE_GPIO1, 0x10); // Set GPIO1 high when sample complete
  writeReg8(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30); //Set Avg sample period
  writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | m_regAlsGain)); // Set the ALS gain
  writeReg8(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF); // Set auto calibration period (Max = 255)/(OFF = 0)
  writeReg8(VL6180X_SYSALS_INTEGRATION_PERIOD, m_regAlsIntPeriod-1); // Set ALS integration time to 100ms
  writeReg8(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01); // perform a single temperature calibration
  //Optional settings from datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  writeReg8(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09); // Set default ranging inter-measurement period to 100ms
  writeReg8(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A); // Set default ALS inter-measurement period to 100ms
  writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24); // Configures interrupt on ‘New Sample Ready threshold event’ 
  //Additional settings defaults from community
  writeReg8(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  writeReg8(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  writeReg16(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );

  writeReg8(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD,0x30);
  writeReg8(VL6180X_FIRMWARE_RESULT_SCALER,0x01);

  m_bBusy = false;
}

void VL6180x::readIdent()
{
  m_bBusy = true;

  m_ident.idModel = readReg8(VL6180X_IDENTIFICATION_MODEL_ID);
  m_ident.idModelRevMajor = readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
  m_ident.idModelRevMinor = readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
  m_ident.idModuleRevMajor = readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
  m_ident.idModuleRevMinor = readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);

  m_ident.idDate = readReg16(VL6180X_IDENTIFICATION_DATE);
  m_ident.idTime = readReg16(VL6180X_IDENTIFICATION_TIME);

  m_bBusy = false;
}

void VL6180x::readTunes()
{
  m_regRangeOffset    = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);

  m_regRangeCrossTalk = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);

  m_regAlsGain = readReg8(VL6180X_SYSALS_ANALOGUE_GAIN);
  m_regAlsGain &= 0x2f;

  m_regAlsIntPeriod = readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD);
  m_regAlsIntPeriod += 1;
}

byte VL6180x::changeAddress(byte new_address)
{
  if( new_address == m_addr )
  {
    return m_addr;
  }

  else if( new_address > 127 )
  {
    return m_addr;
  }
   
  m_bBusy = true;

  writeReg8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_address);
   
  m_addr = readReg8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS); 

  m_bBusy = false;

  return m_addr;
}

byte VL6180x::measureRange()
{
  int   i;
  byte  status;

  m_bBusy = true;

  if( m_bRangeNeedsTuning )
  {
    tuneRangeSensor(m_newRangeOffset, m_newRangeCrossTalk);
  }

  writeReg8(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode

  delay(20);

  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  m_range = readReg8(VL6180X_RESULT_RANGE_VAL);
  status  = readReg8(VL6180X_RESULT_RANGE_STATUS);
  status >>= 4;
  
  // over/under flow (usually means no object)
  if( status != 0 )
  {
    m_range = 0xff;
  }

  m_bBusy = false;

  return m_range;
}

float VL6180x::measureAmbientLight()
{
  unsigned int  alsRaw;
  unsigned int  alsIntegrationPeriodRaw;
  float         alsIntegrationPeriod;
  float         alsGain;
  float         alsCalculated;
  byte          status;

  m_bBusy = true;

  if( m_bAlsNeedsTuning )
  {
    tuneAls(m_newAlsGain, m_newAlsIntPeriod);
  }

  // Calculate actual LUX from Appnotes

  //
  // Set the ALS gain
  //
  // First load in Gain we are using, do it everytime incase someone changes it
  // on us.
  // Note: Upper nibble shoudl be set to 0x4 i.e. for ALS gain of 1.0 write 0x46
  //
  //writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | m_regAlsGain));

  // Start ALS Measurement 
  writeReg8(VL6180X_SYSALS_START, 0x01);

  delay(m_regAlsIntPeriod+10); // give it time... 

  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  // Retrieve the Raw ALS value from the sensor
  alsRaw = readReg16(VL6180X_RESULT_ALS_VAL);
  status = readReg8(VL6180X_RESULT_ALS_STATUS);
  
  //
  // Get Integration Period for calculation, we do this everytime in case
  // someone changes it on us.
  //
  //alsIntegrationPeriodRaw = readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD);
  
  alsIntegrationPeriod = 100.0 / (float)m_regAlsIntPeriod;

  // Calculate actual LUX from Appnotes

  switch( m_regAlsGain )
  {
    case GAIN_20:   alsGain = 20.0; break;
    case GAIN_10:   alsGain = 10.32; break;
    case GAIN_5:    alsGain = 5.21; break;
    case GAIN_2_5:  alsGain = 2.60; break;
    case GAIN_1_67: alsGain = 1.72; break;
    case GAIN_1_25: alsGain = 1.28; break;
    case GAIN_1:    alsGain = 1.01; break;
    case GAIN_40:   alsGain = 40.0; break;
    default:        alsGain = 1.0;  break;
  }

  // Calculate LUX from formula in AppNotes
  m_lux = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

#if 0 // DBG code
  if( status & 0x10 )         // overflow
  {
    m_lux = 1.0;
  }
  else if( status & 0x20 )    // underflow
  {
    m_lux = 2.0;
  }
  else if( !(status & 0x01) ) // busy
  {
    m_lux = 3.0;
  }
  else                        // good
  {
    m_lux = 0.0;
  }
#endif // DBG code

  m_bBusy = false;

  return m_lux;
}

void VL6180x::markRangeForTuning(byte offset, byte crosstalk)
{
  m_newRangeOffset    = offset;
  m_newRangeCrossTalk = crosstalk;
  m_bRangeNeedsTuning = true;
}

boolean VL6180x::tuneRangeSensor(byte offset, byte crosstalk)
{
  if( offset != m_regRangeOffset )
  {
    writeReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
    m_regRangeOffset = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  }

  if( crosstalk != m_regRangeCrossTalk )
  {
    writeReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, crosstalk);
    m_regRangeCrossTalk = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  }

  m_bRangeNeedsTuning = false;

  if( (offset == m_regRangeOffset) && (crosstalk == m_regRangeCrossTalk) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void VL6180x::markAlsForTuning(byte gain, uint16_t intPeriod)
{
  m_newAlsGain      = gain;
  m_newAlsIntPeriod = intPeriod;
  m_bAlsNeedsTuning = true;
}

boolean VL6180x::tuneAls(byte gain, uint16_t intPeriod)
{
  uint16_t    val;

  if( gain != m_regAlsGain )
  {
    writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (byte)(0x40|gain));
    m_regAlsGain = readReg8(VL6180X_SYSALS_ANALOGUE_GAIN);
    m_regAlsGain &= 0x2f;
  }

  if( intPeriod == 0 )
  {
    intPeriod = 1;
  }

  if( intPeriod != m_regAlsIntPeriod )
  {
    val = intPeriod - 1;  // 0 is 1 msec, 1 is 2 msec, etc
    writeReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, val);
    m_regAlsIntPeriod = readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD);
    m_regAlsIntPeriod += 1;
  }

  m_bAlsNeedsTuning = false;

  if( (gain == m_regAlsGain) && (intPeriod == m_regAlsIntPeriod) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void VL6180x::blacklist()
{
  m_bBlackListed = true;
}

boolean VL6180x::isBlacklisted()
{
  return m_bBlackListed;
}

boolean VL6180x::isBusy()
{
  return m_bBusy;
}

void VL6180x::getIdent(VL6180xIdentification *pIdent)
{
}

void VL6180x::getTunes(byte &offset, byte &crosstalk,
                       byte &gain,   uint16_t &intPeriod)
{
  offset    = m_regRangeOffset;
  crosstalk = m_regRangeCrossTalk;
  gain      = m_regAlsGain;
  intPeriod = m_regAlsIntPeriod;
}

byte VL6180x::getRange()
{
  return m_range;
}

float VL6180x::getAmbientLight()
{
  return m_lux;
}

byte VL6180x::readReg8(uint16_t registerAddr)
{
  byte data;

  m_wire.beginTransmission( m_addr ); // Address set on class instantiation
  m_wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  m_wire.write(registerAddr & 0xFF); //LSB of register address
  m_wire.endTransmission(false); //Send address and register address bytes
  m_wire.requestFrom( m_addr , 1);
  data = m_wire.read(); //Read Data from selected register

  return data;
}

uint16_t VL6180x::readReg16(uint16_t registerAddr)
{
  uint8_t data_low;
  uint8_t data_high;
  uint16_t data;

  m_wire.beginTransmission( m_addr ); // Address set on class instantiation
  m_wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  m_wire.write(registerAddr & 0xFF); //LSB of register address
  m_wire.endTransmission(false); //Send address and register address bytes

  m_wire.requestFrom( m_addr, 2);
  data_high = m_wire.read(); //Read Data from selected register
  data_low = m_wire.read(); //Read Data from selected register
  data = (data_high << 8)|data_low;

  return data;
}

void VL6180x::writeReg8(uint16_t registerAddr, byte data)
{
  m_wire.beginTransmission( m_addr ); // Address set on class instantiation
  m_wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  m_wire.write(registerAddr & 0xFF); //LSB of register address
  m_wire.write(data); // Data/setting to be sent to device.
  m_wire.endTransmission(); //Send address and register address bytes
}

void VL6180x::writeReg16(uint16_t registerAddr, uint16_t data)
{
  m_wire.beginTransmission( m_addr ); // Address set on class instantiation
  m_wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  m_wire.write(registerAddr & 0xFF); //LSB of register address
  uint8_t temp;
  temp = (data >> 8) & 0xff;
  m_wire.write(temp); // Data/setting to be sent to device
  temp = data & 0xff;
  m_wire.write(temp); // Data/setting to be sent to device
  m_wire.endTransmission(); //Send address and register address bytes
}
