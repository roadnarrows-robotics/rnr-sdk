////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeWatchDog.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-22 18:11:12 -0700 (Mon, 22 Feb 2016) $
 * $Rev: 4328 $
 *
 * \brief Laelaps built-in Arduino sub-processor.
 * 
 * The Arduino serves to monitor the system health and to control the system
 * RBG Status LED, along with user exposed GPIO and analog pins.
 *
 * This file is included in both Laelaps software and the Arduino firmware.
 * KEEP IT SIMPLE.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _LAE_WATCHDOG_H
#define _LAE_WATCHDOG_H

//
// WatchDog Version
//
#ifdef LAE_WD_FW_VERSION
#define LAE_WD_VERSION  LAE_WD_FW_VERSION     ///< watchdog fw version.
#else
#define LAE_WD_VERSION  1                     ///< watchdog sw/fw version.
#endif

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
#ifndef SWIG
namespace laelaps
{
#endif // SWIG
  //
  // Arduino sub-processor I2C 7-bit address.
  //
  const byte_t LaeI2CAddrArduino = 0x32;  ///< arduino \h_i2c 7-bit address


  //............................................................................
  // Command and Responses
  //............................................................................

  const int    LaeWdMaxCmdLen = 8;   ///< maximum command length
  const int    LaeWdMaxRspLen = 8;   ///< maximum response length
  const byte_t LaeWdArgFail   = 0;   ///< command failure response
  const byte_t LaeWdArgPass   = 1;   ///< command success response
  
  // ---
 
  //
  // Pet watchdog command and response.
  //
  // Command format:        id
  // Response format(v1):   N/A
  // Response format(v2):   batt_chg
  //
  const byte_t LaeWdCmdIdPetDog       = 0;    ///< command id
  const byte_t LaeWdCmdLenPetDog      = 1;    ///< command length (bytes)
  const byte_t LaeWdRspLenPetDog      = 0;    ///< v1 response length (bytes)
  const byte_t LaeWdRspLenPetDog_2    = 1;    ///< v2 response length (bytes)

  const unsigned long LaeWdTimeout  = 6000; ///< watchdog timeout (msec)
  
  // ---
 
  //
  // Get firmware version command and response.
  //
  // Command format:  id
  // Response format: fw_version
  //
  const byte_t LaeWdCmdIdGetVersion   = 1;  ///< command id
  const byte_t LaeWdCmdLenGetVersion  = 1;  ///< command length (bytes)
  const byte_t LaeWdRspLenGetVersion  = 1;  ///< response length (bytes)

  // ---
 
  //
  // Set battery state of charge command and response.
  //
  // Command format:  id batt_charge
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdSetBattCharge  = 2;    ///< command id
  const byte_t LaeWdCmdLenSetBattCharge = 2;    ///< command length (bytes)
  const byte_t LaeWdRspLenSetBattCharge = 0;    ///< response length (bytes)

  const byte_t LaeWdArgBattChargeMin    = 0;    ///< 0% charge
  const byte_t LaeWdArgBattChargeMax    = 100;  ///< 100% charge

  // ---
 
  //
  // Set robot alarms command and response.
  //
  // Command format:  id alarm_type
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdSetAlarms  = 3;  ///< command id
  const byte_t LaeWdCmdLenSetAlarms = 3;  ///< command length (bytes)
  const byte_t LaeWdRspLenSetAlarms = 0;  ///< response length (bytes)

  const int LaeWdArgAlarmNone     = 0x0000;  ///< no/clear alarms
  const int LaeWdArgAlarmGen      = 0x0001;  ///< general, unspecified alarm
  const int LaeWdArgAlarmBatt     = 0x0002;  ///< battery low alarm
  const int LaeWdArgAlarmTemp     = 0x0004;  ///< temperature alarm
  const int LaeWdArgAlarmEStop    = 0x0008;  ///< emergency stop
  const int LaeWdArgAlarmBattCrit = 0x1000;  ///< battery critical modifier bit
  const int LaeWdArgAlarmCrit     = 0x2000;  ///< critical alarm modifier bit
  const int LaeWdArgAlarmTypeMask = 0x0fff;  ///< alarm types mask
  const int LaeWdArgAlarmMask     = 0xffff;  ///< alarm valid bits mask


  // ---
 
  //
  // Set LED RGB color command and response.
  //
  // Command format:  id red green blue
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdSetRgbLed  = 4;    ///< command id
  const byte_t LaeWdCmdLenSetRgbLed = 4;    ///< command length (bytes)
  const byte_t LaeWdRspLenSetRgbLed = 0;    ///< response length (bytes)

  const byte_t LaeWdArgRgbLedMin    = 0;    ///< no channel color
  const byte_t LaeWdArgRgbLedMax    = 255;  ///< full channel color

  // ---
 
  //
  // Reset LED RGB color to state defaults.
  //
  // Command format:  id
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdResetRgbLed  = 5;  ///< command id
  const byte_t LaeWdCmdLenResetRgbLed = 1;  ///< command length (bytes)
  const byte_t LaeWdRspLenResetRgbLed = 0;  ///< response length (bytes)

  // ---
 
  //
  // Common digital pin arguments.
  //
  const byte_t LaeWdArgDPinNumMin   = 2;  ///< min pin number
  const byte_t LaeWdArgDPinNumWMin  = 5;  ///< min write/modify pin number
  const byte_t LaeWdArgDPinNumWMax  = 8;  ///< max write/modify pin number
  const byte_t LaeWdArgDPinNumMax   = 8;  ///< max pin number

  const byte_t LaeWdArgDPinDirIn    = 0;  ///< input
  const byte_t LaeWdArgDPinDirOut   = 1;  ///< output

  const byte_t LaeWdArgDPinValLow   = 0;  ///< pin is low (0V)
  const byte_t LaeWdArgDPinValHigh  = 1;  ///< pin is high (Vcc)

  //
  // Configure digital pin command and response.
  //
  // Command format:  id pin_num pin_dir
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdConfigDPin   = 6;    ///< command id
  const byte_t LaeWdCmdLenConfigDPin  = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenConfigDPin  = 0;    ///< response length (bytes)

  //
  // Read digital pin command and response.
  //
  // Command format:  id pin_num
  // Response format: pin_num pin_val
  //
  const byte_t LaeWdCmdIdReadDPin   = 7;    ///< command id
  const byte_t LaeWdCmdLenReadDPin  = 2;    ///< command length (bytes)
  const byte_t LaeWdRspLenReadDPin  = 2;    ///< response length (bytes)

  //
  // Write digital pin command and response.
  //
  // Command format:  id pin_num pin_val
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdWriteDPin    = 8;    ///< command id
  const byte_t LaeWdCmdLenWriteDPin   = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenWriteDPin   = 0;    ///< response length (bytes)

  // ---
 
  //
  // Common analog pin arguments.
  //
  const byte_t LaeWdArgAInPinNumMin   = 14;   ///< analog input min pin number
  const byte_t LaeWdArgAInPinNumMax   = 17;   ///< analog input max pin number

  const int    LaeWdArgAInPinValMin   = 0;    ///< analog input minimum value
  const int    LaeWdArgAInPinValMax   = 1023; ///< analog input maximum value

  const byte_t LaeWdArgAOutPinNumMin  = 2;    ///< analog output min pin number
  const byte_t LaeWdArgAOutPinNumMax  = 8;    ///< analog output max pin number

  const int    LaeWdArgAOutPinValMin  = 0;    ///< analog output minimum value
  const int    LaeWdArgAOutPinValMax  = 255;  ///< analog output maximum value

  //
  // Read analog pin command and response.
  //
  // Command format:  id pin_num
  // Response format: pin_num pin_val_hi pin_val_lo
  //
  const byte_t LaeWdCmdIdReadAPin   = 9;    ///< command id
  const byte_t LaeWdCmdLenReadAPin  = 2;    ///< command length (bytes)
  const byte_t LaeWdRspLenReadAPin  = 3;    ///< response length (bytes)

  //
  // Write digital pin as analog PWM command and response.
  //
  // Command format:  id pin_num pin_val
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdWriteAPin  = 10;   ///< command id
  const byte_t LaeWdCmdLenWriteAPin = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenWriteAPin = 0;    ///< response length (bytes)

  // ---

  //
  // Enable/disable power to motor controllers.
  //
  // Command format:  id pin_val
  // Response format: P/F
  //
  const byte_t LaeWdCmdIdEnableMotorCtlrs   = 11; ///< command id
  const byte_t LaeWdCmdLenEnableMotorCtlrs  = 2;  ///< command length (bytes)
  const byte_t LaeWdRspLenEnableMotorCtlrs  = 1;  ///< response length (bytes)

  //
  // Enable/disable power to auxilliary ports.
  //
  // Command format:  id aux_port pin_val
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdEnableAuxPort  = 12; ///< command id
  const byte_t LaeWdCmdLenEnableAuxPort = 3;  ///< command length (bytes)
  const byte_t LaeWdRspLenEnableAuxPort = 0;  ///< response length (bytes)

  const byte_t LaeWdArgAuxPortBatt  = 1;  ///< battery auxlliary port
  const byte_t LaeWdArgAuxPort5V    = 2;  ///< regulated 5 volt auxilliary port

  //
  // Read enable lines.
  //
  // Command format:  id
  // Response format: en_motor_ctlrs en_aux_port_batt en_aux_port_5v
  //
  const byte_t LaeWdCmdIdReadEnables  = 13; ///< command id
  const byte_t LaeWdCmdLenReadEnables = 0;  ///< command length (bytes)
  const byte_t LaeWdRspLenReadEnables = 3;  ///< response length (bytes)

  // ---
 
  //
  // Read sensed volatages * 10.
  //
  // If val = 5   then V = val * 0.1 = 0.5
  // If val = 112 then V = val * 0.1 = 11.2
  //
  // Command format:  id 
  // Response format: jack_v batt_v
  //
  const byte_t LaeWdCmdIdReadVolts  = 14; ///< command id
  const byte_t LaeWdCmdLenReadVolts = 1;  ///< command length (bytes)
  const byte_t LaeWdRspLenReadVolts = 2;  ///< response length (bytes)

  const float  LaeWdArgVMult    = 10.0;               ///< voltage multiplier
  const float  LaeWdArgVScale   =  0.1;               ///< voltage scaler

  // ---
 
  //
  // Test watchdog state command and response.
  //
  // Command format:  id 
  // Response format: seq_num op_state alarms_hi alarms_lo led_index
  //                      
  //
  const byte_t LaeWdCmdIdTest     = 15;   ///< command id
  const byte_t LaeWdCmdLenTest    = 1;    ///< command length (bytes)
  const byte_t LaeWdRspLenTest    = 5;    ///< v1 response length (bytes)


#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_WATCHDOG_H
