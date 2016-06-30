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
  //----------------------------------------------------------------------------
  // Common Interface Values
  //----------------------------------------------------------------------------

  //
  // Pass/Fail
  //
  const byte_t LaeWdArgFail   = 0;   ///< command failure response
  const byte_t LaeWdArgPass   = 1;   ///< command success response

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


  //----------------------------------------------------------------------------
  // I2C Slave Binary Interface
  //
  // The I2C inteface is a big-endian, byte oriented, binary interface. Any
  // signed numbers are in 2-compliment format.
  //
  // The (Partial) BNF
  //
  // cmd  ::= cmd_id, {byte}
  // rsp  ::= {byte}
  //
  // cmd_id ::= u8
  //----------------------------------------------------------------------------
 
  //
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
  // I2C Addressing and Packet Format
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

  const byte_t LaeI2CAddrArduino = 0x32;  ///< arduino \h_i2c 7-bit address

  const int    LaeWdMaxCmdLen = 8;   ///< maximum command length
  const int    LaeWdMaxRspLen = 8;   ///< maximum response length

  const unsigned long LaeWdTimeout  = 6000; ///< watchdog timeout (msec)
  
  
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // I2C Commands and Responses
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Pet watchdog command and response.
  //
  // Command format:        cmd_id
  // Response format(v1):   N/A
  // Response format(v2+):  is_charging
  //
  // Argument data types and ranges:
  //  is_charging ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdPetDog       = 0;    ///< command id
  const byte_t LaeWdCmdLenPetDog      = 1;    ///< command length (bytes)
  const byte_t LaeWdRspLenPetDog      = 0;    ///< v1 response length (bytes)
  const byte_t LaeWdRspLenPetDog_2    = 1;    ///< v2 response length (bytes)
  
  // ---
 
  //
  // Get firmware version command and response.
  //
  // Command format:  cmd_id
  // Response format: fw_version
  //
  // Argument data types and ranges:
  //  fw_version ::= u8 [1-255]
  //
  const byte_t LaeWdCmdIdGetVersion   = 1;  ///< command id
  const byte_t LaeWdCmdLenGetVersion  = 1;  ///< command length (bytes)
  const byte_t LaeWdRspLenGetVersion  = 1;  ///< response length (bytes)

  // ---
 
  //
  // Set battery state of charge command and response.
  //
  // Command format:  cmd_id batt_charge
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  batt_charge ::= u8 [0%-100%]
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
  // Command format:  cmd_id alarm_bits_high alarm_bits_low
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  alarm_bits_high ::= u8 [0x00 - 0xff]
  //  alarm_bits_low  ::= u8 [0x00 - 0xff]
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
  // Command format:  cmd_id red green blue
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  red   ::= u8 [0-255]
  //  green ::= u8 [0-255]
  //  blue  ::= u8 [0-255]
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
  // Command format:  cmd_id
  // Response format: N/A
  //
  const byte_t LaeWdCmdIdResetRgbLed  = 5;  ///< command id
  const byte_t LaeWdCmdLenResetRgbLed = 1;  ///< command length (bytes)
  const byte_t LaeWdRspLenResetRgbLed = 0;  ///< response length (bytes)

  // ---
 
  //
  // Configure digital pin command and response.
  //
  // Deprecated in v3+.
  //
  // Command format:  cmd_id pin_num pin_dir
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  pin_num ::= u8 [2-8]
  //  pin_dir ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdConfigDPin   = 6;    ///< command id
  const byte_t LaeWdCmdLenConfigDPin  = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenConfigDPin  = 0;    ///< response length (bytes)

  //
  // Read digital pin command and response.
  //
  // Deprecated in v3+.
  //
  // Command format:  cmd_id pin_num
  // Response format: pin_num pin_val
  //
  // Argument data types and ranges:
  //  pin_num ::= u8 [2-8]
  //  pin_val ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdReadDPin   = 7;    ///< command id
  const byte_t LaeWdCmdLenReadDPin  = 2;    ///< command length (bytes)
  const byte_t LaeWdRspLenReadDPin  = 2;    ///< response length (bytes)

  //
  // Write digital pin command and response.
  //
  // Deprecated in v3+.
  //
  // Command format:  cmd_id pin_num pin_val
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  pin_num ::= u8 [2-8]
  //  pin_val ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdWriteDPin    = 8;    ///< command id
  const byte_t LaeWdCmdLenWriteDPin   = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenWriteDPin   = 0;    ///< response length (bytes)

  // ---
 
  //
  // Read analog pin command and response.
  //
  // Deprecated in v3+.
  //
  // Command format:  cmd_id pin_num
  // Response format: pin_num pin_val_high pin_val_low
  //
  // Argument data types and ranges:
  //  pin_num       ::= u8 [14-17]
  //  pin_val_high  ::= u8 [0x00 - 0x03]
  //  pin_val_low   ::= u8 [0x00 - 0xff]
  //
  const byte_t LaeWdCmdIdReadAPin   = 9;    ///< command id
  const byte_t LaeWdCmdLenReadAPin  = 2;    ///< command length (bytes)
  const byte_t LaeWdRspLenReadAPin  = 3;    ///< response length (bytes)

  //
  // Write digital pin as analog PWM command and response.
  //
  // Deprecated in v3+.
  //
  // Command format:  cmd_id pin_num pin_val
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  pin_num       ::= u8 [2-8]
  //  pin_val       ::= u8 [0x00-0xff]
  //
  const byte_t LaeWdCmdIdWriteAPin  = 10;   ///< command id
  const byte_t LaeWdCmdLenWriteAPin = 3;    ///< command length (bytes)
  const byte_t LaeWdRspLenWriteAPin = 0;    ///< response length (bytes)

  // ---

  //
  // Enable/disable power to motor controllers.
  //
  // Command format:  cmd_id pin_val
  // Response format: pass_fail
  //
  // Argument data types and ranges:
  //  pin_val   ::= u8 {0 | 1}
  //  pass_fail ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdEnableMotorCtlrs   = 11; ///< command id
  const byte_t LaeWdCmdLenEnableMotorCtlrs  = 2;  ///< command length (bytes)
  const byte_t LaeWdRspLenEnableMotorCtlrs  = 1;  ///< response length (bytes)

  //
  // Enable/disable power to auxilliary ports.
  //
  // Command format:  cmd_id aux_port pin_val
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  aux_port  ::= u8 {1 | 2}
  //  pin_val   ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdEnableAuxPort  = 12; ///< command id
  const byte_t LaeWdCmdLenEnableAuxPort = 3;  ///< command length (bytes)
  const byte_t LaeWdRspLenEnableAuxPort = 0;  ///< response length (bytes)

  const byte_t LaeWdArgAuxPortBatt  = 1;  ///< battery auxlliary port
  const byte_t LaeWdArgAuxPort5V    = 2;  ///< regulated 5 volt auxilliary port

  //
  // Read enable lines.
  //
  // Command format:  cmd_id
  // Response format: en_motor_ctlrs en_aux_port_batt en_aux_port_5v
  //
  // Argument data types and ranges:
  //  en_motor_ctlrs    ::= u8 {0 | 1}
  //  en_aux_port_batt  ::= u8 {0 | 1}
  //  en_aux_port_5v    ::= u8 {0 | 1}
  //
  const byte_t LaeWdCmdIdReadEnables  = 13; ///< command id
  const byte_t LaeWdCmdLenReadEnables = 0;  ///< command length (bytes)
  const byte_t LaeWdRspLenReadEnables = 3;  ///< response length (bytes)

  // ---
 
  //
  // Read sensed volatages * 10.
  //
  // Command format:  cmd_id 
  // Response format: jack_v batt_v
  //
  // Argument data types and ranges:
  //  jack_v  ::= v_10
  //  batt_v  ::= v_10
  //  v_10    ::= u8 [0-255]
  //
  // Conversions:
  //  volts = v_10 * 0.1
  //
  // Examples:
  //  if val = 5   then volts = v_10 * 0.1 = 0.5
  //  if val = 112 then volts = v_10 * 0.1 = 11.2
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
  // Command format:  cmd_id 
  // Response format: seq_num op_state alarm_bits_high alarm_bits_low led_index
  //
  // Argument data types and ranges:
  //  seq_num         ::= u8 [0-255]
  //  op_state        ::= u8
  //  alarm_bits_high ::= u8
  //  alarm_bits_low  ::= u8
  //  led_index       ::= u8
  //
  const byte_t LaeWdCmdIdTest     = 15;   ///< command id
  const byte_t LaeWdCmdLenTest    = 1;    ///< command length (bytes)
  const byte_t LaeWdRspLenTest    = 5;    ///< v1 response length (bytes)

  //----------------------------------------------------------------------------
  // Serial ASCII Interface
  //
  // The (Partial) BNF
  //
  // cmd          ::= CMD_ID, {arg}, EOC
  // rsp          ::=   CMD_ID, {arg}, EOR
  //                  | ERROR_RSP_ID, CMD_ID, {arg}, EOR
  //
  // CMD_ID       ::= 'a' | 'c' | 'd' | 'i' | 'p' | 'r' | 't' | 'v' | 'w'
  // ERROR_RSP_ID ::= 'E'
  // EOC          ::= '\n'
  // EOR          ::= '\n'
  //
  // OP_GET_SET   ::= 'g' | 's'
  // OP_RESET     ::= 'r'
  // OP_STET      ::= '-'
  // OFF_ON       ::= '0' | '1'
  // MEASUREMENT  ::= ALS | TOF
  // ALS          ::= 'a'
  // TOF          ::= 'd'
  // NO_SENSOR    ::= '-'
  // SENSOR_ERROR ::= 'error'
  // NO_OBJ       ::= 'noobj'
  //
  // INT          ::= OCTAL | DECIMAL | HEX
  //
  // OCTAL        ::= ['-'] '0', {oct_digit} 
  // oct_digit    ::= '0' | '1' | '2' | '3' | '4' | '5' | '6' | '7'
  //
  // DECIMAL            ::= ['-'] nonzero_dec_digit, {dec_digit} 
  // nonzero_dec_digit  ::= '1' | '2' | '3' | '4' | '5' | '6' | '7' | '8' | '9'
  // dec_digit          ::= '0' | nonzero_dec_digit
  //
  // HEX              ::= ['-'] hex_pre, hex_digit, {hex_digit} 
  // hex_pre          ::= '0x' | '0X'
  // hex_digit        ::= dec_digit | hex_alpha_lower | hex_alpha_upper
  // hex_alpha_lower  ::= 'a' | 'b' | 'c' | 'd' | 'e' | 'f'
  // hex_alpha_upper  ::= 'A' | 'B' | 'C' | 'D' | 'E' | 'F'
  //
  // FLOAT        ::=   ['-'], dec_digit, {dec_digit}
  //                  | ['-'], dec_digit, {dec_digit}, '.', {dec_digit}
  //                  | ['-'], {dec_digit}, '.', dec_digit, {dec_digit}
  //----------------------------------------------------------------------------

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Serial Message Format
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // sizes
  const byte_t LaeWdSerMaxCmdLen    =  80;  ///< max command length (bytes)
  const byte_t LaeWdSerMaxCmdArgs   =   8;  ///< max cmd argument count
  const byte_t LaeWdSerMaxCmdArgLen =   8;  ///< max cmd arg length (bytes)
  const byte_t LaeWdSerMaxRspLen    =  80;  ///< max rsp line length (bytes)
  const byte_t LaeWdSerMaxRspArgs   =   8;  ///< max rsp argument count
  const byte_t LaeWdSerMaxRspArgLen =  16;  ///< max rsp arg length (bytes)

  // separators
  const char LaeWdSerEoC            = '\n'; ///< end of command
  const char LaeWdSerEoR[]          = "\n"; ///< end of response
  const char LaeWdSerSep            = ' ';  ///< argument separator(s)

  // common arguments
  const char LaeWdSerArgOff[]       = "0";  ///< off state
  const char LaeWdSerArgOn[]        = "1";  ///< on state
  const char LaeWdSerArgGet[]       = "g";  ///< get operator
  const char LaeWdSerArgSet[]       = "s";  ///< set operator
  const char LaeWdSerArgReset[]     = "r";  ///< reset operator
  const char LaeWdSerArgStet[]      = "-";  ///< leave as is operator
  const char LaeWdSerArgErrRsp[]    = "E";  ///< response error

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Serial Commands and Responses
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Print help command and response.
  //
  // Command format:  'h' EOC
  // Response format: 'h' cmd_id_0 ... EOR
  // Arguments:
  //  cmd_id_k ::= CMD_ID
  //
  // Note: This command's use is intended only in user interactive mode.
  //
  const char LaeWdSerCmdIdHelp  = 'h';  ///< serial command id

  //
  // Get firmware version command and response.
  //
  // Command format:  'v' EOC
  // Response format: 'v' fw_version EOR
  // Arguments:
  //  fw_version ::= DECIMAL
  //
  const char   LaeWdSerCmdIdGetVersion    = 'v';  ///< serial command id
  const byte_t LaeWdSerCmdArgsGetVersion  = 0;    ///< cmd argument count
  const byte_t LaeWdSerRspArgsGetVersion  = 1;    ///< rsp argument count

  //
  // Pet watchdog command and response.
  //
  // Command format:  'p' EOC
  // Response format: 'p' is_charging EOR
  // Arguments:
  //  is_charging ::= OFF_ON
  //
  const char   LaeWdSerCmdIdPetTheDog   = 'p';  ///< serial command id
  const byte_t LaeWdSerCmdArgsPetTheDog = 0;    ///< cmd argument count
  const byte_t LaeWdSerRspArgsPetTheDog = 1;    ///< rsp argument count

  //
  // Get/set battery state of charge command and response.
  //
  // Command format:  'b' op [batt_charge] EOC
  // Response format: 'b' batt_charge EOR
  // Arguments:
  //  op          ::= OP_GET_SET
  //  batt_charge ::= INT [0-100]
  //
  const char   LaeWdSerCmdIdBattSoC       = 'b';  ///< serial command id
  const byte_t LaeWdSerCmdArgsGetBattSoC  = 1;    ///< cmd set argument count
  const byte_t LaeWdSerCmdArgsSetBattSoC  = 2;    ///< cmd argument count
  const byte_t LaeWdSerRspArgsBattSoC     = 1;    ///< rsp argument count

  //
  // Set robot alarms command and response.
  //
  // Command format:  'a' op [alarm_bits] EOC
  // Response format: 'a' alarm_bits EOR
  // Arguments:
  //  op         ::= OP_GET_SET
  //  alarm_bits ::= INT
  //
  const char   LaeWdSerCmdIdAlarms      = 'a';  ///< serial command id
  const byte_t LaeWdSerCmdArgsGetAlarms = 1;    ///< cmd argument count
  const byte_t LaeWdSerCmdArgsSetAlarms = 2;    ///< cmd argument count
  const byte_t LaeWdSerRspArgsAlarms    = 1;    ///< rsp argument count

  //
  // Set LED RGB color command and response.
  //
  // Command format:  'l' op [red green blue] EOC
  // Response format: 'l' red green blue EOR
  //
  // Argument data types and ranges:
  // Arguments:
  //  op    ::= OP_GET_SET
  //  red   ::= color
  //  green ::= color
  //  blue  ::= color
  //  color ::= INT [0-255]

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_WATCHDOG_H
