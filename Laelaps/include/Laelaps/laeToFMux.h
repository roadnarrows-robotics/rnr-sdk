////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeToFMux.h
//
/*! \file
 *
 * $LastChangedDate: 2016-03-08 10:43:48 -0700 (Tue, 08 Mar 2016) $
 * $Rev: 4346 $
 *
 * \brief Laelaps built-in Time-of-Flight Multiplexer Arduino sub-processor
 * interface.
 * 
 * The ToFMux serves to control up to 8 time-of-flight sensors.
 *
 * This file is included in both Laelaps software and the Arduino firmware.
 * KEEP IT SIMPLE.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
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

#ifndef _LAE_TOF_MUX_H
#define _LAE_TOF_MUX_H

//
// Time-of-Flight Mutliplexer Version
//
#ifdef LAE_TOF_MUX_FW_VERSION
#define LAE_TOF_MUX_VERSION  LAE_TOF_MUX_FW_VERSION ///< fw version.
#else
#define LAE_TOF_MUX_VERSION  1                      ///< sw/fw version.
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
 
  // sensors
  const byte_t LaeToFMuxMinChan   =  0; ///< minimum sensor channel number
  const byte_t LaeToFMuxMaxChan   =  7; ///< maximum sensor channel number
  const byte_t LaeToFMuxNumOfChan =  8; ///< maximum number of channels

  // range sensor values
  const byte_t LaeToFMuxRangeNoObj  = 255;  ///< no object detected 
  const byte_t LaeToFMuxRangeNoDev  = 254;  ///< no sensor device attached
  const byte_t LaeToFMuxRangeErr    = 253;  ///< sensor error
  const byte_t LaeToFMuxRangeMax    = 200;  ///< maximum distance (mm)
  const byte_t LaeToFMuxRangeMin    =   0;  ///< minimum distance (mm)

  // ambient light sensor values
  const float LaeToFMuxLuxNoLight = 0.0;    ///< no light/no sensor


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
 
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // I2C Addressing and Packet Format
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  const byte_t LaeI2CAddrToFMux = 0x71;  ///< arduino \h_i2c 7-bit slave address

  const int    LaeToFMuxI2CMaxCmdLen  = 16; ///< maximum command length
  const int    LaeToFMuxI2CMaxRspLen  = 32; ///< maximum response length

  const byte_t LaeToFMuxI2CArgFail    =  0; ///< command failure response
  const byte_t LaeToFMuxI2CArgPass    =  1; ///< command success response

  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // I2C Commands and Responses
  //. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Get firmware version command and response.
  //
  // Command format:  cmd_id
  // Response format: fw_version
  //
  // Argument data types and ranges:
  //  fw_version ::= u8 [1-255]
  //
  const byte_t LaeToFMuxI2CCmdIdGetVersion  = 0;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenGetVersion = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenGetVersion = 1;  ///< response length (bytes)

  // ---
 
  //
  // Get sensor identity command and response.
  //
  // Command format:  cmd_id sensor
  // Response format: model_id model_major model_minor module_major module_minor
  //                  date_high date_low time_high time_low
  //
  // Argument data types and ranges:
  //  sensor        ::= u8 [1-7]
  //  model_id      ::= u8
  //  model_major   ::= u8
  //  model_minor   ::= u8
  //  modul3_major  ::= u8
  //  modul3_minor  ::= u8
  //  date_high     ::= u8
  //  date_low      ::= u8
  //  time_high     ::= u8
  //  time_low      ::= u8
  //
  // Conversions:
  //  date = date_high << 8 + date_low
  //  time = time_high << 8 + time_low
  //
  const byte_t LaeToFMuxI2CCmdIdGetIdent  = 1;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenGetIdent = 2;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenGetIdent = 9;  ///< response length (bytes)

  // ---
 
  //
  // Get measured ranges.
  //
  // The sub-processor continuously takes range measurments from all devices
  // and places the results in a range buffer.
  //
  // Command format:  cmd_id
  // Response format: dist_0 dist_1 ... dist_7
  //
  // Argument data types and ranges:
  //  dist_k  ::= u8 [0-200, 253, 254, 255]
  //
  const byte_t LaeToFMuxI2CCmdIdGetRanges   = 2;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenGetRanges  = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenGetRanges  = 8;  ///< response length (bytes)

  // ---
 
  //
  // Get measured ambient light lux values.
  //
  // Command format:  cmd_id 
  // Response format: _lux_0_3 lux_0_2 lux_0_1 lux_0_0
  //                  _lux_1_3 lux_1_2 lux_1_1 lux_1_0
  //                  ...
  //                  _lux_7_3 lux_7_2 lux_7_1 lux_7_0
  //
  // Argument data types and ranges:
  //  _lux_k_n  ::= u8
  //
  // Conversions:
  //  lux100_k  = lux_k_3 << 24 + lux_k_2 << 16 + lux_k_1 << 8 + lux_k_0
  //  lux_k     = lux100_k * 0.01
  //
  const byte_t LaeToFMuxI2CCmdIdGetLux  = 3;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenGetLux = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenGetLux = 32; ///< response length (bytes)

  const float LaeToFMuxI2CArgLuxScale   = 0.01;   ///< lux = value * scale
  const float LaeToFMuxI2CArgLuxMult    = 100.0;  ///< value = lux * mult

  // ---
 
  //
  // Tune time-of-flight range sensor.
  //
  // Command format:  cmd_id sensor
  //                    tof_offset tof_cross_talk_high tof_cross_talk_low
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  sensor              ::= u8 [1-7]
  //  tof_offset          ::= u8
  //  tof_cross_talk_high ::= u8
  //  tof_cross_talk_low  ::= u8
  //  tof_cross_talk      ::= u16
  //
  // Conversions:
  //  tof_cross_talk_high = tof_cross_talk >> 8
  //  tof_cross_talk_low  = tof_cross_talk & 0xff
  //
  const byte_t LaeToFMuxI2CCmdIdTuneToFSensor   = 4;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenTuneToFSensor  = 5;  ///< command length
  const byte_t LaeToFMuxI2CRspLenTuneToFSensor  = 0;  ///< response length

  // ---
 
  //
  // Tune ambient light sensor.
  //
  // Command format:  cmd_id sensor
  //                    als_gain als_int_period_high als_int_period_low
  // Response format: N/A
  //
  // Argument data types and ranges:
  //  sensor              ::= u8 [1-7]
  //  als_gain            ::= u8 [0-7]
  //  als_int_period_high ::= u8
  //  als_int_period_low  ::= u8
  //  als_int_period      ::= u16 [1-512]
  //
  // Conversions:
  //  als_int_period_high = als_int_period >> 8
  //  als_int_period_low  = als_int_period & 0xff
  //
  const byte_t LaeToFMuxI2CCmdIdTuneAls   = 5;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenTuneAls  = 5;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenTuneAls  = 0;  ///< response length (bytes)

  // ---
 
  //
  // Get tune parameters.
  //
  // Command format:  cmd_id sensor
  // Response format: tof_offset tof_cross_talk_high tof_cross_talk_low
  //                  als_gain als_int_period_high als_int_period_low
  //
  // Argument data types and ranges:
  //  sensor              ::= u8 [1-7]
  //  tof_offset          ::= u8
  //  tof_cross_talk_high ::= u8
  //  tof_cross_talk_low  ::= u8
  //  als_gain            ::= u8 [0-7]
  //  als_int_period_high ::= u8
  //  als_int_period_low  ::= u8
  //
  // Conversions:
  //  tof_cross_talk = tof_cross_talk_high << 8 + tof_cross_talk_low
  //  als_int_period = als_int_period_high << 8 + als_int_period_low
  //
  const byte_t LaeToFMuxI2CCmdIdGetTunes  = 6;  ///< command id
  const byte_t LaeToFMuxI2CCmdLenGetTunes = 2;  ///< command length (bytes)
  const byte_t LaeToFMuxI2CRspLenGetTunes = 6;  ///< response length (bytes)


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
  const byte_t LaeToFMuxSerMaxCmdLen    =  80; ///< max command length (bytes)
  const byte_t LaeToFMuxSerMaxCmdArgs   =   8; ///< max cmd argument count
  const byte_t LaeToFMuxSerMaxCmdArgLen =   8; ///< max cmd arg length (bytes)
  const byte_t LaeToFMuxSerMaxRspLen    = 100; ///< max rsp line length (bytes)
  const byte_t LaeToFMuxSerMaxRspArgs   =  10; ///< max rsp argument count
  const byte_t LaeToFMuxSerMaxRspArgLen =  16; ///< max rsp arg length (bytes)

  // separators
  const char LaeToFMuxSerEoC        = '\n';     ///< end of command
  const char LaeToFMuxSerEoR[]      = "\n";     ///< end of response
  const char LaeToFMuxSerSep        = ' ';      ///< argument separator(s)

  // common arguments
  const char LaeToFMuxSerArgNoSensor[]    = "-";      ///< sensor not present
  const char LaeToFMuxSerArgNoObj[]       = "noobj";  ///< no object detected
  const char LaeToFMuxSerArgSensorErr[]   = "error";  ///< sensor meas. error
  const char LaeToFMuxSerArgOff[]         = "0";      ///< off state
  const char LaeToFMuxSerArgOn[]          = "1";      ///< on state
  const char LaeToFMuxSerArgGet[]         = "g";      ///< get operator
  const char LaeToFMuxSerArgSet[]         = "s";      ///< set operator
  const char LaeToFMuxSerArgReset[]       = "r";      ///< reset operator
  const char LaeToFMuxSerArgStet[]        = "-";      ///< leave as is
  const char LaeToFMuxSerArgErrRsp[]      = "E";      ///< response error

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
  const char LaeToFMuxSerCmdIdHelp  = 'h';  ///< serial command id

  //
  // Get firmware version command and response.
  //
  // Command format:  'v' EOC
  // Response format: 'v' product subproc fw_version EOR
  // Arguments:
  //  fw_version ::= DECIMAL
  //
  const char   LaeToFMuxSerCmdIdGetVersion    = 'v';  ///< serial command id
  const byte_t LaeToFMuxSerCmdArgsGetVersion  = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetVersion  = 3;    ///< rsp argument count

  //
  // Configure firmware operation.
  //
  // Command format:  'c' op [config] EOC
  // Response format: 'c' config EOR
  // Arguments:
  //  op        ::= OP_GET_SET
  //  config    ::= als_state
  //  als_state ::= OFF_ON
  //
  const char   LaeToFMuxSerCmdIdConfig      = 'c';  ///< serial command id
  const byte_t LaeToFMuxSerCmdArgsGetConfig = 1;    ///< cmd get argument count
  const byte_t LaeToFMuxSerCmdArgsSetConfig = 2;    ///< cmd set argument count
  const byte_t LaeToFMuxSerRspArgsConfig    = 1;    ///< rsp argument count

  //
  // Get sensor identity command and response.
  //
  // Command format:  'i' sensor EOC
  // Response format: 'i' model_id model_ver module_ver date time EOR
  // Arguments:
  //  sensor      ::= INT
  //  model_id    ::= HEX
  //  model_ver   ::= major.minor
  //  module_ver  ::= major.minor
  //  date        ::= DECIMAL
  //  time        ::= DECIMAL
  //  major       ::= DECIMAL
  //  minor       ::= DECIMAL
  //
  const char   LaeToFMuxSerCmdIdGetIdent    = 'i';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetIdent  = 1;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetIdent  = 5;    ///< rsp argument count
 
  //
  // Get measured distances.
  //
  // The sub-processor continuously takes range measurments from all devices
  // and places the results in a range buffer.
  //
  // Command format:  'd' EOC
  // Response format: 'd' dist_0 dist_1 ... dist_7 EOR
  // Arguments:
  //  dist_k ::= DECIMAL | NO_OBJ | SENSOR_ERROR | NO_SENSOR
  //
  const char   LaeToFMuxSerCmdIdGetDist   = 'd';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetDist = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetDist = 8;    ///< rsp argument count

  //
  // Get measured ambient light sensor lux values.
  //
  // Command format:  'a'  EOC
  // Response format: 'a' lux_0 lux_1 ... lux_7 EOR
  // Arguments:
  //  lux_k ::= FLOAT | SENSOR_ERROR | NO_SENSOR
  //
  const char   LaeToFMuxSerCmdIdGetLux    = 'a';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetLux  = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetLux  = 8;    ///< rsp argument count

  //
  // Get/set tune parameters.
  //
  // Command format:  't' op sensor [tunes] EOC
  // Response format: 't' tunes EOR
  // Arguments:
  //  sensor          ::= INT
  //  op              ::= OP_GET_SET
  //  tunes           ::= tof_offset tof_cross_talk als_gain als_int_period
  //  tof_offset      ::= INT | OP_RESET | OP_STET
  //  tof_cross_talk  ::= INT | OP_RESET | OP_STET
  //  als_gain        ::= INT | OP_RESET | OP_STET
  //  als_int_period  ::= INT | OP_RESET | OP_STET
  //
  const char   LaeToFMuxSerCmdIdTunes       = 't';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetTunes  = 2;    ///< cmd get argument count
  const byte_t LaeToFMuxSerCmdArgsSetTunes  = 6;    ///< cmd set argument count
  const byte_t LaeToFMuxSerRspArgsTunes     = 4;    ///< rsp argument count
 
  //
  // Probe for connected ToF sensors.
  //
  // Command format:  'p' EOC
  // Response format: 'p' count EOR
  // Arguments:
  //  count ::= DECIMAL
  //
  const char   LaeToFMuxSerCmdIdProbe   = 'p';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsProbe = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsProbe = 1;    ///< rsp argument count

 
  //
  // List state of ToF sensors.
  //
  // Command format:  'l' EOC
  // Response format: 'l' state_0 state_1 ... state_7 EOR
  // Arguments:
  //  state_k   ::= {sensor | NO_SENSOR}
  //  sensor    ::= DECIMAL
  //
  const char   LaeToFMuxSerCmdIdList   = 'l';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsList = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsList = 1;    ///< rsp argument count

  //
  // Enable serial continuous output mode.
  //
  // Command format:    'o' output EOC
  // Response format:   'o' output EOR
  // Arguments:
  //  output  ::= MEASUREMENT
  // Continuous ambient light measurement output format:
  //  lux_0  lux_1  ... lux_7  EOR
  // with:
  //  lux_k   ::= FLOAT | SENSOR_ERROR | NO_SENSOR
  // Continuous time-of-flight distance measurement output format:
  //  dist_0 dist_1 ... dist_7 EOR
  // with:
  //  dist_k  ::= DECIMAL | NO_OBJ | SENSOR_ERROR | NO_SENSOR
  //
  // Note: This command's use is intended only in user interactive mode.
  // Note: Any command, except this command, turns off continuous output mode.
  //
  const char LaeToFMuxSerCmdIdCont     = 'o'; ///< serial command id
  const byte_t LaeToFMuxSerCmdArgsCont = 1;   ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsCont = 1;   ///< rsp argument count

  //
  // Read register value.
  //
  // Command format:  'r' sensor addr size EOC
  // Response format: 'r' val EOR
  // Arguments:
  //  sensor  ::= INT
  //  addr    ::= INT
  //  size    ::= 1 | 2
  //  val     ::= HEX
  //
  const char   LaeToFMuxSerCmdIdReadReg   = 'r';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsReadReg = 3;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsReadReg = 1;    ///< rsp argument count

  //
  // Write register value.
  //
  // Command format:  'w' sensor addr size val EOC
  // Response format: 'w' val EOR
  // Arguments:
  //  sensor  ::= INT
  //  addr    ::= INT
  //  size    ::= 1 | 2
  //  val     ::= INT
  //
  const char   LaeToFMuxSerCmdIdWriteReg    = 'w';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsWriteReg  = 4;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsWriteReg  = 1;    ///< rsp argument count

  //
  // Debug (not implemented)
  //
  // Command format:  'x' level EOC
  // Response format: 'x' level EOR
  // Arguments:
  //  level ::= INT
  //
  const char   LaeToFMuxSerCmdIdDebug   = 'X';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsDebug = 1;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsDebug = 1;    ///< rsp argument count

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_TOF_MUX_H
