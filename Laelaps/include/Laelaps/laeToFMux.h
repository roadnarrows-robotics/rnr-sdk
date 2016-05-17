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
  // I2C Binary Interface
  //----------------------------------------------------------------------------
 
  //
  // Arduino sub-processor I2C 7-bit address.
  //
  const byte_t LaeI2CAddrToFMux = 0x71;  ///< arduino \h_i2c 7-bit slave address

  //
  // I2C Packet.
  //
  const int    LaeToFMuxMaxCmdLen = 16; ///< maximum command length
  const int    LaeToFMuxMaxRspLen = 32; ///< maximum response length
  const byte_t LaeToFMuxMinChan   =  0; ///< minimum sensor channel number
  const byte_t LaeToFMuxMaxChan   =  7; ///< maximum sensor channel number
  const byte_t LaeToFMuxNumOfChan =  8; ///< maximum number of channels

  const byte_t LaeToFMuxArgFail   =  0; ///< command failure response
  const byte_t LaeToFMuxArgPass   =  1; ///< command success response

  //............................................................................
  // Commands and Responses
  //............................................................................

  //
  // Get firmware version command and response.
  //
  // Command format:  cmd_id
  // Response format: fw_version
  //
  const byte_t LaeToFMuxCmdIdGetVersion   = 0;  ///< command id
  const byte_t LaeToFMuxCmdLenGetVersion  = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetVersion  = 1;  ///< response length (bytes)

  // ---
 
  //
  // Get sensor identity command and response.
  //
  // Command format:  cmd_id sensor
  // Response format: model_id model_major model_minor module_major module_minor
  //                  date_high date_low time_high time_low
  //
  const byte_t LaeToFMuxCmdIdGetIdent   = 1;  ///< command id
  const byte_t LaeToFMuxCmdLenGetIdent  = 2;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetIdent  = 9;  ///< response length (bytes)

  // ---
 
  //
  // Get measured ranges.
  //
  // The sub-processor continuously takes range measurments from all devices
  // and places the results in a range buffer.
  //
  // Command format:  cmd_id
  // Response format: dist0 dist1 ... dist7
  //
  const byte_t LaeToFMuxCmdIdGetRanges  = 2;  ///< command id
  const byte_t LaeToFMuxCmdLenGetRanges = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetRanges = 8;  ///< response length (bytes)

  const byte_t LaeToFMuxArgRangeNoObj = 255;  ///< no object detected 
  const byte_t LaeToFMuxArgRangeNoDev = 254;  ///< no sensor device attached/op
  const byte_t LaeToFMuxArgRangeErr   = 253;  ///< sensor error
  const byte_t LaeToFMuxArgRangeMax   = 200;  ///< maximum distance (mm)
  const byte_t LaeToFMuxArgRangeMin   =   0;  ///< minimum distance (mm)

  // ---
 
  //
  // Get measured ambient light lux values.
  //
  // Command format:  cmd_id 
  // Response format: [value_3 value_2 value_1 value_0] * 8
  //
  const byte_t LaeToFMuxCmdIdGetLux   = 3;  ///< command id
  const byte_t LaeToFMuxCmdLenGetLux  = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetLux  = 32; ///< response length (bytes)

  const float LaeToFMuxArgLuxNoLight  = 0.0;    ///< no light/no sensor
  const float LaeToFMuxArgLuxScale    = 0.01;   ///< lux = value * scale
  const float LaeToFMuxArgLuxMult     = 100.0;  ///< value = lux * mult

  // ---
 
  //
  // Tune time-of-flight range sensor.
  //
  // Command format:  cmd_id sensor tof_offset tof_cross_talk
  // Response format: N/A
  //
  const byte_t LaeToFMuxCmdIdTuneRangeSensor  = 4;  ///< command id
  const byte_t LaeToFMuxCmdLenTuneRangeSensor = 4;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenTuneRangeSensor = 0;  ///< response length (bytes)

  // ---
 
  //
  // Tune ambient light sensor.
  //
  // Command format:  cmd_id sensor
  //                    als_gain als_int_period_high als_int_period_low
  // Response format: N/A
  //
  const byte_t LaeToFMuxCmdIdTuneAls  = 5;  ///< command id
  const byte_t LaeToFMuxCmdLenTuneAls = 5;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenTuneAls = 0;  ///< response length (bytes)

  // ---
 
  //
  // Get tune parameters.
  //
  // Command format:  cmd_id sensor
  // Response format: tof_offset tof_cross_talk
  //                  als_gain als_int_period_high als_int_period_low
  //
  const byte_t LaeToFMuxCmdIdGetTunes   = 6;  ///< command id
  const byte_t LaeToFMuxCmdLenGetTunes  = 2;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetTunes  = 5;  ///< response length (bytes)


  //----------------------------------------------------------------------------
  // Serial ASCII Interface
  //----------------------------------------------------------------------------

  //
  // Serial Packet.
  //

  // sizes
  const byte_t LaeToFMuxSerMaxCmdLen    =  80; ///< max command length (bytes)
  const byte_t LaeToFMuxSerMaxCmdArgs   =   6; ///< max rsp argument count
  const byte_t LaeToFMuxSerMaxCmdArgLen =   8; ///< max rsp arg length (bytes)
  const byte_t LaeToFMuxSerMaxRspLen    = 100; ///< max rsp line length (bytes)
  const byte_t LaeToFMuxSerMaxRspArgs   =  10; ///< max rsp argument count
  const byte_t LaeToFMuxSerMaxRspArgLen =  16; ///< max rsp arg length (bytes)

  // separators
  const char LaeToFMuxSerEoC        = '\n';     ///< end of command
  const char LaeToFMuxSerEoR[]      = "\n";     ///< end of response
  const char LaeToFMuxSerSep        = ' ';      ///< argument separator(s)

  // common arguments
  const char LaeToFMuxSerArgPresent[]     = "+";      ///< sensor is present 
  const char LaeToFMuxSerArgNotPresent[]  = "-";      ///< sensor is not present
  const char LaeToFMuxSerArgNoObj[]       = "noobj";  ///< no object detected
  const char LaeToFMuxSerArgSensorErr[]   = "error";  ///< sensor sense error
  const char LaeToFMuxSerArgOff[]         = "off";    ///< off state
  const char LaeToFMuxSerArgOn[]          = "on";     ///< on state
  const char LaeToFMuxSerArgErrRsp[]      = "E";      ///< response error

  //
  // Print help command and response.
  //
  // Command format:  cmd_id
  // Response format: command synopsis EOR
  //                  command_synopsis EOR
  //                  ...
  // Note: This command's use is intended for interactive mode only.
  //
  const char LaeToFMuxSerCmdIdHelp      = 'h';  ///< serial command id

  //
  // Get firmware version command and response.
  //
  // Command format:  cmd_id
  // Response format: cmd_id fw_version EOR
  // Arguments:
  //  fw_version := DECIMAL
  //
  //
  const char   LaeToFMuxSerCmdIdGetVersion    = 'v';  ///< serial command id
  const byte_t LaeToFMuxSerCmdArgsGetVersion  = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetVersion  = 1;    ///< rsp argument count

  //
  // Get sensor identity command and response.
  //
  // Command format:  cmd_id sensor
  // Response format: cmd_id model_id model_ver module_ver date time EOR
  // Arguments:
  //  sensor      := DECIMAL [0-7]
  //  model_id    := HEX
  //  model_ver   := major.minor
  //  module_ver  := major.minor
  //  date        := INTEGER
  //  time        := INTEGER
  //  major       := INTEGER
  //  minor       := INTEGER
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
  // Command format:  cmd_id
  // Response format: cmd_id dist_0 dist_1 ... dist_7 EOR
  // Arguments:
  //  dist_k := {DECIMAL | 'noobj' | 'error' | '-'}
  //
  const char   LaeToFMuxSerCmdIdGetDist   = 'd';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetDist = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetDist = 8;    ///< rsp argument count

  //
  // Get measured ambient light sensor lux values.
  //
  // Command format:  cmd_id 
  // Response format: cmd_id lux_0 lux_1 ... lux_7 EOR
  // Arguments:
  //  lux_k := {FLOAT | 'error' | '-'}
  //
  const char   LaeToFMuxSerCmdIdGetLux    = 'a';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetLux  = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetLux  = 8;    ///< rsp argument count

  //
  // Get tune parameters.
  //
  // Command format:  cmd_id sensor
  // Response format: cmd_id tof_offset tof_cross_talk
  //                          als_gain als_int_period EOR
  // Arguments:
  //  sensor          := DECIMAL
  //  tof_offset      := DECIMAL
  //  tof_cross_talk  := DECIMAL
  //  als_gain        := DECIMAL
  //  als_int_period  := DECIMAL
  //
  const char   LaeToFMuxSerCmdIdGetTunes    = 't';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsGetTunes  = 1;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsGetTunes  = 4;    ///< rsp argument count
 
  //
  // Probe for connected ToF sensors.
  //
  // Command format:  cmd_id
  // Response format: cmd_id count EOR
  // Arguments:
  //  count := DECIMAL
  //
  const char   LaeToFMuxSerCmdIdProbe   = 'p';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsProbe = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsProbe = 1;    ///< rsp argument count

 
  //
  // List state of ToF sensors.
  //
  // Command format:  cmd_id
  // Response format: cmd_id state_0 state_1 ... state_7 EOR
  // Arguments:
  //  state_k   := {'+' | '-'}
  //
  const char   LaeToFMuxSerCmdIdList   = 'l';  ///< command id
  const byte_t LaeToFMuxSerCmdArgsList = 0;    ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsList = 1;    ///< rsp argument count


  //
  // Toggle continuous output mode on/off.
  //
  // Command format:    cmd_id
  // Response format:   cmd_id mode EOR
  // Continuous format: dist_0 dist_1 ... dist_7 EOU
  // Arguments:
  //  mode    := {'on' | 'off'}
  //  dist_k  := {DECIMAL | 'noobj' | 'error' | '-'}
  //
  // Note: Any command, including this one, will disable continuous output mode.
  // Note: This command's use is intended for interactive mode only.
  //
  const char LaeToFMuxSerCmdIdCont  = 'c';    ///< serial command id
  const byte_t LaeToFMuxSerCmdArgsCont = 0;   ///< cmd argument count
  const byte_t LaeToFMuxSerRspArgsCont = 1;   ///< rsp argument count

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_TOF_MUX_H
