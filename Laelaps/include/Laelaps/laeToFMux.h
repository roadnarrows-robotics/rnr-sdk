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
  //
  // Arduino sub-processor I2C 7-bit address.
  //
  const byte_t LaeI2CAddrToFMux = 0x71;  ///< arduino \h_i2c 7-bit slave address


  //............................................................................
  // Command and Responses
  //............................................................................

  const int    LaeToFMuxMaxCmdLen = 16; ///< maximum command length
  const int    LaeToFMuxMaxRspLen = 32; ///< maximum response length
  const byte_t LaeToFMuxMinChan   =  0; ///< minimum sensor channel number
  const byte_t LaeToFMuxMaxChan   =  7; ///< maximum sensor channel number
  const byte_t LaeToFMuxNumOfChan =  8; ///< maximum number of channels

  const byte_t LaeToFMuxArgFail   =  0; ///< command failure response
  const byte_t LaeToFMuxArgPass   =  1; ///< command success response
  
  // ---
 
  //
  // Get firmware version command and response.
  //
  // Command format:  id
  // Response format: fw_version
  //
  const byte_t LaeToFMuxCmdIdGetVersion   = 0;  ///< command id
  const byte_t LaeToFMuxCmdLenGetVersion  = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetVersion  = 1;  ///< response length (bytes)

  // ---
 
  //
  // Get sensor identity command and response.
  //
  // Command format:  id sensor
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
  // Command format:  id
  // Response format: dist0 dist1 ... dist7
  //
  const byte_t LaeToFMuxCmdIdGetRanges  = 2;  ///< command id
  const byte_t LaeToFMuxCmdLenGetRanges = 1;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetRanges = 8;  ///< response length (bytes)

  const byte_t LaeToFMuxArgRangeNoObj = 255;  ///< no object detected 
  const byte_t LaeToFMuxArgRangeNoDev = 254;  ///< no sensor device attached/op
  const byte_t LaeToFMuxArgRangeErr   = 253;  ///< sensor error
  const byte_t LaeToFMuxArgRangeMin   =   0;  ///< minimum distance (mm)
  const byte_t LaeToFMuxArgRangeMax   = 200;  ///< maximum distance (mm)

  // ---
 
  //
  // Get measured ambient light lux values.
  //
  // Command format:  id 
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
  // Command format:  id sensor tof_offset tof_cross_talk
  // Response format: N/A
  //
  const byte_t LaeToFMuxCmdIdTuneRangeSensor  = 4;  ///< command id
  const byte_t LaeToFMuxCmdLenTuneRangeSensor = 4;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenTuneRangeSensor = 0;  ///< response length (bytes)

  // ---
 
  //
  // Tune ambient light sensor.
  //
  // Command format:  id sensor als_gain als_int_period_high als_int_period_low
  // Response format: N/A
  //
  const byte_t LaeToFMuxCmdIdTuneAls  = 5;  ///< command id
  const byte_t LaeToFMuxCmdLenTuneAls = 5;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenTuneAls = 0;  ///< response length (bytes)

  // ---
 
  //
  // Get tune parameters.
  //
  // Command format:  id sensor
  // Response format: tof_offset tof_cross_talk
  //                  als_gain als_int_period_high als_int_period_low
  //
  const byte_t LaeToFMuxCmdIdGetTunes   = 6;  ///< command id
  const byte_t LaeToFMuxCmdLenGetTunes  = 2;  ///< command length (bytes)
  const byte_t LaeToFMuxRspLenGetTunes  = 5;  ///< response length (bytes)

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_TOF_MUX_H
