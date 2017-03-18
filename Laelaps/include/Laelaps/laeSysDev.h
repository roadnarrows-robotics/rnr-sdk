////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeSysDev.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps system devices.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _LAE_SYS_DEV_H
#define _LAE_SYS_DEV_H

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"

// hardware (except motors)
/*
#include  "Laelaps/laeI2C.h"
#include  "Laelaps/laeI2CMux.h"
#include  "Laelaps/laeWatchDog.h"
#include  "Laelaps/laeVL6180.h"
#include  "Laelaps/laeImu.h"
#include  "Laelaps/laeCams.h"
*/

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
#ifndef SWIG
namespace laelaps
{
#endif // SWIG

  // ---------------------------------------------------------------------------
  // Devices
 
  //
  // Motor controllers serial default device name.
  //
  #ifdef LAE_DEV_MOTOR_CTLRS
  const char* const LaeDevMotorCtlrs  = LAE_DEV_MOTOR_CTLRS;
                                      ///< motor controllers' serial device name
  #else
  const char* const LaeDevMotorCtlrs  = "/dev/ttySAC0";
                              ///< odroid motor controllers' serial device name
  #endif

  //
  // Motor controllers serial default baud rate.
  //
  #ifdef LAE_BAUDRATE_MOTOR_CTLRS
  const int LaeBaudRateMotorCtlrs = LAE_BAUDRATE_MOTOR_CTLRS;
                                          ///< motor controller serial baudrate
  #else
  const int LaeBaudRateMotorCtlrs = 115200;
                                          ///< motor controller serial baudrate
  #endif

  //
  // I2C default device name.
  //
  // Note:  The Odroid comes with 2 available I2C interfaces through the 
  //        expansion pins. The hardware I2C (/dev/i2c-3) and the bit-banged
  //        GPIO I2C (/dev/i2c-10?). The Laelaps uses the hardware I2C.
  //
  #ifdef LAE_DEV_I2C
  const char* const LaeDevI2C = LAE_DEV_I2C;   ///< \h_i2c device name
  #else
  const char* const LaeDevI2C = "/dev/i2c-3";  ///< odroid \h_i2c device name
  #endif

  //
  // Front USB camera.
  //
  // Note: Makes use of udev rule to make symbolic link to device.
  //
  #ifdef LAE_DEV_FCAM
  const char* const LaeDevFCam = LAE_DEV_FCAM;  ///< front cam USB device name
  #else
  const char* const LaeDevFCam = "/dev/fcam";
                                            ///< front cam USB udev linked name
  #endif

  //
  // IMU USB serial default device name.
  //
  // Note: Makes use of udev rule to make symbolic link to device.
  //
  #ifdef LAE_DEV_IMU
  const char* const LaeDevIMU = LAE_DEV_IMU;  ///< IMU USB device name
  #else
  const char* const LaeDevIMU = "/dev/imu";   ///< IMU USB udev linked name
  #endif

  //
  // IMU serial default baud rate.
  //
  #ifdef LAE_BAUDRATE_IMU
  const int LaeBaudRateIMU = LAE_BAUDRATE_IMU;  ///< IMU serial baudrate
  #else
  const int LaeBaudRateIMU = 115200;            ///< IMU serial baudrate
  #endif

  //
  // Dynabus USB serial default device name.
  //
  // Note: Makes use of udev rule to make symbolic link to device.
  //
  #ifdef LAE_DEV_DYNABUS
  const char* const LaeDevDynabus = LAE_DEV_DYNABUS;
                                            ///< dynabus USB device name
  #else
  const char* const LaeDevDynabus = "/dev/dynabus";
                                            ///< dynabus USB udev linked name
  #endif

  //
  // Dynabus default baud rate.
  //
  #ifdef LAE_BAUDRATE_DYNABUS
  const int LaeBaudRateDynabus = LAE_BAUDRATE_DYNABUS;  ///< dynabus baudrate
  #else
  const int LaeBaudRateDynabus = 1000000;               ///< dynabus baudrate
  #endif


  // ---------------------------------------------------------------------------
  // GPIOs
  
  //
  // Motor controller chip select.
  //
  // Deprecated.
  //
  // Expansion Pin: 4 (RTS)
  // States:        0 = select rear motor controller
  //                1 = select front motor controller
  //
  const int LaeGpioMotorCtlrCs = 173; ///< motor controler chip select gpio

  //
  // Motor controllers enable.
  //
  // Expansion Pin: 5 (CTS)
  // States:        0 = disable power to motor controllers
  //                1 = enable power to motor controllers
  //
  const int LaeGpioMotorCtlrEn = 174; ///< motor controler enable gpio

  //
  // Watchdog reset.
  //
  // Expansion Pin: 10 (SPI_CLK)
  // States:        1 to 0 = edge trigger to reset Arduino sub-processor
  //
  const int LaeGpioWdReset     = 189; ///< Watchdog subprocessor reset gpio

  //
  // I2C multiplexer reset.
  //
  // Expansion Pin: 11 (SPI_CSN)
  // States:        1 to 0 = edge trigger to reset I2C.
  //
  const int LaeGpioI2CMuxReset = 190; ///< I2C multiplexer reset gpio

  //
  // Top deck auxilliary battery power out enable.
  //
  // Expansion Pin: 9 (SPI_MISO)
  // States:        0 = disable deck battery power.
  //                1 = enable deck battery power.
  //
  const int LaeGpioAuxBattEn  = 191; ///< auxilliary battery enable gpio

  //
  // Top deck auxilliary regulated 5V power out enable.
  //
  // Expansion Pin: 7 (SPI_MOISO)
  // States:        0 = disable deck regulated 5V power.
  //                1 = enable deck regulated 5V power.
  //
  const int LaeGpioAux5VEn    = 192; ///< Deck regulated 5V enable gpio


  // ---------------------------------------------------------------------------
  // Time of Flight Sensor Layout
  //
  // The Laelaps comes equipped, by default, with 3 front facing sensors. An
  // add-on pack includes 5 more sensors: 2 on the sides, 3 rear facing.
  
  const int ToFSensorMaxNumOf = 8;    ///< maximum number of ToF sensors
  const int ToFSensorStdNumOf = 3;    ///< max num of ToF sensors for std option

  //
  // Sensor multiplexed channel order (CCW from forward).
  //
  const int ToFSensor0Chan    = 0;    ///< sensor 0 channel (bit) number
  const int ToFSensor1Chan    = 1;    ///< sensor 1 channel (bit) number
  const int ToFSensor2Chan    = 2;    ///< sensor 2 channel (bit) number
  const int ToFSensor3Chan    = 3;    ///< sensor 3 channel (bit) number
  const int ToFSensor4Chan    = 4;    ///< sensor 4 channel (bit) number
  const int ToFSensor5Chan    = 5;    ///< sensor 5 channel (bit) number
  const int ToFSensor6Chan    = 6;    ///< sensor 6 channel (bit) number
  const int ToFSensor7Chan    = 7;    ///< sensor 7 channel (bit) number
  
  //
  // Proximity beam direction (CCW from forward).
  //
  const double ToFSensor0Dir  = degToRad(0.0);    ///< sensor 0 direction
  const double ToFSensor1Dir  = degToRad(10.0);   ///< sensor 1 direction
  const double ToFSensor2Dir  = degToRad(90.0);   ///< sensor 2 direction
  const double ToFSensor3Dir  = degToRad(170.0);  ///< sensor 3 direction
  const double ToFSensor4Dir  = degToRad(180.0);  ///< sensor 4 direction
  const double ToFSensor5Dir  = degToRad(190.0);  ///< sensor 5 direction
  const double ToFSensor6Dir  = degToRad(270.0);  ///< sensor 6 direction
  const double ToFSensor7Dir  = degToRad(350.0);  ///< sensor 7 direction

  //
  // Sensor deadzones. Sensed data in the deadzones are filtered out.
  //
  const double ToFSensor0Deadzone   = 0.008;   ///< sensor 0 8mm deadzone
  const double ToFSensor1Deadzone   = 0.0145;  ///< sensor 1 14.5mm deadzone
  const double ToFSensor2Deadzone   = 0.008;   ///< sensor 2 8mm deadzone
  const double ToFSensor3Deadzone   = 0.0145;  ///< sensor 3 14.5mm deadzone
  const double ToFSensor4Deadzone   = 0.008;   ///< sensor 4 8mm deadzone
  const double ToFSensor5Deadzone   = 0.0145;  ///< sensor 5 14.5mm deadzone
  const double ToFSensor6Deadzone   = 0.008;   ///< sensor 6 8mm deadzone
  const double ToFSensor7Deadzone   = 0.0145;  ///< sensor 7 14.5mm deadzone

#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_SYS_DEV_H
