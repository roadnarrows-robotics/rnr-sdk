////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libimu
//
// File:      imuConst.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-25 14:46:44 -0700 (Wed, 25 Feb 2015) $
 * $Rev: 3872 $
 *
 * \brief IMU constants.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _IMU_CONST_H
#define _IMU_CONST_H

/*!
 * \brief RoadNarrows Robotics standard namespace.
 */
namespace rnr
{
  /*!
   * \ingroup periph_imu_base
   * \defgroup imu_ecodes  IMU Error Codes
   *
   * IMU wide error codes.
   *
   * \{
   */
  static const int IMU_OK               =  0; ///< not an error, success

  static const int IMU_ECODE_GEN        =  1; ///< general, unspecified error
  static const int IMU_ECODE_SYS        =  2; ///< system (errno) error
  static const int IMU_ECODE_INTERNAL   =  3; ///< internal error (bug)
  static const int IMU_ECODE_NO_SENSOR  =  4; ///< no sensor
  static const int IMU_ECODE_NO_CAP     =  5; ///< no capabilities
  static const int IMU_ECODE_RANGE      =  6; ///< value out-of-range
  static const int IMU_ECODE_TIMEDOUT   =  7; ///< operation timed out error
  static const int IMU_ECODE_BUSY       =  8; ///< resource busy error

  static const int IMU_ECODE_BADEC      =  9; ///< bad error code

  static const int IMU_ECODE_NUMOF      = 10; ///< number of error codes
  /*! \} */
  
  

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup periph_imu_base
   * \defgroup imu_units  IMU SI Units 
   *
   * Exported interfaces are either in raw or SI units.
   *
   * \{
   */
  const double SI_g0    = 9.80665;    ///< m/s^2 accel. in standard gravity
  const double SI_K     = 273.15;     ///< 0 Celsius in Kelvins
  const double SI_PaPsi = 6894.75729; ///< 1 psi in Pascals
  const double SI_PaAtm = 1.01325e5;  ///< 1 atmosphere in Pascals
  const double SI_PaBar = 10000;      ///< 1 bar in Pascals

  //
  // SI conversion routines.
  //
  double gToMetersPerSec2(double g);
  float  gToMetersPerSec2(float g);

  double degToRad(double deg);
  float  degToRad(float deg);

  double cToK(double c);
  float  cToK(float c);

  double fToK(double f);
  float  fToK(float f);

  /*! \} */

} // namespace rnr


#endif // _IMU_CONST_H
