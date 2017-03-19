////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libimu
//
// File:      imuSensor.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-25 14:46:44 -0700 (Wed, 25 Feb 2015) $
 * $Rev: 3872 $
 *
 * \brief IMU sensor base class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _IMU_SENOSR_H
#define _IMU_SENOSR_H

#include "rnr/imu/imuConst.h"

/*!
 * \brief RoadNarrows Robotics standard namespace.
 */
namespace rnr
{
  /*!
   * \ingroup periph_imu_sensor
   * \brief IMU sensor class.
   */
  class IMUSensor
  {
  public:
    IMU() { }

    virtual ~IMUSensor() { }

    virtual void getCaps() { }

    virtual int readAccelerometer(double &ax, double &ay, double &az)
    {
      return -IMU_ECODE_NO_CAP;
    }

    virtual int readGyroscope(double &gx, double &gy, double &gz);
    {
      return -IMU_ECODE_NO_CAP;
    }

    virtual int readMagnetometer(double &mx, double &my, double &mz);
    {
      return -IMU_ECODE_NO_CAP;
    }

    virtual int readBarometer(double &p);
    {
      return -IMU_ECODE_NO_CAP;
    }

    virtual int readTemperature(double &temp);
    {
      return -IMU_ECODE_NO_CAP;
    }

  protected:
  }

} // namespace rnr


#endif // _IMU_SENOSR_H
