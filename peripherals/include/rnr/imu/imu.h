////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libimu
//
// File:      imu.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-25 14:46:44 -0700 (Wed, 25 Feb 2015) $
 * $Rev: 3872 $
 *
 * \brief IMU class interface.
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

#ifndef _IMU_H
#define _IMU_H

#include "rnr/imu/imuConst.h"
#include "rnr/imu/imuSensor.h"

/*!
 * \brief RoadNarrows Robotics standard namespace.
 */
namespace rnr
{
  /*!
   * \ingroup periph_imu_base
   * \brief IMU class.
   */
  class IMU
  {
  public:
    IMU();

    IMU(IMUSensor &sensor);

    ~IMU();

    void getCaps();

    int readAccelerometer(double &ax, double &ay, double &az);

    int readGyroscope(double &gx, double &gy, double &gz);

    int readMagnetometer(double &mx, double &my, double &mz);

    int readBarometer(double &p);

    int readTemperature(double &temp);

    int quaternion();

    int eulerAngles();

    int acceleration();

    int velocity();

    int position();


  protected:
  }

} // namespace rnr


#endif // _IMU_H
