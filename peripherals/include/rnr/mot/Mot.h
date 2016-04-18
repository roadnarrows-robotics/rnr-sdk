////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libmot
//
// File:      Mot.h
//
/*! \file
 *
 * $LastChangedDate: 2013-04-02 15:23:46 -0600 (Tue, 02 Apr 2013) $
 * $Rev: 2806 $
 *
 * \brief Common Motor Controller Interface.
 *
 * \author: Robin Knight      (robin.knight@roadnarrows.com)
 * \author: Daniel Packard    (daniel@roadnarrows.com)
 * \author: Jessica Trujillo  (jessica@roadnarrows.com)
 * \author: Maurice Woods III (maurice@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////
#ifndef _MOT_H
#define _MOT_H

#define DEF_DEC -1

#include <vector>
#include <string>
#include "rnr/rnrconfig.h"
#include "rnr/units.h"

namespace rnr
{
  class Mot
  {
  public:
	  struct IDFloatTupple
    {
      int m_nMotID; 
      float m_fVal; 
    };

	  struct IDIntTupple
    {
      int   m_nMotID; 
      int   m_nVal; 
    };

	  typedef std::vector <IDFloatTupple> VecSpeedTupples;
	  typedef std::vector <IDIntTupple>   VecSpeedRawTupples;
	  typedef std::vector <int>           VecMotID;
	
    Mot() 
    {
      m_fMaxSpeed = 0;
      m_fMinSpeed = 0;
      m_fSpeedStepSize = 0;
      m_fMaxBrake = 0;
      m_fMinBrake = 0;
      m_fBrakeStepSize = 0;
    }

    Mot(float maxSpeed, float minSpeed, float speedStepSize, float maxBrake, float minBrake, float brakeStepSize)
    {
      m_fMaxSpeed = maxSpeed;
      m_fMinSpeed = minSpeed;
      m_fSpeedStepSize = speedStepSize;
      m_fMaxBrake = maxBrake;
      m_fMinBrake = minBrake;
      m_fBrakeStepSize = brakeStepSize;
    }

    virtual ~Mot() 
    {
    }
	
    /*!
     * \brief Function opens serial communication with motor controller device
     *
     * Uses SerDevOpen to open serial connection with device (devName)
     * Returns: File descriptor >=0 on success, -1 on fail.
     */
    virtual int open(const std::string &devName, int baudRate)=0;

    /*!
     * \brief Function closes serial communication with motor controller device
     *
     * Uses SerDevClose to close serial connection.
     * Returns: 0 on success, -1 on fail.
     */
    virtual int close()=0;

    /*!
     * \brief Function sends commands to the motor controller and receives
     * responses from the motor controller.
     *
     * Function uses SerDevWrite (libserial) to send commands over serial to
	   * the motor controllers, and uses recvResponse to receive echoed command
	   * responses as well as acknowledgements and query responses.
	   *
     * Returns:
     */
    //virtual int sendCommand(int fd, byte_t *buf, int nBytes, int timeout)=0;

    /*!
     * \brief Function is responsible for retrieving responses from motor
     * controllers after a command has been sent.
     *
     * Returns: nBytes (number of bytes sent in response) on success
     */
    //virtual int getResponse(int fd, byte_t *buf, int timeout)=0;

    /*!
     * \brief Function checks for valid motor ID inputs.
     *
     * Returns: 0 on success, -1 on fail.
     */
    virtual bool motIDIsValid(int motID)=0;

    /*!
     * \brief
     *
     * Description
     * \param id  des
     * \param [out] minSpeed	des
     *
     * \return 
     */
    //virtual int getMinSpeedLimit()=0;

    /*!
     * \brief
     *
     * Description
     * \param id  des
     * \param [out] minSpeed	desmotSpeeds
     *
     * \return 
     */
    //virtual int getMaxSpeedLimit()=0;

    /*!motSpeeds
     * \brief
     *
     */
    //virtual int setSpeedLimit(float &minSpeed, float &maxSpeed, float &stepSize,
                             //units_t  units=normalized)=0;

    /*!

     * \brief
     * 
     */
    //virtual int getSpeed(int motID, units_t units=UnitsNorm)=0;
	
    /*!
     * \brief Function retrieves the speed of a specific motor.
     * 
     * Raw speed will be given. 
     *
     *Needs to be passed: Motor ID (1 or 2)
     * 
     * Returns: 0 on success, -1 upon failure.
     */
    //virtual int getSpeed(VecSpeed vecSpeedTupple, units_t units=UnitsNorm)=0;

    /*!
     * \brief Function updates the speed of a single motor. 
     *
     * Needs to be passed: A speed (-1 <= speed <= 1), 
     * and a motor id (0 = left, 1 = right) 
     * Returns: 0 on Success, -1 on incorrect speed, -2 on serial write fail.
     */
    virtual int setSpeed(int motID, float speed, units_t units=units_norm)=0;
	
    /*!
     * \brief Function updates the speed of all motors simultaneously but 
     * individually.
     *
     * Function uses a vector to consturct a string of commands which will then be 
     * sent to multiple motors.
     *
     * Needs to be passed: motSpeeds vetor which includes a Motor ID (1 or 2) and 
     * a value between or equal to -1 and 1 for speed.
     *
     */
    virtual int setSpeed(VecSpeedTupples vecSpeedTupple, units_t units=units_norm)=0;

    /*!
     * \brief Function stops motor specified.
     *
     * Needs to be passed: Motor ID (1 or 2).
     *
     */
    virtual int stop(int motID)=0;    
	
    /*!
     * \brief Function simultaneously halts motor rotation for all motors by 
     * setting speeds to zero.
     *
     * Needs to be passed:
     *
     */
    //virtual int stop(VecMotID vecMotID, units_t units=UnitsNorm)=0;	

    /*!
     * \brief Function simultaneously halts motor rotation for all motors and locks 
     * the motor controller. No further commands can be given until EStop Release 
     * function has been passed.
     *
     */
    virtual int eStop()=0;

    /*!
     * \brief Function releases emergency stop condition and allows normal ops.
     *
     */
    virtual int eStopRelease()=0;


    /*!
     * \brief Function determines if a motor controller can monitor current.
     *
     * If feature is not defined by the user, the controller will not monitor 
     * current.
     */
    //virtual bool hasFeatCurrent()=0;

    /*!
     * \brief Function obtains current limit vaules from motor controller.
     *
     */
    virtual int getCurrentLimits()=0;

    /*!alim
     * \brief Function sets minimum and maximum limits for current.
     */
    virtual int setCurrentLimits(int motID, int current, units_t units=units_amp)=0;

    /*!
     * \brief Function retrieves value for current load on a single motor.
     *
     */
    virtual int getCurrent(int motID, units_t units=units_amp)=0;
	
	  /*!
     * \brief Function retrieves value for current load on a single motor.
     *
     */
    //virtual int getCurrent(vec)=0;

    /*!
     * \brief Function sets value for current supplied to a single motor.
     *
     */
    //virtual int setCurrent(int motID, float current, units_t units=units_norm)=0;    
	
 	  /*!
     * \brief Function sets value for current supplied to a single motor.
     *
     */
    //virtual int setCurrent(vec)=0;

    /*!
     *  \brief Function determines is a motor controller can monitor voltage.
     *
     *  If feature is not defined by the user, the controller will not monitor
     *  voltage.
     */
    //virtual bool hasFeatVoltage()=0;

    /*!
     * \brief Function retrieves voltage limits from motor controller.
     *
     *
     */
    virtual int getVoltageLimits()=0;

    /*!
     * \brief Function sets the minimum and maximum limits for applied voltage
     * for all motors.
     *
     * Needs to be passed: Low Voltage limit (>=5) and a Over Voltage limit (>=29).
     *
     */
    virtual int setVoltageLimits(int lowVoltage, int overVoltage)=0;

    /*!
     * \brief Function retrieves value for voltage applied to a single motor.
     *
     */
    virtual int getVoltage()=0;
	
    /*!
     * \brief Function retrieves value for voltage applied to a single motor.
     *
     */
    //virtual int getVoltage(vec)=0;

    /*!
     * \brief Function sets voltage value for an indivifual motor.
     */
    //virtual int setVoltage(int motID, float voltage)=0;

    /*!
     * \brief Function sets voltage value for an individual motor.
     */
    //virtual int setVoltage(vec)=0;
   
    /*!
     * \breif Function determines if a motor controller can monitor torque.
     *
     * If feature is not defined by the user, the controller will not monitor
     * torque.
     */
    //virtual bool hasFeatTorque()=0

    /*!
     * \brief Function retrieves value for torque output for a single motor.
     *
     */
    //virtual int getTorqueLimit(int id, float torque)=0;
    
    /*!
     * \brief Function sets minimum and maximum limits for motor torque.
     *
     */
    //virtual int setTorqueLimit(float torque )=0;

    /*!
     * \ brief Function retrives tourque values from an individual motor.
     *
     */
    //virtual int getTorque(int motID, float torque)=0;
	
	/*!
     * \ brief Function retrives tourque values from an individual motor.
     *
     */
    //virtual int getTorque(vec)=0;  

    /*!
     * \brief Function sets the torque value for an individual motor.
     * 
     * Higher torque values give the motor more turning strength.
     */
    //virtual int setTorque(int motID, float torque)=0;

    /*!
     * \brief Function sets the torque value for an individual motor.
     * 
     * Higher torque values give the motor more turning strength.
     */
    //virtual int setTorque(vec)=0;

    /*!
     * \brief Function sets the acceleration rate for motors.
     * 
     * Values represent 'slope' of the velocity profile when motor speeds are
     * changed. In the case of deceleration, this is analogous to the 
     * difference between coasting to a stop (high argument values) and 
     * stopping abruptly (low argument values).
     */
    //virtual bool hasFeatSpeedProfile()=0;

    /*!
     * \brief Function sets the acceleration rate for motors.
     * 
     * Values represent 'slope' of the velocity profile when motor speeds are
     * changed. In the case of deceleration, this is analogous to the 
     * difference between coasting to a stop (high argument values) and 
     * stopping abruptly (low argument values).
     */
    virtual int setSpeedProfile(int motID, int accel, int decel=DEF_DEC)=0;
    
    /*!
     * \brief Function sets the acce1 leration rate for motors.
     * 
     * Values represent 'slope' of the velocity profile when motor speeds are
     * changed. In the case of deceleration, this is analogous to the 
     * difference between coasting to a stop (high argument values) and 
     * stopping abruptly (low argument values).
     */
    //virtual int setSpeedProfile(vector<float> vecParams)=0;

    /*!
     * \brief
     *
     * 
     */
    //virtual bool hasFeatPositionProfile()=0;

    /*!
     * \brief
     *
     * 
     */
    //virtual int setPositionProfile(vector<float> vecParams)=0; 

  protected:
    float m_fMaxSpeed;
    float m_fMinSpeed;
    float m_fSpeedStepSize;
    float m_fMaxBrake;
    float m_fMinBrake;
    float m_fBrakeStepSize;

  };
}
#endif
