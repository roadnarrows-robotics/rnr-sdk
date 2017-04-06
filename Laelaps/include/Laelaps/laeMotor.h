////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeMotor.h
//
/*! \file
 *
 * $LastChangedDate: 2016-03-07 17:57:47 -0700 (Mon, 07 Mar 2016) $
 * $Rev: 4345 $
 *
 * \brief Laelaps motors, encoder, and controllers hardware abstraction
 * interfaces.
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

#ifndef _LAE_MOTOR_H
#define _LAE_MOTOR_H

#include <sys/types.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"

namespace laelaps
{
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \ingroup lae_spec
   *
   * \defgroup lae_motor  Laelaps Motors
   *
   * \h_laelaps motors, encoders, and controllers.
   *
   * The \h_laelaps currently uses Ion Motion Control's RoboClaw 30A motor
   * controllers. The motors are...
   *
   * The \h_laelaps two motor controllers control two motors each. The motors
   * are attached to the internal side of the body and drive independent
   * wheel-tire assemblies. The front controller controls the left-front and
   * right-front wheels, while the back controller controls the left-rear and
   * right-rear wheels.
   *
   * The left side motors are mounted to the body such that they have normal
   * sense of rotation. The right side motors have reverse sense.
   * \{
   */

  //
  // Controller/Powertrain/motor string keys.
  //
  static const char* const LaeKeyFront      = "front";        ///< front
  static const char* const LaeKeyRear       = "rear";         ///< rear
  static const char* const LaeKeyLeftFront  = "left_front";   ///< left front
  static const char* const LaeKeyRightFront = "right_front";  ///< right front
  static const char* const LaeKeyLeftRear   = "left_rear";    ///< left rear
  static const char* const LaeKeyRightRear  = "right_rear";   ///< right rear

  //
  // Absolute powertrain/motor ids.
  //
  static const int LaeMotorIdNone     = -1; ///< no motor id
  static const int LaeMotorIdLF       =  0; ///< left front
  static const int LaeMotorIdRF       =  1; ///< right front
  static const int LaeMotorIdLR       =  2; ///< left rear
  static const int LaeMotorIdRR       =  3; ///< right rear
  static const int LaeMotorsNumOf     =  4; ///< number of motors

  //
  // Motor controllers (2 motors/controller).
  //
  static const int LaeMotorCtlrIdNone   = -1; ///< no motor controller id
  static const int LaeMotorCtlrIdFront  =  0; ///< front motor controller
  static const int LaeMotorCtlrIdRear   =  1; ///< rear motor controller
  static const int LaeNumMotorCtlrs     =  2; ///< number of motor controllers

  //
  // Motor controller communication channel addresses.
  //
  static const byte_t LaeMotorCtlrAddrFront = motor::roboclaw::AddrMin;
                                          ///< front motor controller address
  static const byte_t LaeMotorCtlrAddrRear  = motor::roboclaw::AddrMin + 1;
                                          ///< rear motor controller address

  //
  // Motor control motor relative indices.
  //
  static const int LaeMotorLeft     = motor::roboclaw::Motor1; ///< left motors
  static const int LaeMotorRight    = motor::roboclaw::Motor2; ///< right motors
  static const int LaeNumMotorsPerCtlr = motor::roboclaw::NumMotors;
                                                ///< number of motors/controller

  //
  // Motor sense of rotation.
  //
  static const int LaeMotorDirNormal = motor::roboclaw::MotorDirNormal;
                                                                    ///< normal
  static const int LaeMotorDirReverse  = motor::roboclaw::MotorDirReverse;
                                                                    ///< reverse
  //
  // Motor alarms (derived from motor controller status).
  //
  static const int LaeMotorAlarmNone    =  0x00;  ///< no alarms
  static const int LaeMotorAlarmCurrent =  0x01;  ///< under/over current
  static const int LaeMotorAlarmVoltage =  0x02;  ///< under/over voltage
  static const int LaeMotorAlarmEStop   =  0x04;  ///< emergency stopped
  static const int LaeMotorAlarmTemp    =  0x08;  ///< over temperature
  static const int LaeMotorAlarmFault   =  0x10;  ///< motor drive fault

  //
  // Motor warnings (derived from motor controller status).
  //
  static const int LaeMotorWarnNone     =  0x00;  ///< no warnings
  static const int LaeMotorWarnCurrent  =  0x01;  ///< over current
  static const int LaeMotorWarnTemp     =  0x02;  ///< over temperature
  static const int LaeMotorWarnVoltage  =  0x04;  ///< under/over voltage

  //
  // Motor properties.
  //
  // For v2.0 platforms, there is 1 battery circuit with a maximum of 10 amps.
  // For v2.1+ platforms, there are 2 battery curcuits with a max of 20 amps.
  //
  static const int    LaeQuadPulsesPerRev = 64;     ///< pulses/motor revolution
  static const double LaeMotorGearRatio   = 30.0;   ///< gear ratio
  static const double LaeMotorRatedMaxRpm = 350.0;  ///< max output shaft rpm
  static const double LaeMotorStallTorque = 0.776771;
                                                    ///< stall torque (N-m) @5A
  static const double LaeMotorRatedAmps   = 5.0;    ///< motor rated max amps.
  static const double LaeMotorMaxAmps     = 30.0;   ///< max amp limit/motor
  static const double LaeMotorMaxAmps_2_0 = 2.5;    ///< max amp limit/motor
  static const double LaeMotorMaxAmps_2_1 = 5.0;    ///< max amp limit/motor


  // ---------------------------------------------------------------------------
  // LaeMotorCtlrChipSelect Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief RoboClaw motor controller chip select class.
   *
   * The RoboClawChipSelect class instance is used to select the motor
   * controller, given its address.
   *
   * For Laelaps, this is accomplished through a digital GPIO pin.
   *
   * With the new RoboClaw firmware, chip select is no longer necessary. 
   * This class is deprecated.
   */
  class LaeMotorCtlrChipSelect : public motor::roboclaw::RoboClawChipSelect
  {
  public:
    // motor controller chip select values
    static const byte_t LaeMotorCtlrCsHigh = LaeMotorCtlrAddrFront;
                    ///< motor controller address associated with high signal
    static const byte_t LaeMotorCtlrCsLow  = LaeMotorCtlrAddrRear;
                    ///< motor controller address associated with low signal

    /*!
     * \brief Default constructor.
     */
    LaeMotorCtlrChipSelect();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeMotorCtlrChipSelect();

    /*!
     * \brief Open GPIO pin interface.
     *
     * \param pinGpio   The sysfs exported GPIO number.
     *
     * \copydoc doc_return_std
     */
    virtual int open(int pinGpio);

    /*!
     * \brief Close GPIO pin interface.
     *
     * \copydoc doc_return_std
     */
    virtual int close();

    /*!
     * \brief Test if connection is open.
     *
     * \return Returns true or false.
     */
    virtual bool isOpen()
    {
      return m_fdGpio >= 0;
    }

    /*!
     * \brief Motor controller select function.
     *
     * The motor controllers are on a multi-drop serial bus. Serial does not
     * support multiple tx drivers on the same bus. Since there are two motor
     * controllers, each with a transmit line (odroid receive), the select
     * function disconnects one tx while connects the target tx.
     *
     * \param fd        Open serial file descriptor.
     * \param addrSel   Target motor controller to be selected. Controllers
     *                  are identified by their address.
     */
    virtual void select(int fd, byte_t addrSel);

  protected:
    int   m_pinGpio;
    int   m_fdGpio;
  };

} // namespace laelaps

#endif // _LAE_MOTOR_H
