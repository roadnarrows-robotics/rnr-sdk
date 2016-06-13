////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaServoMX12W.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows MX-12W Dynamixel Servo Class Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
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

#ifndef _DYNA_SERVO_MX12W_H
#define _DYNA_SERVO_MX12W_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/MX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"


// ---------------------------------------------------------------------------
// MX-12W Dynamixel Servo Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief MX-12W Dynamixel Servo Class.
 *
 * The DynaServoMX12W class provides the specific interface to the MX-12W
 * Dynamixel servos.
 */
class DynaServoMX12W : public DynaServoGeneric
{
public:
  /*! Model number */
  static const int DYNA_MODEL_NUM = DYNA_MODEL_NUM_MX12W;

  /*!
   * \brief Bare-bones initialization constructor.
   *
   * May be used be derived classes to avoid undue communication and
   * initializaton overhead.
   *
   * \param comm      Dynamixel bus communication instance.
   * 
   */
  DynaServoMX12W(DynaComm &comm) : DynaServoGeneric(comm)
  {
  }

  /*!
   * \brief Initialization constructor.
   *
   * \param comm      Dynamixel bus communication instance.
   * \param nServoId  Servo Id.
   * \param uModelNum Servo model number.
   * \param uFwVer    Servo firmware version.
   */
  DynaServoMX12W(DynaComm &comm,
                 int       nServoId,
                 uint_t    uModelNum = DYNA_MODEL_NUM,
                 uint_t    uFwVer    = DYNA_FWVER_NA);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaServoMX12W();


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Dump contents of the servo EEPROM and RAM control tables.
   */
  virtual void Dump();

protected:
  /*!
   * \brief Initialize servo class instance.
   *
   * \param nServoId  Servo Id.
   * \param uFwVer    Servo firmware version.
   */
  void Init(int nServoid, uint_t uFwVer);

  /*!
   * \brief Initialize servo fixed specification data.
   */
  void InitSpec();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Linking Fuctions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Field Packing Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
};


#endif // _DYNA_SERVO_MX12W_H
