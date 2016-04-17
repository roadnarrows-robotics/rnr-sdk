////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoAX12.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows AX-12, AX-12+, AX-12A Dynamixel Servo Class Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
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

#ifndef _DYNA_SERVO_AX12_H
#define _DYNA_SERVO_AX12_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/AX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"


// ---------------------------------------------------------------------------
// AX-12 Dynamixel Servo Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief AX-12, AX-12+, AX-12A Dynamixel Servo Class.
 *
 * The DynaServoAX12 class provides the specific interface to the AX-12
 * (and AX-12+ and AX-12A) Dynamixel servos
 */
class DynaServoAX12 : public DynaServoGeneric
{
public:
  static const int DYNA_MODEL_NUM = DYNA_MODEL_NUM_AX12;

  /*!
   * \brief Bare-bones initialization constructor.
   *
   * May be used be derived classes to avoid undue communication and
   * initializaton overhead.
   *
   * \param comm      Dynamixel bus communication instance.
   * 
   */
  DynaServoAX12(DynaComm &comm) : DynaServoGeneric(comm)
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
  DynaServoAX12(DynaComm &comm,
                   int       nServoId,
                   uint_t    uModelNum = DYNA_MODEL_NUM,
                   uint_t    uFwVer    = DYNA_FWVER_NA);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaServoAX12();


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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


#endif // _DYNA_SERVO_AX12_H
