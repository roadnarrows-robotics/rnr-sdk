////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      AX.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows AX Series Dynamixel Declarations.
 *
 * \note Only deltas from base values found in Dynamixel.h are defined here.
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

#ifndef _AX_H
#define _AX_H

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// AX Models and Specifications
// ---------------------------------------------------------------------------

/*!
 * \addtogroup dyna_servo_model_num
 *
 * \{
 */
#define DYNA_MODEL_NUM_AX12       0x000c  ///< AX-12, AX-12+, AX-12A
/*! \} */

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_ax12 AX-12 Specification
 *
 * \{
 */
#define DYNA_AX12_SPEC_WEIGHT_G         53.5  ///< weight (grams)
#define DYNA_AX12_SPEC_WIDTH_MM         32.0  ///< width (mm)
#define DYNA_AX12_SPEC_HEIGHT_MM        50.0  ///< height (mm)
#define DYNA_AX12_SPEC_DEPTH_MM         40.0  ///< depth (mm)

#define DYNA_AX12_SPEC_POS_RES_DEG      0.29  ///< resolution (degrees)
#define DYNA_AX12_SPEC_GEAR_RATIO       254.0 ///< gear reduction ratio : 1

#define DYNA_AX12_SPEC_STALL_TORQUE_KGF 15.0
          ///< maximum stall torque (kgf) at optimal power

#define DYNA_AX12_SPEC_MAX_SPEED_RPM    59.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_AX12_SPEC_ANGLE_MIN_DEG    0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_AX12_SPEC_ANGLE_MAX_DEG    300.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_AX12_SPEC_MODES            (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_AX12_SPEC_HAS_360_POS      false
          ///< servo does not provide 360\h_deg position data

#define DYNA_AX12_SPEC_TEMP_MIN_C       (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_AX12_SPEC_TEMP_MAX_C       70.0
          ///< maximum recommended operational temperature (C)

#define DYNA_AX12_SPEC_VOLT_MIN_V       9.0
          ///< minimum operational voltage (V)

#define DYNA_AX12_SPEC_VOLT_MAX_V       12.0
          ///< maximum operational voltage (V)
        
#define DYNA_AX12_SPEC_CTL_METHOD       (DYNA_CTL_METHOD_COMPLIANCE)
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// ---------------------------------------------------------------------------
// AX-12
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// AX-12 Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// AX-12 Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _AX_H
