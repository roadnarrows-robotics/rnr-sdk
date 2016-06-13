////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      EX.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows EX Series Dynamixel Declarations.
 *
 * \note Only deltas from base values found in Dynamixel.h are defined here.
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

#ifndef _EX_H
#define _EX_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// EX Models and Specifications
// ---------------------------------------------------------------------------

/*!
 * \addtogroup dyna_servo_model_num  
 *
 * \{
 */
#define DYNA_MODEL_NUM_EX106P     0x006b  ///< EX-106+
/*! \} */

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_ex106p EX-106+ Specification
 *
 * \{
 */
#define DYNA_EX106P_SPEC_WEIGHT_G         154.0 ///< weight (grams)
#define DYNA_EX106P_SPEC_WIDTH_MM         40.2  ///< width (mm)
#define DYNA_EX106P_SPEC_HEIGHT_MM        65.1  ///< height (mm)
#define DYNA_EX106P_SPEC_DEPTH_MM         46.0  ///< depth (mm)

#define DYNA_EX106P_SPEC_POS_RES_DEG      0.06  ///< resolution (degrees)
#define DYNA_EX106P_SPEC_GEAR_RATIO       184.0 ///< gear reduction ratio : 1

#define DYNA_EX106P_SPEC_STALL_TORQUE_KGF 107.0
          ///< maximum stall torque (kgf) at optimal power

#define DYNA_EX106P_SPEC_MAX_SPEED_RPM    91.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_EX106P_SPEC_ANGLE_MIN_DEG    0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_EX106P_SPEC_ANGLE_MAX_DEG    251.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_EX106P_SPEC_MODES            (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_EX106P_SPEC_HAS_360_POS      true
          ///< servo does provide 360\h_deg position data

#define DYNA_EX106P_SPEC_TEMP_MIN_C       (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_EX106P_SPEC_TEMP_MAX_C       80.0
          ///< maximum recommended operational temperature (C)

#define DYNA_EX106P_SPEC_VOLT_MIN_V       12.0
          ///< minimum operational voltage (V)

#define DYNA_EX106P_SPEC_VOLT_MAX_V       18.5
          ///< maximum operational voltage (V)

#define DYNA_EX106P_SPEC_CTL_METHOD       DYNA_CTL_METHOD_COMPLIANCE
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// ---------------------------------------------------------------------------
// EX-106+
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// EX-106+ Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_ex106p  EX-106+ Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_ex106p
 * \defgroup dyna_common_ex106p_pos  EX-106+ Position Common Parameters
 *
 * \{
 */
#define DYNA_EX106P_POS_MASK       0xffff ///< valid data mask
#define DYNA_EX106P_POS_MIN_RAW    0      ///< minimum raw angular position
#define DYNA_EX106P_POS_MAX_RAW    4095   ///< maximum raw angular position
#define DYNA_EX106P_POS_MODULO     4096   ///< continuous position mod [0-max]
#define DYNA_EX106P_POS_MIN_DEG    0      ///< minimum angular position
#define DYNA_EX106P_POS_MAX_DEG    250.92 ///< maximum angular position
#define DYNA_EX106P_POS_RES_DEG \
    ((double)DYNA_EX106P_POS_MAX_DEG/(double)DYNA_EX106P_POS_MAX_RAW)
                             ///< 0.06\h_deg (servo mode) resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// EX-106+ Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_ex106p  EX-106+ Control Table Memory Map
 *
 * EX-106+ EEPROM and RAM memory map.
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_layout  Address Layout
 * \{
 */
#define DYNA_EX106P_EEPROM_MIN_ADDR  DYNA_EEPROM_MIN_ADDR 
                                      ///< EX-106+ minimum EEPROM address
#define DYNA_EX106P_EEPROM_MAX_ADDR  DYNA_EEPROM_MAX_ADDR
                                      ///< EX-106+ maximum EEPROM address
#define DYNA_EX106P_RAM_MIN_ADDR     DYNA_RAM_MIN_ADDR
                                      ///< EX-106+ minimum RAM address
#define DYNA_EX106P_RAM_MAX_ADDR     57
                                      ///< EX-106+ maximum RAM address
#define DYNA_EX106P_MEM_MAX_SIZ      58
                                      ///< EX-106+ maximum control memory size
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_lim_cw EEPROM Clockwise Angle Limit.
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_common_ex106p_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_EX106P_ADDR_LIM_CW_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_lim_ccw EEPROM Counterclockwise Angle Limit.
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_common_ex106p_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_EX106P_ADDR_LIM_CCW_MASK  0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_drive_mode EEPROM Drive mode.
 *
 * Place to servos in a dual joint drive mode.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_EX106P_ADDR_DRIVE_MODE      10   ///< EEPROM drive mode (RW)

#define DYNA_EX106P_ADDR_DRIVE_MODE_MASK 0x03 ///< valid data mask

#define DYNA_EX106P_DRIVE_MODE_MS_MASTER 0x00 ///< dual joint master
#define DYNA_EX106P_DRIVE_MODE_MS_SLAVE  0x02 ///< dual joint slave
#define DYNA_EX106P_DRIVE_MODE_NR_NORM   0x00
                                    ///< dual joint normal rotation
#define DYNA_EX106P_DRIVE_MODE_NR_REV    0x01
                                    ///< dual joint reverse rotation

#define DYNA_EX106P_DRIVE_MODE_DFT \
  (DYNA_EX106P_DRIVE_MODE_MS_MASTER | DYNA_EX106P_DRIVE_MODE_NR_NORM)
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_ex106p_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_EX106P_ADDR_GOAL_POS_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_ex106p_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_EX106P_ADDR_CUR_POS_MASK 0xffff  ///< valid data mask

#define DYNA_EX106P_CUR_POS_S_MASK    0x0fff  ///< servo mode positiion mask

// In continuous mode, delta position = current position - last position
//                                        modulo 0xffff.
#define DYNA_EX106P_CUR_POS_C_MASK    0xffff  ///< continuous mode position mask
/*! \} */

/*!
 * \ingroup dyna_memmap_ex106p
 * \defgroup dyna_memmap_ex106p_sense RAM RAM Sensed Current
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_EX106P_ADDR_CURRENT_LSB       56 ///< sensed current draw (lsb)
#define DYNA_EX106P_ADDR_CURRENT_MSB       57 ///< sensed current draw (msb)

#define DYNA_EX106P_ADDR_CURRENT_MASK   0x3ff ///< valid data mask

#define DYNA_EX106P_CURRENT_RES_M_AMP      10 ///< 10mA resolution

#define DYNA_EX106P_CURRENT_MAG_MASK   0x01ff ///< current magnitude field mask
#define DYNA_EX106P_CURRENT_MAG_SHIFT       0 ///< current magnitude field shift
#define DYNA_EX106P_CURRENT_ZERO_RAW      512 ///< 0 Amp draw

/*! 
 * \brief Current(mA) / 10.
 *
 * \param val Raw field value.
 *
 */
#define DYNA_EX106P_CURRENT_M_AMP(val) ((val) & DYNA_EX106P_CURRENT_MAG_MASK)

/*! 
 * \brief Torque direction.
 *
 * \param val Raw field value.
 */
#define DYNA_EX106P_CURRENT_TORQUE_DIR(val) \
  ((val) < DYNA_EX106P_CURRENT_ZERO_RAW? DYNA_DIR_CCW: DYNA_DIR_CW)
/*! \} */


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _EX_H
