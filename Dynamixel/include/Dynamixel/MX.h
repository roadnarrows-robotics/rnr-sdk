////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      MX.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows MX Series Dynamixel Declarations.
 *
 * \note Only deltas from base values found in Dynamixel.h are defined here.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#ifndef _MX_H
#define _MX_H

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// MX Models and Specifications
// ---------------------------------------------------------------------------

/*!
 * \addtogroup dyna_servo_model_num
 *
 * \{
 */
#define DYNA_MODEL_NUM_MX12W      0x0168  ///< MX-12W
#define DYNA_MODEL_NUM_MX28       0x001d  ///< MX-28
#define DYNA_MODEL_NUM_MX64       0x0136  ///< MX-64T, MX-64R
#define DYNA_MODEL_NUM_MX106      0x0140  ///< MX-106T, MX-106R
/*! \} */


// ---------------------------------------------------------------------------
// MX-12W
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_mx12w MX-12W Specification
 *
 * \{
 */
#define DYNA_MX12W_SPEC_WEIGHT_G          54.6  ///< weight (grams)
#define DYNA_MX12W_SPEC_WIDTH_MM          32.0  ///< width (mm)
#define DYNA_MX12W_SPEC_HEIGHT_MM         50.0  ///< height (mm)
#define DYNA_MX12W_SPEC_DEPTH_MM          40.0  ///< depth (mm)

#define DYNA_MX12W_SPEC_POS_RES_DEG       0.088 ///< resolution (degrees)
#define DYNA_MX12W_SPEC_GEAR_RATIO        32.0  ///< gear reduction ratio : 1

#define DYNA_MX12W_SPEC_STALL_TORQUE_KGF  10.0
          ///< maximum stall torque (kgf) at optimal power (RDK unknown as yet)

#define DYNA_MX12W_SPEC_MAX_SPEED_RPM     470.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_MX12W_SPEC_ANGLE_MIN_DEG     0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_MX12W_SPEC_ANGLE_MAX_DEG     360.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_MX12W_MODE_MULTI_TURN        0x04
          ///< multi-turn mode
       
#define DYNA_MX12W_SPEC_MODES             (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_MX12W_SPEC_HAS_360_POS       true
          ///< servo does provide 360\h_deg position data

#define DYNA_MX12W_SPEC_TEMP_MIN_C        (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_MX12W_SPEC_TEMP_MAX_C        80.0
          ///< maximum recommended operational temperature (C)

#define DYNA_MX12W_SPEC_VOLT_MIN_V        10.0
          ///< minimum operational voltage (V)

#define DYNA_MX12W_SPEC_VOLT_MAX_V        14.8
          ///< maximum operational voltage (V)

#define DYNA_MX12W_SPEC_CTL_METHOD        DYNA_CTL_METHOD_PID
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-12W Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_mx12w  MX-12W Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_mx12w
 * \defgroup dyna_common_mx12w_pos  MX-12W Position Common Parameters
 *
 * \{
 */
#define DYNA_MX12W_POS_MASK       0x0fff  ///< valid data mask
#define DYNA_MX12W_POS_MIN_RAW    0       ///< minimum raw angular position
#define DYNA_MX12W_POS_MAX_RAW    4095    ///< maximum raw angular position
#define DYNA_MX12W_POS_MODULO     (DYNA_MX12W_POS_MAX_RAW+1)
                                          ///< servo position modulo [0-max]
#define DYNA_MX12W_POS_MIN_DEG    0.0     ///< minimum angular position
#define DYNA_MX12W_POS_MAX_DEG    360.0
                                        ///< maximum angular position
#define DYNA_MX12W_POS_RES_DEG \
    ((double)DYNA_MX12W_POS_MAX_DEG/(double)DYNA_MX12W_POS_MAX_RAW)
                                        ///< 0.088\h_deg (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_speed  Dynamixel Angular Speed Common Values
 *
 * \{
 */
#define DYNA_MX12W_SPEED_RES_RPM  0.114   ///< rpm (servo mode) resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-28 Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_mx28  MX-28 Control Table Memory Map
 *
 * \{
 */
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_cw EEPROM Clockwise Angle Limit.
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX28_ADDR_LIM_CW_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_ccw EEPROM Counterclockwise Angle Limit.
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX28_ADDR_LIM_CCW_MASK  0x0fff ///< valid data mask
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_MX28_ADDR_GOAL_POS_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_MX28_ADDR_CUR_POS_MASK   0x0fff  ///< valid data mask
/*! \} */


// ---------------------------------------------------------------------------
// MX-28
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_mx28 MX-28 Specification
 *
 * \{
 */
#define DYNA_MX28_SPEC_WEIGHT_G         72.0  ///< weight (grams)
#define DYNA_MX28_SPEC_WIDTH_MM         35.6  ///< width (mm)
#define DYNA_MX28_SPEC_HEIGHT_MM        50.6  ///< height (mm)
#define DYNA_MX28_SPEC_DEPTH_MM         35.5  ///< depth (mm)

#define DYNA_MX28_SPEC_POS_RES_DEG      0.088 ///< resolution (degrees)
#define DYNA_MX28_SPEC_GEAR_RATIO       193.0 ///< gear reduction ratio : 1

#define DYNA_MX28_SPEC_STALL_TORQUE_KGF 24.0
          ///< maximum stall torque (kgf) at optimal power

#define DYNA_MX28_SPEC_MAX_SPEED_RPM    55.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_MX28_SPEC_ANGLE_MIN_DEG    0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_MX28_SPEC_ANGLE_MAX_DEG    360.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_MX28_SPEC_MODES            (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_MX28_SPEC_HAS_360_POS      true
          ///< servo does provide 360\h_deg position data

#define DYNA_MX28_SPEC_TEMP_MIN_C       (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_MX28_SPEC_TEMP_MAX_C       80.0
          ///< maximum recommended operational temperature (C)

#define DYNA_MX28_SPEC_VOLT_MIN_V       9.0
          ///< minimum operational voltage (V)

#define DYNA_MX28_SPEC_VOLT_MAX_V       16.8
          ///< maximum operational voltage (V)

#define DYNA_MX28_SPEC_CTL_METHOD       DYNA_CTL_METHOD_PID
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-28 Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_mx28  MX-28 Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_mx28
 * \defgroup dyna_common_mx28_pos  MX-28 Position Common Parameters
 *
 * \{
 */
#define DYNA_MX28_POS_MASK      0x0fff  ///< valid data mask
#define DYNA_MX28_POS_MIN_RAW   0       ///< minimum raw angular position
#define DYNA_MX28_POS_MAX_RAW   4095    ///< maximum raw angular position
#define DYNA_MX28_POS_MODULO    (DYNA_MX28_POS_MAX_RAW+1)
                                        ///< servo position modulo [0-max]
#define DYNA_MX28_POS_MIN_DEG   0.0     ///< minimum angular position
#define DYNA_MX28_POS_MAX_DEG   360.0
                                        ///< maximum angular position
#define DYNA_MX28_POS_RES_DEG \
    ((double)DYNA_MX28_POS_MAX_DEG/(double)DYNA_MX28_POS_MAX_RAW)
                                        ///< 0.088\h_deg (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_speed  Dynamixel Angular Speed Common Values
 *
 * \{
 */
#define DYNA_MX28_SPEED_RES_RPM 0.114   ///< rpm (servo mode) resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-28 Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_mx28  MX-28 Control Table Memory Map
 *
 * \{
 */
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_cw EEPROM Clockwise Angle Limit.
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX28_ADDR_LIM_CW_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_ccw EEPROM Counterclockwise Angle Limit.
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX28_ADDR_LIM_CCW_MASK  0x0fff ///< valid data mask
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_MX28_ADDR_GOAL_POS_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_MX28_ADDR_CUR_POS_MASK   0x0fff  ///< valid data mask
/*! \} */


// ---------------------------------------------------------------------------
// MX-64
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_mx28 MX-64 Specification
 *
 * \{
 */
#define DYNA_MX64_SPEC_WEIGHT_G        126.0  ///< weight (grams)
#define DYNA_MX64_SPEC_WIDTH_MM         40.2  ///< width (mm)
#define DYNA_MX64_SPEC_HEIGHT_MM        61.1  ///< height (mm)
#define DYNA_MX64_SPEC_DEPTH_MM         41.0  ///< depth (mm)

#define DYNA_MX64_SPEC_POS_RES_DEG      0.088 ///< resolution (degrees)
#define DYNA_MX64_SPEC_GEAR_RATIO       200.0 ///< gear reduction ratio : 1

#define DYNA_MX64_SPEC_STALL_TORQUE_KGF 74.4
          ///< maximum stall torque (kgf-cm) at optimal power

#define DYNA_MX64_SPEC_MAX_SPEED_RPM    78.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_MX64_SPEC_ANGLE_MIN_DEG    0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_MX64_SPEC_ANGLE_MAX_DEG    360.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_MX64_SPEC_MODES            (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_MX64_SPEC_HAS_360_POS      true
          ///< servo does provide 360\h_deg position data

#define DYNA_MX64_SPEC_TEMP_MIN_C       (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_MX64_SPEC_TEMP_MAX_C       80.0
          ///< maximum recommended operational temperature (C)

#define DYNA_MX64_SPEC_VOLT_MIN_V       10.0
          ///< minimum operational voltage (V)

#define DYNA_MX64_SPEC_VOLT_MAX_V       14.8
          ///< maximum operational voltage (V)

#define DYNA_MX64_SPEC_CTL_METHOD       DYNA_CTL_METHOD_PID
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-64 Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_mx28  MX-64 Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_mx28
 * \defgroup dyna_common_mx28_pos  MX-64 Position Common Parameters
 *
 * \{
 */
#define DYNA_MX64_POS_MASK      0x0fff  ///< valid data mask
#define DYNA_MX64_POS_MIN_RAW   0       ///< minimum raw angular position
#define DYNA_MX64_POS_MAX_RAW   4095    ///< maximum raw angular position
#define DYNA_MX64_POS_MODULO    (DYNA_MX64_POS_MAX_RAW+1)
                                        ///< servo position modulo [0-max]
#define DYNA_MX64_POS_MIN_DEG   0.0     ///< minimum angular position
#define DYNA_MX64_POS_MAX_DEG   360.0
                                        ///< maximum angular position
#define DYNA_MX64_POS_RES_DEG \
    ((double)DYNA_MX64_POS_MAX_DEG/(double)DYNA_MX64_POS_MAX_RAW)
                                        ///< 0.088\h_deg (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_speed  Dynamixel Angular Speed Common Values
 *
 * \{
 */
#define DYNA_MX64_SPEED_RES_RPM 0.114   ///< rpm (servo mode) resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-64 Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_mx28  MX-64 Control Table Memory Map
 *
 * \{
 */
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_cw EEPROM Clockwise Angle Limit.
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX64_ADDR_LIM_CW_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_lim_ccw EEPROM Counterclockwise Angle Limit.
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX64_ADDR_LIM_CCW_MASK  0x0fff ///< valid data mask
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_MX64_ADDR_GOAL_POS_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx28
 * \defgroup dyna_memmap_mx28_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_MX64_ADDR_CUR_POS_MASK   0x0fff  ///< valid data mask
/*! \} */


// ---------------------------------------------------------------------------
// MX-106
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_spec
 * \defgroup dyan_spec_mx106 MX-106 Specification
 *
 * \{
 */
#define DYNA_MX106_SPEC_WEIGHT_G         153.0  ///< weight (grams)
#define DYNA_MX106_SPEC_WIDTH_MM          40.2  ///< width (mm)
#define DYNA_MX106_SPEC_HEIGHT_MM         61.1  ///< height (mm)
#define DYNA_MX106_SPEC_DEPTH_MM          46.0  ///< depth (mm)

#define DYNA_MX106_SPEC_POS_RES_DEG       0.088 ///< resolution (degrees)
#define DYNA_MX106_SPEC_GEAR_RATIO        225.0 ///< gear reduction ratio : 1

#define DYNA_MX106_SPEC_STALL_TORQUE_NM   8.4
          ///< maximum stall torque (newton-meter) at optimal power

#define DYNA_MX106_SPEC_MAX_SPEED_RPM     45.0
          ///< maximum no-load speed (rpm) at optimal power

#define DYNA_MX106_SPEC_ANGLE_MIN_DEG     0.0
          ///< minimum rotation angle in servo mode (deg)

#define DYNA_MX106_SPEC_ANGLE_MAX_DEG     360.0
          ///< maximum rotation angle in servo mode (deg)

#define DYNA_MX106_SPEC_MODES             (DYNA_MODE_SERVO|DYNA_MODE_CONTINUOUS)
          ///< supported modes (see \ref dyna_servo_mode)

#define DYNA_MX106_SPEC_HAS_360_POS       true
          ///< servo does provide 360\h_deg position data

#define DYNA_MX106_SPEC_TEMP_MIN_C        (-5.0)
          ///< minimum recommended operational temperature (C)

#define DYNA_MX106_SPEC_TEMP_MAX_C        80.0
          ///< maximum recommended operational temperature (C)

#define DYNA_MX106_SPEC_VOLT_MIN_V        10.0
          ///< minimum operational voltage (V)

#define DYNA_MX106_SPEC_VOLT_MAX_V        14.8
          ///< maximum operational voltage (V)

#define DYNA_MX106_SPEC_CTL_METHOD        DYNA_CTL_METHOD_PID
          ///< position control method (see \ref dyna_ctl_method)
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-106 Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_mx106  MX-106 Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_mx106
 * \defgroup dyna_common_mx106_pos  MX-106 Position Common Parameters
 *
 * \{
 */
#define DYNA_MX106_POS_MASK      0x0fff  ///< valid data mask
#define DYNA_MX106_POS_MIN_RAW   0       ///< minimum raw angular position
#define DYNA_MX106_POS_MAX_RAW   4095    ///< maximum raw angular position
#define DYNA_MX106_POS_MODULO    (DYNA_MX106_POS_MAX_RAW+1)
                                        ///< servo position modulo [0-max]
#define DYNA_MX106_POS_MIN_DEG   0.0     ///< minimum angular position
#define DYNA_MX106_POS_MAX_DEG   360.0
                                        ///< maximum angular position
#define DYNA_MX106_POS_RES_DEG \
    ((double)DYNA_MX106_POS_MAX_DEG/(double)DYNA_MX106_POS_MAX_RAW)
                                        ///< 0.088\h_deg (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_speed  Dynamixel Angular Speed Common Values
 *
 * \{
 */
#define DYNA_MX106_SPEED_RES_RPM 0.114   ///< rpm (servo mode) resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// MX-106 Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_mx106  MX-106 Control Table Memory Map
 *
 * \{
 */
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx106
 * \defgroup dyna_memmap_mx106_lim_cw EEPROM Clockwise Angle Limit.
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX106_ADDR_LIM_CW_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx106
 * \defgroup dyna_memmap_mx106_lim_ccw EEPROM Counterclockwise Angle Limit.
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_MX106_ADDR_LIM_CCW_MASK  0x0fff ///< valid data mask
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_mx106
 * \defgroup dyna_memmap_mx106_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_MX106_ADDR_GOAL_POS_MASK   0x0fff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_mx106
 * \defgroup dyna_memmap_mx106_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_ex_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_MX106_ADDR_CUR_POS_MASK   0x0fff  ///< valid data mask
/*! \} */


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _MX_H
