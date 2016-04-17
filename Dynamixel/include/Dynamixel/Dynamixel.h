////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// File:      Dynamixel.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Top-Level Package Header File.
 *
 *  Defines the unifying concepts of the RoadNarrows implementation of the
 *  Robotis Dynamixel servos including:
 *  \li error codes
 *  \li common parameter values 
 *  \li generic base dynamixel descriptions
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

#ifndef _DYNAMIXEL_H
#define _DYNAMIXEL_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// Dynamixel Package Error Codes
// ---------------------------------------------------------------------------

/*!
 * \addtogroup dyna_ecodes 
 *
 * RoadNarrows Dynamixel package-wide error codes.
 * \{
 */
#define DYNA_OK                       0 ///< not an error, success

#define DYNA_ECODE_GEN                1 ///< general, unspecified error
#define DYNA_ECODE_SYS                2 ///< system (errno) error
#define DYNA_ECODE_INTERNAL           3 ///< internal error (bug)
#define DYNA_ECODE_BADEC              4 ///< bad error code
#define DYNA_ECODE_BAD_UNITS          5 ///< unsupported or bad units
#define DYNA_ECODE_BAD_VAL            6 ///< bad value
#define DYNA_ECODE_NO_SERVO           7 ///< no servo found
#define DYNA_ECODE_BAD_DEV            8 ///< no or bad serial device
#define DYNA_ECODE_BADF               9 ///< no comm object or not open
#define DYNA_ECODE_ECOMM             10 ///< dynamixel communication error
#define DYNA_ECODE_TX_FAIL           11 ///< dynamixel transmit packet failure
#define DYNA_ECODE_RX_FAIL           12 ///< dynamixel receive packet failure
#define DYNA_ECODE_TX_ERROR          13 ///< dynamixel transmit packet error
#define DYNA_ECODE_RX_TIMEOUT        14 ///< dynamixel receive packet time out
#define DYNA_ECODE_RX_BAD_PKT        15 ///< dynamixel receive bad packet
#define DYNA_ECODE_ESERVO            16 ///< dynamixel servo in error condition
#define DYNA_ECODE_RSRC              17 ///< no resource
#define DYNA_ECODE_NOT_SUPP          18 ///< feature/function not supported
#define DYNA_ECODE_LINKED            19 ///< linked servos error
#define DYNA_ECODE_SLAVE             20 ///< invalid slave operation
#define DYNA_ECODE_BOTSENSE          21 ///< BotSense proxy error
#define DYNA_ECODE_PARSE             22 ///< Shell parse error.
#define DYNA_ECODE_RUNTIME           23 ///< Shell run-time error.
#define DYNA_ECODE_NOEXEC            24 ///< Shell parse error.

#define DYNA_ECODE_NUMOF             25 ///< number of error codes
/*! \} */


// ---------------------------------------------------------------------------
// Dynamixel Base Parameters
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_servo
 * \defgroup dyna_servo_model_num  Supported Dynamixel Models
 *
 * \{
 */
#define DYNA_MODEL_NUM_GENERIC    0x0000  ///< generic, base model
/*! \} */

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_base  Base Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_servo_fwver Special Firmware Version
 *
 * \{
 */
#define DYNA_FWVER_NA             0x00    ///< firmware version not available
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_servo_id  Dynamixel Identifiers
 *
 * \{
 */
#define DYNA_ID_NONE           -1       ///< no servo id
#define DYNA_ID_MIN             0       ///< minimum servo id
#define DYNA_ID_MAX           253       ///< maximum servo id
#define DYNA_ID_NUMOF         254       ///< number of unique servo id's
#define DYNA_ID_BROADCAST     254       ///< broadcast id
#define DYNA_ID_RESERVED      255       ///< reserved id
#define DYNA_ID_MASK          0xff      ///< id mask
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_servo_mode  Dynamixel Operational Modes
 *
 * In servo mode, the servo has limited rotation (e.g. 300\h_deg). The servo,
 * however, can move to a new goal position without host asssitance using the
 * servo control method, and moves at the current goal speed.
 *
 * In continuous (wheel) mode, the goal position is not applicable. The servo
 * is moved by setting the goal speed and direction (cw/ccw). Position data
 * is invalid within the servo position limits and so cannot be trusted. On
 * newer servos, 360\h_deg position data is availble and the host may then 
 * provide positioning control.
 *
 * \{
 */
#define DYNA_MODE_SERVO       0x01  ///< servo mode with limited rotation
#define DYNA_MODE_CONTINUOUS  0x02  ///< continuous mode with/without position
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_servo_link_type  Dynamixel Servo Link Types
 *
 * \{
 */
#define DYNA_LINK_NONE       0   ///< servo is not linked to another servo
#define DYNA_LINK_MASTER     1   ///< servo is linked, serving as the master
#define DYNA_LINK_SLAVE      2   ///< servo is linked, serving as the slave
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_servo_dir  Dynamixel Direction Common Values
 *
 * \{
 */
#define DYNA_DIR_CW         (-1)      ///< clockwise direction
#define DYNA_DIR_NONE         0       ///< no direction
#define DYNA_DIR_CCW          1       ///< counterclockwise direction

/*!
 * \brief Get the direction component, given the rotational scalar.
 *
 * \param scalar  Signed rotational scaler such as speed or load.
 */
#define DYNA_GET_DIR(scalar) ((scalar)<0? DYNA_DIR_CW: DYNA_DIR_CCW)
/*! \} */

/*!
 * \ingroup dyna_common_base
 * \defgroup dyna_ctl_method  Dynamixel Internal Position Control Methods
 *
 * The compliance control method specifies a simple torque (speed) ramp down
 * and margin to achieve the goal position.
 *
 * The PID control method, that replaced the above method in newer servos,
 * defines the set of constants used by the servo to achieve the goal
 * position.
 * \{
 */
#define DYNA_CTL_METHOD_NONE        0 ///< no control method
#define DYNA_CTL_METHOD_COMPLIANCE  1 ///< compliance control method
#define DYNA_CTL_METHOD_PID         2 ///< pid control method
/*! \} */


// ---------------------------------------------------------------------------
// Generic Dynamixel
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Generic Servo Common Parameters
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_common
 * \defgroup dyna_common_gen  Generic Servo Parameters
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_pos  Dynamixel Position Common Values
 *
 * \{
 */
#define DYNA_POS_MASK       0x03ff  ///< valid data mask
#define DYNA_POS_MIN_RAW    0       ///< minimum raw angular position
#define DYNA_POS_MAX_RAW    1023    ///< maximum raw angular position
#define DYNA_POS_MODULO     (DYNA_POS_MAX_RAW+1)
                                    ///< servo position modulo [0-max]
#define DYNA_POS_MIN_DEG    0       ///< minimum angular position
#define DYNA_POS_MAX_DEG    300     ///< maximum angular position
#define DYNA_POS_RES_DEG    ((double)DYNA_POS_MAX_DEG/(double)DYNA_POS_MAX_RAW)
                                    ///< 0.29\h_deg (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_speed  Dynamixel Angular Speed Common Values
 *
 * \{
 */
#define DYNA_SPEED_MASK       0x03ff  ///< valid data mask
#define DYNA_SPEED_CONT_STOP  0       ///< continuous mode: stop
#define DYNA_SPEED_MAX_NO_CTL 0       ///< servo mode: max speed with no control
#define DYNA_SPEED_MIN_CTL    1       ///< mininum raw speed with control
#define DYNA_SPEED_MAX_CTL    1023    ///< maxinum raw speed with control
#define DYNA_SPEED_MIN_RAW    0       ///< minimum raw value
#define DYNA_SPEED_MAX_RAW    1023    ///< maximum raw value
#define DYNA_SPEED_RES_RPM    0.111   ///< rpm (servo mode) resolution
#define DYNA_SPEED_RES_PCT    (100.0/DYNA_SPEED_MAX_RAW)
                                      ///< percent (servo mode) resolution
#define DYNA_SPEED_RES_PMIL   (1000.0/DYNA_SPEED_MAX_RAW)
                                      ///< permil (servo mode) resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_torque  Dynamixel Torque Common Values
 *
 * \{
 */
#define DYNA_TORQUE_MASK        0x03ff  ///< valid data mask
#define DYNA_TORQUE_MIN_RAW     0       ///< minimum raw torque
#define DYNA_TORQUE_MAX_RAW     1023    ///< maximum raw torque
#define DYNA_TORQUE_RES_PCT     (100.0/DYNA_TORQUE_MAX_RAW)
                                        ///< percent of maximum resolution
#define DYNA_TORQUE_RES_PMIL    (1000.0/DYNA_TORQUE_MAX_RAW)
                                        ///< permil of maximum resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_temp  Dynamixel Temperature Common Values
 *
 * \{
 */
#define DYNA_TEMP_MIN_RAW       10    ///< minimum raw temperature
#define DYNA_TEMP_MAX_RAW       99    ///< maximum raw temperature
#define DYNA_TEMP_MIN_C         DYNA_TEMP_MIN_RAW ///< 10C temperature minimum 
#define DYNA_TEMP_MAX_C         DYNA_TEMP_MAX_RAW ///< 99C temperature maximum 
#define DYNA_TEMP_RES_C         1     ///< 1\h_deg Celsius resolution
/*! \} */

/*!
 * \ingroup dyna_common_gen
 * \defgroup dyna_servo_volt  Dynamixel Voltage Common Values
 *
 * \{
 */
#define DYNA_VOLT_MIN_RAW       50    ///< minimum raw temperature
#define DYNA_VOLT_MAX_RAW       250    ///< maximum raw temperature
#define DYNA_VOLT_RES_V         0.1   ///< 0.1V resolution
/*! \} */


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Generic Servo Control Table
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup dyna_memmap
 * \defgroup dyna_memmap_gen  Generic Control Table Memory Map
 *
 * Default servo EEPROM and RAM memory map. Specific servos may extend, remove,
 * or override.
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_layout  Address Layout
 * \{
 */
#define DYNA_EEPROM_MIN_ADDR       0 ///< minimum EEPROM adderss
#define DYNA_EEPROM_MAX_ADDR      18 ///< maximum EEPROM adderss
#define DYNA_RAM_MIN_ADDR         24 ///< minimum RAM adderss
#define DYNA_RAM_MAX_ADDR         49 ///< maximum RAM adderss
#define DYNA_MEM_MAX_SIZ          50 ///< maximum control memory size (bytes)
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// EEPROM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_model_num EEPROM Model Number
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_MODEL_NUM_LSB     0       ///< model number lsb (R)
#define DYNA_ADDR_MODEL_NUM_MSB     1       ///< model number msb (R)

#define DYNA_ADDR_MODEL_NUM_MASK    0xffff  ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_fwver EEPROM Firmware Version
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_FWVER             2       ///< firmware version (R)

#define DYNA_ADDR_FWVER_MASK        0xff    ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_id EEPROM Servo Identifier
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_ID                3       ///< dynamixel id (RW)

#define DYNA_ADDR_ID_MASK           0xff    ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_baud_rate EEPROM Baud Rate 
 *
 * The Dynamixel servos specify the baud rate through a baud number enumeration.
 * The extended baud number (rates) are not supported by all Dynamixel servos.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_BAUD_RATE         4     ///< baud rate enumeration (RW)

#define DYNA_ADDR_BAUD_RATE_MASK    0xff  ///< valid data mask

// raw baud number enumeration
#define DYNA_BAUDNUM_1000000        1 ///< 1,000,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_500000         3 ///<   500,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_400000         4 ///<   500,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_250000         7 ///<   250,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_200000         9 ///<   200,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_115200        16 ///<   115,200 bps at -2.124% tolerance
#define DYNA_BAUDNUM_57600         34 ///<    57,600 bps at  0.794 % tolerance
#define DYNA_BAUDNUM_19200        103 ///<    19,200 bps at -0.160% tolerance
#define DYNA_BAUDNUM_9600         207 ///<     9,600 bps at -0.160% tolerance

// extended baud rate numbers
#define DYNA_BAUDNUM_EXT_2250000  250 ///< 2,250,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_EXT_2500000  251 ///< 2,500,000 bps at 0.0% tolerance
#define DYNA_BAUDNUM_EXT_3000000  252 ///< 3,000,000 bps at 0.0% tolerance

#define DYNA_BAUDNUM_NUMOF         12 ///< number of supported bit rates

#define DYNA_BAUDNUM_DFT           34 ///< default baud number
#define DYNA_BAUDRATE_DFT       57600 ///< default baud rate

/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_t_ret_delay EEPROM Return Delay Time
 *
 * The delay time between the transmission of Instruction Packet until the
 * return of the Status Packet. 
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_T_RET_DELAY         5     ///< return delay time (RW)

#define DYNA_ADDR_T_RET_DELAY_MASK    0xff  ///< valid data mask

#define DYNA_T_RET_DELAY_MIN_RAW      0x00  ///< raw minimum delay
#define DYNA_T_RET_DELAY_MAX_RAW      0xff  ///< raw maximum delay
#define DYNA_T_RET_DELAY_DFT_RAW      0x00  ///< raw default
#define DYNA_T_RET_DELAY_RES_USEC     2     ///< 2\h_usec resolution
#define DYNA_T_RET_DELAY_MIN_USEC \
  (DYNA_T_RET_DELAY_MIN_RAW * DYNA_T_RET_DELAY_RES_USEC)
                                            ///< \h_usec minimum
#define DYNA_T_RET_DELAY_MAX_USEC \
  (DYNA_T_RET_DELAY_MAX_RAW * DYNA_T_RET_DELAY_RES_USEC)
                                            ///< \h_usec maximum
#define DYNA_T_RET_DELAY_DFT_USEC \
  (DYNA_T_RET_DELAY_DFT_RAW * DYNA_T_RET_DELAY_RES_USEC)
                                            ///< \h_usec default
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_cw EEPROM Clockwise Angle Limit
 *
 * Minimum angle (goal position) limit.
 *
 * \sa \ref dyna_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_LIM_CW_LSB    6       ///< clockwise angle limit lsb (RW)
#define DYNA_ADDR_LIM_CW_MSB    7       ///< clockwise angle limit msb (RW)

#define DYNA_ADDR_LIM_CW_MASK   0x03ff  ///< valid data mask

#define DYNA_CW_POS_CONT_MODE   0       ///< continuous mode (with ccw limit)
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_ccw EEPROM Counterclockwise Angle Limit
 *
 * Maximum angle (goal position) limit.
 *
 * \sa \ref dyna_servo_pos for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_LIM_CCW_LSB   8   ///< counterclockwise angle limit lsb (RW)
#define DYNA_ADDR_LIM_CCW_MSB   9   ///< counterclockwise angle limit msb (RW)

#define DYNA_ADDR_LIM_CCW_MASK  0x03ff  ///< valid data mask

#define DYNA_CCW_POS_CONT_MODE  0       ///< continuous mode (with cw limit)
/*! \} */

#define DYNA_ADDR_RESERVED_1      10 ///< reserved 

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_temp_max EEPROM Temperature Limit
 *
 * Maximum operating temperature limit.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LIM_TEMP_MAX    11    ///< maximum temperature limit (RW)

#define DYNA_ADDR_LIM_TEMP_MASK   0x3f  ///< valid data mask

#define DYNA_LIM_TEMP_MIN_C       DYNA_TEMP_MIN_C ///< minimum maximum limit
#define DYNA_LIM_TEMP_MAX_C       DYNA_TEMP_MAX_C ///< maximum maximum limit
#define DYNA_LIM_TEMP_DFT_C       80    ///< 80C default maximum limit
#define DYNA_LIM_TEMP_RES_C       DYNA_TEMP_RES_C
                                        ///< 1\h_deg Celsius resolution
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_volt_min EEPROM Minimum Voltage Limit
 *
 * Minimum operating voltage limit.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LIM_VOLT_MIN          12    ///< lowest voltage limit (RW)

#define DYNA_ADDR_LIM_VOLT_MIN_MASK     0xff  ///< valid data mask

#define DYNA_LIM_VOLT_MIN_MIN_RAW       DYNA_VOLT_MIN_RAW
                                              ///< minimum minimum raw limit
#define DYNA_LIM_VOLT_MIN_MAX_RAW       DYNA_VOLT_MAX_RAW
                                              ///< maximum minimum raw limit
#define DYNA_LIM_VOLT_MIN_DFT_RAW       60    ///< default minimum raw limit
#define DYNA_LIM_VOLT_MIN_RES_V         DYNA_VOLT_RES_V ///< 0.1V resolution
#define DYNA_LIM_VOLT_MIN_MIN_V \
  (DYNA_LIM_VOLT_MIN_MIN_RAW * DYNA_LIM_VOLT_MIN_RES_V)
                                              ///< min minimum volts limit
#define DYNA_LIM_VOLT_MIN_MAX_V \
  (DYNA_LIM_VOLT_MIN_MAX_V * DYNA_LIM_VOLT_MIN_RES_V)
                                              ///< max minimum volts limit
#define DYNA_LIM_VOLT_MIN_DFT_V \
  (DYNA_LIM_VOLT_MIN_DFT_V * DYNA_LIM_VOLT_MIN_RES_V)
                                              ///< default minimum volts limit
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_volt_max EEPROM Maximum Voltage Limit
 *
 * Maximum operating voltage limit.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LIM_VOLT_MAX          13    ///< highest voltage limit (RW)

#define DYNA_ADDR_LIM_VOLT_MAX_MASK     0xff  ///< valid data mask

#define DYNA_LIM_VOLT_MAX_MIN_RAW       DYNA_VOLT_MIN_RAW
                                              ///< minimum maximum raw limit
#define DYNA_LIM_VOLT_MAX_MAX_RAW       DYNA_VOLT_MAX_RAW
                                              ///< maximum maximum raw limit
#define DYNA_LIM_VOLT_MAX_DFT_RAW       190   ///< default maximum raw limit
#define DYNA_LIM_VOLT_MAX_RES_V         DYNA_VOLT_RES_V ///< 0.1V resolution
#define DYNA_LIM_VOLT_MAX_MIN_V \
  (DYNA_LIM_VOLT_MAX_MIN_RAW * DYNA_LIM_VOLT_MAX_RES_V)
                                              ///< min maximum volts limit
#define DYNA_LIM_VOLT_MAX_MAX_V \
  (DYNA_LIM_VOLT_MAX_MAX_V * DYNA_LIM_VOLT_MAX_RES_V)
                                              ///< max maximum volts limit
#define DYNA_LIM_VOLT_MAX_DFT_V \
  (DYNA_LIM_VOLT_MAX_DFT_V * DYNA_LIM_VOLT_MAX_RES_V)
                                              ///< default maximum volts limit
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_torque_max_on \
 *                                EEPROM On Power-Up Maximum Torque Limit
 *
 * Maximum torque in units of is about 0.1%. 
 *
 * \sa \ref dyna_servo_torque for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB   14      ///< maximum torque lsb (RW)
#define DYNA_ADDR_LIM_TORQUE_MAX_ON_MSB   15      ///< maximum torque msb (RW)

#define DYNA_ADDR_LIM_TORQUE_MAX_ON_MASK  0x03ff  ///< valid data mask

#define DYNA_LIM_TORQUE_MAX_ON_DFT_RAW    DYNA_TORQUE_MAX_RAW
                                                ///< default maximum raw torque
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_srl EEPROM Status Return Level
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_SRL             16    ///< status return level (RW)

#define DYNA_ADDR_SRL_MASK        0xff  ///< valid data mask

#define DYNA_SRL_RET_NONE         0     ///< no return packet (except for PING)
#define DYNA_SRL_RET_READ         1     ///< only READ return status packet
#define DYNA_SRL_RET_ALL          2     ///< return status packet for all 
#define DYNA_SRL_RET_DFT          DYNA_SRL_RET_ALL
                                        ///< return status packet default
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_alarm_led EEPROM Alarm LED
 *
 * LED on alarms mask.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_ALARM_LED         17    ///< alarm LED (RW)

#define DYNA_ADDR_ALARM_LED_MASK    0xff  ///< valid data mask

#define DYNA_ALARM_LED_DFT          DYNA_ALARM_DFT ///< alarm LED default mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_alarm_shutdown EEPROM Alarm Shutdown
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_ALARM_SHUTDOWN        18    ///< alarm shutdown (RW)

#define DYNA_ADDR_ALARM_SHUTDOWN_MASK   0xff  ///< valid data mask

#define DYNA_ALARM_NONE		        0x00  ///< no alarms
#define DYNA_ALARM_VOLTAGE		    0x01  ///< operating voltage out of range
#define DYNA_ALARM_ANGLE		      0x02  ///< goal position limit out of range
#define DYNA_ALARM_TEMP   		    0x04  ///< operating temperature out of range
#define DYNA_ALARM_CMD_RANGE      0x08  ///< command out of range
#define DYNA_ALARM_CHECKSUM		    0x10  ///< checksum error
#define DYNA_ALARM_LOAD		        0x20  ///< operating load (torque) out of rnge
#define DYNA_ALARM_INSTRUCTION	  0x40  ///< bad instruction

#define DYNA_ALARM_DFT (DYNA_ALARM_LOAD | DYNA_ALARM_TEMP)
                                        ///< alarm shutdown default
/*! \} */


// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .
// RAM. All values are read/write (RW) unless otherwise noted.
// .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .   .

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_torque_en RAM Torque Enable
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_TORQUE_EN       24    ///< torque enable (RW)

#define DYNA_ADDR_TORQUE_EN_MASK  0xff  ///< valid data mask

#define DYNA_TORQUE_EN_OFF        0     ///< disable power drive to the motor 
#define DYNA_TORQUE_EN_ON         1     ///< enable power drive to the motor 
#define DYNA_TORQUE_EN_DFT        DYNA_TORQUE_EN_OFF  // default torque enable
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_led RAM LED On/Off
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LED         25    ///< LED on/off (RW)

#define DYNA_ADDR_LED_MASK    0xff  ///< valid data mask

#define DYNA_LED_OFF          0     ///< LED off
#define DYNA_LED_ON           1     ///< LED on
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_ctl_method RAM Position Control Method
 *
 * RAM addresses 26(0x1a) - 29(0x1d) are overlaid with two different position
 * control methods. Which method used is dependent on the servo model.
 *
 * The compliance control method specifies a simple torque (speed) ramp down
 * and margin to achieve the goal position. It is used by many of the existing
 * models.
 *
 * The PID control method, that replaced the above method in newer servos,
 * defines the set of constants used by the servo to achieve the goal
 * position via a classic Proportional-Integral-Derivative controller.
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_ctl_method
 * \defgroup dyna_memmap_gen_compliance RAM Compliance Position Control Method
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_compliance
 * \defgroup dyna_memmap_gen_cw_comp_margin RAM Clockwise Compliance Margin
 *
 * Sets the allowed clockwise compliance margin between goal position and
 * current position.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_CW_COMP_MARGIN  26    ///< clockwise compliance margin (RW)

#define DYNA_ADDR_CW_COMP_MASK    0xff  ///< valid data mask

#define DYNA_COMP_MARGIN_MIN_RAW  0     ///< min position difference (tightest)
#define DYNA_COMP_MARGIN_MAX_RAW  255   ///< max position difference (loosest)
#define DYNA_COMP_MARGIN_DFT_RAW  1     ///< default raw position difference
#define DYNA_COMP_MARGIN_RES_DEG  DYNA_POS_RES_DEG
                                        ///< margin resolution
#define DYNA_COMP_MARGIN_MIN_DEG \
  (DYNA_COMP_MARGIN_MIN_RAW * DYNA_COMP_MARGIN_RES_DEG)
                                        ///< minimum margin in degress
#define DYNA_COMP_MARGIN_MAX_DEG \
  (DYNA_COMP_MARGIN_MAX_RAW * DYNA_COMP_MARGIN_RES_DEG)
                                        ///< maximum margin in degress
#define DYNA_COMP_MARGIN_DFT_DEG \
  (DYNA_COMP_MARGIN_DFT_RAW * DYNA_COMP_MARGIN_RES_DEG)
                                        ///< default margin in degress
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_compliance
 * \defgroup dyna_memmap_gen_ccw_comp_margin \
 *                            RAM Counterclockwise Compliance Margin
 *
 * Sets the allowed counterclockwise compliance margin between goal position and
 * current position.
 *
 * \sa \ref dyna_memmap_gen_cw_comp_margin for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_CCW_COMP_MARGIN 27    ///< counterclockwise compliance margin

#define DYNA_ADDR_CCW_COMP_MASK   0xff  ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_compliance
 * \defgroup dyna_memmap_gen_cw_comp_slope RAM Clockwise Compliance Slope
 *
 * Sets the level of clockwise torque near the goal position. Higher values
 * have more variance for stopping at goal postition.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_CW_COMP_SLOPE       28    ///< clockwise compliance slope

#define DYNA_ADDR_CW_COMP_SLOPE_MASK  0xff  ///< valid data mask

#define DYNA_COMP_SLOPE_TORQUE_1      0x02  ///< compliance torque level 1
#define DYNA_COMP_SLOPE_TORQUE_2      0x04  ///< compliance torque level 2
#define DYNA_COMP_SLOPE_TORQUE_3      0x08  ///< compliance torque level 3
#define DYNA_COMP_SLOPE_TORQUE_4      0x10  ///< compliance torque level 4
#define DYNA_COMP_SLOPE_TORQUE_5      0x20  ///< compliance torque level 5
#define DYNA_COMP_SLOPE_TORQUE_6      0x40  ///< compliance torque level 6
#define DYNA_COMP_SLOPE_TORQUE_7      0x80  ///< compliance torque level 7

#define DYNA_COMP_SLOPE_TORQUE_DFT    DYNA_COMP_SLOPE_TORQUE_5
                                            ///< default compliance torque level
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_compliance
 * \defgroup dyna_memmap_gen_ccw_comp_slope \
 *                                  RAM Counterclockwise Compliance Slope
 *
 * Sets the level of counterclockwise torque near the goal position. Higher
 * values have more variance for stopping at goal postition.
 *
 * \sa \ref dyna_memmap_gen_cw_comp_slope for valid values.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_CCW_COMP_SLOPE      29 ///< counterclockwise compliance slope

#define DYNA_ADDR_CCW_COMP_SLOPE_MASK 0xff  ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_ctl_method
 * \defgroup dyna_memmap_gen_pid RAM PID Position Control Method
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_pid
 * \defgroup dyna_memmap_gen_p_gain RAM PID P Gain
 *
 * K<sub>p</sub> = P<sub>gain</sub> / 8
 *
 * PID proportional term.
 *
 * The new MX series Dynamixels use a PID control method.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_P_GAIN          26      ///< P<sub>gain</sub> (RW)

#define DYNA_ADDR_P_GAIN_MASK     0xff    ///< valid data mask

#define DYNA_P_GAIN_MIN_RAW       0       ///< minimum value
#define DYNA_P_GAIN_MAX_RAW       254     ///< maximum value
#define DYNA_P_GAIN_DFT           32      ///< default value
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_pid
 * \defgroup dyna_memmap_gen_i_gain RAM PID I Gain
 *
 * K<sub>i</sub> = I<sub>gain</sub> * 1000 / 2048
 *
 * PID integral term.
 *
 * The new MX series Dynamixels use a PID control method.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_I_GAIN          27      ///< I<sub>gain</sub> (RW)

#define DYNA_ADDR_I_GAIN_MASK     0xff    ///< valid data mask

#define DYNA_I_GAIN_MIN_RAW       0       ///< minimum value
#define DYNA_I_GAIN_MAX_RAW       254     ///< maximum value
#define DYNA_I_GAIN_DFT           0       ///< default value
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_pid
 * \defgroup dyna_memmap_gen_d_gain RAM PID D Gain
 *
 * K<sub>d</sub> = D<sub>gain</sub> * 4 / 1000
 *
 * PID derivative term.
 *
 * The new MX series Dynamixels use a PID control method.
 *
 * \par Mode: RW
 * \{
 */
#define DYNA_ADDR_D_GAIN          28      ///< D<sub>gain</sub> (RW)

#define DYNA_ADDR_D_GAIN_MASK     0xff    ///< valid data mask

#define DYNA_D_GAIN_MIN_RAW       0       ///< minimum value
#define DYNA_D_GAIN_MAX_RAW       254     ///< maximum value
#define DYNA_D_GAIN_DFT           0       ///< default value
/*! \} */

/*!
 * \ingroup dyna_memmap_gen_pid
 * \defgroup dyna_memmap_gen_reserved RAM Reserved
 * \{
 */
#define DYNA_ADDR_PID_RESERVED    29      ///< reserved 
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_goal_pos RAM Goal Position
 *
 * Move to new goal position. The servo will move at the current goal speed
 * under the current torque limit. These fields are ignored in continuous mode.
 *
 * \sa \ref dyna_servo_pos for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_GOAL_POS_LSB    30      ///< goal position lsb (RW)
#define DYNA_ADDR_GOAL_POS_MSB    31      ///< goal position msb (RW)

#define DYNA_ADDR_GOAL_POS_MASK   0x03ff ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_goal_speed RAM Goal Speed and Direction
 *
 * In servo mode, move to new goal position at the given speed. In continuous
 * mode 
 * under the current torque limit. These field are ignored in continuous mode.
 *
 * \par Servo Mode Fields:
 * \termblock
 * \term speed \termdata 10-bit raw speed. \endterm
 * \endtermblock

 * \par Continuous Mode Fields:
 * \termblock
 * \term speed \termdata 10-bit raw speed. \endterm
 * \term dir \termdata clockwise or countclockwise direction \endterm
 * \endtermblock
 *
 * \sa \ref dyna_servo_speed and \ref dyna_servo_dir for valid values.

 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_GOAL_SPEED_LSB  32      ///< goal speed lsb (RW)
#define DYNA_ADDR_GOAL_SPEED_MSB  33      ///< goal speed msb (RW)

#define DYNA_ADDR_GOAL_SPEED_MASK 0x07ff  ///< valid data mask

#define DYNA_GOAL_SPEED_MAG_MASK  0x03ff  ///< speed magnitude field mask
#define DYNA_GOAL_SPEED_MAG_SHIFT 0       ///< speed magnitude field shift

#define DYNA_GOAL_SPEED_DIR_MASK  0x0400  ///< speed direction field mask
#define DYNA_GOAL_SPEED_DIR_SHIFT 10      ///< speed direction field shift
#define DYNA_GOAL_SPEED_DIR_CCW   0x00    ///< counterclockwise direction
#define DYNA_GOAL_SPEED_DIR_CW    (0x01 << DYNA_CUR_SPEED_DIR_SHIFT)
                                          ///< clockwise direction
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lim_torque_max RAM Maximum Torque Limit
 *
 * \sa \ref dyna_servo_torque for valid values.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_LIM_TORQUE_MAX_LSB  34      ///< current torque limit lsb (RW)
#define DYNA_ADDR_LIM_TORQUE_MAX_MSB  35      ///< current torque limit msb (RW)

#define DYNA_ADDR_LIM_TORQUE_MAX_MASK 0x03ff  ///< valid data mask

#define DYNA_LIM_TORQUE_MAX_ALARMED   0     ///< torque value on alarm shutdown
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_cur_pos RAM Current Position
 *
 * Current servo position. These fields are invalid in continuous mode.
 *
 * \sa \ref dyna_servo_pos for valid values.
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_CUR_POS_LSB     36      ///< current position lsb (R)
#define DYNA_ADDR_CUR_POS_MSB     37      ///< current position msb (R)

#define DYNA_ADDR_CUR_POS_MASK  0x03ff    ///< valid data mask
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_cur_speed RAM Current Speed and Direction
 *
 *  Get the current servo speed and direction.
 *
 * \par Fields:
 * \termblock
 * \term speed \termdata 10-bit raw speed. \endterm
 * \term dir \termdata clockwise or countclockwise direction \endterm
 * \endtermblock
 *
 * \sa \ref dyna_servo_pos and \ref dyna_servo_dir for valid values.

 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_CUR_SPEED_LSB   38      ///< current speed lsb (R)
#define DYNA_ADDR_CUR_SPEED_MSB   39      ///< current speed msb (R)

#define DYNA_ADDR_CUR_SPEED_MASK  0x07ff  ///< valid data mask

#define DYNA_CUR_SPEED_MAG_MASK   0x03ff  ///< speed magnitude field mask
#define DYNA_CUR_SPEED_MAG_SHIFT  0       ///< speed magnitude field shift

#define DYNA_CUR_SPEED_DIR_MASK   0x0400  ///< speed direction field mask
#define DYNA_CUR_SPEED_DIR_SHIFT  10      ///< speed direction field shift
#define DYNA_CUR_SPEED_DIR_CCW    0x00    ///< counterclockwise direction
#define DYNA_CUR_SPEED_DIR_CW     (0x01 << DYNA_CUR_SPEED_DIR_SHIFT)
                                          ///< clockwise direction
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_cur_load RAM Current Load
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_CUR_LOAD_LSB    40      ///< current load lsb (R)
#define DYNA_ADDR_CUR_LOAD_MSB    41      ///< current load msb (R)

#define DYNA_ADDR_CUR_LOAD_MASK   0x07ff  ///< valid data mask

#define DYNA_CUR_LOAD_MAG_MASK    0x03ff  ///< load magnitude field mask
#define DYNA_CUR_LOAD_MAG_SHIFT   0       ///< load magnitude field shift
#define DYNA_CUR_LOAD_MAG_MIN     0       ///< minimum load
#define DYNA_CUR_LOAD_MAG_MAX     1023    ///< maximum load

#define DYNA_CUR_LOAD_DIR_MASK    0x0400  ///< load direction field mask
#define DYNA_CUR_LOAD_DIR_SHIFT   10      ///< load direction field shift
#define DYNA_CUR_LOAD_DIR_CCW     0x00    ///< counterclockwise direction
#define DYNA_CUR_LOAD_DIR_CW      (0x01 << DYNA_CUR_LOAD_DIR_SHIFT)
                                          ///< clockwise direction
#define DYNA_CUR_LOAD_RES_PCT     (100.0/DYNA_CUR_LOAD_MAG_MAX)
                                          ///< percent of maximum resolution
#define DYNA_CUR_LOAD_RES_PMIL    (1000.0/DYNA_CUR_LOAD_MAG_MAX)
                                          ///< permil of maximum resolution
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_cur_volt RAM Current Voltage
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_CUR_VOLT        42    ///< current voltage (R)

#define DYNA_ADDR_CUR_VOLT_MASK   0xff  ///< valid data mask

#define DYNA_CUR_VOLT_RES_V       DYNA_VOLT_RES_V ///< 0.1V precesion
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_cur_temp RAM Current Temperature
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_CUR_TEMP_C      43    ///< current temperature C (R)

#define DYNA_ADDR_CUR_TEMP_MASK   0xff  ///< valid data mask

#define DYNA_CUR_TEMP_RES_C       DYNA_TEMP_RES_C
                                        ///< 1\h_deg Celsius resolution
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_reg_instr RAM Register Instruction
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_REG_INSTR       44    ///< registered instruction (RW)

#define DYNA_ADDR_REG_INSTR_MASK  0xff  ///< valid data mask

#define DYBA_REG_INSTR_NO_WRITE   0     ///< no (pending?) REG_WRITE commands
#define DYBA_REG_INSTR_HAS_WRITE  1     ///< has (pending?) REG_WRITE commands
/*! \} */

#define DYNA_ADDR_RESERVED_2      45 ///< reserved 

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_is_moving RAM Servo Is Moving
 *
 * \par Mode: R
 *
 * \{
 */
#define DYNA_ADDR_IS_MOVING       46    ///< is [not] moving (to goal position)

#define DYNA_ADDR_IS_MOVING_MASK  0xff  ///< valid data mask

#define DYNA_IS_NOT_MOVING        0     ///< not moving
#define DYNA_IS_MOVING            1     ///< is moving
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_lock RAM Lock EEPROM
 *
 * Requires a power reset to force the bit from 1 back to 0.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_EEPROM_LOCK       47    ///< lock the EEPROM (RW)

#define DYNA_ADDR_EEPROM_LOCK_MASK  0xff  ///< valid data mask

#define DYNA_EEPROM_UNLOCKED        0     ///< eeprom is unlocked 
#define DYNA_EEPROM_LOCKED          1     ///< eeprom is locked
#define DYNA_EEPROM_LOCK_DFT        0     ///< eeprom lock default
/*! \} */

/*!
 * \ingroup dyna_memmap_gen
 * \defgroup dyna_memmap_gen_punch RAM Punch
 *
 * Not used.
 *
 * \par Mode: RW
 *
 * \{
 */
#define DYNA_ADDR_PUNCH_LSB       48 ///< punch lsb (not used)
#define DYNA_ADDR_PUNCH_MSB       49 ///< punch msb (not used)
/*! \} */


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _DYNAMIXEL_H
