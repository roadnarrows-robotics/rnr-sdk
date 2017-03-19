////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      dynaLibInternal.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief The libDynamixel internal declarations.
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

#ifndef _DYNA_LIB_INTERNAL_H
#define _DYNA_LIB_INTERNAL_H

#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"


// ---------------------------------------------------------------------------
// Internal Defines, Macros, Types and Structures
// ---------------------------------------------------------------------------

/*!
 * \brief Log Error.
 *
 * \param ecode     \ref dyna_ecodes error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define DYNA_LOG_ERROR(ecode, efmt, ...) \
  LOGERROR("%s(ecode=%d): " efmt, \
      DynaStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log System Error.
 *
 * \param ecode     \ref dyna_ecodes error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define DYNA_LOG_SYS_ERROR(ecode, efmt, ...) \
  LOGSYSERROR("%s(ecode=%d): " efmt, \
      DynaStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Error.
 *
 * \param estat     Errored communication status code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define DYNA_LOG_COMM_ERROR(estat, efmt, ...) \
  LOGERROR("%s(ecode=%d): %s(%d): " efmt, \
      DynaStrError(DYNA_ECODE_ECOMM), DYNA_ECODE_ECOMM, \
      dynaCommStatString(estat), estat, \
      ##__VA_ARGS__)

/*!
 * \brief Log buffer of infrared readings.
 *
 * \param sPreface  Preface string.
 * \param buf       Buffer contents to log.
 * \param uCount    Number of entries to log.
 * \param sFmt      Buffer entry format string.
 */
#ifdef LOG
#define DYNA_LOG_BUF(sPreface, buf, uCount, sFmt) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG3) ) \
    { \
      DynaLogBuf(sPreface, buf, uCount, sFmt); \
    } \
  } while(0)
#else
#define DYNA_LOG_BUF(sPreface, buf, uCount, sFmt)
#endif // LOG

/*!
 * \brief Log servo alarms.
 *
 * \param id      Servo id.
 * \param alarms  Alarm bits fields.
 */
#ifdef LOG
#define DYNA_LOG_SERVO_ALARMS(id, alarms) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG3) && ((alarms) != DYNA_ALARM_NONE) ) \
    { \
      DynaLogServoAlarms(id, alarms); \
    } \
  } while(0)
#else
#define DYNA_LOG_SERVO_ALARMS(id, alarms)
#endif // LOG

/*!
 * \brief Test if Dynamixel return code is not an error.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] comm   Dynamixel bus communication object.
 */
#define DYNA_TRY_RC(rc, fmt, ...) \
  do \
  { \
    if( (rc) < 0 ) \
    { \
      DYNA_LOG_ERROR(rc, fmt, ##__VA_ARGS__); \
      return rc; \
    } \
  } while(0)

/*!
 * \brief Test if bus communication is available exception macro.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] comm   Dynamixel bus communication object.
 */
#define DYNA_TRY_COMM(comm) \
  do \
  { \
    if( !(comm).IsOpen() ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_BADF, "Communication not open."); \
      return -DYNA_ECODE_BADF; \
    } \
  } while(0)

/*!
 * \brief Testing if the servo id is in range exception macro.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] id     Servo id.
 */
#define DYNA_TRY_SERVO_ID(id) \
  do \
  { \
    if( ((int)(id) < DYNA_ID_MIN) || ((int)(id) > DYNA_ID_MAX) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, "Servo id %d: Out-of-range.", \
          (int)(id)); \
      return -DYNA_ECODE_BAD_VAL; \
    } \
  } while(0)

/*!
 * \brief Test if the servo is a master servo.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 */
#define DYNA_TRY_IS_MASTER(pservo) \
  do \
  { \
    if( !((pservo)->IsMaster()) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_SLAVE, \
          "Servo %d: Not a master servo.", \
          (pservo)->m_nServoId); \
      return -DYNA_ECODE_SLAVE; \
    } \
  } while(0)

/*!
 * \brief Test if the servo is unlinked.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 */
#define DYNA_TRY_IS_UNLINKED(pservo) \
  do \
  { \
    if( !((pservo)->IsUnlinked()) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_LINKED, \
          "Servo %d: Operation not supported: Linked with servo %d.", \
          (pservo)->m_nServoId, (pservo)->m_link.m_pServoMate->GetServoId()); \
      return -DYNA_ECODE_LINKED; \
    } \
  } while(0)

/*!
 * \brief Test if the servo has one of the required modes exception macro.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 * \param [in] mode     Required servo modes mask.
 */
#define DYNA_TRY_SERVO_HAS_MODE(pservo, mode) \
  do \
  { \
    if( !((pservo)->m_spec.m_uSupportedModes & (mode)) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_NOT_SUPP, \
          "Servo %d: Required %s mode not supported.",\
          (pservo)->m_nServoId, \
          (((mode)&DYNA_MODE_CONTINUOUS)? "continuous": "servo")); \
      return -DYNA_ECODE_NOT_SUPP; \
    } \
  } while(0)

/*!
 * \brief Test if the servo is in the given mode.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 * \param [in] mode     Required servo modes mask.
 */
#define DYNA_TRY_SERVO_IN_MODE(pservo, mode) \
  do \
  { \
    if( ((pservo)->m_cfg.m_uServoMode != (mode)) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_NOT_SUPP, \
          "Servo %d: Not in required %s mode.",\
          (pservo)->m_nServoId, \
          (((mode)&DYNA_MODE_CONTINUOUS)? "continuous": "servo")); \
      return -DYNA_ECODE_NOT_SUPP; \
    } \
  } while(0)

/*!
 * \brief Test if the servo has positiion control.
 *
 * A servo has position control if:
 * \li in servo mode (servo will translate to new position).
 * \li in continuous mode with 360\h_deg position data (host will translate to
 * new position).
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 */
#define DYNA_TRY_SERVO_HAS_POS_CTL(pservo) \
  do \
  { \
    if( ((pservo)->m_cfg.m_uServoMode != DYNA_MODE_SERVO) && \
        !(pservo)->m_spec.m_bHas360Pos ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_NOT_SUPP, \
          "Servo %d: Does not support position control.", \
          (pservo)->m_nServoId); \
      return -DYNA_ECODE_NOT_SUPP; \
    } \
  } while(0)

/*!
 * \brief Test if the servo has a proxy agent.
 *
 * The proxy agent is required to do position control for servos in continuous
 * mode, since the servos do not support this function.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pservo   Pointer to DynaServo derived object.
 */
#define DYNA_TRY_SERVO_HAS_AGENT(pservo) \
  do \
  { \
    if( (pservo)->m_pAgent == NULL ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_NOT_SUPP, \
          "Servo %d: No servo proxy agent.", \
          (pservo)->m_nServoId); \
      return -DYNA_ECODE_NOT_SUPP; \
    } \
  } while(0)

/*!
 * \brief Test if servo address is valid.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] addr   Servo control table address.
 */
#define DYNA_TRY_ADDR(addr) \
  do \
  { \
    if( !(  (((int)(addr) >= DYNA_EEPROM_MIN_ADDR) && \
             ((int)(addr) <= DYNA_EEPROM_MAX_ADDR)) || \
            (((int)(addr) >= DYNA_RAM_MIN_ADDR) && \
             ((int)(addr) <= DYNA_RAM_MAX_ADDR)) ) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, \
          "Control table address 0x%02x: Out of range.", (int)(addr)); \
      return -DYNA_ECODE_BAD_VAL; \
    } \
  } while(0)

/*!
 * \brief Test if the servo id is valid and present in chain exception macro.
 *
 * If the try fails, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param [in] pchain   Pointer to DynaChain object.
 * \param [in] id       Servo id.
 */
#define DYNA_TRY_SERVO_IN_CHAIN(pchain, id) \
  do \
  { \
    if( ((int)(id) < DYNA_ID_MIN) || ((int)(id) > DYNA_ID_MAX) ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, "Servo %d: Id out of range.", \
          (int)(id)); \
      return -DYNA_ECODE_BAD_VAL; \
    } \
    else if( (pchain)->m_pChain[(id)] == NULL ) \
    { \
      DYNA_LOG_ERROR(DYNA_ECODE_NO_SERVO, "Servor %d: No servo in chain.", \
          (int)(id)); \
      return -DYNA_ECODE_NO_SERVO; \
    } \
  } while(0)

/*!
 * \brief Test if the servo is in the required mode(s) exception macro.
 *
 * If the check is false, an appropriate error is logged and the calling 
 * function immediately returns with the appropriate \h_lt 0 error code.
 *
 * \param expr    Expression to evaluate
 * \param ecode   CogniBoost error code.
 * \param efmt    Error output format string literal.
 * \param ...     Variable arguments.
 */
#define DYNA_TRY_EXPR(expr, ecode, efmt, ...) \
do \
{ \
  if ( !(expr) ) \
  { \
    DYNA_LOG_ERROR(ecode, "%s: " efmt, #expr, ##__VA_ARGS__); \
    return (ecode > 0) ? -(ecode) : ecode; \
  } \
} while(0)


// ---------------------------------------------------------------------------
// Internal Data
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Internal Prototypes
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Allocate new duplicated string.
 *
 * \param s String to duplicate.
 *
 * \return Returns pointer to allocated string if s is not NULL and the
 * length of s \h_gt 0.\n Otherwise returns NULL.
 */
inline char *newstr(const char *s)
{
  char  *t;
  if( (s != NULL) && (*s != 0) )
  {
    t = new char[strlen(s)+1];
    strcpy(t, s);
  }
  else
  {
    t = NULL;
  }
  return t;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Error, Logging, and Debug Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Map DXL library error code to Dynamixel error code.
 *
 * \param nDxlError DXL error code.
 *
 * \return Dynamixel library error code.
 */
extern int DynaMapDxlToEcode(int nDxlError);


#endif // _DYNA_LIB_INTERNAL_H
