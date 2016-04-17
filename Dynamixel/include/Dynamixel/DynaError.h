////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaError.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Library Error and Logging Routines.
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

#ifndef _DYNA_ERROR_H
#define _DYNA_ERROR_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// Defines and Types
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

/*!
 * \brief Get the error string describing the Dynamixel error code.
 *
 * The absolute value of the error code is taken prior retrieving the string.
 * An unknown or out-of-range error code will be mapped to
 * \ref DYNA_ECODE_BADEC.
 *
 * \param ecode Dynamixel error code.
 *
 * \return Returns the appropriate error code string.
 */
extern const char *DynaStrError(int ecode);

/*!
 * \brief Pretty print a byte buffer to opened file stream.
 *
 * \param fp        File pointer.
 * \param sPreface  Optional buffer preface string (set to NULL for no preface).
 * \param buf       Buffer to print.
 * \param sFmt      Buffer entry format string.
 * \param uCount    Number of entries to print.
 * \param uNLFreq   Newline frequency (set to 0 for no newlines).
 * \param uCol      Column alignment number.
 */
extern void DynaPrintBuf(FILE       *fp,
                         const char *sPreface,
                         byte_t      buf[],
                         const char *sFmt,
                         size_t      uCount,
                         size_t      uNLFreq,
                         uint_t      uCol);

#ifdef LOG
/*!
 * \brief Log servo alarms.
 *
 * \param nServoId  Servo id.
 * \param uAlarms   Alarm bits fields.
 */
extern void DynaLogServoAlarms(int nServoId, uint_t uAlarms);

/*! 
 * \brief Log integer data.
 *
 * Print diagnostic logging of the contents of a buffer of bytes.
 *
 * \param sPreface  Buffer preface string.
 * \param buf       Buffer contents to log.
 * \param uCount    Number of entries to log.
 * \param sFmt      Buffer entry format string.
 */
extern void DynaLogBuf(const char *sPreface,
                       byte_t      buf[],
                       size_t      uCount,
                       const char *sFmt);
#endif // LOG

#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _DYNA_ERROR_H
