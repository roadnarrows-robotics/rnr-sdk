////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// File:      libCogniBoost.h
//
/*! \file
 *
 *  $LastChangedDate$
 *  $Rev$
 *
 * \brief CogniBoost Library Interface Declarations.
 *
 * The CogniBoost library libCogniBoost C implementation interface header file.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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
///////////////////////////////////////////////////////////////////////////////

#ifndef _LIBCOGNIBOOST_H
#define _LIBCOGNIBOOST_H

#include <limits.h>
#include <stdio.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/CogniBoostMsgs.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

// ---------------------------------------------------------------------------
// CogniBoost Library libCogniBoost Defines, Types, and Structures
// ---------------------------------------------------------------------------

// ...........................................................................
// Host Library Error Codes
// ...........................................................................

/*!
 * \addtogroup man_libcogniboost_app_ecodes
 *
 * The host library error codes are an extension to the CogniBoost
 * core firmware response error codes. This extended set of error codes are
 * used extensively in libCogniBoost.
 *
 * \note
 * The error codes are divided into two sets:
 * \li CogniBoost firmware response error codes.
 * \li Host library only error codes.
 *
 * There are unused, but reserved error codes in both sets.
 * <b>Do not use.</b>
 * Add user specific application errors after \ref CB_LIB_ECODE_LAST_RSV.
 *
 * \sa cb_syntax_ecodes
 * \{
 */

/*!
 * \brief Convenience macro to make the nth library error code number.
 * \param n   Nth error code.
 */
#define CB_LIB_ECODE(n)         (CB_RSP_ECODE_LAST_RSV+1+(n))

// The error codes
#define CB_LIB_ECODE_GEN        CB_LIB_ECODE(0)   ///< general, unspecified err
#define CB_LIB_ECODE_SYS        CB_LIB_ECODE(1)   ///< system call failed
#define CB_LIB_ECODE_RSP_BADEC  CB_LIB_ECODE(2)   ///< bad response error code
#define CB_LIB_ECODE_NOTSUP     CB_LIB_ECODE(3)   ///< not supported
#define CB_LIB_ECODE_NO_DEV     CB_LIB_ECODE(4)   ///< no device
#define CB_LIB_ECODE_TIMEDOUT   CB_LIB_ECODE(5)   ///< operation timed out
#define CB_LIB_ECODE_NO_RSRC    CB_LIB_ECODE(6)   ///< no resource
#define CB_LIB_ECODE_BUSY       CB_LIB_ECODE(7)   ///< resource busy
#define CB_LIB_ECODE_EXEC       CB_LIB_ECODE(8)   ///< cannot execute error
#define CB_LIB_ECODE_IO         CB_LIB_ECODE(9)   ///< bad I/O
#define CB_LIB_ECODE_READ       CB_LIB_ECODE(10)  ///< bad read
#define CB_LIB_ECODE_WRITE      CB_LIB_ECODE(11)  ///< bad write
#define CB_LIB_ECODE_SYNC       CB_LIB_ECODE(12)  ///< bad sync
#define CB_LIB_ECODE_RECV       CB_LIB_ECODE(13)  ///< bad receive
#define CB_LIB_ECODE_SEND       CB_LIB_ECODE(14)  ///< bad send
#define CB_LIB_ECODE_BAD_HDR    CB_LIB_ECODE(15)  ///< bad packet header
#define CB_LIB_ECODE_BAD_MSG    CB_LIB_ECODE(16)  ///< bad message
#define CB_LIB_ECODE_BAD_RSP    CB_LIB_ECODE(17)  ///< bad or partial response
#define CB_LIB_ECODE_BAD_CHKSUM CB_LIB_ECODE(18)  ///< bad checksum
#define CB_LIB_ECODE_INVAL      CB_LIB_ECODE(19)  ///< invalid value
#define CB_LIB_ECODE_TOO_SMALL  CB_LIB_ECODE(20)  ///< value too small
#define CB_LIB_ECODE_TOO_BIG    CB_LIB_ECODE(21)  ///< value too big
#define CB_LIB_ECODE_RANGE      CB_LIB_ECODE(22)  ///< value out of range
#define CB_LIB_ECODE_BUG        CB_LIB_ECODe(23)  ///< software bug
#define CB_LIB_ECODE_BADEC      CB_LIB_ECODE(24)  ///< bad error code
#define CB_LIB_ECODE_NUMOF      25                ///< number of error codes

#define CB_ECODE_TOT_NUMOF      (CB_RSP_ECODE_NUMOF + CB_LIB_ECODE_NUMOF)
                                              ///< total number of error codes

/*! Error codes through 63 are reserved for future CogniBoost library use. */
#define CB_LIB_ECODE_LAST_RSV   63
/*! \} */

/*!
 * \brief Log CogniBoost error convenience macro.
 *
 * \param hnd    CogniBoost device handle.
 * \param ecode  CogniBoost error code.
 * \param efmt   Error output formatted string literal.
 * \param ...    Error variable arguments.
 */
#define CB_LOG_ERROR(hnd, ecode, efmt, ...) \
  LOGERROR("%s(): Device %d: %s(ecode=%d): " efmt, \
    LOGFUNCNAME, cbAttrGetFd((hnd)), \
    cbStrError(ecode), ((ecode) >=0 ? (ecode) : -(ecode)), \
    ##__VA_ARGS__)

/*!
 * \brief Log system error convenience macro.
 *
 * \param hnd    CogniBoost device handle.
 * \param ecode  CogniBoost error code.
 * \param efmt   Error output format string literal.
 * \param ...    Error variable arguments.    
 */
#define CB_LOG_SYSERROR(hnd, ecode, efmt, ...) \
  LOGSYSERROR("%s(): Device %d: %s(ecode=%d): " efmt, \
    LOGFUNCNAME, cbAttrGetFd((hnd)), \
    cbStrError(ecode), ((ecode)>=0? (ecode): -(ecode)), \
    ##__VA_ARGS__)


// ...........................................................................
// Library Tuneables
// ...........................................................................

/*
 * Compiled Default Tunables.
 *
 * All times are in microseconds.
 *
 * Note:
 * At 115,200 baud rate, maximum through put is approximately 70usec/byte
 * or approximately 18ms per 256 bytes.
 */
#ifndef CB_TUNE_T_READ
#define CB_TUNE_T_READ            2000000   ///< 2 second read timeout default
#endif

#ifndef CB_TUNE_T_WRITE
#define CB_TUNE_T_WRITE           2000000   ///< 2 second write timeout default
#endif

#ifndef CB_TUNE_T_CONN_PROXIED
#define CB_TUNE_T_CONN_PROXIED    30000000  ///< 30s proxied connection time out
#endif

// ...........................................................................
// Library Data Types
// ...........................................................................

/*!
 * CogniBoost Device Handle Type (pointer to opaque structure)
 */
typedef struct _cbDeviceStruct  *cbHnd_T;

#define CB_HND_NONE     (NULL)    ///< no CogniBoost handle

/*!
 * \brief Hardware/Firmware/Software Version Structure
 */
typedef struct
{
  uint_t  m_uVerMajor;      ///< major version number
  uint_t  m_uVerMinor;      ///< minor version number
  uint_t  m_uRevision;      ///< revision number
} cbVer_T;

/*!
 * CogniBoost Identities and Version Numbers Structure
 */
typedef struct
{
  char    m_sMfgName[CB_ID_STR_LEN_MAX+1];  ///< manufactures name string
  char    m_sProdName[CB_ID_STR_LEN_MAX+1]; ///< product name string
  char    m_sHwSN[CB_ID_STR_LEN_MAX+1];     ///< hardware serial number string
  cbVer_T m_verHw;                          ///< hardware dotted version
  char    m_sFwApp[CB_ID_STR_LEN_MAX+1];    ///< firmware application string
  cbVer_T m_verFw;                          ///< firmware dotted version
  char    m_sFwDate[CB_ID_STR_LEN_MAX+1];   ///< firmware build date string
  cbVer_T m_verLib;                         ///< library dotted version
} cbIdentities_T;

/*!
 * \brief CogniBoost CogniMem Train/Match Pattern Structure
 */
typedef struct
{
  uint_t  m_uCount;                           ///< number of bytes in pattern
  byte_t  m_byPatBuf[CM_PATTERN_COMP_NUMOF];  ///< byte pattern buffer
} cbNNPattern_T;

/*!
 * \brief CogniBoost CogniMem Neural Network Set Structure
 */
typedef struct
{
  uint_t          m_uNumNeurons;                    ///< number of neurons
  cbNeuron_T      m_stNeurons[CM_NEURON_NUMOF];     ///< the neurons
} cbNNSet_T;

/*!
 * CogniBoost Client - Server Connection Type
 */
typedef enum
{
  cbConnTypeNone,       ///< no connection
  cbConnTypeDirect,     ///< direct serial USB connection
  cbConnTypeProxy       ///< botsense IP proxied connecton
} cbConnType_T;


// ---------------------------------------------------------------------------
// Utility Functions and Macros
// ---------------------------------------------------------------------------

/*!
 * \brief Convert CogniBoost discrete dotted version numbers to the standard
 * comparable value.
 *
 * \par Example:
 * <tt>if( CB_VERSION(<em>major</em>, <em>minor</em>, <em>rev</em>) )
 *          \>= CB_VERSION(1, 3, 0) { ... }
 * </tt>
 *
 * \param major 8-bit major version number.
 * \param minor 8-bit minor version number.
 * \param rev   8-bit minor revision number.
 */
#define CB_VERSION(major, minor, rev) \
  (uint_t)((((major) & 0xff) << 16) | (((minor) & 0xff) << 8) | ((rev) & 0xff))

/*!
 * \brief Convert CogniBoost cbVer_T instance to the standard comparable value.
 *
 * \par Example:
 * <tt>cvVer_T ver = {1, 1, 3};
 * if( CB_VERSION_V(ver) \>= CB_VERSION(1, 3, 0) { ... }
 * </tt>
 *
 * \param v     A cbVer_T instance.
 */
#define CB_VERSION_V(v) \
  (uint_t)( ((((v).m_uVerMajor) & 0xff) << 16) | \
            ((((v).m_uVerMinor) & 0xff) << 8) | \
            ((v).m_uRevision & 0xff))

/*!
 * \brief Convert CogniBoost dotted version string to discrete version numbers.
 *
 * \param verstr        Version string of form "M.m.rev".
 * \param [out] major   Major version number.
 * \param [out] minor   Minor version number.
 * \param [out] rev     Revision number.
 */
#define CB_VERSION_STOI(verstr, major, minor, rev) \
  sscanf(verstr, "%u.%u.%u", &(major), &(minor), &(rev))

/*!
 * \brief Convert CogniBoost dotted version string to cvVer_T version numbers.
 *
 * \param verstr        Version string of form "M.m.rev".
 * \param [out] v       A cbVer_T instance.
 */
#define CB_VERSION_STOI_V(verstr, v) \
  sscanf(verstr, "%u.%u.%u", \
      &((v).m_uVerMajor), &((v).m_uVerMinor), &((v).m_uRevision))

extern const char *cbStrError(int ecode);


// ---------------------------------------------------------------------------
// CogniBoost Connection Attribute Functions
// ---------------------------------------------------------------------------

// e.g. /dev/ttyUSB0 /dev/ttyUSB1@192.168.2.1:9194
extern const char *cbAttrGetDeviceName(cbHnd_T hnd);

extern int cbAttrGetBaudRate(cbHnd_T hnd);

extern cbConnType_T cbAttrGetConnType(cbHnd_T hnd);

extern int cbAttrGetFd(cbHnd_T hnd);

extern bool_t cbAttrGetTraceState(cbHnd_T hnd);

extern int cbAttrSetTraceState(cbHnd_T hnd, bool_t bEnable);

extern int cbAttrSetLogging(cbHnd_T hnd, int level);


// ---------------------------------------------------------------------------
// CogniBoost Connection, Identity, Test, and Control Functions
// ---------------------------------------------------------------------------

extern cbHnd_T cbConnect(const char *sDeviceName,
                         int         nBaudRate,
                         bool_t      bTrace);

extern int cbDisconnect(cbHnd_T hnd);

// add library version
extern int cbGetIdentities(cbHnd_T hnd, cbIdentities_T *pIds);

// return msec or error
extern int cbPing(cbHnd_T hnd);

// [in,out] buf 
extern int cbLoopback(cbHnd_T hnd, byte_t buf[], size_t uCount);

// connect is lost? if so, then reconnect. no bootloader option in this i/f
extern int cbReboot(cbHnd_T hnd);

extern int cbSleep(cbHnd_T hnd);

extern int cbWakeUp(cbHnd_T hnd);

// reconnect at new baudrate
extern int cbParamSetBaudRate(cbHnd_T hnd, uint_t uBaudRate);
extern int cbParamGetBaudRate(cbHnd_T hnd, uint_t *pBaudRate);

extern int cbParamSetAutoRestore(cbHnd_T hnd, bool_t bEnable);
extern int cbParamGetAutoRestore(cbHnd_T hnd, bool_t *pEnable);

extern int cbParamSetAutoSleep(cbHnd_T hnd, uint_t uMsec);
extern int cbParamGetAutoSleep(cbHnd_T hnd, uint_t *pMsec);

extern int cbParamSetLedBling(cbHnd_T hnd, cbLedBlingParams_T *pLedBling);
extern int cbParamGetLedBling(cbHnd_T hnd, cbLedBlingParams_T *pLedBling);



extern int cbParamSetCMClassifier(cbHnd_T hnd, uint_t uClassifier);
extern int cbParamGetCMClassifier(cbHnd_T hnd, uint_t *pClassifier);

extern int cbParamSetCMContext(cbHnd_T hnd, uint_t uContext);
extern int cbParamGetCMContext(cbHnd_T hnd, uint_t *pContext);

extern int cbParamSetCMNorm(cbHnd_T hnd, uint_t uNorm);
extern int cbParamGetCMNorm(cbHnd_T hnd, uint_t *pNorm);

extern int cbParamSetCMMinIF(cbHnd_T hnd, uint_t uMinIF);
extern int cbParamGetCMMinIF(cbHnd_T hnd, uint_t *pMinIF);

extern int cbParamSetCMMaxIF(cbHnd_T hnd, uint_t uMaxIF);
extern int cbParamGetCMMaxIF(cbHnd_T hnd, uint_t *pMaxIF);

extern int cbParamSetCMMaxClassified(cbHnd_T hnd, uint_t uMaxClassified);
extern int cbParamGetCMMaxClassified(cbHnd_T hnd, uint_t *pMaxClassified);

extern int cbParamSetNNLabel(cbHnd_T hnd, char *sLabel);
extern int cbParamGetNNLabel(cbHnd_T hnd, char buf[], size_t size);

extern int cbParamSetAll(cbHnd_T hnd, cbCogniBoostParams_T *pParams);
extern int cbParamGetAll(cbHnd_T hnd, cbCogniBoostParams_T *pParams);

extern int cbNVMemReset(cbHnd_T hnd);
extern int cbNVMemSave(cbHnd_T hnd);
// may need to reconnect if baudrate differs
extern int cbNVMemRestore(cbHnd_T hnd);
// returns >=0, or -error
extern int cbNVMemGetNNCount(cbHnd_T hnd);

// returns nid or -error
extern int cbNNTrain(cbHnd_T hnd, uint_t uCategory, cbNNPattern_T *pPattern);
extern int cbNNCategorize(cbHnd_T hnd, cbNNPattern_T *pPattern,
    cbClassification_T classification[], size_t size);
extern int cbNNForget(cbHnd_T hnd);
// returns count or -error
extern int cbNNGetCount(cbHnd_T hnd);

extern int cbNNUploadTrainingSet(cbHnd_T hnd, cbNNSet_T *pNNSet);
extern int cbNNDownloadTrainingSet(cbHnd_T hnd, cbNNSet_T *pNNSet);

extern int cbCMReadReg(cbHnd_T hnd, byte_t byAddr, ushort_t *pVal);
extern int cbCMWriteReg(cbHnd_T hnd, byte_t byAddr, ushort_t huVal);


#ifndef SWIG
C_DECLS_END
#endif


#endif // _LIBCOGNIBOOST_H
