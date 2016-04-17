////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibConn.c
//
/*! \file
 *
 * $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 * $Rev: 1398 $
 *
 * \brief CogniBoost client-server connection interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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
////////////////////////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/CogniBoostProto.h"
#include "CogniBoost/CogniBoostMsgs.h"
#include "CogniBoost/libCogniBoost.h"

#include "version.h"

#include "cbLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Connect to a CogniBoost server.
 *
 * The connection may be direct (i.e. the USB connected to this client's host)
 * or IP proxied via RoadNarrows botsense proxy server.
 *
 * \todo TODO: support botsense.
 *
 * \param sDeviceName   Serial device URI. Syntax:\n
 *                      [botsense://domain[:port]]path-to-device\n
 *                      Examples:\n
 *                      /dev/ttyUSB0\n
 *                      botsense://www.mycloud.com/dev/ttyUSB2\n
 *                      botsense://mycomputer/dev/ttyUSB0
 * \param nBaudRate     CogniBoost serial baud rate.
 * \param bTrace        Do [not] trace messages.
 *
 * \return
 * On success, returns a new CogniBoost device handle.
 * On failure, \ref CB_HND_NONE is returned.
 */
cbHnd_T cbConnect(const char *sDeviceName,
                  int         nBaudRate,
                  bool_t      bTrace)
{
  return cbOpen(sDeviceName, nBaudRate, bTrace);
}

/*!
 * \brief Disconnect from CogniBoost server.
 *
 * The device handle is deallocated and is no longer valid.
 *
 * \todo TODO: support botsense.
 *  
 * \param hnd       CogniBoost device handle.
 *
 * \copydoc doc_std_return
 */
int cbDisconnect(cbHnd_T hnd)
{
  int   rc;

  rc = cbClose(hnd);

  return rc;
}

/*!
 * \brief Get CogniBoost hardware, firmware, and host library identities and
 * versions.
 *
 * \param hnd         CogniBoost device handle.
 * \param [out] pIds  Identities.
 *
 * \copydoc doc_std_return
 */
int cbGetIdentities(cbHnd_T hnd, cbIdentities_T *pIds)
{
  static cbMsgId_T  msgIdReq = cbMsgIdReqVersion;
  static cbMsgId_T  msgIdRsp = cbMsgIdRspVersion;

  cbRspVersion_T    msgRsp;                         // response message 
  int               rc;                             // return code

  rc = cbTrans(hnd, msgIdReq, NULL, msgIdRsp, &msgRsp);

  CHK_ECODE(hnd, rc, "Transaction failed.");

  strncpy(pIds->m_sMfgName, msgRsp.m_mfgName, CB_ID_STR_LEN_MAX+1);
  pIds->m_sMfgName[CB_ID_STR_LEN_MAX] = 0;

  strncpy(pIds->m_sProdName, msgRsp.m_prodName, CB_ID_STR_LEN_MAX+1);
  pIds->m_sProdName[CB_ID_STR_LEN_MAX] = 0;

  strncpy(pIds->m_sHwSN, msgRsp.m_hwSN, CB_ID_STR_LEN_MAX+1);
  pIds->m_sHwSN[CB_ID_STR_LEN_MAX] = 0;

  CB_VERSION_STOI_V(msgRsp.m_hwVer, pIds->m_verHw);

  strncpy(pIds->m_sFwApp, msgRsp.m_mfgName, CB_ID_STR_LEN_MAX+1);
  pIds->m_sFwApp[CB_ID_STR_LEN_MAX] = 0;

  CB_VERSION_STOI_V(msgRsp.m_fwVer, pIds->m_verFw);

  strncpy(pIds->m_sFwDate, msgRsp.m_fwDate, CB_ID_STR_LEN_MAX+1);
  pIds->m_sFwDate[CB_ID_STR_LEN_MAX] = 0;

  CB_VERSION_STOI_V(PKG_VERSION, pIds->m_verLib);

  LOGDIAG3("Device %d: %s: ids = {\n"
            "  Manufacturer:   %s\n"
            "  Product:        %s\n"
            "  HW S/N:         %s\n"
            "  HW Version:     %u.%u.%u\n"
            "  FW Application: %s\n"
            "  FW Version:     %u.%u.%u\n"
            "  FW Date:        %s\n"
            "  SW Version:     %u.%u.%u\n"
            "}",
      cbAttrGetFd(hnd), LOGFUNCNAME,
      pIds->m_sMfgName, pIds->m_sProdName, pIds->m_sHwSN,
      pIds->m_verHw.m_uVerMajor, pIds->m_verHw.m_uVerMinor,
      pIds->m_verHw.m_uRevision,
      pIds->m_sFwApp,
      pIds->m_verFw.m_uVerMajor, pIds->m_verFw.m_uVerMinor,
      pIds->m_verFw.m_uRevision,
      pIds->m_sFwDate,
      pIds->m_verLib.m_uVerMajor, pIds->m_verLib.m_uVerMinor,
      pIds->m_verLib.m_uRevision);

  return CB_OK;
}

/*!
 * \brief Ping the CogniBoost server.
 *
 * \param hnd         CogniBoost device handle.
 *
 * \copydoc doc_std_return
 */
int cbPing(cbHnd_T hnd)
{
  static cbMsgId_T  msgIdReq = cbMsgIdReqPing;
  static cbMsgId_T  msgIdRsp = cbMsgIdRspOk;

  struct timeval    tvMark;         // time mark
  uint_t            uSec;           // micro seconds
  double            mSec;           // milli seconds
  int               rc;             // return code

  timer_mark(&tvMark);

  rc = cbTrans(hnd, msgIdReq, NULL, msgIdRsp, NULL);

  uSec = timer_elapsed(&tvMark);

  CHK_ECODE(hnd, rc, "Transaction failed.");

  mSec = (double)uSec / 1000.0;

  LOGDIAG3("Device %d: %s: %.2f ms.", cbAttrGetFd(hnd), LOGFUNCNAME, mSec);

  return CB_OK;
}

/*!
 * \brief Loopback a buffer between the client and the CogniBoost server.
 *
 * \param hnd         CogniBoost device handle.
 * \param buf         Buffer to loop back.
 * \param uCount      Number of bytes in buffer to loop.
 *
 * \copydoc doc_std_return
 */
int cbLoopback(cbHnd_T hnd, byte_t buf[], size_t uCount)
{
  static cbMsgId_T  msgIdReq = cbMsgIdReqLoopback;
  static cbMsgId_T  msgIdRsp = cbMsgIdRspLoopback;

  cbReqLoopback_T   msgReq;                         // request message 
  cbRspLoopback_T   msgRsp;                         // response message 
  int               rc;                             // return code

  if( uCount > CB_REQLOOPBACK_CDATA_LEN )
  {
    uCount = CB_REQLOOPBACK_CDATA_LEN;
  }

  memcpy(msgReq.m_cdata.u.m_buf, buf, uCount);
  msgReq.m_cdata.m_count = uCount;

  rc = cbTrans(hnd, msgIdReq, &msgReq, msgIdRsp, &msgRsp);

  CHK_ECODE(hnd, rc, "Transaction failed.");

  if( LOGABLE(LOG_LEVEL_DIAG3) )
  {
    LOGDIAG3("Device %d: %s:", cbAttrGetFd(hnd), LOGFUNCNAME);
    bsLogAsciiBuf("Sent:", msgReq.m_cdata.u.m_buf, uCount);
    bsLogAsciiBuf("Recv:", msgRsp.m_cdata.u.m_buf, msgRsp.m_cdata.m_count);
  }

  return CB_OK;
}
