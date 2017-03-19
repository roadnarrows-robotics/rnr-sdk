////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibCM.c
//
/*! \file
 *
 * $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 * $Rev: 1398 $
 *
 * \brief CogniBoost CogniMem low-level interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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
////////////////////////////////////////////////////////////////////////////////


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
 * \brief Read the value of a CogniMem register at the given address.
 *
 * \param hnd         CogniBoost device handle.
 * \param byAddr      CogniMem register address.
 * \param [out] pVal  Pointer to read value read.
 *
 * \copydoc doc_std_return
 */
int cbCMReadReg(cbHnd_T hnd, byte_t byAddr, u_short *pVal)
{
  static cbMsgId_T  msgIdReq = cbMsgIdReqCMReadReg;
  static cbMsgId_T  msgIdRsp = cbMsgIdRspCMReadReg;

  cbReqCMReadReg_T  msgReq;                         // request message 
  cbRspCMReadReg_T  msgRsp;                         // response message 
  int               rc;                             // return code

  msgReq.m_addr = byAddr;

  rc = cbTrans(hnd, msgIdReq, &msgReq, msgIdRsp, &msgRsp);

  CHK_ECODE(hnd, rc, "Transaction failed.");

  CHK_EXPR(hnd, (msgReq.m_addr == msgRsp.m_addr), CB_LIB_ECODE_INVAL,
      "Register address=0x%02x != expected address=0x%02x.",
      msgRsp.m_addr, msgReq.m_addr);

  *pVal = msgRsp.m_value;

  LOGDIAG3("Device %d: %s: CM register 0x%02x=0x%04x.",
      cbAttrGetFd(hnd), LOGFUNCNAME, byAddr, *pVal);

  return CB_OK;
}

/*!
 * \brief Write the given value to a CogniMem register at the given address.
 *
 * \param hnd         CogniBoost device handle.
 * \param byAddr      CogniMem register address.
 * \param [in] huVal  Value to write.
 *
 * \copydoc doc_std_return
 */
int cbCMWriteReg(cbHnd_T hnd, byte_t byAddr, u_short huVal)
{
  static cbMsgId_T  msgIdReq = cbMsgIdReqCMWriteReg;
  static cbMsgId_T  msgIdRsp = cbMsgIdRspOk;

  cbReqCMWriteReg_T msgReq;                         // request message 
  int               rc;                             // return code

  msgReq.m_addr  = byAddr;
  msgReq.m_value = huVal;

  rc = cbTrans(hnd, msgIdReq, &msgReq, msgIdRsp, NULL);

  CHK_ECODE(hnd, rc, "Transaction failed.");

  LOGDIAG3("Device %d: %s: CM register 0x%02x=0x%04x.",
      cbAttrGetFd(hnd), LOGFUNCNAME, byAddr, huVal);

  return CB_OK;
}

