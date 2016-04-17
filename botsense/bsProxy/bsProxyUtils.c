////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyUtils.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2010.  RoadNarrows LLC.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------


/*! 
 * \brief Calculate the elapsed time between the given time mark and this call.
 *
 * \param pTvMark   Pointer to timeval holding time mark.
 *
 * \return 
 * Number of microseconds elasped. If the marked time is invalid or the current
 * time cannot be ascertained, UINT_MAX is returned.
 */
uint_t timer_elapsed(struct timeval *pTvMark)
{
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) )
  {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec )
  {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (uint_t)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}

#ifdef LOG
/*!
 * \brief Log message header.
 *
 * \param sPreface  Preface string.
 * \param pMsgHdr   Pointer to message header structure.
 */
void BsProxyLogMsgHdr(const char *sPreface, BsProxyMsgHdr_T *pMsgHdr)
{
  FILE             *fp;
  const NMMsgDef_T *pMsgDef;
  const char       *sMsgName;

  fp = LOG_GET_LOGFP();

  if( pMsgHdr != NULL )
  {
    if( pMsgHdr->m_hdrVConn == BSPROXY_VCONN_SERVER )
    {
      pMsgDef = BsProxyLookupMsgDef((BsProxyMsgId_T)pMsgHdr->m_hdrMsgId);
      sMsgName = pMsgDef!=NULL? pMsgDef->m_sMsgName: "";
    }
    else
    {
      sMsgName = "";
    }
    fprintf(fp, "%s MsgHdr = {\n", sPreface);
    fprintf(fp, "  Magic:   0x%04x\n",  pMsgHdr->m_hdrMagic);
    fprintf(fp, "  Tid:     %u\n",      (uint_t)(pMsgHdr->m_hdrTid));
    fprintf(fp, "  VConn:   %u\n",      (uint_t)(pMsgHdr->m_hdrVConn));
    fprintf(fp, "  MsgId:   %u %s\n",   pMsgHdr->m_hdrMsgId, sMsgName);
    fprintf(fp, "  BodyLen: %u\n",      pMsgHdr->m_hdrBodyLen);
    fprintf(fp, "}\n");
  }
  else
  {
    fprintf(fp, "%s MsgHdr: (null)\n", sPreface);
  }
}
#endif // LOG

