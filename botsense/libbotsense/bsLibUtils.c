////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbsclient
//
// File:      bsLibUtils.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief Library utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2009-2017. RoadNarrows LLC.\n
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
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Pack \h_botsense bsProxy message header.
 *
 * \param [in] pMsgHdr    Pointer to message header structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 *
 * \returns 
 * On success, returns the number of bytes packed.
 * \copydoc doc_return_ecode
 */
int bsPackMsgHdr(BsProxyMsgHdr_T *pMsgHdr, byte_t buf[], size_t bufSize)
{
  int   n = 0;

  if( bufSize < BSPROXY_MSG_HDR_LEN )
  {
    LOGERROR("Buffer size=%zu too small.", bufSize);
    return -BS_ECODE_INTERNAL;
  }

  n += nmPackU16(pMsgHdr->m_hdrMagic, buf+n, bufSize-(size_t)n, NMEndianBig);
  n += nmPackU8(pMsgHdr->m_hdrTid, buf+n, bufSize-(size_t)n, NMEndianBig);
  n += nmPackU8(pMsgHdr->m_hdrVConn, buf+n, bufSize-(size_t)n, NMEndianBig);
  n += nmPackU16(pMsgHdr->m_hdrMsgId, buf+n, bufSize-(size_t)n, NMEndianBig);
  n += nmPackU16(pMsgHdr->m_hdrBodyLen, buf+n, bufSize-(size_t)n, NMEndianBig);

  return n;
}

/*!
 * \brief Unpack \h_botsense bsProxy message header.
 *
 * \param [in] buf        Input message buffer.
 * \param bufSize         Size of input buffer.
 * \param [out] pMsgHdr   Pointer to message header structure.
 *
 * \returns 
 * On success, returns the number of bytes unpacked.
 * \copydoc doc_return_ecode
 */
int bsUnpackMsgHdr(byte_t buf[], size_t bufSize, BsProxyMsgHdr_T *pMsgHdr)
{
  int   n = 0;

  if( bufSize < BSPROXY_MSG_HDR_LEN )
  {
    LOGERROR("Buffer size=%zu too small.", bufSize);
    return -BS_ECODE_INTERNAL;
  }

  n += nmUnpackU16(buf+n, bufSize-(size_t)n, &pMsgHdr->m_hdrMagic, NMEndianBig);
  n += nmUnpackU8(buf+n,  bufSize-(size_t)n, &pMsgHdr->m_hdrTid, NMEndianBig);
  n += nmUnpackU8(buf+n,  bufSize-(size_t)n, &pMsgHdr->m_hdrVConn, NMEndianBig);
  n += nmUnpackU16(buf+n, bufSize-(size_t)n, &pMsgHdr->m_hdrMsgId, NMEndianBig);
  n += nmUnpackU16(buf+n, bufSize-(size_t)n, &pMsgHdr->m_hdrBodyLen,
                                                                  NMEndianBig);

  return n;
}

#ifdef LOG
/*! 
 * \brief Log data bytes.
 *
 * Print diagnostic logging of the contents of a buffer of bytes.
 *
 * \param sBufName  Name of buffer.
 * \param buf       Buffer.
 * \param uCount    Number of bytes to log.
 */
void bsLogBuf(const char *sBufName, byte_t buf[], size_t uCount)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s=", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sBufName);
  for(i=0; i<uCount; ++i)
  {
    if( (i % 16) == 0 )
    {
      fprintf(fp, "\n");
    }
    fprintf(fp, " 0x%02x", buf[i]);
  }
  fprintf(fp, "\n");
}
#endif // LOG

#ifdef LOG
/*! 
 * \brief Log ascii data bytes.
 *
 * Print diagnostic logging of the contents of a buffer of ASCII bytes.
 *
 * \param sBufName  Name of buffer.
 * \param buf       Buffer.
 * \param uCount    Number of bytes to log.
 */
void bsLogAsciiBuf(const char *sBufName, byte_t buf[], size_t uCount)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s=", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sBufName);
  for(i=0; i<uCount; ++i)
  {
    if( isprint(buf[i]) || isspace(buf[i]) )
    {
      switch(buf[i])
      {
        case '\f':
          fprintf(fp, "\\f");
          break;
        case '\n':
          fprintf(fp, "\\n");
          break;
        case '\r':
          fprintf(fp, "\\r");
          break;
        case '\t':
          fprintf(fp, "\\t");
          break;
        case '\v':
          fprintf(fp, "\\v");
          break;
        default:
          fprintf(fp, "%c", buf[i]);
          break;
      }
    }
    else
    {
      fprintf(fp, "\\x%02x", buf[i]);
    }
  }
  fprintf(fp, "\n");
}
#endif // LOG
