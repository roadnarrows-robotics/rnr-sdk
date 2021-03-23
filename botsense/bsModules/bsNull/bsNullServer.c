////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
// Module:    bsNull
// Library:   libbsserver_null
// File:      bsNullServer.c
//
/*! \file
 *
 * $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
 * $Rev: 581 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL /dev/null device module.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsNull.h"
#include "botsense/bsNullMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo = 
{
  .mod_name   = BS_NULL_SERVER_MOD,
  .brief      = "The /dev/null proxied device.",
  .version    = "1.0.0",
  .date       = "2010.07.21",
  .maintainer = "RaodNarrows LLC",
  .license    = "(C) 2010 RoadNarrows LLC. All rights reserved."
};


/*!
 * \ingroup bsmod_null_srv
 * \defgroup bsmod_null_srv_tunes Default Tunables
 *
 * \h_botsense DevNull Interface Module Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */

/*!
 * \brief Maximum number of module supported simultaneous virtual connections.
 */
#define BSMOD_MAX_HANDLES     BSPROXY_VCONN_MOD_NUMOF

/*! \} */

/*!
 * \brief Module resource control block structure type.
 */
typedef struct
{
  int           m_fd;         ///< /dev/null open file descriptor
  bool_t        m_bTrace;     ///< do [not] trace messages 
} BsModCtlBlk_T;

static const char           *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table

/*!
 * \brief Allocate a new resource control block.
 *
 * \param fd        File descriptor.
 * \param bTrace    Do [not] trace messages.
 *
 * \return Pointer to new allocated control block.
 */
static BsModCtlBlk_T *bsModCtlBlkNew(int fd, bool_t bTrace)
{
  BsModCtlBlk_T *pCtlBlk;

  pCtlBlk = NEW(BsModCtlBlk_T);

  pCtlBlk->m_fd       = fd;
  pCtlBlk->m_bTrace   = bTrace;

  return pCtlBlk;
}

/*!
 * \brief Delete an allocated resource control block.
 *
 * \param pCtlBlk   Pointer to allocated control block.
 */
static void bsModCtlBlkDelete(BsModCtlBlk_T *pCtlBlk)
{
  if( pCtlBlk != NULL )
  {
    delete(pCtlBlk);
  }
}

/*!
 * \brief Service a /dev/null write request.
 *
 * \note A module-specific request service must send a response.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Size of request in buffer (number of bytes).
 *
 * \copydoc doc_return_std
 */
static int bsModNullReqWrite(BsVConnHnd_T hndVConn,
                            BsTid_T      uTid,
                            BsMsgId_T    uMsgIdReq,
                            byte_t       bufReq[],
                            size_t       uReqLen)
{
  static BsNullMsgId_T  uMsgIdRsp = BsNullMsgIdRspWrite;

  BsModCtlBlk_T    *pCtlBlk;                      // resource control block
  BsNullReqWrite_T   msgReq;                       // request message 
  BsNullRspWrite_T   msgRsp;                       // response message 
  byte_t            bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
  int               n;                            // number of bytes/return code

  //
  // Parameter checks
  //
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND; \
  }

  // retrieve the resource control block
  pCtlBlk = (BsModCtlBlk_T *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);

  if( pCtlBlk == NULL )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
        "No resources for virtual connection found.");
    return -BS_ECODE_NO_VCONN;
  }

  //
  // Unpack client request.
  //
  n = BsNullUnpackReqWrite(bufReq, uReqLen, &msgReq, pCtlBlk->m_bTrace);

  // check unpacking return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", uMsgIdReq);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //
  n = (int)write(pCtlBlk->m_fd, msgReq.m_writebuf.u.m_buf,
                 (size_t)msgReq.m_writebuf.m_count);

  // check operation return code
  if( n < 0 )
  {
    BSMOD_SEND_SYSERROR_RSP(BsModCallbacks, hndVConn, uTid,
        "write(fd=%d, ...).", pCtlBlk->m_fd);
    return -BS_ECODE_SYS;
  }

  LOGDIAG3("VConn=%d: %d=write(fd=%d, buf=%p, count=%zu).",
      hndVConn, n, pCtlBlk->m_fd, msgReq.m_writebuf.u.m_buf,
      (size_t)msgReq.m_writebuf.m_count);

  //
  // Pack server response.
  //
  msgRsp.m_byteswritten = (byte_t)n;
    
  // pack
  n = BsNullPackRspWrite(&msgRsp, BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);

  // check packing return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", uMsgIdRsp);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Send response.
  //
  BsModCallbacks->m_cbSendRsp(hndVConn, uTid, uMsgIdRsp, bufRsp, (size_t)n);

  return BS_OK;
}


// ---------------------------------------------------------------------------
// Exported Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Initialize /dev/null module.
 *
 * Called once after module is loaded.
 *
 * \param sModUri     Expanded, canonical module path name.
 * \param pCallbacks  Pointer to a set of module -\> bsProxy core callback
 *                    functions.
 *
 * \copydoc doc_return_std
 */
int bsModInit(const char *sModUri, const BsModProxyCb_T *pCallbacks)
{
  BsModUri = new_strdup(sModUri);

  // save bsProxy server callbacks
  BsModCallbacks = pCallbacks;

  // create module resource table
  BsModRsrcTbl = BsModCallbacks->m_cbModRsrcTblNew(BSMOD_MAX_HANDLES);

  return BS_OK;
}

/*!
 * \brief Exit /dev/null module.
 *
 * Called once prior to module being unloaded.
 *
 * All open /dev/null devices will be closed.
 */
void bsModExit()
{
  int             index;
  BsModCtlBlk_T  *pCtlBlk;

  // free all module virtual connnection resources.
  for(index=0; index<BSMOD_MAX_HANDLES; ++index)
  {
    if( (pCtlBlk = (BsModCtlBlk_T *)BsModRsrcTbl->m_vecRsrc[index]) != NULL )
    {
      if( pCtlBlk->m_fd >= 0 )
      {
        close(pCtlBlk->m_fd);
      }
      bsModCtlBlkDelete(pCtlBlk);
    }
  }

  // free resource table
  BsModCallbacks->m_cbModRsrcTblDelete(BsModRsrcTbl);

  delete((char *)BsModUri);
}

/*!
 * \brief Open an /dev/null device and associate with the given handle.
 *
 * Subsequent calls to the module use the given handle to associate the 
 * specific module-device instance.
 *
 * The argument buffer contains packed message arguements specific to the 
 * device and module. For this /dev/null module, there are now additional
 * arguments.
 *
 * \note
 * It is the responsibile of the client to ensure consistency if more than
 * one virtual connection is established.
 *
 * \param hndVConn      Virtual connection handle.
 * \param bTrace        Do [not] enable message tracing on this handle.
 * \param sDevUri       Device URI.
 * \param argbuf        Packed specific open configuration arguments submessage.
 * \param uArgSize      Size of packed argumets in buffer (number of bytes).
 *
 * \return
 * On success, returns a unique resource descriptor \h_ge 0 which is typically
 * an opened file descriptor or socket descriptor, but can be module defined.\n
 * \copydoc doc_return_ecode
 */
int bsModOpen(BsVConnHnd_T  hndVConn,
              bool_t        bTrace,
              const char   *sDevUri,
              byte_t        argbuf[],
              size_t        uArgSize)
{
  BsModCtlBlk_T    *pCtlBlk;
  int               fd;

  // Check if handle is in valid range.
  BSMOD_TRY_VCONN_HND_RANGE(hndVConn);

  // Check if resources are available.
  if( BSMOD_RSRC_INUSE_COUNT(BsModRsrcTbl) >= BSMOD_MAX_HANDLES )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_RSRC,
        "InUseCount=%u: No more resources available.",
        BSMOD_RSRC_INUSE_COUNT(BsModRsrcTbl));
    return -BS_ECODE_NO_RSRC;
  }
  
  // Check if the handle in this module's resource table is not already in-use.
  else if( BSMOD_RSRC_IS_INUSE(BsModRsrcTbl, hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BUSY, 
        "Module virtual connection handle already in use");
    return -BS_ECODE_BUSY;
  }

  // open device
  fd = open(sDevUri, O_WRONLY);

  if( fd < 0 )
  {
    BSMOD_LOG_SYSERROR(hndVConn, "open(%s,...).", sDevUri);
    return -BS_ECODE_SYS;
  }

  // allocate a new control block and initialize
  pCtlBlk = bsModCtlBlkNew(fd, bTrace);

  // add to module resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  LOGDIAG2("VConn=%d: %d=open(dev=%s, ...).", hndVConn, fd, sDevUri);

  return fd;
}

/*!
 * \brief Close the /dev/null device and disassociate virtual connection handle.
 *
 * The actual device and resources are only cleared if this is the last
 * virtual connection reference to this device.
 *
 * \param hndVConn  Virtual connection handle.
 *
 * \return
 * \copydoc doc_return_std
 */
int bsModClose(BsVConnHnd_T hndVConn)
{
  BsModCtlBlk_T  *pCtlBlk;

  // check if handle is in valid range
  BSMOD_TRY_VCONN_HND_RANGE(hndVConn);

  // check is resouce is in-use (i.e. opened)
  if( !BSMOD_RSRC_IS_INUSE(BsModRsrcTbl, hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_VCONN,
        "Resources for virtual connection not found.");
    return -BS_ECODE_NO_VCONN;
  }

  // remove from module's resource table
  pCtlBlk = (BsModCtlBlk_T *)BsModCallbacks->m_cbModRsrcRemove(BsModRsrcTbl,
                                                               hndVConn);

  if( pCtlBlk == NULL )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_INTERNAL, "No module resource found.");
    return -BS_ECODE_INTERNAL;
  }

  if( pCtlBlk->m_fd >= 0 )
  {
    close(pCtlBlk->m_fd);
    LOGDIAG2("VConn=%d: close(fd=%d).", hndVConn, pCtlBlk->m_fd);
  }

  bsModCtlBlkDelete(pCtlBlk);

  return BS_OK;
}

/*!
 * \brief Service an /dev/null request.
 *
 * \note A module-specific request service must send a response.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Size of request in buffer (number of bytes).
 *
 * \copydoc doc_return_std
 */
int bsModRequest(BsVConnHnd_T hndVConn,
                 BsTid_T      uTid,
                 BsMsgId_T    uMsgIdReq,
                 byte_t       bufReq[],
                 size_t       uReqLen)
{
  BsModCtlBlk_T  *pCtlBlk;

  // check if handle is in valid range
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND; \
  }

  // retrieve resource control block
  pCtlBlk = (BsModCtlBlk_T *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);

  // check resource
  if( pCtlBlk == NULL )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
        "No resources for virtual connection found.");
    return -BS_ECODE_NO_VCONN;
  }

  switch( uMsgIdReq )
  {
    case BsNullMsgIdReqWrite:
      return bsModNullReqWrite(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen);
    default:
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_UNKNOWN_REQ,
          "MsgId=%u.", uMsgIdReq);
      return -BS_ECODE_UNKNOWN_REQ;
  }
}

/*!
 * \brief Enable/disable message tracing on handle.
 *
 * \param hndVConn  Virtual connection handle.
 * \param bTrace    Do [not] enable message tracing on this handle.
 *
 * \copydoc doc_return_std
 */
int bsModTrace(BsVConnHnd_T hndVConn, bool_t bTrace)
{
  BsModCtlBlk_T  *pCtlBlk;

  // check if handle is in valid range
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND;
  }

  // retrieve resource control block
  pCtlBlk = (BsModCtlBlk_T *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);

  // check resource
  if( pCtlBlk == NULL )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_VCONN,
        "No resources for virtual connection found.");
    return -BS_ECODE_NO_VCONN;
  }

  pCtlBlk->m_bTrace = bTrace;

  return BS_OK;
}

/*!
 * \brief Query for the static module information.
 *
 * \return
 * Pointer to module static information.
 */
const BsModInfo_T *bsModInfo()
{
  return &BsModInfo;
}
