////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
// Module:    bsI2C
// Library:   libbsserver_i2c
// File:      bsI2CServer.c
//
/*! \file
 *
 * $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
 * $Rev: 581 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL \h_i2c bus device module.
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

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/netmsgs.h"
#include "rnr/i2c.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsI2C.h"
#include "botsense/bsI2CMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo = 
{
  .mod_name   = BS_I2C_SERVER_MOD,
  .brief      = "Raw I2C Bus proxied device.",
  .version    = "2.0.0",
  .date       = "2010.07.20",
  .maintainer = "RaodNarrows LLC",
  .license    = "(C) 2010 RoadNarrows LLC. All rights reserved."
};

/*!
 * \ingroup bsmod_i2c_srv
 * \defgroup bsmod_i2c_srv_tunes Default Tunables
 *
 * \h_botsense \h_i2c Interface Module Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */

/*!
 * \brief Maximum number of module supported simultaneous virtual connections.
 */
#define BSMOD_MAX_HANDLES     32

/*! \} */

/*!
 * \brief Module resource control block structure type.
 */
typedef struct
{
  uint_t        m_uRefCnt;    ///< reference count to this device
  i2c_t         m_hndI2C;     ///< \h_i2c bus handle
  const char   *m_sDevUri;    ///< \h_i2c device path name
  bool_t        m_bTrace;     ///< do [not] trace messages 
} BsModCtlBlk_T;

static const char           *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table


/*!
 * \brief Allocate a new resource control block.
 *
 * Multiple virtual connection to the same \h_i2c device share the same
 * control block.
 *
 * \param sDevUri   Device URI.
 * \param bTrace    Do [not] trace messages.
 *
 * \return Pointer to new allocated control block.
 */
static BsModCtlBlk_T *bsModCtlBlkNew(const char *sDevUri, bool_t bTrace)
{
  BsModCtlBlk_T *pCtlBlk;

  pCtlBlk = NEW(BsModCtlBlk_T);

  pCtlBlk->m_uRefCnt  = 0;
  pCtlBlk->m_sDevUri  = new_strdup(sDevUri);
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
    delete((void *)(pCtlBlk->m_sDevUri));
    delete(pCtlBlk);
  }
}

/*!
 * \brief Find an already allocated resource control block.
 *
 * \param sDevUri   Device URI search parameter.
 *
 * \return
 * If found, returns pointer to matched control block. Else returns NULL.
 */
static BsModCtlBlk_T *bsModCtlBlkFind(const char *sDevUri)
{
  int             index;
  BsModCtlBlk_T  *pCtlBlk;

  // search through modul'es resource table
  for(index=0; index<BSMOD_MAX_HANDLES; ++index)
  {
    if( (pCtlBlk = (BsModCtlBlk_T *)BsModRsrcTbl->m_vecRsrc[index]) != NULL )
    {
      if( !strcmp(pCtlBlk->m_sDevUri, sDevUri) )
      {
        return pCtlBlk;
      }
    }
  }
  return NULL;
}

/*!
 * \brief Service a \h_i2c read request.
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
static int bsModI2CReqRead(BsVConnHnd_T  hndVConn,
                           BsTid_T       uTid,
                           BsMsgId_T     uMsgIdReq,
                           byte_t        bufReq[],
                           size_t        uReqLen)
{
  static BsI2CMsgId_T  uMsgIdRsp = BsI2CMsgIdRspRead;

  BsModCtlBlk_T    *pCtlBlk;                      // resource control block
  BsI2CReqRead_T    msgReq;                       // request message 
  BsI2CRspRead_T    msgRsp;                       // response message 
  byte_t            bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
  i2c_addr_t        addr;                         // \h_i2c device address
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
  n = BsI2CUnpackReqRead(bufReq, uReqLen, &msgReq, pCtlBlk->m_bTrace);

  // check unpacking return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", uMsgIdReq);
    return -BS_ECODE_BAD_MSG;
  }

  addr = (i2c_addr_t)msgReq.m_addr;

  //
  // Execute client request.
  //
  n = i2c_read(&pCtlBlk->m_hndI2C, addr, msgRsp.m_readbuf.u.m_buf,
                        (uint_t)msgReq.m_readlen);

  // check operation return code
  if( n < 0 )
  {
    BSMOD_SEND_SYSERROR_RSP(BsModCallbacks, hndVConn, uTid,
        "i2c_read(fd=%d, addr=0x%02x, ...).", pCtlBlk->m_hndI2C.fd, addr);
    return -BS_ECODE_SYS;
  }

  LOGDIAG3("VConn=%d: %d=i2c_read(fd=%d, addr=0x%02x, buf=%p, count=%zu).",
      hndVConn, n, pCtlBlk->m_hndI2C.fd, addr, msgRsp.m_readbuf.u.m_buf,
      (size_t)msgReq.m_readlen);

  //
  // Pack server response.
  //
  msgRsp.m_readbuf.m_count = (byte_t)n;
    
  // pack
  n = BsI2CPackRspRead(&msgRsp, BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);

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

/*!
 * \brief Service a \h_i2c write request.
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
static int bsModI2CReqWrite(BsVConnHnd_T hndVConn,
                            BsTid_T      uTid,
                            BsMsgId_T    uMsgIdReq,
                            byte_t       bufReq[],
                            size_t       uReqLen)
{
  static BsI2CMsgId_T  uMsgIdRsp = BsI2CMsgIdRspWrite;

  BsModCtlBlk_T    *pCtlBlk;                      // resource control block
  BsI2CReqWrite_T   msgReq;                       // request message 
  BsI2CRspWrite_T   msgRsp;                       // response message 
  byte_t            bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
  i2c_addr_t        addr;                         // \h_i2c device address
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
  n = BsI2CUnpackReqWrite(bufReq, uReqLen, &msgReq, pCtlBlk->m_bTrace);

  // check unpacking return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", uMsgIdReq);
    return -BS_ECODE_BAD_MSG;
  }

  addr = (i2c_addr_t)msgReq.m_addr;

  //
  // Execute client request.
  //
  n = i2c_write(&pCtlBlk->m_hndI2C, addr, msgReq.m_writebuf.u.m_buf,
                      (uint_t)msgReq.m_writebuf.m_count);

  // check operation return code
  if( n < 0 )
  {
    BSMOD_SEND_SYSERROR_RSP(BsModCallbacks, hndVConn, uTid,
        "i2c_write(fd=%d, addr=0x%02x, ...).", pCtlBlk->m_hndI2C.fd, addr);
    return -BS_ECODE_SYS;
  }

  LOGDIAG3("VConn=%d: %d=i2c_write(fd=%d, addr=0x%02x, buf=%p, count=%zu).",
      hndVConn, n, pCtlBlk->m_hndI2C.fd, addr, msgReq.m_writebuf.u.m_buf,
      (size_t)msgReq.m_writebuf.m_count);

  //
  // Pack server response.
  //
  msgRsp.m_byteswritten = (byte_t)n;
    
  // pack
  n = BsI2CPackRspWrite(&msgRsp, BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);

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

/*!
 * \brief Service a \h_i2c write-read transaction request.
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
static int bsModI2CReqTrans(BsVConnHnd_T hndVConn,
                            BsTid_T      uTid,
                            BsMsgId_T    uMsgIdReq,
                            byte_t       bufReq[],
                            size_t       uReqLen)
{
  static BsI2CMsgId_T  uMsgIdRsp = BsI2CMsgIdRspRead;

  BsModCtlBlk_T    *pCtlBlk;                      // resource control block
  BsI2CReqTrans_T   msgReq;                       // request message 
  BsI2CRspRead_T    msgRsp;                       // response message 
  byte_t            bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
  i2c_addr_t        addr;                         // \h_i2c device address
  int               n;                            // number of bytes/return code
  int               rc;                           // return code

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
  n = BsI2CUnpackReqTrans(bufReq, uReqLen, &msgReq, pCtlBlk->m_bTrace);

  // check unpacking return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", uMsgIdReq);
    return -BS_ECODE_BAD_MSG;
  }

  addr = (i2c_addr_t)msgReq.m_addr;

  //
  // Execute client request.
  //
  rc = i2c_transfer(&pCtlBlk->m_hndI2C, addr,
                  msgReq.m_writebuf.u.m_buf, (uint_t)msgReq.m_writebuf.m_count,
                  msgRsp.m_readbuf.u.m_buf, (uint_t)msgReq.m_readlen);

  // check operation return code
  if( rc < 0 )
  {
    BSMOD_SEND_SYSERROR_RSP(BsModCallbacks, hndVConn, uTid, 
        "i2c_transfer(fd=%d, addr=0x%02x, ...).", pCtlBlk->m_hndI2C.fd, addr);
    return -BS_ECODE_SYS;
  }

  LOGDIAG3("VConn=%d: i2c_transfer(fd=%d, addr=0x%02x, wbuf=%p, wcount=%zu, "
            "rbuf=%p, rcount=%zu).",
      hndVConn, pCtlBlk->m_hndI2C.fd,
      msgReq.m_writebuf.u.m_buf, (size_t)msgReq.m_writebuf.m_count,
      msgRsp.m_readbuf.u.m_buf, (size_t)msgReq.m_readlen);

  //
  // Pack server response. Note: transfer fails if full number of bytes are
  // not read.
  //
  msgRsp.m_readbuf.m_count = (byte_t)msgReq.m_readlen;
    
  // pack
  n = BsI2CPackRspRead(&msgRsp, BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);

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

/*!
 * \brief \h_i2c scan callback function.
 *
 * \param pHndI2C   Pointer to \h_i2c Bus handle.
 * \param addr      Discovered \h_i2c device address during scan.
 * \param pContext  Callback context.
 *
 * \return
 *  On success, returns current number of scanned device found.
 *  On error, errno is set and - \ref BS_ECODE_SYS is returned.
 */
static int bsModI2CScanCallback(i2c_t *pHndI2C, i2c_addr_t addr, void *pContext)
{
  BsI2CRspScan_T  *pMsgRsp = (BsI2CRspScan_T *)pContext;  // response message 

  if( pMsgRsp->m_scan.m_count < BSI2C_RSPSCAN_SCAN_LEN )
  {
    pMsgRsp->m_scan.u.m_buf[pMsgRsp->m_scan.m_count++] = addr;
    return (int)pMsgRsp->m_scan.m_count;
  }
  else
  {
    errno = E2BIG;
    return -BS_ECODE_SYS;
  }
}

/*!
 * \brief Service a \h_i2c scan for devices request.
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
static int bsModI2CReqScan(BsVConnHnd_T  hndVConn,
                           BsTid_T       uTid,
                           BsMsgId_T     uMsgIdReq,
                           byte_t        bufReq[],
                           size_t        uReqLen)
{
  static BsI2CMsgId_T  uMsgIdRsp = BsI2CMsgIdRspScan;

  BsModCtlBlk_T    *pCtlBlk;                      // resource control block
  BsI2CRspScan_T    msgRsp;                       // response message 
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

  // no devices yet
  msgRsp.m_scan.m_count = (byte_t)0;
    
  //
  // Execute client request.
  //
  n = i2c_scan(&pCtlBlk->m_hndI2C, bsModI2CScanCallback, &msgRsp);

  // check operation return code
  if( n < 0 )
  {
    BSMOD_SEND_SYSERROR_RSP(BsModCallbacks, hndVConn, uTid,
        "i2c_scan(fd=%d, ...).", pCtlBlk->m_hndI2C.fd);
    return -BS_ECODE_SYS;
  }

  LOGDIAG3("VConn=%d: %d=i2c_scan(fd=%d, ...).",
      hndVConn, n, pCtlBlk->m_hndI2C.fd);

  //
  // Pack server response.
  //
  // pack
  n = BsI2CPackRspScan(&msgRsp, BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);

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
 * \brief Initialize \h_i2c module.
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
 * \brief Exit \h_i2c module.
 *
 * Called once prior to module being unloaded.
 *
 * All open \h_i2c devices will be closed.
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
      if( pCtlBlk->m_uRefCnt <= 1 )
      {
        i2c_close(&pCtlBlk->m_hndI2C);
        bsModCtlBlkDelete(pCtlBlk);
      }
      else
      {
        pCtlBlk->m_uRefCnt--;
      }
    }
  }

  // free resource table
  BsModCallbacks->m_cbModRsrcTblDelete(BsModRsrcTbl);

  delete((char *)BsModUri);
}

/*!
 * \brief Open an \h_i2c device and associate with the given handle.
 *
 * Subsequent calls to the module use the given handle to associate the 
 * specific module-device instance.
 *
 * The argument buffer contains packed message arguements specific to the 
 * device and module. For this \h_i2c module, there are now additional
 * arguments.
 *
 * \note Each call to this exported function will only open the \h_i2c device
 * if it is the first call. All virtual connections to the same device share
 * the same opened file resource descriptor.
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
  BsModCtlBlk_T        *pCtlBlk;
  int                   rc;

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

  //
  // Search for an existing opened device control block. If this is a first 
  // reference to the device, then create a shared control block and open
  // the actual device.
  //
  if( (pCtlBlk = bsModCtlBlkFind(sDevUri)) == NULL )
  {
    // allocate a new control block and initialize
    pCtlBlk = bsModCtlBlkNew(sDevUri, bTrace);

    // open the device
    rc = i2c_open(&pCtlBlk->m_hndI2C, sDevUri);

    if( rc < 0 )
    {
      BSMOD_LOG_SYSERROR(hndVConn, "i2c_open(%p, %s).",
                                  &pCtlBlk->m_hndI2C, sDevUri);
      bsModCtlBlkDelete(pCtlBlk);
      return -BS_ECODE_SYS;
    }
  }

  // bump device reference count
  pCtlBlk->m_uRefCnt++;

  // add (a copy) to this module's resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  LOGDIAG2("VConn=%d: %d=i2c_open(%p, dev=%s).",
      hndVConn, pCtlBlk->m_hndI2C.fd, &pCtlBlk->m_hndI2C, sDevUri);

  return pCtlBlk->m_hndI2C.fd;
}

/*!
 * \brief Close the \h_i2c device and disassociate virtual connection handle.
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

  //
  // Closing last reference to device. So really close and free resources.
  //
  if( pCtlBlk->m_uRefCnt <= 1 )
  {
    i2c_close(&pCtlBlk->m_hndI2C);

    LOGDIAG2("VConn=%d: i2c_close(fd=%d).", hndVConn, pCtlBlk->m_hndI2C.fd);

    bsModCtlBlkDelete(pCtlBlk);
  }

  // Simply decrement reference count.
  else
  {
    pCtlBlk->m_uRefCnt--;
  }

  return BS_OK;
}

/*!
 * \brief Service an \h_i2c request.
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
    case BsI2CMsgIdReqRead:
      return bsModI2CReqRead(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen);
    case BsI2CMsgIdReqTrans:
      return bsModI2CReqTrans(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen);
    case BsI2CMsgIdReqWrite:
      return bsModI2CReqWrite(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen);
    case BsI2CMsgIdReqScan:
      return bsModI2CReqScan(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen);
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
