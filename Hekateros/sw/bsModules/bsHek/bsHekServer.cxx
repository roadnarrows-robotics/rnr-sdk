////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Module:    bsHek
// Library:   libbsserver_hek
// File:      bsHekServer.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2013-06-05 18:13:32 -0600 (Wed, 05 Jun 2013) $
 * $Rev: 3030 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL Hekateros module.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2011-2012.  RoadNarrows LLC.
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsHek.h"
#include "botsense/bsHekMsgs.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaBgThread.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekDesc.h"

using namespace hekateros;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo =
{
  BS_HEK_SERVER_MOD,
  "The proxied Hekateros arm.",
  "1.0.1",
  "2012.09.18",
  "RaodNarrows LLC",
  "(C) 2012 RoadNarrows LLC. All rights reserved."
};


/*!
 * \ingroup bsmod_hek_srv
 * \defgroup bsmod_hek_srv_tunes Default Tunables
 *
 * \h_botsense Hekateros Interface Module Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */

/*!
 * \brief Maximum number of simultaneous virtual connections supported by
 * this module.
 */
#define BSMOD_MAX_HANDLES     2

/*! \} */

//
// Data
//
static char                 *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table

C_DECLS_END

/*!
 * \brief Log Interface Module Hekateros Error.
 *
 * \param hndVConn  Virtual connection handle.
 * \param ecode     Hekateros error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_LOG_DYNA_ERROR(hndVConn, ecode, efmt, ...) \
  BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_EXEC, "%s(hek_ecode=%d): " efmt, \
      DynaStrError(ecode), (ecode>=0? ecode: -ecode), ##__VA_ARGS__)

/*!
 * \brief Log Hekateros Error and Send Error Response.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param ecode     Dynamixel error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, ecode, efmt, ...) \
  do \
  { \
    BSMOD_LOG_DYNA_ERROR(hndVConn, ecode, efmt, ##__VA_ARGS__); \
    BsModCallbacks->m_cbSendErrorRsp(hndVConn, uTid, ecode, efmt, \
                        ##__VA_ARGS__); \
  } while(0)


string readVersion(BsVConnHnd_T  hndVConn)
{
  const char *sFileName = "/etc/hekateros.conf";

  FILE   *fp;           
  char    buf[64];
  string  str = "?.?.?";

  if( (fp = fopen(sFileName, "r")) == NULL )
  {
    BSMOD_LOG_SYSERROR(hndVConn, "fopen(file=%s, mode=r).", sFileName);
  }

  else if( fscanf(fp, "%s", buf) != 1 )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BAD_VAL,
        "fscanf(file=%s, version, ...): no version.");
  }
    
  else
  {
    str = buf;
  }

  if( fp != NULL )
  {
    fclose(fp);
  }

  return str;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsModCtlBlk Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Module resource control block class.
 */
class BsModCtlBlk
{
public:
  /*!
   * \brief Initialization constructor.
   */
  BsModCtlBlk(DynaComm     *pDynaComm,
              DynaChain    *pDynaChain,
              DynaBgThread *pDynaBgThread,
              HekDesc      *pHekDesc,
              bool          bTrace)
  {
    m_pDynaComm     = pDynaComm;
    m_pDynaChain    = pDynaChain;
    m_pDynaBgThread = pDynaBgThread;
    m_pHekDesc      = pHekDesc;
    m_bTrace        = bTrace;
  }

  /*!
   * \brief Default destructor.
   */
  ~BsModCtlBlk()
  {
    if( m_pHekDesc != NULL )
    {
      delete m_pHekDesc;
    }

    if( m_pDynaBgThread != NULL )
    {
      delete m_pDynaBgThread;
    }

    if( m_pDynaChain != NULL )
    {
      delete m_pDynaChain;
    }

    if( m_pDynaComm != NULL )
    {
      delete m_pDynaComm;
    }
  }

  DynaComm     *m_pDynaComm;        ///< dynamixel bus communication
  DynaChain    *m_pDynaChain;       ///< dynamixel chain
  DynaBgThread *m_pDynaBgThread;    ///< dynamixel chain
  HekDesc      *m_pHekDesc;         ///< hekateros description
  bool_t        m_bTrace;           ///< do [not] trace messages 
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReq Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Module device-specific request excution abstract class.
 */
class BsExecReq
{
public:

  /*!
   * \brief Default constructor.
   */
  BsExecReq()
  {
    m_eMsgIdReq = BsHekMsgIdNone;
    m_eMsgIdRsp = BsHekMsgIdNone;
    m_pReqMsg   = NULL;
    m_pRspMsg   = NULL;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~BsExecReq() { }

  /*!
   * \brief Execute a client's device-specific request and response.
   *
   * The module static data are access in this function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param bufReq    Packed request message buffer.
   * \param uReqLen   Size of request in buffer (number of bytes).
   *
   * \copydoc doc_return_bs_std
   */
  virtual int Execute(BsVConnHnd_T     hndVConn,
                      BsTid_T          uTid,
                      byte_t           bufReq[],
                      size_t           uReqLen)
  {
    BsModCtlBlk  *pCtlBlk;                      // resource control block
    byte_t        bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
    int           n;                            // number of bytes/return code
  
    //
    // Parameter checks
    //
    if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid,
          BS_ECODE_BAD_VCONN_HND, "Module vconn handle out-of-range.");
      return -BS_ECODE_BAD_VCONN_HND;
    }
  
    //
    // Retrieve the resource control block.
    //
    pCtlBlk = (BsModCtlBlk *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);
  
    if( pCtlBlk == NULL )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
          "No resources for virtual connection found.");
      return -BS_ECODE_NO_VCONN;
    }
  
    //
    // Unpack client request.
    //
    if( m_pReqMsg != NULL )
    {
      n = UnpackReq(bufReq, uReqLen, pCtlBlk->m_bTrace);
  
      // check unpacking return code
      if( n < 0 )
      {
        BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
          "MsgId=%u", m_eMsgIdReq);
        return -BS_ECODE_BAD_MSG;
      }
    }
  
    //
    // Execute client request.
    //
    if( (n = ExecThis(hndVConn, uTid, pCtlBlk)) != BS_OK )
    {
      return n;
    }
  
    //
    // Pack server response.
    //
    n = PackRsp(BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);
  
    // check packing return code
    if( n < 0 )
    {
      BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
          "MsgId=%u", m_eMsgIdRsp);
      return -BS_ECODE_BAD_MSG;
    }
  
    //
    // Send response.
    //
    BsModCallbacks->m_cbSendRsp(hndVConn, uTid, m_eMsgIdRsp, bufRsp, (size_t)n);
  
    return BS_OK;
  }

protected:
  BsHekMsgId_T   m_eMsgIdReq;    ///< request message id
  BsHekMsgId_T   m_eMsgIdRsp;    ///< response message id
  void          *m_pReqMsg;      ///< allocated request message
  void          *m_pRspMsg;      ///< allocated response message

  /*!
   * \brief Request-specific request execution abstract function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T  hndVConn,
                       BsTid_T       uTid,
                       BsModCtlBlk  *pCtlBlk) = 0;

  /*!
   * \brief Unpack request message.
   *
   * \param [in] bufReq   Packed request message buffer.
   * \param uReqLen       Size of request in buffer (number of bytes).
   * \param bTrace        Do [not] trace unpacking.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int UnpackReq(byte_t bufReq[], size_t uReqLen, bool bTrace)
  {
    return BsHekUnpackMsg(m_eMsgIdReq, bufReq, uReqLen, m_pReqMsg, bTrace);
  }

  /*!
   * \brief Pack response message.
   *
   * \param [out] bufRsp  Packed request message buffer.
   * \param uRspSize      Size of output buffer (number of bytes).
   * \param bTrace        Do [not] trace packing.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int PackRsp(byte_t bufRsp[], size_t uRspSize, bool bTrace)
  {
    return BsHekPackMsg(m_eMsgIdRsp, m_pRspMsg, bufRsp, uRspSize, bTrace);
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqOk Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Module device-specific request excution with standard ok response
 * abstract class.
 */
class BsExecReqOk : public BsExecReq
{
public:

  /*!
   * \brief Default constructor.
   */
  BsExecReqOk() { }

  /*!
   * \brief Default destructor.
   */
  virtual ~BsExecReqOk() { }

  /*!
   * \brief Execute a client's device-specific request and ok response.
   *
   * The module static data are access in this function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param bufReq    Packed request message buffer.
   * \param uReqLen   Size of request in buffer (number of bytes).
   *
   * \copydoc doc_return_bs_std
   */
  virtual int Execute(BsVConnHnd_T     hndVConn,
                      BsTid_T          uTid,
                      byte_t           bufReq[],
                      size_t           uReqLen)
  {
    BsModCtlBlk  *pCtlBlk;                      // resource control block
    byte_t        bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
    int           n;                            // number of bytes/return code
  
    //
    // Parameter checks
    //
    if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid,
          BS_ECODE_BAD_VCONN_HND, "Module vconn handle out-of-range.");
      return -BS_ECODE_BAD_VCONN_HND;
    }
  
    //
    // Retrieve the resource control block.
    //
    pCtlBlk = (BsModCtlBlk *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);
  
    if( pCtlBlk == NULL )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
          "No resources for virtual connection found.");
      return -BS_ECODE_NO_VCONN;
    }
  
    //
    // Unpack client request.
    //
    if( m_pReqMsg != NULL )
    {
      n = UnpackReq(bufReq, uReqLen, pCtlBlk->m_bTrace);
  
      // check unpacking return code
      if( n < 0 )
      {
        BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
          "MsgId=%u", m_eMsgIdReq);
        return -BS_ECODE_BAD_MSG;
      }
    }
  
    //
    // Execute client request.
    //
    if( (n = ExecThis(hndVConn, uTid, pCtlBlk)) != BS_OK )
    {
      return n;
    }

    //
    // Send ok response.
    //
    BsModCallbacks->m_cbSendOkRsp(hndVConn, uTid);
  }

  /*!
   * \brief Pack response message.
   *
   * \param [out] bufRsp  Packed request message buffer.
   * \param uRspSize      Size of output buffer (number of bytes).
   * \param bTrace        Do [not] trace packing.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int PackRsp(byte_t bufRsp[], size_t uRspSize, bool bTrace)
  {
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqRspState Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Module device-specific request excution with servo state response
 * abstract class.
 */
class BsExecReqRspState : public BsExecReq
{
public:

  /*!
   * \brief Default constructor.
   */
  BsExecReqRspState()
  {
    m_eMsgIdRsp   = BsHekMsgIdRspState;
    m_pRspMsg     = new BsHekRspState_T;
    m_uNumServos  = DYNA_ID_NUMOF;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~BsExecReqRspState()
  {
    if( m_pRspMsg != NULL )
    {
      delete (BsHekRspState_T *)m_pRspMsg;
    }
  }

  /*!
   * \brief Execute a client's device-specific request and ok response.
   *
   * The module static data are access in this function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param bufReq    Packed request message buffer.
   * \param uReqLen   Size of request in buffer (number of bytes).
   *
   * \copydoc doc_return_bs_std
   */
  virtual int Execute(BsVConnHnd_T     hndVConn,
                      BsTid_T          uTid,
                      byte_t           bufReq[],
                      size_t           uReqLen)
  {
    BsModCtlBlk  *pCtlBlk;                      // resource control block
    int           n;                            // number of bytes/return code
  
    //
    // Parameter checks
    //
    if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid,
          BS_ECODE_BAD_VCONN_HND, "Module vconn handle out-of-range.");
      return -BS_ECODE_BAD_VCONN_HND;
    }
  
    //
    // Retrieve the resource control block.
    //
    pCtlBlk = (BsModCtlBlk *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);
  
    if( pCtlBlk == NULL )
    {
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
          "No resources for virtual connection found.");
      return -BS_ECODE_NO_VCONN;
    }
  
    //
    // Unpack client request.
    //
    if( m_pReqMsg != NULL )
    {
      n = UnpackReq(bufReq, uReqLen, pCtlBlk->m_bTrace);
  
      // check unpacking return code
      if( n < 0 )
      {
        BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
          "MsgId=%u", m_eMsgIdReq);
        return -BS_ECODE_BAD_MSG;
      }
    }
  
    //
    // Execute client request.
    //
    if( (n = ExecThis(hndVConn, uTid, pCtlBlk)) != BS_OK )
    {
      return n;
    }

    //
    // Send ok response.
    //
    return SendStateRsp(hndVConn, uTid, pCtlBlk);
  }

  /*!
   * \brief Pack response message.
   *
   * \param [out] bufRsp  Packed request message buffer.
   * \param uRspSize      Size of output buffer (number of bytes).
   * \param bTrace        Do [not] trace packing.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int PackRsp(byte_t bufRsp[], size_t uRspSize, bool bTrace)
  {
    return BsHekPackMsg(m_eMsgIdRsp, m_pRspMsg, bufRsp, uRspSize, bTrace);
  }

protected:
  uint_t    m_uNumServos;

  int SendStateRsp(BsVConnHnd_T   hndVConn,
                   BsTid_T        uTid,
                   BsModCtlBlk   *pCtlBlk)
  {
    BsHekRspState_T      *pRsp = (BsHekRspState_T *)m_pRspMsg;
    DynaSpeedPosTuple_T   vecState[DYNA_ID_NUMOF];
    int                   iter;
    int                   nServoId;
    DynaServo            *pServo;
    size_t                i;
    byte_t                bufRsp[BSPROXY_MSG_MAX_LEN];  // rsp buffer
    int                   n;
    int                   rc;

    for(i=0, nServoId = pCtlBlk->m_pDynaChain->IterStart(&iter);
        (nServoId != DYNA_ID_NONE) && (i < m_uNumServos);
        nServoId = pCtlBlk->m_pDynaChain->IterNext(&iter))
    {
      if( (pServo = pCtlBlk->m_pDynaChain->GetServo(nServoId)) == NULL )
      {
        continue;
      }

      pRsp->m_state.u.m_buf[i].m_servo_id = (byte_t)nServoId;
      pRsp->m_state.u.m_buf[i].m_goal_speed =
                                        (short)pServo->GetState().m_nCurSpeed;
      pRsp->m_state.u.m_buf[i].m_goal_pos = (short)pServo->GetOdometer();

      ++i;
    }

    pRsp->m_state.m_count = i;

    //
    // Pack server response.
    //
    n = PackRsp(BSPROXY_BUF_BODY(bufRsp), pCtlBlk->m_bTrace);
  
    // check packing return code
    if( n < 0 )
    {
      BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
          "MsgId=%u", m_eMsgIdRsp);
      return -BS_ECODE_BAD_MSG;
    }
  
    //
    // Send response.
    //
    BsModCallbacks->m_cbSendRsp(hndVConn, uTid, m_eMsgIdRsp, bufRsp, (size_t)n);
  
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqGetVersion Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Get Hekateros version.
 */
class BsExecReqGetVersion : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqGetVersion()
  {
    m_eMsgIdReq = BsHekMsgIdReqGetVersion;
    m_eMsgIdRsp = BsHekMsgIdRspGetVersion;
    m_pReqMsg   = NULL;
    m_pRspMsg   = new BsHekRspGetVersion_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqGetVersion()
  {
    if( m_pRspMsg != NULL )
    {
      delete (BsHekRspGetVersion_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T   hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    BsHekRspGetVersion_T *pRsp = (BsHekRspGetVersion_T *)m_pRspMsg;

    int     rc;

    strcpy(pRsp->m_version, pCtlBlk->m_pHekDesc->m_strVersion.c_str());
    
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqMoveAtSpeedTo Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Move servos at speed to goal position.
 */
class BsExecReqMoveAtSpeedTo : public BsExecReqRspState
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqMoveAtSpeedTo()
  {
    m_eMsgIdReq = BsHekMsgIdReqMoveAtSpeedTo;
    m_pReqMsg   = new BsHekReqMoveAtSpeedTo_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqMoveAtSpeedTo()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsHekReqMoveAtSpeedTo_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    BsHekReqMoveAtSpeedTo_T *pReq = (BsHekReqMoveAtSpeedTo_T *)m_pReqMsg;

    DynaSpeedPosTuple_T vecMove[DYNA_ID_NUMOF];

    size_t    i;
    int       rc;

    m_uNumServos = pReq->m_move.m_count;

    if( m_uNumServos > DYNA_ID_NUMOF )
    {
      m_uNumServos = DYNA_ID_NUMOF;
    }

    for(i=0; i<m_uNumServos; ++i)
    {
      vecMove[i].m_nServoId = (int)pReq->m_move.u.m_buf[i].m_servo_id;
      vecMove[i].m_nSpeed   = (int)pReq->m_move.u.m_buf[i].m_goal_speed;
      vecMove[i].m_nPos     = (int)pReq->m_move.u.m_buf[i].m_goal_pos;
    }

    rc = pCtlBlk->m_pDynaChain->SyncMoveAtSpeedTo(vecMove, m_uNumServos);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "SyncMoveAtSpeedTo(..., %u).", m_uNumServos);
      return rc;
    }

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqGetState Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Get Hekateros servo dynamics state.
 */
class BsExecReqGetState : public BsExecReqRspState
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqGetState()
  {
    m_eMsgIdReq = BsHekMsgIdReqGetState;
    m_pReqMsg   = NULL;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqGetState() { };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqFreeze Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Freeze Hekateros.
 */
class BsExecReqFreeze : public BsExecReqRspState
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqFreeze()
  {
    m_eMsgIdReq = BsHekMsgIdReqFreeze;
    m_pReqMsg   = NULL;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqFreeze() { };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    return pCtlBlk->m_pDynaChain->Freeze();
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqEStop Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Emergency stop Hekateros.
 */
class BsExecReqEStop : public BsExecReqRspState
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqEStop()
  {
    m_eMsgIdReq = BsHekMsgIdReqEStop;
    m_pReqMsg   = NULL;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqEStop() { };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    return pCtlBlk->m_pDynaChain->EStop();
    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqCalibrate Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Calibrate Hekateros.
 */
class BsExecReqCalibrate : public BsExecReqRspState
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqCalibrate()
  {
    m_eMsgIdReq = BsHekMsgIdReqCalibrate;
    m_pReqMsg   = NULL;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqCalibrate() { };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    int           iter;
    int           nServoId;
    DynaServo    *pServo;
    uint_t        uOdZeroPt;
    double        fGearRatio;
    bool          bIsReversed;
    int           rc;

    for(nServoId = pCtlBlk->m_pDynaChain->IterStart(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = pCtlBlk->m_pDynaChain->IterNext(&iter))
    {
      if( (pServo = pCtlBlk->m_pDynaChain->GetServo(nServoId)) == NULL )
      {
        continue;
      }

      if( (rc = pServo->Read(DYNA_ADDR_CUR_POS_LSB, &uOdZeroPt)) != DYNA_OK )
      {
        return rc;
      }

     

      switch( nServoId )
      {
        case HekServoIdBase:
          fGearRatio = pCtlBlk->m_pHekDesc->m_gearRat_Base;
          bIsReversed = false;
          break;
        case HekServoIdShoulderL:
        case HekServoIdShoulderR:
          fGearRatio = pCtlBlk->m_pHekDesc->m_gearRat_Shoulder;
          bIsReversed = false;
          break;
        case HekServoIdElbow:
          fGearRatio = pCtlBlk->m_pHekDesc->m_gearRat_Elbow;
          bIsReversed = false;
          break;
        case HekServoIdWristPitch:
          fGearRatio = pCtlBlk->m_pHekDesc->m_gearRat_WristPitch;
          bIsReversed = false;
          break;
        case HekServoIdWristRot:
          fGearRatio = pCtlBlk->m_pHekDesc->m_gearRat_WristRot;
          bIsReversed = false;
          break;
        case HekServoIdGraboid:
          fGearRatio = 1.0;
          break;
      }

      pServo->ResetOdometer((int)uOdZeroPt, false);
    }

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqGetHealth Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Freeze Hekateros.
 */
class BsExecReqGetHealth : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqGetHealth()
  {
    m_eMsgIdReq = BsHekMsgIdReqGetHealth;
    m_eMsgIdRsp = BsHekMsgIdRspGetHealth;
    m_pReqMsg   = new BsHekReqGetHealth_T;
    m_pRspMsg   = new BsHekRspGetHealth_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqGetHealth()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsHekReqGetHealth_T *)m_pReqMsg;
      m_pReqMsg = NULL;
    }
    if( m_pRspMsg != NULL )
    {
      delete (BsHekRspGetHealth_T *)m_pRspMsg;
      m_pRspMsg = NULL;
    }
  };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    BsHekReqGetHealth_T    *pReq = (BsHekReqGetHealth_T *)m_pReqMsg;
    BsHekRspGetHealth_T    *pRsp = (BsHekRspGetHealth_T *)m_pRspMsg;
    int                     nServoId;
    DynaServo              *pServo;
    const DynaServoState_T *pState;
    int                     i, j;

    for(i=0, j=0; i<pReq->m_servo_id.m_count; ++i)
    {
      nServoId = (int)pReq->m_servo_id.u.m_buf[i];

      if( (pServo = pCtlBlk->m_pDynaChain->GetServo(nServoId)) == NULL )
      {
        continue;
      }
      
      pState = &pServo->GetState();

      pRsp->m_health.u.m_buf[j].m_servo_id = (byte_t)nServoId;
      pRsp->m_health.u.m_buf[j].m_alarms   = (byte_t)pState->m_uAlarms;
      pRsp->m_health.u.m_buf[j].m_load     = (short)pState->m_nCurLoad;
      pRsp->m_health.u.m_buf[j].m_volts    = (ushort_t)pState->m_uCurVolt;
      pRsp->m_health.u.m_buf[j].m_temp     = (ushort_t)pState->m_uCurTemp;

      ++j;
    }

    pRsp->m_health.m_count = j;

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqClearAlarms Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief ClearAlarmskateros.
 */
class BsExecReqClearAlarms : public BsExecReqOk
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqClearAlarms()
  {
    m_eMsgIdReq = BsHekMsgIdReqClearAlarms;
    m_pReqMsg   = new BsHekReqClearAlarms_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqClearAlarms()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsHekReqClearAlarms_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Hekateros communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T        uTid,
                       BsModCtlBlk   *pCtlBlk)
  {
    BsHekReqClearAlarms_T  *pReq = (BsHekReqClearAlarms_T *)m_pReqMsg;
    int                     nServoId;
    DynaServo              *pServo;
    const DynaServoState_T *pState;
    int                     i, j;

    for(i=0, j=0; i<pReq->m_servo_id.m_count; ++i)
    {
      nServoId = (int)pReq->m_servo_id.u.m_buf[i];

      if( (pServo = pCtlBlk->m_pDynaChain->GetServo(nServoId)) == NULL )
      {
        continue;
      }
      else if( pServo->GetState().m_uAlarms != DYNA_ALARM_NONE )
      {
        pServo->ReloadMaxTorqueLimit();
      }
    }
      
    return BS_OK;
  }
};


// ---------------------------------------------------------------------------
// Exported (C) Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Initialize the Hekateros module.
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
  BsModUri = new char[strlen(sModUri)+1];
  strcpy(BsModUri, sModUri);

  // save bsProxy server callbacks
  BsModCallbacks = pCallbacks;

  // create module resource table
  BsModRsrcTbl = BsModCallbacks->m_cbModRsrcTblNew(BSMOD_MAX_HANDLES);

  return BS_OK;
}

/*!
 * \brief Exit the Hekateros module.
 *
 * Called once prior to module being unloaded.
 *
 * All open USB serial devices will be closed.
 */
void bsModExit()
{
  int           index;
  BsModCtlBlk  *pCtlBlk;

  // free all module virtual connnection resources.
  for(index=0; index<BSMOD_MAX_HANDLES; ++index)
  {
    if( (pCtlBlk = (BsModCtlBlk *)BsModRsrcTbl->m_vecRsrc[index]) != NULL )
    {
      delete pCtlBlk;
    }
  }

  // free resource table
  BsModCallbacks->m_cbModRsrcTblDelete(BsModRsrcTbl);

  delete[] BsModUri;
}

/*!
 * \brief Open a proxied Hekateros servo chain and associate with the given
 * handle.
 *
 * Subsequent calls to the module use the given handle to associate the 
 * specific module-device instance.
 *
 * The argument buffer contains packed message arguements specific to the 
 * device and module. For this module, there are now additional arguments.
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
 * \copydoc doc_return_bs_ecode
 */
int bsModOpen(BsVConnHnd_T  hndVConn,
              bool_t        bTrace,
              const char   *sDevUri,
              byte_t        argbuf[],
              size_t        uArgSize)
{
  BsHekReqOpenArgs_T  devcfg;
  DynaComm           *pDynaComm;
  DynaChain          *pDynaChain;
  DynaBgThread       *pDynaBgThread;
  HekDesc            *pHekDesc;
  string              strVersion;
  BsModCtlBlk        *pCtlBlk;
  int                 rc;

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
  
  // Check if the resource in resource table is not already in-use.
  else if( BSMOD_RSRC_IS_INUSE(BsModRsrcTbl, hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BUSY, 
        "Module virtual connection handle already in use");
    return -BS_ECODE_BUSY;
  }

  // unpack device configuration arguments submessage
  rc = BsHekUnpackReqOpenArgs(argbuf, uArgSize, &devcfg, bTrace);

  if( rc < 0 )
  {
    BSMOD_LOG_NMERROR(hndVConn, rc,
        "Unpacking failed on BsHekReqOpenArgs submessage");
    return -BS_ECODE_BAD_MSG;
  }

  pDynaComm = DynaComm::New(sDevUri, (int)devcfg.m_baudrate);

  if( pDynaComm == NULL )
  {
    BSMOD_LOG_SYSERROR(hndVConn, "Open(uri=%s, baudrate=%u).",
        sDevUri, devcfg.m_baudrate);
    return -BS_ECODE_SYS;
  }

  pDynaChain = new DynaChain(*pDynaComm);

  pDynaChain->AddNewServosByScan();

  // create background thread and register chain - leave in ready state
  pDynaBgThread = new DynaBgThread();

  //DHP
  sleep(1);

  pDynaBgThread->RegisterChainAgent(pDynaChain);

  pDynaBgThread->Run();

  strVersion = readVersion(hndVConn);

  pHekDesc = new HekDesc(strVersion.c_str());

  // allocate a new control block and initialize
  pCtlBlk = new BsModCtlBlk(pDynaComm, pDynaChain, pDynaBgThread, pHekDesc,
                                bTrace);

  // add to module resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  LOGDIAG2("VConn=%d: Open(dev=%s, baudrate=%u): %d servos in chain.",
                  hndVConn, sDevUri, devcfg.m_baudrate,
                  pDynaChain->GetNumberInChain());

  return pDynaComm->GetResourceId();
}

/*!
 * \brief Close the USB serial device and disassociate virtual connection
 * handle.
 *
 * The actual device and resources are only cleared if this is the last
 * virtual connection reference to this device.
 *
 * \param hndVConn  Virtual connection handle.
 *
 * \copydoc doc_return_bs_std
 */
int bsModClose(BsVConnHnd_T hndVConn)
{
  BsModCtlBlk  *pCtlBlk;  // resource control block

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
  pCtlBlk = (BsModCtlBlk *)BsModCallbacks->m_cbModRsrcRemove(BsModRsrcTbl,
                                                               hndVConn);

  if( pCtlBlk == NULL )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_INTERNAL, "No module resource found.");
    return -BS_ECODE_INTERNAL;
  }

  delete pCtlBlk;

  LOGDIAG2("VConn=%d: connection closed.", hndVConn);

  return BS_OK;
}

/*!
 * \brief Service a proxied Hekateros servo chain request.
 *
 * \note A module-specific request service must send a response.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Size of request in buffer (number of bytes).
 *
 * \copydoc doc_return_bs_std
 */
int bsModRequest(BsVConnHnd_T hndVConn,
                 BsTid_T      uTid,
                 BsMsgId_T    uMsgIdReq,
                 byte_t       bufReq[],
                 size_t       uReqLen)
{
  BsExecReqGetVersion     reqGetVersion;
  BsExecReqMoveAtSpeedTo  reqMoveAtSpeedTo;
  BsExecReqGetState       reqGetState;
  BsExecReqFreeze         reqFreeze;
  BsExecReqEStop          reqEStop;
  BsExecReqCalibrate      reqCalibrate;
  BsExecReqGetHealth      reqGetHealth;
  BsExecReqClearAlarms    reqClearAlarms;
  int                     rc;       // return code

  // service specific request
  switch( uMsgIdReq )
  {
    case BsHekMsgIdReqGetVersion:
      rc = reqGetVersion.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqMoveAtSpeedTo:
      rc = reqMoveAtSpeedTo.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqGetState:
      rc = reqGetState.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqFreeze:
      rc = reqFreeze.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqEStop:
      rc = reqEStop.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqCalibrate:
      rc = reqCalibrate.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqGetHealth:
      rc = reqGetHealth.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsHekMsgIdReqClearAlarms:
      rc = reqClearAlarms.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    default:
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_UNKNOWN_REQ,
          "MsgId=%u.", uMsgIdReq);
      rc = -BS_ECODE_UNKNOWN_REQ;
      break;
  }

  return rc;
}

/*!
 * \brief Enable/disable message tracing on handle.
 *
 * \param hndVConn  Virtual connection handle.
 * \param bTrace    Do [not] enable message tracing on this handle.
 *
 * \copydoc doc_return_bs_std
 */
int bsModTrace(BsVConnHnd_T hndVConn, bool_t bTrace)
{
  BsModCtlBlk  *pCtlBlk;  // resource control block

  // check if handle is in valid range
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND;
  }

  // retrieve resource control block
  pCtlBlk = (BsModCtlBlk *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);

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

C_DECLS_END
