////////////////////////////////////////////////////////////////////////////////
//
// Package:   kuon
//
// Module:    bsKuon
//
// Library:   libbsserver_kuon
//
// File:      bsKuonServer.c
//
//
/*! \file
 *
 * $LastChangedDate: 2011-01-13 11:05:37 -0700 (Thu, 13 Jan 2011) $
 * $Rev: 657 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL Kuon robot device
 * module.
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
#include "rnr/imu/yost.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsKuon.h"
#include "botsense/bsKuonMsgs.h"

#include "Kuon/kuon.h"
#include "Kuon/RS160DControl.h"

#include "bsKuonInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo = 
{
  BS_KUON_SERVER_MOD,
  "The Kuon proxied robot.",
  "1.0.1",
  "2012.09.13",
  "RaodNarrows LLC",
  "(C) 2012 RoadNarrows LLC. All rights reserved."
};


/*!
 * \ingroup bsmod_kuon_srv
 * \defgroup bsmod_kuon_srv_tunes Default Tunables
 *
 * \h_botsense KUON Interface Module Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */

/*!
 * \brief Maximum number of simultaneous virtual connections supported by
 * this module.
 */
#define BSMOD_MAX_HANDLES     1

/*! \} */

#define BSMOD_KUON_RD         1     ///< fixed pseudo resource id

static BsVConnHnd_T   CurrVConn;

//
// Data
//
static const char           *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table

C_DECLS_END


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Module Utility Functions
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
  BsModCtlBlk(bool bTrace, struct RS160Ds *pRS160, Yost *pYost)
  {
    m_bTrace           = bTrace;
    m_MotorControllers = *pRS160;
    m_pYost            = pYost;
    memset(&m_imu, 0, sizeof(KuonImu_T));
  }

  /*!
   * \brief Default destructor.
   */
  ~BsModCtlBlk()
  {
    delete m_pYost;
  }

  struct RS160Ds  m_MotorControllers;
  KuonImu_T       m_imu;
  Yost           *m_pYost;
  bool_t          m_bTrace;                   ///< do [not] trace messages  
};

/*!
 * \brief Request Execution Function Type.
 */
typedef int (*BsModExecFunc_T)(BsVConnHnd_T hndVConn, 
                               BsTid_T tid, 
                               BsModCtlBlk  *pCtlBlk, 
                               void *pReq, 
                               void *pRsp);


/*!
 * \brief Process a client's device-specific request and response.
 *
 * The module static data are access in this function.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param eMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Size of request in buffer (number of bytes).
 * \param pReq      Pointer to the specific request message body structure. The
 *                  message will be unpacked into this structure. NULL if no
 *                  message body is associated with the request.
 * \param fnExec    Actual request excution function.
 * \param eMsgIdRsp Response message id.
 * \param pRsp      Pointer to the specific response message body structure. The
 *                  message will be packed from this structure. Required.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModDoReq(BsVConnHnd_T     hndVConn,
                      BsTid_T          uTid,
                      BsMsgId_T        eMsgIdReq,
                      byte_t           bufReq[],
                      size_t           uReqLen,
                      void             *pReq,
                      BsModExecFunc_T  fnExec,
                      BsMsgId_T        eMsgIdRsp,
                      void             *pRsp)
{
  BsModCtlBlk    *pCtlBlk;                     // resource control block
  byte_t          bufRsp[BSPROXY_MSG_MAX_LEN];  // req/rsp buffer
  int             n;                            // number of bytes/return code

  //
  // Parameter checks
  //
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
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
  if( pReq != NULL )
  {
    n = BsKuonUnpackMsg((BsKuonMsgId_T)eMsgIdReq, bufReq, uReqLen, pReq,
                          pCtlBlk->m_bTrace);

    // check unpacking return code
    if( n < 0 )
    {
      BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", eMsgIdReq);
      return -BS_ECODE_BAD_MSG;
    }
  }

  //
  // Execute client request.
  //
  if( (n = fnExec(hndVConn, uTid, pCtlBlk, pReq, pRsp)) != BS_OK )
  {
    return n;
  }

  //
  // Pack server response.
  //
  n = BsKuonPackMsg((BsKuonMsgId_T)eMsgIdRsp, pRsp, BSPROXY_BUF_BODY(bufRsp),
                     pCtlBlk->m_bTrace);

  // check packing return code
  if( n < 0 )
  {
    BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", eMsgIdRsp);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Send response.
  //
  BsModCallbacks->m_cbSendRsp(hndVConn, uTid, eMsgIdRsp, bufRsp, (size_t)n);

  return BS_OK;
}

/*!
 * \brief Process a client's device-specific request and response with the
 * BotSense generic ok response.
 *
 * The module static data are access in this function.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param eMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Size of request in buffer (number of bytes).
 * \param pReq      Pointer to the specific request message body structure. The
 *                  message will be unpacked into this structure. NULL if no
 *                  message body is associated with the request.
 * \param fnExec    Actual request excution function.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModDoReqOk(BsVConnHnd_T     hndVConn,
                        BsTid_T          uTid,
                        BsMsgId_T        eMsgIdReq,
                        byte_t           bufReq[],
                        size_t           uReqLen,
                        void            *pReq,
                        BsModExecFunc_T  fnExec)
{
  BsModCtlBlk  *pCtlBlk;                      // resource control block
  int           n;                            // number of bytes/return code

  //
  // Parameter checks
  //
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND; \
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
  if( pReq != NULL )
  {
    n = BsKuonUnpackMsg((BsKuonMsgId_T)eMsgIdReq, bufReq, uReqLen, pReq,
                            pCtlBlk->m_bTrace);

    // check unpacking return code
    if( n < 0 )
    {
      BSMOD_SEND_NMERROR_RSP(BsModCallbacks, hndVConn, uTid, n,
        "MsgId=%u", eMsgIdReq);
      return -BS_ECODE_BAD_MSG;
    }
  }

  //
  // Execute client request.
  //
  if( (n = fnExec(hndVConn, uTid, pCtlBlk, pReq, NULL)) != BS_OK )
  {
    return n;
  }

  //
  // Send response.
  //
  BsModCallbacks->m_cbSendOkRsp(hndVConn, uTid);

  return BS_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Device-Specific Execute Request Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Set the robot motor speeds.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecSetMotorSpeeds(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk   *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsKuonReqSetMotorSpeeds_T  *pMsgReq = (BsKuonReqSetMotorSpeeds_T *)pReq;
  int                       rc;             // return code
  
  // execute client request.
  rc = RS160DUpdateMotorSpeeds( pMsgReq->m_speedleft, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                 pCtlBlk->m_MotorControllers.m_fdRear );

    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "RS160DUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedleft, pCtlBlk->m_MotorControllers.m_fdFront, 0);
    
    return -BS_ECODE_NO_EXEC;
  }

  rc = RS160DUpdateMotorSpeeds( pMsgReq->m_speedleft,
                                pCtlBlk->m_MotorControllers.m_fdRear,
                                0);

  // check execution return code
  if( rc < 0 )
  {
    RS160DEStop(pCtlBlk->m_MotorControllers.m_fdFront, pCtlBlk->m_MotorControllers.m_fdRear);
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "RS160DUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedleft, pCtlBlk->m_MotorControllers.m_fdRear, 0);
    return -BS_ECODE_NO_EXEC;
  }

  rc = RS160DUpdateMotorSpeeds( pMsgReq->m_speedright,
                                pCtlBlk->m_MotorControllers.m_fdFront,
                                1);

  // check execution return code
  if( rc < 0 )
  {
    RS160DEStop(pCtlBlk->m_MotorControllers.m_fdFront, pCtlBlk->m_MotorControllers.m_fdRear);
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "RS160DUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedright, pCtlBlk->m_MotorControllers.m_fdFront, 1);
    return -BS_ECODE_NO_EXEC;
  }

  rc = RS160DUpdateMotorSpeeds( pMsgReq->m_speedright,
                                pCtlBlk->m_MotorControllers.m_fdRear,
                                1);

  // check execution return code
  if( rc < 0 )
  {
    RS160DEStop(pCtlBlk->m_MotorControllers.m_fdFront, pCtlBlk->m_MotorControllers.m_fdRear);
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "RS160DUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedright, pCtlBlk->m_MotorControllers.m_fdRear, 1);
    return -BS_ECODE_NO_EXEC;
  }

  return BS_OK;
}

/*!
 * \brief Set the robot braking rates.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecAlterBrake(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk   *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsKuonReqAlterBrake_T  *pMsgReq = (BsKuonReqAlterBrake_T *)pReq;
  int                       rc;             // return code
  
  // execute client request.
  rc = RS160DAlterBraking( pMsgReq->m_brakeFL, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterBraking( pMsgReq->m_brakeFL, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                0 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterBraking Front Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterBraking( pMsgReq->m_brakeFR, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterBraking( pMsgReq->m_brakeFR, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                1 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterBraking Front Right.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterBraking( pMsgReq->m_brakeRL, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterBraking( pMsgReq->m_brakeRL, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                0 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterBraking Rear Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterBraking( pMsgReq->m_brakeRR, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterBraking( pMsgReq->m_brakeRR, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                1 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterBraking Rear Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }
  return BS_OK;
}

/*!
 * \brief Set the robot slew rates.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecAlterSlew(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk   *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsKuonReqAlterSlew_T  *pMsgReq = (BsKuonReqAlterSlew_T *)pReq;
  int                       rc;             // return code
  
  // execute client request.
  rc = RS160DAlterSlew( pMsgReq->m_slewFL, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterSlew( pMsgReq->m_slewFL, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                0 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterSlew Front Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterSlew( pMsgReq->m_slewFR, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterSlew( pMsgReq->m_slewFR, 
                                pCtlBlk->m_MotorControllers.m_fdFront, 
                                1 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterSlew Front Right.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterSlew( pMsgReq->m_slewRL, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterSlew( pMsgReq->m_slewRL, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                0 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterSlew Rear Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = RS160DAlterSlew( pMsgReq->m_slewRR, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(500);
    rc = RS160DAlterSlew( pMsgReq->m_slewRR, 
                                pCtlBlk->m_MotorControllers.m_fdRear, 
                                1 );
    if(rc < 0) {
      RS160DEStop( pCtlBlk->m_MotorControllers.m_fdFront, 
                   pCtlBlk->m_MotorControllers.m_fdRear );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "RS160DAlterSlew Rear Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }
  return BS_OK;
}

/*!
 * \brief Read IMU.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecReadImu(BsVConnHnd_T hndVConn,
                                BsTid_T        uTid,
                                BsModCtlBlk   *pCtlBlk,
                                void          *pReq,
                                void          *pRsp)
{
  BsKuonRspReadImu_T *pRspMsg = (BsKuonRspReadImu_T *)pRsp;

  pRspMsg->m_ax    = pCtlBlk->m_imu.m_fAccX;
  pRspMsg->m_ay    = pCtlBlk->m_imu.m_fAccY;
  pRspMsg->m_az    = pCtlBlk->m_imu.m_fAccZ;
  pRspMsg->m_gx    = pCtlBlk->m_imu.m_fGyrX;
  pRspMsg->m_gy    = pCtlBlk->m_imu.m_fGyrY;
  pRspMsg->m_gz    = pCtlBlk->m_imu.m_fGyrZ;
  pRspMsg->m_temp  = pCtlBlk->m_imu.m_fTemp;

  return BS_OK;
}

/*!
 * \brief Read IMU decoupled angles.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecReadImuDecoupAngles(BsVConnHnd_T hndVConn,
                                            BsTid_T        uTid,
                                            BsModCtlBlk   *pCtlBlk,
                                            void          *pReq,
                                            void          *pRsp)
{
  BsKuonRspReadImuDecoupAngles_T *pRspMsg =
                                (BsKuonRspReadImuDecoupAngles_T *)pRsp;

  int rc = pCtlBlk->m_pYost->ReadDecoupleAngles(pRspMsg->m_heading,
                                       pRspMsg->m_pitch,
                                       pRspMsg->m_roll);
  if(rc < 0) {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_RSRC,
          "Full message not read from Yost.");
    return -BS_ECODE_NO_RSRC;
  }

  return BS_OK;
}

/*!
 * \brief Zero-out IMU gryoscope accumulators.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecZeroOutImuGyros(BsVConnHnd_T hndVConn,
                                        BsTid_T        uTid,
                                        BsModCtlBlk   *pCtlBlk,
                                        void          *pReq,
                                        void          *pRsp)
{
  pCtlBlk->m_imu.m_fGyrX = 0.0;
  pCtlBlk->m_imu.m_fGyrY = 0.0;
  pCtlBlk->m_imu.m_fGyrZ = 0.0;

  return BS_OK;
}

/*!
 * \brief Stop the robot.
 *
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-Response transaction id.
 * \param [in] pReq   Pointer to request data structure. Ignored.
 * \param [out] pRsp  Pointer to response data structure. Ignored.
 *
 * \copydoc doc_return_bs_std
 */
static int bsModKuonExecStop(BsVConnHnd_T hndVConn,
                           BsTid_T        uTid,
                           BsModCtlBlk   *pCtlBlk,    
                           void          *pReq,
                           void          *pRsp)
{
  int   rc = 0; // return code

  //
  // Execute client request.
  //
  RS160DEStop(pCtlBlk->m_MotorControllers.m_fdFront, 
              pCtlBlk->m_MotorControllers.m_fdRear);


  // todo: get error code from estop
  // check operation return code
  if( rc < 0 )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "Failed to shutdown Kuon, You're in trouble.");
    return -BS_ECODE_NO_EXEC;
  }

  return BS_OK;
}

static int bsModKuonExecAlive(BsVConnHnd_T hndVConn,
                           BsTid_T        uTid,
                           BsModCtlBlk   *pCtlBlk,    
                           void          *pReq,
                           void          *pRsp)
{
  int   rc = BS_OK; // return code

  //
  // Execute client request.
  //
 
  // check operation return code
  if( rc < 0 )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "Failed to shutdown Kuon, You're in trouble.");
    return -BS_ECODE_NO_EXEC;
  }

  return BS_OK;
}

// ---------------------------------------------------------------------------
// Exported Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Initialize the Kuon module.
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
 * \brief Exit the Kuon module.
 *
 * Called once prior to module being unloaded.
 *
 * All open /dev/null devices will be closed.
 */
void bsModExit()
{
  int             index;
  BsModCtlBlk    *pCtlBlk;

  // free all module virtual connnection resources.
  for(index=0; index<BSMOD_MAX_HANDLES; ++index)
  {
    if( (pCtlBlk = (BsModCtlBlk *)BsModRsrcTbl->m_vecRsrc[index]) != NULL )
    {
      RS160DClose(pCtlBlk->m_MotorControllers.m_fdRear);
      RS160DClose(pCtlBlk->m_MotorControllers.m_fdFront);
      delete pCtlBlk;
    }
  }

  // free resource table
  BsModCallbacks->m_cbModRsrcTblDelete(BsModRsrcTbl);

  delete((char *)BsModUri);
}

/*!
 * \brief Open a proxied Kuon robot and associate with the given handle.
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
  BsKuonReqOpenParams_T openParams;
  struct RS160Ds RS160D;
  Yost             *pYost;
  BsModCtlBlk      *pCtlBlk;  // resource control block

  int nTries;
  int nMaxTries = 2;
  int rc; // error/return code

  // Check if handle is in valid range.
  BSMOD_TRY_VCONN_HND_RANGE(hndVConn);

  // Check if the handle in this module's resource table is not already in-use.
  if( BSMOD_RSRC_IS_INUSE(BsModRsrcTbl, hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BUSY, 
        "Module virtual connection handle already in use.");
    return -BS_ECODE_BUSY;
  }

  //
  // Only one, platform-wide robot pseudo-device is supported.
  //
  if( strcmp(sDevUri, BS_KUON_DEV_NAME) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_DEV,
        "device URI \"%s\" != required \"%s\".", sDevUri, BS_KUON_DEV_NAME);
    return -BS_ECODE_NO_DEV;
  }

  // Only one instance of the robot is supported per platform, so no device
  // need operations such as open() are done.
  BsKuonUnpackReqOpenParams( argbuf, uArgSize, &openParams , bTrace);
  
  if( BSMOD_RSRC_INUSE_COUNT(BsModRsrcTbl) >= BSMOD_MAX_HANDLES )
  {
    if(openParams.m_server)
    {
      bsModClose(CurrVConn);
    }
    else
    {
       BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_RSRC,
        "InUseCount=%u: No more resources available.",
        BSMOD_RSRC_INUSE_COUNT(BsModRsrcTbl));
        return -BS_ECODE_NO_RSRC;
    }
  }

  //
  // Open front motor controller.
  //
  for(nTries = 0, rc = -1; (nTries < nMaxTries) && (rc < 0); ++nTries)
  {
    if( nTries > 0 )
    {
      LOGDIAG2("Failed to open front motor controller... Trying again.");
      usleep(1000);
    }
    rc = RS160DOpenConnection(openParams.m_FrontAddr, &(RS160D.m_fdFront));
    //rc = RS160DOpenConnection("/dev/kmot0", &(RS160D.m_fdFront));
  }

  if(rc < 0) {
    LOGDIAG2("Unable to open connection to front motor controller.");
    return -BS_ECODE_NO_RSRC;
  }

  //
  // Open rear motor controller.
  //
  for(nTries = 0, rc = -1; (nTries < nMaxTries) && (rc < 0); ++nTries)
  {
    if( nTries > 0 )
    {
      LOGDIAG2("Failed to open rear motor controller... Trying again.");
      usleep(1000);
    }
    rc = RS160DOpenConnection(openParams.m_RearAddr, &(RS160D.m_fdRear));
    //rc = RS160DOpenConnection("/dev/kmot1", &(RS160D.m_fdRear));
  }

  if(rc < 0) {
    LOGDIAG2("Unable to open connection to rear motor controller.");
    return -BS_ECODE_NO_RSRC;
  }

  //
  // Configure front motor controller.
  //
  for(nTries = 0, rc = -1; (nTries < nMaxTries) && (rc < 0); ++nTries)
  {
    if( nTries > 0 )
    {
      LOGDIAG2("Failed to set front RS160D to serial control... retrying.");
      usleep(1000);
    }
    rc = RS160DSetToSerial(RS160D.m_fdFront);
  }

  if(rc < 0) {
    LOGDIAG2("Unable to set front RS160D to serial control.");
    return -BS_ECODE_NO_RSRC;
  }

  //
  // Configure rear motor controller.
  //
  for(nTries = 0, rc = -1; (nTries < nMaxTries) && (rc < 0); ++nTries)
  {
    if( nTries > 0 )
    {
      LOGDIAG2("Failed to set rear RS160D to serial control... retrying.");
      usleep(1000);
    }
    rc = RS160DSetToSerial(RS160D.m_fdRear);
  }

  if(rc < 0) {
    LOGDIAG2("Unable to set rear RS160D to serial control.");
    return -BS_ECODE_NO_RSRC;
  }

  pYost = new Yost;

  rc = pYost->OpenYost();

  if( rc < 0 )
  {
    LOGDIAG2("OpenYost() failed, rc=%d.", rc);
    return -BS_ECODE_NO_RSRC;
  }
  
  // allocate a new control block and initialize
  pCtlBlk = new BsModCtlBlk(bTrace, &RS160D, pYost);

  // start background thread
  // RDK bsKuonBgThreadStart(&(pCtlBlk->m_imu));

  // add to module resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  CurrVConn = hndVConn;

  LOGDIAG2("VConn=%d: connection open to %s.", hndVConn, sDevUri);

  // fixed resource descriptor
  return BSMOD_KUON_RD;
}

/*!
 * \brief Close the /dev/null device and disassociate virtual connection handle.
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

  RS160DEStop(pCtlBlk->m_MotorControllers.m_fdFront, 
              pCtlBlk->m_MotorControllers.m_fdRear);
  usleep(5000);
  RS160DClose(pCtlBlk->m_MotorControllers.m_fdRear);
  RS160DClose(pCtlBlk->m_MotorControllers.m_fdFront);

  pCtlBlk->m_pYost->CloseYost();

  // kill background thread
  // RDK bsKuonBgThreadStop();
  // RDK bsKuonBgThreadJoin();

  delete pCtlBlk;

  LOGDIAG2("VConn=%d: connection closed.", hndVConn);

  return BS_OK;
}

/*!
 * \brief Service a proxied Kuon robot request.
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
  BsModCtlBlk  *pCtlBlk;  // resource control block
  int           rc;       // return code

  // check if handle is in valid range
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_BAD_VCONN_HND,
          "Module vconn handle out-of-range.");
    return -BS_ECODE_BAD_VCONN_HND; \
  }

  // retrieve resource control block
  pCtlBlk = (BsModCtlBlk *)BSMOD_RSRC(BsModRsrcTbl, hndVConn);

  // check resource
  if( pCtlBlk == NULL )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_VCONN,
        "No resources for virtual connection found.");
    return -BS_ECODE_NO_VCONN;
  }

  // service specific request
  switch( uMsgIdReq )
  {
    case BsKuonMsgIdReqSetMotorSpeeds:
      {
        BsKuonReqSetMotorSpeeds_T *pReq = new BsKuonReqSetMotorSpeeds_T;
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, pReq,
                        bsModKuonExecSetMotorSpeeds);
        delete pReq;
      }
      break;

    case BsKuonMsgIdReqReadImu:
      {
        BsKuonRspReadImu_T *pRsp = new BsKuonRspReadImu_T;
        rc = bsModDoReq(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                      bsModKuonExecReadImu, BsKuonMsgIdRspReadImu, pRsp);
        delete pRsp;
      }
      break;

    case BsKuonMsgIdReqReadImuDecoupAngles:
      {
        BsKuonRspReadImuDecoupAngles_T *pRsp =
                                            new BsKuonRspReadImuDecoupAngles_T;
        rc = bsModDoReq(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                      bsModKuonExecReadImuDecoupAngles,
                      BsKuonMsgIdRspReadImuDecoupAngles, pRsp);
        delete pRsp;
      }
      break;

    case BsKuonMsgIdReqZeroOutImuGyros:
      {
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                        bsModKuonExecZeroOutImuGyros);
      }
      break;

    case BsKuonMsgIdReqAlterBrake:
      {
        BsKuonReqAlterBrake_T *pReq = new BsKuonReqAlterBrake_T;
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, pReq,
                        bsModKuonExecAlterBrake);
        delete pReq;
      }
      break;

    case BsKuonMsgIdReqAlterSlew:
      {
        BsKuonReqAlterSlew_T *pReq = new BsKuonReqAlterSlew_T;
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, pReq,
                        bsModKuonExecAlterSlew);
        delete pReq;
      }
      break;
/*
    case BsKuonMsgIdReqGetMotorSpeeds:
      pRsp = NEW(BsKuonRspGetMotorSpeeds_T);
      rc = bsModDoReq(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                      bsModKuonExecGetSpeeds,
                      BsKuonMsgIdRspGetMotorSpeeds, pRsp);
      break;
*/
    case BsKuonMsgIdReqStop:
      {
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                        bsModKuonExecStop);
      }
      break;

    case BsKuonMsgIdReqAlive:
      {
        rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                        bsModKuonExecAlive);
      }
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
