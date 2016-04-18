////////////////////////////////////////////////////////////////////////////////
//
// Package:   mousebot
//
// Module:    bsMouseBot
// Library:   libbsserver_mousebot
// File:      bsMouseBotServer.c
//
//
/*! \file
 *
 * $LastChangedDate: 2012-04-25 18:46:22 -0600 (Wed, 25 Apr 2012) $
 * $Rev: 1896 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL MouseBot robot device
 * module.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2012.  RoadNarrows LLC.
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
#include "rnr/new.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsMouseBot.h"
#include "botsense/bsMouseBotMsgs.h"

#include "Kuon/mousebot.h"
#include "Kuon/qikControl.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo = 
{
  .mod_name   = BS_MOUSEBOT_SERVER_MOD,
  .brief      = "The MouseBot proxied robot.",
  .version    = "1.0.0",
  .date       = "2011.11.10",
  .maintainer = "RaodNarrows LLC",
  .license    = "(C) 2010 RoadNarrows LLC. All rights reserved."
};


/*!
 * \ingroup bsmod_mousebot_srv
 * \defgroup bsmod_mousebot_srv_tunes Default Tunables
 *
 * \h_botsense MOUSEBOT Interface Module Tunables
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

#define BSMOD_MOUSEBOT_RD         1     ///< fixed pseudo resource id

static BsVConnHnd_T   CurrVConn;

/*!
 * \brief Module resource control block structure type.
 */
typedef struct
{
  struct Qiks m_MotorControllers;
  bool_t         m_bTrace;                   ///< do [not] trace messages  
} BsModCtlBlk_T;

/*!
 * \brief Request Execution Function Type.
 */
typedef int (*BsModExecFunc_T)(BsVConnHnd_T hndVConn, 
                               BsTid_T tid, 
                               BsModCtlBlk_T  *pCtlBlk, 
                               void *pReq, 
                               void *pRsp);

//
// Data
//
static const char           *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table
static MouseBot_T               *BsModMouseBot;       ///< the one mousebot instance


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Module Utility Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Allocate a new resource control block.
 *
 * \param bTrace    Do [not] trace messages.
 *
 * \return Pointer to new allocated control block.
 */
static BsModCtlBlk_T *bsModCtlBlkNew(bool_t bTrace, struct Qiks *pQik)
{
  BsModCtlBlk_T *pCtlBlk;   // resource control block

  pCtlBlk = NEW(BsModCtlBlk_T);

  pCtlBlk->m_bTrace   = bTrace;

  pCtlBlk->m_MotorControllers = *pQik;

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
  BsModCtlBlk_T   *pCtlBlk;                     // resource control block
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
  if( pReq != NULL )
  {
    n = BsMouseBotUnpackMsg(eMsgIdReq, bufReq, uReqLen, pReq, pCtlBlk->m_bTrace);

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
  n = BsMouseBotPackMsg(eMsgIdRsp, pRsp, BSPROXY_BUF_BODY(bufRsp),
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
  BsModCtlBlk_T  *pCtlBlk;                      // resource control block
  int             n;                            // number of bytes/return code

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
  if( pReq != NULL )
  {
    n = BsMouseBotUnpackMsg(eMsgIdReq, bufReq, uReqLen, pReq, pCtlBlk->m_bTrace);

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
static int bsModMouseBotExecSetMotorSpeeds(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk_T *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsMouseBotReqSetMotorSpeeds_T *pMsgReq =(BsMouseBotReqSetMotorSpeeds_T *)pReq;
  int                       rc;             // return code
  
  // execute client request.
  rc = QikUpdateMotorSpeeds( pMsgReq->m_speedleft, 
                             pCtlBlk->m_MotorControllers.m_fdQiks, 
                             0 );

  // check execution return code
  if( rc < 0 )
  {
    QikEStop( pCtlBlk->m_MotorControllers.m_fdQiks );

    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "QikUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedleft, pCtlBlk->m_MotorControllers.m_fdQiks, 0);
    
    return -BS_ECODE_NO_EXEC;
  }

  rc = QikUpdateMotorSpeeds( pMsgReq->m_speedright, pCtlBlk->m_MotorControllers.m_fdQiks, 1);

  // check execution return code
  if( rc < 0 )
  {
    QikEStop(pCtlBlk->m_MotorControllers.m_fdQiks);
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "QikUpdateMotorSpeeds( %d, %d, %d).",
         pMsgReq->m_speedright, pCtlBlk->m_MotorControllers.m_fdQiks, 1);
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
static int bsModMouseBotExecAlterBrake(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk_T *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsMouseBotReqAlterBrake_T  *pMsgReq = (BsMouseBotReqAlterBrake_T *)pReq;
  int                       rc;             // return code
#if 0
  // execute client request.
  rc = QikAlterBraking( pMsgReq->m_brakeFL, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(50000);
    rc = QikAlterBraking( pMsgReq->m_brakeFL, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                0 );
    if(rc < 0) {
      QikEStop( pCtlBlk->m_MotorControllers.m_fdQiks);

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "QikAlterBraking Front Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = QikAlterBraking( pMsgReq->m_brakeFR, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(50000);
    rc = QikAlterBraking( pMsgReq->m_brakeFR, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                1 );
    if(rc < 0) {
      QikEStop( pCtlBlk->m_MotorControllers.m_fdQiks );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "QikAlterBraking Front Right.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

#endif
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
static int bsModMouseBotExecAlterSlew(BsVConnHnd_T hndVConn,
                                     BsTid_T        uTid,
                                     BsModCtlBlk_T *pCtlBlk,
                                     void          *pReq,
                                     void          *pRsp)
{
  BsMouseBotReqAlterSlew_T  *pMsgReq = (BsMouseBotReqAlterSlew_T *)pReq;
  int                       rc;             // return code
#if 0 
  // execute client request.
  rc = QikAlterSlew( pMsgReq->m_slewFL, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                0 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(5000);
    rc = QikAlterSlew( pMsgReq->m_slewFL, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                0 );
    if(rc < 0) {
      QikEStop( pCtlBlk->m_MotorControllers.m_fdQiks );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "QikAlterSlew Front Left.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }

  // execute client request.
  rc = QikAlterSlew( pMsgReq->m_slewFR, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                1 );

  // check execution return code
  if( rc < 0 )
  {
    usleep(5000);
    rc = QikAlterSlew( pMsgReq->m_slewFR, 
                                pCtlBlk->m_MotorControllers.m_fdQiks, 
                                1 );
    if(rc < 0) {
      QikEStop( pCtlBlk->m_MotorControllers.m_fdQiks );

      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
          "QikAlterSlew Front Right.");
    
      return -BS_ECODE_NO_EXEC;
    }
  }
#endif

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
static int bsModMouseBotExecStop(BsVConnHnd_T hndVConn,
                           BsTid_T        uTid,
                           BsModCtlBlk_T *pCtlBlk,    
                           void          *pReq,
                           void          *pRsp)
{
  int   rc = 0; // return code

  //
  // Execute client request.
  //
  QikEStop(pCtlBlk->m_MotorControllers.m_fdQiks);


  // todo: get error code from estop
  // check operation return code
  if( rc < 0 )
  {
    BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_NO_EXEC,
        "Failed to shutdown MouseBot, You're in trouble.");
    return -BS_ECODE_NO_EXEC;
  }

  return BS_OK;
}

static int bsModMouseBotExecAlive(BsVConnHnd_T hndVConn,
                           BsTid_T        uTid,
                           BsModCtlBlk_T *pCtlBlk,    
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
        "Failed to shutdown MouseBot, You're in trouble.");
    return -BS_ECODE_NO_EXEC;
  }

  return BS_OK;
}

// ---------------------------------------------------------------------------
// Exported Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Initialize the MouseBot module.
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
  // initialize MouseBot - MouseBot connection with defaults
#if 0
  if( (BsModMouseBot = mousebotInit()) == NULL )
  {
    LOGERROR("%s(): %s(ecode=%d): mousebotInit() failure, cannot intialize.",
        LOGFUNCNAME, bsStrError(BS_ECODE_NO_EXEC), BS_ECODE_NO_EXEC);
    return -BS_ECODE_NO_EXEC;
  }
#endif

  BsModUri = new_strdup(sModUri);

  // save bsProxy server callbacks
  BsModCallbacks = pCallbacks;

  // create module resource table
  BsModRsrcTbl = BsModCallbacks->m_cbModRsrcTblNew(BSMOD_MAX_HANDLES);

  return BS_OK;
}

/*!
 * \brief Exit the MouseBot module.
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
      QikClose(&pCtlBlk->m_MotorControllers.m_fdQiks);
      bsModCtlBlkDelete(pCtlBlk);
    }
  }

  // free resource table
  BsModCallbacks->m_cbModRsrcTblDelete(BsModRsrcTbl);

  delete((char *)BsModUri);

}

/*!
 * \brief Open a proxied MouseBot robot and associate with the given handle.
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
  struct Qiks Qik;
  BsMouseBotReqOpenParams_T openParams;
  BsModCtlBlk_T    *pCtlBlk;  // resource control block

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
  if( strcmp(sDevUri, BS_MOUSEBOT_DEV_NAME) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_DEV,
        "device URI \"%s\" != required \"%s\".", sDevUri, BS_MOUSEBOT_DEV_NAME);
    return -BS_ECODE_NO_DEV;
  }

  // Only one instance of the robot is supported per platform, so no device
  // need operations such as open() are done.
  BsMouseBotUnpackReqOpenParams( argbuf, uArgSize, &openParams , bTrace);
  
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

  rc = QikOpenConnection(&(Qik.m_fdQiks));
  if(rc < 0) {
    usleep(10000);
    LOGDIAG2("Failed to open front motor controller... Trying again.");
    rc = QikOpenConnection(&(Qik.m_fdQiks));
    if(rc < 0) {
      LOGDIAG2("Unable to open connection to front motor controller.");
    }
  }

  // allocate a new control block and initialize
  pCtlBlk = bsModCtlBlkNew(bTrace, &Qik);

  // add to module resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  CurrVConn = hndVConn;

  LOGDIAG2("VConn=%d: connection open to %s.", hndVConn, sDevUri);

  // fixed resource descriptor
  return BSMOD_MOUSEBOT_RD;
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
  BsModCtlBlk_T  *pCtlBlk;  // resource control block

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

  QikEStop(pCtlBlk->m_MotorControllers.m_fdQiks);
  usleep(20000);
  QikClose(pCtlBlk->m_MotorControllers.m_fdQiks);

  bsModCtlBlkDelete(pCtlBlk);

  LOGDIAG2("VConn=%d: connection closed.", hndVConn);

  return BS_OK;
}

/*!
 * \brief Service a proxied MouseBot robot request.
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
  BsModCtlBlk_T  *pCtlBlk;  // resource control block
  void           *pReq;     // allocated request structure (if any)
  void           *pRsp;     // allocated response structure (if any)
  int             rc;       // return code

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

  pReq = NULL;
  pRsp = NULL;

  // service specific request
  switch( uMsgIdReq )
  {
    case BsMouseBotMsgIdReqSetMotorSpeeds:
      pReq = NEW(BsMouseBotReqSetMotorSpeeds_T);
      rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, pReq,
                        bsModMouseBotExecSetMotorSpeeds);
      break;

/*
    case BsMouseBotMsgIdReqAlterBrake:
      pReq = NEW(BsMouseBotReqAlterBrake_T);
      rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, pReq,
                        bsModMouseBotExecAlterBrake);
      break;
    case BsMouseBotMsgIdReqGetMotorSpeeds:
      pRsp = NEW(BsMouseBotRspGetMotorSpeeds_T);
      rc = bsModDoReq(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                      bsModMouseBotExecGetSpeeds,
                      BsMouseBotMsgIdRspGetMotorSpeeds, pRsp);
      break;
*/

    case BsMouseBotMsgIdReqStop:
      rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                        bsModMouseBotExecStop);
      break;

    case BsMouseBotMsgIdReqAlive:
      rc = bsModDoReqOk(hndVConn, uTid, uMsgIdReq, bufReq, uReqLen, NULL,
                        bsModMouseBotExecAlive);
      break;

    default:
      BSMOD_SEND_ERROR_RSP(BsModCallbacks, hndVConn, uTid, BS_ECODE_UNKNOWN_REQ,
          "MsgId=%u.", uMsgIdReq);
      rc = -BS_ECODE_UNKNOWN_REQ;
      break;
  }

  delete(pReq);
  delete(pRsp);

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
  BsModCtlBlk_T  *pCtlBlk;  // resource control block

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
