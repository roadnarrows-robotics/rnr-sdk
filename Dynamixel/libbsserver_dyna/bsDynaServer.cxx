////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Module:    bsDyna
//
// Library:   libbsserver_dyna
//
// File:      bsDynaServer.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief \h_botsense bsProxy server plug-in DLL Dynamixel module.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
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
#include "botsense/bsDyna.h"
#include "botsense/bsDynaMsgs.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Module static information.
 */
static BsModInfo_T BsModInfo =
{
  BS_DYNA_SERVER_MOD,
  "The proxied Dynamixel servo chain.",
  "1.1.0",
  "2015.01.09",
  "RaodNarrows LLC",
  "(C) 2012-2015 RoadNarrows LLC. All rights reserved."
};


/*!
 * \ingroup bsmod_dyna_srv
 * \defgroup bsmod_dyna_srv_tunes Default Tunables
 *
 * \h_botsense Dynamixel Interface Module Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */

/*!
 * \brief Maximum number of simultaneous virtual connections supported by
 * this module.
 */
#define BSMOD_MAX_HANDLES     8

/*! \} */

//
// Data
//
static char                 *BsModUri;        ///< module canonical name
static const BsModProxyCb_T *BsModCallbacks;  ///< module to bsProxy callbacks
static BsModRsrcTbl_T       *BsModRsrcTbl;    ///< module resource table

C_DECLS_END

/*!
 * \brief Log Interface Module Dynamixel Error.
 *
 * \param hndVConn  Virtual connection handle.
 * \param ecode     Dynamixel error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_LOG_DYNA_ERROR(hndVConn, ecode, efmt, ...) \
  BSMOD_LOG_ERROR(hndVConn, BS_ECODE_NO_EXEC, "%s(dyna_ecode=%d): " efmt, \
      DynaStrError(ecode), (ecode>=0? ecode: -ecode), ##__VA_ARGS__)

/*!
 * \brief Log Dynamixel Error and Send Error Response.
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
  BsModCtlBlk(DynaComm *pDynaComm, bool bTrace)
  {
    m_pDynaComm = pDynaComm;
    m_bTrace    = bTrace;
  }

  /*!
   * \brief Default destructor.
   */
  ~BsModCtlBlk()
  {
    delete m_pDynaComm;
  }

  DynaComm   *m_pDynaComm;    ///< communiction object
  bool_t      m_bTrace;       ///< do [not] trace messages 
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
    m_eMsgIdReq = BsDynaMsgIdNone;
    m_eMsgIdRsp = BsDynaMsgIdNone;
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
    if( (n = ExecThis(hndVConn, uTid, pCtlBlk->m_pDynaComm)) != BS_OK )
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
  BsDynaMsgId_T   m_eMsgIdReq;    ///< request message id
  BsDynaMsgId_T   m_eMsgIdRsp;    ///< response message id
  void           *m_pReqMsg;      ///< allocated request message
  void           *m_pRspMsg;      ///< allocated response message

  /*!
   * \brief Request-specific request execution abstract function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn,
                       BsTid_T      uTid,
                       DynaComm    *pDynaComm) = 0;

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
    return BsDynaUnpackMsg(m_eMsgIdReq, bufReq, uReqLen, m_pReqMsg, bTrace);
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
    return BsDynaPackMsg(m_eMsgIdRsp, m_pRspMsg, bufRsp, uRspSize, bTrace);
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
    if( (n = ExecThis(hndVConn, uTid, pCtlBlk->m_pDynaComm)) != BS_OK )
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
// BsExecReqSetBaudRate Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Set Dynamixel Bus baud rate request execution class.
 */
class BsExecReqSetBaudRate : public BsExecReqOk
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqSetBaudRate()
  {
    m_eMsgIdReq = BsDynaMsgIdReqSetBaudRate;
    m_pReqMsg   = new BsDynaReqSetBaudRate_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqSetBaudRate()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqSetBaudRate_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute set baud rate request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqSetBaudRate_T *pReq = (BsDynaReqSetBaudRate_T *)m_pReqMsg;
    int                     rc;

    rc = pDynaComm->SetBaudRate((int)pReq->m_baudrate);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "SetBaudRate(servo_id=%d)", (int)pReq->m_baudrate);
      return rc;
    }

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqRead8 Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Read 8 bits request execution class.
 */
class BsExecReqRead8 : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqRead8()
  {
    m_eMsgIdReq = BsDynaMsgIdReqRead8;
    m_eMsgIdRsp = BsDynaMsgIdRspRead8;
    m_pReqMsg   = new BsDynaReqRead8_T;
    m_pRspMsg   = new BsDynaRspRead8_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqRead8()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqRead8_T *)m_pReqMsg;
    }

    if( m_pRspMsg != NULL )
    {
      delete (BsDynaRspRead8_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute read 8-bit value request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqRead8_T *pReq = (BsDynaReqRead8_T *)m_pReqMsg;
    BsDynaRspRead8_T *pRsp = (BsDynaRspRead8_T *)m_pRspMsg;
    byte_t            byVal;
    byte_t            byAlarms;
    int               rc;

    rc = pDynaComm->Read8((int)pReq->m_servo_id, pReq->m_addr, &byVal);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "Read8(servo_id=%d,addr=0x%02x)",
          (int)pReq->m_servo_id, (uint_t)pReq->m_addr);
      return rc;
    }

    pRsp->m_alarms  = (byte_t)pDynaComm->GetAlarms();
    pRsp->m_val     = byVal;

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqRead16 Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Read 16 bits request execution class.
 */
class BsExecReqRead16 : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqRead16()
  {
    m_eMsgIdReq = BsDynaMsgIdReqRead16;
    m_eMsgIdRsp = BsDynaMsgIdRspRead16;
    m_pReqMsg   = new BsDynaReqRead16_T;
    m_pRspMsg   = new BsDynaRspRead16_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqRead16()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqRead16_T *)m_pReqMsg;
    }

    if( m_pRspMsg != NULL )
    {
      delete (BsDynaRspRead16_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute read 16-bit value request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqRead16_T  *pReq = (BsDynaReqRead16_T *)m_pReqMsg;
    BsDynaRspRead16_T  *pRsp = (BsDynaRspRead16_T *)m_pRspMsg;
    ushort_t            huVal;
    byte_t              byAlarms;
    int                 rc;

    rc = pDynaComm->Read16((int)pReq->m_servo_id, pReq->m_addr, &huVal);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "Read16(servo_id=%d,addr=0x%02x)",
          (int)pReq->m_servo_id, (uint_t)pReq->m_addr);
      return rc;
    }

    pRsp->m_alarms  = (byte_t)pDynaComm->GetAlarms();
    pRsp->m_val     = huVal;

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqWrite8 Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Write 8 bits request execution class.
 */
class BsExecReqWrite8 : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqWrite8()
  {
    m_eMsgIdReq = BsDynaMsgIdReqWrite8;
    m_eMsgIdRsp = BsDynaMsgIdRspWrite8;
    m_pReqMsg   = new BsDynaReqWrite8_T;
    m_pRspMsg   = new BsDynaRspWrite8_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqWrite8()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqWrite8_T *)m_pReqMsg;
    }

    if( m_pRspMsg != NULL )
    {
      delete (BsDynaRspWrite8_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute write 8-bit value request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqWrite8_T *pReq = (BsDynaReqWrite8_T *)m_pReqMsg;
    BsDynaRspWrite8_T *pRsp = (BsDynaRspWrite8_T *)m_pRspMsg;
    byte_t            byAlarms;
    int               rc;

    rc = pDynaComm->Write8((int)pReq->m_servo_id, pReq->m_addr, pReq->m_val);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "Write8(servo_id=%d,addr=0x%02x,val=0x%04x)",
          (int)pReq->m_servo_id, (uint_t)pReq->m_addr, (uint_t)pReq->m_val);
      return rc;
    }

    pRsp->m_alarms  = (byte_t)pDynaComm->GetAlarms();

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqWrite16 Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Write 16 bits request execution class.
 */
class BsExecReqWrite16 : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqWrite16()
  {
    m_eMsgIdReq = BsDynaMsgIdReqWrite16;
    m_eMsgIdRsp = BsDynaMsgIdRspWrite16;
    m_pReqMsg   = new BsDynaReqWrite16_T;
    m_pRspMsg   = new BsDynaRspWrite16_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqWrite16()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqWrite16_T *)m_pReqMsg;
    }

    if( m_pRspMsg != NULL )
    {
      delete (BsDynaRspWrite16_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute write 16-bit value request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqWrite16_T *pReq = (BsDynaReqWrite16_T *)m_pReqMsg;
    BsDynaRspWrite16_T *pRsp = (BsDynaRspWrite16_T *)m_pRspMsg;
    byte_t              byAlarms;
    int                 rc;

    rc = pDynaComm->Write16((int)pReq->m_servo_id, pReq->m_addr, pReq->m_val);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "Write16(servo_id=%d,addr=0x%02x,val=0x%04x)",
          (int)pReq->m_servo_id, (uint_t)pReq->m_addr, (uint_t)pReq->m_val);
      return rc;
    }

    pRsp->m_alarms  = (byte_t)pDynaComm->GetAlarms();

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqSyncWrite Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Synchronously write values to servos request execution class.
 */
class BsExecReqSyncWrite : public BsExecReqOk
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqSyncWrite()
  {
    m_eMsgIdReq = BsDynaMsgIdReqSyncWrite;
    m_pReqMsg   = new BsDynaReqSyncWrite_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqSyncWrite()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqSyncWrite_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute synchronous request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqSyncWrite_T *pReq = (BsDynaReqSyncWrite_T *)m_pReqMsg;
    DynaSyncWriteTuple_T  tuples[DYNA_ID_NUMOF];
    uint_t                uAddr;
    uint_t                uDataSize;
    uint_t                uCount;
    uint_t                i;
    int                   rc;

    uAddr     = (uint_t)pReq->m_addr;
    uDataSize = (uint_t)pReq->m_data_size;
    uCount    = (uint_t)pReq->m_tuples.m_count;

    if( (uCount == 0) || (uCount > DYNA_ID_NUMOF) )
    {
      rc = -DYNA_ECODE_BAD_VAL;
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "SyncWrite(ucount=%u)", uCount);
      return rc;
    }

    for(i=0; i<uCount; ++i)
    {
      tuples[i].m_nServoId = (int)pReq->m_tuples.u.m_buf[i].m_servo_id;
      tuples[i].m_uVal     = (uint_t)pReq->m_tuples.u.m_buf[i].m_val;
    }

    rc = pDynaComm->SyncWrite(uAddr, uDataSize, tuples, uCount);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "SyncWrite(addr=0x%02x,data_size=%u,...,count=%u)",
          uAddr, uDataSize, uCount);
      return rc;
    }

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqPing Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Ping servo request execution class.
 */
class BsExecReqPing : public BsExecReq
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqPing()
  {
    m_eMsgIdReq = BsDynaMsgIdReqPing;
    m_eMsgIdRsp = BsDynaMsgIdRspPing;
    m_pReqMsg   = new BsDynaReqPing_T;
    m_pRspMsg   = new BsDynaRspPing_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqPing()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqPing_T *)m_pReqMsg;
    }

    if( m_pRspMsg != NULL )
    {
      delete (BsDynaRspPing_T *)m_pRspMsg;
    }
  };

protected:
  /*!
   * \brief Execute ping servo request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqPing_T  *pReq = (BsDynaReqPing_T *)m_pReqMsg;
    BsDynaRspPing_T  *pRsp = (BsDynaRspPing_T *)m_pRspMsg;

    pRsp->m_pong = pDynaComm->Ping((int)pReq->m_servo_id);

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqReset Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Reset servo request execution class.
 */
class BsExecReqReset : public BsExecReqOk
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqReset()
  {
    m_eMsgIdReq = BsDynaMsgIdReqReset;
    m_pReqMsg   = new BsDynaReqReset_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqReset()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqReset_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute ping servo request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqReset_T *pReq = (BsDynaReqReset_T *)m_pReqMsg;
    int               rc;

    rc = pDynaComm->Reset((int)pReq->m_servo_id);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "Reset(servo_id=%d)", (int)pReq->m_servo_id);
      return rc;
    }

    return BS_OK;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// BsExecReqSetHalfDuplexCtl Class
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Set Dynamixel Bus baud rate request execution class.
 */
class BsExecReqSetHalfDuplexCtl : public BsExecReqOk
{
public:
  /*!
   * \brief Default constructor.
   */
  BsExecReqSetHalfDuplexCtl()
  {
    m_eMsgIdReq = BsDynaMsgIdReqSetHalfDuplexCtl;
    m_pReqMsg   = new BsDynaReqSetHalfDuplexCtl_T;
  };

  /*!
   * \brief Default destructor.
   */
  ~BsExecReqSetHalfDuplexCtl()
  {
    if( m_pReqMsg != NULL )
    {
      delete (BsDynaReqSetHalfDuplexCtl_T *)m_pReqMsg;
    }
  };

protected:
  /*!
   * \brief Execute set half-duplex control signal request.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param pDynaComm Dynamixel communication object.
   *
   * \copydoc doc_return_bs_std
   */
  virtual int ExecThis(BsVConnHnd_T hndVConn, BsTid_T uTid, DynaComm *pDynaComm)
  {
    BsDynaReqSetHalfDuplexCtl_T *pReq = (BsDynaReqSetHalfDuplexCtl_T*)m_pReqMsg;
    int                     rc;

    rc = pDynaComm->SetHalfDuplexCtl((int)pReq->m_signal);

    if( rc < 0 )
    {
      BSMOD_SEND_DYNA_ERROR_RSP(hndVConn, uTid, rc,
          "SetHalfDuplexCtl(servo_id=%d)", (int)pReq->m_signal);
      return rc;
    }

    return BS_OK;
  }
};


// ---------------------------------------------------------------------------
// Exported (C) Interface 
// ---------------------------------------------------------------------------

C_DECLS_BEGIN

/*!
 * \brief Initialize the Dynamixel module.
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
 * \brief Exit the Dynamixel module.
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
 * \brief Open a proxied Dynamixel servo chain and associate with the given
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
  DynaComm             *pDynaComm;
  BsDynaReqOpenArgs_T   devcfg;
  BsModCtlBlk          *pCtlBlk;
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
  
  // Check if the resource in resource table is not already in-use.
  else if( BSMOD_RSRC_IS_INUSE(BsModRsrcTbl, hndVConn) )
  {
    BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BUSY, 
        "Module virtual connection handle already in use");
    return -BS_ECODE_BUSY;
  }

  // unpack device configuration arguments submessage
  rc = BsDynaUnpackReqOpenArgs(argbuf, uArgSize, &devcfg, bTrace);

  if( rc < 0 )
  {
    BSMOD_LOG_NMERROR(hndVConn, rc,
        "Unpacking failed on BsDynaReqOpenArgs submessage");
    return -BS_ECODE_BAD_MSG;
  }

  // open dynamixel serial bus device
  pDynaComm = DynaComm::New(sDevUri, (int)devcfg.m_baudrate);

  if( pDynaComm == NULL )
  {
    BSMOD_LOG_SYSERROR(hndVConn, "Open(uri=%s, baudrate=%u).",
        sDevUri, devcfg.m_baudrate);
    return -BS_ECODE_SYS;
  }

  // allocate a new control block and initialize
  pCtlBlk = new BsModCtlBlk(pDynaComm, bTrace);

  // add to module resource table
  BsModCallbacks->m_cbModRsrcAdd(BsModRsrcTbl, hndVConn, pCtlBlk);

  LOGDIAG2("VConn=%d: Open(dev=%s, baudrate=%u).",
                  hndVConn, sDevUri, devcfg.m_baudrate);

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
 * \brief Service a proxied Dynamixel servo chain request.
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
  BsExecReqSetBaudRate      reqSetBaudRate;
  BsExecReqRead8            reqRead8;
  BsExecReqRead16           reqRead16;
  BsExecReqWrite8           reqWrite8;
  BsExecReqWrite16          reqWrite16;
  BsExecReqSyncWrite        reqSyncWrite;
  BsExecReqPing             reqPing;
  BsExecReqReset            reqReset;
  BsExecReqSetHalfDuplexCtl reqSetHalfDuplexCtl;
  int                       rc;       // return code

  // service specific request
  switch( uMsgIdReq )
  {
    case BsDynaMsgIdReqSetBaudRate:
      rc = reqSetBaudRate.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqRead8:
      rc = reqRead8.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqRead16:
      rc = reqRead16.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqWrite8:
      rc = reqWrite8.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqWrite16:
      rc = reqWrite16.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqSyncWrite:
      rc = reqSyncWrite.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqPing:
      rc = reqPing.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqReset:
      rc = reqReset.Execute(hndVConn, uTid, bufReq, uReqLen);
      break;

    case BsDynaMsgIdReqSetHalfDuplexCtl:
      rc = reqSetHalfDuplexCtl.Execute(hndVConn, uTid, bufReq, uReqLen);
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
