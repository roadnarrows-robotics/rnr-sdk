////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libbsclient_dyna
//
// File:      bsDyna.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief \h_botsense bsProxy client library Dynamixel interface.
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

#ifndef _BSDYNA_H
#define _BSDYNA_H

#include "rnr/rnrconfig.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

#define BS_DYNA_SERVER_MOD  "libbsserver_dyna"    ///< server plugin dll module
#define BS_DYNA_CLIENT_LIB  "libbsclient_dyna"    ///< client app library

/*!
 * \brief Get the Dynamixel message name.
 *
 * Each (virtual connection, message id) 2-tuple provides a unique server
 * mapping that can be used associate a name string to the message
 * (provided the id is valid and an application provides the information).
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 * \param uMsgId    Message id.
 *
 * \return
 * Returns message name if it can be determined. Otherwise returns "unknown".
 */
extern const char *bsDynaGetMsgName(BsClient_P   pBsClient,
                                    BsVConnHnd_T hndVConn,
                                    uint_t       uMsgId);


/*!
 * \brief Request proxy server to establish a virtual connection to the
 * Dynamixel servo chain using the USB2Dynamixel serial dongle.
 *
 * \param pClient     \h_botsense client.
 * \param sDevName    Proxied USB serial device name (e.g. /dev/ttyUSB0).
 * \param nBaudRate   Baud rate.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_bs_ecode
 */
extern int bsDynaOpen(BsClient_P  pBsClient,
                      const char *sDevName,
                      int         nBaudRate,
                      bool_t      bInitTrace);

/*!
 * \brief Request proxy server to close client's proxied Dynamixel servo chain
 * vitual connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_bs_std
 */
extern int bsDynaClose(BsClient_P pBsClient, BsVConnHnd_T hndBsVConn);

/*!
 * \brief Proxied request to set the Dynamixel Bus baud rate.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nBaudRate           New baud rate.
 *
 * \return doc_return_bs_std
 */
extern int bsDynaSetBaudRate(BsClient_P   pBsClient,
                             BsVConnHnd_T hndBsVConn,
                             int          nBaudRate);

/*!
 * \brief Proxied request to read an 8-bit value from a servo's control table.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param uAddr               Control table address.
 * \param [out] pVal          Value read.
 * \param [out] pAlarms       Current servo alarms, if any.
 *
 * \copydoc doc_return_bs_std
 */
extern int bsDynaRead8(BsClient_P     pBsClient,
                       BsVConnHnd_T   hndBsVConn,
                       int            nServoId,
                       uint_t         uAddr,
                       byte_t        *pVal,
                       uint_t        *pAlarms);

/*!
 * \brief Proxied request to write an 8-bit value to a servo's control table.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param uAddr               Control table address.
 * \param [in] byVal          Value to write.
 * \param [out] pAlarms       Current servo alarms, if any.
 *
 * \copydoc doc_return_bs_std
 */
extern int bsDynaWrite8(BsClient_P    pBsClient,
                        BsVConnHnd_T  hndBsVConn,
                        int           nServoId,
                        uint_t        uAddr,
                        byte_t        byVal,
                        uint_t       *pAlarms);

/*!
 * \brief Proxied request to read a 16-bit value from a servo's control table.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param uAddr               Control table address.
 * \param [out] pVal          Value read.
 * \param [out] pAlarms       Current servo alarms, if any.
 *
 * \copydoc doc_return_bs_std
 */
extern int bsDynaRead16(BsClient_P    pBsClient,
                        BsVConnHnd_T  hndBsVConn,
                        int           nServoId,
                        uint_t        uAddr,
                        ushort_t     *pVal,
                        uint_t       *pAlarms);

/*!
 * \brief Proxied request to write a 16-bit value to a servo's control table.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param uAddr               Control table address.
 * \param [in] huVal          Value to write.
 * \param [out] pAlarms       Current servo alarms, if any.
 *
 * \copydoc doc_return_bs_std
 */
extern int bsDynaWrite16(BsClient_P   pBsClient,
                         BsVConnHnd_T hndBsVConn,
                         int          nServoId,
                         uint_t       uAddr,
                         ushort_t     huVal,
                         uint_t      *pAlarms);

/*!
 * \brief Proxied request to synchronously write values to servos.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Handle to virtual connection to close.
 * \param uAddr         Control table address.
 * \param uDataSize     Size (bytes) of data to write.
 * \param tuples        Servo Id, Value 2-tuples
 * \param uCount        Number of 2-tuples
 *
 * \copydoc doc_return_bs_std
 */
extern int bsDynaSyncWrite(BsClient_P           pBsClient,
                           BsVConnHnd_T         hndBsVConn,
                           uint_t               uAddr,
                           uint_t               uDataSize,
                           DynaSyncWriteTuple_T tuples[],
                           uint_t               uCount);

/*!
 * \brief Proxied request to ping a servo.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 *
 * \return Returns true if the servo responded, else false.
 */
extern int bsDynaPing(BsClient_P    pBsClient,
                      BsVConnHnd_T  hndBsVConn,
                      int           nServoId,
                      bool_t       *pPong);

/*!
 * \brief Proxied request to reset a servo to defaults.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 *
 * \return doc_return_bs_std
 */
extern int bsDynaReset(BsClient_P   pBsClient,
                       BsVConnHnd_T hndBsVConn,
                       int          nServoId);

/*!
 * \brief Proxied request to set the half-duplex control signal.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nSignal             Signal number.
 *
 * \return doc_return_bs_std
 */
int bsDynaSetHalfDuplexCtl(BsClient_P    pClient,
                           BsVConnHnd_T  hndVConn,
                           int           nSignal);

#ifndef SWIG
C_DECLS_END
#endif // SWIG

#endif // _BSDYNA_H
