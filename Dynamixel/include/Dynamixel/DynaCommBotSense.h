////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaCommBotSense.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-04 12:38:36 -0700 (Wed, 04 Mar 2015) $
 * $Rev: 3873 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Botsene IP Proxied Dynamixel Bus Communication Class
 * Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _DYNA_COMM_BOTSENSE_H
#define _DYNA_COMM_BOTSENSE_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"


// ---------------------------------------------------------------------------
// Dynamixel BotSense IP Proxied Bus Communications Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief BotSense IP Proxied Dynamixel Bus Communications Class.
 *
 * The host with the direct serial connection acts as the bsProxy server.
 */
class DynaCommBotSense : public DynaComm
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaCommBotSense();

  /*!
   * \brief Initialization constructor.
   *
   * The given proxied serial device is opened at the baud rate on the host of
   * the given BotSense proxy server.
   *
   * \param sSerialDevName    Proxied serial device name.
   * \param nBaudRate         Proxied serial device baud rate.
   * \param sBsProxyHostName  BotSense proxy server host name
   *                          (domain name or IP address).
   * \param nBsProxyIPPort    BotSense proxy server IP port number.
   */
  DynaCommBotSense(const char *sSerialDevName,
                   int         nBaudRate,
                   const char *sBsProxyHostName = BSPROXY_URI_HOSTNAME_DFT,
                   int         nBsProxyIPPort   = BSPROXY_LISTEN_PORT_DFT);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaCommBotSense();

  /*!
   * \brief Get the Dynamixel Bus serial device name.
   *
   * \return Returns name.
   */
  const char *GetSerialDeviceName()
  {
    return m_sSerialDevName;
  }

  /*!
   * \brief Get the BotSense proxy server host name.
   *
   * \return Returns name.
   */
  const char *GetProxyServerHostName()
  {
    return m_sBsProxyHostName;
  }

  /*!
   * \brief Get the BotSense proxy server port.
   *
   * \return Returns port number.
   */
  const int GetProxyServerPort() const
  {
    return m_nBsProxyIPPort;
  }

  /*!
   * \brief Get this BotSense proxy client.
   *
   * \return Returns port number.
   */
  const BsClient_P GetProxyClient() const
  {
    return m_pBsClient;
  }

  /*!
   * \brief Get System-unique resource identifier.
   *
   * \return Resource id.
   */
  virtual int GetResourceId() const
  {
    return (int)m_hndBsVConn;
  }

  /*!
   * \brief Open serial communication to dynamixel bus.
   *
   * The given proxied serial device is opened at the baud rate on the 
   * local host with default port number.
   *
   * \param sSerialDevName    Proxied serial device name.
   * \param nBaudRate         Proxied serial device baud rate.
   *
   * \copydoc doc_std_return
   */
  virtual int Open(const char *sSerialDevName, int nBaudRate)
  {
    DynaCommBotSense(sSerialDevName, nBaudRate, BSPROXY_URI_HOSTNAME_DFT,
                    BSPROXY_LISTEN_PORT_DFT);
  }

  /*!
   * \brief Open serial communication to dynamixel bus.
   *
   * The given proxied serial device is opened at the baud rate on the host of
   * the given BotSense proxy server.
   *
   * \param sSerialDevName    Proxied serial device name.
   * \param nBaudRate         Proxied serial device baud rate.
   * \param sBsProxyHostName  BotSense proxy server host name
   *                          (domain name or IP address).
   * \param sBsProxyHostName  BotSense proxy server host.
   * \param nBsProxyIPPort    BotSense proxy server IP port number.
   *
   * \copydoc doc_std_return
   */
  virtual int Open(const char *sSerialDevName,
                   int         nBaudRate,
                   const char *sBsProxyHostName = BSPROXY_URI_HOSTNAME_DFT,
                   int         nBsProxyIPPort = BSPROXY_LISTEN_PORT_DFT);

  /*!
   * \brief (Re)Open serial communication to dynamixel bus.
   *
   * \copydoc doc_std_return
   */
  virtual int Open();

  /*!
   * \brief Close serial communication to dynamixel bus and connection to
   * BotSense proxy server.
   *
   * \copydoc doc_std_return
   */
  virtual int Close();

  /*!
   * Set the Dynamixel Bus new baud rate.
   *
   * \param nNewBaudRate  New baudrate.
   *
   * \copydoc doc_return_std
   */
  virtual int SetBaudRate(int nNewBaudRate);

  /*!
   * \breif Set Dynamixel Bus half-duplex software control.
   *
   * The Dynamixel 3-wire bus is half-duplex. Hardware may automatically
   * control toggling between transmit and receive (e.g. RoadNarrows
   * DynaUSB dongle). If there is no hardware support, then software must
   * provide the tx/rx toggle functions.
   *
   * \param nSignal     Signal assign to toggle.
   * \param fnEnableTx  Enable transmit function.
   * \param fnEnableRx  Enable receive function.
   *
   * \copydoc doc_return_std
   */
  virtual int SetHalfDuplexCtl(int                nSignal,
                               HalfDuplexTxFunc_T fnEnableTx = NULL,
                               HalfDuplexRxFunc_T fnEnableRx = NULL);

  /*!
   * Enable/disable botsense message tracing.
   *
   * \param bEnable Enable [disable].
   *
   * \copydoc doc_return_std
   */
  virtual int SetMsgTracing(bool bEnabled);

  /*!
   * \brief Read an 8-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read8(int nServoId, uint_t uAddr, byte_t *pVal);

  /*!
   * \brief Write an 8-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param byVal         Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write8(int nServoId, uint_t uAddr, byte_t byVal);

  /*!
   * \brief Read a 16-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read16(int nServoId, uint_t uAddr, ushort_t *pVal);

  /*!
   * \brief Write a 16-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param uhVal         Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write16(int nServoId, uint_t uAddr, ushort_t uhVal);

  /*!
   * \brief Synchronous Write 8/16-bit values to a list of Dynamixel servos.
   *
   * \param uAddr     Servo control table write address.
   * \param uValSize  Value storage size at addtess. 1 or 2 bytes.
   * \param tuples    Array of servo id, write value tuples.
   * \param uCount    Number of tuples.
   *
   * \copydoc doc_std_return
   */
  virtual int SyncWrite(uint_t                uAddr,
                        uint_t                uValSize,
                        DynaSyncWriteTuple_T  tuples[],
                        uint_t                uCount);

  /*!
   * \brief Ping the servo.
   *
   * \param nServoId      Servo id.
   *
   * \return Returns true if the servo responded, else false.
   */
  virtual bool Ping(int nServoId);

  /*!
   * \brief Reset a servo back to default values.
   *
   * \warning All configuration is lost.
   *
   * \param nServoId      Servo id.
   *
   * \copydoc doc_std_return
   */
  virtual int Reset(int nServoId);

protected:
  char         *m_sSerialDevName;   ///< proxied serial device name
  char         *m_sBsProxyHostName; ///< BotSense proxy server domain/IP address
  int           m_nBsProxyIPPort;   ///< BotSense proxy server IP port number
  BsClient_P    m_pBsClient;        ///< BotSense client
  BsVConnHnd_T  m_hndBsVConn;       ///< virtual connection to proxied device
  bool          m_bBsTrace;         ///< do [not] trace messaging
};


#endif // _DYNA_COMM_BOTSENSE_H
