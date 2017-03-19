////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaCommSerial.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Bus Communication over Serial Interface Class
 * Interface.
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

#ifndef _DYNA_COMM_SERIAL_H
#define _DYNA_COMM_SERIAL_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/dxl/dxl.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"


// ---------------------------------------------------------------------------
// Dynamixel Serial Bus Communications Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief Dynamixel Serial Bus Communications Class
 */
class DynaCommSerial : public DynaComm
{
public:
  static const int RetryMax  = 5;      ///< maximum number of I/O retries
  static const int RetryWait = 20000;  ///< microsecond wait between retries

  /*!
   * \brief Half-duplex control signals. 
   *
   * \note Keep out of any potential non-modem signal space
   * (e.g. GPIO 123, etc).
   */
  enum CtlSignal
  {
    // GPIO: [-999, 999]
    CtlSignalMinGPIO  = -999, ///< gpio signal is inverted (tx = low, rx = high)
    CtlSignalMaxGPIO  =  999, ///< gpio signal (tx = high, rx = low)

    // modem
    CtlSignalModemRTS = 1000, ///< request to send 
    CtlSignalModemCTS = 1001, ///< clear to send 

    // other
    CtlSignalCustom   = 1010, ///< custom interface (requires callbacks)
    CtlSignalNone     = 1011  ///< not a signal, but a value to clear signal
  };

  /*!
   * \brief Default constructor.
   */
  DynaCommSerial();

  /*!
   * \brief Initialization constructor.
   *
   * The given serial device is opened at the baud rate.
   *
   * \param sSerialDevName    Serial device name.
   * \param nBaudRate         Serial device baud rate.
   */
  DynaCommSerial(const char *sSerialDevName, int nBaudRate);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaCommSerial();

  /*!
   * Get the Dynamixel Bus serial device name.
   *
   * \return Returns name.
   */
  const char *GetSerialDeviceName() const
  {
    return m_sSerialDevName;
  }

  /*!
   * \brief Get system-unique resource identifier.
   *
   * \return Resource id.
   */
  virtual int GetResourceId() const
  {
    return m_fd;
  }

  /*!
   * \brief Open serial communication to dynamixel bus.
   *
   * The given serial device is opened at the given baud rate.
   *
   * \param sSerialDevName    Serial device name.
   * \param nBaudRate         Serial device baud rate.
   *
   * \copydoc doc_std_return
   */
  virtual int Open(const char *sSerialDevName, int nBaudRate);

  /*!
   * \brief (Re)Open serial communication to dynamixel bus.
   *
   * \copydoc doc_std_return
   */
  virtual int Open();

  /*!
   * \brief Close serial communication to dynamixel bus.
   *
   * \copydoc doc_std_return
   */
  virtual int Close();

  /*!
   * \brief Set the Dynamixel Bus new baud rate.
   *
   * \param nNewBaudRate    New baud rate.
   *
   * \copydoc doc_std_return
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
   * \copydoc doc_std_return
   */
  virtual int SetHalfDuplexCtl(int                nSignal,
                               HalfDuplexTxFunc_T fnEnableTx = NULL,
                               HalfDuplexRxFunc_T fnEnableRx = NULL);

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
   * \param tuples    Array of servo id, write value 2-tuples.
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
  char       *m_sSerialDevName;   ///< serial device name
  int         m_fd;               ///< serial file descriptor
  int         m_nGpioNum;         ///< gpio number
  int         m_fdGpio;           ///< gpio file descriptor
  int         m_nGpioVal;         ///< gpio shadow'ed value
  libdxl::dxl m_dxl;              ///< dxl low-level interface

  /*!
   * \brief Make serial index from serial device name.
   *
   * \par Examples:
   *  /dev/ttyUSB0 \h_leftdblarrow 0\n
   *  COM3 \h_leftdblarrow 3
   *
   * \param sSeriaDevName   Serial device name.
   *
   * \return Returns serial index on success.\n
   * \copydoc doc_std_ecode
   */
  int MakeSerialIndex(const char *sSerialDevName);

  /*!
   * \brief Intialize RTS signalling.
   */
  void InitRTS();

  /*!
   * \brief Enable transmit via RTS signal.
   *
   * \param pArg  Pointer to this.
   */
  static void EnableTxRTS(void *pArg);

  /*!
   * \brief Enable receive via RTS signal.
   *
   * \param pArg          Pointer to this.
   * \param uNumTxBytes   Number of transmit bytes (being) transmitted.
   */
  static void EnableRxRTS(void *pArg, size_t uNumTxBytes);

  /*!
   * \brief Intialize CTS signalling.
   */
  void InitCTS();

  /*!
   * \brief Enable transmit via CTS signal.
   *
   * \param pArg  Pointer to this.
   */
  static void EnableTxCTS(void *pArg);

  /*!
   * \brief Enable receive via CTS signal.
   *
   * \param pArg          Pointer to this.
   * \param uNumTxBytes   Number of transmit bytes (being) transmitted.
   */
  static void EnableRxCTS(void *pArg, size_t uNumTxBytes);

  /*!
   * \brief Intialize GPIO signalling.
   *
   * \param nGpioNum  GPIO number.
   */
  void InitGPIO(int nGpioNum);

  /*!
   * \brief Enable transmit via GPIO signal.
   *
   * \param pArg  Pointer to this.
   */
  static void EnableTxGPIO(void *pArg);

  /*!
   * \brief Enable receive via GPIO signal.
   *
   * \param pArg          Pointer to this.
   * \param uNumTxBytes   Number of transmit bytes (being) transmitted.
   */
  static void EnableRxGPIO(void *pArg, size_t uNumTxBytes);
};


#endif // _DYNA_COMM_SERIAL_H
