////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaComm.h
//
/*! \file
 *
 * $LastChangedDate: 2015-03-04 12:38:36 -0700 (Wed, 04 Mar 2015) $
 * $Rev: 3873 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Bus Communications Abstract Base Class
 * Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
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

#ifndef _DYNA_COMM_H
#define _DYNA_COMM_H

#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/shm.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaError.h"


// ---------------------------------------------------------------------------
// Dynamixel Bus Communications Abstract Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief Dynamixel Bus Communications Abstract Base Class
 *
 * The DynaComm abstract class provides the I/O template to communicate
 * with Dynamixel servos on a Dynamixel bus.
 */
class DynaComm
{
public:
  /*! \brief Half-duplex control transmit function type. */
  typedef void (*HalfDuplexTxFunc_T)(void *pArg);

  /*! \brief Half-duplex control receive function type. */
  typedef void (*HalfDuplexRxFunc_T)(void *pArg, size_t uNumTxBytes);

  /*!
   * \breif Default constructor.
   */
  DynaComm();

  /*!
   * \breif Initialization constructor.
   *
   * \param sUri        Dynamixel Bus device Uniform Resource Identifier string.
   * \param nBaudRate   Device baud rate.
   */
  DynaComm(const char *sUri, int nBaudRate);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaComm();

  /*!
   * \brief Archetype constructor to create a new Dynamixel bus communication
   * derived instance.
   *
   * The specific DynaComm object created depends on the URI specified.
   * Currently two derived objects are supported:
   * \termblock
   * \term DynaCommSerial \termdata Direct connected serial device. \endterm
   * \term DynaCommBotSense \termdata BotSense proxied serial device. \endterm
   * \endtermblock
   *
   * \par Supported URIs:
   * <tt>/devicepath</tt>\n
   * <tt>botsense://[hostname][:port]/devicepath</tt>\n
   *
   * \param sUri        Uniform Resource Identifier string.
   * \param nBaudRate   Dynamixel bus supported baud rate.
   *
   * \return On succes, returns pointer to the allocated DynaComm derived
   * object.\n
   * On failure, NULL is returned.
   */
  static DynaComm *New(const char *sUri, int nBaudRate);

  /*!
   * Get the Dynamixel Bus Uniform Resource Identifier.
   *
   * \return Returns URI.
   */
  const char *GetDeviceUri() const
  {
    return m_sDevUri;
  }

  /*!
   * Get the current baud rate.
   *
   * \return Return buad rate.
   */
  const int GetBaudRate() const
  {
    return m_nBaudRate;
  }

  /*!
   * \brief Get system-unique resource identifier.
   *
   * \return Resource id.
   */
  virtual int GetResourceId() const = 0;

  /*!
   * \brief Open communication to dynamixel bus.
   *
   * The given serial device is opened at the given baud rate.
   *
   * \param sDevUri    Device Uniform Resource Identifier.
   * \param nBaudRate  Device baud rate.
   *
   * \copydoc doc_std_return
   */
  virtual int Open(const char *sDevUri, int nBaudRate) = 0;

  /*!
   * \brief (Re)Open communication to dynamixel bus.
   *
   * \copydoc doc_std_return
   */
  virtual int Open() = 0;

  /*!
   * \brief Close communication to dynamixel bus.
   *
   * \copydoc doc_std_return
   */
  virtual int Close() = 0;

  /*!
   * \brief Set the Dynamixel Bus new baud rate.
   *
   * \param nNewBaudRate    New baud rate.
   *
   * \copydoc doc_std_return
   */
  virtual int SetBaudRate(int nNewBaudRate) = 0;

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
                               HalfDuplexRxFunc_T fnEnableRx = NULL) = 0;

  /*!
   * \brief Read an 8-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read8(int nServoId, uint_t uAddr, byte_t *pVal) = 0;
                    
  /*!
   * \brief Read an 8-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read8(int nServoId, uint_t uAddr, uint_t *pVal)
  {
    byte_t  val;
    int     rc;

    if( (rc = Read8(nServoId, uAddr, &val)) == DYNA_OK )
    {
      *pVal = (uint_t)val;
    }
    return rc;
  }

  /*!
   * \brief Write an 8-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param byVal         Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write8(int nServoId, uint_t uAddr, byte_t byVal) = 0;

  /*!
   * \brief Write an 8-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param uVal          Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write8(int nServoId, uint_t uAddr, uint_t uVal)
  {
    byte_t  val = (byte_t)uVal;

    return Write8(nServoId, uAddr, val);
  }

  /*!
   * \brief Read a 16-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read16(int nServoId, uint_t uAddr, ushort_t *pVal) = 0;

  /*!
   * \brief Read a 16-bit value from Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param [out] pVal    Value read.
   *
   * \copydoc doc_std_return
   */
  virtual int Read16(int nServoId, uint_t uAddr, uint_t *pVal)
  {
    ushort_t  val;
    int       rc;

    if( (rc = Read16(nServoId, uAddr, &val)) == DYNA_OK )
    {
      *pVal = (uint_t)val;
    }
    return rc;
  }

  /*!
   * \brief Write a 16-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param uhVal         Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write16(int nServoId, uint_t uAddr, ushort_t uhVal) = 0;

  /*!
   * \brief Write a 16-bit value to Dynamixel servo control table.
   *
   * \param nServoId      Servo id.
   * \param uAddr         Servo control table address.
   * \param uVal          Value written.
   *
   * \copydoc doc_std_return
   */
  virtual int Write16(int nServoId, uint_t uAddr, uint_t uVal)
  {
    ushort_t  val = (ushort_t)uVal;

    return Write16(nServoId, uAddr, val);
  }

  /*!
   * \brief Synchronous write 8/16-bit values to a list of Dynamixel servos.
   *
   * \param uAddr     Servo control table write address.
   * \param uValSize  Value storage size of field at addtess. 1 or 2 bytes.
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 2-tuples of type
   *                  (int,uint_t) specifying the servo id and the packed raw
   *                  field value.
   *
   * \copydoc doc_std_return
   */
  virtual int vSyncWrite(uint_t uAddr, uint_t uValSize, uint_t uCount, ...);

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
                        uint_t                uCount) = 0;

  /*!
   * \brief Ping the servo.
   *
   * \param nServoId      Servo id.
   *
   * \return Returns true if the servo responded, else false.
   */
  virtual bool Ping(int nServoId) = 0;

  /*!
   * \brief Reset a servo back to default values.
   *
   * \warning All configuration is lost.
   *
   * \param nServoId      Servo id.
   *
   * \copydoc doc_std_return
   */
  virtual int Reset(int nServoId) = 0;

  /*!
   * \brief Test if Dynamixel Bus is open.
   *
   * \return Returns true or false.
   */
  virtual bool IsOpen()
  {
    return m_bIsOpen;
  }

  /*!
   * \brief Get current alarms.
   *
   * \return Returns alarm bits.
   */
  virtual uint_t GetAlarms()
  {
    return m_uAlarms;
  }

  /*!
   * \brief Clear current alarms.
   */
  virtual void ClearAlarms()
  {
    m_uAlarms = DYNA_ALARM_NONE;
  }

  /*!
   * \brief Get a formatted servo alarms string associated with the alarms.
   *
   * \param uAlarms     Dynamixel servo alarm bits.
   * \param strSep      Separator string between alarm substrings.
   *
   * \return Returns string.
   */
  static std::string GetAlarmsString(const uint_t       uAlarms,
                                     const std::string &strSep="; ");

  /*!
   * \brief Get a formatted servo alarms short string associated with the
   * alarms.
   *
   * \param uAlarms     Dynamixel servo alarm bits.
   * \param strSep      Separator string between alarm substrings.
   *
   * \return Returns string.
   */
  static std::string GetAlarmsShortString(const uint_t      uAlarms,
                                          const std::string &strSep=",");

  /*!
   * \brief Get the Dynamixel Bus status.
   *
   * \return Status.
   */
  virtual uint_t GetBusStatus()
  {
    return m_uBusStatus;
  }

  /*!
   * \brief Get the string describing the Dynamixel servo communication status.
   *
   * \param uBusStatus  Dynamixel bus communication status.
   *
   * \return Returns the appropriate status string.
   */
  static const char *GetBusStatusString(uint_t uBusStatus);

  /*!
   * \brief Map baud rate to Dynamixel baud number.
   *
   * \param nBaudRate    Baud rate.
   *
   * \return Returns baud number on success.\n
   * \copydoc doc_std_ecode
   */
  static int BaudRateToNum(int nBaudRate);

  /*!
   * \brief Map baud number to Dynamixel baud rate.
   *
   * \param nBaudNum    Baud number.
   *
   * \return Returns baud rate on success.\n
   * \copydoc doc_std_ecode
   */
  static int BaudNumToRate(int nBaudRate);

  /*!
   * \brief Get the baud rate associated with the given index.
   *
   * This function can be used to iterate of all supported baud rates.
   *
   * \param nIndex    Zero based index.
   *
   * \return
   * If the index is in range, returns the baud rate at the index.\n
   * Otherwise 0 is returned.
   *
   */
  static int BaudRateAt(int nIndex);

  /*!
   * \brief Get the baud number associated with the given index.
   *
   * This function can be used to iterate of all supported baud rates.
   *
   * \param nIndex    Zero based index.
   *
   * \return
   * If the index is in range, returns the baud number at the index.\n
   * Otherwise 0 is returned.
   *
   */
  static int BaudNumAt(int nIndex);

protected:
  static const key_t  ShmKey;   ///< shared memory key

  char           *m_sDevUri;    ///< dynamixel bus device URI
  int             m_nBaudRate;  ///< baud rate
  bool            m_bIsOpen;    ///< dynamixel bus communication is [not] open
  uint_t          m_uBusStatus; ///< bus comminication status
  uint_t          m_uAlarms;    ///< servo alarms from last I/O operation
  shm_mutex_t     m_mutexComm;  ///< synchonization mutex
};


#endif // _DYNA_COMM_H
