////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaCommSerial.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-10 12:42:59 -0600 (Fri, 10 Apr 2015) $
 * $Rev: 3924 $
 *
 * \brief RoadNarrows Dynamixel bus communication over serial interface class.
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

#include <unistd.h>
#include <ctype.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/shm.h"
#include "rnr/log.h"
#include "rnr/serdev.h"
#include "rnr/gpio.h"

#include "Dynamixel/dxl/dxl.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaCommSerial.h"

#include "DynaLibInternal.h"

using namespace std;
using namespace libdxl;

// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// DynaCommSerial Class
// ---------------------------------------------------------------------------

DynaCommSerial::DynaCommSerial() : DynaComm()
{
  m_sSerialDevName  = NULL;
  m_fd              = -1;
  m_nGpioNum        = 0;
  m_fdGpio          = -1;
  m_nGpioVal        = -1;
}

DynaCommSerial::DynaCommSerial(const char *sSerialDevName, int nBaudRate) :
                      DynaComm(sSerialDevName, nBaudRate)
{
  m_sSerialDevName  = newstr(sSerialDevName);
  m_fd              = -1;
  m_nGpioNum        = 0;
  m_fdGpio          = -1;
  m_nGpioVal        = -1;

  Open();
}

DynaCommSerial::~DynaCommSerial()
{
  Close();

  if( m_sSerialDevName != NULL )
  {
    delete[] m_sSerialDevName;
  }
}

int DynaCommSerial::Open(const char *sSerialDevName, int nBaudRate)
{
  Close();

  if( m_sSerialDevName != NULL )
  {
    delete[] m_sSerialDevName;
  }

  m_sSerialDevName  = newstr(sSerialDevName);
  m_nBaudRate       = nBaudRate;

  return Open();
}

int DynaCommSerial::Open()
{
#if 0 // old dxl library
  int nSerialIndex;
  int nBaudNum;
#endif
  int rc;

  Close();

  if( (m_sSerialDevName == NULL) || (*m_sSerialDevName == 0) )
  {
    rc = -DYNA_ECODE_BAD_VAL;
    DYNA_LOG_ERROR(rc, "Unspecified serial device.");
  }

#if 0 // old dxl library
  // make serial index from serial device
  else if( (nSerialIndex = MakeSerialIndex(m_sSerialDevName)) < 0 )
  {
    DYNA_LOG_ERROR(nSerialIndex, "Bad serial device name: \"%s\".",
        m_sSerialDevName);
    rc = nSerialIndex;
  }

  // map baudrate to internal baud enumeration value
  else if( (nBaudNum = BaudRateToNum(m_nBaudRate)) < 0 )
  {
    DYNA_LOG_ERROR(nBaudNum, "%d: Unsupported baudrate.", m_nBaudRate);
    rc = nBaudNum;
  }

  else if( m_dxl.dxl_initialize(nSerialIndex, nBaudNum) == 0 ) 
  {
    rc = -DYNA_ECODE_BAD_DEV;
    DYNA_LOG_ERROR(rc, "Serial device \"%s\": Failed to initalize interface.",
        m_sSerialDevName);
  }
#endif

  else if( m_dxl.open(m_sSerialDevName, m_nBaudRate) == 0 ) 
  {
    rc = -DYNA_ECODE_BAD_DEV;
    DYNA_LOG_ERROR(rc, "Serial device \"%s\": Failed to initalize interface.",
        m_sSerialDevName);
  }

  else
  {
    m_bIsOpen = true;
    m_fd = m_dxl.getFd();
    rc = DYNA_OK;
    LOGDIAG3("Dynamixel bus communication opened on \"%s\"@%d.",
        m_sSerialDevName, m_nBaudRate);
  }

  return rc;
}

int DynaCommSerial::Close()
{
  if( m_bIsOpen )
  {
    m_dxl.close();
    m_fd = -1;
    m_bIsOpen = false;
  }
  return DYNA_OK;
}

int DynaCommSerial::SetBaudRate(int nNewBaudRate)
{
  int   rc;   // return code

  // check baud rate against supported
  if( (rc = BaudRateToNum(nNewBaudRate)) < 0 )
  {
    DYNA_LOG_ERROR(rc, "%d: Unsupported baudrate.", nNewBaudRate);
    return rc;
  }
  
  // already open, adjust serial parameters
  if( m_bIsOpen )
  {
    if( !m_dxl.setBaudRate(nNewBaudRate) )
    {
      rc = -DYNA_ECODE_SYS;
      DYNA_LOG_SYS_ERROR(rc,
          "Serial device \"%s\": Failed to set baud rate %d.",
          m_sSerialDevName, nNewBaudRate);
      return rc;
    }
  }

  m_nBaudRate = nNewBaudRate;

  return DYNA_OK;
}

int DynaCommSerial::SetHalfDuplexCtl(int                          nSignal,
                                     DynaComm::HalfDuplexTxFunc_T fnEnableTx,
                                     DynaComm::HalfDuplexRxFunc_T fnEnableRx)
{
  switch( nSignal )
  {
    case CtlSignalModemRTS: // serial request to send
      InitRTS();
      SerDevSetHwFlowControl(m_fd, true);
      m_dxl.setHalfDuplexCallbacks(DynaCommSerial::EnableTxRTS,
                                   DynaCommSerial::EnableRxRTS,
                                   this);
      break;
    case CtlSignalModemCTS: // serial clear to send
      InitCTS();
      SerDevSetHwFlowControl(m_fd, true);
      m_dxl.setHalfDuplexCallbacks(DynaCommSerial::EnableTxCTS,
                                   DynaCommSerial::EnableRxCTS,
                                   this);
      break;
    case CtlSignalNone:     // clear callbacks
      m_dxl.setHalfDuplexCallbacks(NULL, NULL, NULL);
      break;
    case CtlSignalCustom:   // custom signal
      m_dxl.setHalfDuplexCallbacks(fnEnableTx, fnEnableRx, this);
      break;
    default:                // gpio signal
      InitGPIO(nSignal);
      m_dxl.setHalfDuplexCallbacks(DynaCommSerial::EnableTxGPIO,
                                   DynaCommSerial::EnableRxGPIO,
                                   this);
      break;
  }

  return DYNA_OK;
}

int DynaCommSerial::Read8(int nServoId, uint_t uAddr, byte_t *pVal)
{
  int   nTries = 0;
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  do
  {
    *pVal = (byte_t)m_dxl.readByte(nServoId, (int)uAddr);

    m_uBusStatus = (uint_t)m_dxl.getResult();
   
    if( m_uBusStatus == DXL_COMM_RXSUCCESS )
    {
      m_uAlarms = (uint_t)m_dxl.getRxPacketErrBits();
      DYNA_LOG_SERVO_ALARMS(nServoId, m_uAlarms);
      rc = DYNA_OK;
    }
    else
    {
      rc = -DynaMapDxlToEcode(m_uBusStatus);
      usleep(RetryWait);
    }
  } while( (rc == -DYNA_ECODE_RX_TIMEOUT) && (++nTries < RetryMax) );

  shm_mutex_unlock(&m_mutexComm);

  if (rc < 0 )
  {
    DYNA_LOG_ERROR(rc, "Read8(%d, 0x%02x)", nServoId, uAddr);
  }

  return rc;
}

int DynaCommSerial::Write8(int nServoId, uint_t uAddr, byte_t byVal)
{
  int   nTries = 0;
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  do
  {
    m_dxl.writeByte(nServoId, (int)uAddr, (int)byVal);

    m_uBusStatus = (uint_t)m_dxl.getResult();
   
    if( m_uBusStatus == DXL_COMM_RXSUCCESS )
    {
      m_uAlarms = (uint_t)m_dxl.getRxPacketErrBits();
      DYNA_LOG_SERVO_ALARMS(nServoId, m_uAlarms);
      rc = DYNA_OK;
    }
    else
    {
      rc = -DynaMapDxlToEcode(m_uBusStatus);
      usleep(RetryWait);
    }
  } while( (rc == -DYNA_ECODE_RX_TIMEOUT) && (++nTries < RetryMax) );

  shm_mutex_unlock(&m_mutexComm);

  if (rc < 0 )
  {
    DYNA_LOG_ERROR(rc, "Write8(%d, 0x%02x, 0x%02x)", nServoId, uAddr, byVal);
  }

  return rc;
}

int DynaCommSerial::Read16(int nServoId, uint_t uAddr, ushort_t *pVal)
{
  int   nTries = 0;
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  do
  {
    *pVal = (ushort_t)m_dxl.readWord(nServoId, (int)uAddr);

    m_uBusStatus = (uint_t)m_dxl.getResult();

    if( m_uBusStatus == DXL_COMM_RXSUCCESS )
    {
      m_uAlarms = (uint_t)m_dxl.getRxPacketErrBits();
      DYNA_LOG_SERVO_ALARMS(nServoId, m_uAlarms);
      rc = DYNA_OK;
    }
    else
    {
      rc = -DynaMapDxlToEcode(m_uBusStatus);
      usleep(RetryWait);
    }
  } while( (rc == -DYNA_ECODE_RX_TIMEOUT) && (++nTries < RetryMax) );

  shm_mutex_unlock(&m_mutexComm);

  if (rc < 0 )
  {
    DYNA_LOG_ERROR(rc, "Read16(%d, 0x%02x)", nServoId, uAddr);
  }

  return rc;
}

int DynaCommSerial::Write16(int nServoId, uint_t uAddr, ushort_t uhVal)
{
  int   nTries = 0;
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  do
  {
    m_dxl.writeWord(nServoId, (int)uAddr, (int)uhVal);

    m_uBusStatus = (uint_t)m_dxl.getResult();

    if( m_uBusStatus == DXL_COMM_RXSUCCESS )
    {
      m_uAlarms = (uint_t)m_dxl.getRxPacketErrBits();
      DYNA_LOG_SERVO_ALARMS(nServoId, m_uAlarms);
      rc = DYNA_OK;
    }
    else
    {
      rc = -DynaMapDxlToEcode(m_uBusStatus);
      usleep(RetryWait);
    }
  } while( (rc == -DYNA_ECODE_RX_TIMEOUT) && (++nTries < RetryMax) );

  shm_mutex_unlock(&m_mutexComm);

  if (rc < 0 )
  {
    DYNA_LOG_ERROR(rc, "Write16(%d, 0x%02x, 0x%04x)", nServoId, uAddr, uhVal);
  }

  return rc;
}

int DynaCommSerial::SyncWrite(uint_t                uAddr,
                              uint_t                uValSize,
                              DynaSyncWriteTuple_T  tuples[],
                              uint_t                uCount)
{
  int i;
  int j;
  int nTries = 0;
  int rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  //
  // packet header
  //
  j = 0;
  m_dxl.setTxPacketId(DXL_BROADCAST_ID);              // broadcast id
  m_dxl.setTxPacketInstruction(DXL_INST_SYNC_WRITE);  // instruction
  m_dxl.setTxPacketParameter(j++, (int)uAddr);        // write address
  m_dxl.setTxPacketParameter(j++, (int)uValSize);     // value size at address

  //
  // servo_id, val_lsb[, val_msb]
  //
  for(i=0; i<(int)uCount; ++i)
  {
    m_dxl.setTxPacketParameter(j++, tuples[i].m_nServoId);

    if( uValSize == 1 )
    {
      m_dxl.setTxPacketParameter(j++, m_dxl.getLowByte((int)tuples[i].m_uVal));
    }
    else
    {
      m_dxl.setTxPacketParameter(j++, m_dxl.getLowByte((int)tuples[i].m_uVal));
      m_dxl.setTxPacketParameter(j++, m_dxl.getHighByte((int)tuples[i].m_uVal));
    }
  }

  // packet length in header
  m_dxl.setTxPacketLength(j+2);

  do
  {
    // send sync packet
    m_dxl.txrxPacket();
  
    m_uBusStatus = (uint_t)m_dxl.getResult();

    if( m_uBusStatus == DXL_COMM_RXSUCCESS )
    {
      rc = DYNA_OK;
    }
    else
    {
      rc = -DynaMapDxlToEcode(m_uBusStatus);
      usleep(RetryWait);
    }
  } while( (rc == -DYNA_ECODE_RX_TIMEOUT) && (++nTries < RetryMax) );

  shm_mutex_unlock(&m_mutexComm);

  if (rc < 0 )
  {
    DYNA_LOG_ERROR(rc, "SyncWrite(0x%02x, %d, tuples, %d)",
        uAddr, uValSize, uCount);
  }

  return rc;
}

bool DynaCommSerial::Ping(int nServoId)
{
  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  m_dxl.ping(nServoId);

  shm_mutex_unlock(&m_mutexComm);

  return m_dxl.getResult() == DXL_COMM_RXSUCCESS? true: false;
}

int DynaCommSerial::Reset(int nServoId)
{
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  // make reset packet
  m_dxl.setTxPacketId(nServoId);
  m_dxl.setTxPacketInstruction(DXL_INST_RESET);
  m_dxl.setTxPacketLength(2);

  m_dxl.txrxPacket();
  
  m_uBusStatus = (uint_t)m_dxl.getResult();

  if( m_uBusStatus == DXL_COMM_RXSUCCESS )
  {
    m_uAlarms = (uint_t)m_dxl.getRxPacketErrBits();
    DYNA_LOG_SERVO_ALARMS(nServoId, m_uAlarms);
    rc = DYNA_OK;
  }
  else
  {
    rc = -DynaMapDxlToEcode(m_uBusStatus);
    DYNA_LOG_ERROR(rc, "Reset(%d)", nServoId);
  }

  shm_mutex_unlock(&m_mutexComm);

  return rc;
}

int DynaCommSerial::MakeSerialIndex(const char *sSerialDevName)
{
  int   i, j, index;

  if( (sSerialDevName == NULL) || (*sSerialDevName == 0) )
  {
    return -DYNA_ECODE_BAD_VAL;
  }

  if( access(sSerialDevName, F_OK) )
  {
    return -DYNA_ECODE_BAD_DEV;
  }

  for(i=j=(int)strlen(sSerialDevName)-1; i>=0; --i)
  {
    if( !isdigit(sSerialDevName[i]) )
    {
      break;
    }
  }

  if( (i < 0) || (i == j) )
  {
    return -DYNA_ECODE_BAD_VAL;
  }

  index = atoi(sSerialDevName+i+1);

  return index;
}

void DynaCommSerial::InitRTS()
{
  SerDevSetHwFlowControl(m_fd, true);
}

void DynaCommSerial::EnableTxRTS(void *pArg)
{
  DynaCommSerial *pThis = (DynaCommSerial *)pArg;

  SerDevAssertRTS(pThis->m_fd);
}

void DynaCommSerial::EnableRxRTS(void *pArg, size_t uNumTxBytes)
{
  DynaCommSerial *pThis = (DynaCommSerial *)pArg;

  SerDevFIFOOutputDrain(pThis->m_fd);
  SerDevDeassertRTS(pThis->m_fd);
}

void DynaCommSerial::InitCTS()
{
  SerDevSetHwFlowControl(m_fd, true);
}

void DynaCommSerial::EnableTxCTS(void *pArg)
{
  DynaCommSerial *pThis = (DynaCommSerial *)pArg;

  SerDevAssertCTS(pThis->m_fd);
}

void DynaCommSerial::EnableRxCTS(void *pArg, size_t uNumTxBytes)
{
  DynaCommSerial *pThis = (DynaCommSerial *)pArg;

  SerDevFIFOOutputDrain(pThis->m_fd);
  SerDevDeassertCTS(pThis->m_fd);
}

void DynaCommSerial::InitGPIO(int nGpioNum)
{
  if( m_fdGpio >= 0 )
  {
    gpioClose(m_fdGpio);
    m_fdGpio = -1;
  }

  m_nGpioNum = nGpioNum;
  m_nGpioVal = -1;

  if( (m_fdGpio = gpioOpen(abs(m_nGpioNum))) < 0 )
  {
    LOGERROR("GPIO %d: Failed to open.", abs(m_nGpioNum));
  }
}

void DynaCommSerial::EnableTxGPIO(void *pArg)
{
  DynaCommSerial *pThis = (DynaCommSerial *)pArg;
  int             val;

  if( pThis->m_fdGpio >= 0 )
  {
    val = pThis->m_nGpioNum < 0? 0: 1;
    if( gpioQuickWrite(pThis->m_fdGpio, val) == OK )
    {
      pThis->m_nGpioVal = val;
    }
  }
}

void DynaCommSerial::EnableRxGPIO(void *pArg, size_t uNumTxBytes)
{
  static double TuneMargin = -30.0;   // sleep margin in usec

  DynaCommSerial *pThis   = (DynaCommSerial *)pArg;
  int             val;
  float           usecTx;

  if( pThis->m_fdGpio < 0 )
  {
    return;
  }

  val = pThis->m_nGpioNum < 0? 1: 0;

  // already in receive mode
  if( val == pThis->m_nGpioVal )
  {
    return;
  }

  // tcdrain() takes too long to return - disappointing. so can't use
  //SerDevFIFOOutputDrain(pThis->m_fd);

  //
  // Estimate time to transmit. Each byte = 10 bits with Start and Stop bits.
  //
  usecTx  = (double)(uNumTxBytes * 10)/(double)pThis->m_nBaudRate * 1000000.0 
                + TuneMargin;

  if( usecTx > 0.0 )
  {
    unsigned int usec = (unsigned int)usecTx;

    // wait
    usleep(usec);
  }

  // already at this value
  if( gpioQuickWrite(pThis->m_fdGpio, val) == OK )
  {
    pThis->m_nGpioVal = val;
  }
}
