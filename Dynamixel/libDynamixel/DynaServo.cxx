////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServo.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief Dynamixel servo abstract base class.
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

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/AX.h"
#include "Dynamixel/EX.h"
#include "Dynamixel/MX.h"
#include "Dynamixel/RX.h"

#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaOlio.h"

#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"
#include "Dynamixel/DynaServoAX12.h"
#include "Dynamixel/DynaServoEX106P.h"
#include "Dynamixel/DynaServoMX12W.h"
#include "Dynamixel/DynaServoMX28.h"
#include "Dynamixel/DynaServoMX64.h"
#include "Dynamixel/DynaServoMX106.h"
#include "Dynamixel/DynaServoRX10.h"
#include "Dynamixel/DynaServoRX24F.h"
#include "Dynamixel/DynaServoRX28.h"
#include "Dynamixel/DynaServoRX64.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// DynaServo Abstract Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaServo::~DynaServo()
{
  if( m_spec.m_sModelName != NULL )
  {
    delete[] m_spec.m_sModelName;
  }
}

DynaServo *DynaServo::New(DynaComm &comm, int nServoId)
{
  uint_t      uModelNum;
  uint_t      uFwVer;
  DynaServo  *pDynaServo;
  int         rc;

  // read the servo's model number
  if( (rc = ReadModelNumber(comm, nServoId, &uModelNum)) < 0 )
  {
    DYNA_LOG_ERROR(rc, "Servo %d: Failed to get the model number.", nServoId);
    return NULL;
  }

  // read the servo's firmware version
  if( (rc = ReadFirmwareVersion(comm, nServoId, &uFwVer)) < 0 )
  {
    DYNA_LOG_ERROR(rc, "Servo %d: Failed to get the firmware verions.",
        nServoId);
    return NULL;
  }

  //
  // Instantiate the specific derived DynaServo object.
  //
  switch( uModelNum )
  {
    case DynaServoAX12::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoAX12(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoEX106P::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoEX106P(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoMX12W::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoMX12W(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoMX28::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoMX28(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoMX64::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoMX64(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoMX106::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoMX106(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoRX10::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoRX10(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoRX24F::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoRX24F(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoRX28::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoRX28(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoRX64::DYNA_MODEL_NUM:
      pDynaServo = new DynaServoRX64(comm, nServoId, uModelNum, uFwVer);
      break;

    case DynaServoGeneric::DYNA_MODEL_NUM:
    default:
      pDynaServo = new DynaServoGeneric(comm, nServoId, uModelNum, uFwVer);
      break;
  }

  /*!
   * \todo TODO Add checks here, if object not errored (or exception thrown).
   * If error, delete object and return null.
   */

  return pDynaServo;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Odometer Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServo::ResetOdometer(int nEncZeroPt, bool bIsReverse)
{
  int   nEncCurPos;
  int   nOdPos;

  nEncCurPos = (int)m_state.m_uCurPos;

  m_state.m_od.m_nEncZeroPt   = nEncZeroPt;
  m_state.m_od.m_nOdDir       = bIsReverse? -1: 1;
  m_state.m_od.m_nEncLastPos  = nEncCurPos;

  nOdPos = m_state.m_od.m_nOdDir * (nEncCurPos - nEncZeroPt);

  m_state.m_od.m_bOdEnabled = true;
  m_state.m_od.m_nOdometer  = nOdPos;

  return m_state.m_od.m_nOdometer;
}

int DynaServo::UpdateOdometer(int nEncCurPos)
{
  int   dp;       // minimum delta position
  int   nOdPos;   // new odometer position

  if( m_state.m_od.m_bOdEnabled )
  {
    dp = nEncCurPos - m_state.m_od.m_nEncLastPos;

    if( iabs(dp) > m_spec.m_uRawPosMax/2 )
    {
      if( dp > 0 )
      {
        dp = dp - (int)m_spec.m_uRawPosMax;
      }
      else  // dp < 0
      {
        dp = dp + (int)m_spec.m_uRawPosMax;
      }
    }

    nOdPos = m_state.m_od.m_nOdometer + dp * m_state.m_od.m_nOdDir;

    m_state.m_od.m_nOdometer    = nOdPos;
    m_state.m_od.m_nEncLastPos  = nEncCurPos;

  }
  else
  {
    m_state.m_od.m_nOdometer = nEncCurPos;
  }

  return m_state.m_od.m_nOdometer;
}

void DynaServo::DisableOdometer()
{
  m_state.m_od.m_nEncZeroPt   = 0;
  m_state.m_od.m_nOdDir       = 1;
  m_state.m_od.m_nEncLastPos  = m_state.m_uCurPos;
  m_state.m_od.m_bOdEnabled   = false;
  m_state.m_od.m_nOdometer    = m_state.m_uCurPos;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServo::ReadModelNumber(DynaComm   &comm,
                               int         nServoId,
                               uint_t     *pModelNum)
{
  ushort_t  val = 0;
  int       rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  rc = comm.Read16(nServoId, DYNA_ADDR_MODEL_NUM_LSB, &val);

  if( rc == DYNA_OK )
  {
    *pModelNum = (uint_t)(val & DYNA_ADDR_MODEL_NUM_MASK);
  }

  return rc;
}

int DynaServo::ReadFirmwareVersion(DynaComm   &comm,
                                   int         nServoId,
                                   uint_t     *pFwVer)
{
  byte_t    val = 0;
  int       rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  rc = comm.Read8(nServoId, DYNA_ADDR_FWVER, &val);

  if( rc == DYNA_OK )
  {
    *pFwVer = (uint_t)(val & DYNA_ADDR_FWVER_MASK);
  }

  return rc;
}

int DynaServo::ReadServoId(DynaComm   &comm,
                           int         nServoId,
                           int        *pServoId)
{
  uint_t    val = 0;
  int       rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  rc = comm.Read8(nServoId, DYNA_ADDR_ID, &val);

  if( rc == DYNA_OK )
  {
    *pServoId = (int)(val & DYNA_ADDR_ID_MASK);
  }

  return rc;
}

int DynaServo::WriteServoId(DynaComm   &comm,
                            int         nServoId,
                            int         nNewServoId)
{
  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);
  DYNA_TRY_SERVO_ID(nNewServoId);

  return comm.Write8(nServoId, DYNA_ADDR_ID, (uint_t)nNewServoId);
}

int DynaServo::ReadBaudRate(DynaComm &comm, int nServoId, int *pBaudRate)
{
  uint_t    uBaudNum;
  int       nBaudRate;
  int       rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  rc = comm.Read8(nServoId, DYNA_ADDR_BAUD_RATE, &uBaudNum);
  
  if( rc == DYNA_OK )
  {
    // convert to baudrate
    if( (nBaudRate = comm.BaudNumToRate((int)uBaudNum)) < 0 )
    {
      rc = nBaudRate;
      DYNA_LOG_ERROR(rc, "Servo %d: Unexpected baud number %u read.",
        nServoId, uBaudNum);
    }
    else
    {
      *pBaudRate = nBaudRate;
      rc = DYNA_OK;
    }
  }

  return rc;
}

int DynaServo::WriteBaudRate(DynaComm   &comm,
                             int         nServoId,
                             int         nNewBaudRate)
{
  int     nBaudNum;
  int     rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  // convert to baudrate
  if( (nBaudNum = comm.BaudRateToNum(nNewBaudRate)) < 0 )
  {
    rc = nBaudNum;
    DYNA_LOG_ERROR(rc, "Servo %d: Unsupported baud rate %d.",
        nServoId, nNewBaudRate);
  }

  else
  {
    rc = comm.Write8(nServoId, DYNA_ADDR_BAUD_RATE, (uint_t)nBaudNum);
  }

  return rc;
}

bool DynaServo::Ping(DynaComm &comm, int nServoId)
{
  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  return comm.Ping(nServoId);
}

int DynaServo::Reset(DynaComm &comm, int nServoId)
{
  int   rc;

  DYNA_TRY_COMM(comm);
  DYNA_TRY_SERVO_ID(nServoId);

  return comm.Reset(nServoId);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaServo::Init(int nServoId, uint_t uModelNum, uint_t uFwVer) 
{
  m_spec.m_sModelName = newstr("abstract class");

  m_cfg.m_uModelNum   = uModelNum;
  m_cfg.m_uFwVer      = uFwVer;
  m_cfg.m_nServoId    = nServoId;

  m_nServoId          = nServoId;   // for ease of use

  m_pAgent            = NULL;
  m_pAgentArg         = NULL;

  Unlink();
}

void DynaServo::DumpCtlTbl(const char              *sTblName,
                           const DynaCtlTblEntry_T  tblInfo[],
                           size_t                   uSize)
{
  const DynaCtlTblEntry_T  *p;

  size_t    i;
  uint_t    uVal;
  uint_t    uMask;
  int       iVal;
  int       rc;

  // table name
  printf("               %s Control Table\n", sTblName);
  printf("Address   Name                       Raw     Value\n");
  printf("-------   ----                       ---     -----\n");

  //
  // Loop through control table information and print.
  //
  for(i=0; i<uSize; ++i)
  {
    p = &tblInfo[i];

    printf("%2u (0x%02x) %-26s ", p->m_uAddr, p->m_uAddr, p->m_sName);

    //
    // Control table entry size in bytes.
    //
    switch( p->m_uSize )
    {
      case 1:
        rc = m_comm.Read8(m_nServoId, p->m_uAddr, &uVal);
        break;
      case 2:
        rc = m_comm.Read16(m_nServoId, p->m_uAddr, &uVal);
        break;
      default:
        rc = -DYNA_ECODE_BAD_VAL;
        break;
    }

    if( rc != DYNA_OK )
    {
      printf("Failed To Read\n");
      continue;
    }

    //
    // Print the raw value before masking to display the exact contents of
    // the control table entry.
    //
    switch( p->m_uSize )
    {
      case 1:
        printf("0x%02x    ", uVal);
        break;
      case 2:
      default:
        printf("0x%04x  ", uVal);
        break;
        break;
    }

    //
    // Signed entry value. Upper most mask bit is the sign.
    //
    if( p->m_bSigned )
    {
      uMask = p->m_uMask >> 1;
      iVal = (int)(uVal & uMask);
      if( uVal & ~uMask )
      {
        iVal = -iVal;
      }
    }

    //
    // Unsigned entry value.
    //
    else
    {
      iVal = (int)(uVal & p->m_uMask);
    }

    //
    // Print interpreted value.
    //
    if( p->m_sFmt != NULL )
    {
      printf(p->m_sFmt, iVal);
    }

    printf("\n");
  }
}
