////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaChain.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 11:54:07 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3846 $
 *
 * \brief Dynamixel Chain class.
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaOlio.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// DynaChain Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaChain::DynaChain(DynaComm &comm) : m_comm(comm)
{
  Init();
}

DynaChain::~DynaChain()
{
  int   nServoId;

  for(nServoId=DYNA_ID_MIN; nServoId<DYNA_ID_NUMOF; ++nServoId)
  {
    if( m_pChain[nServoId] != NULL )
    {
      delete m_pChain[nServoId];
    }
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Attribute Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

int DynaChain::LinkServos(int  nServoIdMaster,
                          int  nServoIdSlave,
                          bool bIsReversed)
{
  DynaServo  *pServoM;
  DynaServo  *pServoS;
  int         rc;

  DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdMaster);
  DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdSlave);

  DYNA_TRY_EXPR(((m_links[nServoIdMaster].m_uLinkType == DYNA_LINK_NONE) &&
                 (m_links[nServoIdSlave].m_uLinkType == DYNA_LINK_NONE)),
      DYNA_ECODE_LINKED,
      "Servos %d and/or %d: Already have linkage.",
      nServoIdMaster, nServoIdSlave);
 
  pServoM = m_pChain[nServoIdMaster];
  pServoS = m_pChain[nServoIdSlave];

  if( pServoM->GetModelNumber() != pServoS->GetModelNumber() )
  {
    rc = -DYNA_ECODE_LINKED;
    DYNA_LOG_ERROR(rc, "Cannot link unmatched servos: "
        "servo %d model %s(%d) != servo %d model %s(%d).",
        nServoIdMaster, pServoM->GetModelName(), pServoM->GetModelNumber(),
        nServoIdSlave,  pServoS->GetModelName(), pServoS->GetModelNumber());
    return rc;
  }

  pServoM->Link(DYNA_LINK_MASTER, pServoS, bIsReversed);
  pServoS->Link(DYNA_LINK_SLAVE,  pServoM, bIsReversed);

  SetLinkData(nServoIdMaster, DYNA_LINK_MASTER, nServoIdSlave, bIsReversed);
  SetLinkData(nServoIdSlave, DYNA_LINK_SLAVE, nServoIdMaster, bIsReversed);

  GetNumberOfMastersInChain();

  return DYNA_OK;
}

int DynaChain::UnlinkServos(int nServoIdMaster)
{
  int         nServoIdSlave;
  DynaServo  *pServoM;
  DynaServo  *pServoS;
  int         rc;

  DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdMaster);

  pServoM = m_pChain[nServoIdMaster];

  nServoIdSlave = m_links[nServoIdMaster].m_nServoIdMate;

  DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdSlave);

  pServoS = m_pChain[nServoIdSlave];

  DYNA_TRY_EXPR(
      ((m_links[nServoIdMaster].m_uLinkType != DYNA_LINK_NONE) &&
       (m_links[nServoIdSlave].m_uLinkType != DYNA_LINK_NONE) &&
       (m_links[nServoIdSlave].m_nServoIdMate == nServoIdMaster)),
      DYNA_ECODE_LINKED,
      "Servos %d and/or %d: Not linked.",
      nServoIdMaster, nServoIdSlave);
 
  pServoM->Unlink();
  pServoS->Unlink();

  ClearLinkData(nServoIdMaster);
  ClearLinkData(nServoIdSlave);

  GetNumberOfMastersInChain();

  return DYNA_OK;
}

uint_t DynaChain::GetNumberOfMastersInChain()
{
  int   i;
  int   nServoId;

  m_uNumMastersInChain = 0;

  for(i=0; i<(int)m_uNumInChain; ++i)
  {
    nServoId = m_nIIdx[i];

    if( (nServoId != DYNA_ID_NONE) && IsMaster(nServoId) )
    {
      ++m_uNumMastersInChain;
    }
  }

  return m_uNumMastersInChain;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Chain Servo Add/Remove Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

int DynaChain::AddNewServo(int nServoId)
{
  int   rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_ID(nServoId);

  // Ping the servo
  if( !DynaServo::Ping(m_comm, nServoId) )
  {
    rc = -DYNA_ECODE_NO_SERVO;
    DYNA_LOG_ERROR(rc, "Pinging servo %d failed. Cannot add.", nServoId);
  }

  // create and insert chain entry
  else if( (rc = ChainEntryNew(nServoId)) == DYNA_OK )
  {
    AuditLinks();
  }

  return rc;
}

int DynaChain::AddNewServosByScan()
{
  int     nServoId;
  int     nNumAdded;

  DYNA_TRY_COMM(m_comm);

  // remove and delete all servos
  for(nServoId=DYNA_ID_MIN; nServoId<DYNA_ID_NUMOF; ++nServoId)
  {
    ChainEntryDelete(nServoId);
  }

  // add all by scan
  for(nServoId=DYNA_ID_MIN, nNumAdded=0; nServoId<DYNA_ID_NUMOF; ++nServoId)
  {
    if( DynaServo::Ping(m_comm, nServoId) )
    {
      if( ChainEntryNew(nServoId) == DYNA_OK )
      {
        ++nNumAdded;
      }
    }
  }

  AuditLinks();

  return nNumAdded;
}

int DynaChain::AddNewServoForced(int nServoId, uint_t uModelNum)
{
  /*! \todo TODO force add servo to chain */
  return -DYNA_ECODE_NOT_SUPP;
}

int DynaChain::RemoveServo(int nServoId)
{
  DYNA_TRY_SERVO_IN_CHAIN(this, nServoId);

  ChainEntryDelete(nServoId);
  AuditLinks();

  return DYNA_OK;
}

int DynaChain::RemoveAllServos()
{
  int   nServoId;

  for(nServoId=DYNA_ID_MIN; nServoId<DYNA_ID_NUMOF; ++nServoId)
  {
    if( m_pChain[nServoId] != NULL )
    {
      ChainEntryDelete(nServoId);
    }
  }

  AuditLinks();

  return DYNA_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaChain::Init()
{
  int   nServoId;

  m_uNumInChain        = 0;
  m_uNumMastersInChain = 0;

  for(nServoId=DYNA_ID_MIN; nServoId<DYNA_ID_NUMOF; ++nServoId)
  {
    m_pChain[nServoId] = NULL;
    m_nIIdx[nServoId]  = DYNA_ID_NONE;
    ClearLinkData(nServoId);
  }
}

int DynaChain::ChainEntryNew(int nServoId)
{
  DynaServo  *pServo;
  int         rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_ID(nServoId);

  // create derived servo object 
  pServo = DynaServo::New(m_comm, nServoId);

  if( pServo == NULL )
  {
    rc = -DYNA_ECODE_NO_SERVO;
    DYNA_LOG_ERROR(rc, "Failed to create servo %d. Cannot add.", nServoId);
    return rc;
  }

  // remove any existing servo with the same id
  if( m_pChain[nServoId] != NULL )
  {
    ChainEntryDelete(nServoId);
  }

  // insert into chain
  m_pChain[nServoId] = pServo;
  m_nIIdx[m_uNumInChain++] = nServoId;

  LOGDIAG3("%s servo %d (model number 0x%04x) added to chain.",
      pServo->GetModelName(), nServoId, pServo->GetModelNumber());

  return DYNA_OK;
}
 
int DynaChain::ChainEntryDelete(int nServoId)
{
  DynaServo *pServo;
  uint_t     i, j;

  DYNA_TRY_SERVO_ID(nServoId);

  // okay not to be in chain
  if( (pServo = m_pChain[nServoId]) == NULL )
  {
    return DYNA_OK;
  }

  // remove from indirect index list
  for(i=0; i<m_uNumInChain; ++i)
  {
    if( m_nIIdx[i] == nServoId )
    {
      m_nIIdx[i] = DYNA_ID_NONE;
      m_uNumInChain--;
      break;
    }
  }

  // compress indirect index list
  for(j=i; j<m_uNumInChain; ++j)
  {
    m_nIIdx[j] = m_nIIdx[j+1];
    m_nIIdx[j+1] = DYNA_ID_NONE;
  }

  LOGDIAG3("%s servo %d (model number 0x%04x) removed from chain.",
      pServo->GetModelName(), nServoId, pServo->GetModelNumber());

  // delete from chain
  delete m_pChain[nServoId];

  m_pChain[nServoId] = NULL;

  return DYNA_OK;
}
 
void DynaChain::AuditLinks()
{
  int         nServoIdThis;
  uint_t      uLinkType;
  int         nServoIdMate;
  bool        bRotReversed;
  DynaServo  *pServoThis;
  DynaServo  *pServoMate;

  for(nServoIdThis=DYNA_ID_MIN; nServoIdThis<DYNA_ID_NUMOF; ++nServoIdThis)
  {
    uLinkType     = m_links[nServoIdThis].m_uLinkType;
    nServoIdMate  = m_links[nServoIdThis].m_nServoIdMate;
    bRotReversed  = m_links[nServoIdThis].m_bRotReversed;
    pServoThis    = m_pChain[nServoIdThis];

    // this servo is not present in the chain
    if( pServoThis == NULL )
    {
      ClearLinkData(nServoIdThis);
    }

    // this link is either the master or slave servo
    else if( (uLinkType == DYNA_LINK_MASTER) || (uLinkType == DYNA_LINK_SLAVE) )
    {
      // no linked mate
      if( (nServoIdMate < DYNA_ID_MIN) ||
          (nServoIdMate > DYNA_ID_MAX) ||
          ((pServoMate = m_pChain[nServoIdMate]) == NULL) )
      {
        pServoThis->Unlink();
        ClearLinkData(nServoIdThis);
      }

      // linked servos mismatch
      else if(pServoThis->GetModelNumber() != pServoMate->GetModelNumber())
      {
        pServoThis->Unlink();
        pServoMate->Unlink();
        ClearLinkData(nServoIdThis);
      }

      // this link is the master servo
      else if( uLinkType == DYNA_LINK_MASTER )
      {
        pServoThis->Link(DYNA_LINK_MASTER, pServoMate, bRotReversed);
        pServoMate->Link(DYNA_LINK_SLAVE,  pServoThis, bRotReversed);
        SetLinkData(nServoIdMate, DYNA_LINK_SLAVE, nServoIdThis, bRotReversed);
      }

      // this link is the slave servo
      else
      {
        pServoThis->Link(DYNA_LINK_SLAVE,  pServoMate, bRotReversed);
        pServoMate->Link(DYNA_LINK_MASTER, pServoThis, bRotReversed);
        SetLinkData(nServoIdMate, DYNA_LINK_MASTER, nServoIdThis, bRotReversed);
      }
    }

    // this servo has no link 
    else // DYNA_LINK_NONE
    {
      pServoThis->Unlink();
      ClearLinkData(nServoIdThis);
    }
  }

  GetNumberOfMastersInChain();
}

void DynaChain::SetLinkData(int     nServoId,
                            uint_t  uLinkType,
                            int     nServoIdMate,
                            bool    bRotReversed)
{
  m_links[nServoId].m_uLinkType     = uLinkType;
  m_links[nServoId].m_nServoIdMate  = nServoIdMate;
  m_links[nServoId].m_bRotReversed  = bRotReversed;
}

void DynaChain::ClearLinkData(int nServoId)
{
  m_links[nServoId].m_uLinkType     = DYNA_LINK_NONE;
  m_links[nServoId].m_nServoIdMate  = DYNA_ID_NONE;
  m_links[nServoId].m_bRotReversed  = false;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Master and Synchronous Move Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

int DynaChain::MoveTo(int nServoId, int nGoalOdPos)
{
  DynaServo *pServo;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_IN_CHAIN(this, nServoId);

  pServo = m_pChain[nServoId];

  DYNA_TRY_IS_MASTER(pServo);

  return pServo->MoveTo(nGoalOdPos);
}

int DynaChain::vSyncMoveTo(uint_t uCount, ...)
{
  DynaPosTuple_T    tuplesPos[DYNA_ID_NUMOF];
  va_list           ap;
  int               i;

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuplesPos[i].m_nServoId = va_arg(ap, int);
    tuplesPos[i].m_nPos     = va_arg(ap, int);
  }

  va_end(ap);

  return SyncMoveTo(tuplesPos, (uint_t)i);
}

int DynaChain::SyncMoveTo(DynaPosTuple_T tuplesPos[], uint_t uCount)
{
  DynaServo            *pServo;
  DynaPosTuple_T        tuple[DYNA_ID_NUMOF];
  int                   i;
  int                   j;
  int                   rc;

  DYNA_TRY_COMM(m_comm);

  for(i=0, j=0, rc=DYNA_OK;
      i<(int)uCount && i<DYNA_ID_NUMOF && rc==DYNA_OK;
      ++i)
  {
    DYNA_TRY_SERVO_IN_CHAIN(this, tuplesPos[i].m_nServoId);

    pServo = m_pChain[tuplesPos[i].m_nServoId];

    DYNA_TRY_IS_MASTER(pServo);
    DYNA_TRY_SERVO_HAS_POS_CTL(pServo);

    //
    // In continuous mode: Start host controlled positioning.
    //
    if( pServo->GetServoMode() == DYNA_MODE_CONTINUOUS )
    {
      rc = pServo->MoveTo(tuplesPos[i].m_nPos);
    }
    //
    // Let Dynmixel firmware take control of the move.
    else
    {
      tuple[j].m_nServoId = tuplesPos[i].m_nServoId;
      tuple[j].m_nPos     = tuplesPos[i].m_nPos;
      ++j;
    }
  }

  if( (rc == DYNA_OK) && (j > 0) )
  {
    rc = SyncWriteGoalPos(tuplesPos, (uint_t)j);
  }

  return rc;
}

int DynaChain::MoveAtSpeedTo(int nServoId, int nGoalSpeed, int nGoalOdPos)
{
  DynaServo *pServo;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_IN_CHAIN(this, nServoId);

  pServo = m_pChain[nServoId];

  DYNA_TRY_IS_MASTER(pServo);
  DYNA_TRY_SERVO_HAS_POS_CTL(pServo);

  return pServo->MoveAtSpeedTo(nGoalSpeed, nGoalOdPos);
}

int DynaChain::vSyncMoveAtSpeedTo(uint_t uCount, ...)
{
  DynaSpeedPosTuple_T tuplesSpeedPos[DYNA_ID_NUMOF];
  va_list             ap;
  int                 i;

  DYNA_TRY_COMM(m_comm);

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuplesSpeedPos[i].m_nServoId  = va_arg(ap, int);
    tuplesSpeedPos[i].m_nSpeed    = va_arg(ap, int);
    tuplesSpeedPos[i].m_nPos      = va_arg(ap, int);
  }

  va_end(ap);

  return SyncMoveAtSpeedTo(tuplesSpeedPos, (uint_t)i);
}

int DynaChain::SyncMoveAtSpeedTo(DynaSpeedPosTuple_T tuplesSpeedPos[],
                                 uint_t              uCount)
{
  DynaSpeedTuple_T  tuplesSpeed[DYNA_ID_NUMOF];
  DynaPosTuple_T    tuplesPos[DYNA_ID_NUMOF];
  DynaServo        *pServo;
  int               i, j;
  int               rc;

  DYNA_TRY_COMM(m_comm);

  for(i=0, j=0, rc=DYNA_OK;
      i<(int)uCount && i<DYNA_ID_NUMOF && rc==DYNA_OK;
      ++i)
  {
    DYNA_TRY_SERVO_IN_CHAIN(this, tuplesSpeedPos[i].m_nServoId);

    pServo = m_pChain[tuplesSpeedPos[i].m_nServoId];

    DYNA_TRY_IS_MASTER(pServo);
    DYNA_TRY_SERVO_HAS_POS_CTL(pServo);

    //
    // In continuous mode: Start host controlled positioning.
    //
    if( pServo->HasAgent() )
    {
      rc = pServo->MoveAtSpeedTo(tuplesSpeedPos[i].m_nSpeed,
                                 tuplesSpeedPos[i].m_nPos);
    }

    //
    // Servo controlled move.
    //
    else
    {
      tuplesSpeed[j].m_nServoId   = tuplesSpeedPos[i].m_nServoId;
      tuplesSpeed[j].m_nSpeed     = tuplesSpeedPos[i].m_nSpeed;

      tuplesPos[j].m_nServoId     = tuplesSpeedPos[i].m_nServoId;
      tuplesPos[j].m_nPos         = tuplesSpeedPos[i].m_nPos;

      ++j;
    }
  }

  if( (j > 0) && (rc == DYNA_OK) )
  {
    rc = SyncWriteGoalSpeed(tuplesSpeed, (uint_t)j);

    if( rc == DYNA_OK )
    {
      rc = SyncWriteGoalPos(tuplesPos, (uint_t)j);
    }
  }

  return rc;
}

int DynaChain::MoveAtSpeed(int nServoId, int nGoalSpeed)
{
  DynaServo *pServo;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_IN_CHAIN(this, nServoId);

  pServo = m_pChain[nServoId];

  DYNA_TRY_IS_MASTER(pServo);

  return pServo->MoveAtSpeed(nGoalSpeed);
}

int DynaChain::vSyncMoveAtSpeed(uint_t uCount, ...)
{
  DynaSpeedTuple_T  tuplesSpeed[DYNA_ID_NUMOF];
  va_list           ap;
  int               i;

  DYNA_TRY_COMM(m_comm);

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuplesSpeed[i].m_nServoId = va_arg(ap, int);
    tuplesSpeed[i].m_nSpeed   = va_arg(ap, int);

    DYNA_TRY_SERVO_IN_CHAIN(this, tuplesSpeed[i].m_nServoId);
    DYNA_TRY_IS_MASTER(m_pChain[tuplesSpeed[i].m_nServoId]);
    DYNA_TRY_SERVO_IN_MODE(m_pChain[tuplesSpeed[i].m_nServoId],
                          DYNA_MODE_CONTINUOUS);
  }

  va_end(ap);

  return SyncWriteGoalSpeed(tuplesSpeed, (uint_t)i);
}

int DynaChain::SyncMoveAtSpeed(DynaSpeedTuple_T tuplesSpeed[],
                               uint_t           uCount)
{
  int   i;
  int   rc;

  DYNA_TRY_COMM(m_comm);

  for(i=0; i<(int)uCount && i<DYNA_ID_NUMOF; ++i)
  {
    DYNA_TRY_SERVO_IN_CHAIN(this, tuplesSpeed[i].m_nServoId);
    DYNA_TRY_IS_MASTER(m_pChain[tuplesSpeed[i].m_nServoId]);
    DYNA_TRY_SERVO_IN_MODE(m_pChain[tuplesSpeed[i].m_nServoId],
                          DYNA_MODE_CONTINUOUS);
  }

  return SyncWriteGoalSpeed(tuplesSpeed, (uint_t)i);
}

int DynaChain::EStop()
{
  int nMaxTries = 3;
  int nTries;
  int rc;

  DYNA_TRY_COMM(m_comm);

  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    if( (rc = SyncWriteTorqueEnable(false)) == DYNA_OK )
    {
      break;
    }
  }

  return rc;
}

int DynaChain::Stop()
{
  int         nServoId;   // working servo id
  int         iter;       // iterator
  int         rc;         // return code

  DYNA_TRY_COMM(m_comm);

  for(nServoId=IterStartMaster(&iter);
      nServoId!=DYNA_ID_NONE;
      nServoId=IterNextMaster(&iter))
  {
    rc = m_pChain[nServoId]->Stop();
  }

  return rc;
}

int DynaChain::Freeze()
{
  int         rc_tmp;     // temporary return code
  int         rc;         // return code

  DYNA_TRY_COMM(m_comm);

  rc = SyncWriteTorqueEnable(true);

  rc_tmp = Stop();

  if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  return rc;
}

int DynaChain::Release()
{
  DYNA_TRY_COMM(m_comm);

  return SyncWriteTorqueEnable(false);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Master and Synchronous Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

int DynaChain::SyncWriteTorqueEnable(bool bState)
{
  uint_t                uVal;
  int                   nServoId;
  int                   iter;
  DynaSyncWriteTuple_T  tuples[DYNA_ID_NUMOF];
  uint_t                uCount;
  int                   rc;

  DYNA_TRY_COMM(m_comm);

  uVal = bState? DYNA_TORQUE_EN_ON: DYNA_TORQUE_EN_OFF;
 
  for(nServoId=IterStart(&iter), uCount=0;
      nServoId!=DYNA_ID_NONE;
      nServoId=IterNext(&iter), ++uCount)
  {
    tuples[uCount].m_nServoId = nServoId;
    tuples[uCount].m_uVal     = uVal;
  }

  rc = m_comm.SyncWrite(DYNA_ADDR_TORQUE_EN, 1, tuples, uCount);

  if( rc == DYNA_OK )
  {
    for(iter=0; iter<(int)uCount; ++iter)
    {
      m_pChain[tuples[iter].m_nServoId]->m_state.m_bTorqueEnabled = bState;
    }
  }

  return rc;
}

int DynaChain::vSyncWriteGoalPos(uint_t uCount, ...)
{
  DynaPosTuple_T  tuplesPos[DYNA_ID_NUMOF]; 
  va_list         ap;
  int             i;

  DYNA_TRY_COMM(m_comm);

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuplesPos[i].m_nServoId = va_arg(ap, int);
    tuplesPos[i].m_nPos     = va_arg(ap, int);
  }

  va_end(ap);

  return SyncWriteGoalPos(tuplesPos, (uint_t)i);
}

int DynaChain::SyncWriteGoalPos(DynaPosTuple_T tuplesPos[], uint_t uCount)
{
  DynaSyncWriteTuple_T  tuples[DYNA_ID_NUMOF];  // servo_id, enc_pos tuples
  int                   nServoIdM;              // master servo id
  int                   nServoIdS;              // slave servo id
  DynaServo            *pServoM;                // master servo
  DynaServo            *pServoS;                // slave servo
  int                   nGoalOdPosM;            // master goal odometer position
  int                   nGoalOdPosS;            // slave goal odometer position
  int                   nEncPos;                // working encoder position
  bool                  bRotReversed;           // slave rotation is reverse
  int                   i, j;                   // working indices
  int                   rc;                     // return code

  DYNA_TRY_COMM(m_comm);

  // 
  // Loop through tuples.
  //
  for(i=0, j=0; (i<(int)uCount) && (j<DYNA_ID_NUMOF-1); ++i)
  {
    nServoIdM   = tuplesPos[i].m_nServoId;
    nGoalOdPosM = tuplesPos[i].m_nPos;

    DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdM);

    pServoM = m_pChain[nServoIdM];

    DYNA_TRY_IS_MASTER(pServoM);

    nEncPos = pServoM->OdometerToEncoder(nGoalOdPosM);

    if( pServoM->GetServoMode() == DYNA_MODE_SERVO )
    {
      DYNA_TRY_EXPR(((nEncPos >= pServoM->m_spec.m_uRawPosMin) &&
                     (nEncPos <= pServoM->m_spec.m_uRawPosMax)),
        DYNA_ECODE_BAD_VAL,
        "Master servo %d: Goal odometer position %d(encoder=%d): Out of range.",
        pServoM->GetServoId(), nGoalOdPosM, nEncPos);
    }

    tuples[j].m_nServoId  = nServoIdM;
    tuples[j].m_uVal      = (uint_t)nEncPos;

    j++;

    // master-slave linked servos
    if( m_links[nServoIdM].m_uLinkType == DYNA_LINK_MASTER )
    {
      nServoIdS = m_links[nServoIdM].m_nServoIdMate;

      DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdS);

      pServoS = m_pChain[nServoIdS];

      rc = pServoM->CalcMatesGoalPos(nGoalOdPosM, &nGoalOdPosS);

      DYNA_TRY_RC(rc, "Slave servo %d: Failed.", nServoIdS);

      nEncPos = pServoS->OdometerToEncoder(nGoalOdPosS);

      tuples[j].m_nServoId  = nServoIdS;
      tuples[j].m_uVal      = (uint_t)nEncPos;

      j++;
    }
  }

  // sync write
  rc = m_comm.SyncWrite(DYNA_ADDR_GOAL_POS_LSB, 2, tuples, (uint_t)j);

  if( rc == DYNA_OK )
  {
    for(i=0; i<j; ++i)
    {
      m_pChain[tuples[i].m_nServoId]->m_state.m_uGoalPos = tuples[i].m_uVal;
    }
  }

  return rc;
}

int DynaChain::vSyncWriteGoalSpeed(uint_t uCount, ...)
{
  DynaSpeedTuple_T  tuplesSpeed[DYNA_ID_NUMOF];
  va_list           ap;
  int               i;

  DYNA_TRY_COMM(m_comm);

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuplesSpeed[i].m_nServoId = va_arg(ap, int);
    tuplesSpeed[i].m_nSpeed   = va_arg(ap, int);
  }

  va_end(ap);

  return SyncWriteGoalSpeed(tuplesSpeed, (uint_t)i);
}

int DynaChain::SyncWriteGoalSpeed(DynaSpeedTuple_T tuplesSpeed[], uint_t uCount)
{
  DynaSyncWriteTuple_T  tuples[DYNA_ID_NUMOF];
  int                   nServoIdM;
  int                   nServoIdS;
  DynaServo            *pServoM;
  DynaServo            *pServoS;
  DynaServo            *pServo;
  int                   nGoalSpeedM;
  int                   nGoalSpeedS;
  int                   i, j;
  int                   rc;

  DYNA_TRY_COMM(m_comm);

  for(i=0, j=0; (i<(int)uCount) && (j<DYNA_ID_NUMOF-1); ++i)
  {
    nServoIdM       = tuplesSpeed[i].m_nServoId;
    nGoalSpeedM     = tuplesSpeed[i].m_nSpeed;

    DYNA_TRY_SERVO_IN_CHAIN(this, nServoIdM);

    pServoM = m_pChain[tuplesSpeed[i].m_nServoId];

    DYNA_TRY_IS_MASTER(pServoM);

    DYNA_TRY_EXPR( (iabs(nGoalSpeedM) <= pServoM->m_spec.m_uRawSpeedMax),
        DYNA_ECODE_BAD_VAL,
        "Master servo %d: Goal speed %d: Out of range.",
        pServoM->GetServoId(), nGoalSpeedM);

    tuples[j].m_nServoId  = nServoIdM;
    tuples[j].m_uVal = pServoM->PackGoalSpeed(nGoalSpeedM);
    j++;

    // master-slave linked servos
    if( m_links[nServoIdM].m_uLinkType == DYNA_LINK_MASTER )
    {
      nServoIdS = m_links[nServoIdM].m_nServoIdMate;
      pServoS   = m_pChain[nServoIdS];

      nGoalSpeedS = pServoM->CalcMatesGoalSpeed(nGoalSpeedM);

      tuples[j].m_nServoId  = nServoIdS;
      tuples[j].m_uVal = pServoS->PackGoalSpeed(nGoalSpeedS);
      j++;
    }
  }

  rc = m_comm.SyncWrite(DYNA_ADDR_GOAL_SPEED_LSB, 2, tuples, (uint_t)j);

  if( rc == DYNA_OK )
  {
    for(i=0; i<j; ++i)
    {
      pServo = m_pChain[tuples[i].m_nServoId];
      pServo->m_state.m_nGoalSpeed = pServo->UnpackGoalSpeed(tuples[i].m_uVal);
    }
  }

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Chain Iterators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

int DynaChain::IterStart(int *pIter)
{
  if( m_uNumInChain == 0 )
  {
    return DYNA_ID_NONE;
  }

  *pIter = 1;

  return m_nIIdx[0];
}

int DynaChain::IterNext(int *pIter)
{
  int   i;

  i = *pIter;

  if( (i >= (int)m_uNumInChain) || (m_nIIdx[i] == DYNA_ID_NONE) )
  {
    return DYNA_ID_NONE;
  }

  *pIter = i + 1;

  return m_nIIdx[i];
}

int DynaChain::IterStartMaster(int *pIter)
{
  int   i;
  int   nServoId;

  for(i=0; i<(int)m_uNumInChain; ++i)
  {
    nServoId = m_nIIdx[i];
    if( nServoId == DYNA_ID_NONE )
    {
      return DYNA_ID_NONE;
    }
    else if( m_pChain[nServoId]->IsMaster() )
    {
      *pIter = i + 1;
      return nServoId;
    }
  }

  return DYNA_ID_NONE;
}

int DynaChain::IterNextMaster(int *pIter)
{
  int   i;
  int   nServoId;

  for(i = *pIter; i<(int)m_uNumInChain; ++i)
  {
    nServoId = m_nIIdx[i];
    if( nServoId == DYNA_ID_NONE )
    {
      return DYNA_ID_NONE;
    }
    else if( m_pChain[nServoId]->IsMaster() )
    {
      *pIter = i + 1;
      return nServoId;
    }
  }

  return DYNA_ID_NONE;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

const DynaServo *DynaChain::operator[](int nServoId)
{
  if( (nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX) )
  {
    return m_pChain[nServoId];
  }
  else
  {
    return NULL;
  }
}
