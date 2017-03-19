////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_recording.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Recording Class
 *
 * \note May promote this class to libDynamixel library.
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

#include <time.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaChain.h"

#include "dynashell.h"
#include "dynashell_recording.h"
#include "dynashell_util.h"

using namespace std;


//-----------------------------------------------------------------------------
// Private Interface
//-----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
// DynaRecord Class
// -----------------------------------------------------------------------------


/*!
 * \brief Add a field tuple to the record.
 *
 * \param nPos    Servo position.
 * \param nSpeed  Servo speed.
 *
 * \return
 * On success, returns the newly added field number.\n
 * On failure, \ref DynaRecord::END is returned.
 */
int DynaRecord::AddField(int nPos, int nSpeed)
{
  if( m_nFieldCnt < DYNA_ID_NUMOF )
  {
    int nFldNum = m_nFieldCnt;
    m_field[nFldNum].m_nPos   = nPos;
    m_field[nFldNum].m_nSpeed = nSpeed;
    m_nFieldCnt++;
    return nFldNum;
  }
  else
  {
    return DynaRecord::END;
  }
}

/*!
 * \brief Assignment operator.
 *
 * \param rhs   Right hand side.
 *
 * \return Reference to this.
 */
DynaRecord &DynaRecord::operator=(const DynaRecord &rhs)
{
  int   i;

  m_nFieldCnt = rhs.m_nFieldCnt;

  for(i=0; i<m_nFieldCnt; ++i)
  {
    m_field[i] = rhs.m_field[i];
  }

  return *this;
}

/*! 'Not a Field' field. */
const DynaRecord::FieldTuple_T DynaRecord::NoField = {INT_MAX, INT_MAX};


// -----------------------------------------------------------------------------
// DynaRecording Class
// -----------------------------------------------------------------------------

/*!
 * \brief (Re)Initialize recording.
 *
 * Any previously recorded data is 'erased'. 
 *
 * \param nSamplePeriod   Recording sample period (msec).
 * \param sData           Recording date string.
 */
void DynaRecording::Init(int nSamplePeriod, const char *sDate)
{
  int         nRecNum;  // working record number

  // clear vector of information about the servos in recording
  m_vecServoInfo.clear();
  m_mapIIdx.clear();

  SetDate(sDate);

  m_nSamplePeriod = nSamplePeriod;
  m_nRecordCnt    = 0;

  // clear any old records
  for(nRecNum=0; nRecNum<MaxRecords; ++nRecNum)
  {
    m_record[nRecNum].ResetFieldCount();
  }
}

/*!
 * \brief Add a new empty record to the recording.
 *
 * \return
 * If the number of records already in the recording is \ref MaxRecords and
 * there are registered servos listed in the recording header, then
 * the new record number is returned.\n
 * Otherwise \ref END is returned.
 */
int DynaRecording::AddRecord()
{
  int nRecNum;

  if( (m_nRecordCnt >= MaxRecords) || (GetNumOfServosInRecording() == 0) )
  {
    return END;
  }
  else
  {
    nRecNum = m_nRecordCnt;
    m_record[nRecNum].ResetFieldCount();
    ++m_nRecordCnt;
    return nRecNum;
  }
}

/*!
 * \brief Add new recording field tuple.
 *
 * The field position must match the registered servo position listed in the
 * header.
 *
 * \param nRecNum   Record number.
 * \param nServoId  Servo id of field tuple data.
 * \param nPos      Servo position.
 * \param nSpeed    Servo speed.
 *
 * \return
 * Returns added field number on success.\n
 * Returns \ref END of failure to add.
 */
int DynaRecording::AddFieldTuple(int    nRecNum,
                                 int    nServoId,
                                 int    nPos,
                                 int    nSpeed)
{
  int nFieldCnt;

  nFieldCnt = m_record[nRecNum].GetFieldCount();

  if( nRecNum == END )
  {
    return END;
  }
  else if( nRecNum >= m_nRecordCnt )
  {
    LOGERROR("Record number %d >= record count = %d.", nRecNum, m_nRecordCnt);
    return END;
  }
  else if( nFieldCnt >= GetNumOfServosInRecording() )
  {
    LOGERROR("Record %d field count %d at maximum.", nRecNum, nFieldCnt);
    return END;
  }
  else if( !HasServoAt(nFieldCnt, nServoId) )
  {
    LOGERROR("Servo %d not found in recording header at field position %d.",
        nServoId, nFieldCnt);
    return END;
  }
  else
  {
    return m_record[nRecNum].AddField(nPos, nSpeed);
  }
}

/*!
 * \brief Register servo information in recording header.
 *
 * \note The order of registration is important and must match field order.
 * \note All relevant servos must be registered prior to adding records.
 *
 * \param nServoId    Servo Id.
 * \param uModelNum   Servo model number.
 *
 * \copydoc doc_return_std
 */
int DynaRecording::RegisterServoInfo(int nServoId, uint_t uModelNum)
{
  ServoInfo_T info;

  // duplicate servo registration
  if( m_mapIIdx.find(nServoId) != m_mapIIdx.end() )
  {
    return -DYNA_ECODE_BAD_VAL;
  }
  else if( (int)m_vecServoInfo.size() >= DYNA_ID_NUMOF )
  {
    return -DYNA_ECODE_RSRC;
  }

  info.m_nServoId   = nServoId;
  info.m_uModelNum  = uModelNum;

  m_vecServoInfo.push_back(info);
  m_mapIIdx[nServoId] = (int)m_vecServoInfo.size() - 1;

  return DYNA_OK;
}

/*!
 * \breif Set recording date string.
 *
 * \param sData   Recording date string. If NULL, then the current local time
 *                and date are used.
 */
void DynaRecording::SetDate(const char *sDate)
{
  size_t      n = 32;   // size of data buffer
  time_t      tnow;     // time now
  struct tm   bdtime;   // broken-down time

  // delete old recording date string
  DELOBJ(m_sDate);

  // default recording date string is 'now'
  if( sDate == NULL )
  {
    m_sDate = new char[n];
    tnow = time(NULL);
    if( (localtime_r(&tnow, &bdtime) == NULL) ||
        (strftime(m_sDate, n, "%a %b %H:%M:%S %Y", &bdtime) == 0) )
    {
      strcpy(m_sDate, "Unknown Date");
    }
  }

  // set recording date string
  else
  {
    m_sDate = newstr(sDate);
  }
}
