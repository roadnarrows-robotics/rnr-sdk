////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_recording.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Recording Class.
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

#ifndef _DYNASHELL_RECORDING_H
#define _DYNASHELL_RECORDING_H

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

#include "dynashell_util.h"

using namespace std;


// -----------------------------------------------------------------------------
// ServoInfo_T Structure
// -----------------------------------------------------------------------------

/*!
 * \brief Key Recorded Servo Information Structure Type.
 */
typedef struct
{
  int     m_nServoId;   ///< servo id
  uint_t  m_uModelNum;  ///< servo model number
} ServoInfo_T;


// -----------------------------------------------------------------------------
// DynaRecord Class
// -----------------------------------------------------------------------------

/*!
 * Dynamixel record.
 */
class DynaRecord
{
public:
  static const int END = -1;             ///< past-the-end mark

  /*!
   * \brief Record Field Tuple Structure Type.
   */
  typedef struct
  {
    int     m_nPos;       ///< servo position
    int     m_nSpeed;     ///< servo speed
  } FieldTuple_T;

  /*! 'Not a Field' field. */
  static const FieldTuple_T NoField;

  /*!
   * \brief Default constructor.
   */
  DynaRecord()
  {
    m_nFieldCnt = 0;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaRecord() { }

  /*!
   * \brief Get the current number of field tuples in record.
   *
   * \return Field tuple count.
   */
  int GetFieldCount() const
  {
    return m_nFieldCnt;
  }

  /*!
   * \brief Reset the field count to zero.
   */
  void ResetFieldCount()
  {
    m_nFieldCnt = 0;
  }

  virtual int AddField(int nPos, int nSpeed);

  /*!
   * \brief Test if field is 'Not a Field'.
   *
   * \param field   Field tuple to test.
   *
   * \return Returns true or false.
   */
  static const bool NaF(FieldTuple_T &field)
  {
    return  (field.m_nPos   == DynaRecord::NoField.m_nPos) &&
            (field.m_nSpeed == DynaRecord::NoField.m_nSpeed);
  }

  DynaRecord &operator=(const DynaRecord &rhs);

  /*!
   * \brief Subscript operator.
   *
   * \param nFldNum   Field number subscript.
   *
   * \return Field tuple.
   */
  FieldTuple_T const &operator[](const int nFldNum) const
  {
    return nFldNum < m_nFieldCnt? m_field[nFldNum]: NoField;
  }

protected:
    int           m_nFieldCnt;            ///< number of field tuples
    FieldTuple_T  m_field[DYNA_ID_NUMOF]; ///< one record  of field data

    friend class DynaRecording;
};


// -----------------------------------------------------------------------------
// DynaRecording Class
// -----------------------------------------------------------------------------

/*!
 * Dynamixel recording. The recording is usually made by a training session,
 * but does not have to be.
 */
class DynaRecording
{
public:
  static const int MaxRecords = 5000;    ///< maximum number of records 
  static const int END = -1;             ///< past-the-end mark

  /*!
   * \brief Default constructor.
   */
  DynaRecording()
  {
    m_sDate       = NULL;
    m_nRecordCnt  = 0;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaRecording()
  {
    m_vecServoInfo.clear();
    m_mapIIdx.clear();
    DELOBJ(m_sDate);
  }

  virtual void Init(int nSamplePeriod, const char *sDate = NULL);

  /*!
   * \brief Get the first record number in the recording.
   *
   * \return
   * If a recording is present, returns the first record number (0).\n
   * Otherwise \ref END is returned.
   */
  virtual int FirstRecord()
  {
    return m_nRecordCnt > 0? 0: END;
  }

  /*!
   * \brief Get the next record number in the recording after the given record
   * number.
   *
   * \param nRecNum   Record number.
   *
   * \return
   * If there is a next record present in the recording, returns the next
   * record number.\n
   * Otherwise \ref END is returned.
   */
  virtual int NextRecord(int nRecNum)
  {
    if( nRecNum == END )
    {
      return END;
    }
    ++nRecNum;
    return nRecNum < m_nRecordCnt? nRecNum: END;
  }

  virtual int AddRecord();

  /*!
   * \brief Copy source record to destination record.
   *
   * \param nRecNumDst    Destination record number.
   * \param nRecNumSrc    Source record number.
   *
   * \copydoc doc_return_std
   */
  virtual int CopyRecord(int nRecNumDst, int nRecNumSrc)
  {
    if( (nRecNumDst == END) || (nRecNumDst >= m_nRecordCnt) ||
        (nRecNumSrc == END) || (nRecNumSrc >= m_nRecordCnt) )
    {
      return -DYNA_ECODE_BAD_VAL;
    }

    m_record[nRecNumDst] = m_record[nRecNumSrc];

    return DYNA_OK;
  }

  /*!
   * \brief Get the first field number in the given record.
   *
   * \param nRecNum   Record number.
   *
   * \return
   * If a field is present, returns the first field number (0).\n
   * Otherwise \ref END is returned.
   */
  virtual int FirstField(int nRecNum)
  {
    if( (nRecNum < m_nRecordCnt) && (m_record[nRecNum].GetFieldCount() > 0) )
    {
      return 0;
    }
    else
    {
      return END;
    }
  }

  /*!
   * \brief Get the next field number in the record after the given field
   * number.
   *
   * \param nRecNum   Record number.
   * \param nFldNum   Field number.
   *
   * \return
   * If there is a next field present in the record, returns the next
   * field number.\n
   * Otherwise \ref END is returned.
   */
  virtual int NextField(int nRecNum, int nFldNum)
  {
    if( (nFldNum == END) || (nRecNum >= m_nRecordCnt) )
    {
      return END;
    }

    ++nFldNum;

    return nFldNum < m_record[nRecNum].GetFieldCount()? nFldNum: END;
  }

  virtual int AddFieldTuple(int nRecNum, int nServoId, int nPos, int nSpeed);

  /*!
   * \brief Get the given recorded field tuple.
   *
   * \param nRecNum   Record number.
   * \param nFldNum   Field number.
   *
   * \return
   * On success, returns the specified field tuple.\n
   * On failure, returns \ref DynaRecord::NoField.
   * (use \ref DynaRecord::NaF() to test);
   */
  virtual const DynaRecord::FieldTuple_T &GetField(const int nRecNum,
                                                   const int nFldNum) const
  {
    if( (nRecNum < m_nRecordCnt) &&
        (nFldNum < m_record[nRecNum].GetFieldCount()) )
    {
      return m_record[nRecNum][nFldNum];
    }
    else
    {
      DynaRecord::NoField;
    }
  }

  /*!
   * \brief Get the number of records in the recording.
   *
   * \reuturn Number of records.
   */
  virtual int GetNumOfRecords()
  {
    return m_nRecordCnt;
  }

  /*!
   * \brief Get the number of servos in the recording.
   *
   * The number of servos is the count of registered servos in the recording
   * header.
   *
   * \return Number of servos.
   */
  virtual int GetNumOfServosInRecording()
  {
    return (int)m_vecServoInfo.size();
  }

  virtual int RegisterServoInfo(int nServoId, uint_t uModelNum);

  /*!
   * \brief Get the servo id associated with the given field number.
   *
   * \return
   * On success, returns the servo id. Otherwise \ref DYNA_ID_NONE is returned.
   */
  virtual const int GetServoId(int nFldNum)
  {
    return  (nFldNum < (int)m_vecServoInfo.size())?
            m_vecServoInfo[nFldNum].m_nServoId: DYNA_ID_NONE;
  }

  /*!
   * \brief Check if the given servo is associated with the given field number,
   * as per the ordering of the registered servos in the recording header.
   *
   * \param nFldNum   Field number.
   * \param nServoid  Servo Id.
   *
   * \return Returns true or false.
   */
  virtual bool HasServoAt(int nFldNum, int nServoId) const
  {
    return  (nFldNum < (int)m_vecServoInfo.size()) &&
            (m_vecServoInfo[nFldNum].m_nServoId == nServoId);
  }

  /*!
   * \brief Check if the given servo is in the list of registered servos in the
   * recording header.
   *
   * \param nServoid  Servo Id.
   *
   * \return Returns true or false.
   */
  virtual bool HasServo(int nServoId)
  {
    return m_mapIIdx.find(nServoId) != m_mapIIdx.end();
  }

  /*!
   * \brief Get the registered servo model number.
   *
   * \param nServoid  Servo Id.
   *
   * \return Returns the model number. If no servo is found returns
   * \ref DYNA_MODEL_NUM_GENERIC.
   */
  virtual uint_t GetServoModelNumber(int nServoId)
  {
    MapIIdx::iterator iter;

    if( (iter = m_mapIIdx.find(nServoId)) != m_mapIIdx.end() )
    {
      return m_vecServoInfo[m_mapIIdx[nServoId]].m_uModelNum;
    }
    else
    {
      return DYNA_MODEL_NUM_GENERIC;
    }
  }

  /*!
   * \brief Get the sample period of the recording.
   *
   * \return Returns the sample period in milliseconds.
   */
  virtual int GetSamplePeriod() const
  {
    return m_nSamplePeriod;
  }

  /*!
   * \brief Set the sample period of the recording.
   *
   * \param nSamplePeriod Sample period in milliseconds.
   */
  virtual int SetSamplePeriod(int nSamplePeriod)
  {
    if( nSamplePeriod < 1 )
    {
      nSamplePeriod = 1;
    }
    m_nSamplePeriod = nSamplePeriod;
  }

  /*!
   * \brief Get the recording data.
   *
   * \return Returns date string.
   */
  virtual const char *GetDate() const
  {
    return m_sDate;
  }

  virtual void SetDate(const char *sDate);

  /*!
   * \brief Subscript operator.
   *
   * \param nRecNum   Record number subscript.
   *
   * \return DynaRecord.
   */
  DynaRecord const &operator[](const int nRecNum) const
  {
    // TODO return nRecNum < m_nFieldCnt? m_record[nRecNum]: NoRecord;
    return m_record[nRecNum];
  }

protected:

  /*!
   * \brief Map of record servos ids and model numbers indexed by field number.
   */
  typedef vector<ServoInfo_T> VecServoInfo;

  /*!
   * \brief Indirect indexing by servo id of servo info map.
   */
  typedef map<int,int>   MapIIdx;

  char         *m_sDate;              ///< recording date
  VecServoInfo  m_vecServoInfo;       ///< vector of recorded servo information
  MapIIdx       m_mapIIdx;            ///< indirect index map
  int           m_nSamplePeriod;      ///< recording sample period (ms)
  int           m_nRecordCnt;         ///< number of records recorded
  DynaRecord    m_record[MaxRecords]; ///< the recording data
};


#endif // _DYNASHELL_RECORDING_H
