////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_train.c
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief Dynamixel shell record and playback functions.
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

#include <sys/select.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>

#include <math.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaPidSpeed.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaOlio.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_recording.h"
#include "dynashell_util.h"

using namespace std;


//-----------------------------------------------------------------------------
// Private Interface
//-----------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Helper Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Block, waiting for either timeout or user keyboard press.
 *
 * No characters are read.
 *
 * \param nMSec   Wait period (ms)
 *
 * \return Returns true if wait was interrupted by keyboard press. Else returns
 * false on time out.
 */
static bool_t waitkey(int nMSec)
{
   int            fno = fileno(stdin);
   fd_set         fdset;
   uint_t         usec = (uint_t)nMSec * 1000;
   struct timeval timeout;
   int            rc;

   FD_ZERO(&fdset);
   FD_SET(fno, &fdset);
   
   // timeout (gets munged after each select())
   timeout.tv_sec  = (time_t)(usec / 1000000);
   timeout.tv_usec = (time_t)(usec % 1000000);

   if( (rc = select(fno+1, &fdset, NULL, NULL, &timeout)) > 0 )
   {
     fflush(stdin);
   }

   return rc>0? true: false;
}

/*!
 * \brief Block, waiting for either timeout or user keyboard press.
 *
 * No characters are read.
 *
 * \param fSec    Wait period (seconds)
 *
 * \return Returns true if wait was interrupted by keyboard press. Else returns
 * false on time out.
 */
static bool_t waitkey(double fSec)
{
   int            fno = fileno(stdin);
   fd_set         fdset;
   struct timeval timeout;
   int            rc;

   FD_ZERO(&fdset);
   FD_SET(fno, &fdset);
   
   // timeout (gets munged after each select())
   timeout.tv_sec  = (time_t)((int)fSec);
   timeout.tv_usec = (time_t)(((fSec - (double)timeout.tv_sec)) * 1000000.0);

   if( (rc = select(fno+1, &fdset, NULL, NULL, &timeout)) > 0 )
   {
     fflush(stdin);
   }

   return rc>0? true: false;
}


// -----------------------------------------------------------------------------
// DynaShellCmdGetPid Class
// -----------------------------------------------------------------------------

/*!
 * \brief Get the PID values for the given servos.
 */
class DynaShellCmdGetPid : public DynaShellCmdChainIn
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdGetPid()
  {
    m_sCmdName      = "pid";
    m_sCmdHelpBrief = "Get the current servo PID values.";
    m_sCmdHelpArgs  = "<servo_id> [servo_id...]\nchain";
    m_sCmdHelpDesc  = "Get the current PID Kp, Ki, and Kd parameters for the "
                      "specified servos. "
                      "If the keyword 'chain' is specified, then the PID "
                      "parameters for all of the servos in the chain are "
                      "retrieved.\n"
                      "  <servo_id>  Servo id [0-253].";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdGetPid() { }


protected:

  /*!
   * \brief Get servo PID parameters.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo)
  {
    printf("Servo %3d: Kp %9.3lf, Ki %9.3lf, Kd %9.3lf\n",
        pServo->GetServoId(), pServo->GetSpeedPid().GetKp(),
        pServo->GetSpeedPid().GetKi(), pServo->GetSpeedPid().GetKd());
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdSetPid Class
// -----------------------------------------------------------------------------
/*!
 * \brief Get the PID values for the given servos.
 */
class DynaShellCmdSetPid : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdSetPid() : DynaShellCmd(4, 4)
  {
    m_sCmdName      = "pid";
    m_sCmdHelpBrief = "Set PID constants for the given servo(s).";
    m_sCmdHelpArgs  = "<servo_id> <Kp> <Ki> <Kd>\nchain <Kp> <Ki> <Kd>";
    m_sCmdHelpDesc  = "Set the PID Kp, Ki, and Kd parameters for the "
                      "specified servo(s). "
                      "If the keyword 'chain' is specified, then the PID "
                      "parameters for all of the servos in the chain are "
                      "set.\n"
                      "  <servo_id>  Servo id [0-253].\n"
                      "  <Kp>        Proportional constant.\n"
                      "  <Ki>        Integral constant.\n"
                      "  <Kd>        Derivative constant.";
  }

  /*
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdSetPid() { }

  /*!
   * \brief Execute 'read-like' command on servos.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    bool        bAll;
    int         nServoId;
    double      fKp;
    double      fKi;
    double      fKd;
    int         i;
    int         iter;
    DynaServo  *pServo;

    TRY( ChkArgCnt(shell, argc) );

    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    for(i=0; i<argc; ++i)
    {
      switch( i )
      {
        case 0:
          if( !strcmp(argv[i], "chain") )
          {
            bAll = true;
          }
          else
          {
            TRY( ToInt(shell, argv[i], &nServoId) );
            TRY( ChkChainHasServo(shell, nServoId) );
            bAll = false;
          }
          break;
        case 1:
          TRY( ToDouble(shell, argv[i], &fKp) );
          break;
        case 2:
          TRY( ToDouble(shell, argv[i], &fKi) );
          break;
        case 3:
          TRY( ToDouble(shell, argv[i], &fKd) );
          break;
        default:
          shell.Error("Huh?");
          return;
      }
    }

    if( bAll )
    {
      for(nServoId = shell.m_pDynaChain->IterStart(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter))
      {
        pServo = shell.m_pDynaChain->GetServo(nServoId);
        doExec(shell, pServo, fKp, fKi, fKd);
      }
    }
    else
    {
      pServo = shell.m_pDynaChain->GetServo(nServoId);
      doExec(shell, pServo, fKp, fKi, fKd);
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL <servo_id> | chain [servo_id [servo_id ...]]
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    int   nArgNum;
    char  buf[16];
  
    if( (shell.m_pDynaChain == NULL) ||
        (shell.m_pDynaChain->GetNumberInChain() == 0) )
    {
      return NULL;
    }
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    // only first argument can be expanded
    if( nArgNum > 0 )
    {
      return NULL;
    }

    //
    // New command to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
    }
  
    while( m_nTabServoId != DYNA_ID_NONE )
    {
      snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
      buf[sizeof(buf)-1] = 0;
  
      m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
  
      if( !strncmp(buf, sText, uTextLen) )
      {
        return ReadLine::dupstr(buf);
      }
    }
  
    if( !strncmp("chain", sText, uTextLen) )
    {
      return ReadLine::dupstr("chain");
    }
  
    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id

  /*!
   * \brief Get servo PID parameters.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo.
   * \param fKp     PID proportional constant.
   * \param fKi     PID integral constant.
   * \param fKd     PID derivative constant.
   */
  virtual void doExec(DynaShell &shell,
                      DynaServo *pServo,
                      double     fKp,
                      double     fKi,
                      double     fKd)
  {
    pServo->GetSpeedPid().SetConstants(fKp, fKi, fKd);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdLoadRecording Class
// -----------------------------------------------------------------------------

/*!
 * \brief Load recording from file.
 */
class DynaShellCmdLoadRecording : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdLoadRecording() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "recording";
    m_sCmdHelpBrief = "Load recording from a file.";
    m_sCmdHelpArgs  = "<file>";
    m_sCmdHelpDesc  = "Load a recording from a file. The current recording in "
                      "memory is overwritten.\n"
                      "  <file>    Recording file name.";

    m_fp            = NULL;
    m_sFileName     = NULL;
    m_nLineNum      = 0;
    m_nColNum       = 0;
    m_sCursor       = NULL;
    m_sField        = NULL;
    m_sizeBuf       = 0;
    m_buf           = NULL;

    m_sDate         = NULL;
    m_nNumServos    = 0;
    m_nNumRecords   = 0;
    m_nSamplePeriod = 0;
    m_pRecording    = NULL;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdLoadRecording() { }

  /*!
   * \brief Execute load recording.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    FILE  *fp;

    TRY( ChkArgCnt(shell, argc) );

    if( Init(shell, argv[0]) != DYNA_OK )
    {
      return;
    }

    Load(shell);

    Cleanup(shell);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <file>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    return ReadLine::FileCompletionGenerator(sText, nState);
  }

protected:
  FILE           *m_fp;             ///< open file pointer
  char           *m_sFileName;      ///< file name
  int             m_nLineNum;       ///< line number
  int             m_nColNum;        ///< column number in line
  char           *m_sCursor;        ///< current line parse cursor
  char           *m_sField;         ///< current field being parsed
  char           *m_buf;            ///< input buffer
  size_t          m_sizeBuf;        ///< size of input buffer

  int             m_nNumServos;     ///< NUM_SERVOS value
  int             m_nNumRecords;    ///< NUM_RECORDS value
  int             m_nSamplePeriod;  ///< SAMPLE_PERIOD value
  char           *m_sDate;          ///< DATE value
  DynaRecording  *m_pRecording;     ///< working and new recording

  /*!
   * \brief Load recording.
   *
   * \param shell   Dynamixel shell.
   */
  void Load(DynaShell &shell)
  {
    int     n;
    int     rc = DYNA_OK;

    while( (rc == DYNA_OK) && GetLine() )
    {
      m_sField = GetFirstWord();

      // empty line
      if( m_sField == NULL )
      {
        continue;
      }

      // comment
      else if( *m_sField == '#' )
      {
        continue;
      }

      // version - ignored for now
      else if( !strcmp(m_sField, "VERSION") )
      {
        continue;
      }

      // recording date
      else if( !strcmp(m_sField, "DATE") )
      {
        SetDateField(shell, GetEolPhrase());
      }

      // number of servos recorded
      else if( !strcmp(m_sField, "NUM_SERVOS") )
      {
        if( ParseIntField(shell, GetNextWord(), &m_nNumServos) != DYNA_OK )
        {
          rc = -DYNA_ECODE_PARSE;
        }
        else if( (m_nNumServos <= 0) || (m_nNumServos > DYNA_ID_NUMOF) )
        {
          shell.Error("Line %dc%d: Field %s: Value %d: Out-of-range.",
              m_nLineNum, m_nColNum, m_sField, m_nNumServos);
          rc = -DYNA_ECODE_PARSE;
        }
      }

      // list of servo id, model number pairs
      else if( !strcmp(m_sField, "SERVOS") )
      {
        rc = ParseServoList(shell) != DYNA_OK;
      }

      // sample period (msec)
      else if( !strcmp(m_sField, "SAMPLE_PERIOD") )
      {
        if( ParseIntField(shell, GetNextWord(), &m_nSamplePeriod) != DYNA_OK )
        {
          rc = -DYNA_ECODE_PARSE;
        }
        else if( m_nSamplePeriod <= 0 )
        {
          shell.Error("Line %dc%d: Field %s: Value %d: Out-of-range.",
              m_nLineNum, m_nColNum, m_sField, m_nNumServos);
          rc = -DYNA_ECODE_PARSE;
        }
        else
        {
          m_pRecording->SetSamplePeriod(m_nSamplePeriod);
        }
      }

      // number of records
      else if( !strcmp(m_sField, "NUM_RECORDS") )
      {
        if( ParseIntField(shell, GetNextWord(), &m_nNumRecords) != DYNA_OK )
        {
          rc = -DYNA_ECODE_PARSE;
        }
        else if( (m_nNumRecords <= 0) ||
                 (m_nNumRecords > DynaRecording::MaxRecords) )
        {
          shell.Error("Line %dc%d: Field %s: Value %d: Out-of-range.",
              m_nLineNum, m_nColNum, m_sField, m_nNumServos);
          rc = -DYNA_ECODE_PARSE;
        }
      }

      // recorded servo data - last extended field
      else if( !strcmp(m_sField, "DATA") )
      {
        //
        // Final checks and parsing
        //
        if( m_nNumServos == 0 )
        {
          shell.Error("Field NUM_SERVOS: Not specified in file header.",
              m_sField);
          rc = -DYNA_ECODE_PARSE;
        }

        else if( m_pRecording->GetNumOfServosInRecording() != m_nNumServos )
        {
          shell.Error(
              "Field SERVOS lists %d servos != Field NUM_SERVOS value of %d.",
            m_pRecording->GetNumOfServosInRecording(), m_nNumServos);
          rc = -DYNA_ECODE_PARSE;
        }

        else if( m_nNumRecords == 0 )
        {
          shell.Error("Field NUM_RECORDS: Not specified in file header.");
          rc = -DYNA_ECODE_PARSE;
        }

        else if( m_nSamplePeriod == 0 )
        {
          shell.Error("Field SAMPLE_PERIOD: Not specified in file header.");
          rc = -DYNA_ECODE_PARSE;
        }

        else
        {
          rc = ParseRecordedData(shell);
        }

        break;
      }

      else
      {
        shell.Warning("Line %d: Field %s: Unknown field - ignoring.", 
            m_nLineNum, m_sField);
      }
    }

    //
    // If ok, then the recording was successfully loaded from the file.
    //
    if( rc == DYNA_OK )
    {
      if( m_sDate == NULL )
      {
        shell.Warning("Field DATE: Not specified in file header.");
        SetDateField(shell, "Unknown File Date");
      }

      shell.RecordingReplace(m_pRecording);
      m_pRecording = NULL; // make sure the cleanup does not delete this

      shell.Response("Loaded recording from file %s\n", m_sFileName);
      shell.Response("  Recording date:    %s.\n",
          shell.m_pRecording->GetDate());
      shell.Response("  Number of servos:  %d\n",
        shell.m_pRecording->GetNumOfServosInRecording());
      shell.Response("  Number of records: %d\n",
        shell.m_pRecording->GetNumOfRecords());
      shell.Response("  Sample period:     %dms.\n",
        shell.m_pRecording->GetSamplePeriod());
    }
  }

  /*!
   * \brief Parse servo list and save in working recording.
   *
   * \param shell   Dynamixel shell.
   *
   * \copydoc doc_return_std
   */
  int ParseServoList(DynaShell &shell)
  {
    char   *sWord;
    int     nServoId;
    uint_t  uModelNum;
    int     rc = DYNA_OK;
   
    while( (rc == DYNA_OK) && ((sWord = GetNextWord()) != NULL) )
    {
      // 2-tuple servo_id, model_num
      if( (rc = ParseIntField(shell, sWord, &nServoId)) == DYNA_OK )
      {
        rc = ParseIntField(shell, GetNextWord(), (int *)&uModelNum);
      }

      if( rc != DYNA_OK )
      {
      }

      else if( (nServoId < DYNA_ID_MIN) || (nServoId > DYNA_ID_MAX) )
      {
        shell.Error("Line %dc%d: Field %s: Value %d': Out-of-range.",
             m_nLineNum, m_nColNum, m_sField, nServoId);
        rc = -DYNA_ECODE_PARSE;
      }

      else
      {
        rc = m_pRecording->RegisterServoInfo(nServoId, uModelNum);

        if( rc != DYNA_OK )
        {
          shell.Error("Line %dc%d: Field %s: Servo %d: Duplicate.",
                m_nLineNum, m_nColNum, m_sField, nServoId);
        }
      }
    }

    return rc;
  }

  /*!
   * \brief Parse recorded data and save in working recording.
   *
   * \param shell   Dynamixel shell.
   *
   * \copydoc doc_return_std
   */
  int ParseRecordedData(DynaShell &shell)
  {
    int     nRecNum;
    int     nFldNum;
    char   *sWord;
    int     nServoId;
    int     nPos;
    int     nSpeed;
    int     rc;
   
    while( GetLine() )
    {
      if( LineHasNoData() )
      {
        continue;
      }

      m_sField = (char *)"DATA";    // multiline parsing clobbers this

      nRecNum = m_pRecording->AddRecord();
     
      if( nRecNum == DynaRecording::END )
      {
        shell.Error(
            "Line %d: Field DATA: Exceeded maximum number of %d records.",
            m_nLineNum, m_sField, DynaRecording::MaxRecords);
        return -DYNA_ECODE_PARSE;
      }

      else if( nRecNum >= m_nNumRecords )
      {
        shell.Error(
            "Line %d: Field DATA: Exceeded NUM_RECORDS value of %d records.",
              m_nLineNum, m_sField, m_nNumRecords);
        return -DYNA_ECODE_PARSE;
      }

      for(sWord = GetFirstWord(), nFldNum=0;
          sWord != NULL;
          sWord = GetNextWord(), ++nFldNum)
      {
        if( nFldNum >= m_nNumServos )
        {
          shell.Error(
              "Line %dc%d: Field %s: Exceeded NUM_SERVOS value of %d servos.",
              m_nLineNum, m_sField, m_nColNum, m_nNumServos);
          return -DYNA_ECODE_PARSE;
        }

        // read 2-tuple position, speed
        if( (rc = ParseIntField(shell, sWord, &nPos)) != DYNA_OK )
        {
          return -DYNA_ECODE_PARSE;
        }

        if( (rc = ParseIntField(shell, GetNextWord(), &nSpeed)) != DYNA_OK )
        {
          return -DYNA_ECODE_PARSE;
        }

        if( (nServoId = m_pRecording->GetServoId(nFldNum)) == DYNA_ID_NONE )
        {
          shell.Error("Line %dc%d: Field %s: No registered servo at pos %d.",
              m_nLineNum, m_nColNum, m_sField, nFldNum);
          return -DYNA_ECODE_PARSE;
        }

        rc = m_pRecording->AddFieldTuple(nRecNum, nServoId, nPos, nSpeed);

        if( rc == DynaRecording::END )
        {
          shell.Error("Line %dc%d: Field %s: Cannot add any more fields.",
              m_nLineNum, m_nColNum, m_sField);
          return -DYNA_ECODE_PARSE;
        }
      }

      if( nFldNum < m_nNumServos )
      {
        shell.Error(
          "Line %d: Field %s: Only %d (pos,speed) 2-tuples found, expected %d.",
          m_nLineNum, m_sField, nFldNum, m_nNumServos);
        return -DYNA_ECODE_PARSE;
      }
    }

    if( nRecNum+1 < m_nNumRecords )
    {
      shell.Error("Line %d: Field %s: Only %d records found, expected %d.",
            m_nLineNum, m_sField, nRecNum+1, m_nNumRecords);
      return -DYNA_ECODE_PARSE;
    }

    return DYNA_OK;
  }

  /*!
   * \brief Set date value.
   *
   * \param shell   Dynamixel shell.
   * \param sDate   Date string.
   *
   * \copydoc doc_return_std
   */
  void SetDateField(DynaShell &shell, const char *sDate)
  {
    if( m_sDate != NULL )
    {
      delete[] m_sDate;
      m_sDate = NULL;
    }
    if( (sDate != NULL) && (*sDate != 0) )
    {
      m_sDate = newstr(sDate);
      m_pRecording->SetDate(m_sDate);
    }
  }

  /*!
   * \brief Parse integer field.
   *
   * String field can be specified in hexidecimal, decimal, or octal.
   *
   * \param shell             Dynamixel shell.
   * \param sWord             Word to parse.
   * \param [out] pFieldVal   Converted value.
   *
   * \copydoc doc_return_std
   */
  int ParseIntField(DynaShell &shell, const char *sWord, int *pFieldVal)
  {
    if( sWord == NULL )
    {
      shell.Error("Line %dc%d: Field %s: No field value found.",
          m_nLineNum, m_nColNum, m_sField);
      return -DYNA_ECODE_PARSE;
    }
    else if( sscanf(sWord, "%i", pFieldVal) != 1 )
    {
      shell.Error("Line %dc%d: Field %s: Value %s: Not an integer.",
          m_nLineNum, m_nColNum, m_sField, sWord);
      return -DYNA_ECODE_PARSE;
    }
    else
    {
      return DYNA_OK;
    }
  }

  /*!
   * \brief Get the first word in line buffer.
   *
   * A word is defined as characters separated by white space.
   *
   * \return
   * Returns pointer to start of word if a word is found.\n
   * Else returns NULL.
   */
  char *GetFirstWord()
  {
    m_sCursor = m_buf;
    m_nColNum = 1;

    return GetNextWord();
  }

  /*!
   * \brief Get the next word in line buffer.
   *
   * A word is defined as characters separated by white space.
   *
   * \return
   * Returns pointer to start of word if a word is found.\n
   * Else returns NULL.
   */
  char *GetNextWord()
  {
    char    *sWord;

    m_nColNum = (int)(m_sCursor - m_buf) + 1;

    while( *m_sCursor && isspace((int)*m_sCursor) )
    {
      ++m_sCursor;
      ++m_nColNum;
    }

    if( *m_sCursor == 0 )
    {
      return NULL;
    }

    sWord = m_sCursor;

    while( *m_sCursor && !isspace((int)*m_sCursor) )
    {
      ++m_sCursor;
    }

    if( *m_sCursor != 0 )
    {
      *m_sCursor++ = 0;
    }

    //fprintf(stderr, "DBG: word=%s\n", sWord);

    return sWord;
  }

  /*!
   * \brief Get the end-of-line phrase
   *
   * The eol phrase is defined as a set of characters bracketed by non-white
   * space characters.
   *
   * \return
   * Returns pointer to start of phrase if a phrase is found.\n
   * Else returns NULL.
   */
  char *GetEolPhrase()
  {
    char    *sPhrase;
    char    *s;

    m_nColNum = (int)(m_sCursor - m_buf) + 1;

    while( *m_sCursor && isspace((int)*m_sCursor) )
    {
      ++m_sCursor;
      ++m_nColNum;
    }

    if( *m_sCursor == 0 )
    {
      return NULL;
    }

    sPhrase = m_sCursor;

    while( *m_sCursor && (*m_sCursor != '\n') )
    {
      ++m_sCursor;
    }

    if( *m_sCursor != 0 )
    {
      ++m_sCursor;
    }

    s = m_sCursor - 1;

    while( (s > sPhrase) && isspace((int)*s) )
    {
      --s;
    }

    *++s = 0;

    //fprintf(stderr, "DBG: phrase=%s\n", sPhrase);

    return sPhrase;
  }

  /*!
   * \brief Test if line is all white space or a comment.
   *
   * \return Returns true if line contains no data, false otherwise.
   */
  bool LineHasNoData()
  {
    char  *s;

    for(s=m_buf; *s; ++s)
    {
      if( *s == '#' )
      {
        return true;
      }
      else if( !isspace((int)*s) )
      {
        return false;
      }
    }
    return true;
  }

  /*!
   * \brief Read the next line of data.
   *
   * \return Returns true if line found, false otherwise.
   */
  bool GetLine()
  {
    if( getline(&m_buf, &m_sizeBuf, m_fp) != -1 )
    {
      m_nLineNum++;
      m_nColNum = 0;
      return true;
    }
    else
    {
      return false;
    }
  }

  /*!
   * \brief Initialize data prior to file loading.
   *
   * \param shell       Dynamixel shell.
   * \param sFileName   File to open.
   *
   * \copydoc doc_return_std
   */
  int Init(DynaShell &shell, char *sFileName)
  {
    if( (m_fp = fopen(sFileName, "r")) == NULL ) 
    {
      shell.Error("%s: cannot open.", sFileName);
      return -DYNA_ECODE_BADF;
    }

    m_sFileName     = newstr(sFileName);
    m_nLineNum      = 0;
    m_nColNum       = 0;
    m_sizeBuf       = 4096;
    m_buf           = new char[m_sizeBuf];

    m_sDate         = NULL;
    m_nNumServos    = 0;
    m_nNumRecords   = 0;
    m_nSamplePeriod = 0;
    m_pRecording    = new DynaRecording;

    return DYNA_OK;
  }

  /*!
   * \brief Clean up data after to file loading parsing.
   *
   * \param shell       Dynamixel shell.
   */
  void Cleanup(DynaShell &shell)
  {
    if( m_fp != NULL)
    {
      fclose(m_fp);
      m_fp = NULL;
    }

    if( m_sFileName != NULL )
    {
      delete[] m_sFileName;
      m_sFileName = NULL;
    }

    if( m_buf != NULL )
    {
      delete[] m_buf;
      m_buf = NULL;
    }
 
    if( m_sDate != NULL )
    {
      delete[] m_sDate;
      m_sDate = NULL;
    }

    // load failed, delete associated recording
    if( m_pRecording != NULL )
    {
      delete m_pRecording;
      m_pRecording = NULL;
    }

    m_nLineNum      = 0;
    m_nColNum       = 0;
    m_sCursor       = NULL;
    m_sField        = NULL;
    m_nNumServos    = 0;
    m_nNumRecords   = 0;
    m_nSamplePeriod = 0;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdSaveRecording Class
// -----------------------------------------------------------------------------

/*!
 * \brief Save recording to file.
 */
class DynaShellCmdSaveRecording : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdSaveRecording() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "recording";
    m_sCmdHelpBrief = "Save the current recording to a file.";
    m_sCmdHelpArgs  = "<file>";
    m_sCmdHelpDesc  = "Save the current recording to an ASCII file."
                      "The current recording is the last 'train' or 'load' "
                      "command executed during this shell's session.\n"
                      "  <file>    Recording file name.";
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdSaveRecording() { }

  /*!
   * \brief Execute save recording.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    FILE  *fp;

    TRY( ChkArgCnt(shell, argc) );

    if( (shell.m_pRecording == NULL) ||
        (shell.m_pRecording->GetNumOfRecords() == 0) )
    {
      shell.Error("No recording to save.");
      return;
    }

    if( (fp = fopen(argv[0], "w")) == NULL ) 
    {
      shell.Error("%s: cannot open.", argv[1]);
      return;
    }

    Save(shell, fp);

    fclose(fp);

    shell.Response("Saved recording to file %s\n", argv[0]);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <file>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    return ReadLine::FileCompletionGenerator(sText, nState);
  }

protected:

  /*!
   * \brief Save recording.
   *
   * \param shell   Dynamixel shell.
   * \param fp      File pointer.
   */
  void Save(DynaShell &shell, FILE *fp)
  {
    DynaRecording            *pRecording = shell.m_pRecording;
    int                       nRecNum;
    int                       nFldNum;
    int                       nServoId;
    DynaRecord::FieldTuple_T  tupRec;

    fprintf(fp, "# .DYD v1.0 - Dynamixel Data file format\n");
    fprintf(fp, "VERSION 1.0\n");
    fprintf(fp, "DATE %s\n", pRecording->GetDate());
    fprintf(fp, "NUM_SERVOS %d\n", pRecording->GetNumOfServosInRecording());
    fprintf(fp, "SERVOS");
  
    for(nFldNum = pRecording->FirstField(0);
        nFldNum != DynaRecording::END;
        nFldNum = pRecording->NextField(0, nFldNum))
    {
      nServoId = pRecording->GetServoId(nFldNum);
      fprintf(fp, " %d 0x%04x",
          nServoId,
          pRecording->GetServoModelNumber(nServoId));
    }
    fprintf(fp, "\n");

    fprintf(fp, "SAMPLE_PERIOD %d\n", pRecording->GetSamplePeriod());
    fprintf(fp, "NUM_RECORDS %d\n", pRecording->GetNumOfRecords());
    fprintf(fp, "DATA ascii\n");

    for(nRecNum = pRecording->FirstRecord();
        nRecNum != DynaRecording::END;
        nRecNum = pRecording->NextRecord(nRecNum))
    {
      for(nFldNum = pRecording->FirstField(nRecNum);
          nFldNum != DynaRecording::END;
          nFldNum = pRecording->NextField(nRecNum, nFldNum))
      {
        tupRec = pRecording->GetField(nRecNum, nFldNum);
        fprintf(fp, " %4d %-4d", tupRec.m_nPos, tupRec.m_nSpeed);
      }
      fprintf(fp, "\n");
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdTrain Class
// -----------------------------------------------------------------------------

/*!
 * \brief Record dynamixel chain movements.
 */
class DynaShellCmdTrain : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdTrain() : DynaShellCmd(2, 2)
  {
    m_sCmdName      = "train";
    m_sCmdHelpBrief = "Record a training session.";
    m_sCmdHelpArgs  = "<start_delay> <sample_period>";
    m_sCmdHelpDesc  = "Record a dynamixel chain training session. "
                      "The current recording in memory is overwritten.\n"
                      "  <start_delay>    Recording start delay (sec).\n"
                      "  <sample_period>  Recording sample period (msec).";
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdTrain() { }

  /*!
   * \brief Execute recording.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int         nStartDelay;
    int         nSamplePeriod;
    int         iter;
    int         nServoId;
    DynaServo  *pServo;

    //
    // Parse arguments
    //
    TRY( ChkArgCnt(shell, argc) );
    TRY( ToInt(shell, argv[0], &nStartDelay) );
    TRY( ToInt(shell, argv[1], &nSamplePeriod) );

    //
    // Check state
    //
    TRY( ChkComm(shell) );
    TRY( ChkChainNotEmpty(shell) );

    
    // initialize recording data
    shell.RecordingInit(nSamplePeriod);

    // register servos in chain to be trained with recording header
    for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = shell.m_pDynaChain->IterNextMaster(&iter))
    {
      pServo = shell.m_pDynaChain->GetServo(nServoId);
      shell.m_pRecording->RegisterServoInfo(nServoId, pServo->GetModelNumber());
    }

    // record training session
    RecordTraining(shell, nStartDelay, nSamplePeriod);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL NULL <loop>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    // direction keyword table
    static const char *keywordsLoop[] = {"close", "open"}; 

    int         nArgNum;
    const char *s;
    char        buf[16];
  
    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    if( nArgNum != 2 )
    {
      return NULL;
    }

    //
    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIndex   = 0;
    }
  
    while( m_nTabIndex < arraysize(keywordsLoop) )
    {
      s = keywordsLoop[m_nTabIndex++];

      if( !strncmp(s, sText, uTextLen) )
      {
        return ReadLine::dupstr(s);
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int  m_nTabIndex;       ///< tab completion: keyword index

  /*!
   * \brief Record the movements of a dynamixel chain.
   *
   * \param shell               Dynamixel shell.
   * \param nStartDelay         Delay in seconds, before recording starts.
   * \param nSamplePeriod       Recording sample period (ms)
   */
  void RecordTraining(DynaShell &shell,
                      int        nStartDelay,
                      int        nSamplePeriod)
  {
    DynaRecording  *pRecording = shell.m_pRecording;
    int             nRecNum;
    int             iter;
    int             nServoId;
    DynaServo      *pServo;
    int             nCurPos;
    int             nCurSpeed;
    int             rc;

    if( nSamplePeriod < 1 )
    {
      nSamplePeriod = 1;
    }

    printf("Starting to record in %u seconds.\n", nStartDelay);
    printf("  Recording sample rate: %dms.\n", nSamplePeriod);
    printf("Press <CR> to stop recording.\n\n");
    fflush(stdout);

    // count down
    for(iter=0; iter<nStartDelay; ++iter)
    {
      printf("%d ", nStartDelay-iter);
      fflush(stdout);
      sleep(1);
    }

    printf("Go\n\n");

    //
    // Add record at sampling rate
    //
    while( (nRecNum = pRecording->AddRecord()) != DynaRecording::END )
    {
      printf("record[%d]: ", nRecNum);

      //
      // Record current position and speed for each master servo in chain.
      //
      for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNextMaster(&iter))
      {
        pServo = shell.m_pDynaChain->GetServo(nServoId);

        // get current position
        rc = pServo->ReadCurPos(&nCurPos);

        if( rc != DYNA_OK )
        {
          shell.Error(rc, "Servo %d: ReadCurPos.", nServoId);
          return;
        }

        // RDK TODO check alarms for out-of-range servo positions
        
        // get current speed
        rc = pServo->ReadCurSpeed(&nCurSpeed);

        if( rc != DYNA_OK )
        {
          shell.Error(rc, "Servo %d: ReadCurSpeed.", nServoId);
          return;
        }

        // and field tuple to record
        pRecording->AddFieldTuple(nRecNum, nServoId, nCurPos, nCurSpeed);

        printf("%d %d %d   ", nServoId, nCurPos, nCurSpeed);
      }

      printf("\n");

      // recording sample period wait and user <CR> stop
      if( waitkey(nSamplePeriod) )
      {
        break;
      }
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdPlay Class
// -----------------------------------------------------------------------------

/*!
 * \brief Record dynamixel chain movements.
 */
class DynaShellCmdPlay : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdPlay() : DynaShellCmd(1, 3)
  {
    m_sCmdName      = "play";
    m_sCmdHelpBrief = "Playback the current dynamixel recording.";
    m_sCmdHelpArgs  = "<sub_sample> [<speed> [<plot_file>]]";
    m_sCmdHelpDesc  = "Playback the current recording. "
                      "The current recording is the last 'train' or 'load' "
                      "command executed during this shell's session.\n"
                      "  <sub_sample>  Number of control sub-sample points.\n"
                      "  <speed>       Playback at % of recorded speed.\n"   
                      "                Default: 100%\n"
                      "  <loop>           Do [not] loop recording.\n"
                      "                   One of: loop true noloop false.\n"
                      "                   Default: noloop.";
                      "  <plot_file>   Output plot file in gnuplot format.\n"
                      "                Default: no file";

    m_nNumSubSamplePts  = 0;
    m_fSpeedPct         = 0.0;
    m_fpPlot            = NULL;
    m_nPlotLineCnt      = 0;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdPlay() { }

  /*!
   * \brief Execute playback of recording.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[])
  {
    DynaRecording  *pRecording = shell.m_pRecording;
    int             i;
    char           *sPlotFileName;
    int             iter;
    int             nServoId;
    DynaServo      *pServo;

    //
    // Check and parse command-line arguments.
    //
    TRY( ChkArgCnt(shell, argc) );

    // Is there a recording?
    if( (pRecording == NULL) || (pRecording->GetNumOfRecords() == 0) )
    {
      shell.Error("No recording to play.");
      return;
    }

    // defaults
    m_fSpeedPct     = 100.0;
    sPlotFileName   = NULL;
    m_fpPlot        = NULL;

    // RDK loop option
    //
    // Parse arguments
    //
    for(i=0; i<argc; ++i)
    {
      switch( i )
      {
        // sub-sample points
        case 0:
          TRY( ToInt(shell, argv[i], &m_nNumSubSamplePts) );
          // sub-sample period too big - adjust
          if( m_nNumSubSamplePts < 1 )
          {
            m_nNumSubSamplePts = 1;
          }
          break;
        // playback speed (optional)
        case 1:
          TRY( ToDouble(shell, argv[i], &m_fSpeedPct) );
          if( m_fSpeedPct < 0.0 )
          {
            m_fSpeedPct = 100.0;
          }
          break;
        case 2:
          // create gnuplot file (optional)
          sPlotFileName = argv[i];
          break;
        default:
          shell.Error("Huh?");
          break;
      }
    }

    //
    // Validate recording against chain.
    //

    // Does the recording have the same number of servos?
    if( pRecording->GetNumOfServosInRecording() != 
        shell.m_pDynaChain->GetNumberOfMastersInChain() )
    {
      shell.Error("%d servos in recording != %d master servos in chain.", 
        pRecording->GetNumOfServosInRecording(),
        shell.m_pDynaChain->GetNumberOfMastersInChain());
      return;
    }

    // Do the servos match in id and model number?
    for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = shell.m_pDynaChain->IterNextMaster(&iter))
    {
      if( !pRecording->HasServo(nServoId) )
      {
        shell.Error("Servo %d: Not in recording.");
        return;
      }

      pServo = shell.m_pDynaChain->GetServo(nServoId);

      if(pRecording->GetServoModelNumber(nServoId) != pServo->GetModelNumber())
      {
        shell.Error("Servo %d: Recording model number 0x%04x != 0x%04x.",
            pRecording->GetServoModelNumber(nServoId),
            pServo->GetModelNumber());
        return;
      }
    }

    //
    // Open gnuplot file
    //
    if( sPlotFileName != NULL )
    {
      if( (m_fpPlot = fopen(sPlotFileName, "w")) == NULL ) 
      {
        shell.Error("%s: cannot open.", sPlotFileName);
        return;
       }
    }

    // initialiize 
    Init(shell);

    // play back the recording
    Play(shell);

    // clean up any resources
    Cleanup(shell);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes NULL 100 <plot_file>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    int   nArgNum;

    // argument number of already (expanded) arguments
    nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

    //
    // New command argument to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIndex   = 0;
    }
  
    // only match default playback <speed>
    if( nArgNum == 1 )
    {
      if( (m_nTabIndex++ == 0) && !strncmp("100", sText, uTextLen) )
      {
        return ReadLine::dupstr("100");
      }
    }
    
    // <plot_file>
    else if( nArgNum == 2 )
    {
      return ReadLine::FileCompletionGenerator(sText, nState);
    }

    return NULL;
  }

protected:
  typedef vector< vector<uint_t> >    VecCurves;

  int         m_nNumSubSamplePts; ///< sub-sample playback control period (msec)
  double      m_fSpeedPct;        ///< playback speed as a % of recorded speed
  double      m_fSamplePeriod;    ///< playback sample period (seconds)
  double      m_fSubSamplePeriod; ///< control sub-sample period (seconds)
  double      m_dt;               ///< playback delta time (seconds)
  VecCurves   m_vecCurves;        ///< vector of fitted smooth curves
  double      m_fTAccum;          ///< accumulated time
  FILE       *m_fpPlot;           ///< plot data file pointer
  int         m_nPlotLineCnt;     ///< plotted data line count
  int         m_nTabIndex;        ///< tab completion: keyword index

  /*!
   * \brief Play back the previously recorded Dynamixel chain motion sequence.
   *
   * \param shell   Dynamixel shell.
   */
  void Play(DynaShell &shell)
  {
    DynaRecording  *pRecording = shell.m_pRecording;
    double          dt;
    double          t;
    int             nRecNum;
    int             rc;
  
    shell.Response("\nPlaying back recording made on %s.\n",
        pRecording->GetDate());
    shell.Response("  Number of records:      %d\n",
        pRecording->GetNumOfRecords());
    shell.Response("  Playback sample period: %.1lfms.\n", m_fSamplePeriod);
    shell.Response("  Sub-sample period:      %.3lfms.\n", dt);
    shell.Response("Press <CR> to abort recording.\n\n");
    fflush(stdout);

    m_fTAccum = 0.0;

    // move to the starting position
    MoveToStart(shell);
  
    //
    // Play back the recording.
    //
    for(nRecNum = pRecording->FirstRecord();
        nRecNum != DynaRecording::END;
        nRecNum = pRecording->NextRecord(nRecNum))
    {
      // set record goals
      rc = SetRecordGoals(shell, nRecNum);
  
      if( rc != DYNA_OK )
      {
        return;
      }

      //
      // Now control the speed profiles at the sub-sampled rate.
      //
      for(t=0.0; t<m_fSamplePeriod; t+=dt)
      {
        // control servos to goals
        rc = ControlToGoals(shell, nRecNum, dt);
       
        if( rc != DYNA_OK )
        {
          return;
        }
  
        // wait for next time interval or user keypress to abort
        if( waitkey(dt) )
        {
          // RDK could be that user aborted because of badness SecureArm(shell);
          return;
        }

        m_fTAccum += dt;
      }
    }
  }

  /*!
   * \brief Set record goals.
   *
   * \li Set new PID setpoints
   * \li Initiate synchronous move of all servos to the recorded end position.
   *
   * \param shell     Dynamixel shell.
   * \param nRecNum   Record number in the recording.
   *
   * \copydoc doc_return_std
   */
  int SetRecordGoals(DynaShell &shell, int nRecNum)
  {
    DynaRecord::FieldTuple_T  tupRec;
    int                       nFldNum;
    int                       nServoId;
    DynaPosTuple_T            tupPos[DYNA_ID_NUMOF];
    uint_t                    uCount;
    int                       rc;
  
    printf("\n  goalpos_%d: ", nRecNum);

    //
    // Build synchronous write tuple while setting new pid setpoints.
    //
    for(nFldNum = shell.m_pRecording->FirstField(nRecNum), uCount=0;
        nFldNum != DynaRecording::END;
        nFldNum = shell.m_pRecording->NextField(nRecNum, nFldNum), ++uCount)
    {
      nServoId = shell.m_pRecording->GetServoId(nFldNum);

      tupRec = shell.m_pRecording->GetField(nRecNum, nFldNum);

      tupPos[nFldNum].m_nServoId = nServoId;
      tupPos[nFldNum].m_nPos     = tupRec.m_nPos;
  
      rc = PidSetPoint(shell, nServoId, tupRec.m_nPos, tupRec.m_nSpeed);

      if( rc < 0 )
      {
        return rc;
      }

      printf("%d %d   ", nServoId, tupRec.m_nPos);
    }
  
    printf("\n");
  
    //
    // Move synchronously to new position.
    //
    rc = shell.m_pDynaChain->SyncMoveTo(tupPos, uCount);
  
    if( rc < 0 )
    {
      printf("Error: SyncMoveTo: %s\n", DynaStrError(rc));
      return rc;
    }
  
    return DYNA_OK;
  }

  /*!
   * \brief Control the servos speed to reach the goal positions, hopefully at
   * the end of the sampled period and with the sampled speed.
   *
   * \param shell     Dynamixel shell.
   * \param nRecNum   Record number in the recording.
   * \param dt        Delta time (seconds).
   *
   * \copydoc doc_return_std
   */
  int ControlToGoals(DynaShell &shell, int nRecNum, double dt)
  {
    int               nFldNum;
    int               nServoId;
    DynaServo        *pServo;
    int               nCurPos;
    int               nCurSpeed;
    int               nGoalSpeed;
    DynaSpeedTuple_T  tupSpeed[DYNA_ID_NUMOF];
    uint_t            uCount;
    int               rc;
  
    printf("  speed_%d: ", nRecNum);
  
    //
    // Determine new speed goals and set.
    //
    for(nFldNum = shell.m_pRecording->FirstField(nRecNum), uCount=0;
        nFldNum != DynaRecording::END;
        nFldNum = shell.m_pRecording->NextField(nRecNum, nFldNum), ++uCount)
    {
      nServoId = shell.m_pRecording->GetServoId(nFldNum);
  
      pServo = shell.m_pDynaChain->GetServo(nServoId);

      // read current servo position
      rc = pServo->ReadCurPos(&nCurPos);
  
      if( rc != DYNA_OK )
      {
        printf("Error: Servo %d: ReadCurPos: %s\n", nServoId, DynaStrError(rc));
        return rc;
      }
  
      // read current speed
      rc = pServo->ReadCurSpeed(&nCurSpeed);
  
      if( rc != DYNA_OK )
      {
        printf("Error: Servo %d: ReadCurSpeed: %s\n", nServoId,
            DynaStrError(rc));
        return rc;
      }
  
      // pid control
      nGoalSpeed = pServo->GetSpeedPid().Control(nCurSpeed, dt);

      tupSpeed[nFldNum].m_nServoId = nServoId;
      tupSpeed[nFldNum].m_nSpeed   = nGoalSpeed;
  
      if( m_fpPlot != NULL )
      {
        PlotWriteData(shell, nRecNum, dt, nServoId, nCurPos);
      }

      printf("%d %d   ", nServoId, nGoalSpeed);
    }
  
    printf("\n");
  
    //
    // Synchronously write new goal speeds.
    //
    rc = shell.m_pDynaChain->SyncWriteGoalSpeed(tupSpeed, uCount);
  
    if( rc != DYNA_OK )
    {
      printf("Error: Servo %d: SyncWriteGoalSpeed: %s\n",
              nServoId, DynaStrError(rc));
      return rc;
    }
  
    return DYNA_OK;
  }
  
  void Init(DynaShell &shell)
  {
    DynaRecording  *pRecording = shell.m_pRecording;
    int             nNumPidCtlPts = 1;   // may become an argument

    // recorded sample period, converted to seconds and scaled to play speed
    m_fSamplePeriod = ((double)pRecording->GetSamplePeriod() / 1000.0 ) *
                                    100.0 / m_fSpeedPct;

    // 1 millisecond sample period is as good as it gets
    if( m_fSamplePeriod < 0.001 )
    {
      m_fSamplePeriod = 0.001;
    }
    
    // control sub-sample period (seconds)
    m_fSubSamplePeriod = m_fSamplePeriod / (double)m_nNumSubSamplePts;

    if( m_fSubSamplePeriod < 0.001 )
    {
      m_fSubSamplePeriod = 0.001;
    }
    
    // sub-sample control delta time per step (ms)
    m_dt = m_fSubSamplePeriod / (double)nNumPidCtlPts;

    if( m_dt < 0.001 )
    {
      m_dt = 0.001;
    }

    // Transform recording to basis spline curves
    SmoothRecordingCurves(shell);

    // initialize speed PIDs
    PidInit(shell);
  
    // initialize any plot output
    PlotInit(shell);
  }

  void SmoothRecordingCurves(DynaShell &shell)
  {
  }

  void BSplineCurve(DynaShell &shell, int nFldNum)
  {
    DynaRecording  *pRecording = shell.m_pRecording;
    size_t          uNumRecs;
    double          fTLen;
    size_t          uOrder;
    size_t          uNumCoeffs;
    size_t          uNumBreaks;
    size_t          i, j;
    double          xi, yi;
    double          yerr;
    double          chisq;

    uNumRecs  = (size_t)pRecording->GetNumOfRecords();
    fTLen     = (double)uNumRecs * m_fSamplePeriod - m_fSamplePeriod;

    // TODO coeff + order < numrecs
    
    uOrder      = 4;      // cubic b-spline order
    uNumCoeffs  = 15;     // number of bases
    uNumBreaks  = uNumCoeffs + 2 + uOrder;  // number of break points

    // allocate cubic (k=4) bspline workspace
    gsl_bspline_workspace *pWsBspline = gsl_bspline_alloc(uOrder, uNumBreaks);

    // allocate working vectors and matrices
    gsl_vector *B = gsl_vector_alloc(uNumCoeffs);
    gsl_vector *y = gsl_vector_alloc(uNumRecs);
    gsl_matrix *X = gsl_matrix_alloc(uNumRecs, uNumCoeffs);
    gsl_vector *c = gsl_vector_alloc(uNumCoeffs);
    gsl_vector *w = gsl_vector_alloc(uNumRecs);
    gsl_matrix *Cov = gsl_matrix_alloc(uNumCoeffs, uNumCoeffs);

    // allocate x workspace
    gsl_multifit_linear_workspace *pWsMFitLinear =
                              gsl_multifit_linear_alloc(uNumRecs, uNumCoeffs);

    // use uniform breakpoints on [0, len]
    gsl_bspline_knots_uniform(0.0, fTLen, pWsBspline);

    // construct the fit matrix X
    for(i=0, xi=0.0; i<uNumRecs; ++i, xi+=m_fSamplePeriod)
    {
      gsl_vector_set(y, i, (*pRecording)[i][nFldNum].m_nPos);

      // compute B_j(xi) for all j
      gsl_bspline_eval(xi, B, pWsBspline);

      /* fill in row i of X */
      for(j=0; j<uNumCoeffs; ++j)
      {
        double Bj = gsl_vector_get(B, j);
        gsl_matrix_set(X, i, j, Bj);
      }
    }

    // fit
    gsl_multifit_wlinear(X, w, y, c, Cov, &chisq, pWsMFitLinear);

    // smooth curve

    for(xi=0.0; xi<fTLen; xi+=m_fSubSamplePeriod)
    {
      gsl_bspline_eval(xi, B, pWsBspline);
      gsl_multifit_linear_est(B, c, Cov, &yi, &yerr);
    }

    // free
    gsl_bspline_free(pWsBspline);
    gsl_vector_free(B);
    gsl_vector_free(y);
    gsl_matrix_free(X);
    gsl_vector_free(c);
    gsl_vector_free(w);
    gsl_matrix_free(Cov);
    gsl_multifit_linear_free(pWsMFitLinear);
  }

  /*!
   * \brief Initialize servo PIDs.
   *
   * \param shell     Dynamixel shell.
   */
  void PidInit(DynaShell &shell)
  {
    int         iter;
    int         nServoId;
    DynaServo  *pServo;

    for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = shell.m_pDynaChain->IterNextMaster(&iter))
    {
      pServo = shell.m_pDynaChain->GetServo(nServoId);
      pServo->GetSpeedPid().InitControl();
    }
  }
  
  /*!
   * \brief Specify (new) PID goal position setpoint.
   *
   * \param shell       Dynamixel shell.
   * \param nServoId    Servo Id.
   * \param uGoalPos    Goal odometer position.
   * \param nGoalSpeed  Goal speed. Direction is important since a position may
   *                    be obtained in two rotation directions. 
   *
   * \copydoc doc_return_std
   */
  int PidSetPoint(DynaShell &shell,
                  int        nServoId,
                  int        nGoalPos,
                  int        nGoalSpeed)
  {
    int         nDir = nGoalSpeed < 0? DYNA_DIR_CW: DYNA_DIR_CCW;
    DynaServo  *pServo;
    int         nCurPos;
    int         nOdPos;
    int         rc;

    pServo = shell.m_pDynaChain->GetServo(nServoId);

    // read current servo position
    rc = pServo->ReadCurPos(&nCurPos);
  
    if( rc != DYNA_OK )
    {
      printf("Error: Servo %d: ReadCurPos: %s\n", nServoId, DynaStrError(rc));
      return rc;
    }
  
    // set new setpoint
    pServo->GetSpeedPid().SpecifySetPoint(nOdPos);

    return DYNA_OK;
  }

  /*!
   * \brief Clean up any allocated playback resources.
   *
   * \param shell   Dynamixel shell.
   */
  void Cleanup(DynaShell &shell)
  {
    if( m_fpPlot != NULL )
    {
      fclose(m_fpPlot);
      m_fpPlot = NULL;
    }

    m_vecCurves.clear();
  }

  bool MoveToStart(DynaShell &shell)
  {
    return true;
  }

  /*!
   * \brief Secure arm in safe postition if posible.
   *
   * \param pDynaChain    Pointer to Dynamixel chain handle.
   */
  void SecureArm(DynaShell &shell)
  {
    shell.m_pDynaChain->Freeze();
  }

  /*!
   * \brief Initialize plot data output.
   *
   * \param shell             Dynamixel shell.
   */
  void PlotInit(DynaShell &shell)
  {
    if( m_fpPlot != NULL )
    {
      m_nPlotLineCnt = 0;

      fprintf(m_fpPlot, "#\n");
      fprintf(m_fpPlot, "# Dynamixel Play Back Plot Data\n");
      fprintf(m_fpPlot, "#   Recorded:                 %s\n",
          shell.m_pRecording->GetDate());
      fprintf(m_fpPlot, "#   Number of records:        %d\n",
          shell.m_pRecording->GetNumOfRecords());
      fprintf(m_fpPlot, "#   Recording sample period:  %dms\n",
          shell.m_pRecording->GetSamplePeriod());
      fprintf(m_fpPlot, "#   Playback sample period:   %.3f\n",
          m_fSamplePeriod);
      fprintf(m_fpPlot, "#   Playback delta time step: %.4lfs\n", m_dt);
      fprintf(m_fpPlot, "#\n");
      fprintf(m_fpPlot, "#Fields:\n");
      fprintf(m_fpPlot, "# time servo goal_pos goal_speed cur_pos "
                        "cur_speed\n");
      fprintf(m_fpPlot, "#\n");
    }
  }

  /*!
   * \brief Write plot data to plot file.
   *
   * \param shell       Dynamixel shell.
   * \param nRecNum     Record number.
   * \param dt          Playback delta time step (seconds).
   * \param nServoid    Servo Id.
   * \param nCurPos     Servo current position.
   */
  void PlotWriteData(DynaShell &shell,
                     int        nRecNum,
                     double     dt,
                     int        nServoId,
                     int        nCurPos)
  {
    DynaServo  *pServo;

    if( m_fpPlot != NULL )
    {
      pServo = shell.m_pDynaChain->GetServo(nServoId);
      fprintf(m_fpPlot, "%.4f %d %u %d %u\n",
             m_fTAccum,
             nServoId,
            (uint_t)pServo->GetSpeedPid().GetSP(),
            nCurPos,
            (uint_t)pServo->GetSpeedPid().GetOutput());
    }
  }

  /*!
   * \brief Wait until all servos in chain have stopped moving utility function.
   *
   * \note Currently this member function is not used. But could be promoted to
   * a usefull shell command.
   *
   * \param shell       Dynamixel shell.
   */
  void WaitStop(DynaShell &shell)
  {
    int         nNumServos;
    int         nMoving;
    int         iter;
    int         nServoId;
    DynaServo  *pServo;
    bool        bIsMoving;
    int         rc;
  
    nNumServos = (int)shell.m_pDynaChain->GetNumberInChain();
    nMoving    = nNumServos;
  
    while( nMoving > 0 )
    {
      //printf(".");
  
      for(nServoId = shell.m_pDynaChain->IterStart(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter))
      {
        pServo = shell.m_pDynaChain->GetServo(nServoId);
  
        if( ((rc = pServo->ReadIsMoving(&bIsMoving)) == DYNA_OK) && bIsMoving )
        {
          ++nMoving;
        }
      }
    }

    //printf("\n");
  }
};


//-----------------------------------------------------------------------------
// Public Interface
//-----------------------------------------------------------------------------

/*!
 * \brief Publish shell servo commands to shell.
 *
 * \brief shell   Dynamixel shell.
 */
void PublishShellTrainCommands(DynaShell &shell)
{
  shell.PublishCommand("get", new DynaShellCmdGetPid());
  shell.PublishCommand("set", new DynaShellCmdSetPid());
  shell.PublishCommand("load", new DynaShellCmdLoadRecording());
  shell.PublishCommand("save", new DynaShellCmdSaveRecording());
  shell.PublishCommand(NULL, new DynaShellCmdTrain());
  shell.PublishCommand(NULL, new DynaShellCmdPlay());
}
