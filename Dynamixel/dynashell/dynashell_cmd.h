////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_cmd.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The dynashell Command Class Interface.
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

#ifndef _DYNASHELL_CMD_H
#define _DYNASHELL_CMD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"

#include "dynashell.h"

using namespace std;


// ----------------------------------------------------------------------------
// DynaShellCmd Class
// ----------------------------------------------------------------------------

/*!
 * \brief Dynamixel shell command abstract base class.
 */
class DynaShellCmd
{
  /*!
   * \brief Try boolean expression.
   *
   * \param boolexpr  Boolean C/C++ expression.
   *
   * \return Returns from calling function if expression evaluates to false.
   */
  #define TRY(boolexpr, ...) do { if( !(boolexpr) ) { return; } } while(0)

public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmd(int nArgCntMin=0, int nArgCntMax=0) :
      m_nArgCntMin(nArgCntMin),
      m_nArgCntMax(nArgCntMax)
  {
    m_sCmdName      = "";
    m_sCmdHelpBrief = "";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "";
    m_nPubLevel     = 0;
    m_sPubName      = NULL;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmd() { }

  /*!
   * \brief Get dynamixel shell command name.
   *
   * \return Command name string.
   */
  const char *GetCmdName() { return m_sCmdName; }

  /*!
   * \brief Get shell command name brief description.
   *
   * \return Command name string.
   */
  const char *GetCmdHelpBrief() { return m_sCmdHelpBrief; }

  /*!
   * \brief Execute command abstract function.
   *
   * \param shell   Embedding dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  virtual void Exec(DynaShell &shell, int argc, char *argv[]) = 0;

  virtual void PrintHelp(int indent=0, int width=80);

  /*!
   * Get the command's published level.
   *
   * The root level is 0.
   *
   * \return Published level.
   */
  int GetPublishedLevel() const
  {
    return m_nPubLevel;
  }

  /*!
   * Get the command's published name.
   *
   * \return Published name.
   */
  const char *GetPublishedName() const
  {
    return m_sPubName;
  }

  /*!
   * Set the command's published information.
   *
   * \param nLevel  Published level. The root level is 0.
   * \param sParent Parent to which this command was published.
   */
  int SetPublishedInfo(int nLevel, const char *sParent=NULL)
  {
    m_nPubLevel = nLevel;

    if( m_sPubName != NULL )
    {
      delete[] m_sPubName;
    }
    if( sParent == NULL )
    {
      m_sPubName = newstr(m_sCmdName);
    }
    else
    {
      m_sPubName = new char[strlen(sParent)+1+strlen(m_sCmdName)+1];
      sprintf(m_sPubName, "%s %s", sParent, m_sCmdName);
    }
  }

  /*!
   * \brief Default command tab completion generator.
   *
   * The default is no completion.
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST,then initialize any statics.
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
    //return ReadLine::FileCompletionGenerator(sText, nState);
    return NULL;
  }

  /*!
   * \brief Check that the argument count is within the class (min,max).
   *
   * \param shell   Dynamixel shell.
   * \param argc    Argument count.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  virtual bool ChkArgCnt(DynaShell &shell, int argc)
  {
    if( (m_nArgCntMin == 0) && (m_nArgCntMax == 0) )
    {
      return ChkArgCnt0(shell, argc);
    }
    else if( m_nArgCntMin == m_nArgCntMax )
    {
      return ChkArgCntEQ(shell, argc, m_nArgCntMin);
    }
    
    if( (m_nArgCntMin > 0) && !ChkArgCntGE(shell, argc, m_nArgCntMin) )
    {
      return false;
    }
    
    if( (m_nArgCntMax > 0) && !ChkArgCntLE(shell, argc, m_nArgCntMax) )
    {
      return false;
    }

    return true;
  }

  /*!
   * \brief Check that the argument count is zero.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Argument count.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkArgCnt0(DynaShell &shell, int argc) 
  { 
    if( argc != 0 )
    { 
      shell.Error("Command has no arguments.");
      return false;
    }
    return true;
  }

  /*!
   * \brief Check that the argument count is equal to the required.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Argument count.
   * \param min     Required minimum count.
   *
   * \return
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkArgCntEQ(DynaShell &shell, int argc, int eq) 
  { 
    if( argc != eq )
    { 
      shell.Error("Argument count %d != required %d.", argc, eq);
      return false;
    }
    return true;
  }

  /*!
   * \brief Check argument count against minimum required.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Argument count.
   * \param min     Required minimum count.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkArgCntGE(DynaShell &shell, int argc, int min) 
  { 
    if( argc < min )
    { 
      shell.Error("Argument count %d < minimum %d.", argc, min);
      return false;
    }
    return true;
  }

  /*!
   * \brief Check argument count against maximum allowed.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Argument count.
   * \param min     Maximum count.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkArgCntLE(DynaShell &shell, int argc, int max) 
  { 
    if( argc > max )
    { 
      shell.Error("Argument count %d > maximum %d.", argc, max);
      return false;
    }
    return true;
  }

  /*!
   * \brief Check that Dynamixel communication exists and is open.
   *
   * \param shell   Dynamixel shell.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkComm(DynaShell &shell)
  { 
    if( (shell.m_pDynaComm == NULL) || !shell.m_pDynaComm->IsOpen() )
    { 
      shell.Error("No open dynamixel communication.");
      return false;
    } 
    return true;
  }

  /*!
   * \brief Check that the servo chain exists.
   *
   * \param shell   Dynamixel shell.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkChain(DynaShell &shell)
  { 
    if( shell.m_pDynaChain == NULL )
    { 
      shell.Error("No dynamixel chain.");
      return false;
    } 
    return true;
  }

  /*!
   * \brief Check that the servo chain exists and is not empty
   *
   * \param shell   Dynamixel shell.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkChainNotEmpty(DynaShell &shell)
  { 
    if( !ChkChain(shell) )
    {
      return false;
    }

    if( shell.m_pDynaChain->GetNumberInChain() == 0 )
    { 
      shell.Error("No servos in dynamixel chain. Try scanning.");
      return false;
    } 
    return true;
  }

  /*!
   * \brief Check that the given servo is present in the chain.
   *
   * \param shell     Dynamixel shell.
   * \param nServoId  Servo id.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkChainHasServo(DynaShell &shell, int nServoId)
  { 
    if( !ChkChainNotEmpty(shell) )
    {
      return false;
    }

    if( !shell.m_pDynaChain->HasServo(nServoId) )
    { 
      shell.Error("Servo %d not found in chain.", nServoId);
      return false;
    } 
    return true;
  }

  /*!
   * \brief Check that the given servo is a master.
   *
   * \param shell     Dynamixel shell.
   * \param nServoId  Servo id.
   *
   * \return 
   * Returns true if the check passed.\n
   * Otherwise a shell error is printed and false is returned.
   */
  bool ChkChainIsMasterServo(DynaShell &shell, int nServoId)
  { 
    if( !ChkChainHasServo(shell, nServoId) )
    {
      return false;
    }

    if( !shell.m_pDynaChain->GetServo(nServoId)->IsMaster() )
    { 
      shell.Error("Servo %d not a master.", nServoId);
      return false;
    } 
    return true;
  }

  /*!
   * \brief Convert command argument to integer.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, otherwise a shell error is printed and
   * false is returned.
   */
  bool ToInt(DynaShell &shell, const char *sArg, int *pVal)
  {
    char  *t;

    *pVal = (int)strtol(sArg, &t, 0);

    if( *t != 0 )
    {
      shell.Error("Argument value \"%s\": Not an integer.", sArg);
      return false;
    }
    return true;
  }

  /*!
   * \brief Convert command argument to unsigned integer.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, otherwise a shell error is printed and
   * false is returned.
   */
  bool ToUInt(DynaShell &shell, const char *sArg, uint_t *pVal)
  {
    return ToInt(shell, sArg, (int *)pVal);
  }

  /*!
   * \brief Convert command argument to double.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, otherwise a shell error is printed and
   * false is returned.
   */
  bool ToDouble(DynaShell &shell, const char *sArg, double *pVal)
  {
    if( sscanf(sArg, "%lf", pVal) != 1 )
    {
      shell.Error("Argument value \"%s\": Not a floating-point number.", sArg);
      return false;
    }
    return true;
  }

  /*!
   * \brief Convert command argument to boolean.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, otherwise a shell error is printed and
   * false is returned.
   */
  bool ToBool(DynaShell &shell, const char *sArg, bool *pVal)
  {
    int   i;
    char *t;

    if( !strcasecmp(sArg, "t") || !strcasecmp(sArg, "true") )
    {
      *pVal = true;
      return true;
    }
    else if( !strcasecmp(sArg, "f") || !strcasecmp(sArg, "false") )
    {
      *pVal = false;
      return true;
    }

    i = (int)strtol(sArg, &t, 0);

    if( *t != 0 )
    {
      shell.Error("Argument value \"%s\": Not an boolean.", sArg);
      return false;
    }

    *pVal = i? true: false;

    return true;
  }

protected:
  const char *m_sCmdName;         ///< command name
  const char *m_sCmdHelpBrief;    ///< command help brief
  const char *m_sCmdHelpArgs;     ///< command help arguments
  const char *m_sCmdHelpDesc;     ///< command help description
  const int   m_nArgCntMin;       ///< minimum argument count
  const int   m_nArgCntMax;       ///< maximum argument count (0 if not max)
  int         m_nPubLevel;        ///< command's published level (depth)
  char       *m_sPubName;         ///< command's published name

  virtual void PrintSynopses(int indent, int width);
  virtual void PrintBlock(int col, int indent, int width, const char *sText);
  virtual void PrintDelim(int width, const char cDelim);
  char *eow(const char *s);
};


// -----------------------------------------------------------------------------
// DynaShellCmdChainIn Class
// -----------------------------------------------------------------------------

/*!
 * \brief Dynamixel chain input command abstract base class.
 *
 * This class supports parsing a list of (master) servos as arguments to a
 * derived command class executinng servo 'read-like' input operations. Input
 * operations include read and get.
 *
 * \par Syntax:
 * \verbatim
 * CMD args
 * args ::=
 *      'chain'
 *    | servo_list
 *  servo_list ::=
 *      servo_id
 *    | servo_list servo_id
 *  servo_id ::= INT
 * \endverbatim
 */
class DynaShellCmdChainIn : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdChainIn(bool bMasterOpOnly=false) :
      DynaShellCmd(1, DYNA_ID_NUMOF)
  {
    m_bMasterOpOnly = bMasterOpOnly;

  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdChainIn() { }

  virtual void Exec(DynaShell &shell, int argc, char *argv[]);

  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext);

protected:
  bool m_bMasterOpOnly;   ///< is [not] a master-only supported operation
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Derived class specific execution function.
   *
   * \param shell   Dynamixel shell.
   * \param pServo  Pointer to servo to perform the action on.
   */
  virtual void doExec(DynaShell &shell, DynaServo *pServo) = 0;
};


// -----------------------------------------------------------------------------
// DynaShellCmdChainOut Class
// -----------------------------------------------------------------------------

/*!
 * \brief Execute 2-tuple structure type.
 */
typedef struct
{
  int m_nServoId;     ///< servo id
  int m_nVal;         ///< value
} ExecTup_T;


/*!
 * \brief Dynamixel chain output command abstract base class.
 *
 * This class supports parsing a 2-tuple list of (master) servos as arguments
 * to a derived command class executinng servo 'write-like' output operations.
 * Output operations include write, move, and set.
 *
 * \par Syntax:
 * \verbatim
 * CMD args
 * args ::=
 *      'chain' value
 *    | tuple_list
 *  tuple_list ::=
 *      tuple
 *    | tuple_list tuple
 *  tuple ::=
 *      servo_id value
 *  servo_id ::= INT
 *  value ::= VAL_TO_UINT
 * \endverbatim
 */
class DynaShellCmdChainOut : public DynaShellCmd
{
public:
  /*!
   * \brief Default constructor.
   */
  DynaShellCmdChainOut(bool bMasterOpOnly=false) :
      DynaShellCmd(2, DYNA_ID_NUMOF * 2)
  {
    m_bMasterOpOnly = bMasterOpOnly;
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaShellCmdChainOut() { }

  virtual void Exec(DynaShell &shell, int argc, char *argv[]);

  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext);

protected:
  bool m_bMasterOpOnly;   ///< is [not] a master-only supported operation
  int  m_nTabIter;        ///< tab completion: servo id iterator
  int  m_nTabServoId;     ///< tab completion: current servo id
  bool m_bTabIncChain;    ///< tab completion: [do not] include chain keyword

  /*!
   * \brief Derived class specific execution function.
   *
   * \param shell   Dynamixel shell.
   * \param tup     Array of (servo id, value) 2-tuples.
   * \param uCount  Number of tuples.
   */
  virtual void doExec(DynaShell &shell, ExecTup_T tup[], uint_t uCount) = 0;

  /*!
   * \brief Convert command argument into unsigned integer.
   *
   * \param shell       Dynamixel shell.
   * \param sArg        String argument to convert.
   * \param [out] pVal  Converted argument.
   *
   * \return Returns true on success, false otherwise.
   */
  virtual bool ArgToUInt(DynaShell &shell, const char *sArg, uint_t *pVal)
  {
    return ToUInt(shell, sArg, pVal);
  }

  virtual bool ArgToInt(DynaShell &shell, const char *sArg, int *pVal)
  {
    return ToInt(shell, sArg, pVal);
  }
};


// ----------------------------------------------------------------------------
// Prototypes
// ----------------------------------------------------------------------------

extern void PublishShellCoreCommands(DynaShell &shell);
extern void PublishShellInterfaceCommands(DynaShell &shell);
extern void PublishShellServoCommands(DynaShell &shell);
extern void PublishShellTrainCommands(DynaShell &shell);


#endif // _DYNASHELL_CMD_H
