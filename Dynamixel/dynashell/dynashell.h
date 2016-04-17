////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The simple dynashell declarations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows LLC.
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

#ifndef _DYNASHELL_H
#define _DYNASHELL_H

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "dynashell_readline.h"
#include "dynashell_recording.h"
#include "dynashell_util.h"

using namespace std;

#define SH_C_COMMENT  '#'

/*
 * Forward declarations
 */
class DynaShellCmd;
class ShCmdNode;

/*!
 * \brief Shell command node map key compare helper class.
 */
class ShCmdNodeKeyCmp
{
public:
  /*!
   * \brief Compare operator required by \<map\>.
   *
   * \param lhs   Left hand side key.
   * \param rhs   Right had side key.
   *
   * \return Returns true if lhs \h_lt rhs, lexicographically.
   */
  bool operator()(const char* const &lhs, const char* const &rhs) const
  {
    return strcmp(lhs, rhs) < 0;
  }
};

/*!
 * \brief Shell command map type.
 */
typedef map<const char *, ShCmdNode, ShCmdNodeKeyCmp>  ShCmdMap;


// ----------------------------------------------------------------------------
// ShCmdNode Class
// ----------------------------------------------------------------------------

/*!
 * \brief Shell command node class.
 *
 * Each node holds either a command or a map of commands.
 */
class ShCmdNode
{
public:
  /*!
   * \brief Node type.
   */
  enum NodeType { NodeTypeNull, NodeTypeCmd, NodeTypeMap };

  /*!
   * \brief Default constructor.
   */
  ShCmdNode()
  {
    m_obj.m_pCmd  = NULL;
    m_eType = NodeTypeNull;
  }

  /*!
   * \brief Shell command initialization constructor.
   *
   * \param pCmd  Allocated shell command.
   */
  ShCmdNode(DynaShellCmd *pCmd)
  {
    m_obj.m_pCmd  = pCmd;
    m_eType = NodeTypeCmd;
  }

  /*!
   * \brief Map of commands initialization constructor.
   *
   * \param pMap    Allocated map of commands.
   */
  ShCmdNode(ShCmdMap *pMap)
  {
    m_obj.m_pMap  = pMap;
    m_eType = NodeTypeMap;
  }

  /*!
   * \brief Default destructor.
   */
  ~ShCmdNode() { }

  /*!
   * \brief Shell command assignment operator.
   *
   * \param rhs   Pointer to allocated shell command right hand side.
   *
   * \return Reference to this.
   */
  ShCmdNode &operator=(DynaShellCmd *rhs)
  {
    m_obj.m_pCmd  = rhs;
    m_eType = NodeTypeCmd;
    return *this;
  }

  /*!
   * \brief Shell map assignment operator.
   *
   * \param rhs   Pointer to allocated map of commands right hand side.
   *
   * \return Reference to this.
   */
  ShCmdNode &operator=(ShCmdMap *rhs)
  {
    m_obj.m_pMap  = rhs;
    m_eType = NodeTypeMap;
    return *this;
  }

  /*!
   * \brief Get command object.
   *
   * \return Pointer to shell command.
   */
  DynaShellCmd *GetCmd()
  {
    return m_eType == NodeTypeCmd?  m_obj.m_pCmd: NULL;
  }

  /*!
   * \brief Get map object.
   *
   * \return Pointer to map of commands.
   */
  ShCmdMap *GetMap()
  {
    return m_eType == NodeTypeMap?  m_obj.m_pMap: NULL;
  }

  /*
   * \brief Get map's brief description.
   *
   * \return strBrief   Brief description.
   */
  string GetMapBrief()
  {
    return m_strMapBrief;
  }

  /*
   * \brief Set map's brief description.
   *
   * \param strBrief  Brief description.
   */
  void SetMapBrief(const string &strBrief)
  {
    m_strMapBrief = strBrief;
  }

  /*!
   * \brief Test if node type is null.
   *
   * \return Returns true or false.
   */
  bool IsNull() { return m_eType == NodeTypeNull; }

  /*!
   * \brief Test if node type is shell command.
   *
   * \return Returns true or false.
   */
  bool IsCmd() { return m_eType == NodeTypeCmd; }

  /*!
   * \brief Test if node type is map of commands.
   *
   * \return Returns true or false.
   */
  bool IsMap() { return m_eType == NodeTypeMap; }

protected:
  NodeType        m_eType;        ///< node type
  union
  {
    DynaShellCmd   *m_pCmd;       ///< pointer to allocated shell command
    ShCmdMap       *m_pMap;       ///< pointer to allocated map of commands
  } m_obj;                        ///< node object
  string          m_strMapBrief;  ///< map brief description
};


// ----------------------------------------------------------------------------
// ShScript Class
// ----------------------------------------------------------------------------

/*!
 * \brief Shell Script Class.
 */
class ShScript
{
public:
  /*!
   * \brief Initialization constructor.
   *
   * \param sScripsFile   Script file name.
   * \param fp            Opened file pointer to script file.
   */
  ShScript(const char *sScriptFile, FILE *fp)
  {
    m_sScriptFile = NEWSTR(sScriptFile);
    m_fpScript    = fp;
    m_nLineNum    = 0;
  }

  /*!
   * \brief Default destructor.
   */
  ~ShScript()
  {
    if( m_fpScript != NULL )
    {
      fclose(m_fpScript);
    }
    DELSTR(m_sScriptFile);
  }

  /*!
   * \brief Get script file name.
   *
   * \return char*
   */
  const char *GetScriptName() const
  {
    return m_sScriptFile;
  }

  /*!
   * \brief Get file pointer.
   *
   * \return FILE*
   */
  FILE *GetFp()
  {
    return m_fpScript;
  }

  /*!
   * \brief Get current line number.
   *
   * \return int
   */
  const int GetLineNum() const
  {
    return m_nLineNum;
  }

  /*!
   * \brief Bump line number by one.
   */
  void BumpLineNum()
  {
    m_nLineNum++;
  }

protected:
  char *m_sScriptFile;    ///< script file name
  FILE *m_fpScript;       ///< open file pointer 
  int   m_nLineNum;       ///< current line number
};


// ----------------------------------------------------------------------------
// DynaShell Class
// ----------------------------------------------------------------------------

/*!
 * The Dynamixel shell class.
 */
class DynaShell
{
public:
  static const int  m_nMaxScriptDepth = 5;    ///< maximum script stack depth

  bool            m_bRun;             ///< do [not] run shell
  bool            m_bIsInteractive;   ///< [not] user interactive
  bool            m_bXTrace;          ///< do [not] trace script
  bool            m_bSilent;          ///< do [not] silence non-error responses
  DynaComm       *m_pDynaComm;        ///< dynamixel bus communication
  DynaChain      *m_pDynaChain;       ///< dynamixel chain
  DynaBgThread   *m_pDynaBgThread;    ///< dynamixel chain
  DynaRecording  *m_pRecording;       ///< dynamixel recording

  DynaShell();

  ~DynaShell();

  void PublishMap(const char *sParent, const string &strBrief);
  void PublishCommand(const char *sParent, DynaShellCmd *pNewCmd);
  void DeleteCommands(ShCmdMap &cmdMap);

  DynaShellCmd *MatchCmd(int argc, char *argv[], ShCmdMap &cmdMap);
  
  /*!
   * \brief (Re)Initialize recording.
   *
   * If the shell does not have a recording object, it is created.
   * Any previously recorded data is 'erased'. 
   *
   * \param nSamplePeriod   Recording sample period (msec).
   * \param sData           Recording date string.
   */
  void RecordingInit(int nSamplePeriod, const char *sDate = NULL)
  {
    if( m_pRecording == NULL )
    {
      m_pRecording = new DynaRecording;
    }
    m_pRecording->Init(nSamplePeriod, sDate);
  }

  /*!
   * \brief Replace recording.
   *
   * \param pNewRecording  Allocated new recording.
   */
  void RecordingReplace(DynaRecording *pNewRecording)
  {
    if( m_pRecording != NULL )
    {
      delete m_pRecording;
    }
    m_pRecording = pNewRecording;
  }

  void ScriptPush(const char *sScriptFile, FILE *fp);
  void ScriptPop();
  void ScriptFlush();
  void ScriptTrace(const char *sLine);

  /*!
   * \brief Test if executing a script.
   *
   * \return true or false.
   */
  bool IsRunningScript()
  {
    return m_vecScripts.empty()? false: true;
  }


  /*!
   * \brief Print standard ok success response.
   */
  virtual void Ok()
  {
    if( !m_bSilent )
    {
      printf("ok\n");
    }
  }

  void Response(const char *sFmt, ...);
  void Warning(const char *sFmt, ...);
  void Error(int rc, const char *sFmt, ...);
  void Error(int nServoId, uint_t uAlarms);
  void Error(const char *sFmt, ...);

  int Run();

  /*!
   * Get map of shell root commands.
   *
   * \return Returns command map.
   */
  ShCmdMap &GetRootCmds()
  {
    return m_mapCmds;
  }

  char *CmdListGenerator(int         nUid,
                         const char *sText,
                         size_t      uTextLen,
                         int         nState,
                         const char *sContext);

  char *CmdSpecGenerator(int         nUid,
                         const char *sText,
                         size_t      uTextLen,
                         int         nState,
                         const char *sContext);

protected:
  typedef map<int,DynaShellCmd*>  MapTabCompleteGen;

  const char         *m_sShellName;   ///< shell name
  const char         *m_sPS1;         ///< primary user prompt
  const char         *m_sPS2;         ///< secondary user prompt
  bool                m_bNoExec;      ///< parse only, no execution
  char               *m_sInputLine;   ///< working, allocated line of input
  ReadLine            m_ReadLine;     ///< readline object
  ShCmdMap            m_mapCmds;      ///< map of published commnds
  MapTabCompleteGen   m_mapTabGen;    ///< command tab-completion generators map
  vector<ShScript*>   m_vecScripts;   ///< script stack
  int                 m_rc;           ///< last command executed return code

  void InputInit();

  char *GetInputLine();

  int GetInput();
    
  /*!
   * \brief Test if line is a potential executable statement.
   *
   * \note Assumes line has been stripped of leading white space.
   *
   * \param sLine   Line to test.
   *
   * \return true or false.
   */
  bool IsExecStmt(const char *sLine)
  {
    return (sLine!=NULL) && (*sLine!=0) && (*sLine!=SH_C_COMMENT)? true: false;
  }

  static char *CmdListGeneratorWrap(int         nUid,
                                    const char *sText,
                                    size_t      uTextLen,
                                    int         nState,
                                    const char *sContext,
                                    void       *pArg);

  static char *CmdSpecGeneratorWrap(int         nUid,
                                    const char *sText,
                                    size_t      uTextLen,
                                    int         nState,
                                    const char *sContext,
                                    void       *pArg);

  void AddToHistory(char *sLine);

  /*!
   * \brief Get the current input file pointer.
   *
   * \return File pointer.
   */
  FILE *InputFp()
  {
    return !m_vecScripts.empty()? m_vecScripts.back()->GetFp(): stdin;
  }

  /*!
   * \brief Bump the line number of the current executing script, if any,
   * by one.
   */
  void ScriptBumpLineNum()
  {
    if( !m_vecScripts.empty() )
    {
      m_vecScripts.back()->BumpLineNum();
    }
  }

  /*!
   * \brief Test if input is at End Of File condition.
   *
   * \return Returns true or false.
   */
  bool IsEOF()
  {
    return feof(InputFp())? true: false;
  }
};


#endif // _DYNASHELL_H
