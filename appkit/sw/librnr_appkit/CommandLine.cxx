////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CommandLine.cxx
//
/*! \file
 *
 * \brief Command line interface class implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par License:
 * MIT
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/select.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <ctype.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/LogStream.h"
#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/IOManip.h"
#include "rnr/appkit/RegEx.h"
#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/ReadLine.h"
#include "rnr/appkit/Token.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CmdArgDef.h"
#include "rnr/appkit/CmdFormDef.h"
#include "rnr/appkit/CmdDef.h"
#include "rnr/appkit/CommandLine.h"

using namespace std;
using namespace rnr;
using namespace rnr::str;
using namespace rnr::io;
using namespace rnr::cmd;

// -----------------------------------------------------------------------------
// Private Implementation
// -----------------------------------------------------------------------------

/*!
 * \brief Debugging macros.
 */
#undef CL_ENABLE_DEBUG  ///< define to enable

#ifdef CL_ENABLE_DEBUG
static int dbg_calldepth_ = -1;

#define CL_CALL_DEPTH        dbg_calldepth_
#define CL_SET_CALL_DEPTH(_n) dbg_calldepth_ = _n
#define CL_PUSH_CALL_DEPTH() ++dbg_calldepth_
#define CL_POP_CALL_DEPTH()  --dbg_calldepth_

/*!
 * \brief Enter function debug macro.
 * \param _args Stream of function arguments.
 * \param _post Stream of post call data.
 */
#define CL_DBG_CALL_IN(_args, _post) \
do \
{ \
  CL_PUSH_CALL_DEPTH(); \
  cerr << space(dbg_calldepth_*2) << __func__ << "(" << _args << ")" <<_post; \
} while(0)

/*!
 * \brief Enter parse function debug macro.
 * \param _cmddef   Command definition.
 * \param _formdef  Form definition.
 * \param _toks     Input tokens.
 * \param _pos      Parse position in tokens.
 * \param _post     Stream of post call data.
 */
#define CL_DBG_PARSE_CALL_IN(_cmddef, _formdef, _toks, _pos, _post) \
  CL_DBG_CALL_IN("cmddef(uid=" << (_cmddef).getUid() \
      << ", name=" << (_cmddef).getName() << ")"     \
      << ", form(index=" << (_formdef).getIndex()    \
      << ", argc=" << (_formdef).numOfArgs() << ")"  \
      << ", tokens=" << (_toks).size()               \
      << ", pos=" << (_pos), _post)

/*!
 * \brief Exit function debug macro with results on New Line.
 * \param _res    Results.
 */
#define CL_DBG_CALL_OUT_NL(_res) \
do \
{ \
  cerr << space(dbg_calldepth_*2) << "--> (" << _res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

/*!
 * \brief Exit function debug macro with results In Line.
 * \param _res    Results.
 */
#define CL_DBG_CALL_OUT_IL(_res) \
do \
{ \
  cerr << "--> (" << _res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

/*!
 * \brief Debug macro with no indentation.
 * \param _data   Any data stream.
 */
#define CL_DBG(_data) cerr << _data

#else
#define CL_CALL_DEPTH
#define CL_SET_CALL_DEPTH(_n)
#define CL_PUSH_CALL_DEPTH()
#define CL_POP_CALL_DEPTH()
#define CL_DBG_CALL_IN(_args, _post)
#define CL_DBG_PARSE_CALL_IN(_cmddef, _formdef, _toks, _pos, _post)
#define CL_DBG_CALL_OUT_NL(_res)
#define CL_DBG_CALL_OUT_IL(_res)
#define CL_DBG(os)
#endif // CL_ENABLE_DEBUG

namespace rnr
{
  namespace cmd
  {
    /*!
     * \brief Command usage syntax special characters.
     *
     * \param c Character to test.
     *
     * \return Returns true if c is special, false otherwise.
     */
    static inline bool isspecial(int c)
    {
      return( (c == '<') || (c == ':') || (c == '>') ||
              (c == '{') || (c == '|') || (c == '}') ||
              (c == '[') || (c == ']') ||
              (c == '(') || (c == ')') );
    }

    /*!
     * \brief Test if c is a double quote character.
     */
    static inline bool isdquote(int c)
    {
      return(c == '"');
    }

    /*!
     * \brief Test if c is a open parenthesis character.
     */
    static inline bool isoparen(int c)
    {
      return(c == '(');
    }

    /*!
     * \brief Test if c is a close parenthesis character.
     */
    static inline bool iscparen(int c)
    {
      return(c == ')');
    }

    /*!
     * \brief Testif c is an escape character.
     */
    static inline bool isesc(int c)
    {
      return(c == '\\');
    }

    /*!
     * \brief Convert ascii character to binary character value.
     *
     * \param c ASCII character.
     *
     * \return Binary character.
     */
    static char tohex(int c)
    {
      if( (c >= '0') && (c <= '9') )
      {
        return (char)(c - '0');
      }
      else if( (c >= 'a') && (c <= 'f') )
      {
        return (char)(c - 'a');
      }
      else if( (c >= 'A') && (c <= 'F') )
      {
        return (char)(c - 'A');
      }
      else
      {
        return (char)0;
      }
    }

    static CmdDef       nocmddef;     ///< "no cmd def" command definition
    static const string noprompt;     ///< "no prompt" prompt value

  } //namespace cmd
} // namespace rnr
      

// -----------------------------------------------------------------------------
// CmdExec Class
// -----------------------------------------------------------------------------

int CmdExec::execute(const StringVec &argv)
{
  switch( m_variant )
  {
    case Variant1:
      return m_exec.fn1 != NULL? m_exec.fn1(argv): ENoExec;
    case Variant2:
    case Variant3:
    default:
      return ENoExec;
  }
}

int CmdExec::execute(CommandLine &cli, const CmdExtArgVec &argv)
{
  switch( m_variant )
  {
    case Variant2:
      return m_exec.fn2 != NULL? m_exec.fn2(argv): ENoExec;
    case Variant3:
      return m_exec.fn3 != NULL? m_exec.fn3(cli, argv): ENoExec;
    case Variant1:
    default:
      return ENoExec;
  }
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdExec &obj)
{
  os << indent()
    << "uid = " << obj.m_uid << ", "
    << "variant = " << obj.m_variant << ", "
    << "fn = " << obj.m_exec.p;

  return os;
}
      

// -----------------------------------------------------------------------------
// DataSect Class
// -----------------------------------------------------------------------------

DataSect::DataSect()
{
  m_pData     = NULL;
  m_fnDealloc = NULL;
}

DataSect::DataSect(const string &ns, void *pData, DeallocFunc fn)
{
  m_strNs     = ns;
  m_pData     = pData;
  m_fnDealloc = fn;
}

//DataSect::DataSect(const DataSect &src)
//{
//  m_strNs     = src.m_strNs;
//  m_pData     = src.m_pData;
//  m_fnDealloc = src.m_fnDealloc;
//}

DataSect::~DataSect()
{
  if( (m_pData != NULL) && (m_fnDealloc != NULL) )
  {
    m_fnDealloc(m_pData);
  }
}

void DataSect::set(const string &ns, void *pData, DeallocFunc fn)
{
  m_strNs     = ns;
  m_pData     = pData;
  m_fnDealloc = fn;
}

ostream &rnr::cmd::operator<<(ostream &os, const DataSect &obj)
{
  os << indent()
    << "namespace = " << obj.m_strNs << ", "
    << "data = " << obj.m_pData << ", "
    << "dealloc = " << (void *)obj.m_fnDealloc;

  return os;
}


// -----------------------------------------------------------------------------
// CommandLine Class
// -----------------------------------------------------------------------------

//
// Logging bookmarks.
//
static const string markExec("ExecMark");

//
// Logging common prefix strings.
//
static const string    labelExec("Execute: ");
static const string  labelNoExec("NoExec:  ");
static const string labelCompile("Compile: ");
static const string   labelParse("Parse:   ");
static const string   labelInput("Input:   ");
static const string      labelAt("At:      ");
static const string  labelSelect("Select:  ");
static const string     labelTry("Try:     ");
static const string     labelFit("Fitness: ");
static const string labelNoMatch("NoMatch: ");
static const string   labelMatch("Match:   ");
static const string  labelSyntax("Error:   ");
static const string    labelFail("Failure: ");
static const string   labelBlank("         ");

CommandLine::CommandLine(const string strName,
                         const string strPrompt,
                         bool         bUseRlLib,
                         bool         bIgnoreCase) :
    m_strName(strName),
    m_bIgnoreCase(bIgnoreCase),
    m_readline(strName, strPrompt, bUseRlLib),
    m_log("Command Line Log", 50)
{
  m_nUidCnt     = 0;
  m_bIsCompiled = false;

  addDataSection(DataSectNsCore, new DataSectCore(), DataSectCore::dealloc);

  pushPrompt(strPrompt);
}

CommandLine::~CommandLine()
{
}

bool CommandLine::isDefined() const
{
  return m_bIsCompiled;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Addition and Compile Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::addCommand(const CmdDesc &desc)
{
  int uid;

  // command syntax must be defined
  if( desc.syntax.empty() )
  {
    m_log << "No syntax forms specified." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return NoUid;
  }

  // add command
  if( (uid = addCommand(desc.syntax)) != NoUid )
  {
    m_cmdDefs[uid].addHelp(desc.synopsis, desc.longdesc);
  }

  return uid;
}

int CommandLine::addCommand(const CmdDesc &desc, CmdExec1Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(desc)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::addCommand(const CmdDesc &desc, CmdExec2Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(desc)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::addCommand(const CmdDesc &desc, CmdExec3Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(desc)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::addCommand(const string strSyntax)
{
  StringVec   syntaxForms;
  string      strName;
  int         uid;

  //
  // Split command syntax forms along newlines.
  //
  split(strSyntax, '\n', syntaxForms);

  //
  // No syntax forms.
  //
  if( syntaxForms.size() == 0 || syntaxForms[0].empty() )
  {
    m_log << "No syntax forms specified." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return NoUid;
  }

  //
  // New command.
  //
  CmdDef newdef;

  // set command's unique id
  newdef.setUid(m_nUidCnt++);

  uid = newdef.getUid();

  // add to map of commands 
  m_cmdDefs[uid] = newdef;

  // just added
  CmdDef &cmddef = cmdAt(uid);

  //
  // Add all syntax forms.
  //
  for(size_t i = 0; i < syntaxForms.size(); ++i)
  {
    CmdFormDef formdef(syntaxForms[i]);

    // add command form
    cmddef.pushForm(formdef);
  }

  // save raw syntax
  cmddef.m_strSyntax = strSyntax;

  m_bIsCompiled = false;

  return uid;
}

int CommandLine::addCommand(const string strSyntax, CmdExec1Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(strSyntax)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::addCommand(const string strSyntax, CmdExec2Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(strSyntax)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::addCommand(const string strSyntax, CmdExec3Func fnExec)
{
  int uid;

  // add command
  if( (uid = addCommand(strSyntax)) != NoUid )
  {
    m_cmdExecs[uid] = CmdExec(uid, fnExec);
  }

  return uid;
}

int CommandLine::removeCommand(const int uid)
{
  CmdDefIter  dpos; // definition position
  CmdExecIter epos; // execution position
  int         rc;   // return code

  if( uid == NoUid )
  {
    rc = EBadVal;
  }
  else if( (dpos = m_cmdDefs.find(uid)) == m_cmdDefs.end() )
  {
    rc = EBadVal;
  }
  else
  {
    // remove command definition
    m_cmdDefs.erase(dpos);

    // remove any command execution function
    if( (epos = m_cmdExecs.find(uid)) != m_cmdExecs.end() )
    {
      m_cmdExecs.erase(epos);
    }

    if( m_cmdDefs.size() == 0 )
    {
      m_bIsCompiled = false;
    }

    m_log << "Command " << uid << " removed." << eoe;
    rc = AOk;
  }

  if( rc != AOk )
  {
    m_log << "No command with uid " << uid << " found." << eoe;
    LOGERROR_STREAM(getErrorStr());
  }

  return rc;
}

int CommandLine::removeCommand(const string &strCmd)
{
  CmdDef &cmddef = cmdAt(strCmd);

  return removeCommand(cmddef.getUid());
}

int CommandLine::removeAllCommands()
{
  m_cmdDefs.clear();
  m_bIsCompiled = false;
  m_log << "All commands removed." << eoe;
  return AOk;
}

int CommandLine::compile()
{
  CmdDefIter  iter;
  int         rc = AOk;

  CL_DBG_CALL_IN("", endl);

  m_bIsCompiled = false;

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;

  //
  // No commands added.
  //
  if( m_nUidCnt == 0 )
  {
    m_log << labelFail << "No commands added." << eoe;
    LOGERROR_STREAM(getErrorStr());
    rc = EError;
  }

  //
  // Compile each command.
  //
  for(iter = m_cmdDefs.begin();
      (rc == AOk) && (iter != m_cmdDefs.end());
      ++iter)
  {
    CmdDef &cmddef = iter->second;

    if( !cmddef.isDefined() )
    {
      rc = compile(cmddef);
    }
  }

  //
  // Finalize compilation.
  //
  if( rc == AOk )
  {
    rc = finalize();
  }

  //
  // Build readline generator.
  //
  if( rc == AOk )
  {
    rc = rlBuildReadLineGenerator();
  }

  //
  // All good.
  //
  if( rc == AOk )
  {
    // register readline generator callback
    m_readline.registerAltGenerator(rlGeneratorWrapper, this);

    m_bIsCompiled = true;

    m_log.clear();
    m_log << bookmark(markExec) << labelExec << __func__ << eoe;
    m_log << labelCompile << "Compiled " << numOfCmds() << " commands." << eoe;
    m_log << labelExec << "Ok" << eoe;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc);

  return rc;
}

int CommandLine::compile(CmdDef &cmddef)
{
  TokenVec    tokens;
  ssize_t     tokcnt;
  int         i;
  int         rc;

  CL_DBG_CALL_IN("cmddef.uid=" << cmddef.getUid(), " ***" << endl);

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;
  m_log << labelCompile << "cmddef " << cmddef.getUid()
        << ": " << cmddef.getName() << eoe;

  // reset command definition to pre-compiled state
  cmddef.reset();

  for(i = 0, rc = AOk; (i < cmddef.numOfForms()) && (rc == AOk); ++i)
  {
    CmdFormDef &form = cmddef.formAt(i);

    if( (tokcnt = tokenizeSyntax(form.getSyntax(), tokens)) < 0 )
    {
      rc = tokcnt;
    }

    else if( tokcnt == 0 )
    {
      m_log << labelFail
            << "cmddef " << cmddef.m_nUid << ", "
            << "form " << form.getIndex() << ": No syntax specified." << eoe;
      LOGERROR_STREAM(getErrorStr());
      rc = EBadSyntax;
    }

    else
    {
      rc = parseSyntax(cmddef, form, tokens);
    }
  }

  if( rc == AOk )
  {
    m_log << labelExec << "Ok" << eoe;
  }
  else
  {
    m_log << labelFail << "Could not compile." << eoe;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc);
  CL_DBG(endl);

  return rc;
}

int CommandLine::finalize()
{
  CmdDefIter  iter, jter; // command definition iterators

  for(iter = m_cmdDefs.begin(); iter != m_cmdDefs.end(); ++iter)
  {
    CmdDef &cmddef_i = iter->second;
    
    jter = iter;

    for(++jter; jter != m_cmdDefs.end(); ++jter)
    {
      CmdDef &cmddef_j = jter->second;

      if( cmddef_i.getName() == cmddef_j.getName() )
      {
        m_log << labelSyntax << "Duplicate command names found at uid's "
              << cmddef_i.getUid() << " and " << cmddef_j.getUid() << "."
              << eoe;
        LOGERROR_STREAM(getErrorStr());
        return EAmbigCmd;
      }
    }
  }

  return AOk;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Line Data Section Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::addDataSection(const string          &ns,
                                void                  *pData,
                                DataSect::DeallocFunc fn)
{
  DataSectIter pos = m_dataSects.find(ns);

  if( pos == m_dataSects.end() )
  {
    m_dataSects[ns] = DataSect();       // avoid double deletes 
    m_dataSects[ns].set(ns, pData, fn); // now assign
    return AOk;
  }
  else
  {
    LOGERROR_STREAM("Data section '" << ns << "' already exist - cannot add.");
    return EBadVal;
  }
}

int CommandLine::removeDataSection(const string &ns)
{
  DataSectIter pos = m_dataSects.find(ns);

  if( pos != m_dataSects.end() )
  {
    LOGERROR_STREAM("No data section '" << ns << "' exist - cannot remove.");
    return EBadVal;
  }
  else if( isReservedDataSection(pos->first) )
  {
    LOGERROR_STREAM("Reserved data section '" << ns << "' cannot be removed.");
    return ENoExec;
  }
  else
  {
    m_dataSects.erase(pos);
    return AOk;
  }
}

void *CommandLine::getDataSection(const string &ns)
{
  DataSectIter pos = m_dataSects.find(ns);

  return pos != m_dataSects.end()? pos->second.data(): NULL;
}

bool CommandLine::isReservedDataSection(const string &ns) const
{
  return  (ns == DataSectNsCore) || (ns == DataSectNsOS) ||
          (ns == DataSectNsNet);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Line Interface Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::readCommand(FILE *fp, int &uid, int &iform, StringVec &argv)
{
  CmdExtArgVec  extargv;
  int           rc;

  argv.clear();

  uid   = NoUid;
  iform = NoIndex;

  if( ((rc = readCommand(fp, extargv)) == AOk) && (extargv.size() > 0) )
  {
    uid   = extargv[0].uid();
    iform = extargv[0].formIndex();

    toVec(extargv, argv);
  }

  return rc;
}

int CommandLine::readCommand(FILE *fp, CmdExtArgVec &argv)
{
  string  strLine;
  int     rc;

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;

  argv.clear();

  //
  // Preliminary checks.
  //
  if( numOfCmds() == 0 )
  {
    m_log << labelNoExec << "No commands added to interface." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return ENoExec;
  }
  else if( !isDefined() )
  {
    m_log << labelNoExec << "Commands not fully compiled." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return ENoExec;
  }

  //
  // Read a line of input.
  //
  if( fileno(fp) == fileno(stdin) )
  {
    strLine = m_readline.rlreadLine();
  }
  else
  {
    strLine = m_readline.ireadLine(fp);
  }

  m_log << labelInput << "stdin" << "[" << getLineNum() << "] "
    << strLine << eoe;

  //
  // Process input, match to best command.
  //
  if( (rc = processInput(strLine, argv)) == AOk )
  {
    m_log << labelExec << "Ok" << eoe;

    if( argv.size() > 0 )
    {
      LOGDIAG3_STREAM("Command "
        << m_cmdDefs[argv[0].uid()].getName()
        << "(" << argv[0].uid() << "), form "
        << argv[0].formIndex() << " matched.");
    }
  }
  else
  {
    m_log << labelFail << "Input not matched to any command." << eoe;

    LOGDIAG3("No command matched to input.");
  }

  if( getBtEnable() )
  {
    backtrace(cerr);
  }

  return rc;
}

bool CommandLine::kbhit(FILE *fp)
{
  int            fd = fileno(fp);
  struct termios ctio, ntio;
  fd_set         fdset;
  struct timeval timeout;
  bool           bAvail;

  // get current file stream parameters
  if( tcgetattr(fd, &ctio) < 0 )
  {
    return false;
  }

  ntio = ctio;

  // set parameters for non-blocking
  ntio.c_lflag &= ~ICANON;
  ntio.c_lflag &= ~ECHO;
  ntio.c_cc[VMIN] = 0;
  ntio.c_cc[VTIME] = 0;

  // apply new parameter values
  if( tcsetattr(fd, TCSANOW, &ntio) < 0 )
  {
    return false;
  }

  // build select set
  FD_ZERO(&fdset);
  FD_SET(fd, &fdset);
   
  // timeout (gets munged after each select())
  timeout.tv_sec  = (time_t)0;
  timeout.tv_usec = (time_t)10;

  bAvail = select(fd+1, &fdset, NULL, NULL, &timeout) > 0? true: false;

  // restore old parameters
  tcsetattr(fd, TCSANOW, &ctio);

  return bAvail;
}

int CommandLine::execute(const StringVec &argv)
{
  int         uid;
  CmdExecIter pos;

  // no command nor arguments
  if( argv.size() == 0 )
  {
    return EBadSyntax;
  }

  // find the added command by name (the first argument)
  CmdDef &cmd = cmdAt(argv[0]);

  // get the command's unique id
  uid = cmd.getUid();

  // no command added with this uid
  if( uid == NoUid )
  {
    return EUnknownCmd;
  }

  // find the uid associated command execution function
  if( (pos = m_cmdExecs.find(uid)) == m_cmdExecs.end() )
  {
    // no execution function
    return EUnknownCmd;
  }

  // execute
  return pos->second.execute(argv);
}

int CommandLine::execute(const CmdExtArgVec &argv)
{
  CmdExecIter pos;

  // no command nor arguments
  if( argv.size() == 0 )
  {
    return EBadSyntax;
  }

  // find the uid associated command execution function
  if( (pos = m_cmdExecs.find(argv[0].uid())) == m_cmdExecs.end() )
  {
    // no execution function
    return EUnknownCmd;
  }

  // execute
  return pos->second.execute(*this, argv);
}

void CommandLine::addToHistory(const StringVec &argv)
{
  m_readline.addToHistory(str::c14n(argv));
}

void CommandLine::addToHistory(const CmdExtArgVec &argv)
{
  StringVec v;

  toVec(argv, v);
  m_readline.addToHistory(str::c14n(v));
}

void CommandLine::pushPrompt(const string &strPrompt)
{
  m_prompts.push_back(strPrompt);
  m_readline.setPrompt(strPrompt);
}

void CommandLine::popPrompt()
{
  if( !m_prompts.empty() )
  {
    m_prompts.pop_back();
    m_readline.setPrompt(getPrompt());
  }
  else
  {
    m_readline.setPrompt(noprompt);
  }
}

const string &CommandLine::getPrompt() const
{
  if( !m_prompts.empty() )
  {
    return m_prompts.back();
  }
  else
  {
    return noprompt;
  }
}

int CommandLine::processInput(const string &strLine, CmdExtArgVec &argv)
{
  TokenVec  tokens; // tokenized input arguments
  int       argc;   // argument count
  int       rc;     // return code

  argv.clear();

  //
  // The line is empty - find out why?
  //
  if( strLine.empty() )
  {
    rc = checkReadResult();
  }

  //
  // Tokenize the input line.
  //
  else if( (argc = (int)tokenizeInput(strLine, tokens)) < 0 )
  {
    // bad token - error string already set
    rc = EBadSyntax;
  }

  //
  // Line full of whitespace - again, find out why?
  //
  else if( argc == 0 )
  {
    rc = checkReadResult();
  }

  //
  // Otherwise match the input arguments to the best form of the best fit 
  // command definition.
  //
  else
  {
    rc = match(tokens, argv);
  }

  return rc;
}

int CommandLine::match(const TokenVec &tokens, CmdExtArgVec &argv)
{
  CmdDefIter    iter;           // command definition iterator

  double        fUltFitness;    // best fitness value
  int           uidUlt;         // best command unique id
  int           iformUlt;       // best form index

  double        fPenultFitness; // second best fitness value
  int           uid2nd;         // seconds best command unique id
  int           iform2nd;       // second best form index

  CmdExtArgVec  argvCmd;        // working command extended arguments
  double        fFitness;       // working fitness
  int           cnt;            // count of commands matching argv0
  int           rc;             // return code

  // This is considered a bug. 
  if( tokens.size() == 0 )
  {
    LOGERROR("Bug: No tokens.");
    return EError;
  }

  argvCmd.clear();

  fPenultFitness  = fUltFitness = 0.0;
  uid2nd          = uidUlt      = NoUid;
  iform2nd        = iformUlt    = NoIndex;

  cnt = 0;

  //
  // Find the best and second best command fits.
  //
  for(iter = m_cmdDefs.begin(); iter != m_cmdDefs.end(); ++iter)
  {
    rc = matchCommand(iter->second, tokens, argvCmd, fFitness);

    switch( rc )
    {
      case AOk:      // complete match
        ++cnt;
        break;
      case EArgv0:  // argv0 not matched
        break;
      default:      // argv0 matched, but args failed to match 
        ++cnt;
        continue;
    }

    // Found a new best.
    if( fFitness > fUltFitness )
    {
      fPenultFitness  = fUltFitness;
      uid2nd          = uidUlt;
      iform2nd        = iformUlt;

      argv        = argvCmd;
      fUltFitness = fFitness;
      uidUlt      = iter->second.getUid();
      iformUlt    = argv[0].formIndex();
    }

    // Found a new second best, but not a new best.
    else if( fFitness > fPenultFitness )
    {
      fPenultFitness  = fFitness;
      uid2nd          = iter->second.getUid();
      iform2nd        = argvCmd[0].formIndex();
    }
  }

  //
  // No interface match.
  //
  if( uidUlt == NoUid )
  {
    if( cnt > 0 )
    {
      m_log << labelNoMatch << c14n(tokens) << eoe;
    }
    else
    {
      m_log << labelSyntax << "Command " << tokens[0] << " not found." << eoe;
    }
    rc = EUnknownCmd;
  }

  //
  // Duplicate fitness.
  //
  else if( fUltFitness == fPenultFitness )
  {
    m_log << labelNoMatch
          << "Command '" << tokens[0] << "' ambiguous. Matches:"
          << eoe;
    m_log << labelBlank << cmdAt(uidUlt).formAt(iformUlt).getSyntax() << eoe;
    m_log << labelBlank << cmdAt(uid2nd).formAt(iform2nd).getSyntax() << eoe;
    rc = EAmbigCmd;
  }

  //
  // Got a unambiguous, matched command.
  //
  else
  {
    m_log << labelSelect << cmdAt(uidUlt).formAt(iformUlt).getSyntax() << eoe;
    rc = AOk;
  }

  return rc;
}

int CommandLine::matchCommand(const CmdDef   &cmddef,
                              const TokenVec &tokens,
                              CmdExtArgVec   &argv,
                              double         &fFitness)
{
  CmdExtArgVec  argvForm;     // working form extended arguments
  double        fMaxFitness;  // working and best form fitness
  int           iBest;        // best form index
  int           i;            // working index
  int           rc;           // return code

  // This is considered a bug. 
  if( tokens.size() == 0 )
  {
    LOGERROR("Bug: No tokens.");
    return EError;
  }

  argv.clear();

  fFitness    = 0.0;
  fMaxFitness = 0.0;
  iBest       = NoIndex;

  //
  // If argv0 doesn't match, don't try to apply matching algorithm to this
  // command.
  //
  // Note: All command forms must have the same argv0 argument type and value.
  //
  if( cmddef.at(0).at(0).match(tokens[0].value()) == 0.0 )
  {
    return EArgv0;
  }

  for(i = 0; i < cmddef.numOfForms(); ++i)
  {
    const CmdFormDef &form = cmddef.at(i);

    m_log << labelTry << form.getSyntax() << eoe;

    if( (rc = matchCommandForm(form, tokens, argvForm, fFitness)) == AOk )
    {
      if( fFitness > fMaxFitness )
      {
        argv        = argvForm;
        fMaxFitness = fFitness;
        iBest       = i;
      }
    }

    m_log << labelFit << fFitness << eoe;

    if( fMaxFitness >= 1.0 )
    {
      break;
    }
  }

  //
  // Found the best command form that matches the input.
  //
  if( iBest >= 0 )
  {
    m_log << labelMatch
          << "Command '" << cmddef.getName()
          << "', form " << iBest << ": Fitness " << fMaxFitness
          << eoe;
    fFitness = fMaxFitness;
    rc = AOk;
  }

  //
  // No fit.
  //
  else
  {
    m_log << labelNoMatch
          << "Command '" << cmddef.getName()
          << "', all forms."
          << eoe;
    rc = EBadSyntax;
  }

  return rc;
}

int CommandLine::matchCommandForm(const CmdFormDef &form,
                                  const TokenVec   &tokens,
                                  CmdExtArgVec     &argv,
                                  double           &fFitness)
{
  int     argcToks = (int)tokens.size();  // number of input tokens
  int     argcForm = form.numOfArgs();    // form total number of arguments
  double  fWeight, fDecay;                // match weight and decay multiplier
  int     iArg;                           // working form argument index
  int     iTok;                           // working input tokens index
  int     rc;                             // return code

  argv.clear();

  fFitness = 0.0;
  fDecay   = 1.0;

  // command name (for logging)
  const string &strCmdName = m_cmdDefs[form.getParentCmdUid()].getName();

  // 
  // Too many input arguments.
  //
  if( argcToks > argcForm )
  {
    m_log << labelNoMatch
          << "Command '" << strCmdName << "', "
          << "form " << form.getIndex() << ": "
          << "Too many arguments: "
          << argcToks << " specified, "
          << argcForm << " maximum."
          << eoe;
    return EBadSyntax;
  }

  //
  // Missing required input arguments.
  //
  else if( argcToks < form.numOfRequiredArgs() )
  {
    m_log << labelNoMatch
          << "Command '" << strCmdName << "', "
          << "form " << form.getIndex() << ": "
          << "Missing required arguments: "
          << argcToks << " specified, "
          << form.numOfRequiredArgs() << " required."
          << eoe;
    return EBadSyntax;
  }

  //
  // Match input arguments to form syntax.
  //
  for(iArg = 0, iTok = 0, rc = AOk;
      (iArg < argcForm) && (iTok < argcToks) && (rc == AOk);
      ++iArg)
  {
    const CmdArgDef &argdef = form.at(iArg);

    // try to match input token to argument sytax
    fWeight = argdef.match(tokens[iTok].value(), m_bIgnoreCase);

    // no match
    if( fWeight == 0.0 )
    {
      rc = EBadSyntax;

      // start log entry
      m_log << labelNoMatch
            << "Input argument " << tokens[iTok] << " doesn't match "
            << "command '" << strCmdName << "', "
            << "form " << form.getIndex() << ", "
            << (argdef.isOptional()? "(opt)": "")
            << "arg " << argdef.getIndex()
            << " definition."
            << eoe;
    
      //
      // Log reason of match failure.
      //
      switch( argdef.getType() )
      {
        case CmdArgDef::ArgTypeLiteral:
          m_log << labelBlank << "  Not one of: "
                << argdef.constructLiteralList() << "."
                << eoe;
        break;

        case CmdArgDef::ArgTypeWord:
        case CmdArgDef::ArgTypeMultiWord:
        case CmdArgDef::ArgTypeIdentifier:
        case CmdArgDef::ArgTypeBoolean:
        case CmdArgDef::ArgTypeFile:
          m_log << labelBlank << "  Not a "
                << "'" << CmdArgDef::lookupArgSymbol(argdef.getType())
                << "' type." << eoe;
          break;

        case CmdArgDef::ArgTypeInteger:
        case CmdArgDef::ArgTypeFpn:
          m_log << labelBlank << "  Not a "
                << "'" << CmdArgDef::lookupArgSymbol(argdef.getType())
                << "' type";
          if( argdef.getRanges().size() > 0 )
          {
            m_log << " or out of range " << argdef.constructRangeList();
          }
          m_log << "." << eoe;
          break;

        case CmdArgDef::ArgTypeRegEx:
          m_log << labelBlank << "  Failed to match re: "
                << "'" << argdef.getRegEx() << "'."
                << eoe;
          break;

        case CmdArgDef::ArgTypeUndef:
        default:
          LOGERROR("Bug: Unknown type %d.", argdef.getType());
          rc = EError;
          break;
      }
    }

    //
    // Good argument match.
    //
    if( rc == AOk )
    {
      m_log << labelMatch
            << "Input argument " << tokens[iTok] << " matches "
            << "cmd '" << strCmdName << "', "
            << "form " << form.getIndex() << ", "
            << "arg " << argdef.getIndex() << " "
            << "definition."
            << eoe;

      argv.push_back(argdef.convert(tokens[iTok].value(), m_bIgnoreCase));

      // don't decay perfection
      if( fWeight == 1.0 )
      {
        fFitness += fWeight;
      }
      else
      {
        fFitness += fWeight * fDecay;
      }

      fDecay *= 0.9;

      ++iTok;
    }

    //
    // Bad match, but argument is optional, and there are more optionals.
    //
    else if( argdef.isOptional() && (iArg < argcForm-1) )
    {
      rc = AOk;
    }

    //
    // Bad match.
    //
    else
    {
      fFitness = 0.0;
    }
  }

  // normalize fitness
  fFitness /= (double)argcToks;

  return rc;
}

int CommandLine::checkReadResult()
{
  // EOF
  if( m_readline.isEoF() )
  {
    m_log << "EOF" << eoe;
    return EEoF;
  }

  // file read error
  else if( m_readline.isFError() )
  {
    m_log << m_readline.getErrorStr() << eoe;
    return ERead;
  }

  // read successful
  else
  {
    return AOk;
  }
}

void CommandLine::toVec(const CmdExtArgVec &v1, StringVec &v2)
{
  for(size_t i = 0; i < v1.size(); ++i)
  {
    v2.push_back(v1[i].arg());
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Info and Argument Access Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::numOfArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfArgs();
}

int CommandLine::numOfArgs(const CmdExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfArgs();
}

int CommandLine::numOfRequiredArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfRequiredArgs();
}

int CommandLine::numOfRequiredArgs(const CmdExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfRequiredArgs();
}

int CommandLine::numOfOptionalArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfOptionalArgs();
}

int CommandLine::numOfOptionalArgs(const CmdExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfOptionalArgs();
}

const string &CommandLine::getArgName(const CmdExtArg &arg) const
{
  // all terrain armored transport, of course
  const CmdArgDef &argdef= at(arg.uid()).at(arg.formIndex()).at(arg.argIndex());

  return argdef.getName();
}

CmdArgDef::ArgType CommandLine::getArgDefType(const CmdExtArg &arg) const
{
  const CmdArgDef &argdef= at(arg.uid()).at(arg.formIndex()).at(arg.argIndex());

  return argdef.getType();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ReadLine Generator Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::rlBuildReadLineGenerator()
{
  return AOk;
}

const string CommandLine::rlGeneratorWrapper(void         *pAppArg,
                                             const string &strText,
                                             int           nIndex,
                                             const string &strContext,
                                             int           nStart,
                                             int           nEnd,
                                             unsigned     &uFlags)
{
  if( pAppArg != NULL )
  {
    return ((CommandLine *)pAppArg)->rlGenerator(strText, nIndex, strContext,
                                                 nStart, nEnd, uFlags);
  }
  else
  {
    return emptystring;
  }
}

const string CommandLine::rlGenerator(const string &strText,
                                      int           nIndex,
                                      const string &strContext,
                                      int           nStart,
                                      int           nEnd,
                                      unsigned     &uFlags)
{
  static vector<CmdArgDef*> argdefs;

  StringVec tabList;
  size_t    i;

  //cerr  << "{text=" << strText
  //      << ", index=" << nIndex
  //      << ", context=" << strContext
  //      << ", start=" << nStart
  //      << ", end=" << nEnd
  //      << "}" << endl;

  if( nIndex == 0 )
  {
    //cerr << "rebuild args" << endl;
    argdefs.clear();
    rlArgDefs(strContext.substr(0, nStart), argdefs);
  }

  rlTabList(strText, argdefs, tabList, uFlags);

  if( nIndex < tabList.size() )
  {
    return tabList[nIndex];
  }
  else
  {
    return RlTabEnd;
  }
}

void CommandLine::rlArgDefs(const string       &strSubtext,
                            vector<CmdArgDef*> &argdefs)
{
  CmdDefIter          iter;         // command definition iterator
  vector<CmdArgDef*>  v[2];         // work swap vectors of argument definitions
  int                 tog0, tog1;   // vector toggle source/destination indices
  StringVec           tokens;       // input tokens
  size_t              argc;         // token working argument count
  size_t              i, j;         // working indices

  argdefs.clear();

  tokenizeInput(strSubtext, tokens);
    
  tog0 = 0, tog1 = 1;

  //
  // Seed with argv0 (command).
  //
  for(iter = m_cmdDefs.begin(); iter != m_cmdDefs.end(); ++iter)
  {
    for(j = 0; j < iter->second.numOfForms(); ++j)
    {
      CmdFormDef &form = iter->second.formAt(j);
      v[tog0].push_back(&form.argAt(0));
    }
  }

  //
  // Walk across token arguments, paring unmatched argument defs from list.
  //
  for(argc = 0; argc < tokens.size(); ++argc)
  {
    v[tog1].clear();

    for(i = 0; i < v[tog0].size(); ++i)
    {
      CmdArgDef *p = v[tog0][i];

      if( p->match(tokens[argc], m_bIgnoreCase) > 0.0 )
      {
        CmdFormDef &form =
                m_cmdDefs[p->getParentCmdUid()].formAt(p->getParentFormIndex());

        if( argc+1 < form.numOfArgs() )
        {
          v[tog1].push_back(&form.argAt(argc+1));
        }
      }
    }

    tog0 = tog1;
    tog1 = (tog1 + 1) % 2;
  }

  // copy to interface parameter
  for(i = 0; i < v[tog0].size(); ++i)
  {
    argdefs.push_back(v[tog0][i]);
  }
}

void CommandLine::rlTabList(const string       &strText,
                            vector<CmdArgDef*> &argdefs,
                            StringVec          &tabList,
                            unsigned           &uFlags)
{
  size_t                len;          // length of input text
  set<string>           A;            // set with unique keys == values
  set<string>::iterator iter, jter;   // bidirectional set iterators
  string                str;          // working string
  size_t                i, j, min;    // working indices, minimum

  // clear TAB list of all existing candidates
  tabList.clear();

  // no filename completion
  uFlags = ReadLine::FlagTabNoFilename;

  // nothing to TAB complete
  if( argdefs.size() == 0 )
  {
    return;
  }

  // length of partial text to complete
  len = strText.size();

  //
  // Build tab completion super list.
  //
  // Note: Sets automatically handle duplicates.
  //
  for(i = 0; i < argdefs.size(); ++i)
  {
    switch( argdefs[i]->getType() )
    {
      // Find all literals that partially match text.
      case CmdArgDef::ArgTypeLiteral:
        for(j = 0; j < argdefs[i]->numOfLiterals(); ++j)
        {
          str = argdefs[i]->literalAt(j);

          if( rlPartialMatch(strText, str, len) )
          {
            A.insert(str);
          }
        }
        break;

      case CmdArgDef::ArgTypeBoolean:
        for(j = 0; FalseHood[j] != NULL; ++j)
        {
          if( rlPartialMatch(strText, FalseHood[j], len) )
          {
            // bit of hack. don't want 'false' and 'f' in TAB complition list
            if( FalseHood[j] != "f" )
            {
              A.insert(FalseHood[j]);
            }
          }
        }
        for(j = 0; TruthHood[j] != NULL; ++j)
        {
          if( rlPartialMatch(strText, TruthHood[j], len) )
          {
            // bit of hack. don't want 'true' and 't' in TAB complition list
            if( TruthHood[j] != "t" )
            {
              A.insert(TruthHood[j]);
            }
          }
        }
        break;

      // Argument types not posible to TAB complete, provide syntax if no text.
      case CmdArgDef::ArgTypeWord:
      case CmdArgDef::ArgTypeMultiWord:
      case CmdArgDef::ArgTypeIdentifier:
      case CmdArgDef::ArgTypeInteger:
      case CmdArgDef::ArgTypeFpn:
      case CmdArgDef::ArgTypeRegEx:
        if( len == 0 )
        {
          A.insert(argdefs[i]->constructSyntax());
          uFlags |= ReadLine::FlagTabNoDefault;
        }
        break;

      // Filename argument type, allow filename completion.
      case CmdArgDef::ArgTypeFile:
        uFlags &= ~ReadLine::FlagTabNoFilename;
        break;

      // Unknown.
      case CmdArgDef::ArgTypeUndef:
      default:
        break;
    }
  } 

  //
  // Find the greatest common prefix substring.
  //
  if( !(uFlags & ReadLine::FlagTabNoDefault) && (A.size() > 1) )
  {
    jter = A.begin();
    iter = jter++;

    size_t  min = iter->size();

    for(; (jter != A.end()) && (min > 0); ++iter, ++jter)
    {
      j = gcss(*iter, *jter);

      if( j < min )
      {
        min = j;
      }
    }

    // Common substring is longer than text. This is the only default TAB entry.
    if( min > len )
    {
      str = *A.begin();
      A.clear();
      A.insert(str.substr(0, min));
      uFlags |= ReadLine::FlagTabNoSpace;
    }
  }

  //
  // Copy set to vector list
  //
  for(iter = A.begin(); iter != A.end(); ++iter)
  {
    tabList.push_back(*iter);
  }

  // Disable default if multiple unique matches.
  if( tabList.size() > 1 )
  {
    uFlags |= ReadLine::FlagTabNoDefault;
  }
}

bool CommandLine::rlPartialMatch(const string &strText,
                                 const string strLiteral,
                                 size_t       uLen)
{
  if( m_bIgnoreCase )
  {
    return lowercase(strText) == lowercase(strLiteral.substr(0, uLen));
  }
  else
  {
    return strText == strLiteral.substr(0, uLen);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attribute and Data Access Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool CommandLine::hasCmd(const int uid) const
{
  return m_cmdDefs.find(uid) != m_cmdDefs.end();
}

bool CommandLine::hasCmd(const string &strName) const
{
  return at(strName).isDefined();
}

const CmdDef &CommandLine::at(const int uid) const
{
  CmdDefCIter pos;

  if( (pos = m_cmdDefs.find(uid)) != m_cmdDefs.end() )
  {
    return pos->second;
  }
  else
  {
    return nocmddef;
  }
}

const CmdDef &CommandLine::at(const string &strName) const
{
  CmdDefCIter iter;

  for(iter = m_cmdDefs.begin(); iter != m_cmdDefs.end(); ++iter)
  {
    if( iter->second.getName() == strName )
    {
      return iter->second;
    }
  }

  return nocmddef;
}

CmdDef &CommandLine::cmdAt(const int uid)
{
  CmdDefIter  pos;

  if( (pos = m_cmdDefs.find(uid)) != m_cmdDefs.end() )
  {
    return pos->second;
  }
  else
  {
    return nocmddef;
  }
}

CmdDef &CommandLine::cmdAt(const string &strName)
{
  CmdDefIter  iter;

  for(iter = m_cmdDefs.begin(); iter != m_cmdDefs.end(); ++iter)
  {
    if( iter->second.getName() == strName )
    {
      return iter->second;
    }
  }

  return nocmddef;
}

const string &CommandLine::getErrorStr() const
{
  return m_log.lastText();
}

ostream &CommandLine::backtrace(ostream &os, const bool bAll) const
{
  if( bAll )
  {
    return m_log.printLog(os);
  }
  else
  {
    return m_log.printToMark(os, markExec, LogBook::NEWEST);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Lexical Analyzer Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ssize_t CommandLine::tokenizeInput(const string &strInput, TokenVec &tokens)
{
  size_t    len;      // length of input string
  ssize_t   cursor;   // cursor character position along input string
  size_t    start;    // start character position of token in input string

  CL_DBG_CALL_IN("\"" << strInput << "\"", endl);

  tokens.clear();

  len = strInput.length();

  for(cursor = 0; (cursor >= 0) && (cursor < len); )
  {
    // find start of the next token
    while( (cursor < len) && isspace((int)strInput[cursor]) )
    {
      ++cursor;
    }

    // no more tokens
    if( cursor >= len ) 
    {
      break;
    }

    // new qouted string token
    if( isdquote(strInput[cursor]) )
    {
      cursor = lexQuotedString(strInput, cursor, tokens);
    }

    // new contiguous word token
    else
    {
      cursor = lexWord(strInput, cursor, tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << tokens[i];
      sep = ", ";
    }
    CL_DBG_CALL_OUT_NL(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return cursor >= 0? (ssize_t)tokens.size(): cursor;
}

ssize_t CommandLine::tokenizeInput(const string &strInput, StringVec &tokens)
{
  TokenVec  tokenz;
  ssize_t   cnt;
  ssize_t   i;

  tokens.clear();

  cnt = tokenizeInput(strInput, tokenz);

  for(i = 0; i < cnt; ++i)
  {
    tokens.push_back(tokenz[i].value());
  }

  return cnt;
}

ssize_t CommandLine::tokenizeSyntax(const string &strSyntax, TokenVec &tokens)
{
  size_t    len;      // length of syntax string
  ssize_t   cursor;   // cursor character position along syntax string
  size_t    start;    // start character position of token in syntax string

  CL_DBG_CALL_IN("\"" << strSyntax << "\", tokens", endl);

  tokens.clear();

  len = strSyntax.length();

  for(cursor = 0; (cursor >= 0) && (cursor < len); )
  {
    // find start of the next token
    while( (cursor < len) && isspace((int)strSyntax[cursor]) )
    {
      ++cursor;
    }

    // no more tokens
    if( cursor >= len ) 
    {
      break;
    }

    // new parenthetical expression
    if( isoparen(strSyntax[cursor]) )
    {
      cursor = lexSyntaxParenExpr(strSyntax, cursor, tokens);
    }

    //  special character token
    else if( isspecial(strSyntax[cursor]) )
    {
      start = cursor++;
      pushToken(strSyntax, start, cursor, tokens);
    }

    // new contiguous word token
    else
    {
      cursor = lexSyntaxWord(strSyntax, cursor, tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << tokens[i];
      sep = ", ";
    }
    CL_DBG_CALL_OUT_NL(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return cursor >= 0? (ssize_t)tokens.size(): cursor;
}

ssize_t CommandLine::lexSyntaxWord(const string &strSyntax,
                                   ssize_t      cursor,
                                   TokenVec     &tokens)
{
  size_t    len;
  size_t    start;

  len   = strSyntax.length();
  start = cursor;

  // find end of word
  while( (cursor < len) &&
         !isspace((int)strSyntax[cursor]) &&
         !isspecial(strSyntax[cursor]) )
  {
    ++cursor;
  }

  if( cursor > start )
  {
    pushToken(strSyntax, start, cursor, tokens);
  }
  else
  {
    logLexToken(strSyntax, start, cursor, tokens);
    m_log << labelSyntax << "No <word> found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    cursor = EBadSyntax;
  }

  return cursor;
}

ssize_t CommandLine::lexSyntaxParenExpr(const string &strSyntax,
                                        ssize_t       cursor,
                                        TokenVec     &tokens)
{
  size_t    len;      // length of input string;
  size_t    start;    // start character position of token in input string
  int       pdepth;   // parenthesis depth
  bool      escseq;   // is [not] in an escape sequence
  bool      eop;      // is [not] at end of parenthetical
  char      c;        // working character
  string    value;    // working token value

  len     = strSyntax.length();
  start   = cursor;
  pdepth  = 0;
  escseq  = false;
  eop     = false;

  if( isoparen(strSyntax[cursor]) )
  {
    // '(' token
    start = cursor++;
    pushToken(strSyntax, start, cursor, tokens);

    start = cursor;
    ++pdepth;
  }
  else
  {
    logLexToken(strSyntax, start, cursor, tokens);
    m_log << labelSyntax << "No starting '(' found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return EBadSyntax;
  }

  while( (cursor >= 0) && (cursor < len) && !eop )
  {
    switch( strSyntax[cursor] )
    {
      case '\\':
        escseq = true;
        break;
      case '(':
        if( !escseq )
        {
          ++pdepth;
        }
        escseq = false;
        break;
      case ')':
        if( !escseq )
        {
          if( --pdepth == 0 )
          {
            eop = true;
          }
        }
        escseq = false;
        break;
      default:
        escseq = false;
        break;
    }

    if( !eop )
    {
      value.push_back(strSyntax[cursor++]);
    }
  }

  if( iscparen(strSyntax[cursor]) )
  {
    // expression token
    pushToken(strSyntax, start, cursor, tokens);

    // ')' token
    start = cursor++;
    pushToken(strSyntax, start, cursor, tokens);

    return cursor;
  }
  else
  {
    logLexToken(strSyntax, start, cursor, tokens);
    m_log << labelSyntax << "No ending ')' found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    return EBadSyntax;
  }
}

ssize_t CommandLine::lexWord(const string &strInput,
                             ssize_t      cursor,
                             TokenVec     &tokens)
{
  size_t    len;
  size_t    start;

  len   = strInput.length();
  start = cursor;

  while( (cursor < len) && !isspace((int)strInput[cursor]) )
  {
    ++cursor;
  }

  if( cursor > start )
  {
    pushToken(strInput, start, cursor, tokens);
  }
  else
  {
    logLexToken(strInput, start, cursor, tokens);
    m_log << labelSyntax << "No <word> found." << eoe;
    cursor = EBadSyntax;
  }

  return cursor;
}

ssize_t CommandLine::lexQuotedString(const string &strInput,
                                     ssize_t       cursor,
                                     TokenVec     &tokens)
{
  size_t    len;      // length of input string;
  size_t    start;    // start character position of token in input string
  bool      escseq;   // is [not] in an escape sequence
  bool      eos;      // is [not] at end of string
  char      c;        // working converted character
  string    value;    // working token value

  len   = strInput.length();

  if( isdquote(strInput[cursor]) )
  {
    ++cursor;
    start = cursor;
  }
  else
  {
    logLexToken(strInput, cursor, cursor, tokens);
    m_log << labelSyntax << "No starting double qoute '\"' found." << eoe;
    return EBadSyntax;
  }

  escseq  = false;
  eos     = false;

  while( (cursor >= 0) && (cursor < len) && !eos )
  {
    // escape sequence
    if( escseq )
    {
      switch( strInput[cursor] )
      {
        case 't':   // tab
          c = '\t';
          break;
        case 'n':   // newline
          c = '\n';
          break;
        case 'r':   // carriage return
          c = '\r';
          break;
        case 'v':   // vertical tab
          c = '\v';
          break;
        case 'f':   // formfeed
          c = '\f';
          break;
        case 'x':   // h[h]
          {
            int i = cursor + 1;
            if( (i < len) && isxdigit(strInput[i]) )
            {
              c = tohex(strInput[i]);
              ++cursor;
              ++i;
              if( (i < len) && isxdigit(strInput[i]) )
              {
                c <<= 4;
                c |= tohex(strInput[i]);
                ++cursor;
              }
            }
            else
            {
              c = 'x';
            }
          }
          break;
        default:
          c = strInput[cursor];
          break;
      }

      value.push_back(c);
      escseq = false;
      ++cursor;
    }

    else
    {
      switch( strInput[cursor] )
      {
        case '\\':
          escseq = true;
          break;
        case '"':
          eos = true;
          break;
        default:
          value.push_back(strInput[cursor++]);
          break;
      }
    }
  }

  if( isdquote(strInput[cursor]) )
  {
    pushToken(strInput, start, cursor, tokens);

    ++cursor;   // get past quote

    return cursor;
  }
  else
  {
    logLexToken(strInput, start, cursor, tokens);
    m_log << labelSyntax << "No ending double qoute '\"' found." << eoe;
    return EBadSyntax;
  }
}

void CommandLine::logLexToken(const string  &strSource,
                              const size_t  start,
                              const ssize_t cursor,
                              TokenVec      &tokens,
                              const bool    bLoc)
{
  stringstream ss;

  pushToken(strSource, start, cursor, tokens);
  tokens.back().printAnnotated(ss, strSource);
  m_log << labelAt << ss.str() << eoe;
}

void CommandLine::pushToken(const string  &strSource,
                            const size_t  start,
                            const ssize_t cursor,
                            TokenVec      &tokens)
{
  Token tok(strSource.substr(start, cursor-start),
            getLineNum(), start, cursor-1);
  tokens.push_back(tok);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Extended Usage Syntax Parsing Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::parseSyntax(CmdDef         &cmddef,
                             CmdFormDef     &form,
                             const TokenVec &tokens)
{
  size_t  tokcnt  = tokens.size();
  size_t  pos     = 0;
  bool    bOk     = true;
  int     rc;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, 0, endl);

  m_log << labelParse
        << "cmddef " << cmddef.getUid() << ", "
        << "form " << form.getIndex() << ": "
        << form.getSyntax()
        << eoe;

  // command
  if( !parseArgv0(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parssing special argv0." << eoe;
    bOk = false;
  }

  // required argmuents - may be none
  else if( !parseRequiredArgList(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parsing required argument list." << eoe;
    bOk = false;
  }

  // optional arguments - may be none
  else if( !parseOptionalArgList(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parsing optional argument list." << eoe;
    bOk = false;
  }

  // final checks
  else if( pos < tokcnt )
  {
    m_log << labelFail
          << "Extraneous tokens found after optional arguments."
          << eoe;
    bOk = false;
  }

  // all good
  if( bOk )
  {
    m_log << labelParse << "Ok" << eoe;
    LOGDIAG2_STREAM("Command("
        << "uid=" << cmddef.getUid() << ", "
        << "name=" << cmddef.getName() << ", "
        << "form=" << form.getIndex() << ", "
        << "argc=" << form.numOfArgs() << ") "
        << "successfully parsed.");
    rc = AOk;
  }

  // not so good - log error
  else
  {
    LOGERROR_STREAM(getErrorStr());
    rc = EBadSyntax;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc << ", " << okstr(bOk) << ", pos=" << pos);

  return rc;
}

bool CommandLine::parseArgv0(CmdDef         &cmddef,
                             CmdFormDef     &form,
                             const TokenVec &tokens,
                             size_t         &pos)
{
  bool    bOk;
  string  strName;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  if( pos < tokens.size() )
  {
    if( peekEq("<", tokens[pos]) )
    {
      bOk = parseVariableArg(cmddef, form, tokens, pos);
    }
    else
    {
      bOk = parseLiteralArg(cmddef, form, tokens, pos);
    }
  }
  else
  {
    m_log << labelSyntax << "Command argument not found in form "
          << form.getIndex() << "."
          << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  if( bOk )
  {
    form.m_nArgcReq++;

    form.lastArg().orFlags(CmdArgDef::FlagCommand);

    // set command's name
    if( form.lastArg().getType() == CmdArgDef::ArgTypeLiteral )
    {
      // literals are anonymous, use the literal itself
      strName = form.lastArg().literalAt(0);
    }
    else
    {
      // variables have required names
      strName = form.lastArg().getName();
    }

    // assign command name
    if( cmddef.getName().empty() )
    {
      cmddef.setName(strName);
    }
    // all forms must have the same command name
    else if( cmddef.getName() != strName )
    {
      m_log << labelSyntax << "Expected command name '" << cmddef.getName()
            << "', but found '" << strName << "' in form " << form.getIndex()
            << "."
            << eoe;
      LOGERROR_STREAM(getErrorStr());
      bOk = false;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseRequiredArgList(CmdDef         &cmddef,
                                       CmdFormDef     &form,
                                       const TokenVec &tokens,
                                       size_t         &pos)
{
  bool  bOk = true;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( bOk && (pos < tokens.size()) )
  {
    // peek if start of optional arguments
    if( peekEq("[", tokens[pos]) )
    {
      break;
    }
    
    // required argument
    else if( (bOk = parseArg(cmddef, form, tokens, pos)) )
    {
      form.m_nArgcReq++;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseOptionalArgList(CmdDef         &cmddef,
                                       CmdFormDef     &form,
                                       const TokenVec &tokens,
                                       size_t         &pos)
{
  bool  bOk = true;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( bOk && (pos < tokens.size()) )
  {
    // peek if start an optional arguments
    if( !peekEq("[", tokens[pos]) )
    {
      break;
    }
    
    // "[ argument ]"
    if( tokEq("[", tokens, pos) &&
        parseArg(cmddef, form, tokens, pos) &&
        tokEq("]", tokens, pos) )
    {
      form.lastArg().orFlags(CmdArgDef::FlagOptional);
      form.m_nArgcOpt++;
    }
    else
    {
      bOk = false;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseArg(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  if( pos < tokens.size() )
  {
    if( peekEq("{", tokens[pos]) )
    {
      bOk = parseXorListArg(cmddef, form, tokens, pos);
    }
    else if( peekEq("<", tokens[pos]) )
    {
      bOk = parseVariableArg(cmddef, form, tokens, pos);
    }
    else
    {
      bOk = parseLiteralArg(cmddef, form, tokens, pos);
    }
  }
  else
  {
    m_log << labelSyntax << "Command argument not found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorListArg(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  StringVec literals;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = tokEq("{", tokens, pos) &&
        parseXorList(cmddef, form, tokens, pos, literals) &&
        tokEq("}", tokens, pos);

  if( bOk )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::ArgTypeLiteral);
    form.lastArg().orFlags(CmdArgDef::FlagXorList);

    for(size_t i = 0; i < literals.size(); ++i)
    {
      form.lastArg().addLiteralValue(literals[i]);
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVariableArg(CmdDef         &cmddef,
                                   CmdFormDef     &form,
                                   const TokenVec &tokens,
                                   size_t         &pos)
{
  bool                bOk;
  CmdArgDef           argdef;
  string              strName;
  CmdArgDef::ArgType  eType;
  CmdArgDef::RangeVec ranges;
  RegEx               re;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = tokEq("<", tokens, pos) &&
        parseIdentifier(cmddef, form, tokens, pos, strName);

  if( bOk )
  {
    // variable modifier
    if( peekEq(":", tokens[pos]) )
    {
      bOk = tokEq(":", tokens, pos) &&
            parseVarMod(cmddef, form, tokens, pos, eType, ranges, re);
    }
    // default
    else
    {
      eType = CmdArgDef::ArgTypeWord;
    }
  }

  bOk = bOk && tokEq(">", tokens, pos);

  if( bOk )
  {
    form.lastArg().setName(strName);
    form.lastArg().setType(eType);
    form.lastArg().setRanges(ranges);
    form.lastArg().setRegEx(re);
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseLiteralArg(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  string    strValue;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue);

  if( bOk )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::ArgTypeLiteral);
    form.lastArg().addLiteralValue(strValue);
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorList(CmdDef         &cmddef,
                               CmdFormDef     &form,
                               const TokenVec &tokens,
                               size_t         &pos,
                               StringVec      &literals)
{
  bool    bOk   = true;
  bool    more = true;
  string  strValue;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( more && bOk && (pos < tokens.size()) )
  {
    if( peekEq("}", tokens[pos]) )
    {
      m_log << labelSyntax << "Expected literal value but found '}'." << eoe;
      LOGERROR_STREAM(getErrorStr());
      bOk = false;
    }

    else if( (bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue)) )
    {
      literals.push_back(strValue);

      // peek if start of another xor option
      if( peekEq("|", tokens[pos]) )
      {
        bOk = tokEq("|", tokens, pos);
      }
      else
      {
        more = false;
      }
    }
  }

  if( more && bOk )
  {
    m_log << labelSyntax << "No literal value found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseIdentifier(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos,
                                  string         &strIdent)
{
  bool    bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  strIdent.clear();

  if( pos < tokens.size() )
  {
    strIdent = tokens[pos].value();
    bOk = tokIdentifier(tokens, pos);
  }
  else
  {
    m_log << labelSyntax << "No identifier found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", identifier='" << strIdent
      << "', pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarMod(CmdDef              &cmddef,
                              CmdFormDef          &form,
                              const TokenVec      &tokens,
                              size_t              &pos,
                              CmdArgDef::ArgType  &eType,
                              CmdArgDef::RangeVec &ranges,
                              RegEx               &re)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  bOk = parseVarType(cmddef, form, tokens, pos, eType);

  if( bOk )
  {
    if( peekEq("(", tokens[pos]) )
    {
      switch( eType )
      {
        case CmdArgDef::ArgTypeInteger:
        case CmdArgDef::ArgTypeFpn:
          bOk = tokEq("(", tokens, pos) &&
                parseVarRangeExpr(cmddef, form, tokens, pos, ranges) &&
                tokEq(")", tokens, pos);
          break;
        case CmdArgDef::ArgTypeRegEx:
          bOk = tokEq("(", tokens, pos) &&
                parseVarRegExpr(cmddef, form, tokens, pos, re) &&
                tokEq(")", tokens, pos);
          break;
        default:
          m_log << labelSyntax
                << "Unexpected '(' token found for argument type "
                << "'" << CmdArgDef::lookupArgSymbol(eType) << "'."
                << eoe;
          LOGERROR_STREAM(getErrorStr());
          bOk = false;
      }
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", type=" << eType << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarType(CmdDef             &cmddef,
                               CmdFormDef         &form,
                               const TokenVec     &tokens,
                               size_t             &pos,
                               CmdArgDef::ArgType &eType)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  eType = CmdArgDef::ArgTypeUndef;

  if( pos < tokens.size() )
  {
    eType = CmdArgDef::lookupArgType(tokens[pos].value());

    if( eType != CmdArgDef::ArgTypeUndef )
    {
      ++pos;
      bOk = true;
    }
    else
    {
      m_log << labelSyntax << "Unknown variable type '" << tokens[pos].value()
            << "'." << eoe;
      LOGERROR_STREAM(getErrorStr());
      bOk = false;
    }
  }

  else
  {
    m_log << labelSyntax << "No variable type found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", type=" << eType << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarRangeExpr(CmdDef              &cmddef,
                                    CmdFormDef          &form,
                                    const TokenVec      &tokens,
                                    size_t              &pos,
                                    CmdArgDef::RangeVec &ranges)
{
  bool              bOk = true;
  StringVec         subranges;
  StringVec         minmax;
  CmdArgDef::range  r;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  ranges.clear();

  if( pos < tokens.size() )
  {
    split(tokens[pos].value(), ',', subranges);

    for(size_t i = 0; bOk && (i < subranges.size()); ++i)
    {
      minmax.clear();

      split(subranges[i], ':', minmax);

      if( minmax.size() == 0 )
      {
        m_log << labelSyntax << "No 'min[:max]' subrange found in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }
      else if( minmax.size() > 2 )
      {
        m_log << labelSyntax << "Too many 'min:max:?' subrange values found in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }

      if( todouble(minmax[0], r.min) != OK )
      {
        m_log << labelSyntax << "Subrange min '" << minmax[0] << "' NaN in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }

      if( minmax.size() == 2 )
      {
        if( todouble(minmax[1], r.max) != OK )
        {
          m_log << labelSyntax << "Subrange max '" << minmax[1] << "' NaN in "
                << "'" << tokens[pos].value() << "'.'"
                << eoe;
          bOk = false;
          break;
        }
      }
      else
      {
        r.max = r.min;
      }

      // reverse ranges are ok, just swap order
      if( r.max < r.min )
      {
        double tmp;

        r.min = tmp;
        r.min = r.max;
        r.max = r.min;
      }

      ranges.push_back(r);
    }
  }

  // everything is cool, but no ranges specified
  if( bOk && (ranges.size() == 0) )
  {
    m_log << labelSyntax << "Range expression not found." << eoe;
    bOk = false;
  }

  if( bOk )
  {
    ++pos; // advance token parse position
  }
  else
  {
    LOGERROR_STREAM(getErrorStr());
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", numranges=" << ranges.size()
      << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarRegExpr(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos,
                                  RegEx          &re)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  if( pos < tokens.size() )
  {
    if( m_bIgnoreCase )
    {
      re.setFlags(RegEx::ReFlagICase);
    }

    re = tokens[pos++].value();

    if( re.isValid() )
    {
      bOk = true;
    }
    else
    {
      m_log << labelSyntax << "Regular expression '"
            << "'" << re.getRegEx() << "': "
            << re.getErrorStr() << "."
            << eoe;
      LOGERROR_STREAM(getErrorStr());
      bOk = false;
    }
  }

  else
  {
    m_log << labelSyntax << "Regular expression not found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", re='" << re.getRegEx() << "', pos="
      << pos);

  return bOk;
}

bool CommandLine::parseLiteralValue(CmdDef         &cmddef,
                                    CmdFormDef     &form,
                                    const TokenVec &tokens,
                                    size_t         &pos,
                                    string         &strValue)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  strValue.clear();

  if( pos < tokens.size() )
  {
    strValue = tokens[pos++].value();
    bOk = true;
  }
  else
  {
    m_log << labelSyntax << "Literal value not found." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", value='" << strValue << "', "
      << "pos=" << pos);

  return bOk;
}

bool CommandLine::tokEq(const string   strCmp,
                        const TokenVec &tokens,
                        size_t         &pos)
{
  bool  bOk;

  CL_DBG_CALL_IN("input, strcmp=\"" << strCmp << "\", "
      << "tokens, pos=" << pos, " ");

  if( pos >= tokens.size() )
  {
    m_log << labelSyntax << "Token " << pos << " '" << strCmp
          << "' does not exist." << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  else if( tokens[pos].value() != strCmp )
  {
    m_log << labelSyntax << "Expected token '" << strCmp << "'"
          << ", but found " << tokens[pos]
          << eoe;
    LOGERROR_STREAM(getErrorStr());
    bOk = false;
  }

  else
  {
    ++pos;
    bOk = true;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::tokIdentifier(const TokenVec &tokens, size_t &pos)
{
  bool    bOk;

  CL_DBG_CALL_IN("tokens=" << tokens.size() << ", pos=" << pos, " ");

  bOk = isIdentifier(tokens[pos].value());

  if( bOk )
  {
    ++pos;
  }
  else
  {
    m_log << labelSyntax << "Invalid identifier " << tokens[pos] << eoe;
    LOGERROR_STREAM(getErrorStr());
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Convenience Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

string CommandLine::c14n(const TokenVec &tokens)
{
  stringstream  ss;
  string        sep;

  for(size_t i = 0; i < tokens.size(); ++i)
  {
    ss << sep << prettify(tokens[i].value());

    if( i == 0 )
    {
      sep = " ";
    }
  }

  return ss.str();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Output Methods and Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &rnr::cmd::operator<<(ostream &os, const CommandLine &cl)
{
  string  sep;

  //
  // start of command line interface
  //
  os << setindent(0);
  os << "CommandLine " << cl.m_strName << endl;
  os << "{" << endl;

  os << deltaindent(2);

  //
  // attributes
  //
  os << indent() << "name          = " << cl.m_strName << endl;
  os << indent() << "have_readline = " << cl.m_readline.haveRlLib() << endl;
  os << indent() << "use_readline  = " << cl.m_readline.useRlLib() << endl;
  os << indent() << "prompt        = " << cl.getPrompt() << endl;
  os << indent() << "ignorecase    = " << cl.m_bIgnoreCase << endl;

  //
  // command definitions map
  //
  os << indent() << "cmddefs[" << cl.numOfCmds() << "] =" << endl;
  os << indent() << "{" << endl;
  os << deltaindent(2);
  for(CmdDefCIter iter = cl.m_cmdDefs.begin();
      iter != cl.m_cmdDefs.end();
      ++iter)
  {
    os << sep << iter->second;
    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;
  os << deltaindent(-2);
  os << indent() << "}" << endl;

  //
  // command execution functions
  //
  os << indent() << "execs[" << cl.m_cmdExecs.size() << "] =" << endl;
  os << indent() << "{" << endl;
  os << deltaindent(2);
  sep.clear();
  for(CmdExecCIter iter = cl.m_cmdExecs.begin();
      iter != cl.m_cmdExecs.end();
      ++iter)
  {
    os << sep << iter->second;
    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;
  os << deltaindent(-2);
  os << indent() << "}" << endl;

  //
  // command data sections
  //
  os << indent() << "ds[" << cl.m_dataSects.size() << "] =" << endl;
  os << indent() << "{" << endl;
  os << deltaindent(2);
  sep.clear();
  for(DataSectCIter iter = cl.m_dataSects.begin();
      iter != cl.m_dataSects.end();
      ++iter)
  {
    os << sep << iter->second;
    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;
  os << deltaindent(-2);
  os << indent() << "}" << endl;

  //
  // end of command line interface
  //
  os << deltaindent(-2);
  os << indent() << "}";
  os << setindent(0);

  return os;
}
