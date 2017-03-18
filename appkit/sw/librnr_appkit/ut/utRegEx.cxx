////////////////////////////////////////////////////////////////////////////////
//
// Package:   appkit
//
// Program:   utRegEx   
//
// File:      utRegEx.cxx
//
/*! \file
 *
 * \brief Unit test RegEx class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2017-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/appkit/RegEx.h"

#include "version.h"

using namespace std;
using namespace rnr;

/*!
 * \ingroup apps
 * \defgroup unittest utRegEx
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;              ///< the command
static double   OptsHz    = 2;      ///< thread hertz rate

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Unit test librnr_appkit RegEx class.",

  // long_desc = 
  "The %P command unit tests the librnr_appkit RegEx class operation.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  {NULL, }
};

/*!
 * \brief Globally constructed, pre-compiled re's.
 */
const RegEx re0;
const RegEx re1("^[a-z]*$", RegEx::ReFlagICase);
RegEx re2 = re1;
const RegEx re3("['\"]{1,2}([0-9]+)(hello)(^)");
const RegEx re4("^[a-z]*$");
RegEx reGo = "([cC]at)|([dD]og)";

static string getline(istream &is)
{
  char    buf[256];
  char    *s;

  is.getline(buf, sizeof(buf));

  for(s = buf; *s && isspace(*s); ++s);

  return string(s);
}

/*!
 * \brief Print RegEx data.
 *
 * \param re    RegEx object.
 */ 
static void utPr(ostream &os, const string name, const RegEx &re)
{
  os << " ... RE " << name << endl;

  os << "getRegEx()      " << re.getRegEx() << endl;
  os << "isValid()       " << re.isValid() << endl;
  os << "getFlags()      " << re.getFlags() << endl;
  os << "getReturnCode() " << re.getReturnCode() << endl;
  os << "getErrorStr()   " << re.getErrorStr() << endl;
  os << "operator<<()    " << re << endl;
}

/*!
 * \brief Print RegEx data.
 *
 * \param re    RegEx object.
 */ 
static void utConstRe(ostream &os, const RegEx &re)
{
  const char *testinputs[] =
  {
    "abcd",
    "ABCD",
    "wXyZ",
    "12",
    "jkl mno",
    NULL
  };

  os << "  Test Constant RegEx:" << endl;

  utPr(os, "sut", re);

  for(size_t i = 0; testinputs[i] != NULL; ++i)
  {
    string input(testinputs[i]);

    os << input << " --> ";

    if( re.match(input) )
    {
      os << "match";
    }
    else
    {
      os << "no match";
    }
    os << endl;
  }
}

/*!
 * \brief Test RegEx operations.
 */
static void utRun(ostream &os)
{
  string            cmd, arg;
  RegEx::ReMatchVec matches;

  os << "  Test Run-Time Operations:" << endl;
  os << "q         - Quit." << endl;
  os << "n <re>    - Specify new re." << endl;
  os << "m <input> - Match input to re." << endl;
  os << "a <input> - Match all input substrings to re." << endl;
  os << "p         - Print re." << endl;
  os << endl;

  while( true )
  {
    if( cin.fail() )
    {
      cin.clear();
      cin.ignore(INT_MAX, '\n');    // awkward flush
    }

    os << "cmd> ";
    cin >> cmd;


    if( cmd == "q" )
    {
      return;
    }
    else if( cmd == "n" )
    {
      cin >> reGo;
      if( cin.fail() )
      {
        os << "bad input" << endl;
      }
      else if( !reGo.isValid() )
      {
        os << "bad re: " << reGo.getErrorStr() << endl;
      }
      else
      {
        os << "new valid re" << endl;
      }
    }
    else if( cmd == "m" )
    {
      arg = getline(cin);

      os << "try match on '" << arg << "' --> ";
      if( reGo.match(arg) )
      {
        os << "match" << endl;
      }
      else
      {
        os << "no match" << endl;
      }
    }
    else if( cmd == "a" )
    {
      arg = getline(cin);

      os << "try match all on '" << arg << "'" << endl;

      reGo.match(arg, matches);

      for(size_t i = 0; i < matches.size(); ++i)
      {
        os << i << ". (" << matches[i].m_uStart << ","
          << matches[i].m_uEnd << ") '"
          << matches[i].m_strMatch << "'" << endl;
      }
    }
    else if( cmd == "p" )
    {
      utPr(os, "Go", reGo);
    }
    else
    {
      os << "'" << cmd << "': Bad command. One of: a m n p q" << endl;
    }
  }
}

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);
}

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  mainInit(argc, argv);

  //
  // Test pre-compiled constructors
  //
  cout << "  Test Pre-Compiled:" << endl;
  utPr(cout, "0", re0);
  utPr(cout, "1", re1);
  utPr(cout, "copy of 1", re2);
  utPr(cout, "3", re3);
  utPr(cout, "4", re4);
  utPr(cout, "Go", reGo);
  cout << endl;

  // Constant regex match
  utConstRe(cout, re1);
  utConstRe(cout, re4);
  cout << endl;

  //
  // Test run-time
  //
  utRun(cout);

  return APP_EC_OK;
}

/*!
 * \}
 */
