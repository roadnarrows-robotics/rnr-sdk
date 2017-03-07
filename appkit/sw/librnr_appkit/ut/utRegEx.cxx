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
 * \par Copyright:
 * (C) 2017  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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
  cout << " ... RE " << name << endl;

  os << "getRegEx()      " << re.getRegEx() << endl;
  os << "isValid()       " << re.isValid() << endl;
  os << "getFlags()      " << re.getFlags() << endl;
  os << "getReturnCode() " << re.getReturnCode() << endl;
  os << "getErrorStr()   " << re.getErrorStr() << endl;
  os << "operator<<()    " << re << endl;
}

/*!
 * \brief Test RegEx operations.
 */
static void utRun()
{
  string            cmd, arg;
  RegEx::ReMatchVec matches;

  cout << "  Test Run-Time Operations:" << endl;
  cout << "q         - Quit." << endl;
  cout << "n <re>    - Specify new re." << endl;
  cout << "m <input> - Match input to re." << endl;
  cout << "a <input> - Match all input substrings to re." << endl;
  cout << "p         - Print re." << endl;
  cout << endl;

  while( true )
  {
    if( cin.fail() )
    {
      cin.clear();
      cin.ignore(INT_MAX, '\n');    // awkward flush
    }

    cout << "cmd> ";
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
        cout << "bad input" << endl;
      }
      else if( !reGo.isValid() )
      {
        cout << "bad re: " << reGo.getErrorStr() << endl;
      }
      else
      {
        cout << "valid new re" << endl;
      }
    }
    else if( cmd == "m" )
    {
      arg = getline(cin);

      cout << "match on '" << arg << "'" << endl;
      if( reGo.match(arg) )
      {
        cout << "match" << endl;
      }
      else
      {
        cout << "no match" << endl;
      }
    }
    else if( cmd == "a" )
    {
      arg = getline(cin);

      cout << "match all on '" << arg << "'" << endl;

      reGo.match(arg, matches);
      for(size_t i = 0; i < matches.size(); ++i)
      {
        cout << i << ". (" << matches[i].m_uStart << ","
          << matches[i].m_uEnd << ") '"
          << matches[i].m_strMatch << "'" << endl;
      }
    }
    else if( cmd == "p" )
    {
      utPr(cout, "Go", reGo);
    }
    else
    {
      cout << "'" << cmd << "': Bad command. One of: a m n p q" << endl;
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
  utPr(cout, "Go", reGo);

  //
  // Test run-time
  //
  utRun();

  return APP_EC_OK;
}

/*!
 * \}
 */
