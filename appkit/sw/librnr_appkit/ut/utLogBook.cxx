////////////////////////////////////////////////////////////////////////////////
//
// Package:   appkit
//
// Program:   utLogBook   
//
// File:      utLogBook.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Unit test LogBook class.
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
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/appkit/LogBook.h"

#include "version.h"

using namespace std;
using namespace rnr;

/*!
 * \ingroup apps
 * \defgroup unittest utCommandLine
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;              ///< the command

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Unit test librnr_appkit LogBook class.",

  // long_desc = 
  "The %P command unit tests the librnr_appkit LogBook operation.",

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

static string HdrSep("-------------------------------------------------------");

static void printTestHdr(const string strSynopsis)
{
  cout << endl << HdrSep << endl;
  cout << strSynopsis << endl;
  cout << HdrSep << endl;
}

static void captainsLogMuddsWomen(LogBook &log)
{
  log.setName("Captain's Log, USS Enterprise - Mudd's Women");

  log << bookmark("Captain's Log, Stardate 1328.8.")
      << "The USS Enterprise in pursuit of an unidentified vessel."
      << eoe;

  log << bookmark("Captain's Log, Stardate 1329.1.")
      <<
"We've taken aboard from unregistered transport vessel, its captain,\n"
"and three unusual females. These women have a mysterious magnetic effect on\n"
"the male members of my crew. Including myself. Explanation unknown at present."
      << eoe;

  log << bookmark("Captain's Log, Stardate 1329.2.")
     <<
"On board the USS Enterprise, a ship's hearing has been convened against the\n"
"transport vessel's captain. I'm becoming concerned about the almost hypnotic\n"
"effect produced by the women."
     << eoe;

  log << bookmark("Captain's Log, Stardate 1330.1.")
      <<
"Position, 14 hours out of Rigel XII. We're on auxiliary impulse engines.\n"
"Fuel low, barely sufficient to achieve orbit over the planet. Lithium\n"
"replacements are now imperative. The effect of Mudd's women on my crew\n"
"continues to grow, still totally unexplained. Harry Mudd is confined to his\n"
"quarters under guard."
      << eoe;

  log << bookmark("Captain's Log, Supplemental 1.")
      <<
"Transporting down to surface of planet Rigel XII to acquire replacement\n"
"lithium crystals. Expect further difficulty from miners."
      << eoe;

  log << bookmark("Captain's Log, Supplemental 2.")
      <<
"I've transported aboard the Enterprise to implement search with infrared\n"
"scanners and sensing system. Magnetic storms on the planet's surface are\n"
"cutting down speed and efficiency of our equipment. Search now in progress\n"
"for three hours, 18 minutes."
      << eoe;

  log << bookmark("Captain's Log, Supplemental 3.")
      <<
"Have expended all but 43 minutes of power. Ship's condition: critical.\n"
"Search now in progress, 7 hours, 31 minutes. Magnetic storms are easing."
      << eoe;
}

static void captainsLogManTrap(LogBook &log)
{
  log.setName("Captain's Log, USS Enterprise - The Man Trap");

  log << bookmark("Captain's Log, Stardate 1513.1.")
    <<
"Our position, orbiting planet M-113. On board the Enterprise, Mr. Spock,\n"
"temporarily in command. On the planet, the ruins of an ancient and long dead\n"
"civilization. Ship's surgeon McCoy and myself are now beaming down to the\n"
"planet's surface. Our mission, routine medical examination of archaeologist\n"
"Robert Crater, and his wife, Nancy. Routine, but for the fact that Nancy\n"
"Crater is that one woman in Dr. McCoy's past."
    << eoe;

  log << bookmark("Captain's Log, additional entry 1.")
      <<
"Since our mission was routine, we had beamed down to the planet without\n"
"suspicion. We were totally unaware that each member of the landing party was\n"
"seeing a different woman. A different Nancy Crater."
      << eoe;

  log << bookmark("Captain's Log, Stardate 1513.4.")
      <<
"In orbit around planet M-113. One crewman, member of the landing party, dead\n"
"by violence. Cause, unknown. We are certain the cause of death was not poison."
      << eoe;

  log << bookmark("Captain's Log, Stardate 1513.8.")
      <<
"I am now certain that the violent death of my crewman was caused by some\n"
"strange lifeform."
      << eoe;

  log << bookmark("Captain's Log, additional 2.")
      <<
"Armed and able-bodied crewmen are not attacked and slaughtered this easily.\n"
"Apparently the killer can immobilize them as it approaches, perhaps with\n"
"some hypnotic or paralyzing power. The answer lies with Professor Crater."
      << eoe;

  log << bookmark("Captain's Log, continuing.")
      <<
"The Enterprise has been invaded by a creature capable of assuming any form,\n"
"and with the capacity to paralyze and draw the life from any one of us."
      << eoe;
}

static void printMarks(LogBook &log, int whence)
{
  LogBook::BookMarkList list;

  cout << "+getBookMarks, whence = " << whence << endl;
  size_t n = log.getBookMarks(list, whence);

  for(size_t i = 0; i < n; ++i)
  {
    cout << i << ": \"" << list[i].m_strMark
      << "\", "  << list[i].m_index << endl;
  }
}

static void testLogAttrs(LogBook &log, bool bIsTest = true)
{
  if( bIsTest )
  {
    printTestHdr("Test LogBook Attributes");
  }

  cout << "Name:       " << log.getName() << endl;
  cout << "max_size:   " << log.max_size() << endl;
  cout << "size:       " << log.size() << endl;
  cout << "total_ever: " << log.numOfTotalEver() << endl;
  cout << "flags:      " << "0x" << hex << log.getFlags() << dec << endl;
}

static void testLogEdits(LogBook &log)
{
  char tcnt = 'A';

  printTestHdr("Test LogBook Edits");

  cout << endl << "++Current State" << endl;

  // print bookmarks and text only
  log.setFlags(LogBook::FlagOMark);
  cout << "+log.getflags 0x" << hex << log.getFlags() << dec << endl;

  //log.orFlags(LogBook::FlagDebug);

  cout << "+log attrs" << endl;
  testLogAttrs(log, false);

  cout << "+cout << log" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test creating a default log and inserting some entries.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+create logMudd with defaults" << endl;

  LogBook logMudd;

  logMudd.setFlags(LogBook::FlagOMark);

  cout << "+insert entries" << endl;
  captainsLogMuddsWomen(logMudd);

  cout << "+logMudd attrs" << endl;
  testLogAttrs(logMudd, false);

  cout << "+cout << logMudd" << endl;
  cout << logMudd;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test creating a another log and inserting some entries.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+create logTrap with 24 max entries" << endl;

  LogBook logTrap("Temp Name", 24);

  logTrap.setFlags(LogBook::FlagOMark);

  cout << "+insert entries" << endl;
  captainsLogManTrap(logTrap);

  cout << "+logTrap attrs" << endl;
  testLogAttrs(logTrap, false);

  cout << "+cout << logTrap" << endl;
  cout << logTrap;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test assignment with populated log
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+logTrap = logMudd" << endl;
  logTrap = logMudd;

  cout << "+logTrap attrs" << endl;
  testLogAttrs(logTrap, false);

  cout << "+cout << logTrap" << endl;
  cout << logTrap;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test merge with populated log
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+logTrap << logMudd" << endl;
  logTrap << logMudd;

  cout << "+logTrap attrs" << endl;
  testLogAttrs(logTrap, false);

  cout << "+cout << logTrap" << endl;
  cout << logTrap;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test clear
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+log.clear" << endl;
  log.clear();

  cout << "+log attrs" << endl;
  testLogAttrs(log, false);

  cout << "+cout << log" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test merge with cleared log of smaller size
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+re-creating logTrap entries" << endl;
  logTrap.clear();
  captainsLogManTrap(logTrap);

  cout << "+log << logMudd << logTrap" << endl;
  log << logMudd << logTrap;

  cout << "+log attrs" << endl;
  testLogAttrs(log, false);

  cout << "+cout << log" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test erase entries.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  size_t  n = 2;
  int     whence = LogBook::OLDEST;

  cout << "+log.eraseEntries(n=" << n << ", whence=" << whence << ")" << endl;
  log.eraseEntries(n, whence);

  cout << "+cout << log" << endl;
  cout << log;

  cout << endl << "++Test " << tcnt++ << endl;
  whence = LogBook::NEWEST;

  cout << "+log.eraseEntries(n=" << n << ", whence=" << whence << ")" << endl;
  log.eraseEntries(n, whence);

  cout << "+cout << log" << endl;
  cout << log;

  cout << "+log attrs" << endl;
  testLogAttrs(log, false);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test erase entries to mark.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  LogBook::BookMarkList list;
  string mark;

  whence = LogBook::OLDEST;
  n = log.getBookMarks(list, whence);

  mark = list[1].m_strMark;

  cout << "+log.eraseEntries(mark=\"" << mark << "\", whence="
    << whence << ")" << endl;
  log.eraseToMark(mark, whence);

  cout << "+cout << log" << endl;
  cout << log;

  cout << endl << "++Test " << tcnt++ << endl;
  whence = LogBook::NEWEST;
  mark = list[list.size()-1].m_strMark;

  cout << "+log.eraseEntries(mark=\"" << mark << "\", whence="
    << whence << ")" << endl;
  log.eraseToMark(mark, whence);

  cout << "+cout << log" << endl;
  cout << log;

  cout << "+log attrs" << endl;
  testLogAttrs(log, false);
}

static void testLogOutput(LogBook &log)
{
  char tcnt = 'A';

  printTestHdr("Test LogBook Output");

  // print all fields
  log.setFlags(LogBook::FlagOAllF);
  cout << "+getflags 0x" << hex << log.getFlags() << dec << endl;

  //log.orFlags(LogBook::FlagDebug);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test full output, forward direction
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+operator<<(os, log), forward" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test full output, forward direction
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  log.orFlags(LogBook::FlagORev);
  cout << "+getflags 0x" << hex << log.getFlags() << dec << endl;

  cout << "+operator<<(os, log), reverse" << endl;
  cout << log;

  LogBook::BookMarkList list;
  int whence = LogBook::OLDEST;

  printMarks(log, whence);
  size_t n = log.getBookMarks(list, whence);

  unsigned flagsFwd = LogBook::FlagONum | LogBook::FlagOMark;
  unsigned flagsRev = flagsFwd | LogBook::FlagORev;

  for(size_t i = 0; i < n; ++i)
  {
    for(int endpt = LogBook::OLDEST; endpt <= LogBook::NEWEST; ++endpt)
    {
      cout << endl << "++Test " << tcnt++ << endl;
      cout << "+printToMark(mark=\"" << list[i].m_strMark << "\", endpt="
        << endpt << "), forward" 
        << endl;
      log.setFlags(flagsFwd);
      log.printToMark(cout, list[i].m_strMark, endpt);

      cout << endl << "++Test " << tcnt++ << endl;
      cout << "+printToMark(mark=\"" << list[i].m_strMark << "\", endpt="
        << endpt << "), reverse" 
        << endl;
      log.setFlags(flagsRev);
      log.printToMark(cout, list[i].m_strMark, endpt);
    }
  }
}

static void testLogFundamentals(LogBook &log)
{
  char buf[64];

  // fundamental data
  const bool          vBool       = true;
  char                vChar       = 'z';
  byte_t              vByte       = 43;
  short               vShort      = -9;
  unsigned short      vUShort     = 9;
  int                 vInt        = 53;
  unsigned            vUInt       = 53;
  long                vLong       = -153;
  unsigned long       vULong      = 153;
  long long           vLongLong   = 0xffffffff;
  unsigned long long  vULongLong  = 0x7fffffff;
  float               vFloat      = -4.1414;
  double              vDouble     = -9e-12;
  long double         vLongDouble = -9e-12;
  char *              vS          = buf;
  void *              vVoid       = (void *)vS;
  strcpy(buf, "hello");
  const char *        vConstS     = "good bye";
  const string        vConstStr("constant pain");
  string              vStr("pleasure");

  // test tracking data
  char    tcnt = 'A';           // subtest count
  int     ecnt = 0, mcnt = 0;   // entry and mark counts
  string  li("(p) ");           // entry "list type" bullet
  string  what;                 // test whate

  printTestHdr("Test LogBook Fundamentals");

  // remember this marks
  string mark0("Test Fundamentals (top)");
  string mark1;
  string mark2;
  string mark;

  //log.orFlags(LogBook::FlagDebug);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test bookmark, constants, endl os manipulator.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "constants";

  log << bookmark(mark0) << li << what << ": "
    << 1 << " \"lonely number\" " << 0x13 << " endl" << endl
    << "Second line of entry." << eoe;

  cout << "+inserted bookmark " << mcnt++ << ": \"" << mark0 << "\"" 
    << " and entry " << ecnt++ << ": \"" << li << what << "...\"" << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test bool.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "bool";

  log << li << what << ": " << vBool << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test bookmark, ints, and hex manipulator.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "hex & dec int";

  mark1 = li + "only the lonely";
  log << bookmark(mark1.c_str()); // use char* for this test

  cout << "+inserted bookmark " << mcnt++ << ": \"" << mark1 << "\"" << endl; 

  log << li << what << ": " << hex << vInt << dec << " " << vInt << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // Validate bookmarks.
  printMarks(log, LogBook::OLDEST);
  printMarks(log, LogBook::NEWEST);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test getting entry at mark.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  cout << "+textAt(mark), where mark is good, bad, good " << endl;
  cout << "0: " << log.textAt(mark0) << endl;
  cout << "1: " << log.textAt("bad mark") << endl;
  cout << "2: " << log.textAt(mark1) << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test char and byte.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "char & byte";

  log << li << what << ": " << vChar << " " << vByte << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test shorts.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "short & ushort";

  log << li << what << ": " << vShort << " " << vUShort << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test anonymous mark and ints.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "int & uint";
  mark = "anon@ints";

  log << bookmark << li << what << ": " << vInt << " " << vUInt << eoe;

  cout << "+inserted bookmark " << mcnt++ << ": \"" << mark << "\"" 
    << " and entry " << ecnt++ << ": \"" << li << what << "...\"" << endl;

  // Validate bookmarks.
  printMarks(log, LogBook::OLDEST);

  // Set up output flags for these fundamental tests.
  log.orFlags(LogBook::FlagOMark);
  cout << "+getflags 0x" << hex << log.getFlags() << dec << endl;

  // Validate log book.
  cout << "+operator<<(os, log)" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test longs.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "long & ulong";

  log << li << what << ": " << vLong << " " << vULong << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test anonymous mark and longlongs.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "longlong & ulonglong";
  mark = "anon@longlong";

  log << bookmark << li << what << ": "
    << vLongLong << "(0x" << hex << vLongLong << dec << ") "
    << vULongLong << "(0x" << hex << vULongLong << dec << ") "
    << eoe;

  cout << "+inserted bookmark " << mcnt++ << ": \"" << mark << "\"" 
    << " and entry " << ecnt++ << ": \"" << li << what << "...\"" << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test float.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "float";

  log << li << what << ": " << vFloat << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // Validate bookmarks.
  printMarks(log, LogBook::OLDEST);

  // Validate log book.
  cout << "+operator<<(os, log)" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test inserting pending mark, then inserting double entry.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "double";
  mark2 = li + "DoubleFun";

  log << bookmark(mark2);

  cout << "+inserted pending bookmark " << mcnt++
    << ": \"" << mark2 << "\"" << endl;

  // Validate log book (no pending mark should be in book).
  cout << "+operator<<(os, log)" << endl;
  cout << log;

  log << li << what << ": " << vDouble << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // Validate log book.
  cout << "+operator<<(os, log)" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test longdouble.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "longdouble";

  log << li << what << ": " << vLongDouble << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // Validate log book.
  cout << "+operator<<(os, log)" << endl;
  cout << log;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test char* and const char*.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "char* & const char*";

  log << li << what << ": " << vS << " " << vConstS << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test void*.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "void*";

  log << li << what << ": " << vVoid << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Test string and const string
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "string & const string";

  log << li << what << ": " << vStr << " " << vConstStr << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Fini
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  cout << endl << "++Test " << tcnt++ << endl;
  li[1] = 'a' + ecnt;
  what  = "End of Fundamentals";

  log << li << what << eoe;

  cout << "+inserted entry " << ecnt++ << ": \"" << li << what << "...\""
    << endl;

  // Validate log book.
  cout << "+operator<<(os, log)" << endl;
  cout << log;
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
  string    strName("Star Trek Log");

  mainInit(argc, argv);

  LogBook log(strName, 10);

  testLogAttrs(log);

  testLogFundamentals(log);

  testLogAttrs(log);

  testLogOutput(log);

  testLogEdits(log);

  testLogAttrs(log);

  return APP_EC_OK;
}

/*!
 * \}
 */
