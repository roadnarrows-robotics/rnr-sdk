////////////////////////////////////////////////////////////////////////////////
//
// Package:   appkit
//
// Program:   rnr_eg_cli   
//
// File:      rnr_eg_cli.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Command line interface example program.
 *
 * Demonstrates the usage of librnr_appkit's CommandLine, ReadLine, and
 * LogBook classes to build a command line interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016-2017  RoadNarrows
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

#include "rnr/appkit/CommandLine.h"

#include "version.h"

using namespace std;
using namespace rnr;
using namespace rnr::cmd;

/*!
 * \ingroup apps
 * \defgroup appkit_eg rnr_eg_cli
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
  "An RoadNarrows Robotics librnr_appkit CLI example program.",

  // long_desc = 
  "The %P command demonstrates the use of librnr_appkit's CommandLine, "
  "ReadLine, and LogBook classes to build a command line interface.",

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

#if 0 // SHOW_VERBATIM
/* Aardvark
        _.---._    /\\
     ./'       "--`\//
   ./              o \
  /./\  )______   \__ \
 ./  / /\ \   | \ \  \ \
    / /  \ \  | |\ \  \7
     "     "    "  "
*/
#endif // SHOW_VERBATIM

static const char *AsciiAardvark[] =
{
"        _.---._    /\\\\",
"     ./'       \"--`\\//",
"   ./              o \\",
"  /./\\  )______   \\__ \\",
" ./  / /\\ \\   | \\ \\  \\ \\",
"    / /  \\ \\  | |\\ \\  \\7",
"     \"     \"    \"  \"",
};

#if 0 // SHOW_VERBATIM
/* Mandrill
 ::        ,-,       ::
 ::       :o~o:      ::    	
  ::.___.-:{|}:--..-::;
   "=___   '_'   ,..-"
        :-,    ' :__
        \  \     /  :
         \  :. .:,__:
*/
#endif // SHOW_VERBATIM

static const char *AsciiMandrill[] =
{
" ::        ,-,       ::",
" ::       :o~o:      ::",
"  ::.___.-:{|}:--..-::;",
"   \"=___   '_'   ,..-\"",
"        :-,    ' :__",
"        \\  \\     /  :",
"         \\  :. .:,__:",
NULL
};

#if 0 // SHOW_VERBATIM
/* Numbat
 _             ____              ~^ 
 \`.|\..----''` / /`'--.,.,,     ~'',
 /  ' `      / /,/ / /  ,,. -    ~,,-
 )/' _/     \ / `-_,/  /   - ",,,' -
 | /' `"\_  ,_.-;_.-\_ ',   '',,,'`
 |/   _.-'_/   <__.'   ; /
    {_.-``-'         <__/
*/
#endif // SHOW_VERBATIM

static const char *AsciiNumbat[] =
{
" _             ____              ~^",
" \\`.|\\..----''` / /`'--.,.,,     ~''",
" /  ' `     / /,/ / /  ,,. -    ~,,",
" )/' _/     \\ / `-_,/  /   - \",,,' ",
" | /' `\"\\_  ,_.-;_.-\\_ ',   '',,,'",
" |/   _.-'_/   <__.'   ; ",
"    {_.-``-'         <__",
NULL
};

#if 0 // SHOW_VERBATIM
/* Zebra
         ,,_
        =/.-"
 ~._   =//
   _(||||\|_
  /        /

   \\/),
  ,'.' /,
 (_)- / /,
    /\_/ |__..--,  *
   (\ _ /\ \ \ / ).'
    \(-'./ / (_ //
     \\ \,'--'\_(
     )(_/  )_/ )_)
    (_,'  (_.'(_.'
*/
#endif // SHOW_VERBATIM

static const char *AsciiZebraSmall[] =
{
"         ,,_   ",
"        =/.-\" ",
" ~._   =//     ",
"   _(||||\\|_  ",
"  /        /   ",
NULL
};

static const char *AsciiZebra[] =
{
"   \\\\/),",
"  ,'.' /,",
" (_)- / /,",
"    /\\_/ |__..--,  *",
"   (\\ _ /\\ \\ \\ / ).'",
"    \\(-'./ / (_ //",
"     \\\\ \\,'--'\\_(",
"     )(_/  )_/ )_)",
"    (_,'  (_.'(_.'",
NULL
};

#if 0 // SHOW_VERBATIM
/* Ant
 '\__
  (o )     ___
  <>(_)(_)(___)
    < < > >
    ' ' ` `

  Grub
 \
  '-.__.-'
  /oo |--.--,--,--.
  \_.-'._i__i__i_.'
        """"""""" 
*/
#endif // SHOW_VERBATIM

static const char *AsciiAnt[] =
{
" '\\__",
"  (o )     ___",
"  <>(_)(_)(___)",
"    < < > >",
"    ' ' ` `",
NULL
};

static const char *AsciiGrub[] =
{
" \\",
"  '-.__.-'",
"  /oo |--.--,--,--.",
"  \\_.-'._i__i__i_.'",
"        \"\"\"\"\"\"\"\"\"",
NULL
};

#if 0 // SHOW_VERBATIM
/* Carrot
    \/'
   \/'
  _/'
 (,;)
 (,.)
 (,/
 |/
*/
#endif // SHOW_VERBATIM

static const char *AsciiCarrot[] =
{
"    \\/'",
"   \\/'",
"  _/'",
" (,;)",
" (,.)",
" (,/",
" |/",
NULL
};

static void showAsciiArt(const char *art[])
{
  for(int i = 0; art[i] != NULL; ++i)
  {
    cout << art[i] << endl;
  }
}

static void showAsciiArt(const char *art0[],
                         const char *art1[],
                         const char *art2[] = NULL,
                         const char *art3[] = NULL)
{
  int nPieces = 2;
  int linecnt[4];
  int maxlen[4];
  int maxlinecnt = 0;
  int len;
  int i, j;

  const char **pieces[4];

  pieces[0] = art0;
  pieces[1] = art1;

  if( art2 != NULL )
  {
    ++nPieces;
    pieces[2] = art2;
  }

  if( art3 != NULL )
  {
    ++nPieces;
    pieces[3] = art3;
  }

  // gather stats
  for(i = 0; i < nPieces; ++i)
  {
    linecnt[i]  = 0;
    maxlen[i]   = 0;

    for(j = 0; pieces[i][j] != NULL; ++j)
    {
      len = strlen(pieces[i][j]);
      if( len > maxlen[i] )
      {
        maxlen[i] = len;
      }
      linecnt[i] += 1;
    }

    if( linecnt[i] > maxlinecnt )
    {
      maxlinecnt = linecnt[i];
    }
  }

  char    ofill  = cout.fill();
  size_t  owidth = cout.width();

  cout.fill(' ');

  // show
  for(int line = 0; line < maxlinecnt; ++line)
  {
    for(i = 0; i < nPieces; ++i)
    {
      if( linecnt[i] >= maxlinecnt-line )
      {
        j = linecnt[i] - (maxlinecnt - line);
        // cout << "line " << line << ", art " << i << " j " << j << endl;
        len = strlen(pieces[i][j]);
        cout << pieces[i][j] << setw(maxlen[i]-len+1) << "";
      }
      else
      {
        cout << setw(maxlen[i]) << "";
      }
    }
    cout << endl;
  }

  cout.fill(ofill);
  cout.width(owidth);
}

enum Activity
{
  ActIdle,
  ActSleeping,
  ActFeeding,
  ActWalking
};

struct AnimalState
{
  string      m_sCommonName;
  string      m_sScientificName;
  string      m_strGivenName;
  bool        m_bIsAdopted;
  Activity    m_eActivity;
};

size_t  NumOfAnimals = 4;
string  NoName("noname");
bool    Quit = false;

AnimalState animal[] = 
{
  {"aardvark",  "orycteropus afer",       NoName,   false,  ActIdle},
  {"mandrill",  "mandrillus sphinx",      NoName,   false,  ActIdle},
  {"numbat",    "myrmecobius fasciatus",  NoName,   false,  ActIdle},
  {"zebra",     "equus quagga",           NoName,   false,  ActIdle}
};

// forward declaration
static int execHelp(const StringVec &argv);

static int execQuit(const StringVec &argv)
{
  Quit = true;

  return OK;
}

static int execAdopt(const StringVec &argv)
{
}

static int execSleep(const StringVec &argv)
{
}

static int execNameAnimal(const StringVec &argv)
{
}

static int execFeedAnimal(const StringVec &argv)
{
}

static int execWalkAnimal(const StringVec &argv)
{
}

    
static int execListAnimals(const StringVec &argv)
{
}

static int execGetPetsState(const StringVec &argv)
{
}

static int execDump(const StringVec &argv)
{
  //cout << cl;
}

CmdExec Commands[] =
{
  { "help",
    "help [{--usage | -u | --long | -l}] [<cmd:word>]",
    "Print this help.",
    NULL,
    execHelp
  },

  { "quit",
    "quit",
    "Quit test.",
    NULL,
    execQuit
  },

  { "adopt",
    "adopt <animal>",
    "Adopt an available animal",
    NULL,
    execAdopt
  },

  { "sleep",
    "sleep <animal> [<seconds:int>]",
    "Sleep for some <seconds> because, why not?",
    NULL,
    execSleep
  }, 

  { "set",
    "set <animal> <name:quoted-string>",
    "Name your cute adopted pet.",
    NULL,
    execNameAnimal
  },

  // feed, 3 forms
  { "feed",
    "feed {aardvark | mandrill | numbat} {ants | grubs}\n"
    "feed {mandrill | zebra} carrots\n"
    "feed <animal:re> vitamins",
    "Feed your pet some nutritious food.",
    NULL,
    execFeedAnimal
  },

  { "walk",
    "walk <animal> <minutes:float>",
    "Walk your adopted pet for some fun-filled <minutes>",
    NULL,
    execWalkAnimal
  },

  { "list",
    "list animals",
    "Get list of animal's adoption availability.",
    NULL,
    execListAnimals
  },

  { "get",
    "get <animal> {adoption | name | activity | species | selfie | all}",
    "Get an animal's current (partial) state.",
    NULL,
    execGetPetsState
  },

  { "dump",
    "dump",
    "Dump command data",
    NULL,
    execDump
  },

};

const size_t NumOfCmds = arraysize(Commands);
map<int, int>  IdToIndexMap;

// help [{usage | long}] [<cmd>]
static int execHelp(const StringVec &argv)
{
  bool    bLongHelp = true;
  string  strCmdName;
  int     cnt;
  size_t  i;

  i = 1;

  for(size_t i = 1; i < argv.size(); ++i)
  {
    if( i == 1 )
    {
      if( (argv[i] == "--usage") || (argv[i] == "-u") )
      {
        bLongHelp = false;
      }
      else if( (argv[i] == "--long") || (argv[i] == "-l") )
      {
        bLongHelp = true;
      }
      else
      {
        strCmdName = argv[i];
      }
    }
    else if( i == 2 )
    {
      strCmdName = argv[i];
    }
  }

  cnt = help(Commands, NumOfCmds, strCmdName, bLongHelp);

  if( cnt == 0 )
  {
    cout << "Error: No help for command \"" << strCmdName << "\"." << endl;
  }

  return OK;
}

static void initData()
{
}

static int loadCommands(CommandLine &cl)
{
  int   nId;
  int   rc;

  for(size_t i = 0; i < NumOfCmds; ++i)
  {
    nId = cl.addCommand(Commands[i].m_sSyntax);

    if( nId == CommandLine::NoUid )
    {
      cerr << "Error: Failed to add command " << Commands[i].m_sName
        << "." << endl;
      return RC_ERROR;
    }

    IdToIndexMap[nId] = i;
  }

  if( (rc = cl.compile()) != OK )
  {
    cerr << "Error: Compile failed." << endl;
  }

  //cl.backtrace(cerr, true);

  return rc;
}

static int run(CommandLine &cl)
{
  int       uid;
  StringVec argv;
  int       rc;

  while( !Quit )
  {
    rc = cl.readCommand(uid, argv);

    if( rc == OK )
    {
      cl.backtrace(cerr);
      if( uid != CommandLine::NoUid )
      {
        rc = Commands[IdToIndexMap[uid]].m_fnExec(argv);

        if( rc == OK )
        {
          cl.addToHistory(argv);
        }
      }
    }
    else
    {
      cout << "Error: Bad command. (backtrace):" << endl;
      cl.backtrace(cout);
    }
  }

  return OK;
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
  string    strClName("Adopt-An-Animal");

  mainInit(argc, argv);

  initData();

  CommandLine cl(strClName, "aaa> ");

  if( loadCommands(cl) != OK )
  {
    cerr << "Error: Failed to load commands." << endl;;
    return APP_EC_EXEC;
  }

  //cerr << cl << endl;

  showAsciiArt(AsciiAardvark);
  cout << strClName << " CommandLine/ReadLine/LogBook Example" << endl;
  cout << "  (enter 'help' for list of commands)" << endl << endl;

  if( run(cl) != OK )
  {
    cerr << "Error: Failed to run commands." << endl;
    return APP_EC_EXEC;
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
