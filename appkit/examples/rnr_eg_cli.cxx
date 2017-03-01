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

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/appkit/Time.h"
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

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Shell Command-Line
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
  "A RoadNarrows Robotics librnr_appkit CLI example program.",

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


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ASCII Art and Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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

/*!
 * \brief ASCII ardvaark.
 */
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

/*!
 * \brief ASCII Mandrill.
 */
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

/*!
 * \brief ASCII numbat.
 */
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

/*!
 * \brief ASCII small zebra.
 */
static const char *AsciiZebraSmall[] =
{
"         ,,_   ",
"        =/.-\" ",
" ~._   =//     ",
"   _(||||\\|_  ",
"  /        /   ",
NULL
};

/*!
 * \brief ASCII zebra.
 */
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

/*!
 * \brief ASCII ant.
 */
static const char *AsciiAnt[] =
{
" '\\__",
"  (o )     ___",
"  <>(_)(_)(___)",
"    < < > >",
"    ' ' ` `",
NULL
};

/*!
 * \brief ASCII grub.
 */
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

/*!
 * \brief ASCII carrot.
 */
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

#if 0 // SHOW_VERBATIM
/* Vitamins
     _____
   _(_|_|_)_
  /  _      \
 '  (C) ( \  '
 | ___   \B\ |
 |(_A_)   \ )|
 '___________'
*/
#endif // SHOW_VERBATIM

/*!
 * \brief ASCII bottle of vitamins.
 */
static const char *AsciiVitamins[] =
{
"     _____",
"   _(_|_|_)_",
"  /  _      \\",
" '  (C) ( \\  '",
" | ___   \\B\\ |",
" |(_A_)   \\ )|",
" '___________'",
NULL
};

#if 0 // SHOW_VERBATIM
/* Walking

 |''''|">
 |\'''|\">
 /\'''/\">
 /|''/|">

 */
#endif // SHOW_VERBATIM

/*!
 * \brief ASCII walk sequence.
 */
static const char *AsciiWalking[][2] =
{
  {" |''''|\"> ",   NULL},
  {" |\\'''|\\\">", NULL},
  {" /\\'''/\\\">", NULL},
  {" /|''/|\"> ",   NULL}
};

/*!
 * \brief ASCII art output stream operator.
 *
 * \param os  Output stream.
 * \param art Art to insert.
 * 
 * \return Reference to output stream.
 */
static ostream &operator<<(ostream &os, const char *art[])
{
  for(int i = 0; art[i] != NULL; ++i)
  {
    os << art[i] << endl;
  }
  return os;
}

/*!
 * \brief Write a set of ASCII art to cout.
 *
 * The art written left to right, bottom justified.
 *
 * \param os    Output stream.
 * \param art0  Required art 0.
 * \param art1  Required art 1.
 * \param art2  Optional art 2.
 * \param art3  Optional art 3.
 */
static void showAsciiArt(ostream    &os,
                         const char *art0[],
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

  char    ofill  = os.fill();
  size_t  owidth = os.width();

  os.fill(' ');

  // show
  for(int line = 0; line < maxlinecnt; ++line)
  {
    for(i = 0; i < nPieces; ++i)
    {
      if( linecnt[i] >= maxlinecnt-line )
      {
        j = linecnt[i] - (maxlinecnt - line);
        // os << "line " << line << ", art " << i << " j " << j << endl;
        len = strlen(pieces[i][j]);
        os << pieces[i][j] << setw(maxlen[i]-len+1) << "";
      }
      else
      {
        os << setw(maxlen[i]) << "";
      }
    }
    os << endl;
  }

  os.fill(ofill);
  os.width(owidth);
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// The Animals
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Animal activities.
 */
enum Activity
{
  ActIdle,      ///< idle, doing nothing
  ActSleeping,  ///< sleeping
  ActEating,    ///< eating
  ActWalking    ///< walking
};

/*
 * \brief Animal Information type.
 */
struct AnimalInfo
{
  // fixed
  const char   *m_sCommonName;        ///< command name
  const char   *m_sScientificName;    ///< scientific name
  const char  **m_sSelfie;            ///< ASCII selfie

  // state
  string        m_strGivenName;       ///< adopted given name by owner
  bool          m_bIsAdopted;         ///< is [not] adopted
  Activity      m_eActivity;          ///< current activity
  double        m_fActStart;          ///< activity start time
  double        m_fActDuration;       ///< activitiy uninterrupted duration
};

const char *NoName = "No given name";   ///< 'no name' name

/*!
 * \brief The animals.
 */
AnimalInfo Animals[] = 
{
  { "aardvark",  "orycteropus afer",      AsciiAardvark,
    NoName, false, ActIdle, 0.0, 0.0
  },

  { "mandrill",  "mandrillus sphinx",     AsciiMandrill,
    NoName, false, ActIdle, 0.0, 0.0
  },

  { "numbat",    "myrmecobius fasciatus", AsciiNumbat,
    NoName, false, ActIdle, 0.0, 0.0
  },

  { "zebra",     "equus quagga",          AsciiZebra,
    NoName, false, ActIdle, 0.0, 0.0
  }
};

size_t NumOfAnimals = arraysize(Animals);   ///< number of animals

/*!
 * \brief Find the animal with the common name.
 *
 * \param strCommonName   Common name.
 *
 * \return
 * On succes, returns pointer to the animal's info.
 * On failure, NULL is returned.
 */
static AnimalInfo *findAnimal(const string &strCommonName)
{
  for(size_t i = 0; i < NumOfAnimals; ++i)
  {
    if( strCommonName == Animals[i].m_sCommonName )
    {
      return &Animals[i];
    }
  }
  return NULL;
}

Time ActivityTime;  ///< activity time


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// The CommandLine Interface.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

const string    CliName("Adopt-An-Animal");   ///< CLI name 
const string    CliPrompt("aaa> ");           ///< CLI prompt
CommandLine     Cli(CliName, CliPrompt);      ///< the CLI
bool            CliQuit = false;              ///< do [not] quit
map<int, int>   UidToIndexMap;                ///< uid to index map


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// CommandLine Commands Definitions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

// forward declarations
static int execHelp(int uid, int iform, const StringVec &argv);

/*!
 * \brief Execute 'quit' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execQuit(int uid, int iform, const StringVec &argv)
{
  CliQuit = true;

  return OK;
}

/*!
 * \brief Execute 'adopt' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execAdopt(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  else if( p->m_bIsAdopted )
  {
    cout << "Error: The " << argv[n] << " is already adopted." << endl;
    return RC_ERROR;
  }

  else
  {
    p->m_bIsAdopted = true;
    return OK;
  }
}

/*!
 * \brief Execute 'sleep' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execSleep(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  // optional defaults
  long    secs = 10;

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  else if( !p->m_bIsAdopted )
  {
    cout << "The " << p->m_sCommonName << " is not yours to rock to sleep."
      << endl;
    return OK;
  }

  ++n;

  //
  // Optional arguments
  //

  if( n < argc )
  {
    // convert to integer
    secs = Cli.convert(uid, iform, n, argv[n]).i();
  }

  p->m_eActivity    = ActSleeping;
  p->m_fActStart    = ActivityTime.now();
  p->m_fActDuration = (double)secs;

  cout << "The " << p->m_sCommonName << " is sleeping for " << secs
    << " seconds." << endl;

  return OK;
}

/*!
 * \brief Execute 'name' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execNameAnimal(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'" << endl;
    return RC_ERROR;
  }

  else if( !p->m_bIsAdopted )
  {
    cout << "You have not adopted the " << argv[n] << endl;
    return OK;
  }

  ++n;  // next argument

  p->m_strGivenName = argv[n];

  return OK;
}

/*!
 * \brief Execute 'namaste' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execNamaste(int uid, int iform, const StringVec &argv)
{
  cout << "Aye. Baaaa to the divine Ewe." << endl;

  return OK;
}

/*!
 * \brief Execute 'feed' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execFeedAnimal(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  else if( !p->m_bIsAdopted )
  {
    cout << "The " << p->m_sCommonName << " is not yours to feed." << endl;
    return OK;
  }

  ++n;  // next argument

  const char **food = NULL;
  double      fSecs;

  if( argv[n] == "ants" )
  {
    food  = AsciiAnt;
    fSecs = 30.0;
  }
  else if( argv[n] == "carrots" )
  {
    food  = AsciiCarrot;
    fSecs = 10.0;
  }
  else if( argv[n] == "grubs" )
  {
    food  = AsciiGrub;
    fSecs = 20.0;
  }
  else if( argv[n] == "vitamins" )
  {
    food  = AsciiVitamins;
    fSecs = 5.0;
  }
  else
  {
    cout << "Error: Unknown food '" << argv[n] << "'" << endl;
    return RC_ERROR;
  }

  if( (p->m_sCommonName == "numbat") || (p->m_sCommonName == "zebra") )
  {
    showAsciiArt(cout, food, p->m_sSelfie);
  }
  else
  {
    showAsciiArt(cout, p->m_sSelfie, food);
  }
  cout << "Delicious! Give me " << fSecs << " seconds to enjoy." << endl;

  p->m_eActivity    = ActEating;
  p->m_fActStart    = ActivityTime.now();
  p->m_fActDuration = fSecs;

  return OK;
}

/*!
 * \brief Execute 'walk' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execWalkAnimal(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  else if( !p->m_bIsAdopted )
  {
    cout << "The " << p->m_sCommonName << " is not yours to walk." << endl;
    return OK;
  }

  ++n;  // next argument

  double  minutes;

  // convert to double
  minutes = Cli.convert(uid, iform, n, argv[n]).f();

  p->m_eActivity    = ActWalking;
  p->m_fActStart    = ActivityTime.now();
  p->m_fActDuration = (double)minutes * 60.0;

  cout << "You are walking the " << p->m_sCommonName << " for " << minutes
    << " minutes." << endl;

  //
  // Fun stuff
  //
  const char *gait;

  //
  // I used printf here, instead of cout insertion. Lot easier. But then, I've
  // always preferred stdio over fstream.
  //
  for(int i = 0; i < 40; ++i)
  {
    gait = AsciiWalking[i%4][0];
    printf("%*s%s\r", i, " ", gait);
    fflush(stdout);
    usleep(50000);
  }

  usleep(250000);
  printf("%*s\r", 80, " ");
  fflush(stdout);

  printf("\n");

  return OK;
}

/*!
 * \brief Execute 'list' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execListAnimals(int uid, int iform, const StringVec &argv)
{
  for(size_t i = 0; i < NumOfAnimals; ++i)
  {
    cout << Animals[i].m_sCommonName << " ";
  }

  cout << endl;

  return OK;
}

/*!
 * \brief Execute 'get' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execGetPetsState(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  // optional arguments defaults
  long eState = 0;

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  string  strAdoptState;
  string  strActivity;

  strAdoptState = p->m_bIsAdopted? "Adopted": "Not adopted";

  switch( p->m_eActivity )
  {
    case ActIdle:
      strActivity = "Just chill'n dude.";
      break;
    case ActSleeping:
      strActivity = "Sleeping, don't wake me.";
      break;
    case ActEating:
      strActivity = "Num, num, num.";
      break;
    case ActWalking:
      strActivity = "And I'm walking, and walking, walking...";
      break;
    default:
      strActivity = "I dunno what I'm doing.";
      break;
  }

  ++n;  // next argument

  //
  // Optional arguments.
  //

  // 
  // Argument 2
  //
  // State: all | adoption | given | activity | species | selfie
  //
  if( n < argc )
  {
    // convert to enum index
    eState = Cli.convert(uid, iform, n, argv[n]).e();
  }

  switch( eState )
  {
    case 1: // adoption
      cout << strAdoptState << endl;
      break;
    case 2: // given
      cout << p->m_strGivenName << endl;
      break;
    case 3: // activity
      cout << strActivity << endl;
      break;
    case 4: // species
      cout << p->m_sCommonName << " (" << p->m_sScientificName << ")" << endl;
      break;
    case 5: // selfie
      cout << p->m_sSelfie;
      break;
    case 0: // all
      cout << p->m_sSelfie;
      cout << p->m_sCommonName << " (" << p->m_sScientificName << ")" << endl;
      cout << strAdoptState << endl;
      cout << p->m_strGivenName << endl;
      cout << strActivity << endl;
      break;
    default:
      cout << "oops" << endl;
      break;
  }

  return OK;
}

/*!
 * \brief Execute 'reward' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execReward(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required Arguments
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  else if( !p->m_bIsAdopted )
  {
    cout << "The " << p->m_sCommonName << " is not yours to reward." << endl;
    return OK;
  }

  ++n;  // next argument

  bool isGood;

  // convert to boolean
  isGood = Cli.convert(uid, iform, n, argv[n]).b();

  cout << "Your " << p->m_sCommonName << " ";

  if( p->m_sCommonName == "aardvark" )
  {
    if( isGood )
    {
      cout << "is a good girl." << endl;
      cout << "She ate all of those pesky ants pilfering food in the kitchen."
        << endl;
    }
    else
    {
      cout << "was a bad girl." << endl;
      cout << "She dug under the foundation again." << endl;
    }
  }

  else if( p->m_sCommonName == "mandrill" )
  {
    if( isGood )
    {
      cout << "is a good boy." << endl;
      cout << "You are now well groomed and free of fleas." << endl;
    }
    else
    {
      cout << "is a mischieveus boy." << endl;
      cout << "Why does he love reality tv so much?" << endl;
    }
  }

  else if( p->m_sCommonName == "numbat" )
  {
    if( isGood )
    {
      cout << "is a good lad." << endl;
      cout << "Those termites won't be invading any time soon." << endl;
    }
    else
    {
      cout << "is a snarky lad." << endl;
      cout << "He needs to quit putting his long, narrow tongue where it "
        << "doesn't belong!" << endl;
    }
  }

  else if( p->m_sCommonName == "zebra" )
  {
    if( isGood )
    {
      cout << "is a good lass." << endl;
      cout << "You rode her all the way to town this time without being "
        << "thrown." << endl;
    }
    else
    {
      cout << "is a very naughty lass." << endl;
      cout << "She ate all of the apples ripening on the tree." << endl;
    }
  }

  else
  {
    cout << " is " << (isGood? "good": "not good") << "." << endl;
  }

  return OK;
}

/*!
 * \brief Execute 'save' command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execSave(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 1;  // skip argv0

  //
  // Required arguments.
  //

  AnimalInfo *p = findAnimal(argv[n]);

  if( p == NULL )
  {
    cout << "Error: Unknown animal '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  ++n;  // next argument

  bool bSave = false;

  if( access(argv[n].c_str(), F_OK) == 0 )
  {
    string line;
    string ans;

    cout << "Overwrite file '" << argv[n] << "'? [ny] ";

    if( std::getline(cin, line) )
    {
      istringstream iss(line);
      if( (iss >> ans) && (ans == "y") )
      {
        bSave = true;
      }
    }
  }
  else
  {
    bSave = true;
  }

  if( bSave )
  {
    ofstream selfie;

    selfie.open(argv[n].c_str());

    if( selfie.is_open() )
    {
      selfie << p->m_sSelfie;
      selfie.close();
      cout << "A " << p->m_sCommonName << " selfie writen to file '"
        << argv[n] << "'." << endl;
    }
    else
    {
      cout << "Error: Failed to open file '" << argv[n] << "'." << endl;
    }
  }

  return OK;
}

/*!
 * \brief Execute command-line interface debug command.
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execCliTest(int uid, int iform, const StringVec &argv)
{
  size_t  argc = argv.size();
  size_t  n;  

  if( argc < Cli.numOfRequiredArgs(uid, iform) )
  {
    cout << "Error: Command '" << Cli.at(uid).getName()
      << "' has missing arguments." << endl;
    return RC_ERROR;
  }

  n = 0;  // argv0 is the test function

  if( argv[n] == "dump" )
  {
    cout << Cli << endl;
  }
  else if( argv[n] == "bt" )
  {
    Cli.backtrace(cout, true);
  }
  else if( argv[n] == "print" )
  {
    ++n;  // next argument
    if( n < argc )
    {
      if( argv[n] == "name" )
      {
        cout << Cli.getName() << endl;
      }
      else if( argv[n] == "prompt" )
      {
        cout << Cli.getPrompt() << endl;
      }
      else if( argv[n] == "numcmds" )
      {
        cout << Cli.numOfCmds() << endl;
      }
      else if( argv[n] == "errstr" )
      {
        cout << Cli.getErrorStr() << endl;
      }
      else
      {
        cout << "Error: Unknown <attr> '" << argv[n] << "'." << endl;
        return RC_ERROR;
      }
    }
    else
    {
      cout << "Error: Nothing to get." << endl;
      return RC_ERROR;
    }
  }
  else if( argv[n] == "push" )
  {
    ++n;  // next argument
    if( n < argc )
    {
      Cli.pushPrompt(argv[n]);
    }
    else
    {
      cout << "Error: Nothing to push." << endl;
      return RC_ERROR;
    }
  }
  else if( argv[n] == "pop" )
  {
    Cli.popPrompt();
  }
  else
  {
    cout << "Error: Do not know how to execute '" << argv[n] << "'." << endl;
    return RC_ERROR;
  }

  return OK;
}

/*!
 * \brief Command description and exectuion structure.
 */
struct CmdExec
{
  CmdDesc       m_desc;     ///< command description and syntax specification
  CmdExec2Func  m_fnExec;   ///< command execution function
};

/*!
 * \brief The command descriptions.
 */
CmdExec Commands[] =
{
  { { "help",
      "help [{--usage | -u}] [<cmd:word>]",
      "Print this help.",
      "Print command help. If the --usage option is specified, then only the "
      "command(s) usages are printed. If the <cmd> is specified, then only "
      "help for that command is printed. Otherwise all command help is printed."
    },
    execHelp
  },

  { { "quit",
      "quit",
      "Quit example application.",
      NULL
    },
    execQuit
  },

  { { "adopt",
      "adopt <animal>",
      "Adopt an available animal",
      "Adopt a cuddly animal. In order to name, feed, walk, sleep, or reward "
      "an animal, "
      "you must adopt first. See the 'list' command for list of animals and "
      "the 'get' command for an animal's current state of being.\n\n"
      "Demonstrates a variable with default 'word' type."
    },
    execAdopt
  },

  { { "sleep",
      "sleep <animal> [<seconds:int>]",
      "Sleep for some <seconds> because, why not?",
      "Put your adopted animal to sleep (no, not kill you brute). "
      "The <animal> goes to sleep for the given seconds.\n"
      "  Default: 10 seconds\n\n"
      "Demonstrates an optional integer variable."
    },
    execSleep
  }, 

  { { "name",
      "name <animal> <name:multiword>",
      "Name your cute adopted pet.",
      "Name your adopted animal. Double qoute '\"' the name if it contains "
      "any whitespace.\n\n"
      "Demonstrates a multword variable type."
    },
    execNameAnimal
  },

  { { "namaste",
      "namaste",
      "Bow to the divine ewe.",
      "Say namaste.\n\n"
      "Demonstrates commands with similar names such as 'name' and 'namaste'."
    },
    execNamaste
  },

  // feed, 3 forms
  { { "feed",
      "feed {aardvark | mandrill | numbat} {ants | grubs}\n"
      "feed {mandrill | zebra} carrots\n"
      "feed <animal:re> vitamins",
      "Feed your pet some nutritious food.",
      "Feed your adopted animal. Of course, the food has to match the "
      "animal's diet.\n\n"
      "Demonstrates multi-form commands and enumerated literal arguments. "
      "Also demonstrate a regular expresson variable type (future)."
    },
    execFeedAnimal
  },

  { { "walk",
      "walk <animal> <minutes:fpn>",
      "Walk your adopted pet for some fun-filled <minutes>",
      "Walk your adopted animal for the specified minutes.\n\n"
      "Demonstrates a floating-point number variable."
    },
    execWalkAnimal
  },

  { { "list",
      "list",
      "Get list of animals.",
      NULL
    },
    execListAnimals
  },

  { { "get",
      "get <animal> [{all | adoption | given | activity | species | selfie}]",
      "Get an animal's current (partial) state.",
      "Get the animal's state of being.\n"
      "  Default: all\n\n"
      "Demonstrates optional enumerated literal arguments."
    },
    execGetPetsState
  },

  { { "reward",
      "reward <animal> <good:bool>",
      "Do [not] reward your pet for its behaviour.",
      "Reward your adopted animal or not.\n\n"
      "Demonstrates a boolean variable."
    },
    execReward
  },

  { { "save",
      "save <animal:word> <fname:file>",
      "Save ascii animal to file.",
      "Save an animal's ASCII redention to a file. The art is fantastic btw."
      "If the file exist, you will be prompted if you wish to overwrite.\n\n"
      "Demonstrates a file variable type."
    },
    execSave
  },

  { { "clitest",
      "<clitest:word> [<modifier:multiword>]",
      "Test CommandLine features.",
      "The 'clitest' command validates command wild carding and provides test "
      "functions to validate the CommandLine and underlining classes.\n\n"
      "Supported Test Functions:\n"
      "dump           - Dump definitions.\n"
      "bt             - Backtrace log.\n"
      "print <attr>   - Print attribute, where <attr> is one of:\n"
      "                   name prompt numcmds errstr.\n"
      "push <prompt>  - Push new <prompt> string.\n"
      "pop            - Pop current prompt string.\n\n"
      "Demonstrates command name wild carding."
    },
    execCliTest
  }
};

/*!
 * \brief Number of commands.
 */
const size_t NumOfCmds = arraysize(Commands);

/*!
 * \brief Find command by name.
 *
 * \param strName Command to find.
 *
 * \return On succes, returns index to Commands[], otherwise -1 is returned.
 */
int findCommand(const std::string &strName)
{
  for(int i = 0; i < NumOfCmds; ++i)
  {
    if( strName == Commands[i].m_desc.m_sName )
    {
      return i;
    }
  }
  return -1;
}

/*!
 * \brief Execute 'help' command.
 *
 * help [{usage | long}] [<cmd>]
 *
 * \param argv  Command line arguments.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int execHelp(int uid, int iform, const StringVec &argv)
{
  bool    bLongHelp = true;
  string  strCmdName;
  int     iCmd;
  size_t  i;

  //
  // Process Input arguments.
  //
  for(i = 1; i < argv.size(); ++i)
  {
    if( i == 1 )
    {
      if( (argv[i] == "--usage") || (argv[i] == "-u") )
      {
        bLongHelp = false;
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

  //
  // Print help for all commands.
  //
  if( strCmdName.empty() )
  {
    for(i = 0; i < NumOfCmds; ++i)
    {
      help(cout, Commands[i].m_desc, bLongHelp);
      if( bLongHelp )
      {
        cout << "    ---" << endl << endl;
      }
      else
      {
        cout << endl;
      }
    }
    cout << "  " << NumOfCmds << " commands" << endl;
    return OK;
  }

  //
  // Print help for a solitary command.
  //
  else if( (iCmd = findCommand(strCmdName)) >= 0 )
  {
    help(cout, Commands[iCmd].m_desc, bLongHelp);
    return OK;
  }

  //
  // No help.
  //
  else
  {
    cout << "Error: No help for command '" << strCmdName << "'." << endl;
    return RC_ERROR;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Main Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
 * \brief Update all animals' state data.
 */
static void updateAnimals()
{
  double  fNow;

  fNow = ActivityTime.now();

  for(size_t i = 0; i < NumOfAnimals; ++i)
  {
    if( Animals[i].m_eActivity == ActIdle )
    {
      continue;
    }

    if( (fNow - Animals[i].m_fActStart) >= Animals[i].m_fActDuration )
    {
      Animals[i].m_eActivity    = ActIdle;
      Animals[i].m_fActStart    = 0.0;
      Animals[i].m_fActDuration = 0.0;
    }
  }
}

/*!
 * \brief Load commands into command line.
 *
 * Loading involves adding all commands and then compiling.
 *
 * \param cli Command line interface.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int loadCommands(CommandLine &cli)
{
  int   nUid;
  int   rc;

  for(size_t i = 0; i < NumOfCmds; ++i)
  {
    nUid = cli.addCommand(Commands[i].m_desc.m_sSyntax);

    if( nUid == CommandLine::NoUid )
    {
      cerr << "Error: Failed to add command '" << Commands[i].m_desc.m_sName
        << "'." << endl;
      return RC_ERROR;
    }

    UidToIndexMap[nUid] = i;
  }

  if( (rc = cli.compile()) != OK )
  {
    cerr << "Error: Compile failed." << endl;
  }

  cli.backtrace(cerr, true);

  return rc;
}

/*!
 * \brief Command line interface main loop.
 *
 * \param cli Command line interface.
 *
 * \return OK(0) on success, negative value on failure.
 */
static int run(CommandLine &cli)
{
  int       uid;      // command unique id
  int       iform;    // command form index
  StringVec argv;     // vector of string input arguments
  int       rc;       // return code

  while( !CliQuit )
  {
    rc = cli.readCommand(uid, iform, argv);

    updateAnimals();

    if( rc == OK )
    {
      cli.backtrace(cerr);

      if( uid != CommandLine::NoUid )
      {
        rc = Commands[UidToIndexMap[uid]].m_fnExec(uid, iform, argv);

        if( rc == OK )
        {
          cli.addToHistory(argv);
        }
      }
    }
    else
    {
      cout << "Error: Bad command. (backtrace):" << endl;
      cli.backtrace(cout);
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
  mainInit(argc, argv);

  if( loadCommands(Cli) != OK )
  {
    cerr << "Error: Failed to load commands." << endl;;
    return APP_EC_EXEC;
  }

  //cerr << Cli << endl;

  cout << AsciiAardvark;
  cout << CliName << " CommandLine/ReadLine/LogBook Example" << endl;
  cout << "  (enter 'help' for list of commands)" << endl << endl;

  if( run(Cli) != OK )
  {
    cerr << "Error: Failed to run commands." << endl;
    return APP_EC_EXEC;
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
