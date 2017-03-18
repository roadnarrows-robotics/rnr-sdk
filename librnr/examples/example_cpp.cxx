////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_cpp.cxx
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Nonsensical example that merely test compiling and linking a C++
 * applciation agains librnr C library.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <string>
#include <libgen.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "version.h"

using namespace std;

// 
// The command with option and argument values.
//
static char *Argv0;               ///< the command
static int   OptsTest = 1;        ///< test option value


//.............................................................................
// Log and Options Validation
//.............................................................................

/*!
 * \brief Program information.
 *
 * \note The dot initialization of C is far superior to C++ fixed order for
 * complex fixed structures.
 */
static OptsPgmInfo_T LogExamplePgmInfo = 
{
  // usage_args
  NULL,

  // synopsis
  "Example of C++ using librnr grab bag of goodies.",

  // long_desc 
  "The %P command demonstrates compiling-linking C++ with librnr facilities.",

  // diagnostic
  NULL
};

/*
 * \brief Command line options information.
 */
static OptsInfo_T LogExampleOptsInfo[] =
{
  // -t, --test <test>
  {
    "test",             // long_opt
    't',                // short_opt
    required_argument,  // has_arg
    true,               // has_default
    &OptsTest,          // opt_addr
    OptsCvtArgInt,      // fn_cvt
    OptsFmtInt,         // fn_fmt
    "<test>",           // arg_name
                        // opt_desc
    "Set the logging test(s) to conduct.\n"
    "%A is one of:\n"
    "  0   - No tests.\n"
    "  1   - Test #1.\n"
    "  2   - Test #2.\n"
    "  3   - All tests."
  },

  {NULL, }
};

/*!
 * \brief Log test number 1.
 *
 * \param p         Void pointer.
 * \param sFruit    Character pointer.
 * \param n         Integer.
 * \param half      Short.
 * \param f         Double.
 * \param hex       Integer in hex.
 * \param bAreRipe  Boolean.
 */
static void logtest_num1(void *p, const char *sFruit, int n, int half,
    double f, int hex, bool_t bAreRipe)
{
  // test log function call diagnostics 1
  LOGDIAG1CALL(_TPTR(p), _TSTR(sFruit), _TINT(n), _TSHORT(half), _TFLOAT(f),
      _THEX(hex), _TBOOL(bAreRipe));

  // test diagnostic 2 test
  if( bAreRipe )
  {
    LOGDIAG2("There are %d juicy %s in %s", n, sFruit, "Tuscany");
  }
  else
  {
    LOGDIAG2("There are %d green %s in %s", n, sFruit, "Utah");
  }
}

/*!
 * \brief Log test number 2.
 *
 * \param count   Count.
 */
static int logtest_num2(int count)
{
  FILE        *fp;
  const char  *sFileName = "PackingShedOnStrike.newsflash";

  // test log function call diagnostics 2
  LOGDIAG2CALL(_TINT(count));

  // test error logging
  while( count-- > 0 )
  {
    LOGERROR("Salmonella scare %d...", count);
  }

  // test system error logging
  if( (fp = fopen(sFileName, "r")) == NULL )
  {
    LOGSYSERROR("%s", sFileName);
  }
  else
  {
    LOGDIAG4("%s: what a pleasant surprise", sFileName);
    fclose(fp);
  }
}

/*!
 * \brief Run log test(s).
 *
 * \returns 1 on success, \h_gt 0 on failure.
 */
static int runlogtests()
{
  short   h = 10;
  int     i = 12;

  switch( OptsTest )
  {
    case 0:
      break;
    case 1:
      logtest_num1(&i, "apples", 53, h, 123.5, 0xdead, false);
      break;
    case 2:
      logtest_num2(3);
      break;
    case 3:
      logtest_num1(&h, "pears", 12, h, -1.5E-2, 0xf1, true);
      logtest_num2(3);
      break;
    default:
      LOGERROR("bug: %d: unexpected test", OptsTest);
      return 0;
  }
  return 1;
}


//.............................................................................
// Associative Validation
//.............................................................................

#include "rnr/assoc.h"

/*!
 * \brief Major and minor solar body ids (partial, of course).
 */
enum BodyId_T
{
  unknown,

  Sun,
  Mercury,
  Venus,
  Earth, Moon,
  Mars, Phobos, Deimos,
  Ceres,
  Jupiter, Io, Europa, Ganymede, Callisto,
  Saturn, Titan,
  Uranus, Titania,
  Neptune, Triton,
  Pluto, Charon,
  Eris,

  NumBodies
};

/*!
 * \brief Solar body name-id pairs (partial).
 */
Nvp_T SolarBodyNames[] =
{
  {"unknown",   unknown}, // default
  {"Sun",       Sun},
  {"Mercury",   Mercury},
  {"Venus",     Venus},
  {"Earth",     Earth},
  {"Moon",      Moon},
  {"Mars",      Mars},
  {"Phobos",    Phobos},
  {"Deimos",    Deimos},
  {"Jupiter",   Jupiter},
  {"Io",        Io},
  {"Europa",    Europa},
  {"Ganymede",  Ganymede},
  {"Callisto",  Callisto},
  {"Saturn",    Saturn},
  {"Titan",     Titan},
  {"Uranus",    Uranus},
  {"Titania",   Titania},
  {"Neptune",   Neptune},
  {"Triton",    Triton},
  {"Ceres",     Ceres},
  {"Pluto",     Pluto},
  {"Charon",    Charon}
};

/*!
 * \brief Test value to name association.
 *
 * \param oid Value.
 */
void test_assoc(BodyId_T oid)
{
  cout  << oid << " --> " 
        << NvpVal2Name(SolarBodyNames, arraysize(SolarBodyNames), (int)oid)
        << endl;
}

/*!
 * \brief Test name to value association.
 *
 * \param name  Name.
 */
void test_assoc(const char *name)
{
  cout  << name << " --> " 
        << NvpName2Val(SolarBodyNames, arraysize(SolarBodyNames), name)
        << endl;
}


//.............................................................................
// Char Validation
//.............................................................................

#include "rnr/char.h"

/*!
 * \brief Test character routine(s).
 */
void test_char()
{
  byte_t  buf[] = "\x04\xf3 apple\t\n\va day.";

  PrettyPrintBuf(stdout, buf, sizeof(buf));
  cout << endl;
}


//.............................................................................
// Config (INI) Validation
//.............................................................................

#include "rnr/config.h"

/*!
 * \brief Test INI configuration routines.
 */
void test_config(string inifile)
{
  Config_T *ini = NULL;
  string path[] = {".", "./examples", "/prj/pkg/librnr/examples"};
  string filepath;
  int i;

  for(i=0; (i<arraysize(path)) && (ini == NULL); ++i)
  {
    filepath = path[i] + "/" + inifile;
    ini = ConfigDbRead(filepath.c_str());
  }

  if( ini != NULL )
  {
    ConfigDbPrint(ini, stdout);
  }
}


//.............................................................................
// Doubly-Linked List Validation
//.............................................................................

#include "rnr/dliststr.h"

/*!
 * \brief Test doubly-linked list routines.
 */
void test_dlist()
{
  DListStr_T      *dlist;
  DListStrIter_T   iter;
  char            *s;

  dlist = DListStrNewDft();

  DListStrAppend(dlist, (char *)"elephant");
  DListStrAppend(dlist, (char *)"rhinocerus");
  DListStrPrepend(dlist, (char *)"hippopotomous");

  cout << "count = " << DListStrCount(dlist) << endl;

  for(s=DListStrIterDataFirst(dlist, &iter);
      s!=NULL;
      s=DListStrIterDataNext(&iter))
  {
    cout << s << endl;
  }
}


//.............................................................................
// Hash Validation
//.............................................................................

#include "rnr/hash.h"
#include "rnr/new.h"

/*!
 * \brief Colorado's 14,000+ foot mountains. 
 *
 * N.B. there are 53 of them, I'm stopping at the highest 16.
 */
static string Colorado14ers[][2] =
{
  {"mt_elbert",         "14,433"},
  {"mt_massive",        "14,421"},
  {"mt_harvard",        "14,420"},
  {"blanca_peak",       "14,345"},
  {"la_plata_peak",     "14,336"},
  {"uncompahgre_peak",  "14,309"},
  {"crestone_peak",     "14,294"},
  {"mt_lincoln",        "14,286"},
  {"grays_peak",        "14,270"},
  {"mt_antero",         "14,269"},
  {"torreys_peak",      "14,267"},
  {"castle_peak",       "14,265"},
  {"quandary_peak",     "14,265"},
  {"mt_evans",          "14,264"},
  {"longs_peak",        "14,255"},
  {"mt_wilson",         "14,246"}
};

/*!
 * \brief Delete node data - both key and value are dynamically allocated.
 *
 * \param key   Data key.
 * \param val   Data value.
 */
void del_node_data(void *key, void *val)
{
  delete((char *)key);
  delete((char *)val);
}

/*!
 * \brief Test hash and new routines.
 */
void test_hash()
{
  hash_t   *h = NULL;
  hscan_t   hs;
  hnode_t  *hn;

  h = hash_table_create(
        true,             // dynamic table sizing
        (hashcount_t)4,   // minimum size
        HASHCOUNT_T_MAX,  // maximum size
        NULL,             // use default comparator function
        NULL,             // use default hashing function
        del_node_data);   // hash node data deletion func

  hash_set_self_verify(true); // off by default

  char         *key, *val;

  for(size_t i=0; i<arraysize(Colorado14ers); ++i)
  {
    key = new_strdup(Colorado14ers[i][0].c_str());
    val = new_strdup(Colorado14ers[i][1].c_str());

    hash_insert(h, key, val);
  }

  hash_scan_begin(&hs, h);
  while( (hn = hash_scan_next(&hs)) != NULL )
  {
    cout << (char *)hnode_getkey(hn) << " -> " << (char *)hnode_get(hn) << endl;
  }
}


//.............................................................................
// Main Functions
//.............................................................................

/*!
 * \brief Main initialization.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \return 1 on success, exits on failure.
 */
static int init(int argc, char *argv[])
{
  FILE  *fp;
  int    i;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &LogExamplePgmInfo, LogExampleOptsInfo, true,
                 &argc, argv);

  // test log helper macro
  CHKEXPR_INT(OptsTest, ((OptsTest >= 0) || (OptsTest <= 3)), 0);

  // test complicated logging
  if( LOGABLE(LOG_LEVEL_DIAG1) )
  {
    LOGDIAG1("Post options processed non-option arguments:");
    fp = LOG_GET_LOGFP();
    for(i=0; i<argc; ++i)
    {
      fprintf(fp, "  argv[%d]=\"%s\"\n", i, argv[i]);
    }
  }
 
  return 1;
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  cout  << "Testing librnr Linkage" << endl
        << "----------------------" << endl;

  cout << "* Validate options and logging." << endl;
  if( !init(argc, argv) )
  {
    return 3;
  }
  runlogtests();

  cout << "* Validate associative mapping." << endl;
  test_assoc(Titania);
  test_assoc("Moon");
  cout << endl;

  cout << "* Validate char." << endl;
  test_char();
  cout << endl;

  cout << "* Validate config (ini)." << endl;
  test_config("example.ini");
  cout << endl;

  cout << "* Validate string dlist." << endl;
  test_dlist();
  cout << endl;

  cout << "* Validate hash and new." << endl;
  test_hash();
  cout << endl;

  return 0;
}
