////////////////////////////////////////////////////////////////////////////////
//
// Package:   NetMsgs
//
// Program:   nmExample
//
// File:      nmExample.c
//
/*! \file
 *
 * $LastChangedDate: 2010-05-03 13:49:15 -0600 (Mon, 03 May 2010) $
 * $Rev: 365 $
 *
 * \brief Example NetMsgs message packing/unpacking application.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/opts.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "nmExample.h"
#include "AstroMsgs.h"

#include "version.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Application Exit Codes
//
#define APP_EC_OK        0                ///< success exit code
#define APP_EC_USAGE     2                ///< usage error exit code
#define APP_EC_EXEC      4                ///< execution error exit code

//
// The command and command-line options.
//
static char    *Argv0;                    ///< this command basename
static bool_t   OptsTrace = false;        ///< do [not] trace packing/unpacking
// add others later

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  .synopsis =
    "Simple example packing and unpacking NetMsgs messages.",

  .long_desc = 
    "The %P application uses the python NetMsgs package generated C code "
    "from the AstroMsgs.xml NetMsgs XML specification and the libnetmsgs "
    "library to pack and unpack the messages."
};

/*!
 * \brief Command-line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --proxy=<addr[:port]>
  {
    .long_opt   = "trace",
    .short_opt  = 't',
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsTrace,
    .fn_cvt     = OptsCvtArgBool,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   = "Do [not] enable tracing of packing and unpacking "
                  "operations. If enabled, tracing is printed to stderr."
  },

  {NULL, }
};

//
// State
//


// ...........................................................................
// Initialization Functions
// ...........................................................................

/*!
 * \brief Initialize simple_khr2 command.
 *
 * Any command-line error immediately terminates application.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \return
 * Returns \ref APP_EC_OK on success, >0 exit code on failure.
 */
static int MainInit(int argc, char *argv[])
{
  // Name of this process.
  Argv0 = basename(argv[0]);

    // 
  // Parse input options.
  //
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  return APP_EC_OK;
}


// ...........................................................................
// Execution Functions
// ...........................................................................

typedef int (*Packer_T)(void *, byte_t [], size_t, bool_t);   ///< packer type
typedef int (*Unpacker_T)(byte_t [], size_t, void *, bool_t); ///< unpacker type

#define MSG_NS  "Astro"     ///< message namespace

/*!
 * \brief Concatenate two tokens.
 * \param a Token a.
 * \param b Token b.
 */
#define CAT(a, b)     a ## b

/*!
 * \brief Concatenate three tokens.
 * \param a Token a.
 * \param b Token b.
 * \param c Token c.
 */
#define CAT3(a, b, c) a ## b ## c

/*!
 * \brief Print side-by-side comparision of prepacked tx and post-unpacked rx
 * member.
 *
 * \note Implicit use of txmsg and rxmst external variables.
 *
 * \param mem   Structure member of txmsg and rxmsg.
 * \param fmt   Member print format.
 */
#define PRCMP(mem, fmt) \
  do \
  { \
    int k; \
    k = fprintf(stderr, "%s: ", #mem); \
    if( k > 38 ) \
    { \
      fprintf(stderr, "\n"); \
      k = 0; \
    } \
    fprintf(stderr, "%*stx " fmt "\n", 40-k, "", txmsg.mem); \
    fprintf(stderr, "%*srx " fmt "\n\n", 40, "", rxmsg.mem); \
  } while(0)

/*!
 * \brief Execute the pack, unpack identiy function.
 *
 * \param xmlname   XML name token
 * \param txmsg     Pre-packed, populated message structure.
 * \param rxmsg     Post-unacked, populated message structure.
 */
#define IDENTITY(xmlname, txmsg, rxmsg) \
  Identity(#xmlname, (Packer_T)CAT3(Astro, Pack,   xmlname), &txmsg, \
                     (Unpacker_T)CAT3(Astro, Unpack, xmlname), &rxmsg)


/*!
 * \brief Execute the pack, unpack identiy function.
 *
 * \param sXmlName    XML name string
 * \param fnPacker    Packing function.
 * \param pTxMsg      Pointer to pre-packed, populated message structure.
 * \param fnUnpacker  Unpacking function.
 * \param pRxMsg      Pointer to post-unacked, message structure.
 *
 * \return
 * Number of bytes unpacked (== packed) on success.\n
 * \h_lt 0 on error.
 */
int Identity(const char *sXmlName,  Packer_T fnPacker,      void *pTxMsg,
                                    Unpacker_T fnUnpacker,  void *pRxMsg)
{
  byte_t        buf[2048];
  int           n;

  if( (n = fnPacker(pTxMsg, buf, sizeof(buf), OptsTrace)) < 0 )
  {
    LOGERROR("%s(ecode=%d): %sPack%s()", nmStrError(n), n, MSG_NS, sXmlName);
  }


  else if( (n = fnUnpacker(buf, (size_t)n, pRxMsg, OptsTrace)) < 0 )
  {
    LOGERROR("%s(ecode=%d): %sUnpack%s()", nmStrError(n), n, MSG_NS, sXmlName);
  }

  return n;
}

/*! Test Function Type */
typedef int (*tstfunc_t)();

/*!
 * \brief Test AstroRspMsg message packing and unpacking.
 *
 * \return
 * Number of bytes unpacked (== packed) on success.\n
 * \h_lt 0 on error.
 */
int tstRspErr()
{
  AstroRspErr_T txmsg, rxmsg;
  int           n;

  txmsg.m_ECode   = 55;
  strcpy(txmsg.m_EMsg, "this is an error message");

  n = IDENTITY(RspErr, txmsg, rxmsg);

  if( n >= 0 )
  {
    fprintf(stderr, "\n--- AstroRspErr_T Results\n");
    PRCMP(m_ECode, "%u");
    PRCMP(m_EMsg, "'%s'");
  }

  return n;
}

/*!
 * \brief Test CmdZodiac message packing and unpacking.
 *
 * \return
 * Number of bytes unpacked (== packed) on success.\n
 * \h_lt 0 on error.
 */
int tstCmdZodiac()
{
  int     n = 0;
  size_t  i;

  AstroCmdZodiac_T  txmsg, rxmsg;

  strcpy(txmsg.m_Family, "test const");

  strcpy(txmsg.m_Constellation.m_Name, "Orion");

  txmsg.m_Constellation.m_Designation.u.m_buf[ALPHA] = ALPHA;
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[ALPHA].m_Name, "Betelgeuse");
  txmsg.m_Constellation.m_Star.u.m_buf[ALPHA].m_Type = 'M';
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[ALPHA].m_Color, "Red");
  txmsg.m_Constellation.m_Star.u.m_buf[ALPHA].m_TempK = 3500;
  txmsg.m_Constellation.m_Star.u.m_buf[ALPHA].m_MassSun = 19.0;

  txmsg.m_Constellation.m_Designation.u.m_buf[BETA] = BETA;
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[BETA].m_Name, "Rigel");
  txmsg.m_Constellation.m_Star.u.m_buf[BETA].m_Type = 'B';
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[BETA].m_Color, "Blue");
  txmsg.m_Constellation.m_Star.u.m_buf[BETA].m_TempK = 11000;
  txmsg.m_Constellation.m_Star.u.m_buf[BETA].m_MassSun = 17.0;

  txmsg.m_Constellation.m_Designation.u.m_buf[GAMMA] = GAMMA;
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[GAMMA].m_Name, "Bellatrix");
  txmsg.m_Constellation.m_Star.u.m_buf[GAMMA].m_Type = 'B';
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[GAMMA].m_Color, "Blue");
  txmsg.m_Constellation.m_Star.u.m_buf[GAMMA].m_TempK = 21500;
  txmsg.m_Constellation.m_Star.u.m_buf[GAMMA].m_MassSun = 8.5;

  txmsg.m_Constellation.m_Designation.u.m_buf[DELTA] = DELTA;
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[DELTA].m_Name, "Mintaka");
  txmsg.m_Constellation.m_Star.u.m_buf[DELTA].m_Type = 'B';
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[DELTA].m_Color, "Blue");
  txmsg.m_Constellation.m_Star.u.m_buf[DELTA].m_TempK = 30000;
  txmsg.m_Constellation.m_Star.u.m_buf[DELTA].m_MassSun = 20.0;

  txmsg.m_Constellation.m_Designation.u.m_buf[EPSILON] = EPSILON;
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[EPSILON].m_Name, "Alnilam");
  txmsg.m_Constellation.m_Star.u.m_buf[EPSILON].m_Type = 'B';
  strcpy(txmsg.m_Constellation.m_Star.u.m_buf[EPSILON].m_Color, "Blue");
  txmsg.m_Constellation.m_Star.u.m_buf[EPSILON].m_TempK = 25000;
  txmsg.m_Constellation.m_Star.u.m_buf[EPSILON].m_MassSun = 40.0;

  txmsg.m_Constellation.m_Designation.m_count = EPSILON+1;
  txmsg.m_Constellation.m_Star.m_count = EPSILON+1;

  n = IDENTITY(CmdZodiac, txmsg, rxmsg);

  if( n >= 0 )
  {
    fprintf(stderr, "\n--- AstroCmdZodiac_T Results\n");
    PRCMP(m_Family, "%s");
    PRCMP(m_Constellation.m_Name, "%s");
    PRCMP(m_Constellation.m_Designation.m_count, "%zu");
    PRCMP(m_Constellation.m_Star.m_count, "%zu");
    for(i=0; i<txmsg.m_Constellation.m_Star.m_count; ++i)
    {
      PRCMP(m_Constellation.m_Designation.u.m_buf[i], "%hhu");
      PRCMP(m_Constellation.m_Star.u.m_buf[i].m_Name, "%s");
      PRCMP(m_Constellation.m_Star.u.m_buf[i].m_Type, "'%c'");
      PRCMP(m_Constellation.m_Star.u.m_buf[i].m_Color, "%s");
      PRCMP(m_Constellation.m_Star.u.m_buf[i].m_TempK, "%u");
      PRCMP(m_Constellation.m_Star.u.m_buf[i].m_MassSun, "%f");
    }
  }

  return n;
}

/*!
 * \brief Test CmdUniverse message packing and unpacking.
 *
 * \return
 * Number of bytes unpacked (== packed) on success.\n
 * \h_lt 0 on error.
 */
int tstCmdUniverse()
{
  static char *p1 = "p-branes";
  static char *p2 = "bad captain kirk";

  AstroCmdUniverse_T txmsg =
  {
    .m_Truth = true,
    .m_Aura   = 'z',
    .m_Gluon  = 3,
    .m_Electron = -3,
    .m_Planet = 16433,
    .m_StellarObj = -31000,
    .m_StarSystem = 999999,
    .m_Galaxy = -11,
    .m_GalaxyGroup = 0xffffffff,
    .m_Filament = 85,
    .m_VisUnivLY = (float)-53.42,
    .m_FullUnivLY = 53e12,
    .m_Math.m_One = 1,
    .m_Math.m_Two = 2,
    .m_Math.m_DotDotDot = 101,
    .m_StarType = 'O',
    .m_StarInfo.m_Type = 'M',
    .m_StarInfo.m_TempK = 10000,
    .m_StarInfo.m_MassSun = (float)2.6,
  };
  AstroCmdUniverse_T  rxmsg;
  int                 n;

  txmsg.m_HiddenDim  = p1;
  txmsg.m_ParallelUniv  = p2;
  strcpy(txmsg.m_TheUnamed, "God");
  txmsg.m_Physics.m_count = 3;
  txmsg.m_Physics.u.m_buf[0] = 31415;
  txmsg.m_Physics.u.m_buf[1] = 214;
  txmsg.m_Physics.u.m_buf[2] = 317;
  txmsg.m_Physics.u.m_buf[3] = 555555;
  strcpy(txmsg.m_StarInfo.m_Name, "Rigel");
  strcpy(txmsg.m_StarInfo.m_Color, "Blue");

  n = IDENTITY(CmdUniverse, txmsg, rxmsg);

  if( n >= 0 )
  {
    fprintf(stderr, "\n--- AstroCmdUniverse_T Results\n");
    PRCMP(m_Truth, "%hhu");
    PRCMP(m_Aura, "'%c'");
    PRCMP(m_Gluon, "%hhu");
    PRCMP(m_Electron, "%hhd");
    PRCMP(m_Planet, "%hu");
    PRCMP(m_StellarObj, "%hd");
    PRCMP(m_StarSystem, "%u");
    PRCMP(m_Galaxy, "%d");
    PRCMP(m_GalaxyGroup, "%llu");
    PRCMP(m_Filament, "%lld");
    PRCMP(m_VisUnivLY, "%f");
    PRCMP(m_FullUnivLY, "%e");
    fprintf(stderr, "%*s%p->\"%s\"\n", 43, "", p1, p1);
    PRCMP(m_HiddenDim, "%p");
    fprintf(stderr, "%*s%p->\"%s\"\n", 43, "", p2, p2);
    PRCMP(m_ParallelUniv, "%p");
    PRCMP(m_TheUnamed, "%s");
    PRCMP(m_Math.m_One, "%hu");
    PRCMP(m_Math.m_Two, "%u");
    PRCMP(m_Math.m_DotDotDot, "%llu");
    PRCMP(m_Physics.m_count, "%zu");
    PRCMP(m_Physics.u.m_buf[0], "%u");
    PRCMP(m_Physics.u.m_buf[1], "%u");
    PRCMP(m_Physics.u.m_buf[2], "%u");
    PRCMP(m_StarType, "'%c'");
    PRCMP(m_StarInfo.m_Type, "'%c'");
    PRCMP(m_StarInfo.m_TempK, "%u");
    PRCMP(m_StarInfo.m_MassSun, "%f");
    PRCMP(m_StarInfo.m_Name, "%s");
    PRCMP(m_StarInfo.m_Color, "%s");
  }

  return n;
}

/*!
 * Table of Test Functions.
 */
tstfunc_t TstTbl[] =
{
  tstRspErr,
  tstCmdZodiac,
  tstCmdUniverse,
  NULL
};

/*!
 * \brief Simple NetMsg example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  tstfunc_t   fnTst;
  int         i;
  int         n = NM_OK;

  // Initialize command.
  MainInit(argc, argv);

  for(i=0; (fnTst=TstTbl[i])!=NULL; ++i)
  {
    if( (n = fnTst()) < 0 )
    {
      break;
    }
  }

  return n>=0? APP_EC_OK: APP_EC_EXEC;
}
