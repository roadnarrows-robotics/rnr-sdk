////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      opts.c
//
/*! \file
 *
 * $LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
 * $Rev: 3823 $
 *
 * \brief Standard command-line options built-in options parsing and
 * validation definitions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
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
#include <errno.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdarg.h>
#include <libgen.h>
#include <limits.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/pkg.h"
#include "rnr/new.h"
#include "rnr/assoc.h"
#include "rnr/log.h"

#include "rnr/opts.h"

// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------

#ifdef LOG
/*!
 * Name-Value Pair table of names to log levels.
 */
static Nvp_T OptsLogLevelTbl[] =
{
  {"nosym", -1},
  {"off",   LOG_LEVEL_OFF},
  {"error", LOG_LEVEL_ERROR},
  {"diag1", LOG_LEVEL_DIAG1},
  {"diag2", LOG_LEVEL_DIAG2},
  {"diag3", LOG_LEVEL_DIAG3},
  {"diag4", LOG_LEVEL_DIAG4},
  {"diag5", LOG_LEVEL_DIAG5}
};
#endif // LOG

//
// RN logging modifier options.
//
#define OPTS_LONGOPT_LOG        "log"           ///< log long option name
#define OPTS_LONGOPT_LOGFILE    "logfile"       ///< log file long option name
#define OPTS_LONGOPT_LOGNOCOLOR "log-no-color"  ///< log with no color option
#define OPTS_LONGOPT_LOGNOTIME  "log-no-timestamp" ///< log no timestamps option

/*!
 * Internal options control structure
 */
typedef struct
{
  struct option *m_pLongOpts;     ///< long options structure
  char          *m_sShortOpts;    ///< short options string
} OptsCtl_T;

/*!
 * Built-in logging option working variables.
 */
static int    OptsLogLevel  = LOG_LEVEL_DFT;     ///< working log level value
static char  *OptsLogFile   = LOG_FILENAME_DFT;  ///< working log file value
static bool_t OptsLogNoColor      = false;  ///< working log no color value
static bool_t OptsLogNoTimestamp  = false;  ///< working log no timestamp value

/*!
 * \brief Built-In Options
 */
static OptsInfo_T OptsBuiltIn[] =
{
#ifdef LOG
  // -l, --log <level>
  {
    .long_opt   = OPTS_LONGOPT_LOG,
    .short_opt  = 'l',
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsLogLevel,
    .fn_fmt     = OptsFmtLogLevel,
    .arg_name   = "<level>",
    .opt_desc   =
      "Set logging threshold level. All logging events with priority "
      "<= %A will be "
      "logged. All others will be ignored. Error events are always logged.\n"
      "%A is one of:\n"
      "  'off' or 0   - Disable all logging.\n"
      "  'error' or 1 - Enable error and warning logging.\n"
      "  'diag1' or 2 - Enable diagnostics 1 logging.\n"
      "  'diag2' or 3 - Enable diagnostics 2 logging.\n"
      "  'diag3' or 4 - Enable diagnostics 3 logging.\n"
      "  'diag4' or 5 - Enable diagnostics 4 logging.\n"
      "  'diag5' or 6 - Enable diagnostics 5 logging.\n"
      "  >6           - Enable user-defined logging.",
    .pvt_retval = OPTS_RVAL_LOG
  },

  // --logfile <file>
  {
    .long_opt   = OPTS_LONGOPT_LOGFILE,
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsLogFile,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<file>",
    .opt_desc   =
      "Set log file %A.\n"
      "Special %A names:\n"
      "  '" LOG_FILENAME_STDERR "'  - log to standard error.\n"
      "  '" LOG_FILENAME_STDOUT "'  - log to standard output.",
    .pvt_retval = OPTS_RVAL_LOGFILE
  },

  // --log-no-color
  {
    .long_opt   = OPTS_LONGOPT_LOGNOCOLOR,
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsLogNoColor,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   =
      "Disable logging with compiled ANSI color strings.",
    .pvt_retval = OPTS_RVAL_LOGNOCOLOR
  },

  // --log-no-timestamp
  {
    .long_opt   = OPTS_LONGOPT_LOGNOTIME,
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsLogNoTimestamp,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   =
      "Disable logging with timestamps.",
    .pvt_retval = OPTS_RVAL_LOGNOTIME
  },
#endif // LOG

  // --help option
  {
    .long_opt   = "help",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = no_argument,
    .opt_desc   = "Display this help and exit.",
    .pvt_retval = OPTS_RVAL_HELP
  },

  // --version option
  {
    .long_opt   = "version",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = no_argument,
    .opt_desc   = "Output version information and exit.",
    .pvt_retval = OPTS_RVAL_VERSION
  },

  // null entry
  {NULL, }
};


/*!
 * \brief Reset argc,argv to start of non-option command arguments.
 * \param optind  Last option index. 
 * \param argc    Argument count.
 * \param argv    Argument vector.
 */
#define OPTS_RESET_ARGCV(optind, argc, argv) \
  { argc -= optind; argv += optind; }

#define MAX_FMT_BUF   ((size_t)1024) ///< Format buffer size

/*!
 * PrettyPrint formats
 */
#define HELP_OPT_OVER   30                ///< opt help desc 1st line indent
#define HELP_OPT_OVER_2 (HELP_OPT_OVER+2) ///< opt help desc 2nd line+ indent
#define HELP_MAX_COL    79                ///< max help column

/*!
 * \brief Indents (spaces) over, returning number of added indentation
 * \param over  Number of spaces.
 */
inline int HelpIndent(int over) { return printf("%*s", over, ""); }

/*!
 * \brief Puts help newline, returning new output column position.
 * \param over  Number of spaces.
 */
inline int HelpPutNL(int over)  { printf("\n"); return HelpIndent(over); }

/*!
 * \brief Puts help buffer, returning new output column position
 *
 * \param col             Current column position.
 * \param colstart        Column to start.
 * \param buf             Buffer to output.
 * \param pbuflen [out]   Buffer length.
 *
 * \return New column position.
 */
static int HelpPutBuf(int col, int colstart, char buf[], size_t *pbuflen)
{
  if( *pbuflen == 0 )
  {
    return col;
  }
  if( (col > colstart) && (col >= (HELP_MAX_COL - *pbuflen)) )
  {
    col = HelpPutNL(colstart);
  }
  buf[*pbuflen] = 0; 
  col += printf("%s", buf); 
  *pbuflen = 0;
  return col;
}

/*!
 * \brief Puts help string, returning new output column position
 *
 * \param col             Current column position.
 * \param colstart        Column to start.
 * \param s               Help string.
 *
 * \return New column position.
 */
static int HelpPutStr(int col, int colstart, const char *s)
{
  int n;

  if( (s == NULL) || (*s == 0) )
  {
    return col;
  }
  n = (int)strlen(s);
  if( (col > colstart) && (col >= (HELP_MAX_COL - n)) )
  {
    col = HelpPutNL(colstart);
  }
  col += printf("%s", s); 
  return col;
}

/*!
 * \brief Puts help character, returning new output column position
 *
 * \param col             Current column position.
 * \param colstart        Column to start.
 * \param c               Help character.
 *
 * \return New column position.
 */
static int HelpPutC(int col, int colstart, int c)
{
  if( col > HELP_MAX_COL )
  {
    col = HelpPutNL(colstart);
  }
  col += printf("%c", c);
  return col;
}

/*!
 * \brief Gets help character into local buffer, returning new/same output 
 * column position
 *
 * \param col             Current column position.
 * \param colstart        Column to start.
 * \param buf [out]       Buffer to put.
 * \param bufsiz          Buffer maximum size.
 * \param pbuflen [out]   Buffer insert point.
 * \param c               Help character to get (put into buffer).
 *
 * \return New column position.
 */
static int HelpGetC(int col, int colstart, char buf[], size_t bufsiz,
                    size_t *pbuflen, int c)
{
  if( *pbuflen >= bufsiz )
  {
    *pbuflen = bufsiz - 1;
    col = HelpPutBuf(col, colstart, buf, pbuflen);
  }
  else
  {
    buf[*pbuflen] = (char)c;
    *pbuflen += 1;
  }
  return col;
}

/*!
 * \brief Prints formatted description. 
 *
 * \par Formatting Directives:
 * The description argumnet sFmtDesc may contain the following formatting 
 * directives:\n
 * \b %P  - print program name (argv0)\n
 * \b %L  - print name of Long argument\n
 * \b %S  - print name of Short argument\n
 * \b %A  - print name of option Argument\n
 * \b %D  - print value of Default\n
 * \b %%  - print %
 *   
 *
 * \param col           Current column position.
 * \param colstart      Column to start.
 * \param sFmtDesc      Formatted description string.
 * \param sPgmName      Program name (argv0).
 * \param pOptsInfo     Options info.
 *
 * \return New column position.
 */
static int OptsPrintf(int col,
                      int colstart,
                      const char *sFmtDesc,
                      const char *sPgmName,
                      OptsInfo_T *pOptsInfo)
{
  char        buf[1028];
  size_t      buflen = 0;
  const char  *s;

  for(s=sFmtDesc; s && *s; ++s)
  {
    switch( *s )
    {
      case '\n':
        col = HelpPutBuf(col, colstart, buf, &buflen);
        col = HelpPutNL(colstart);
        break;
      case ' ':
      case '\t':
        col = HelpPutBuf(col, colstart, buf, &buflen);
        col = HelpPutC(col, colstart, ' ');
        break;
      case '%':
        ++s;
        switch(*s)
        {
          case 'P':
            if( sPgmName != NULL )
            {
              col = HelpPutBuf(col, colstart, buf, &buflen);
              col = HelpPutStr(col, colstart, sPgmName);
            }
            break;
          case 'L':
            if((pOptsInfo != NULL) && (pOptsInfo->long_opt != NULL))
            {
              col = HelpPutBuf(col, colstart, buf, &buflen);
              col = HelpPutStr(col, colstart, pOptsInfo->long_opt);
            }
            break;
          case 'S':
            if((pOptsInfo != NULL) && (pOptsInfo->short_opt != OPTS_NO_SHORT))
            {
              col = HelpGetC(col, colstart, buf, sizeof(buf), &buflen,
                              pOptsInfo->short_opt);
            }
            break;
          case 'A':
            if((pOptsInfo != NULL) && (pOptsInfo->arg_name != NULL))
            {
              col = HelpPutBuf(col, colstart, buf, &buflen);
              col = HelpPutStr(col, colstart, pOptsInfo->arg_name);
            }
            break;
          case 'D':
            if((pOptsInfo != NULL) && (pOptsInfo->pvt_opt_dft != NULL))
            {
              col = HelpPutBuf(col, colstart, buf, &buflen);
              col = HelpPutStr(col, colstart, pOptsInfo->pvt_opt_dft);
            }
            break;
          case '%':
            col = HelpGetC(col, colstart, buf, sizeof(buf), &buflen, *s);
            break;
          default:
            --s; 
            LOGERROR("Invalid print directive: '%.2s'", s);
            break;
        }
        break;
      default:
        col = HelpGetC(col, colstart, buf, sizeof(buf), &buflen, *s);
        break;
    }
  }

  // the last word
  if( buflen > 0 )
  {
    col = HelpPutBuf(col, colstart, buf, &buflen);
  }

  return col;
}

/*!
 * \brief Prints option syntax and description in standard 'usage' format. 
 *
 * \param argv0         Program name.
 * \param pOptsInfo     Options info.
 *
 * \return New column position.
 */
static void OptsPrintOpt(const char *argv0, OptsInfo_T *pOptsInfo)
{
  char        buf[1028], buf2[256];
  size_t      buflen = sizeof(buf), buflen2 = sizeof(buf2);
  int         col;

  // indent
  col = snprintf(buf, buflen, "  ");

  // short option
  if( pOptsInfo->short_opt != OPTS_NO_SHORT )
  {
    col += snprintf(buf+col, buflen-(size_t)col, "-%c", pOptsInfo->short_opt);
  }
  else
  {
    col += snprintf(buf+col, buflen-(size_t)col, "  ");
  }

  // long option
  if( pOptsInfo->long_opt != NULL )
  {
    col += snprintf(buf+col, buflen-(size_t)col, "%s--%s", 
                    (pOptsInfo->short_opt != OPTS_NO_SHORT? ", ": "  "),
                    pOptsInfo->long_opt);
  }

  // option argument name
  if( (pOptsInfo->has_arg != no_argument) && 
      (pOptsInfo->arg_name != NULL) )
  {
    col += snprintf(buf+col, buflen-(size_t)col, "%s%s", 
                    (pOptsInfo->long_opt != NULL? "=": " "),
                    pOptsInfo->arg_name);

  }

  // print option keyword(s) syntax
  buf[sizeof(buf)-1] = 0;
  printf("%s", buf);
  buflen = 0;

  // scoot over to start of description column
  if( col > HELP_OPT_OVER )
  {
    printf("\n");
    col = 0;
  }
  HelpIndent(HELP_OPT_OVER-col);
  col = HELP_OPT_OVER;

  // option default value string
  if( pOptsInfo->has_default
      && (pOptsInfo->fn_fmt != NULL)
      && (pOptsInfo->opt_addr != NULL) )
  {
    pOptsInfo->pvt_opt_dft = 
      pOptsInfo->fn_fmt(buf2, buflen2, pOptsInfo->opt_addr);
  }
  else
  {
    pOptsInfo->pvt_opt_dft = NULL;
  }

  // print description
  col = OptsPrintf(col, HELP_OPT_OVER_2, pOptsInfo->opt_desc, argv0,
                   pOptsInfo);

  printf("\n");

  // print default value
  if( pOptsInfo->pvt_opt_dft != NULL )
  {
    HelpIndent(HELP_OPT_OVER_2);
    printf("DEFAULT: %s\n", pOptsInfo->pvt_opt_dft); 
  }
}

/*!
 * \brief Prints command-line help. 
 *
 * This functions is called to act on --help option.
 *
 * \param argv0         Program name.
 * \param pOptsInfo     Options info.
 * \param pPgmInfo      Program info.
 * \param bHasLogging   This program does [not] support the logging optoins.
 */
static void OptsHelp(const char *argv0,
                     OptsInfo_T *pOptsInfo,
                     OptsPgmInfo_T *pPgmInfo,
                     bool_t bHasLogging)
{
  OptsInfo_T  *p;
  int          col;

  // usage
  col = printf("Usage: %s [OPTIONS] ", argv0);
  if( (pPgmInfo != NULL) && (pPgmInfo->usage_args != NULL) )
  {
    col = OptsPrintf(col, 2, pPgmInfo->usage_args, argv0, NULL);
  }

  // synopsis
  printf("\n");
  if( (pPgmInfo != NULL) && (pPgmInfo->synopsis != NULL) )
  {
    col = OptsPrintf(0, 0, pPgmInfo->synopsis, argv0, NULL);
    printf("\n");
  }

  // options
  printf(
"\n"
"Mandatory arguments to long options are also mandatory for short options.\n");

  if( pOptsInfo != NULL )
  {
    for(p=pOptsInfo; p->long_opt!=NULL; ++p)
    {
      OptsPrintOpt(argv0, p);
    }
  }

  for(p=OptsBuiltIn; p->long_opt!=NULL; ++p)
  {
    if( !bHasLogging && 
        ((p->pvt_retval == OPTS_RVAL_LOG)
         || (p->pvt_retval == OPTS_RVAL_LOGFILE)) )
    {
      continue;
    }
    if( p->pvt_retval == OPTS_RVAL_HELP )
    {
      printf("\n");
    }
    OptsPrintOpt(argv0, p);
  }

  // long description
  if( (pPgmInfo != NULL) && (pPgmInfo->long_desc != NULL) )
  {
    printf("\n");
    col = OptsPrintf(0, 0, pPgmInfo->long_desc, argv0, NULL);
    printf("\n");
  }

  // diagnostics
  if( (pPgmInfo != NULL) && (pPgmInfo->diagnostics != NULL) )
  {
    printf("\n");
    printf("DIAGNOSTICS:\n");
    col = OptsPrintf(0, 0, pPgmInfo->diagnostics, argv0, NULL);
    printf("\n");
  }
}

/*!
 * \brief Prints command-line program version string(s). 
 *
 * This functions is called to act on --version option.
 *
 * \param argv0         Program name.
 * \param pPkgInfo      Package info.
 */
static void OptsVersion(const char *argv0, const PkgInfo_T *pPkgInfo)
{
  printf(
"%s (package %s-%s %s)\n" 
"Written by %s\n\n"
"Copyright (C) %s %s\n"
"%s\n",
    argv0,
    pPkgInfo->m_sPkgName, pPkgInfo->m_sPkgVersion, pPkgInfo->m_sPkgTimeStamp,
    pPkgInfo->m_sPkgAuthors,
    pPkgInfo->m_sPkgDate, pPkgInfo->m_sPkgOwners,
    pPkgInfo->m_sPkgDisclaimer);
}

/*!
 * \brief Set program logging level.
 *
 * This function is called to act on the --log \<level\> option.
 *
 * \param argv0     Program name.
 * \param sOptName  Long option name.
 * \param optarg    Long option argument (from getopt_long(3)).
 */
static void OptsLogSetLevel(const char *argv0, const char *sOptName,
                            char *optarg)
{
#ifdef LOG
  int level;

  OptsCvtArgLogLevel(argv0, sOptName, optarg, &level);
  LOG_SET_THRESHOLD(level);
#endif // LOG
}

/*!
 * \brief Set program logging output file.
 *
 * This function is called to act on the --logfile \<file\> option.
 *
 * \param argv0     Program name.
 * \param sOptName  Long option name.
 * \param optarg    Long option argument (from getopt_long(3)).
 */
static void OptsLogSetFile(const char *argv0, const char *sOptName,
                            char *optarg)
{
#ifdef LOG
  char *sFileName;

  OptsCvtArgStr(argv0, sOptName, optarg, &sFileName);
  LOG_SET_LOGFILE(sFileName);
#endif // LOG
}

/*!
 * \brief Disable program logging in color.
 *
 * This function is called to act on the --log-no-color option.
 *
 * \param argv0     Program name.
 * \param sOptName  Long option name.
 * \param optarg    Long option argument (from getopt_long(3)).
 */
static void OptsLogDisableColor(const char *argv0, const char *sOptName,
                                char *optarg)
{
#ifdef LOG
  LOG_SET_COLOR_ENABLE(false);
#endif // LOG
}


/*!
 * \brief Disable program logging with timestamps.
 *
 * This function is called to act on the --log-no-timestamp option.
 *
 * \param argv0     Program name.
 * \param sOptName  Long option name.
 * \param optarg    Long option argument (from getopt_long(3)).
 */
static void OptsLogDisableTimestamp(const char *argv0, const char *sOptName,
                                    char *optarg)
{
#ifdef LOG
  LOG_SET_TIMESTAMP_ENABLE(false);
#endif // LOG
}
/*!
 * \brief Allocate and build short and long options getopt_long() parameters
 * from the provided option info.
 *
 * \param pOptsInfo     Options info table.
 * \param bHasLogging   This program does [not] support the logging optoins.
 *
 * \return New options control structure.
 */
static OptsCtl_T *OptsNew(OptsInfo_T *pOptsInfo, bool_t bHasLogging)
{
  size_t         n;
  struct option *pLongOpts, *p;
  OptsInfo_T    *q;
  char          *sShortOpts, *s;
  OptsCtl_T     *pOptsCtl;

  // count the number of options
  for(n=0; pOptsInfo[n].long_opt!=NULL; ++n);   // user

  n += arraysize(OptsBuiltIn) - 1;

  // allocate memory
  pLongOpts   = new(sizeof(struct option)*(n+1)); // include null
  sShortOpts  = new(sizeof(char)*(n*3+1));        // include null

  //
  // Make Built-In long and short option arguments to getopt_long()
  //
  for(p=pLongOpts, s=sShortOpts, q=OptsBuiltIn; q->long_opt!=NULL; ++q)
  {
    // ignore logging options
    if( !bHasLogging 
        && (!strcmp(q->long_opt, OPTS_LONGOPT_LOG) 
            || !strcmp(q->long_opt, OPTS_LONGOPT_LOGFILE)) )
    {
      continue;
    }

    // long option also has corresponding short option
    if( q->short_opt != OPTS_NO_SHORT )
    {
      *s++ = (char)(q->short_opt);
      if( q->has_arg != no_argument )
      {
        *s++ = ':';
      }
      if( q->has_arg == optional_argument )
      {
        *s++ = ':';
      }
      
      // check return value must be an ascii characater
      if( q->pvt_retval != q->short_opt )
      {
        fprintf(stderr,
            "Warning: Built-in option pvt_retval 0x%x != short_opt '%c'\n",
            q->pvt_retval, q->short_opt);
      }
    }

    // long option
    p->name     = q->long_opt;
    p->has_arg  = q->has_arg;
    p->flag     = NULL;
    p->val      = q->pvt_retval;
    p++;
  }
  
  //
  // Make User-Defined long and short option arguments to getopt_long()
  //
  for(n=0, q=pOptsInfo; q->long_opt!=NULL; ++n, ++q)
  {
    // long option only
    if( q->short_opt == OPTS_NO_SHORT )
    {
      q->pvt_retval  = OPTS_RVAL_USER + (int)n;
    }
    // long option also has corresponding short option
    else
    {
      *s++ = (char)(q->short_opt);
      if( q->has_arg != no_argument )
      {
        *s++ = ':';
      }
      if( q->has_arg == optional_argument )
      {
        *s++ = ':';
      }
      q->pvt_retval  = q->short_opt;
    }

    // long option
    p->name     = q->long_opt;
    p->has_arg  = q->has_arg;
    p->flag     = NULL;
    p->val      = q->pvt_retval;
    p++;
  }

  // end of long options
  p->name     = NULL;
  p->has_arg  = no_argument;
  p->flag     = NULL;
  p->val      = 0;

  // end of short options string
  *s = 0;

  pOptsCtl = NEW(OptsCtl_T);
  pOptsCtl->m_pLongOpts   = pLongOpts;
  pOptsCtl->m_sShortOpts  = sShortOpts;

  return pOptsCtl;
}

/*!
 * \brief Delete an allocated options control structure.
 *
 * \param pOptsCtl    Options control structure.
 */
static void OptsDelete(OptsCtl_T *pOptsCtl)
{
  if( pOptsCtl != NULL )
  {
    delete(pOptsCtl->m_pLongOpts);
    delete(pOptsCtl->m_sShortOpts);
    delete(pOptsCtl);
  }
}

// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------
                                                                                
/*!
 * \brief Gets, validates, and sets all command line options.
 *
 * \param argv0       (Massaged) program name.
 * \param pPkgInfo    Package info.
 * \param pPgmInfo    Program info.
 * \param pOptsInfo   Program options info.
 * \param bHasLogging Do [not] include logging options.
 * \param[out] pargc  New non-option argument count.      
 * \param argv        Command-line options and arguments.
 *                    arguments.
 *
 * \return Reposition command-line arguments to start of first non-option
 *         comand argument.
 */
char **OptsGet(const char *argv0,
               const PkgInfo_T *pPkgInfo,
               OptsPgmInfo_T *pPgmInfo,
               OptsInfo_T *pOptsInfo,
               bool_t bHasLogging,
               int *pargc, char *argv[])
{
  OptsCtl_T     *pOptsCtl;
  OptsInfo_T    *p;
  int           argc = *pargc;
  int           c;
  int           option_index;
  bool_t        bHasOpt;

  //
  // Set external getopt_long() variables. N.B. makes OptsGet() non-reentrant.
  //
  optarg  = NULL; // current argument to option
  optind  = 0;    // next index in argv to look for options
  opterr  = 1;    // allow getopt_long() to print error messages
  optopt  = 0;    // current parsed option character

  // build option arguments from info
  pOptsCtl = OptsNew(pOptsInfo, bHasLogging);

  // parse and convert options
  while(true)
  {
    option_index = 0;

    // parse next long/short option
    c = getopt_long(argc, argv, pOptsCtl->m_sShortOpts, pOptsCtl->m_pLongOpts,
                    &option_index);

    // end of option list
    if( c == -1 )
    {
      break;
    }

    // process option
    switch(c)
    {
      case OPTS_RVAL_ERROR:         // -? options error
        OptsInvalid(argv0, "");
        break;
      case OPTS_RVAL_HELP:          // --help
        OptsHelp(argv0, pOptsInfo, pPgmInfo, bHasLogging);
        exit(OK);
        break;
      case OPTS_RVAL_VERSION:       // --version
        OptsVersion(argv0, pPkgInfo);
        exit(OK);
        break;
      case OPTS_RVAL_LOG:           // -l, --log <level>
        OptsLogSetLevel(argv0, OPTS_LONGOPT_LOG, optarg);
        break;
      case OPTS_RVAL_LOGFILE:       // --logfile <file>
        OptsLogSetFile(argv0, OPTS_LONGOPT_LOGFILE, optarg);
        break;
      case OPTS_RVAL_LOGNOCOLOR:    // --log-no-color
        OptsLogDisableColor(argv0, OPTS_LONGOPT_LOG, optarg);
        break;
      case OPTS_RVAL_LOGNOTIME:     // --log-no-timestamp
        OptsLogDisableTimestamp(argv0, OPTS_LONGOPT_LOG, optarg);
        break;
      default:                      // user application option
        bHasOpt = false;
        for(p=pOptsInfo; (p!=NULL) && (p->long_opt!=NULL); ++p)
        {
          if( p->pvt_retval == c )
          {
            bHasOpt = true;

            // user supplied option conversion function
            if( p->fn_cvt != NULL )
            {
              p->fn_cvt(argv0, p->long_opt, optarg, p->opt_addr);
            }
            // default for option w/o argument is boolean true value
            else if( p->has_arg == no_argument )
            {
              *((int *)p->opt_addr) = true;
            }
            // default for option w/ argument is string value
            else
            {
              OptsCvtArgStr(argv0, p->long_opt, optarg, p->opt_addr);
            }
          }
        }
        if( !bHasOpt )
        {
          fprintf(stderr, "Warning: getopt_long() return character code 0x%x\n",
                            c);
          exit(EC_BAD_OPT);
        }
        break;
    }
  }

  OptsDelete(pOptsCtl);

  OPTS_RESET_ARGCV(optind, argc, argv);

  *pargc = argc;

  return argv;
}

/*!
 * \brief Convert options string argument to string.
 *
 * \note Function returns integer for derived applications. librnr Opts
 * ignores the return value.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value.
 *
 * \par Default:
 * \p "" (zero-length string)
 *
 * \return If returns, then returns OK.
 */
int OptsCvtArgStr(const char *argv0, const char *sOptName, char *optarg,
               void *pOptVal)
{
  *((char **)pOptVal) = new_strdup(optarg);
  return OK;
}

//
// Convert options boolean argument to bool
//
/*!
 * \brief Convert options boolean argument to bool_t.
 *
 *    1 or "true" converts to \p true.
 * \n 0 or "false" converts to \p false.
 *
 * \note Function returns integer for derived applications. librnr Opts
 * ignores the return value.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value.
 *
 * \par Default:
 * \p true
 *
 * \exception OPTSBADARG()
 *
 * \return If returns, then returns OK.
 */
int OptsCvtArgBool(const char *argv0, const char *sOptName, char *optarg,
               void *pOptVal)
{
  if( optarg )
  {
    if( !strcmp(optarg, "true") || !strcmp(optarg, "1") )
    {
      *((bool_t *)pOptVal) = true;
    }
    else if( !strcmp(optarg, "false") || !strcmp(optarg, "0") )
    {
      *((bool_t *)pOptVal) = false;
    }
    else
    {
      OPTSBADARG(argv0, sOptName, optarg);
    }
  }
  // optional argument default
  else
  {
    *((bool_t *)pOptVal) = true;
  }

  return OK;
}

/*!
 * \brief Convert options integer argument to integer.
 *
 * Accepted formats: decimal hexidecimal(0x<hexdigits>) octal(0<octaldigits>).
 *
 * \note Function returns integer for derived applications. librnr Opts
 * ignores the return value.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value.
 *
 * \par Default:
 * \p 1
 *
 * \exception OPTSBADARG()
 *
 * \return If returns, then returns OK.
 */
int OptsCvtArgInt(const char *argv0, const char *sOptName, char *optarg,
               void *pOptVal)
{
  int   optargInt;
  char  *optargEnd;

  if( optarg )
  {
    optargInt = (int)strtol(optarg, &optargEnd, 0);
    if( *optargEnd != 0 )
    {
      OPTSBADARG(argv0, sOptName, optarg);
    }
    *((int *)pOptVal) = optargInt;
  }
  // optional argument default
  else
  {
    *((int *)pOptVal) = 1;
  }

  return OK;
}

/*!
 * \brief Convert options float argument to double.
 *
 * Accepted format: optionally signed floating-point number. 
 *
 * \note Function returns integer for derived applications. librnr Opts
 * ignores the return value.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value.
 *
 * \par Default:
 * \p 1.0
 *
 * \exception OPTSBADARG()
 *
 * \return If returns, then returns OK.
 */
int OptsCvtArgFloat(const char *argv0, const char *sOptName, char *optarg,
               void *pOptVal)
{
  double  optargFloat;

  if( optarg )
  {
    if( sscanf(optarg, "%lf", &optargFloat) != 1 )
    {
      OPTSBADARG(argv0, sOptName, optarg);
    }
    *((double *)pOptVal) = optargFloat;
  }
  // optional argument default
  else
  {
    *((double *)pOptVal) = 1.0;
  }

  return OK;
}

/*!
 * \brief Convert options string argument to log threshold level.
 *
 * Format: "off", "error", "diag1", "diag2", "diag3", "diag4", "diag5", or >= 0
 *
 * \note Function returns integer for derived applications. librnr Opts
 * ignores the return value.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value.
 *
 * \par Default:
 * \p LOG_LEVEL_DFT
 *
 * \exception OPTSBADARG()
 *
 * \return If returns, then returns OK.
 */
int OptsCvtArgLogLevel(const char *argv0, const char *sOptName, char *optarg,
               void *pOptVal)
{
  int logLevel;

  if( optarg )
  {
    logLevel = NvpName2Val(OptsLogLevelTbl, arraysize(OptsLogLevelTbl), optarg);

    // assume integer format
    if( logLevel < LOG_LEVEL_OFF )
    {
      OptsCvtArgInt(argv0, sOptName, optarg, &logLevel);
    }
  }
  // optional argument default
  else
  {
    logLevel = LOG_LEVEL_DFT;
  }

  if( logLevel < LOG_LEVEL_OFF )
  {
    logLevel = LOG_LEVEL_OFF;
  }

  *((int *)pOptVal) = logLevel;

  return OK;
}

/*!
 * \brief String option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtStr(char *buf, size_t buflen, void *pOptVal)
{
  
  if( (pOptVal == NULL) || (*((char **)pOptVal) == NULL) )
  {
    buf[0] = 0;
  }
  else
  {
    strncpy(buf, *((char **)pOptVal), buflen);
    buf[buflen-1] = 0;
  }
  return buf;
}

/*!
 * \brief Integer option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtInt(char *buf, size_t buflen, void *pOptVal)
{
  snprintf(buf, buflen, "%d", *((int *)pOptVal));
  buf[buflen-1] = 0;
  return buf;
}

/*!
 * \brief Float option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtFloat(char *buf, size_t buflen, void *pOptVal)
{
  snprintf(buf, buflen, "%f", *((double *)pOptVal));
  buf[buflen-1] = 0;
  return buf;
}

/*!
 * \brief Boolean option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtBool(char *buf, size_t buflen, void *pOptVal)
{
  char  *s;

  if( *((int *)pOptVal) == true )
  {
    s = "true";
  }
  else
  {
    s = "false";
  }
  strncpy(buf, s, buflen);
  buf[buflen-1] = 0;
  return buf;
}

/*!
 * \brief Character option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtChar(char *buf, size_t buflen, void *pOptVal)
{
  buf[0] = *((char *)pOptVal);
  buf[1] = 0;
  return buf;
}

/*!
 * \brief Log Level option value string formatter.
 *
 * \param[out] buf  Output buffer.
 * \param buflen    Buffer length.
 * \param pOptVal   Option value.
 *
 * \return Pointer to start of buf.
 */
char *OptsFmtLogLevel(char *buf, size_t buflen, void *pOptVal)
{
  int   level = *((int *)pOptVal);
  char *off   = "off";

  if( level == LOG_LEVEL_OFF )
  {
    return OptsFmtStr(buf, buflen, &off);
  }
  else
  {
    return OptsFmtInt(buf, buflen, pOptVal);
  }
}

/*!
 * \brief Invalid option or option argument print and exit.
 *
 * \param argv0 Program name.
 * \param sFmt  Error format string.
 * \param ...   Variable arguments to format string.
 *
 * \note The function terminates process with exit status \p EC_BAD_OPT(2).
 */
void OptsInvalid(const char *argv0, const char *sFmt, ...)
{
  va_list ap;
  char    fmtbuf[MAX_FMT_BUF];

  if( (sFmt != NULL) && (*sFmt != 0) )
  {
    snprintf(fmtbuf, MAX_FMT_BUF, "%s: %s\n", argv0, sFmt);
    fmtbuf[MAX_FMT_BUF-1] = 0;
    va_start(ap, sFmt);
    vfprintf(stderr, fmtbuf, ap); 
    va_end(ap);
  }
  fprintf(stderr, "Try '%s --help' for more information.\n", argv0);
  exit(EC_BAD_OPT);
}
