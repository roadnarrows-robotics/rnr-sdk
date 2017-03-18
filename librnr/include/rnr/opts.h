////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      opts.h
//
/*! \file
 *
 * $LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
 * $Rev: 3823 $
 *
 * \brief Standard command-line options built-in options and parsing.
 *
 * Every application that uses librnr options facilities are provides with
 * a set of built-in options (describe below) to standardize RoadNarrows
 * application interfaces.
 *
 * \par Built-In Options:
 * <dl>
 * <dt> \b --log, \b -l </dt> <dd>Diagnostics logging level.</dd>
 * <dt> \b --logfile </dt> <dd>Diagnostics logging output file.</dd>
 * <dt> \b --help </dt> <dd>Print help (usage) for the command.</dd>
 * <dt> \b --version </dt> <dd>Print command version string(s).</dd>
 * </dl>
 *
 * \sa 
 * Page \ref example_log under "Related Pages" for an example usage of options.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
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

#ifndef _OPTS_H
#define _OPTS_H

#include <limits.h>
#include <getopt.h>

#include "rnr/rnrconfig.h"
#include "rnr/pkg.h"

C_DECLS_BEGIN

//
// No short option
//
#define OPTS_NO_SHORT   0   ///< no short option equivalent

//
// Reserved short and long option return values.
//
#define OPTS_RVAL_ERROR       '?'                   ///< options error
#define OPTS_RVAL_LOG         'l'                   ///< -l, --log return value
#define OPTS_RVAL_LONG        (CHAR_MAX + 1)        ///< long return value start
#define OPTS_RVAL_HELP        (OPTS_RVAL_LONG)      ///< --help return value
#define OPTS_RVAL_VERSION     (OPTS_RVAL_LONG + 1)  ///< --version return value
#define OPTS_RVAL_LOGFILE     (OPTS_RVAL_LONG + 2)  ///< --logfile return value
#define OPTS_RVAL_LOGNOCOLOR  (OPTS_RVAL_LONG + 3)  ///< --log-no-color rval
#define OPTS_RVAL_LOGNOTIME   (OPTS_RVAL_LONG + 4)  ///< --log-no-timestamp rval
#define OPTS_RVAL_USER        (OPTS_RVAL_LONG + 5)  ///< start of user available


/*!
 * \brief Option Argument Conversion Function Type.
 */
typedef int (*OptsCvtFunc_T)(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal);

/*!
 * \brief Option Value String Formatter Type.
 */
typedef char *(*OptsFmtFunc_T)(char *buf, size_t buflen, void *pOptVal);

/*!
 * \brief Short and Long Options Info.
 *
 * Short options are assumed to be a proper subset of the long options.
 *
 * \note The member naming convention is not RN Hungarian to accommodate
 * readability of .name=val structure initialization.
 */
typedef struct 
{
  /*! \brief Long option string name. */
  const char     *long_opt;

  /*! \brief Short option character. 
   *
   * Set to \p OPTS_NO_SHORT if no short option equivalent exists.
   */
  int             short_opt;

  /*! \brief Option does [not] have an argument of type.
   *
   * One of: \p no_argument, \p required_argument, \p optional_argument.
   */
  int             has_arg;

  /*! \brief Option does [not] have a default value.
   *
   * \note *opt_addr must hold the default at OptsGet() call.
   */
  bool_t          has_default;

  /*! \brief Address of option variable.
   *
   * The option variable holds the parsed option converted value. If there
   * is a default value the variable must be initialized to the default.
   */
  void           *opt_addr;

  /*! \brief Option conversion function.
   *
   * \sa OptsCvtArgStr(), OptsCvtArgBool(), OptsCvtArgInt(),
   * OptsCvtArgLogLevel() for built-in conversion functions.
   *
   */
  OptsCvtFunc_T   fn_cvt;

  /*! \brief Option value string formatting function.
   *
   * \sa OptsFmtStr(), OptsFmtInt(), OptsFmtBool(), OptsFmtChar(),
   * OptsFmtLogLevel() for built-in option string formatters.
   */
  OptsFmtFunc_T   fn_fmt;

  /*! \brief Option argument name string.
   *
   * \note  Only used in help.
   */
  const char     *arg_name;

  /*! \brief Option description string.
   *
   * The option description may contain the following formatting directives:
   * \n \p \%P  - print program name (argv0)
   * \n \p \%L  - print name of Long argument
   * \n \p \%S  - print name of Short argument
   * \n \p \%A  - print name of option Argument
   * \n \p \%D  - print value of Default
   * \n \p %%  - print %
    */
  const char     *opt_desc;

  /*! \brief Return value.
   *
   * Expected return value of parsed option. from \p getopt_long().
   * If short option exist, then short option ascii value will be used.
   *
   * \note Internal (private) use by opts processing.
   */
  int             pvt_retval;

  /*! \brief Option default value in string format
   *
   * Converts option default using specified string formatter.
   *
   * \note Internal (private) use by opts processing.
   */
  char           *pvt_opt_dft;
} OptsInfo_T;

/*!
 * \brief Program Description Strings Info Structure.
 *
 * Strings may use the \p "%P" (program name) formatting directive,
 * plus \p '\\n' and \p '\\t' spacing characters.
 *
 * Set any string to \p NULL to ignore.
 *
 * \note The member naming convention is not RN Hungarian to accommodate
 * readability of .name=val structure initialization.
 */
typedef struct
{
  /*! \brief Single line description string of non-option arguments.
   *
   * \par Example:
   * \p "THING1 [THING2] [FILE...]"
   */
  const char  *usage_args;

  /*! \brief Simple program synopsis string.
   *
   * \par Example:
   * \p "Do THING1, maybe THING2 to FILE(s)"
   */
  const char  *synopsis;

  /*! \brief Program full description string.
   *
   * \par Example:
   * \p "By actions of %P the Cat-In-The-Hat does things.\nBad things."
   */
  const char  *long_desc;

  /*! \brief Programs diagnostics subsection string.
   *
   * \par Example:
   * \p "Exit statis is 0 if THING(s) are bad, 1 if captured."
   */
  const char  *diagnostics;
} OptsPgmInfo_T;

/*!
 * \brief Standard bad option error message reporter
 * \param argv0 Command name string.
 * \param opt   Option name string.
 * \param arg   Option argument name string.
 */
#define OPTSBADARG(argv0, opt, arg) \
  OptsInvalid(argv0, "Invalid '%s' argument to '%s' option.", arg, opt)


//
// Prototypes
//

// The options parser
extern char **OptsGet(const char *argv0,
                      const PkgInfo_T *pPkgInfo,
                      OptsPgmInfo_T *pPgmInfo,
                      OptsInfo_T *pOptsInfo,
                      bool_t bHasLogging,
                      int *pargc, char *argv[]);

// Standard option convsersion functions
extern int OptsCvtArgStr(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
extern int OptsCvtArgBool(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
extern int OptsCvtArgInt(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
extern int OptsCvtArgFloat(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
extern int OptsCvtArgLogLevel(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);

// Standard option value string formatter functions
extern char *OptsFmtStr(char *buf, size_t buflen, void *pOptVal);
extern char *OptsFmtInt(char *buf, size_t buflen, void *pOptVal);
extern char *OptsFmtFloat(char *buf, size_t buflen, void *pOptVal);
extern char *OptsFmtBool(char *buf, size_t buflen, void *pOptVal);
extern char *OptsFmtChar(char *buf, size_t buflen, void *pOptVal);
extern char *OptsFmtLogLevel(char *buf, size_t buflen, void *pOptVal);

// Error reporters
extern void OptsInvalid(const char *argv0, const char *sFmt, ...);

C_DECLS_END


#endif // _OPTS_H
