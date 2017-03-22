////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Standard command-line options options and parsing.
 *
 * Every application that uses librnr options facilities are provided with
 * a set of built-in options (described below) to standardize RoadNarrows
 * application interfaces.
 *
 * \par Built-In Options
 * <dl>
 *
 * <dt> <b>-l, --log=<i>level</i></b></dt>
 * <dd>
 * Set logging threshold level. All logging events 
 * with priority \h_le <b><i>level</i></b> will be logged. All 
 * others will be ignored. The <b><i>level</i></b> argument is one of:
 * <table style="border:0">
 *  <tr><td>\c 'off' or \c 0</td><td>Disable all logging.</td></tr>
 *  <tr><td>\c 'error' or \c 1</td>
 *    <td>Enable error and warning logging.</td></tr>
 *  <tr><td>\c 'diag1' or \c 2</td><td>Enable diagnostics 1 logging.</td></tr>
 *  <tr><td>\c 'diag2' or \c 3</td><td>Enable diagnostics 2 logging.</td></tr>
 *  <tr><td>\c 'diag3' or \c 4</td><td>Enable diagnostics 3 logging.</td></tr>
 *  <tr><td>\c 'diag4' or \c 5</td><td>Enable diagnostics 4 logging.</td></tr>
 *  <tr><td>\c 'diag5' or \c 6</td><td>Enable diagnostics 5 logging.</td></tr>
 *  <tr><td>\h_gt \c 6     </td><td>Enable user-defined logging.</td></tr>
 * </table>
 * \b DEFAULT: \c off
 * </dd>
 *
 * <dt> <b>--logfile=<i>file</i></b></dt>
 * <dd>
 * Set log file <b><i>file</i></b>. Special <b><i>file</i></b> names:
 * <table style="border:0">
 *  <tr><td>\c 'stderr'</td><td>log to standard error.</td></tr>
 *  <tr><td>\c 'stdout'</td><td>log to standard output.</td></tr>
 * </table>
 * \b DEFAULT: \c stderr
 * </dd>
 *
 * <dt> <b>--log-no-color</b></dt>
 * <dd>
 * Disable logging with compiled ANSI color strings.<br>
 * \b DEFAULT: \c false
 * </dd>
 *
 * <dt> <b>--log-no-timestamp</b></dt>
 * <dd>
 * Disable logging with timestamps.<br>
 * \b DEFAULT: \c false
 *
 * </dd>
 * <dt> <b>--help</b></dt>
 * <dd>Display this help and exit.</dd>
 *
 * <dt> <b>--version</b></dt>
 * <dd>Output version information and exit.</dd>
 *
 * </dl>
 * 
 * \sa 
 * Page \ref example_log under "Related Pages" for an example usage of options.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/opts.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
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
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_OPTS_H
#define _RNR_OPTS_H

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


#endif // _RNR_OPTS_H
