////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      log.h
//
/*! \file
 *
 * $LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
 * $Rev: 3823 $
 *
 * \brief Logger declarations.
 *
 * Logging supports error and diagnostics (debugging) reports to stderr or a
 * specified file. Logging reporting is filtered by a simple threshold level.
 *
 * The logging facilities are designed to support independent logging 
 * capabilities through namespaceing. Copy log.h and log.c to user application
 * source tree and compile with LOGNS uniquely defined.
 *
 * \par There are 7 defined loggging levels:
 * <dl>
 * <dt> \ref LOGGING_NA </dt>
 *  <dd>Logging is not compiled into the application.
 *      See \ref LOG_GET_THRESHOLD() to test level including this level.</dd>
 * <dt> \ref LOG_LEVEL_OFF (0) </dt>
 *  <dd>Logging is turned off.</dd>
 * <dt> \ref LOG_LEVEL_ERROR (1) </dt>
 *  <dd>Log errors only.</dd>
 * <dt> \ref LOG_LEVEL_DIAG1 (2) </dt>
 *  <dd>Log errors plus level 1 diagnostics.</dd>
 * <dt> \ref LOG_LEVEL_DIAG2 (3) </dt>
 *  <dd>Log errors plus levels 1-2 diagnostics.</dd>
 * <dt> \ref LOG_LEVEL_DIAG3 (4) </dt>
 *  <dd>Log errors plus levels 1-3 diagnostics.</dd>
 * <dt> \ref LOG_LEVEL_DIAG4 (5) </dt>
 *  <dd>Log errors plus levels 1-4 diagnostics.</dd>
 * <dt> \ref LOG_LEVEL_DIAG5 (6) </dt>
 *  <dd>Log errors plus levels 1-5 diagnostics.</dd>
 * <dt> \b &gt;7 </dt>
 * <dd>User defined loggging levels.</dd>
 * </dl>
 *
 * \par Special make defines:
 * <dl>
 * <dt> \b LOG </dt>
 *  <dd>If defined, complile logging into an application.\n
 *      Default: not defined.</dd>
 * <dt> \ref LOGNS </dt>
 *  <dd>Log namespace string. All external identifiers will be
 *      prefixed by \ref LOGNS\<<em>id</em>\>. This is usefull, for example, if
 *      a program uses logging and is also linked with a library
 *      that also uses logging.\n
 *      Default: \<emptystring\> </dd>
 * <dt> \ref LOGMOD </dt>
 *  <dd>If defined, all logging output will be prefaced with this string.\n
 *      Default: \ref LOGNS </dd>
 * <dt> \ref LOG_LEVEL_DFT </dt>
 *  <dd>Initial default logging threshold level.\n
 *      Default: 0 (off)</dd>
 * </dl>
 *
 * \sa 
 * Page \ref example_log under "Related Pages" for an example usage of logging.
 *
 * \todo
 * Define log class concepts whereby logging can be filter by threshold and
 * class.
 *    LogClassSet(...), LogClassClear(), LogClassAdd(...), LogClassDel(...)
 * Class maps to bit in bitmap. All logging macros check loglevel and class bit.
 * Reserved names: '_all_', '_none_', '_common_' (default)
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2005-2012.  RoadNarrows LLC.
 * \n All Rights Reserved
 */
//
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

#ifndef _LOG_H
#define _LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

//
// Usefull macro utilities
//
#define CONCAT_(x, y) x ## y              ///< build concatenation operator
#define CONCAT(x, y)  CONCAT_(x, y)       ///< now concatenate
#define MKSTR_(x)     #x                  ///< build token into string operator
#define MKSTR(x)      MKSTR_(x)           ///< now make string literal

//
// Logging Entity's Namespace
//
#ifndef LOGNS

#undef  LOGNS_DEF                         ///< default logging namespace
#define LOGNS                             ///< default namespace preface token
#define LOGNS_NAME ""                     ///< default namespace string name
#undef  LOGNS_PREFACE                     ///< default logging preface string

#else // user defined namespace

#define LOGNS_DEF                         ///< separate logging namespace
#define LOGNS_NAME MKSTR(LOGNS)           ///< namespace string name
#define LOGNS_PREFACE LOGNS_NAME          ///< logging preface string

#endif // LOGNS

#define LOGNS_PUT(id)  CONCAT(LOGNS, id)  ///< put identifier into namespace

//
// Loggging module name
//
#ifdef LOGMOD
#define LOG_PREFACE_TEXT LOGMOD ": "          ///< user logging preface 
#elif defined(LOGNS_PREFACE)
#define LOG_PREFACE_TEXT LOGNS_PREFACE ": "   ///< namespace logging preface
#else
#define LOGMOD                                ///< default logging module name
#define LOG_PREFACE_TEXT ""                   ///< no logging preface
#endif // LOGMOD


//
// Global Names in namespace
//
#define LOG_VAR_THRESHOLD LOGNS_PUT(LogThresholdLevel)
                                ///< current threshold variable name

#define LOG_SET_THRESHOLD LOGNS_PUT(LogSetThresholdLevel)
                                ///< set threshold function name

#define LOG_GET_THRESHOLD LOGNS_PUT(LogGetThresholdLevel)
                                ///< get threshold function name

#define LOG_VAR_COLOR_EN LOGNS_PUT(LogColorEnable)
                                ///< logging in color is [not] enabled

#define LOG_SET_LOGFILE   LogSetLogFile
                                ///< set output log file function name
#define LOG_GET_LOGFILE   LogGetLogFile
                                ///< get output log file function name
#define LOG_SET_COLOR_ENABLE   LogSetColorEnable
                                ///< enable/disable logging in color func name
#define LOG_SET_TIMESTAMP_ENABLE LogSetTimestampEnable
                                ///< enable/disable logging timestamps func name
#define LOG_ATTACH_LOGFP  LogAttachLogFp
                                ///< attach file pointer as logging out stream
#define LOG_GET_LOGFP     LogGetLogFp
                                ///< get current logging out stream file pointer
#define LOGGER            LogPrintf
                                ///< logger function name
#define LOGGER_CALL       LogCallPrintf
                                ///< logger of "function call" function name

//
// Log Levels (levels >6 are user defined)
//
#define LOGGING_NA      -1   ///< logging not available (not compiled)
#define LOG_LEVEL_OFF    0   ///< turn off all non-error logging
#define LOG_LEVEL_ERROR  1   ///< errors
#define LOG_LEVEL_DIAG1  2   ///< diagnostic level 1
#define LOG_LEVEL_DIAG2  3   ///< diagnostic level 2
#define LOG_LEVEL_DIAG3  4   ///< diagnostic level 3
#define LOG_LEVEL_DIAG4  5   ///< diagnostic level 4
#define LOG_LEVEL_DIAG5  6   ///< diagnostic level 5
#ifndef LOG_LEVEL_DFT
#define LOG_LEVEL_DFT    0   ///< default log level is off
#endif

/*!
 * \brief Test if given level is logable at current threshold.
 *
 * \param level Level to test.
 */
#define LOGABLE(level) ((level) <= LOG_VAR_THRESHOLD)

/*!
 * \brief Test if logging in color.
 */
#define LOG_IN_COLOR() (LOG_VAR_COLOR_EN)

/*!
 * \brief Test if logging includes timestamps.
 */
#define LOG_WITH_TIMESTAMP() (LOG_VAR_TIMESTAMP_EN)

//
// Log Filenames and Pointers
//
#define LOG_FILENAME_STDERR   "stderr"            ///< 'stderr' log filename
#define LOG_FILENAME_STDOUT   "stdout"            ///< 'stdout' log filename
#define LOG_FILENAME_DFT      LOG_FILENAME_STDERR ///< default log filename
#define LOG_FP_DFT            stderr              ///< default log out stream

//
// Other Logging Options
//
#define LOG_COLOR_EN_DFT      true   ///< default is to log in color
#define LOG_TIMESTAMP_EN_DFT  true   ///< default is to include timestamps

//
// Log function name formatters (must keep literal)
// Note: Might be protability issues with this macro.
//
#define LOGFUNCNAME    __func__           ///< function name

//
// Log Colors. Colors are expressed in ANSI escape code sequences.
//
#define LOG_COLOR_PRE           "\033["     ///< color escape sequence prefix
#define LOG_COLOR_POST          "\033[0m"   ///< color escape sequence postfix
#define LOG_COLOR_RED           "0;31m"     ///< normal red
#define LOG_COLOR_GREEN         "0;32m"     ///< normal green
#define LOG_COLOR_YELLOW        "0;33m"     ///< normal yellow
#define LOG_COLOR_BLUE          "0;34m"     ///< normal blue
#define LOG_COLOR_MAGENTA       "0;35m"     ///< normal magenta
#define LOG_COLOR_CYAN          "0;36m"     ///< normal cyan
#define LOG_COLOR_LIGHT_RED     "1;31m"     ///< light red
#define LOG_COLOR_LIGHT_GREEN   "1;32m"     ///< light green
#define LOG_COLOR_LIGHT_YELLOW  "1;33m"     ///< light yellow
#define LOG_COLOR_LIGHT_BLUE    "1;34m"     ///< light blue
#define LOG_COLOR_LIGHT_MAGENTA "1;35m"     ///< light magenta
#define LOG_COLOR_LIGHT_CYAN    "1;36m"     ///< light cyan

#define LOG_COLOR_ERROR   LOG_COLOR_PRE LOG_COLOR_RED     ///< error color
#define LOG_COLOR_WARN    LOG_COLOR_PRE LOG_COLOR_YELLOW  ///< warning color
#define LOG_COLOR_DIAG    LOG_COLOR_PRE LOG_COLOR_GREEN   ///< diagnostics color

#ifdef LOGMOD_COLOR
#define LOG_PREFACE LOG_COLOR_PRE LOGMOD_COLOR LOG_PREFACE_TEXT LOG_COLOR_POST
                                            ///< loging preface string
#else
#define LOG_PREFACE LOG_PREFACE_TEXT        ///< loging preface string
#endif

#define LOG_PREFACE_PLAIN LOG_PREFACE_TEXT  ///< loging preface with no color


//-----------------------------------------------------------------------------
// Logging Diagnostics and Error Argument Macros
//-----------------------------------------------------------------------------

/*!
 * \brief Standard diagnostic logging output arguments with compiled color.
 * \param level Loggging level.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_DIAG(level, fmt, ...) \
  LOG_PREFACE LOG_COLOR_DIAG "Diag%d: %s[%d] " LOG_COLOR_POST fmt, \
    (level-1), __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard diagnostic logging output arguments in plain text.
 * \param level Loggging level.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_DIAG_PLAIN(level, fmt, ...) \
  LOG_PREFACE_PLAIN "Diag%d: %s[%d] " fmt, \
    (level-1), __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard warning logging output arguments with compiled color.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_WARN(fmt, ...) \
  LOG_PREFACE LOG_COLOR_WARN "Warning: %s[%d] " LOG_COLOR_POST fmt, \
    __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard warning logging output arguments in plain text.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_WARN_PLAIN(fmt, ...) \
  LOG_PREFACE_PLAIN "Warning: %s[%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard error logging output arguments with compiled color.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_ERR(fmt, ...) \
  LOG_PREFACE LOG_COLOR_ERROR "Error: %s[%d] " LOG_COLOR_POST fmt, \
    __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard error logging output arguments in plain text.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_ERR_PLAIN(fmt, ...) \
  LOG_PREFACE_PLAIN "Error: %s[%d] " fmt, __FILE__, __LINE__, ##__VA_ARGS__

/*!
 * \brief Standard system error logging output arguments with compiled color.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_SYSERR(fmt, ...) \
  LOG_PREFACE LOG_COLOR_ERROR "Error: %s[%d] " LOG_COLOR_POST "%s(errno=%d): " \
    fmt, __FILE__, __LINE__, strerror(errno), errno, ##__VA_ARGS__

/*!
 * \brief Standard system error logging output arguments with compiled color.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGARGS_SYSERR_PLAIN(fmt, ...) \
  LOG_PREFACE_PLAIN "Error: %s[%d] " "%s(errno=%d): " fmt, \
    __FILE__, __LINE__, strerror(errno), errno, ##__VA_ARGS__


//-----------------------------------------------------------------------------
// Logging Diagnostics and Error Macros
//   See http://gcc.gnu.org/onlinedocs/cpp/Variadic-Macros.html#Variadic-Macros
//-----------------------------------------------------------------------------
#ifdef LOG

/*!
 * \brief Standard Diagnostic logging.
 * \param level Loggging level.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG(level, fmt, ...) \
  do \
  { \
    if( LOGABLE(level) ) \
    { \
      if( LOG_IN_COLOR() ) \
      { \
        LOGGER(LOGARGS_DIAG(level, fmt), ##__VA_ARGS__); \
      } \
      else \
      { \
        LOGGER(LOGARGS_DIAG_PLAIN(level, fmt), ##__VA_ARGS__); \
      } \
    } \
  } while(0)
                                                                  
/*!
 * \brief Standard User Diagnostic logging.
 * \param level Loggging user level.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 * \sa LOGDIAG()
 */
#define LOGUSER(level, fmt, ...) \
  LOGDIAG(level+LOG_LEVEL_DIAG3+1, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 5 logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG5(fmt, ...) LOGDIAG(LOG_LEVEL_DIAG5, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 4 logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG4(fmt, ...) LOGDIAG(LOG_LEVEL_DIAG4, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 3 logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG3(fmt, ...) LOGDIAG(LOG_LEVEL_DIAG3, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 2 logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG2(fmt, ...) LOGDIAG(LOG_LEVEL_DIAG2, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 1 logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGDIAG1(fmt, ...) LOGDIAG(LOG_LEVEL_DIAG1, fmt, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic function call tracing.
 * \param level Loggging level.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAGCALL(level, ...) \
  do \
  { \
    if( LOGABLE(level) ) \
    { \
      if( LOG_IN_COLOR() ) \
      { \
        LOGGER_CALL(LOG_PREFACE, level, __FILE__, __LINE__, \
          LOGFUNCNAME, ##__VA_ARGS__, NULL, 0); \
      } \
      else \
      { \
        LOGGER_CALL(LOG_PREFACE_PLAIN, level, __FILE__, __LINE__, \
          LOGFUNCNAME, ##__VA_ARGS__, NULL, 0); \
      } \
    } \
  } while(0)

/*!
 * \brief Standard Diagnostic Level 5 function call tracing.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAG5CALL(...) LOGDIAGCALL(LOG_LEVEL_DIAG5, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 4 function call tracing.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAG4CALL(...) LOGDIAGCALL(LOG_LEVEL_DIAG4, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 3 function call tracing.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAG3CALL(...) LOGDIAGCALL(LOG_LEVEL_DIAG3, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 2 function call tracing.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAG2CALL(...)  LOGDIAGCALL(LOG_LEVEL_DIAG2, ##__VA_ARGS__)

/*!
 * \brief Standard Diagnostic Level 1 function call tracing.
 * \param ...   Optional argument_format,argument pairs.
 */
#define LOGDIAG1CALL(...) LOGDIAGCALL(LOG_LEVEL_DIAG1, ##__VA_ARGS__)

/*!
 * \brief Standard Warning logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGWARN(fmt, ...) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_ERROR) ) \
    { \
      if( LOG_IN_COLOR() ) \
      { \
        LOGGER(LOGARGS_WARN(fmt), ##__VA_ARGS__); \
      } \
      else \
      { \
        LOGGER(LOGARGS_WARN_PLAIN(fmt), ##__VA_ARGS__); \
      } \
    } \
  } while(0)

/*!
 * \brief Standard Error logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGERROR(fmt, ...) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_ERROR) ) \
    { \
      if( LOG_IN_COLOR() ) \
      { \
        LOGGER(LOGARGS_ERR(fmt), ##__VA_ARGS__); \
      } \
      else \
      { \
        LOGGER(LOGARGS_ERR_PLAIN(fmt), ##__VA_ARGS__); \
      } \
    } \
  } while(0)

/*!
 * \brief Standard System Error logging.
 * \param fmt   Optional specifics format string.
 * \param ...   Optional specific variable arguments for fmt.
 */
#define LOGSYSERROR(fmt, ...) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_ERROR) ) \
    { \
      int errsave = errno; \
      if( LOG_IN_COLOR() ) \
      { \
        LOGGER(LOGARGS_SYSERR(fmt), ##__VA_ARGS__); \
      } \
      else \
      { \
        LOGGER(LOGARGS_SYSERR_PLAIN(fmt), ##__VA_ARGS__); \
      } \
      errno = errsave; \
    } \
  } while(0)                                                                         

#else // do not compile logging

#define LOGDIAG(...)
#define LOGUSER(...)
#define LOGDIAG5(...)
#define LOGDIAG4(...)
#define LOGDIAG3(...)
#define LOGDIAG2(...)
#define LOGDIAG1(...)
#define LOGDIAG5CALL(...)
#define LOGDIAG4CALL(...)
#define LOGDIAG3CALL(...)
#define LOGDIAG2CALL(...)
#define LOGDIAG1CALL(...)
#define LOGWARN(...)
#define LOGERROR(...)
#define LOGSYSERROR(...)

#endif // LOG


//-----------------------------------------------------------------------------
// Helper Formatters
//-----------------------------------------------------------------------------

//
// Helper formaters
//
#define _VARFMT(var, fmt) #var "=" fmt            ///< log Variable Format.
#define _TF(b)            ((b)? "true": "false")  ///< boolean to string format
#define _CHARFMT(var)     (isprint(var)? #var "='%c'": #var "=0x%02x")
                                                  ///< char format string

/*!
 * \brief Family of 2-tuple ("<varname>=<fmt>", \<var\>) variable logging
 * formatters.
 *
 *  The \p _T*() macros are used to create a format string, variable pair of the
 *  form: "<varname>=<fmt>",\<var\>
 *
 * \param var Variable identifier.
 * \param fmt Format string to print variable.
 *
 * \note Intended use are calls to the LOGDIAGxCALL() macros,
 * although can be used anywhere, although only once per each LOGDIAGx(),
 * LOGERROR(), or LOGSYSERROR() call. 
 *
 * \sa _VARFMT() for a more general varaible formatter.
 */
#define _TFMT(var, fmt) #var "=" fmt, var       ///< raw Format,Variable Pair
#define _TSTR(var)      _TFMT(var, "\"%s\"")    ///< string variable
#define _TLONG(var)     _TFMT(var, "%ld")       ///< long int (decimal)
#define _TULONG(var)    _TFMT(var, "%lu")       ///< unsigned long int (decimal)
#define _THEX(var)      _TFMT(var, "0x%x")      ///< int (hex)
#define _TINT(var)      _TFMT(var, "%d")        ///< int (decimal)
#define _TUINT(var)     _TFMT(var, "%u")        ///< unsigned int (decimal)
#define _TSHORT(var)    _TFMT(var, "%hd")       ///< short int (decimal)
#define _TUSHORT(var)   _TFMT(var, "%hu")       ///< unsigned short (decimal)
#define _TFLOAT(var)    _TFMT(var, "%f")        ///< float
#define _TPTR(var)      _TFMT(var, "%p")        ///< pointer
#define _TBOOL(var)     _VARFMT(var, "%s"), _TF(var)  ///< boolean
#define _TCHAR(var)     _CHARFMT(var), var      ///< character
#define _TVEC(var, i, fmt)  #var "[%d]=" fmt, i, var[i] ///< vector variable
#define _TVEC_UINT(var, i)  _TVEC(var, i, "%u") ///< unsigned vector variable


/*!
 * function name w/ parens
 */
#define LFF            "%s()", LOGFUNCNAME

/*!
 * \brief Log formatter for function with arguments.
 *
 *  Format: function(var=val, ...)
 *
 * \param fmt Format string to print function arguments.
 * \param ... Arguments to fmt string.
 */
#define LFF_ARGS(fmt, ...)  \
  "%s(" fmt "%s", LOGFUNCNAME, ##__VA_ARGS__, ")"

/*!
 * \brief Log formatter for function with a bad argument.
 *
 *  Format: function(var=val): errmsg
 *
 * \param var     Variable identifier.
 * \param fmt     Format string to print function argument.
 * \param postfmt Format string to print after function string.
 * \param ...     Arguments to postfmt string 
 */
#define LFF_BADARG(var, fmt, postfmt, ...)  \
  "%s(" #var "=" fmt "): " postfmt, LOGFUNCNAME, var, ##__VA_ARGS__

/*!
 * \brief Log formatter for function with a bad vector argument.
 *
 *  Format: function(var[i]=val): errmsg
 *
 * \param var     Vector variable identifier.
 * \param i       Variable index.
 * \param fmt     Format string to print function argument.
 * \param postfmt Format string to print after function string.
 * \param ...     Arguments to postfmt string 
 */
#define LFF_BADARG_VEC(var, i, fmt, postfmt, ...)  \
  "%s(" #var "[%d]=" fmt "): " postfmt, LOGFUNCNAME, i, var[i], ##__VA_ARGS__


//-----------------------------------------------------------------------------
// Data Validation Helpers
//-----------------------------------------------------------------------------

/*!
 * \brief Checks validity of pointer.
 *
 * If null then log error and return from the embedding function with given
 * (optional) error return code.
 *
 *  \param p    Pointer that should not be NULL.
 *  \param ...  Error return code (optional).
 */
#define CHKPTR(p, ...) \
  do \
  {  \
    if( (p) == NULL ) \
    {  \
      LOGERROR(_TPTR(p)); return __VA_ARGS__; \
    } \
  } while(0)

/*!
 * \brief Checks validity of value against the given validation expression.
 *
 * The \p CHKEXPR*() macros are used validate a value against an expression.
 * If the expression evaluates to false, then log an error message 
 * and return from the embedding functin with the given (optional)
 * error return code.
 *
 * \param val    Value identifier to check.
 * \param expr   Validation expression to evaluate.
 * \param valfmt Value format string to use on error.
 * \param ...    Error return code (optional).
 */
#define CHKEXPR(val, expr, valfmt, ...) \
  do \
  {  \
    if( !(expr) ) \
    {  \
      LOGERROR(_VARFMT(val, valfmt) ": failed check: %s", val, #expr); \
      return __VA_ARGS__; \
    } \
  } while(0)
                                                                                
//
// Family of macro data validators.
//

/*! \brief check integer */
#define CHKEXPR_INT(val, expr, ...)    CHKEXPR(val, expr, "%d", ##__VA_ARGS__)

/*! \brief check string */
#define CHKEXPR_STR(val, expr, ...)    CHKEXPR(val, expr, "'%s'", ##__VA_ARGS__)

/*! \brief check long integer */
#define CHKEXPR_LONG(val, expr, ...)   CHKEXPR(val, expr, "%ld", ##__VA_ARGS__)

/*! \brief check unsigned long integer */
#define CHKEXPR_ULONG(val, expr, ...)  CHKEXPR(val, expr, "%lu", ##__VA_ARGS__)

/*! \brief check integer hex */
#define CHKEXPR_HEX(val, expr, ...)    CHKEXPR(val, expr, "0x%x", ##__VA_ARGS__)

/*! \brief check unsigned integer */
#define CHKEXPR_UINT(val, expr, ...)   CHKEXPR(val, expr, "%u", ##__VA_ARGS__)

/*! \brief check short integer */
#define CHKEXPR_SHORT(val, expr, ...)  CHKEXPR(val, expr, "%hd",## __VA_ARGS__)

/*! \brief check unsigned short integer */
#define CHKEXPR_USHORT(val, expr, ...) CHKEXPR(val, expr, "%hu", ##__VA_ARGS__)

/*! \brief check pointer */
#define CHKEXPR_PTR(val, expr, ...)    CHKEXPR(val, expr, "%p", ##__VA_ARGS__)


//
// Data 
//

// Log threshold level (log iff level <= threshold)
#ifdef LOG
extern int    LOG_VAR_THRESHOLD;  ///< current logging threshold variable
extern bool_t LOG_VAR_COLOR_EN;   ///< color logging is [not] enabled
#endif // LOG


//-----------------------------------------------------------------------------
// Namespace C Definitions
//-----------------------------------------------------------------------------

/*!
 * \brief If logging is defined is a different logging name space, then
 * LOGNS_DEF must be included into the application or library.
 */
#if defined(LOGNS_DEF)
  #if defined(LOG)
    #define LOGNS_DEF_BLOCK \
extern "C" { \
  int LOG_VAR_THRESHOLD = LOG_LEVEL_OFF; \
  int LOG_SET_THRESHOLD(int nLevel) \
  { \
    LOG_VAR_THRESHOLD = nLevel; \
    if( LOG_VAR_THRESHOLD >= LOG_LEVEL_DIAG1 ) \
    { \
      LOGGER(LOGARGS_DIAG(2, "Logging level set to %d", LOG_VAR_THRESHOLD)); \
    } \
    return LOG_VAR_THRESHOLD; \
  } \
  int LOG_GET_THRESHOLD() \
  { \
    return LOG_VAR_THRESHOLD; \
  } \
}
  #else
    #define LOGNS_DEF_BLOCK \
extern "C" { \
  int LOG_VAR_THRESHOLD = LOG_LEVEL_NA; \
  int LOG_SET_THRESHOLD(int nLevel)
  { \
    return LOG_VAR_THRESHOLD; \
  } \
  int LOG_GET_THRESHOLD() \
  { \
    return LOG_VAR_THRESHOLD; \
  } \
}
  #endif
#else
  #define LOGNS_DEF_BLOCK
#endif


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

//
// <namespace>LogSetThresholdLevel()
//  Set new logging threshold level. Always compile for blind sets.
//
extern int LOG_SET_THRESHOLD(int nLevel);

//
// <namespace>LogGetThresholdLevel()
//  Get current logging threshold level. Always compile to determine logging
//  availability.
//
extern int LOG_GET_THRESHOLD();

//
// <namespace>LogSetColorEnable()
//  Enable/disable color logging.
//
extern void LOG_SET_COLOR_ENABLE(bool_t enable);

//
// <namespace>LogSetTimestampEnable()
//  Enable/disable logging timestamps.
//
extern void LOG_SET_TIMESTAMP_ENABLE(bool_t enable);

//
// <namespace>LogSetLogFile()
//  Set logging ouput to file. Always compile for blind sets.
//
extern int LOG_SET_LOGFILE(const char *sLogFileName);

//
// <namespace>LogGetLogFile()
//  Get logging ouput to file name.
//
#ifdef LOG
extern const char *LOG_GET_LOGFILE();
#endif // LOG

//
// <namespace>LogAttachLogFp()
//  Attach file pointer as logging ouput stream.
//
#ifdef LOG
extern void LOG_ATTACH_LOGFP(FILE *fp, const char *sFpFileName);
#endif // LOG

//
// <namespace>LogGetLogFp()
//  Get current logging ouput stream file pointer.
//
#ifdef LOG
extern FILE *LOG_GET_LOGFP();
#endif // LOG

//
// <namespace>LogPrintf()
//    Print loggging diagnostics, debug, error, and system error messages to
//    logging output stream.
//
//  Arguments:
//    sFmt        - format string
//    ...         - variable format arguments
//
//  Return Values:
//    None
//
#ifdef LOG
extern void LOGGER(const char *sFmt, ...);
#endif // LOG

//
// <namespace>LogCallPrintf()
//    Print function call tracing to logging output stream.
//
//  Arguments:
//    sFmt        - format string
//    ...         - format-arguments pairs
//
//  Return Values:
//    None
//
#ifdef LOG
extern void LOGGER_CALL(const char *sPreface, int nLevel,
                        const char *sFile, int nLine,
                        const char *sFuncName, ...);
#endif // LOG

C_DECLS_END


#endif // _LOG_H
