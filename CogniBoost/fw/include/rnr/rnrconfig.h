////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Header
//
// File:      rnrconfig.h
//
/*! \file
 *
 * $LastChangedDate: 2010-04-01 13:29:03 -0600 (Thu, 01 Apr 2010) $
 * $Rev: 322 $
 *
 * \brief RoadNarrows Robotics common configuration file.
 *
 * The rnrconfig.h file provides a somewhat unified method to support
 * various RoadNarrows supported processor architectures and host platforms.
 *
 * Common typedefs, sizes, return codes, and useful utility declarations are
 * declared.
 *
 * \par Supported CPU archetectures:
 * \param i386 Intel i386+ family of 32-bit processors.
 * \param x86_64 AMD family of 64-bit processors.
 * \param armpxa XScale PXA Arm processors.
 *
 * \par Supported platforms: 
 * \param windows           Requires posix libraries.
 *                          (see http://www.cygwin.com/)
 * \param "posix linux"     Tested on Ubuntu and Fedora Cores 5 &amp; 6. 
 * \param "familiar linux"  Tested on K-Team's KoreBot embedded 2.4 linux SBC.
 * \param "angstrom linux"  Tested on K-Team's KoreBot embedded 2.6 linux SBC.
 * \param os-x              Mac OS-X
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2007-2017. RoadNarrows LLC.\n
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

#ifndef _RNRCONFIG_H
#define _RNRCONFIG_H

#include <stddef.h> //BHW size_t
#include <stdint.h> //BHW uint8_t etc

//
// C declarations in C++
//
# define C_DECLS_BEGIN                 ///< C declaration block begin in C
# define C_DECLS_END                   ///< C declaration block end in C
# define C_DECLS                       ///< C declaration line in C
# define C_EXTERN      extern          ///< C extern declaration in C

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Platforms 
// Note: Need to expand this. Posix needs to be factored in plus
// are there other platforms.
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

#define OSNAME  "Atmel"     ///< OS simple name

//
// Dynamic Library C Export Declarators
// Note: Needs some tweaking.
//
// #define DLL_EXPORT                   ///< declarator at definition 
// #define DLL_EXPORT_DECL  C_DECLS     ///< declarator at declaration 

//
// Header defined inline function declaration
//
#define INLINE_IN_H C_DECLS static inline ///< inline C funtion in C header

//
// Basic types (ifdef by architecture to fit into these types if needed)
//
typedef uint8_t       byte_t;       ///< 8-bit byte
typedef int8_t        schar_t;      ///< 8-bit signed integer
typedef uint16_t      ushort_t;     ///< 16-bit unsigned integer
typedef int16_t       sshort_t;     ///< 16-bit signed integer
typedef uint32_t      uint_t;       ///< 32-bit unsigned integer
typedef int32_t       sint_t;       ///< 32-bit signed integer
typedef uint32_t      ulong_t;      ///< 32/64-bit unsigned long
typedef int32_t       slong_t;      ///< 32/64-bit signed long
typedef uint64_t      ulonglong_t;  ///< 64-bit unsigned long long integer
typedef int64_t       slonglong_t;  ///< 64-bit signed long long integer
typedef uint8_t       bool_t;       ///< "boolean" T/F
 
#ifndef __cplusplus
#ifndef true
#define true            (bool_t)1   ///< (if only it were) true
#endif
#ifndef false
#define false           (bool_t)0  ///< (never to thy self be) false
#endif
#endif // __cplusplus

#ifndef NULL
#define NULL ((void *)0)          ///< null pointer
#endif

//
// Sizes
//

#ifndef MAX_LINE
#define MAX_LINE 2048
///< Maximum bytes of input line (including NULL)
#endif

#ifndef MAX_PATH
#define MAX_PATH 1024
///< Maximum bytes of a fully qualified, absolute file name (including NULL)
#endif

#ifndef MAX_BASENAME
#define MAX_BASENAME 256
///< Maximum bytes of a file basename of path component (including NULL)
#endif

#ifndef MAX_SEARCH_PATH
#define MAX_SEARCH_PATH   4096
///< Maximum bytes of a PATH_SEP separated search path string (including NULL)
#endif

//
// File
//
#ifndef FILE_SEP
#if defined(__windows__)
#define PATH_SEP_CHAR ';'   ///< path separator (char version)
#define PATH_SEP_STR  ";"   ///< path separator (string version)
#define DIR_SEP_CHAR  '\\'  ///< directory component separator (char version)
#define DIR_SEP_STR   "\\"  ///< directory component separator (string version)
#else
#define PATH_SEP_CHAR ':'   ///< path separator (char version)
#define PATH_SEP_STR  ":"   ///< path separator (string version)
#define DIR_SEP_CHAR  '/'   ///< directory component separator (char version)
#define DIR_SEP_STR   "/"   ///< directory component separator (string version)
#endif // __windows__
#define FILE_SEP            ///< file separator group define 
#endif // FILE_SEP

//
// Current function name (keep literal)
// Note: Might be protability issues with this macro.
//
#define THISFUNCNAME    __func__           ///< function name

//
// Array Macros
//

#ifndef arraysize
/*!
 * \brief array size, i.e. number of array entries
 * \param array the array to size
 */
#define arraysize(array)  (sizeof(array)/sizeof(array[0]))
#endif // arraysize

//
// Structure Member Macros
//

#ifndef member_defines

/*!
 * \brief structure member offset (bytes)
 * \param structname  structure name or typedef
 * \param member      structure member name
 */
#define memberoffset(structname, member)  (size_t)(&(((structname *)0)->member))

/*!
 * \brief member lvalue
 * \param ctype   lvalue type
 * \param ptr     pointer type
 * \param offset  byte offset from pointer
 */
#define memberlvalue(ctype, ptr, offset)  *((ctype *)(ptr + offset))

/*!
 * \brief member assignment
 * \param ctype   lvalue type
 * \param ptr     pointer type
 * \param offset  byte offset from pointer
 * \param val     value to assign lvalue.
 */
#define memberassign(ctype, ptr, offset, val) \
  memberlvalue(ctype, ptr, offset) = (ctype)val

#define member_defines  ///< member group defines

#endif // member_defines

//
// Simple Exit and Return codes
//
#ifndef OK
#define OK  0           ///< Okay
#endif

// General error exit code
#ifndef EC_ERROR
#define EC_ERROR  4     ///< general error exit code
#endif

// Bad command-line option exit code
#ifndef EC_BAD_OPT
#define EC_BAD_OPT  2   ///< bad command line option exit code
#endif

// General function error return code
#ifndef RC_ERROR
#define RC_ERROR  (-1)  ///< common function error return code
#endif

#endif // _RNRCONFIG_H
