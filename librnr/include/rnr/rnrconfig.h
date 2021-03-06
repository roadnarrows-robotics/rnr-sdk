////////////////////////////////////////////////////////////////////////////////
/*! \file
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
 * \param "posix linux"     Tested on Ubuntu and Odroid Arm
 * \param "familiar linux"  Tested on K-Team's KoreBot embedded 2.4 linux SBC.
 * \param "angstrom linux"  Tested on K-Team's KoreBot embedded 2.6 linux SBC.
 * \param os-x              Mac OS-X
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/rnrconfig.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2007-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNRCONFIG_H
#define _RNRCONFIG_H

#include <sys/param.h>
#include <sys/types.h>

//
// Compiling for C++/C
//
#if defined(__CPLUSPLUS__) || defined(__cplusplus)
# ifndef __cplusplus 
#  define __cplusplus   ///< always use this C++ convention for tests
# endif
#else
# undef __cplusplus
#endif

//
// C declarations in C++
//
#ifdef __cplusplus
# define C_DECLS_BEGIN extern "C" {    ///< C declaration block begin in C++
# define C_DECLS_END   }               ///< C declaration block end in C++
# define C_DECLS       extern "C"      ///< C declaration line in C++
# define C_EXTERN      extern "C"      ///< C extern declaration in C++
#else // C
# define C_DECLS_BEGIN                 ///< C declaration block begin in C
# define C_DECLS_END                   ///< C declaration block end in C
# define C_DECLS                       ///< C declaration line in C
# define C_EXTERN      extern          ///< C extern declaration in C
#endif // __cplusplus


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Platforms 
// Note: Need to expand this. Posix needs to be factored in plus
// are there other platforms.
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

//
// Linux Platforms
//
#if defined(__linux__) || defined(ARCH_i386) || defined(ARCH_x86_64) || \
    defined(ARCH_armpxa) || defined(ARCH_armang)

#ifndef __linux__
#define __linux__           ///< OS key define
#endif // __linux__

#ifndef OSNAME
#define OSNAME  "linux"     ///< OS simple name
#endif // OSNAME

//
// OS-X Platforms
//
#elif defined(__osx__) || defined(ARCH_osx)

#ifndef __osx__
#define __osx__             ///< OS key define
#endif // __osx__

#ifndef OSNAME
#define OSNAME  "osx"       ///< OS simple name
#endif // OSNAME

//
// Windows Cygwin Platforms
//
#elif defined(__CYGWIN__) || defined(ARCH_cygwin)

#ifndef __cygwin__
#define __cygwin__          ///< OS key define
#endif // __cygwin__

#ifndef OSNAME
#define OSNAME  "cygwin"    ///< OS simple name
#endif // OSNAME

//
// Windows Native Platforms
//
#elif defined(__windows__) || defined(ARCH_win)

#ifndef __windows__
#define __windows__         ///< OS key define
#endif // __windows__

#ifndef OSNAME
#define OSNAME  "windows"   ///< OS simple name
#endif // OSNAME

#endif // Platforms

//
// Dynamic Library C Export Declarators
// Note: Needs some tweaking.
//
#ifdef __windows__
#define DLL_EXPORT       __declspec(dllexport)    ///< declarator at definition
#define DLL_EXPORT_DECL  C_DECLS __declspec(dllexport) ///< at declaration
#else
#define DLL_EXPORT                   ///< declarator at definition 
#define DLL_EXPORT_DECL  C_DECLS     ///< declarator at declaration 
#endif // __windows__

//
// Header defined inline function declaration
//
#ifdef __cplusplus
#define INLINE_IN_H C_DECLS inline        ///< inline C funtion in C++ header
#else
#define INLINE_IN_H C_DECLS static inline ///< inline C funtion in C header
#endif // __cplusplus

//
// Basic types (ifdef by architecture to fit into these types if needed)
//

// explicit
typedef __int8_t            s8_t;         ///< 8-bit signed integer
typedef __uint8_t           u8_t;         ///< 8-bit unsigned integer
typedef __int16_t           s16_t;        ///< 16-bit signed integer
typedef __uint16_t          u16_t;        ///< 16-bit unsigned integer
typedef __int32_t           s32_t;        ///< 32-bit signed integer
typedef __uint32_t          u32_t;        ///< 32-bit unsigned integer
typedef __int64_t           s64_t;        ///< 64-bit signed integer
typedef __uint64_t          u64_t;        ///< 64-bit unsigned integer
typedef float               f32_t;        ///< 32-bit floating-point number
typedef double              f64_t;        ///< 64-bit floating-point number

// symbolic
typedef u8_t                byte_t;       ///< 8-bit byte
typedef s8_t                schar_t;      ///< 8-bit signed integer
typedef u16_t               ushort_t;     ///< 16-bit unsigned integer
typedef s16_t               sshort_t;     ///< 16-bit signed integer
typedef u32_t               uint_t;       ///< 32-bit unsigned integer
typedef s32_t               sint_t;       ///< 32-bit signed integer
typedef unsigned long       ulong_t;      ///< 32/64-bit unsigned long
typedef signed long         slong_t;      ///< 32/64-bit signed long
typedef u64_t               ulonglong_t;  ///< 64-bit unsigned long long integer
typedef s64_t               slonglong_t;  ///< 64-bit signed long long integer
typedef int                 bool_t;       ///< "boolean" T/F
 
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


//.............................................................................
// Compiler Pragmas
//.............................................................................

/*!
 * \brief CPP pragma macro.
 *
 * C99 introduces the _Pragma operator. Calling _Pragma("a b c") will produce
 * \#pragma a b c
 *
 * \param x   The pragma.
 */
#if defined(_Pragma)
# define DO_PRAGMA(x)       _Pragma(#x)
#else
# define DO_PRAGMA(x)
#endif

/*!
 * \brief CPP diagnostics pragma macro.
 *
 * \note The current Angstrom Arm GCC cross-compiler does not support
 * diagnostics very well.
 *
 * \param x   Unquoted front part of pragma
 * \param y   Qouted back part of pragma.
 */
#ifndef ARCH_armang
#define PRAGMA_DIAG(x, y)   DO_PRAGMA(x #y)
#else
#define PRAGMA_DIAG(x, y)
#endif

/*!
 * \brief Disable compiler warnings on the diagnostics filter.
 *
 * \sa "gcc --help=warnings" for list of filters.
 */
#define PRAGMA_IGNORED(filter) PRAGMA_DIAG(GCC diagnostic ignored, -W ## filter)

/*!
 * \brief Enable compiler warnings on the diagnostics filter.
 *
 * \sa "gcc --help=warnings" for list of filters.
 */
#define PRAGMA_WARNING(filter) PRAGMA_DIAG(GCC diagnostic warning, -W ## filter)

/*!
 * \brief Treat the diagnostics filter as a compiler error
 *
 * \sa "gcc --help=warnings" for list of filters.
 */
#define PRAGMA_ERROR(filter) PRAGMA_DIAG(GCC diagnostic error, -W ## filter)


#endif // _RNRCONFIG_H
