////////////////////////////////////////////////////////////////////////////////
//
// Package:   netmsgs
//
// Library:   libnetmsgs
//
// File:      nmLibInternal.c
//
/*! \file
 *
 * $LastChangedDate: 2010-02-12 14:27:57 -0700 (Fri, 12 Feb 2010) $
 * $Rev: 247 $
 *
 * \brief Internal intra-library definitions
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2009-2017. RoadNarrows LLC.\n
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
#include <stdlib.h>
#include <stdarg.h>
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/netmsgs.h"

#include "nmLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Set field value.
 *
 * \param _this This field's specific information.
 * \param pv    Pointer to field value.
 * \param T     Field C type.
 */
#define _NMSET_VAL(_this, pv, T) \
  (T)(_this.m_bits & NMBITS_HAS_CONST? (T)(_this.m_valConst): *((T*)pv))

/*!
 * \brief Check value is \h_ge minimum.
 *
 * \note Out-of-range violation results is calling function return.
 *
 * \param fname This field's string name.
 * \param _this This field's specific information.
 * \param v     Field value.
 * \param T     Field C type.
 * \param fmt   Format string on error.
 */
#define _NMCHK_MIN(fname, _this, v, T, fmt) \
  do \
  { \
    if( _this.m_bits & NMBITS_HAS_MIN ) \
    { \
      if( (v) < (T)(_this.m_valMin) ) \
      { \
        NMLIB_RAISE_ERROR(NM_ECODE_RANGE, "%s val=" fmt " < min=" fmt, \
            fname, v, (T)(_this.m_valMin)); \
      } \
    } \
  } while(0)

/*!
 * \brief Check value is \h_le maximum.
 *
 * \note Out-of-range violation results is calling function return.
 *
 * \param fname This field's string name.
 * \param _this This field's specific information.
 * \param v     Field value.
 * \param T     Field C type.
 * \param fmt   Format string on error.
 */
#define _NMCHK_MAX(fname, _this, v, T, fmt) \
  do \
  { \
    if( _this.m_bits & NMBITS_HAS_MAX ) \
    { \
      if( (v) > (T)(_this.m_valMax) ) \
      { \
        NMLIB_RAISE_ERROR(NM_ECODE_RANGE, "%s val=" fmt " > max=" fmt, \
            fname, v, (T)(_this.m_valMax)); \
      } \
    } \
  } while(0)


// ---------------------------------------------------------------------------
// Internal Public Interface 
// ---------------------------------------------------------------------------

const int NMHashOffset  = (int)NMFTypeBool;   ///< hash base offset
const int NMHashNoIdx   = 0xff;               ///< hash no index value

/*!
 * Field Type Hash Table.
 *
 * Maps field type to zero base index.
 *
 * \note The table is hand-crafted to optimize access speed.
 */
const byte_t NMHashTbl[] =
{
  //   [0]  ?     @     A     B     C     D     E     F
            0,    0xff, 0xff, 1,    0xff, 0xff, 0xff, 2,

  //   [8]  G     H     I     J     K     L     M     N
            0xff, 3,    4,    0xff, 0xff, 0xff, 0xff, 0xff,

  //  [16]  O     P     Q     R     S     T     U     V
            0xff, 5,    6,    0xff, 0xff, 0xff, 0xff, 0xff,

  //  [24]  W     X     Y     Z     [     \     ]     ^
            0xff, 0xff, 0xff, 0xff, 7,    0xff, 0xff, 0xff,

  //  [32]  _     `     a     b     c     d     e     f
            0xff, 0xff, 0xff, 8,    9,    0xff, 0xff, 10,

  //  [40]  g     h     i     j     k     l     m     n
            0xff, 11,   12,   0xff, 0xff, 0xff, 0xff, 0xff,

  //  [48]  o     p     q     r     s     t     u     v
            0xff, 13,   14,   0xff, 15,   0xff, 0xff, 0xff,

  //  [56]  w     x     y     z     {
            0xff, 16,   0xff, 0xff, 17
};

/*! number of hash entries */
const size_t  NMHashNumEntries = arraysize(NMHashTbl);

/*!
 * \brief Helper function to set an unsigned 8-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetU8(const NMFieldDef_T *pFieldDef, void *pValIn, byte_t *pValOut)
{
  byte_t      val;

  // set value, override if is constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u8, pValIn, byte_t);
  //val = (pFieldDef->m_this.m_u8.m_bits & NMBITS_HAS_CONST? 
  //    (byte_t)(pFieldDef->m_this.m_u8.m_valConst):
  //    *((byte_t*)pValIn))

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u8, val, byte_t, "%hhu");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u8, val, byte_t, "%hhu");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a signed 8-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetS8(const NMFieldDef_T *pFieldDef, void *pValIn, signed char *pValOut)
{
  signed char val;

  // set value, override if is constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u8, pValIn, signed char);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u8,
      val, signed char, "%hhd");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u8,
      val, signed char, "%hhd");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set an unsigned 16-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetU16(const NMFieldDef_T *pFieldDef, void *pValIn, ushort_t *pValOut)
{
  ushort_t    val;

  // set value, override if is constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u16, pValIn, ushort_t);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u16,
      val, ushort_t, "%hu");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u16,
      val, ushort_t, "%hu");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a signed 16-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetS16(const NMFieldDef_T *pFieldDef, void *pValIn, short *pValOut)
{
  short       val;

  // set value, override if is constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u16, pValIn, short);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u16, val, short, "%hd");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u16, val, short, "%hd");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set an unsigned 32-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetU32(const NMFieldDef_T *pFieldDef, void *pValIn, uint_t *pValOut)
{
  uint_t      val;

  // set value, override if is constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u32, pValIn, uint_t);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u32, val, uint_t, "%u");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u32, val, uint_t, "%u");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a signed 32-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetS32(const NMFieldDef_T *pFieldDef, void *pValIn, int *pValOut)
{
  int         val;

  // set value, override if constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u32, pValIn, int);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u32, val, int, "%d");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u32, val, int, "%d");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set an unsigned 64-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetU64(const NMFieldDef_T *pFieldDef,
             void               *pValIn,
             ulonglong_t        *pValOut)
{
  ulonglong_t  val;

  // set value, override if constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u64, pValIn, ulonglong_t);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u64,
      val, ulonglong_t, "%llu");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u64,
      val, ulonglong_t, "%llu");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a signed 64-bit integer.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetS64(const NMFieldDef_T *pFieldDef, void *pValIn, long long *pValOut)
{
  long long   val;

  // set value, override if constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_u64, pValIn, long long);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_u64,
      val, long long, "%hhu");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_u64,
      val, long long, "%hhu");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a 32-bit floating point number.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetF32(const NMFieldDef_T *pFieldDef, void *pValIn, float *pValOut)
{
  float val;

  // set value, override if constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_f32, pValIn, float);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_f32, val, float, "%f");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_f32, val, float, "%f");

  // set
  *pValOut = val;

  return NM_OK;
}

/*!
 * \brief Helper function to set a 64-bit floating point number.
 *
 * \param pFieldDef     Pointer to message field definition.
 * \param [in] pValIn   Pointer to field value.
 * \param [out] pValOut Pointer to constrained, assigned field value.
 *
 * \return
 * On success, returns \ref NM_OK.\n
 * On error, the appropriate negated
 * \ref man_libnetmsgs_ecodes "netmsgs error code" (\h_lt 0) is returned.
 */
int nmSetF64(const NMFieldDef_T *pFieldDef, void *pValIn, double *pValOut)
{
  double  val;

  // set value, override if constant value
  val = _NMSET_VAL(pFieldDef->m_this.m_f64, pValIn, double);

  // check against minimum limits
  _NMCHK_MIN(pFieldDef->m_sFName, pFieldDef->m_this.m_f64, val, double, "%f");

  // check against maximum limits
  _NMCHK_MAX(pFieldDef->m_sFName, pFieldDef->m_this.m_f64, val, double, "%f");

  // set
  *pValOut = val;

  return NM_OK;
}

#ifdef NMLIB_DEBUG
/*!
 * \brief Trace packing/unpacking of a message field.
 *
 * \param pFieldDef   Pointer to field definition.
 * \param buf         Packed/unpacked buffer.
 * \param uCount      Number of bytes in buffer to trace.
 * \param uDepth      Depth of field in recursive definition.
 * \param sFmt        Field representation format string.
 * \param ...         Field representation variable arguments.
 */
void nmTraceField(const NMFieldDef_T *pFieldDef,
                  byte_t              buf[],
                  size_t              uCount,
                  uint_t              uDepth,
                  const char          *sFmt,
                  ...)
{
  va_list ap;
  char    bufRepr[256];
  uint_t  uIndent;
  int     n;

  va_start(ap, sFmt);
  vsnprintf(bufRepr, sizeof(bufRepr), sFmt, ap);
  va_end(ap);
  bufRepr[sizeof(bufRepr)-1] = 0;

  uIndent = uDepth * 2;

  if( pFieldDef != NULL )
  {
    // "normal"
    if( pFieldDef->m_eFId != NMFID_NONE )
    {
      n = fprintf(stderr, "%*s%c:%s(%u) = %s: ",
          uIndent, "", pFieldDef->m_eFType, pFieldDef->m_sFName,
          pFieldDef->m_eFId, bufRepr);
    }
    // vector items have no field ids
    else
    {
      n = fprintf(stderr, "%*s[%c]: = %s: ",
          uIndent, "", pFieldDef->m_eFType, bufRepr);
    }
  }
  else
  {
    n = fprintf(stderr, "%*s%s: ", uDepth*2, "", bufRepr);
  }

  if( uCount > 0 )
  {
    if( n > 36 )
    {
      fprintf(stderr, "\n");
      n = 0;
    }

    fprintf(stderr, "%*s", 38-n, "");
    nmPrintBuf(stderr, NULL, buf, uCount, 8, 38);
  }
  fprintf(stderr, "\n");
}
#endif // NMLIB_DEBUG
