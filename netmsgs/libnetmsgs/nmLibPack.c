////////////////////////////////////////////////////////////////////////////////
//
// Package:   netmsgs
//
// Library:   libnetmsgs
//
// File:      nmLibPack.c
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:32:30 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1578 $
 *
 * \brief Field value packing/unpacking definitions.
 *
 * The host may be either a big-endian or little-endian architecture. Currenly,
 * any exotic architectures are not supported. It is assumed that 1, 2, and 4
 * byte (unsigned) integers and 4 byte floats are supported. For systems that
 * do not support 8 byte (unsigned) integers or 8 byte doubles, promotion
 * will be done in software. However, on unpacking, rounding or truncation will
 * occur if the unpacked element exceeds the 4 byte limits.
 *
 * Floats are expected to be in IEEE 754 format. 
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
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

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <libgen.h>
#include <string.h>
#include <ctype.h>
#include <endian.h>

#ifdef ARCH_cygwin
#include "cyg-ieee754.h"
#else
#include <ieee754.h>
#endif

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/netmsgs.h"

#include "nmLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Host Architecture Prerequisites.
//
// Adjust the checks out as more architectures are supported.
// 
// For example, if an embedded MCU only supports 16-bit integers, then the U32
// and S32 packing and unpacking routines, amoung others, must change to
// pack/unpack 2 16-bit integers.
//
#if UCHAR_MAX != 0xff
#error "The netmsgs package requires 8-bit bytes."
#endif

#if USHRT_MAX != 0xffff
#error "The netmsgs package requires 16-bit shorts."
#endif

#if UINT_MAX != 0xffffffff
#error "The netmsgs package requires 32-bit integers."
#endif

//
// This Host Byte Order
//
#if __BYTE_ORDER == __BIG_ENDIAN
#define HOST_BIG_ENDIAN           ///< big-endian byte order
#else
#define HOST_LITTLE_ENDIAN        ///< little-endian byte order
#endif

#if __WORDSIZE == 64
#define HOST_WORD_SIZE  64        ///< 64-bit architecture
#else
#define HOST_WORD_SIZE  32        ///< 32-bit architecture
#endif


//.............................................................................
// Big-Endian Host Core Packing and Unpacking Operations
//.............................................................................
#ifdef HOST_BIG_ENDIAN

/*!
 * \brief Pack an element into the message buffer in big-endian byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] p    Byte pointer to typecasted element to be packed.
 * \param size      Byte size of element to pack.
 * \param [out] buf Output message buffer.
 */
static inline void packbig(byte_t *p, size_t size, byte_t buf[])
{
  size_t    n = 0;

  // forward copy
  while( n < size )
  {
    buf[n++] = *p++;
  }
}

/*!
 * \brief Unpack an element from the message buffer in big-endian byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] buf  Output message buffer.
 * \param [out] p   Byte pointer to typecasted element to be unpacked (set).
 * \param size      Byte size of element to unpack.
 */
static inline void unpackbig(byte_t buf[], byte_t *p, size_t size)
{
  size_t    n = 0;

  // forward copy
  while( n < size )
  {
    *p++ = buf[n++];
  }
}

/*!
 * \brief Pack an element into the message buffer in little-endian byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] p    Byte pointer to typecasted element to be packed.
 * \param size      Byte size of element to pack.
 * \param [out] buf Output message buffer.
 */
static inline void packlittle(byte_t *p, size_t size, byte_t buf[])
{
  size_t    n = 0;
  size_t    k = size - 1;

  // reverse copy
  while( n < size )
  {
    buf[n++] = p[k--];
  }
}

/*!
 * \brief Unpack an element from the message buffer in little-endian byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] buf  Output message buffer.
 * \param [out] p   Byte pointer to typecasted element to be unpacked (set).
 * \param size      Byte size of element to unpack.
 */
static inline void unpacklittle(byte_t buf[], byte_t *p, size_t size)
{
  size_t    n = 0;
  size_t    k = size - 1;

  // reverse copy
  while( n < size )
  {
    p[k--] = buf[n++];
  }
}

/*!
 * \brief Pack an element into the message buffer in native byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] val  Type T element to be packed.
 * \param [out] buf Output message buffer.
 */
#define PACKNATIVE(val, buf)  packbig((byte_t *)&(val), sizeof(val), buf)

/*!
 * \brief Unpack an element from the message buffer in native byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] buf    Output message buffer.
 * \param [out] pval  Type T* element to be unpacked (set).
 */
#define UNPACKNATIVE(buf, pval) unpackbig(buf, (byte_t *)pval, sizeof(*pval))

/*!
 * \brief Pack two 4-btye unsigned integers into the message buffer in native
 * byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] uMSB   Most Significant Bytes.
 * \param [in] uLSB   Least Significant Bytes.
 * \param [out] buf   Output message buffer.
 */
#define PACK64NATIVE(uMSB, uLSB, buf) PACK64BIG(uMSB, uLSB, buf)

/*!
 * \brief Unpack two 4-byte unsigned integers from the message buffer in native
 * byte order.
 *
 * Big-Endian Host Version
 *
 * \param [in] buf      Output message buffer.
 * \param [out] puMSB   Most Significant Bytes.
 * \param [out] puLSB   Least Significant Bytes.
 */
#define UNPACK64NATIVE(buf, puMSB, puLSB) UNPACK64BIG(buf, puMSB, puLSB)


//.............................................................................
// Little-Endian Host Core Packing and Unpacking Operations
//.............................................................................
#else // HOST_LITTLE_ENDIAN

/*!
 * \brief Pack an element into the message buffer in big-endian byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] p    Byte pointer to typecasted element to be packed.
 * \param size      Byte size of element to pack.
 * \param [out] buf Output message buffer.
 */
static inline void packbig(byte_t *p, size_t size, byte_t buf[])
{
  size_t    n = 0;
  size_t    k = size - 1;

  // reverse copy
  while( n < size )
  {
    buf[n++] = p[k--];
  }
}

/*!
 * \brief Unpack an element from the message buffer in big-endian byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] buf  Output message buffer.
 * \param [out] p   Byte pointer to typecasted element to be unpacked (set).
 * \param size      Byte size of element to unpack.
 */
static inline void unpackbig(byte_t buf[], byte_t *p, size_t size)
{
  size_t    n = 0;
  size_t    k = size - 1;

  // reverse copy
  while( n < size )
  {
    p[k--] = buf[n++];
  }
}

/*!
 * \brief Pack an element into the message buffer in little-endian byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] p    Byte pointer to typecasted element to be packed.
 * \param size      Byte size of element to pack.
 * \param [out] buf Output message buffer.
 */
static inline void packlittle(byte_t *p, size_t size, byte_t buf[])
{
  size_t    n = 0;

  // forward copy
  while( n < size )
  {
    buf[n++] = *p++;
  }
}

/*!
 * \brief Unpack an element from the message buffer in little-endian byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] buf  Output message buffer.
 * \param [out] p   Byte pointer to typecasted element to be unpacked (set).
 * \param size      Byte size of element to unpack.
 */
static inline void unpacklittle(byte_t buf[], byte_t *p, size_t size)
{
  size_t    n = 0;

  // forward copy
  while( n < size )
  {
    *p++ = buf[n++];
  }
}

/*!
 * \brief Pack an element into the message buffer in native byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] val  Type T element to be packed.
 * \param [out] buf Output message buffer.
 */
#define PACKNATIVE(val, buf)  packlittle((byte_t *)&(val), sizeof(val), buf)

/*!
 * \brief Unpack an element from the message buffer in native byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] buf    Output message buffer.
 * \param [out] pval  Type T* element to be unpacked (set).
 */
#define UNPACKNATIVE(buf, pval) unpacklittle(buf, (byte_t *)pval, sizeof(*pval))

/*!
 * \brief Pack two 4-btye unsigned integers into the message buffer in native
 * byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] uMSB   Most Significant Bytes.
 * \param [in] uLSB   Least Significant Bytes.
 * \param [out] buf   Output message buffer.
 */
#define PACK64NATIVE(uMSB, uLSB, buf) PACK64LITTLE(uMSB, uLSB, buf)

/*!
 * \brief Unpack two 4-byte unsigned integers from the message buffer in native
 * byte order.
 *
 * Little-Endian Host Version
 *
 * \param [in] buf     Output message buffer.
 * \param [out] puMSB  Pointer to Most Significant Bytes.
 * \param [out] puLSB  Pointer to Least Significant Bytes.
 */
#define UNPACK64NATIVE(buf, puMSB, puLSB) UNPACK64LITTLE(buf, puMSB, puLSB)

#endif // HOST_BIG_ENDIAN


//.............................................................................
// Auto-Endian Host Core Packing and Unpacking Operations
//.............................................................................

/*!
 * \brief Pack an element into the message buffer in big-endian byte order.
 *
 * \param [in] val  Type T element to be packed.
 * \param [out] buf Output message buffer.
 */
#define PACKBIG(val, buf)     packbig((byte_t *)&(val), sizeof(val), buf)

/*!
 * \brief Unpack an element from the message buffer in big-endian byte order.
 *
 * \param [in] buf    Output message buffer.
 * \param [out] pval  Type T* element to be unpacked (set).
 */
#define UNPACKBIG(buf, pval)  unpackbig(buf, (byte_t *)pval, sizeof(*pval));

/*!
 * \brief Pack an element into the message buffer in little-endian byte order.
 *
 * \param [in] val  Type T element to be packed.
 * \param [out] buf Output message buffer.
 */
#define PACKLITTLE(val, buf)  packlittle((byte_t *)&(val), sizeof(val), buf)

/*!
 * \brief Unpack an element from the message buffer in little-endian byte order.
 *
 * \param [in] buf    Output message buffer.
 * \param [out] pval  Type T* element to be unpacked (set).
 */
#define UNPACKLITTLE(buf, pval) unpacklittle(buf, (byte_t *)pval, sizeof(*pval))

/*!
 * \brief Pack two 4-btye unsigned integers into the message buffer in 
 * big-endian byte order.
 *
 * Safe 8-byte packing.
 *
 * \note The netmsgs package requires the host to have 4-byte integers
 * available, but not 8-byte versions. 
 *
 * \param [in] uMSB   Most Significant Bytes.
 * \param [in] uLSB   Least Significant Bytes.
 * \param [out] buf   Output message buffer.
 */
#define PACK64BIG(uMSB, uLSB, buf) \
  do \
  { \
    PACKBIG(uMSB, buf); \
    PACKBIG(uLSB, &(buf)[4]); \
  } while(0)

/*!
 * \brief Unpack two 4-byte unsigned integers from the message buffer in
 * big-endian byte order.
 *
 * Safe 8-byte packing.
 *
 * \note The netmsgs package requires the host to have 4-byte integers
 * available, but not 8-byte versions. 
 *
 *
 * \param [in] buf     Output message buffer.
 * \param [out] puMSB  Pointer to Most Significant Bytes.
 * \param [out] puLSB  Pointer to Least Significant Bytes.
 */
#define UNPACK64BIG(buf, puMSB, puLSB) \
  do \
  { \
    UNPACKBIG(buf, puMSB); \
    UNPACKBIG(&(buf)[4], puLSB); \
  } while(0)

/*!
 * \brief Pack two 4-btye unsigned integers into the message buffer in 
 * little-endian byte order.
 *
 * Safe 8-byte packing.
 *
 * \note The netmsgs package requires the host to have 4-byte integers
 * available, but not 8-byte versions. 
 *
 * \param [in] uMSB   Most Significant Bytes.
 * \param [in] uLSB   Least Significant Bytes.
 * \param [out] buf   Output message buffer.
 */
#define PACK64LITTLE(uMSB, uLSB, buf) \
  do \
  { \
    PACKLITTLE(uLSB, buf); \
    PACKLITTLE(uMSB, &(buf)[4]); \
  } while(0)

/*!
 * \brief Unpack two 4-byte unsigned integers from the message buffer in
 * little-endian byte order.
 *
 * Safe 8-byte packing.
 *
 * \note The netmsgs package requires the host to have 4-byte integers
 * available, but not 8-byte versions. 
 *
 *
 * \param [in] buf     Output message buffer.
 * \param [out] puMSB  Pointer to Most Significant Bytes.
 * \param [out] puLSB  Pointer to Least Significant Bytes.
 */
#define UNPACK64LITTLE(buf, puMSB, puLSB) \
  do \
  { \
    UNPACKLITTLE(buf, puLSB); \
    UNPACKLITTLE(&(buf)[4], puMSB); \
  } while(0)


//.............................................................................
// IEEE754 Floating-Pointer Numbers Constants, Macros, and Functions
//.............................................................................

//
// Single Precision 32-Bit Floating Point Numbers
//
// Shifts are relative to packing into a 4-byte unsigned integer.
// Masks are right-justified (pre-shifted) values.
//
#define IEEE754_F32_SIGN_SHIFT      31          ///< f32 sign shift
#define IEEE754_F32_SIGN_MASK       0x00000001  ///< f32 sign mask
#define IEEE754_F32_EXPONENT_SHIFT  23          ///< f32 exponent shift
#define IEEE754_F32_EXPONENT_MASK   0x000000ff  ///< f32 exponent mask
#define IEEE754_F32_MANTISSA_SHIFT  0           ///< f32 mantissa shift
#define IEEE754_F32_MANTISSA_MASK   0x007fffff  ///< f32 mantissa mask


//
// Double Precision 64-Bit Floating Point Numbers
//
// Shifts are relative to packing into a 4-byte unsigned integer. Two unsigned
// integers are required to fully represent the bit-pattern.
// Masks are right-justified (pre-shifted) values.
//
#define IEEE754_F64_SIGN_SHIFT      31          ///< f64 sign shift
#define IEEE754_F64_SIGN_MASK       0x00000001  ///< f64 sign mask
#define IEEE754_F64_EXPONENT_SHIFT  20          ///< f64 exponent shift
#define IEEE754_F64_EXPONENT_MASK   0x000007ff  ///< f64 exponent mask
#define IEEE754_F64_MANTISSA0_SHIFT 0           ///< f64 msb mantissa shift
#define IEEE754_F64_MANTISSA0_MASK  0x000fffff  ///< f64 msb mantissa mask
#define IEEE754_F64_MANTISSA1_SHIFT 0           ///< f64 lsb mantissa shift
#define IEEE754_F64_MANTISSA1_MASK  0xffffffff  ///< f64 lsb mantissa mask

//
// Conversions between 32-bit and 64-bit Floating-Point Numbers Shifts and Masks
//
#define IEEE754_F32_F64_MANTISSA0_SHIFT   3           ///< msb mantissa shift
#define IEEE754_F32_F64_MANTISSA1_SHIFT   29          ///< lsb mantissa shift
#define IEEE754_F32_F64_MANTISSA1_MASK    0x00000007  ///< lsb mantissa mask

//
// Floating-Point Numbers Limits
//
#define IEEE754_F32_EXP_MAX         127         ///< maximum power of 2 exponent
#define IEEE754_F32_EXP_MIN       (-126)        ///< minimum power of 2 exponent
#define IEEE754_F64_EXP_MAX         1023        ///< maximum power of 2 exponent
#define IEEE754_F64_EXP_MIN       (-1022)       ///< minimum power of 2 exponent

//
// Floating-Point Number Infinities
//
#define IEEE754_F32_POS_INF         HUGE_VALF   ///< f32 positive infinity
#define IEEE754_F32_NEG_INF       (-HUGE_VALF)  ///< f32 negative infinity
#define IEEE754_F64_POS_INF         HUGE_VAL    ///< f64 positive infinity
#define IEEE754_F64_NEG_INF       (-HUGE_VAL)   ///< f64 negative infinity

/*!
 * Not-a-Number bit pattern for 32-bit floating-point numbers.
 */
static const union ieee754_float IEEE754_F32_NaN =
{
  .ieee.negative  = 0,
  .ieee.exponent  = IEEE754_F32_EXPONENT_MASK, //   0xff,
  .ieee.mantissa  = 0x400000,
};

/*!
 * Not-a-Number bit pattern for 64-bit floating-point numbers.
 */
static const union ieee754_double IEEE754_F64_NaN =
{
  .ieee.negative  = 0,
  .ieee.exponent  = IEEE754_F64_EXPONENT_MASK, //0x7ff,
  .ieee.mantissa0 = 0x80000,
  .ieee.mantissa1 = 0 
};

/*!
 * \h_plusmn Infinity bit pattern for 32-bit floating-point numbers.
 */
static const union ieee754_float IEEE754_F32_Inf =
{
  .ieee.negative  = 0,
  .ieee.exponent  = IEEE754_F32_EXPONENT_MASK, //   0xff,
  .ieee.mantissa  = 0
};

/*!
 * \h_plusmn Infinity bit pattern for 64-bit floating-point numbers.
 */
static const union ieee754_double IEEE754_F64_Inf =
{
  .ieee.negative  = 0,
  .ieee.exponent  = IEEE754_F64_EXPONENT_MASK, //0x7ff,
  .ieee.mantissa0 = 0,
  .ieee.mantissa1 = 0 
};

/*!
 * \h_plusmn 0 bit pattern for 32-bit floating-point numbers.
 */
static const union ieee754_float IEEE754_F32_Zero =
{
  .ieee.negative  = 0,
  .ieee.exponent  = 0,
  .ieee.mantissa  = 0
};

/*!
 * \h_plusmn 0 bit pattern for 64-bit floating-point numbers.
 */
static const union ieee754_double IEEE754_F64_Zero =
{
  .ieee.negative  = 0,
  .ieee.exponent  = 0,
  .ieee.mantissa0 = 0,
  .ieee.mantissa1 = 0 
};

/*!
 * \brief Test if 32-bit floating-point number is a NaN.
 * \param f   32-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f32_is_nan(union ieee754_float f)
{
  return (f.ieee.exponent == IEEE754_F32_NaN.ieee.exponent)
         && (f.ieee.mantissa != 0)? true: false;
}

/*!
 * \brief Test if 64-bit floating-point number is a NaN.
 * \param d   64-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f64_is_nan(union ieee754_double d)
{
  return (d.ieee.exponent == IEEE754_F64_NaN.ieee.exponent)
         && ((d.ieee.mantissa0 != 0) || (d.ieee.mantissa1 != 0))? true: false;
}

/*!
 * \brief Test if 32-bit floating-point number is \h_plusmn infinity.
 * \param f   32-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f32_is_inf(union ieee754_float f)
{
  return (f.ieee.exponent == IEEE754_F32_Inf.ieee.exponent)
         && (f.ieee.mantissa == 0)? true: false;
}

/*!
 * \brief Test if 64-bit floating-point number is \h_plusmn infinity.
 * \param d   64-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f64_is_inf(union ieee754_double d)
{
  return (d.ieee.exponent == IEEE754_F64_Inf.ieee.exponent)
         && ((d.ieee.mantissa0 == 0) || (d.ieee.mantissa1 == 0))? true: false;
}

/*!
 * \brief Test if 32-bit floating-point number is \h_plusmn 0.0.
 * \param f   32-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f32_is_zero(union ieee754_float f)
{
  return (f.ieee.exponent == IEEE754_F32_Zero.ieee.exponent)
         && (f.ieee.mantissa == 0)? true: false;
}

/*!
 * \brief Test if 64-bit floating-point number is \h_plusmn 0.0.
 * \param d   64-bit float with bit overlay.
 * \return Returns true or false.
 */
static inline bool_t ieee754_f64_is_zero(union ieee754_double d)
{
  return (d.ieee.exponent == IEEE754_F64_Zero.ieee.exponent)
         && ((d.ieee.mantissa0 == 0) || (d.ieee.mantissa1 == 0))? true: false;
}


//.............................................................................
// Helper Checks
//.............................................................................

/*!
 * \brief Check if memory is available.
 * 
 * On check failure, returns from calling function with \ref NM_ECODE_NOMEM.
 *
 * \param size  Available memory in bytes.
 * \param count Required memory.
 */
#define _NM_CHK_MEM(size, count) \
  do \
  { \
    if( (size) < (count) ) \
    { \
      return -NM_ECODE_NOMEM; \
    } \
  } \
  while(0)

/*!
 * \brief Check if host architecture supports basic element sizes.
 *
 * On check failure, returns from calling function with
 * \ref NM_ECODE_ARCH_NOTSUP.
 *
 * \param val   Element to check
 * \param size  Required byte size.
 */
#define _NM_CHK_ARCH(val, size) \
  do \
  { \
    if( sizeof(val) != (size) ) \
    { \
      return -NM_ECODE_ARCH_NOTSUP; \
    } \
  } \
  while(0)


// ---------------------------------------------------------------------------
// Base Packing Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Pack an unsigned 8-bit byte into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackU8(byte_t val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U8;      // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  buf[0] = val & 0xff;

  return (int)fvalLen;
}

/*!
 * \brief Pack an unsigned 16-bit integer into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackU16(ushort_t val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U16;     // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  switch( eEndian )
  {
    case NMEndianLittle:
      PACKLITTLE(val, buf);
      break;
    case NMEndianNative:
      PACKNATIVE(val, buf);
      break;
    case NMEndianBig:
    default:
      PACKBIG(val, buf);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack an unsigned 32-bit integer into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackU32(uint_t val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U32;     // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  switch( eEndian )
  {
    case NMEndianLittle:
      PACKLITTLE(val, buf);
      break;
    case NMEndianNative:
      PACKNATIVE(val, buf);
      break;
    case NMEndianBig:
    default:
      PACKBIG(val, buf);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack an unsigned 64-bit integer into the message buffer.
 *
 * If the machine architecture only supports 32-bit long longs, then
 * the 4 MSBs are padded with zero's.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackU64(ulonglong_t val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U64;     // message field value length

  uint_t        uMSB;     // most significant bytes
  uint_t        uLSB;     // least signifcant bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte unsigned long long
  if( sizeof(ulonglong_t) == fvalLen )
  {
    switch( eEndian )
    {
      case NMEndianLittle:
        PACKLITTLE(val, buf);
        break;
      case NMEndianNative:
        PACKNATIVE(val, buf);
        break;
      case NMEndianBig:
      default:
        PACKBIG(val, buf);
        break;
    }
  }

  // 4-byte unsigned long long
  else
  {
    _NM_CHK_ARCH(val, (size_t)NMFVAL_LEN_U32);

    uMSB = 0;
    uLSB = (uint_t)val;

    switch( eEndian )
    {
      case NMEndianLittle:
        PACK64LITTLE(uMSB, uLSB, buf);
        break;
      case NMEndianNative:
        PACK64NATIVE(uMSB, uLSB, buf);
        break;
      case NMEndianBig:
      default:
        PACK64BIG(uMSB, uLSB, buf);
        break;
    }
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack an signed 64-bit integer into the message buffer.
 *
 * If the machine architecture only supports 32-bit long longs, then
 * the 4 MSBs are signed extended.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackS64(long long val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_S64;     // message field value length

  uint_t        uMSB;     // most significant bytes
  uint_t        uLSB;     // least signifcant bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte signed long long
  if( sizeof(long long) == fvalLen )
  {
    switch( eEndian )
    {
      case NMEndianLittle:
        PACKLITTLE(val, buf);
        break;
      case NMEndianNative:
        PACKNATIVE(val, buf);
        break;
      case NMEndianBig:
      default:
        PACKBIG(val, buf);
        break;
    }
  }

  // 4-byte signed long long
  else
  {
    _NM_CHK_ARCH(val, (size_t)NMFVAL_LEN_S32);

    uMSB = val & 0x80000000? 0xffffffff: 0x00000000;  // extend sign
    uLSB = (uint_t)val;

    switch( eEndian )
    {
      case NMEndianLittle:
        PACK64LITTLE(uMSB, uLSB, buf);
        break;
      case NMEndianNative:
        PACK64NATIVE(uMSB, uLSB, buf);
        break;
      case NMEndianBig:
      default:
        PACK64BIG(uMSB, uLSB, buf);
        break;
    }
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack a 32-bit float into the message buffer.
 *
 * The IEEE 754 32-bit format is used with both the big and little endian byte
 * orders.
 *
 * \note
 *  <em>On some machines, while integers were represented in little-endian form,
 *  floating point numbers were represented in big-endian form. Because there
 *  are many floating point formats, and a lack of a standard <b>network</b>
 *  representation, no standard for transferring floating point values has been
 *  made</em> - Wikipedia.
 *
 *  \todo
 *  Detect non-IEEE 754 formats and convert to IEEE 754 prior to packing.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackF32(float val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_F32;     // message field value length

  union ieee754_float f;      // 4 byte floating-point number
  uint_t              uBits;  // bits

  _NM_CHK_MEM(bufSize, fvalLen);
  _NM_CHK_ARCH(val, fvalLen);

  f.f = val;

  uBits  = (uint_t)((f.ieee.negative & IEEE754_F32_SIGN_MASK)
                      << IEEE754_F32_SIGN_SHIFT);
  uBits |= (uint_t)((f.ieee.exponent & IEEE754_F32_EXPONENT_MASK)
                      << IEEE754_F32_EXPONENT_SHIFT);
  uBits |= (uint_t)((f.ieee.mantissa & IEEE754_F32_MANTISSA_MASK)
                      << IEEE754_F32_MANTISSA_SHIFT);

  switch( eEndian )
  {
    case NMEndianLittle:
      PACKLITTLE(uBits, buf);
      break;
    case NMEndianNative:
      PACKNATIVE(f.f, buf);
      break;
    case NMEndianBig:
    default:
      PACKBIG(uBits, buf);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack a 64-bit float into the message buffer.
 *
 * The IEEE 754 64-bit format is used with both the big and little endian byte
 * orders. If the machine architecture only supports 32-bit doubles, then a
 * conversion to 64-bit representation is performed first.
 *
 * \note
 *  <em>On some machines, while integers were represented in little-endian form,
 *  floating point numbers were represented in big-endian form. Because there
 *  are many floating point formats, and a lack of a standard <b>network</b>
 *  representation, no standard for transferring floating point values has been
 *  made</em> - Wikipedia.
 *
 *  \todo
 *  Detect non-IEEE 754 formats and convert to IEEE 754 prior to packing.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackF64(double val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_F64;     // message field value length

  union ieee754_float   f;        // 4-byte floating-point number
  union ieee754_double  d;        // 8-byte floating-point number
  uint_t                uMSB;     // Most Signifcant Bytes
  uint_t                uLSB;     // Least Signifcant Bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte double
  if( sizeof(double) == fvalLen )
  {
    d.d = val;
  }

  // 4-byte double
  else
  {
    _NM_CHK_ARCH(val, (size_t)NMFVAL_LEN_F32);

    f.f = (float)val;

    // special infinity cases
    if( ieee754_f32_is_inf(f) )
    {
      d = IEEE754_F64_Inf;
    }

    // special zero cases
    else if( ieee754_f32_is_zero(f) )
    {
      d = IEEE754_F64_Zero;
    }

    // special Not-a-Number case
    else if( ieee754_f32_is_nan(f) )
    {
      d = IEEE754_F64_NaN;
    }
    
    // "normal" floating-point numbers
    else
    {
      d.ieee.exponent   = (f.ieee.exponent  + (uint_t)IEEE754_DOUBLE_BIAS
                                            - (uint_t)IEEE754_FLOAT_BIAS)
                            & IEEE754_F64_EXPONENT_MASK;
      d.ieee.mantissa0  = ((uint_t)(f.ieee.mantissa 
                                  >> (uint_t)IEEE754_F32_F64_MANTISSA0_SHIFT))
                            & IEEE754_F64_MANTISSA0_MASK;
      d.ieee.mantissa1  = (((uint_t)(f.ieee.mantissa)) 
                                  & (uint_t)IEEE754_F32_F64_MANTISSA1_MASK)
                            << IEEE754_F32_F64_MANTISSA1_SHIFT;
    }

    // preserve the sign
    d.ieee.negative = f.ieee.negative;
  }

  //
  // Stuff the bits into two 4-byte unsigned integers.
  //
  uMSB  = (uint_t)((d.ieee.negative & IEEE754_F64_SIGN_MASK)
                        << IEEE754_F64_SIGN_SHIFT);
  uMSB |= (uint_t)((d.ieee.exponent & IEEE754_F64_EXPONENT_MASK)
                        << IEEE754_F64_EXPONENT_SHIFT);
  uMSB |= (uint_t)((d.ieee.mantissa0 & IEEE754_F64_MANTISSA0_MASK)
                        << IEEE754_F64_MANTISSA0_SHIFT);
  uLSB  = (uint_t)((d.ieee.mantissa1 & IEEE754_F64_MANTISSA1_MASK)
                        << IEEE754_F64_MANTISSA1_SHIFT);

  switch( eEndian )
  {
    case NMEndianLittle:
      PACK64LITTLE(uMSB, uLSB, buf);
      break;
    case NMEndianNative:
      PACK64NATIVE(uMSB, uLSB, buf);
      break;
    case NMEndianBig:
    default:
      PACK64BIG(uMSB, uLSB, buf);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Pack a 32-bit pointer into the message buffer.
 *
 * If the machine architecture has 64-bit pointers, then only the 
 * LSB 4 bytes are packed.
 *
 * \note Pointer is always packed in native byte order.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackP32(void *val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_P32;     // message field value length
  ulong_t       luVal;
  uint_t        uVal;

  _NM_CHK_MEM(bufSize, fvalLen);

  // long's are same size as void *'s. exceptions?
  luVal = *((ulong_t *)val);

  // 4-byte natvie pointer
  if( sizeof(void *) == fvalLen )
  {
    PACKNATIVE(luVal, buf);
  }

  // 8-byte native pointer - "convert"
  if( sizeof(void *) == NMFVAL_LEN_P64 )
  {
#if HOST_WORD_SIZE == 64
    //if( luVal & 0xffffffff00000000 )
    if( (luVal) & 0xffffffff00000000 )
    {
      NMLIB_WARNING("%p: Packing 64-bit pointer in 32 bits lost non-zero "
                    "significance.", (void *)luVal);
    }
#endif // HOST_WORD_SIZE
    uVal = (uint_t)(luVal & 0xffffffff);
    PACKNATIVE(uVal, buf);
  }


  return (int)fvalLen;
}

/*!
 * \brief Pack a 64-bit pointer into the message buffer.
 *
 * If the machine architecture only supports 32-bit pointers, then
 * the 4 MSBs are padded with 0's.
 *
 * \note Pointer is always packed in native byte order.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
int nmPackP64(void *val, byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_P64;     // message field value length

  ulong_t               luVal;
  uint_t                uMSB;     // Most Signifcant Bytes
  uint_t                uLSB;     // Least Signifcant Bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // long's are same size as void *'s. exceptions?
  luVal = *((ulong_t *)val);

  // 8-byte pointer
  if( sizeof(void *) == fvalLen )
  {
#if HOST_WORD_SIZE == 64
    uMSB  = (uint_t)(luVal >> 32);
    uLSB  = (uint_t)(luVal & 0xffffffff);
#else
    uMSB  = 0;
    uLSB  = (uint_t)luVal;
#endif // HOST_WORD_SIZE
  }

  // 4-byte pointer
  else
  {
    _NM_CHK_ARCH(val, (size_t)NMFVAL_LEN_P32);

    uMSB  = 0;
    uLSB  = (uint_t)luVal;
  }

  PACK64NATIVE(uMSB, uLSB, buf);

  return (int)fvalLen;
}

/*!
 * \brief Pack a byte buffer into the message buffer.
 *
 * The source and message buffer memory areas can overlap.
 *
 * \note  The buffer is always packed in sequential order ignoring the byte
 *        order parameter.
 *
 * \param [in] bufSrc Source buffer.
 * \param uCount      Number of bytes to copy.
 * \param [out] buf   Output message buffer.
 * \param bufSize     Size of output buffer.
 * \param eEndian     Packing order. See \ref NMEndian_T.
 *
 * \copydoc doc_return_pack_std
 */
int nmPackBuf(byte_t      bufSrc[],
              size_t      uCount,
              byte_t      buf[],
              size_t      bufSize,
              NMEndian_T  eEndian)
{
  _NM_CHK_MEM(bufSize, uCount);

  // safe copy 
  memmove(buf, bufSrc, uCount);

  return (int)uCount;
}

// ---------------------------------------------------------------------------
// Base Unpacking Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Unpack an unsigned 8-bit byte from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
int nmUnpackU8(byte_t buf[], size_t bufSize, byte_t *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U8;      // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  *pVal = buf[0];

  return (int)fvalLen;
}

/*!
 * \brief Unpack an unsigned 16-bit integer from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
int nmUnpackU16(byte_t buf[],
                size_t bufSize,
                ushort_t *pVal,
                NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U16;     // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  switch( eEndian )
  {
    case NMEndianLittle:
      UNPACKLITTLE(buf, pVal);
      break;
    case NMEndianNative:
      UNPACKNATIVE(buf, pVal);
      break;
    case NMEndianBig:
    default:
      UNPACKBIG(buf, pVal);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack an unsigned 32-bit integer from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
int nmUnpackU32(byte_t buf[], size_t bufSize, uint_t *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U32;     // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);

  switch( eEndian )
  {
    case NMEndianLittle:
      UNPACKLITTLE(buf, pVal);
      break;
    case NMEndianNative:
      UNPACKNATIVE(buf, pVal);
      break;
    case NMEndianBig:
    default:
      UNPACKBIG(buf, pVal);
      break;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack an unsigned 64-bit integer from the message buffer.
 *
 * If the machine architecture only supports 32-bit unsigned long longs, then
 * the 4 MSBs are truncated.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
int nmUnpackU64(byte_t buf[],
                size_t bufSize,
                ulonglong_t *pVal,
                NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_U64;     // message field value length

  uint_t        uMSB;     // most significant bytes
  uint_t        uLSB;     // least signifcant bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte unsigned long long
  if( sizeof(ulonglong_t) == fvalLen )
  {
    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACKLITTLE(buf, pVal);
        break;
      case NMEndianNative:
        UNPACKNATIVE(buf, pVal);
        break;
      case NMEndianBig:
      default:
        UNPACKBIG(buf, pVal);
        break;
    }
  }

  // 4-byte unsigned long long
  else
  {
    _NM_CHK_ARCH(*pVal, (size_t)NMFVAL_LEN_U32);

    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACK64LITTLE(buf, &uMSB, &uLSB);
        break;
      case NMEndianNative:
        UNPACK64NATIVE(buf, &uMSB, &uLSB);
        break;
      case NMEndianBig:
      default:
        UNPACK64BIG(buf, &uMSB, &uLSB);
        break;
    }

    if( uMSB != 0 )
    {
      NMLIB_WARNING("0x%08x%08x: Unpacking unsigned 64-bit integer into "
                    "32 bits lost non-zero significance.",
                    uMSB, uLSB);
    }

    *pVal = (ulonglong_t)uLSB;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack an signed 64-bit integer from the message buffer.
 *
 * If the machine architecture only supports 32-bit long longs, then
 * the 4 MSBs are truncated.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
int nmUnpackS64(byte_t buf[],
                size_t bufSize,
                long long *pVal,
                NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_S64;     // message field value length

  uint_t        uMSB;     // most significant bytes
  uint_t        uLSB;     // least signifcant bytes

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte unsigned long long
  if( sizeof(ulonglong_t) == fvalLen )
  {
    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACKLITTLE(buf, pVal);
        break;
      case NMEndianNative:
        UNPACKNATIVE(buf, pVal);
        break;
      case NMEndianBig:
      default:
        UNPACKBIG(buf, pVal);
        break;
    }
  }

  // 4-byte unsigned long long
  else
  {
    _NM_CHK_ARCH(*pVal, (size_t)NMFVAL_LEN_S32);

    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACK64LITTLE(buf, &uMSB, &uLSB);
        break;
      case NMEndianNative:
        UNPACK64NATIVE(buf, &uMSB, &uLSB);
        break;
      case NMEndianBig:
      default:
        UNPACK64BIG(buf, &uMSB, &uLSB);
        break;
    }

    if( (uMSB != 0) && (uMSB != 0xffffffff) )
    {
      NMLIB_WARNING("0x%08x%08x: Unpacking signed 64-bit integer into "
                    "32 bits lost non-zero significance.",
                    uMSB, uLSB);
    }

    *pVal = (long long)uLSB;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack a 32-bit float from the message buffer.
 *
 * The IEEE 754 32-bit format is used with both the big and little endian byte
 * orders.
 *
 * \note
 *  <em>On some machines, while integers were represented in little-endian form,
 *  floating point numbers were represented in big-endian form. Because there
 *  are many floating point formats, and a lack of a standard <b>network</b>
 *  representation, no standard for transferring floating point values has been
 *  made</em> - Wikipedia.
 *
 *  \todo
 *  Detect non-IEEE 754 formats and convert to IEEE 754 prior to packing.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
int nmUnpackF32(byte_t buf[], size_t bufSize, float *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_F32;     // message field value length

  _NM_CHK_MEM(bufSize, fvalLen);
  _NM_CHK_ARCH(*pVal, fvalLen);

  switch( eEndian )
  {
    case NMEndianLittle:
      UNPACKLITTLE(buf, pVal);
      break;
    case NMEndianNative:
      UNPACKNATIVE(buf, pVal);
      break;
    case NMEndianBig:
    default:
      UNPACKBIG(buf, pVal);
      break;
  }


  return (int)fvalLen;
}

/*!
 * \brief Unpack a 64-bit float from the message buffer.
 *
 * The IEEE 754 64-bit format is used with both the big and little endian byte
 * orders. If the machine architecture only supports 32-bit doubles, then 
 * mantissa truncation will occur.
 *
 * \note
 *  <em>On some machines, while integers were represented in little-endian form,
 *  floating point numbers were represented in big-endian form. Because there
 *  are many floating point formats, and a lack of a standard <b>network</b>
 *  representation, no standard for transferring floating point values has been
 *  made</em> - Wikipedia.
 *
 *  \todo
 *  Detect non-IEEE 754 formats and convert to IEEE 754 prior to packing.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
int nmUnpackF64(byte_t buf[], size_t bufSize, double *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_F64;     // message field value length

  uint_t                uMSB;       // Most Signifcant Bytes
  uint_t                uLSB;       // Least Signifcant Bytes
  int                   exponent;   // ieee754 unbiased exponent
  union ieee754_float   f;          // 4-byte floating-point number
  union ieee754_double  d;          // 8-byte floating-point number

  _NM_CHK_MEM(bufSize, fvalLen);

  // 8-byte doubles
  if( sizeof(double) == (size_t)NMFVAL_LEN_F64 )
  {
    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACKLITTLE(buf, &d.d);
        break;
      case NMEndianNative:
        UNPACKNATIVE(buf, &d.d);
        break;
      case NMEndianBig:
      default:
        UNPACKBIG(buf, &d.d);
        break;
    }

    *pVal = d.d;
  }

  else // 4-byte doubles
  {
    _NM_CHK_ARCH(*pVal, (size_t)NMFVAL_LEN_F32);

    switch( eEndian )
    {
      case NMEndianLittle:
        UNPACK64LITTLE(buf, &uMSB, &uLSB);
        break;
      case NMEndianNative:
        UNPACK64NATIVE(buf, &uMSB, &uLSB);
        break;
      case NMEndianBig:
      default:
        UNPACK64BIG(buf, &uMSB, &uLSB);
        break;
    }

    d.ieee.negative   = ((uMSB >> IEEE754_F64_SIGN_SHIFT)
                            & IEEE754_F64_SIGN_MASK)? 1: 0;
    d.ieee.exponent   = (uMSB >> IEEE754_F64_EXPONENT_SHIFT)
                            & IEEE754_F64_EXPONENT_MASK;
    d.ieee.mantissa0  = (uMSB >> IEEE754_F64_MANTISSA0_SHIFT)
                            & IEEE754_F64_MANTISSA0_MASK;
    d.ieee.mantissa1  = (uLSB >> IEEE754_F64_MANTISSA1_SHIFT)
                            & IEEE754_F64_MANTISSA1_MASK;

    // unbias the exponent
    exponent  = (int)d.ieee.exponent - IEEE754_DOUBLE_BIAS;

    // special Not-a-Number case
    if( ieee754_f64_is_nan(d) )
    {
      f = IEEE754_F32_NaN;
    }

    // special infinity cases
    else if( ieee754_f64_is_inf(d) )
    {
      f = IEEE754_F32_Inf;
    }

    // exponent overflow
    else if( exponent > IEEE754_F32_EXP_MAX )
    {
      f = IEEE754_F32_Inf;
      NMLIB_WARNING("floating-point number overflow.");
    }

    // exponent overflow
    else if( exponent < IEEE754_F32_EXP_MIN )
    {
      f = IEEE754_F32_Zero;
      NMLIB_WARNING("floating-point number underflow.");
    }

    // special zero cases
    else if( ieee754_f64_is_zero(d) )
    {
      f = IEEE754_F32_Zero;
    }

    // "normal" floating-point numbers
    else
    {
      f.ieee.exponent   = (byte_t)(exponent + IEEE754_FLOAT_BIAS);
      f.ieee.mantissa = ( (uint_t)((d.ieee.mantissa0
                                    & IEEE754_F64_MANTISSA0_MASK)
                                << IEEE754_F32_F64_MANTISSA0_SHIFT)
                        | (uint_t)((d.ieee.mantissa1
                                    >> IEEE754_F32_F64_MANTISSA1_SHIFT)
                                & IEEE754_F32_F64_MANTISSA1_MASK) )
                        & IEEE754_F32_MANTISSA_MASK;
      if( (d.ieee.mantissa1 & (uint_t)~IEEE754_F32_F64_MANTISSA1_MASK) != 0 )
      {
        NMLIB_WARNING("floating-point number mantissa truncation.");
      }
    }

    // preserve the sign
    f.ieee.negative   = d.ieee.negative;

    *pVal = (double)f.f;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack a 32-bit pointer from the message buffer.
 *
 * If the machine architecture has 64-bit pointers, then only the 
 * LSB 4 bytes are packed.
 *
 * \note Pointer is always unpacked in native byte order.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
int nmUnpackP32(byte_t buf[], size_t bufSize, void *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_P32;     // message field value length

  ulong_t  *pluVal;   // longs are same size as pointers on 32/64 bit machines
  uint_t    uP32;

  _NM_CHK_MEM(bufSize, fvalLen);
  
  pluVal = (ulong_t *)pVal;

  UNPACKNATIVE(buf, &uP32);

  *pluVal = (ulong_t)uP32;

  return (int)fvalLen;

}

/*!
 * \brief Unpack a 64-bit pointer from the message buffer.
 *
 * If the machine architecture only supports 32-bit pointers, then
 * the 4 MSBs are padded with 0's.
 *
 * \note Pointer is always unpacked in native byte order.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
int nmUnpackP64(byte_t buf[], size_t bufSize, void *pVal, NMEndian_T eEndian)
{
  static size_t fvalLen = NMFVAL_LEN_P64;     // message field value length

  uint_t    uMSB;     // Most Signifcant Bytes
  uint_t    uLSB;     // Least Signifcant Bytes
  ulong_t  *pluVal;   // longs are same size as pointers on 32/64 bit machines

  _NM_CHK_MEM(bufSize, fvalLen);
  
  pluVal = (ulong_t *)pVal;

  UNPACK64NATIVE(buf, &uMSB, &uLSB);

  // 8-byte pointer
  if( sizeof(void *) == fvalLen )
  {
#if HOST_WORD_SIZE == 64
    *pluVal = (ulong_t)(((ulong_t)uMSB  << 32) | (uLSB & 0xffffffff));
#else
    *pluVal = (ulong_t)uLSB;
#endif // HOST_WORD_SIZE
  }

  // 4-byte pointer
  else
  {
    _NM_CHK_ARCH(pVal, (size_t)NMFVAL_LEN_P32);

    if( uMSB != 0 )
    {
      NMLIB_WARNING("0x%08x%08x: Unpacking 64-bit pointer into 32 bits "
                    "lost non-zero significance.\n",
                    uMSB, uLSB);
    }
    *pluVal = (ulong_t)uLSB;
  }

  return (int)fvalLen;
}

/*!
 * \brief Unpack a byte buffer from the message buffer.
 *
 * The source and message buffer memory areas can overlap.
 *
 * \note  The buffer is always packed in sequential order ignoring the byte
 *        order parameter.
 *
 * \param [in] buf      Input message buffer.
 * \param bufSize       Size of input buffer.
 * \param [out] bufDst  Destination output buffer.
 * \param uCount        Number of bytes to copy.
 * \param eEndian       Unpacking order. See \ref NMEndian_T.
 *
 * \copydoc doc_return_unpack_std
 */
int nmUnpackBuf(byte_t      buf[],
                size_t      bufSize,
                byte_t      bufDst[],
                size_t      uCount,
                NMEndian_T  eEndian)
{
  _NM_CHK_MEM(bufSize, uCount);

  // safe copy 
  memmove(bufDst, buf, uCount);

  return (int)uCount;
}
