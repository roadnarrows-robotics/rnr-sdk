////////////////////////////////////////////////////////////////////////////////
//
// Package:   netmsgs
//
// Library:   libnetmsgs
//
// File:      nmLibPackIVT.c
//
/*! \file
 *
 * $LastChangedDate: 2010-07-31 08:48:56 -0600 (Sat, 31 Jul 2010) $
 * $Rev: 521 $
 *
 * \brief Identifier-Type-Value (ITV) message packing/unpacking definitions.
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
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/netmsgs.h"

#include "nmLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Forward Declarations
//
static NMPackFunc_T nmLookupITVPacker(NMFType_T eFType);
static NMUnpackFunc_T nmLookupITVUnpacker(NMFType_T eFType);
static int nmPackITVStream(const NMMsgDef_T *pMsgDef, void *pStruct,
                           byte_t buf[], size_t bufSize, NMEndian_T eEndian,
                           NMCtl_T *pCtl);
static int nmUnpackITVStream(const NMMsgDef_T *pMsgDef, byte_t buf[],
                             size_t uMsgLen, size_t uCount, void *pStruct,
                             NMEndian_T eEndian, NMCtl_T *pCtl);


// ...........................................................................
// Private Packing Interface 
// ...........................................................................

/*!
 * \brief Pack simple field type ITV header.
 *
 * \param p           Pointer to message field definition.
 * \param [out] buf   Output message buffer.
 * \param n           Size of output buffer.
 * \param ctl         Internal control.
 *
 * \copydoc doc_return_pack_std 
 */
#define ITVPACK_SIMPLE_HDR(p, buf, n, ctl) \
  ((ctl)->m_bNoHdr? 0: nmPackITVHdrSimple(p, buf, n))
  
/*!
 * \brief Pack ITV simple header.
 *
 * \param pFieldDef   Pointer to message field definition.
 * \param [out] buf   Output message buffer.
 * \param bufSize     Size of output buffer.
 *
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVHdrSimple(const NMFieldDef_T *pFieldDef,
                              byte_t              buf[],
                              size_t              bufSize)
{
  // pack simple ITV header
  if( NMITV_FHDR_SIZE_SIMPLE <= bufSize )
  {
    buf[0] = (byte_t)(pFieldDef->m_eFId & 0xff);
    buf[1] = (byte_t)(pFieldDef->m_eFType & 0xff);
    return NMITV_FHDR_SIZE_SIMPLE;
  }

  // no more space in buffer for header
  else
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, NMITV_FHDR_SIZE_STRING);
  }
}

/*!
 * \brief Pack ITV String header.
 *
 * \param pFieldDef   Pointer to message field definition.
 * \param uCount      Number of character bytes in string.
 * \param [out] buf   Output message buffer.
 * \param bufSize     Size of output buffer.
 *
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVHdrString(const NMFieldDef_T *pFieldDef,
                              size_t              uCount,
                              byte_t              buf[],
                              size_t              bufSize)
{
  // pack string ITV header
  if( NMITV_FHDR_SIZE_STRING <= bufSize )
  {
    buf[0] = (byte_t)(pFieldDef->m_eFId & 0xff);
    buf[1] = (byte_t)(pFieldDef->m_eFType & 0xff);
    buf[2] = (byte_t)(uCount & 0xff);
    return NMITV_FHDR_SIZE_STRING;
  }

  // no more space in buffer for header
  else
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, NMITV_FHDR_SIZE_STRING);
  }
}

/*!
 * \brief Pack ITV Struct header.
 *
 * \param pFieldDef   Pointer to message field definition.
 * \param [out] buf   Output message buffer.
 * \param bufSize     Size of output buffer.
 *
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVHdrStruct(const NMFieldDef_T *pFieldDef,
                              byte_t              buf[],
                              size_t              bufSize)
{
  // pack struct ITV header
  if( NMITV_FHDR_SIZE_STRUCT <= bufSize )
  {
    buf[0] = (byte_t)(pFieldDef->m_eFId & 0xff);
    buf[1] = (byte_t)(pFieldDef->m_eFType & 0xff);
    buf[2] = (byte_t)pFieldDef->m_this.m_struct->m_uCount;
    return NMITV_FHDR_SIZE_STRUCT;
  }

  // no more space in buffer for header
  else
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%zu", bufSize, NMITV_FHDR_SIZE_STRUCT);
  }
}

/*!
 * \brief Pack ITV Vector header.
 *
 * \param pFieldDef   Pointer to message field definition.
 * \param uCount      Number of vector elements.
 * \param [out] buf   Output message buffer.
 * \param bufSize     Size of output buffer.
 *
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVHdrVector(const NMFieldDef_T *pFieldDef,
                              size_t              uCount,
                              byte_t              buf[],
                              size_t              bufSize)
{
  // pack vector ITV header
  if( NMITV_FHDR_SIZE_VECTOR <= bufSize )
  {
    buf[0] = (byte_t)(pFieldDef->m_eFId & 0xff);
    buf[1] = (byte_t)(pFieldDef->m_eFType & 0xff);
    buf[2] = (byte_t)(uCount & 0xff);
    buf[3] = (byte_t)pFieldDef->m_this.m_vector.m_pThisElem->m_eFType;
    return NMITV_FHDR_SIZE_VECTOR;
  }

  // no more space in buffer for header
  else
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, NMITV_FHDR_SIZE_VECTOR);
  }
}

/*!
 * \brief Pack pad bytes into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVPad(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  size_t    uCount = pFieldDef->m_this.m_pad.m_uCount;

  // pack pad bytes
  if( uCount <= bufSize )
  {
    memset(buf, NMFVAL_PAD, uCount);  
    NMLIB_TRACE_FIELD(NULL, buf, uCount, pCtl, "pad(%u)", pFieldDef->m_eFId);
    return (int)uCount;
  }

  // no more space in buffer for pad bytes
  else
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < pad_count=%zu", bufSize, uCount);
  }
}

/*!
 * \brief Pack a boolean into an 8-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVBool(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  bool_t  val;    // constrained field value
  int     n;      // byte count/error code
  int     k;      // subbyte count/error code

  val = *((bool_t *)pVal);

  // try to pack header
  if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackBool(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%s", (val? "true": "false"));

    return n;
  }
}

/*!
 * \brief Pack an unsigned 8-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVU8(const NMFieldDef_T *pFieldDef,
                       void               *pVal,
                       byte_t              buf[],
                       size_t              bufSize,
                       NMEndian_T          eEndian,
                       NMCtl_T            *pCtl)
{
  byte_t  val;    // constrained field value
  int     n;      // byte count/error code
  int     k;      // subbyte count/error code

  // try to set field value
  if( (n = nmSetU8(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackU8(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhu", val);

    return n;
  }
}

/*!
 * \brief Pack a signed 8-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVS8(const NMFieldDef_T *pFieldDef,
                       void               *pVal,
                       byte_t              buf[],
                       size_t              bufSize,
                       NMEndian_T          eEndian,
                       NMCtl_T            *pCtl)
{
  signed char val;  // constrained field value
  int         n;    // byte count/error code
  int         k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetS8(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackS8(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhd", val);

    return n;
  }
}

/*!
 * \brief Pack an unsigned 16-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVU16(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  ushort_t  val;  // constrained field value
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetU16(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackU16(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hu", val);

    return n;
  }
}

/*!
 * \brief Pack a signed 16-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVS16(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  short   val;  // constrained field value
  int     n;    // byte count/error code
  int     k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetS16(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackS16(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0)
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hd", val);

    return n;
  }
}

/*!
 * \brief Pack an unsigned 32-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVU32(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  uint_t  val;  // constrained field value
  int     n;    // byte count/error code
  int     k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetU32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackU32(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%u", val);

    return n;
  }
}

/*!
 * \brief Pack a signed 32-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVS32(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  int   val;  // constrained field value
  int   n;    // byte count/error code
  int   k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetS32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackS32(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%d", val);

    return n;
  }
}

/*!
 * \brief Pack an unsigned 64-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVU64(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  ulonglong_t val;  // constrained field value
  int         n;    // byte count/error code
  int         k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetU64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackU64(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%llu", val);

    return n;
  }
}

/*!
 * \brief Pack a signed 64-bit ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVS64(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  long long val;  // constrained field value
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetS64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackS64(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%lld", val);

    return n;
  }
}

/*!
 * \brief Pack a 32-bit floating-point number ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVF32(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  float   val;  // constrained field value
  int     n;    // byte count/error code
  int     k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetF32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackF32(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%f", val);

    return n;
  }
}

/*!
 * \brief Pack a 64-bit floating-point number ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVF64(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  double  val;  // constrained field value
  int     n;    // byte count/error code
  int     k;    // subbyte count/error code

  // try to set field value
  if( (n = nmSetF64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // try to pack header
  else if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackF64(val, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%e", val);

    return n;
  }
}

/*!
 * \brief Pack a 32-bit pointer ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVP32(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // try to pack header
  if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackP32(pVal, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

    return n;
  }
}

/*!
 * \brief Pack a 64-bit pointer ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVP64(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // try to pack header
  if( (n = ITVPACK_SIMPLE_HDR(pFieldDef, buf, bufSize, pCtl)) < 0 )
  {
    return n;
  }

  // try to pack field value
  else if( (k = nmPackP64(pVal, &buf[n], bufSize-(size_t)n, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

    return n;
  }
}

/*!
 * \brief Pack null-terminated, variable length string ITV field into the
 * message buffer.
 *
 * \note The null character is not packed.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVString(const NMFieldDef_T  *pFieldDef,
                           void                *pVal,
                           byte_t               buf[],
                           size_t               bufSize,
                           NMEndian_T           eEndian,
                           NMCtl_T             *pCtl)
{
  const char   *sVal;
  const char   *sConst;
  size_t        uMaxCount;
  size_t        uCount;
  int           n = 0;

  sVal      = (const char *)pVal;     // override value pointer type
  sConst    = pFieldDef->m_this.m_string.m_sConst;
  uMaxCount = pFieldDef->m_this.m_string.m_uMaxCount;

  if( sConst != NULL )
  {
    sVal = sConst;
  }

  uCount = strlen(sVal);            // string length w/o null

  // string length exceeds maximum
  if( uCount > uMaxCount )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_RANGE, pFieldDef,
        "char_count=%zu > max_count=%zu.", uCount, uMaxCount);
  }

  // try to pack ITV header
  if( (n = nmPackITVHdrString(pFieldDef, uCount, buf, bufSize)) < 0 )
  {
    return n;
  }

  // no more space in buffer for string
  else if( uCount > bufSize-(size_t)n )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < str_len=%zu.", bufSize, uCount);
  }

  // copy string field value
  else
  {
    memcpy(&buf[n], sVal, uCount);

    n += (int)uCount;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "'%s'", sVal);

    return n;
  }
}

/*!
 * \brief Pack a structure ITV field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVStruct(const NMFieldDef_T *pFieldDef,
                           void               *pVal,
                           byte_t              buf[],
                           size_t              bufSize,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  const NMMsgDef_T *pStructDef;
  int               n = 0;
  int               k;

  pStructDef  = pFieldDef->m_this.m_struct;

  // try to pack ITV header
  if( (n = nmPackITVHdrStruct(pFieldDef, buf, bufSize)) < 0 )
  {
    return n;
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "structdef %s, member_count=%zu",
      pStructDef->m_sMsgName, pStructDef->m_uCount);

  pCtl->m_uDepth++;

  // copy struct field value
  k = nmPackITVStream(pStructDef, pVal, &buf[n], bufSize-(size_t)n, eEndian,
                        pCtl);

  pCtl->m_uDepth--;

  return k>=0? n+k: k;
}

/*!
 * \brief Pack variable length vector ITV field into the message buffer.
 *
 * A vector is a one-dimension array of the given type.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVVector(const NMFieldDef_T *pFieldDef,
                           void               *pVal,
                           byte_t              buf[],
                           size_t              bufSize,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  size_t              uMaxCount;
  size_t              uElemSize;
  const NMFieldDef_T *pElemDef;
  NMVec_T            *pVec;
  size_t              uCount;
  byte_t             *pItem;
  NMPackFunc_T        fnPack = NULL;
  int                 n = 0;
  int                 k;
  size_t              i;

  uMaxCount = pFieldDef->m_this.m_vector.m_uMaxCount;
  uElemSize = pFieldDef->m_this.m_vector.m_uElemSize;
  pElemDef  = pFieldDef->m_this.m_vector.m_pThisElem;
  pVec      = (NMVec_T *)pVal;                    // generalized vector overlay
  uCount    = pVec->m_uCount;                     // number of vector items
  pItem     = (byte_t *)(pVec) + memberoffset(NMVec_T, u.m_buf);
  fnPack    = nmLookupITVPacker(pElemDef->m_eFType);

  if( fnPack == NULL )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_FTYPE, pFieldDef,
        "vector_elem_ftype='%c'(0x%02x)",
        NMLIB_ASCII_FTYPE(pElemDef->m_eFType), pElemDef->m_eFType);
  }

  // exceeds maximum
  else if( uCount > uMaxCount )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_RANGE, pFieldDef,
        "vector_count=%zu > max_count=%zu.", uCount, uMaxCount);
  }

  // try to pack ITV header
  else if( (n = nmPackITVHdrVector(pFieldDef, uCount, buf, bufSize)) < 0 )
  {
    return n;
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%c:vector[%zu]",
      pElemDef->m_eFType, uCount);

  // no header for simple vector element types
  if( NMFTYPE_IS_SIMPLE(pElemDef->m_eFType) )
  {
    pCtl->m_bNoHdr = true;
  }

  pCtl->m_uDepth++;

  // copy vector field value
  for(i=0; i<uCount; ++i)
  {
    k = fnPack(pElemDef, pItem, &buf[n], bufSize-(size_t)n, eEndian, pCtl);

    if( k < 0 )
    {
      return k;
    }

    n += k;

    pItem += uElemSize;
  }

  pCtl->m_uDepth--;
  pCtl->m_bNoHdr = false;

  return n;
}

/*!
 * \brief Pack a ITV message.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param eEndian         Packing order. See \ref NMEndian_T.
 * \param pCtl            Pointer to Internal control.
 *
 * \copydoc doc_return_pack_std 
 */
static int nmPackITVStream(const NMMsgDef_T *pMsgDef,
                           void             *pStruct,
                           byte_t            buf[],
                           size_t            bufSize,
                           NMEndian_T        eEndian,
                           NMCtl_T          *pCtl)
{
  size_t              uFieldNum;
  const NMFieldDef_T *pFieldDef;
  NMPackFunc_T        fnPack = NULL;
  void               *pVal;
  int                 n = 0;
  int                 k;

  //
  // Pack message fields
  //
  for(uFieldNum=0, pFieldDef=pMsgDef->m_pFields;
      uFieldNum < pMsgDef->m_uCount;
      ++uFieldNum, ++pFieldDef)
  {
    if( n < bufSize )
    {
      if( pFieldDef->m_eFType == NMFTypePad )
      {
        pVal = NULL;
      }

      else
      {
        pVal = pStruct + pFieldDef->m_uOffset;
      }

      fnPack = nmLookupITVPacker(pFieldDef->m_eFType);

      if( fnPack == NULL )
      {
        NMLIB_RAISE_FIELD_ERROR(NM_ECODE_FTYPE, pFieldDef,
               "ftype='%c'(0x%02x).",
                NMLIB_ASCII_FTYPE(pFieldDef->m_eFType), pFieldDef->m_eFType);
      }

      k = fnPack(pFieldDef, pVal, &buf[n], bufSize-(size_t)n, eEndian, pCtl);

      if( k < 0 )
      {
        return k;
      }
    }

    else
    {
      NMLIB_RAISE_ERROR(NM_ECODE_NOMEM,
        "buf_size=%zu, msgid=%u", bufSize, pMsgDef->m_eMsgId);
    }

    n += k;
  }

  return n;
}


// ...........................................................................
// Private Unpacking Interface 
// ...........................................................................

/*!
 * \brief Move parse cursor position past current field in message stream field.
 *
 * No unpacking field into an output variable is executed.
 *
 * \param pFieldDef   Pointer to message field definition.
 * \param [in] buf    Input message buffer.
 * \param bufSize     Size of input buffer.
 * \param uFLen       Length of field (bytes).
 * \param pCtl        Pointer to Internal control.
 *
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVNoExec(const NMFieldDef_T *pFieldDef,
                             byte_t              buf[],
                             size_t              bufSize,
                             size_t              uFLen,
                             NMCtl_T            *pCtl)
{
  if( uFLen > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < flen=%zu", bufSize, uFLen);
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, uFLen, pCtl, "ignore");

  return (int)uFLen;
}

/*!
 * \brief Unpack pad byte from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVPad(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int   n = 0;

  // skip over all pad bytes
  while( (buf[n] == NMFVAL_PAD) && (n < bufSize) )
  {
    ++n;
  }

  NMLIB_TRACE_FIELD(NULL, buf, n, pCtl, "pad");

  return n;
}

/*!
 * \brief Unpack an unsigned 8-bit boolean ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVBool(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_U8,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackBool(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%s",
        (*((bool_t *)pVal)? "true": "false"));

    return n;
  }
}

/*!
 * \brief Unpack an unsigned 8-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVU8(const NMFieldDef_T *pFieldDef,
                         byte_t              buf[],
                         size_t              bufSize,
                         void               *pVal,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_U8,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackU8(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhu", *((byte_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a signed 8-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVS8(const NMFieldDef_T *pFieldDef,
                         byte_t              buf[],
                         size_t              bufSize,
                         void               *pVal,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_S8,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackS8(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhd",
        *((signed char *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack an unsigned 16-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVU16(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_U16,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackU16(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hu", *((ushort_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a signed 16-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVS16(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_S16,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackS16(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hd", *((short *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack an unsigned 32-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVU32(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_U32,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackU32(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%u", *((uint_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a signed 32-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVS32(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_S32,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackS32(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%d", *((int *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack an unsigned 64-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVU64(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_U64,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackU64(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%llu",
        *((ulonglong_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a signed 64-bit ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVS64(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_S64,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackS64(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%lld", *((long long *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a 32-bit floating-point number ITV field from the message
 * buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVF32(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_F32,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackF32(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%f", *((float *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a 64-bit floating-point number ITV field from the message
 * buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVF64(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_F64,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackF64(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%e", *((double *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a 32-bit pointer ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVP32(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_P32,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackP32(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack a 64-bit pointer ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVP64(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code
  int       k;    // subbyte count/error code

  // header size
  n = pCtl->m_bNoHdr? 0: NMITV_FHDR_SIZE_SIMPLE;

  // not enough input buffer for header
  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fhdr_size=%u", bufSize, n);
  }

  // ignore this field
  else if( pCtl->m_bNoExec )
  {
    return nmUnpackITVNoExec(pFieldDef, buf, bufSize, (size_t)n+NMFVAL_LEN_P64,
                              pCtl);
  }

  // try to unpack value
  else if( (k = nmUnpackP64(&buf[n], bufSize-(size_t)n, pVal, eEndian)) < 0 )
  {
    return k;
  }

  // success
  else
  {
    n += k;

    NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

    return n;
  }
}

/*!
 * \brief Unpack variable length string ITV field from the message buffer.
 *
 * \note The null character is appended to the end of the unpacked value.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVString(const NMFieldDef_T *pFieldDef,
                             byte_t              buf[],
                             size_t              bufSize,
                             void               *pVal,
                             NMEndian_T          eEndian,
                             NMCtl_T            *pCtl)
{
  size_t    uCount;
  size_t    uMaxCount;
  size_t    uFSize;
  char     *sVal;
  int       n;

  n = NMITV_FHDR_SIZE_STRING;

  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu, fhdr_size=%u", bufSize, n);
  }

  // string subheader
  uCount      = (size_t)buf[2];

  // field definition and derived values
  uMaxCount   = pFieldDef->m_this.m_string.m_uMaxCount;
  uFSize      = (size_t)n + uCount;
  sVal        = (char *)pVal;

  // exceeds maximum string length (sans null)
  if( uCount > uMaxCount )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_RANGE, pFieldDef,
        "string_count=%zu > max_count=%zu.", uCount, uMaxCount);
  }

  // not enough space in buffer for string
  else if( uFSize > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < fsize=%zu", bufSize, uFSize);
  }

  // success
  else
  {
    memcpy(sVal, buf+n, uCount);
    sVal[uCount] = 0;

    NMLIB_TRACE_FIELD(pFieldDef, buf, uFSize, pCtl, "'%s'", sVal);

    return (int)uFSize;
  }
}


/*!
 * \brief Unpack structure ITV field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVStruct(const NMFieldDef_T *pFieldDef,
                             byte_t              buf[],
                             size_t              bufSize,
                             void               *pVal,
                             NMEndian_T          eEndian,
                             NMCtl_T            *pCtl)
{
  const NMMsgDef_T *pStructDef;   // structure definition
  size_t            uCount;       // struct field count
  int               n;            // byte count/error code
  int               k;            // subbyte count/error code

  n = NMITV_FHDR_SIZE_STRUCT;

  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu, fhdr_size=%u", bufSize, n);
  }

  // number of fields in struct
  uCount = (size_t)buf[2];  // fix

  pStructDef  = pFieldDef->m_this.m_struct;

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "structdef %s, member_count=%u",
      pStructDef->m_sMsgName, buf[2]); //pStructDef->m_uCount);

  pCtl->m_uDepth++;

  // copy struct field value
  k = nmUnpackITVStream(pStructDef, &buf[n], bufSize-(size_t)n, uCount,
                                  pVal, eEndian, pCtl);

  pCtl->m_uDepth--;

  return k>=0? n+k: k;
}

/*!
 * \brief Unpack variable length vector ITV field from the message buffer.
 *
 * A vector is a one-dimension array of the given type.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVVector(const NMFieldDef_T *pFieldDef,
                             byte_t              buf[],
                             size_t              bufSize,
                             void               *pVal,
                             NMEndian_T          eEndian,
                             NMCtl_T            *pCtl)
{
  size_t              uCount;
  byte_t              byElemFType;
  size_t              uMaxCount;
  size_t              uElemSize;
  const NMFieldDef_T *pElemDef;
  NMVec_T            *pVec;
  void               *pItem;
  NMUnpackFunc_T      fnUnpack = NULL;
  int                 n = NMITV_FHDR_SIZE_VECTOR;
  int                 k;
  size_t              i;

  n = NMITV_FHDR_SIZE_VECTOR;

  if( n > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu, fhdr_size=%u", bufSize, n);
  }

  // vector subheader 
  uCount      = (size_t)buf[2];
  byElemFType = (byte_t)buf[3];

  // field definition and derived values
  uMaxCount = pFieldDef->m_this.m_vector.m_uMaxCount;
  uElemSize = pFieldDef->m_this.m_vector.m_uElemSize;
  pElemDef  = pFieldDef->m_this.m_vector.m_pThisElem;
  pVec      = (NMVec_T *)pVal;                    // generalized vector overlay
  pItem     = (byte_t *)(pVec) + memberoffset(NMVec_T, u.m_buf);
  fnUnpack  = nmLookupITVUnpacker(pElemDef->m_eFType);

  // received vector type mismatch with definition
  if( byElemFType != (byte_t)pElemDef->m_eFType )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_EMSG, pFieldDef,
           "vector_elem_ftype='%c'(0x%02x) != rcv'd ftype='%c'(0x%02x).",
           NMLIB_ASCII_FTYPE(pElemDef->m_eFType), pElemDef->m_eFType,
           NMLIB_ASCII_FTYPE(byElemFType), byElemFType);
  }

  // exceeds maximum
  else if( uCount > uMaxCount )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_RANGE, pFieldDef,
        "vector_count=%zu > max_count=%zu.", uCount, uMaxCount);
  }

  // no unpacker
  else if( fnUnpack == NULL )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_FTYPE, pFieldDef,
        "vector_elem_ftype='%c'(0x%02x).",
        NMLIB_ASCII_FTYPE(pElemDef->m_eFType), pElemDef->m_eFType);
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl,
        "vector_count=%zu, vector_elem_ftype='%c'(0x%02x)",
         uCount, NMLIB_ASCII_FTYPE(byElemFType), byElemFType);

  // no header for simple vector element types
  if( NMFTYPE_IS_SIMPLE(pElemDef->m_eFType) )
  {
    pCtl->m_bNoHdr = true;
  }

  pCtl->m_uDepth++;

  // unpack field into vector field
  for(i=0; i<uCount; ++i)
  {
    k = fnUnpack(pElemDef, &buf[n], bufSize-(size_t)n, pItem, eEndian, pCtl);

    if( k < 0 )
    {
      return k;
    }

    n += k;

    pItem += uElemSize;
  }

  pCtl->m_uDepth--;
  pCtl->m_bNoHdr = false;

  // actual number of vector items
  pVec->m_uCount = n>0? uCount: 0;

  return n;
}

/*!
 * \brief Unpack an ITV byte stream.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message
 *                        structure to populate.
 * \param uCount          Received message/struct field count
 * \param eEndian         Unpacking order. See \ref NMEndian_T.
 * \param pCtl            Pointer to Internal control.
 *
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackITVStream(const NMMsgDef_T *pMsgDef,
                             byte_t            buf[],
                             size_t            uMsgLen,
                             size_t            uCount,
                             void             *pStruct,
                             NMEndian_T        eEndian,
                             NMCtl_T          *pCtl)
{
  const NMFieldDef_T *pFieldDef;
  NMUnpackFunc_T      fnUnpack = NULL;
  byte_t              bNoExec;
  byte_t              byFId;
  byte_t              byFType;
  void               *pVal;
  int                 n = 0;
  int                 k;

  bNoExec = pCtl->m_bNoExec;

  while( (uCount > 0) && (n+NMITV_FID_SIZE < uMsgLen) )
  {
    byFId = buf[n]; // field id

    // Padding (reserved packed id)
    if( byFId ==  NMFTypePadTr )
    {
      byFType   = (byte_t)NMFTypePad; // internal pad field type
      pFieldDef = NULL;
      pVal      = NULL;
    }

    // ITV field
    else
    {
      if( n+NMITV_FHDR_SIZE_BASE >= uMsgLen )
      {
        NMLIB_RAISE_ERROR(NM_ECODE_NOMEM,
                "msg_len=%zu, msg_id=%u", uMsgLen, pMsgDef->m_eMsgId);
      }

      byFType = buf[n+1]; // field type

      if( (pFieldDef = nmFindFieldDef(pMsgDef, byFId)) == NULL )
      {
        pCtl->m_bNoExec = true;
        pVal = NULL;
        NMLIB_WARNING("Unknown fid=%u @byte=%d - ignoring.", (uint_t)byFId, n);
      }
      else if( pFieldDef->m_eFType != (NMFType_T)byFType )
      {
        NMLIB_RAISE_ERROR(NM_ECODE_EMSG,
           "Field definition ftype='%c'(0x%02x) != receive ftype='%c'(0x%02x).",
           NMLIB_ASCII_FTYPE(pFieldDef->m_eFType), pFieldDef->m_eFType,
           NMLIB_ASCII_FTYPE(byFType), byFType);
      }

      pVal = pStruct + pFieldDef->m_uOffset;
    }
    
    fnUnpack  = nmLookupITVUnpacker(byFType);

    if( fnUnpack == NULL )
    {
      NMLIB_RAISE_FIELD_ERROR(NM_ECODE_FTYPE, pFieldDef,
               "ftype='%c'(0x%02x).",
                NMLIB_ASCII_FTYPE(pFieldDef->m_eFType), pFieldDef->m_eFType);
    }

    k = fnUnpack(pFieldDef, &buf[n], uMsgLen-(size_t)n, pVal, eEndian, pCtl);

    if( k < 0 )
    {
      return k;
    }

    n += k;

    pCtl->m_bNoExec = bNoExec;

    uCount--;
  }

  if( uCount > 0 )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_EMSG,
                "%s(%u): %u field(s) left unparsed - no more message",
                pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, uCount);
  }

  return n;
}


// ...........................................................................
// Private Support Functions
// ...........................................................................

/*!
 * ITV Info Lookup Table
 *
 * \warning Keep in asending order by ftype.
 */
static NMLookupTblEntry_T NMITVLookupTbl[] =
{
  {NMFTypeBool,       nmPackITVBool,    nmUnpackITVBool},         //  [0] '?'
  {NMFTypeU8,         nmPackITVU8,      nmUnpackITVU8},           //  [1] 'B'
  {NMFTypeF64,        nmPackITVF64,     nmUnpackITVF64},          //  [2] 'F'
  {NMFTypeU16,        nmPackITVU16,     nmUnpackITVU16},          //  [3] 'H'
  {NMFTypeU32,        nmPackITVU32,     nmUnpackITVU32},          //  [4] 'I'
  {NMFTypeP64,        nmPackITVP64,     nmUnpackITVP64},          //  [5] 'P'
  {NMFTypeU64,        nmPackITVU64,     nmUnpackITVU64},          //  [6] 'Q'
  {NMFTypeVector,     nmPackITVVector,  nmUnpackITVVector},       //  [7] '['
  {NMFTypeS8,         nmPackITVS8,      nmUnpackITVS8},           //  [8] 'b'
  {NMFTypeChar,       nmPackITVU8,      nmUnpackITVU8},           //  [9] 'c'
  {NMFTypeF32,        nmPackITVF32,     nmUnpackITVF32},          // [10] 'f'
  {NMFTypeS16,        nmPackITVS16,     nmUnpackITVS16},          // [11] 'h'
  {NMFTypeS32,        nmPackITVS32,     nmUnpackITVS32},          // [12] 'i'
  {NMFTypeP32,        nmPackITVP32,     nmUnpackITVP32},          // [13] 'p'
  {NMFTypeS64,        nmPackITVS64,     nmUnpackITVS64},          // [14] 'q'
  {NMFTypeString,     nmPackITVString,  nmUnpackITVString},       // [15] 's'
  {NMFTypePad,        nmPackITVPad,     nmUnpackITVPad},          // [16] 'x'
  {NMFTypeStruct,     nmPackITVStruct,  nmUnpackITVStruct}        // [17] '{'
};

/*!
 * \brief Lookup ITV packer function, given the message field type.
 *
 * \param eFType  Message field type.
 *
 * \return Returns packer function on success, NULL on failure.
 */
static NMPackFunc_T nmLookupITVPacker(NMFType_T eFType)
{
  int idx;
 
  idx = NMHashFType(eFType);

  if( (idx != NMHashNoIdx) && (idx < arraysize(NMITVLookupTbl)) )
  {
    if( NMITVLookupTbl[idx].m_eFType == eFType )
    {
      return NMITVLookupTbl[idx].m_fnPack;
    }
    else
    {
      NMLIB_ERROR(NM_ECODE_INTERNAL,
        "hashed eFtype='%c'(0x%02x) != tbl[%d].m_eFType='%c'(0x%02x)",
        NMLIB_ASCII_FTYPE(eFType), eFType,
        idx, NMLIB_ASCII_FTYPE(NMITVLookupTbl[idx].m_eFType),
        NMITVLookupTbl[idx].m_eFType);
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Lookup ITV unpacker function, given the message field type.
 *
 * \param eFType  Message field type.
 *
 * \return Returns unpacker function on success, NULL on failure.
 */
static NMUnpackFunc_T nmLookupITVUnpacker(NMFType_T eFType)
{
  int idx;
 
  idx = NMHashFType(eFType);

  if( (idx != NMHashNoIdx) && (idx < arraysize(NMITVLookupTbl)) )
  {
    if( NMITVLookupTbl[idx].m_eFType == eFType )
    {
      return NMITVLookupTbl[idx].m_fnUnpack;
    }
    else
    {
      NMLIB_ERROR(NM_ECODE_INTERNAL,
        "hashed eFtype='%c'(0x%02x) != tbl[%d].m_eFType='%c'(0x%02x)",
        NMLIB_ASCII_FTYPE(eFType), eFType,
        idx, NMLIB_ASCII_FTYPE(NMITVLookupTbl[idx].m_eFType),
        NMITVLookupTbl[idx].m_eFType);
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Pack an ITV message.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param eEndian         Packing order. See \ref NMEndian_T.
 *
 * \copydoc doc_return_pack_std 
 */
int nmPackITVMsg(const NMMsgDef_T  *pMsgDef,
                 void              *pStruct,
                 byte_t             buf[],
                 size_t             bufSize,
                 NMEndian_T         eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code
  int       k;      // subbyte count/error code

  // too small
  if( bufSize < NMITV_MSGHDR_SIZE )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_NOMEM, "%s(%u): msg_len=%u, hdr_size=%u",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, bufSize, NMITV_MSGHDR_SIZE);
  }

  // pack message id
  if( (n = nmPackU16((ushort)(pMsgDef->m_eMsgId), buf, bufSize, eEndian)) < 0 )
  {
    return n;
  }

  // pack field count
  buf[n++] = (byte_t)pMsgDef->m_uCount;

  // pack stream
  k = nmPackITVStream(pMsgDef, pStruct, &buf[n], bufSize-(size_t)n, eEndian,
                                &ctl);

  return k >= 0? n+k: k;
}

/*!
 * \brief Pack an ITV message, tracing message packing to stderr.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] pStruct    Pointer to the associated, populated message
 *                        structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 * \param eEndian         Packing order. See \ref NMEndian_T.
 *
 * \copydoc doc_return_pack_std 
 */
int nmPackITVMsgDebug(const NMMsgDef_T *pMsgDef,
                      void             *pStruct,
                      byte_t            buf[],
                      size_t            bufSize,
                      NMEndian_T        eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code
  int       k;      // subbyte count/error code

  ctl.m_bTrace  = true;

  // too small
  if( bufSize < NMITV_MSGHDR_SIZE )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_NOMEM, "%s(%u): msg_len=%u, hdr_size=%u",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, bufSize, NMITV_MSGHDR_SIZE);
  }

  // pack message id
  if( (n = nmPackU16((ushort)(pMsgDef->m_eMsgId), buf, bufSize, eEndian)) < 0 )
  {
    return n;
  }

  // pack field count
  buf[n++] = (byte_t)pMsgDef->m_uCount;

  // trace
  fprintf(stderr, "\n--- Packing ITV Message %s(%u): field_count=%zu\n",
                  pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, pMsgDef->m_uCount);

  // pack stream
  k = nmPackITVStream(pMsgDef, pStruct, &buf[n], bufSize-(size_t)n, eEndian,
                                &ctl);

  if( k < 0 )
  {
    return k;
  }

  n += k;

  // trace final buffer
  fprintf(stderr, "Output Buffer (%zu bytes):\n", (size_t)n);
  nmPrintBuf(stderr, NULL, buf, (size_t)n, 16, 0);
  fprintf(stderr, "\n");

  return n;
}

/*!
 * \brief Unpack a ITV message.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message
 *                        structure to populate.
 * \param eEndian         Unpacking order. See \ref NMEndian_T.
 *
 * \returns 
 * On success, returns the number of bytes unpacked.
 * On error, returns \h_lt 0.
 */
int nmUnpackITVMsg(const NMMsgDef_T  *pMsgDef,
                   byte_t             buf[],
                   size_t             uMsgLen,
                   void              *pStruct,
                   NMEndian_T         eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  ushort_t  eMsgId; // message id in buffer
  size_t    uCount; // field count in buffer
  int       n;      // byte count/error code
  int       k;      // subbyte count/error code

  // too small
  if( uMsgLen < NMITV_MSGHDR_SIZE )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_NOMEM, "%s(%u): msg_len=%u, hdr_size=%u",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, uMsgLen, NMITV_MSGHDR_SIZE);
  }

  // unpack message id
  if( (n = nmUnpackU16(buf, uMsgLen, &eMsgId, eEndian)) < 0 )
  {
    return n;
  }

  // field count
  uCount = (size_t)buf[n++];

  // mismatch error
  if( (uint_t)(pMsgDef->m_eMsgId) != (uint_t)eMsgId )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_MSGID, "%s(%u): rcv'd msgid=%u != msgdef's",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, eMsgId);
  }

  // unpack stream
  k = nmUnpackITVStream(pMsgDef, &buf[n], uMsgLen-(size_t)n, uCount,
                            pStruct, eEndian, &ctl);

  // return code
  return k>=0? n+k: k;
}

/*!
 * \brief Unpack a ITV message, tracing unpacking to stderr.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message
 *                        structure to populate.
 * \param eEndian         Unpacking order. See \ref NMEndian_T.
 *
 * \copydoc doc_return_unpack_std 
 */
int nmUnpackITVMsgDebug(const NMMsgDef_T *pMsgDef,
                        byte_t            buf[],
                        size_t            uMsgLen,
                        void             *pStruct,
                        NMEndian_T        eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  ushort_t  eMsgId; // message id in buffer
  size_t    uCount; // field count in buffer
  int       n;      // byte count/error code
  int       k;      // subbyte count/error code

  ctl.m_bTrace  = true;

  // too small
  if( uMsgLen < NMITV_MSGHDR_SIZE )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_NOMEM, "%s(%u): msg_len=%u, hdr_size=%u",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, uMsgLen, NMITV_MSGHDR_SIZE);
  }

  // unpack message id
  if( (n = nmUnpackU16(buf, uMsgLen, &eMsgId, eEndian)) < 0 )
  {
    return n;
  }

  // field count
  uCount = (size_t)buf[n++];

  // mismatch error
  if( (uint_t)(pMsgDef->m_eMsgId) != (uint_t)eMsgId )
  {
    NMLIB_RAISE_ERROR(NM_ECODE_MSGID, "%s(%u): rcv'd msgid=%u != msgdef's",
            pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, eMsgId);
  }

  // trace
  fprintf(stderr, "\n--- Unpacking ITV Message %s(%u): field_count=%zu\n",
                  pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, uCount);
  fprintf(stderr, "Input Buffer (%zu bytes):\n", uMsgLen);
  nmPrintBuf(stderr, NULL, buf, uMsgLen, 16, 0);
  fprintf(stderr, "\n");

  // unpack stream
  k = nmUnpackITVStream(pMsgDef, &buf[n], uMsgLen-(size_t)n, uCount,
                            pStruct, eEndian, &ctl);

  // return code
  return k>=0? n+k: k;
}

/*!
 * \brief Get ITV message id from input buffer.
 *
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param eEndian         Unpacking order. See \ref NMEndian_T.
 *
 * \return
 * On success, returns unpacked message id.\n
 * On error, returns \ref NMMSG_ID_NONE (0).
 */
uint_t nmGetITVMsgId(byte_t buf[], size_t uMsgLen, NMEndian_T eEndian)
{
  ushort_t  eMsgId; // message id in buffer
  int       n;

  // too small
  if( uMsgLen < NMITV_MSGHDR_SIZE )
  {
    return (uint_t)NMMSG_ID_NONE;
  }
  
  // unpack message id
  else if( (n = nmUnpackU16(buf, uMsgLen, &eMsgId, eEndian)) < 0 )
  {
    return (uint_t)NMMSG_ID_NONE;
  }

  else
  {
    return (uint_t)eMsgId;
  }
}
