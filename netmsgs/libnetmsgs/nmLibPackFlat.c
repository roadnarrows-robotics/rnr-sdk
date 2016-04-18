////////////////////////////////////////////////////////////////////////////////
//
// Package:   botsense
//
// Library:   libBotSense
//
// File:      bsLibPackFlat.c
//
/*! \file
 *
 * $LastChangedDate: 2010-02-16 08:12:53 -0700 (Tue, 16 Feb 2010) $
 * $Rev: 248 $
 *
 * \brief Flat message packing/unpacking definitions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010  RoadNarrows LLC.
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


// ...........................................................................
// Private Packing Interface 
// ...........................................................................

//
// Forward Declarations
//
static NMPackFunc_T nmLookupFlatPacker(NMFType_T eFType);
static NMUnpackFunc_T nmLookupFlatUnpacker(NMFType_T eFType);
static int nmPackFlatStream(const NMMsgDef_T *pMsgDef, void *pStruct,
                           byte_t buf[], size_t bufSize, NMEndian_T eEndian,
                           NMCtl_T *pCtl);
static int nmUnpackFlatStream(const NMMsgDef_T *pMsgDef, byte_t buf[],
                             size_t uMsgLen, void *pStruct,
                             NMEndian_T eEndian, NMCtl_T *pCtl);


// ...........................................................................
// Private Packing Interface 
// ...........................................................................

/*!
 * \brief Pack pad bytes into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatPad(const NMFieldDef_T *pFieldDef,
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
 * \brief Pack a boolean into an 8-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatBool(const NMFieldDef_T *pFieldDef,
                          void               *pVal,
                          byte_t              buf[],
                          size_t              bufSize,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  bool_t  val;    // constrained field value
  int     n;      // byte count/error code

  val = *((bool_t *)pVal);

  // pack field value
  n = nmPackBool(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%s", (val? "true": "false"));

  return n;
}

/*!
 * \brief Pack an unsigned 8-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatU8(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  byte_t  val;    // constrained field value
  int     n;      // byte count/error code

  // try to set field value
  if( (n = nmSetU8(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackU8(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhu", val);

  return n;
}

/*!
 * \brief Pack a signed 8-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatS8(const NMFieldDef_T *pFieldDef,
                        void               *pVal,
                        byte_t              buf[],
                        size_t              bufSize,
                        NMEndian_T          eEndian,
                        NMCtl_T            *pCtl)
{
  signed char val;  // constrained field value
  int         n;    // byte count/error code

  // try to set field value
  if( (n = nmSetS8(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackS8(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhd", val);

  return n;
}

/*!
 * \brief Pack an unsigned 16-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatU16(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  ushort_t  val;  // constrained field value
  int       n;    // byte count/error code

  // try to set field value
  if( (n = nmSetU16(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackU16(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hu", val);

  return n;
}

/*!
 * \brief Pack a signed 16-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatS16(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  short   val;  // constrained field value
  int     n;    // byte count/error code

  // set field value
  if( (n = nmSetS16(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackS16(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hd", val);

  return n;
}

/*!
 * \brief Pack an unsigned 32-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatU32(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  uint_t  val;  // constrained field value
  int     n;    // byte count/error code

  // set field value
  if( (n = nmSetU32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackU32(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%u", val);

  return n;
}

/*!
 * \brief Pack a signed 32-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatS32(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  int   val;  // constrained field value
  int   n;    // byte count/error code

  // set field value
  if( (n = nmSetS32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackS32(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%d", val);

  return n;
}

/*!
 * \brief Pack an unsigned 64-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatU64(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  ulonglong_t val;  // constrained field value
  int         n;    // byte count/error code

  // set field value
  if( (n = nmSetU64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackU64(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%llu", val);

  return n;
}

/*!
 * \brief Pack a signed 64-bit Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatS64(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  long long val;  // constrained field value
  int       n;    // byte count/error code

  // set field value
  if( (n = nmSetS64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackS64(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%lld", val);

  return n;
}

/*!
 * \brief Pack a 32-bit floating-point number Flat field into the message
 * buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatF32(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  float   val;  // constrained field value
  int     n;    // byte count/error code

  // set field value
  if( (n = nmSetF32(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackF32(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%f", val);

  return n;
}

/*!
 * \brief Pack a 64-bit floating-point number Flat field into the message
 * buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatF64(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  double  val;  // constrained field value
  int     n;    // byte count/error code

  // set field value
  if( (n = nmSetF64(pFieldDef, pVal, &val)) != NM_OK )
  {
    return n;
  }

  // pack field value
  n = nmPackF64(val, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%e", val);

  return n;
}

/*!
 * \brief Pack a 32-bit pointer Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatP32(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // pack field value
  n = nmPackP32(pVal, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

  return n;
}

/*!
 * \brief Pack a 64-bit pointer Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatP64(const NMFieldDef_T *pFieldDef,
                         void               *pVal,
                         byte_t              buf[],
                         size_t              bufSize,
                         NMEndian_T          eEndian,
                         NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // pack field value
  n = nmPackP64(pVal, buf, bufSize, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

  return n;
}

/*!
 * \brief Pack null-terminated, fixed length string Flat field into the
 * message buffer.
 *
 * \note The null character is not packed.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatString(const NMFieldDef_T  *pFieldDef,
                            void                *pVal,
                            byte_t               buf[],
                            size_t               bufSize,
                            NMEndian_T           eEndian,
                            NMCtl_T             *pCtl)
{
  const char   *sVal;       // string field value pointer
  const char   *sConst;     // string constant value, if any
  size_t        uMaxCount;  // string maximum packed size
  size_t        uCount;     // string length

  sVal      = (const char *)pVal;                     // override void *
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

  // no more space in buffer for string
  else if( uMaxCount > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < str_max_len=%zu.", bufSize, uMaxCount);
  }

  // copy string field value
  memcpy(buf, sVal, uCount);

  // pad with zeros
  if( uCount < uMaxCount )
  {
    memset(&buf[uCount], 0, uMaxCount-uCount);
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, uMaxCount, pCtl, "'%s'", sVal);

  return (int)uMaxCount;
}

/*!
 * \brief Pack a structure Flat field into the message buffer.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatStruct(const NMFieldDef_T *pFieldDef,
                            void               *pVal,
                            byte_t              buf[],
                            size_t              bufSize,
                            NMEndian_T          eEndian,
                            NMCtl_T            *pCtl)
{
  const NMMsgDef_T *pStructDef;   // pointer to structure message definition
  int               n;            // byte count/error code

  pStructDef  = pFieldDef->m_this.m_struct;

  NMLIB_TRACE_FIELD(pFieldDef, buf, 0, pCtl, "structdef %s, member_count=%zu",
      pStructDef->m_sMsgName, pStructDef->m_uCount);

  pCtl->m_uDepth++;

  // copy struct field value
  n = nmPackFlatStream(pStructDef, pVal, buf, bufSize, eEndian, pCtl);

  pCtl->m_uDepth--;

  return n;
}

/*!
 * \brief Pack variable length vector Flat field into the message buffer.
 *
 * A vector is a one-dimension array of the given type.
 *
 * \copydoc doc_params_pack_fdef
 * \copydoc doc_return_pack_std 
 */
static int nmPackFlatVector(const NMFieldDef_T *pFieldDef,
                            void               *pVal,
                            byte_t              buf[],
                            size_t              bufSize,
                            NMEndian_T          eEndian,
                            NMCtl_T            *pCtl)
{
  size_t              uMaxCount;          // vector maximum number of elements
  size_t              uElemSize;          // vector element size
  const NMFieldDef_T *pElemDef;           // vector element field definition
  NMVec_T            *pVec;               // pointer to vector field template
  size_t              uCount;             // number of vector items to pack
  byte_t             *pItem;              // working vector item pointer
  NMPackFunc_T        fnPack;             // vector item packing function
  int                 n;                  // byte count/error code
  int                 k;                  // subbyte count/error code
  size_t              i;                  // working vector index

  uMaxCount = pFieldDef->m_this.m_vector.m_uMaxCount;
  uElemSize = pFieldDef->m_this.m_vector.m_uElemSize;
  pElemDef  = pFieldDef->m_this.m_vector.m_pThisElem;
  pVec      = (NMVec_T *)pVal;                    // generalized vector overlay
  uCount    = pVec->m_uCount;                     // number of vector items
  pItem     = (byte_t *)(pVec) + memberoffset(NMVec_T, u.m_buf);
  fnPack    = nmLookupFlatPacker(pElemDef->m_eFType);

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
        "vector_count=%zu > vector_max_count=%zu.", uCount, uMaxCount);
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, 0, pCtl, "%c:vector[%zu], max_count=%zu",
      pElemDef->m_eFType, uCount, uMaxCount);

  // no header for simple vector element types
  if( NMFTYPE_IS_SIMPLE(pElemDef->m_eFType) )
  {
    pCtl->m_bNoHdr = true;
  }

  pCtl->m_uDepth++;

  // copy vector field value
  for(i=0, n=0; i<uCount; ++i)
  {
    k = fnPack(pElemDef, pItem, &buf[n], bufSize-(size_t)n, eEndian, pCtl);

    if( k < 0 )
    {
      return k;
    }

    n += k;

    pItem += uElemSize;
  }

  // pad out fixed size vector with zero's.
  if( uCount < uMaxCount )
  {
    i = (uMaxCount - uCount) * uElemSize;

    if( i > bufSize-(size_t)n)
    {
      NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu < vector_pad_size=%zu", bufSize-(size_t)n, i);
    }

    else
    {
      memset(&buf[n], 0, i);
      n += (int)i;
    }
  }

  pCtl->m_uDepth--;
  pCtl->m_bNoHdr = false;

  return n;
}

/*!
 * \brief Pack a Flat message.
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
static int nmPackFlatStream(const NMMsgDef_T *pMsgDef,
                            void             *pStruct,
                            byte_t            buf[],
                            size_t            bufSize,
                            NMEndian_T        eEndian,
                            NMCtl_T          *pCtl)
{
  size_t              uFieldNum;        // field instance number (not id)
  const NMFieldDef_T *pFieldDef;        // working field definition pointer
  NMPackFunc_T        fnPack;           // field packing function
  void               *pVal;             // pointer to field value
  int                 n = 0;            // byte count/error code
  int                 k;                // subbyte count/error code

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
        pVal    = NULL;
      }

      else
      {
        pVal    = pStruct + pFieldDef->m_uOffset;
      }

      fnPack  = nmLookupFlatPacker(pFieldDef->m_eFType);

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
 * \brief Unpack pad byte from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatPad(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  size_t    uCount = pFieldDef->m_this.m_pad.m_uCount;

  // unpack pad bytes
  if( uCount <= bufSize )
  {
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
 * \brief Unpack an unsigned 8-bit boolean Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatBool(const NMFieldDef_T *pFieldDef,
                            byte_t              buf[],
                            size_t              bufSize,
                            void               *pVal,
                            NMEndian_T          eEndian,
                            NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackBool(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%s",
        (*((bool_t *)pVal)? "true": "false"));

  return n;
}

/*!
 * \brief Unpack an unsigned 8-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatU8(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackU8(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhu", *((byte_t *)pVal));

  return n;
}

/*!
 * \brief Unpack a signed 8-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatS8(const NMFieldDef_T *pFieldDef,
                          byte_t              buf[],
                          size_t              bufSize,
                          void               *pVal,
                          NMEndian_T          eEndian,
                          NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackS8(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hhd", *((signed char *)pVal));

  return n;
}

/*!
 * \brief Unpack an unsigned 16-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatU16(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackU16(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hu", *((ushort_t *)pVal));

 return n;
}

/*!
 * \brief Unpack a signed 16-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatS16(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackS16(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%hd", *((short *)pVal));

  return n;
}

/*!
 * \brief Unpack an unsigned 32-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatU32(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackU32(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%u", *((uint_t *)pVal));

  return n;
}

/*!
 * \brief Unpack a signed 32-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatS32(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackS32(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%d", *((int *)pVal));

  return n;
}

/*!
 * \brief Unpack an unsigned 64-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatU64(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackU64(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%llu", *((ulonglong_t *)pVal));

  return n;
}

/*!
 * \brief Unpack a signed 64-bit Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatS64(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackS64(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%lld", *((long long *)pVal));

  return n;
}

/*!
 * \brief Unpack a 32-bit floating-point number Flat field from the message
 * buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatF32(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackF32(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%f", *((float *)pVal));

  return n;
}

/*!
 * \brief Unpack a 64-bit floating-point number Flat field from the message
 * buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatF64(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackF64(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%e", *((double *)pVal));

  return n;
}

/*!
 * \brief Unpack a 32-bit pointer Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatP32(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackP32(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

  return n;
}

/*!
 * \brief Unpack a 64-bit pointer Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatP64(const NMFieldDef_T *pFieldDef,
                           byte_t              buf[],
                           size_t              bufSize,
                           void               *pVal,
                           NMEndian_T          eEndian,
                           NMCtl_T            *pCtl)
{
  int       n;    // byte count/error code

  // unpack value
  n = nmUnpackP64(buf, bufSize, pVal, eEndian);

  NMLIB_TRACE_FIELD(pFieldDef, buf, n, pCtl, "%p", *((ulong_t *)pVal));

  return n;
}

/*!
 * \brief Unpack variable length string Flat field from the message buffer.
 *
 * \note The null character is appended to the end of the unpacked value.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatString(const NMFieldDef_T *pFieldDef,
                              byte_t              buf[],
                              size_t              bufSize,
                              void               *pVal,
                              NMEndian_T          eEndian,
                              NMCtl_T            *pCtl)
{
  size_t    uMaxCount;    // maximum string length
  char     *sVal;         // string pointer

  // field definition and derived values
  uMaxCount   = pFieldDef->m_this.m_string.m_uMaxCount;
  sVal        = (char *)pVal;

  if( uMaxCount > bufSize )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_NOMEM, pFieldDef,
        "buf_size=%zu, max_count=%zu", bufSize, uMaxCount);
  }

  memcpy(sVal, buf, uMaxCount);
  sVal[uMaxCount] = 0;

  NMLIB_TRACE_FIELD(pFieldDef, buf, uMaxCount, pCtl, "'%s'", sVal);

  return (int)uMaxCount;
}


/*!
 * \brief Unpack structure Flat field from the message buffer.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatStruct(const NMFieldDef_T *pFieldDef,
                              byte_t              buf[],
                              size_t              bufSize,
                              void               *pVal,
                              NMEndian_T          eEndian,
                              NMCtl_T            *pCtl)
{
  const NMMsgDef_T *pStructDef;   // structure definition
  int               n;            // byte count/error code

  pStructDef  = pFieldDef->m_this.m_struct;

  NMLIB_TRACE_FIELD(pFieldDef, buf, 0, pCtl, "structdef %s, member_count=%zu",
      pStructDef->m_sMsgName, pStructDef->m_uCount);

  pCtl->m_uDepth++;

  // copy struct field value
  n = nmUnpackFlatStream(pStructDef, buf, bufSize, pVal, eEndian, pCtl);

  pCtl->m_uDepth--;

  return n;
}

/*!
 * \brief Unpack variable length vector Flat field from the message buffer.
 *
 * A vector is a one-dimension array of the given type.
 *
 * \copydoc doc_params_unpack_fdef
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatVector(const NMFieldDef_T *pFieldDef,
                              byte_t              buf[],
                              size_t              bufSize,
                              void               *pVal,
                              NMEndian_T          eEndian,
                              NMCtl_T            *pCtl)
{
  size_t              uMaxCount;          // vector maximum number of elements
  size_t              uElemSize;          // vector element size
  const NMFieldDef_T *pElemDef;           // vector element field definition
  NMVec_T            *pVec;               // pointer to vector field template
  byte_t             *pItem;              // working vector item pointer
  NMFType_T           eVType;             // vector itim field type
  NMUnpackFunc_T      fnUnpack;           // vector item unpacking function
  int                 n;                  // byte count/error code
  int                 k;                  // subbyte count/error code
  size_t              i;                  // working vector index

  // field definition and derived values
  uMaxCount = pFieldDef->m_this.m_vector.m_uMaxCount;
  uElemSize = pFieldDef->m_this.m_vector.m_uElemSize;
  pElemDef  = pFieldDef->m_this.m_vector.m_pThisElem;
  pVec      = (NMVec_T *)pVal;                    // generalized vector overlay
  pItem     = (byte_t *)(pVec) + memberoffset(NMVec_T, u.m_buf);
  eVType    = pElemDef->m_eFType;
  fnUnpack  = nmLookupFlatUnpacker(eVType);

  // no unpacker
  if( fnUnpack == NULL )
  {
    NMLIB_RAISE_FIELD_ERROR(NM_ECODE_FTYPE, pFieldDef,
        "vector_elem_ftype='%c'(0x%02x).",
        NMLIB_ASCII_FTYPE(pElemDef->m_eFType), eVType);
  }

  NMLIB_TRACE_FIELD(pFieldDef, buf, 0, pCtl,
        "vector_max_count=%zu, vector_elem_ftype='%c'(0x%02x)",
         uMaxCount, NMLIB_ASCII_FTYPE(eVType), eVType);

  pCtl->m_uDepth++;

  // unpack field into vector field
  for(i=0, n=0; i<uMaxCount; ++i)
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

  return n;
}

/*!
 * \brief Unpack an Flat byte stream.
 *
 * \param pMsgDef         Pointer to message definition.
 * \param [in] buf        Input message buffer.
 * \param uMsgLen         Length of message (bytes) in input buffer.
 * \param [out] pStruct   Pointer to the associated message
 *                        structure to populate.
 * \param eEndian         Unpacking order. See \ref NMEndian_T.
 * \param pCtl            Pointer to Internal control.
 *
 * \copydoc doc_return_unpack_std 
 */
static int nmUnpackFlatStream(const NMMsgDef_T *pMsgDef,
                              byte_t            buf[],
                              size_t            uMsgLen,
                              void             *pStruct,
                              NMEndian_T        eEndian,
                              NMCtl_T          *pCtl)
{
  size_t              uFieldNum;        // field instance number (not id)
  const NMFieldDef_T *pFieldDef;        // working field definition pointer
  NMUnpackFunc_T      fnUnpack;         // field unpack function
  void               *pVal;             // pointer to field value
  int                 n = 0;            // byte count/error code
  int                 k;                // subbyte count/error code

  for(uFieldNum=0, pFieldDef=pMsgDef->m_pFields;
      uFieldNum < pMsgDef->m_uCount;
      ++uFieldNum, ++pFieldDef)
  {
    if( n < uMsgLen )
    {
      if( pFieldDef->m_eFType == NMFTypePad )
      {
        pVal    = NULL;
      }

      else
      {
        pVal    = pStruct + pFieldDef->m_uOffset;
      }

      fnUnpack = nmLookupFlatUnpacker(pFieldDef->m_eFType);

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
    }

    else
    {
      NMLIB_RAISE_ERROR(NM_ECODE_NOMEM,
        "buf_size=%zu, msgid=%u", uMsgLen, pMsgDef->m_eMsgId);
    }

    n += k;
  }

  return n;
}


// ...........................................................................
// Private Support Functions
// ...........................................................................

/*!
 * Flat Info Lookup Table
 *
 * \warning Keep in asending order by ftype.
 */
static NMLookupTblEntry_T NMFlatLookupTbl[] =
{
  {NMFTypeBool,       nmPackFlatBool,     nmUnpackFlatBool},      //  [0] '?'
  {NMFTypeU8,         nmPackFlatU8,       nmUnpackFlatU8},        //  [1] 'B'
  {NMFTypeF64,        nmPackFlatF64,      nmUnpackFlatF64},       //  [2] 'F'
  {NMFTypeU16,        nmPackFlatU16,      nmUnpackFlatU16},       //  [3] 'H'
  {NMFTypeU32,        nmPackFlatU32,      nmUnpackFlatU32},       //  [4] 'I'
  {NMFTypeP64,        nmPackFlatP64,      nmUnpackFlatP64},       //  [5] 'P'
  {NMFTypeU64,        nmPackFlatU64,      nmUnpackFlatU64},       //  [6] 'Q'
  {NMFTypeVector,     nmPackFlatVector,   nmUnpackFlatVector},    //  [7] '['
  {NMFTypeS8,         nmPackFlatS8,       nmUnpackFlatS8},        //  [8] 'b'
  {NMFTypeChar,       nmPackFlatU8,       nmUnpackFlatU8},        //  [9] 'c'
  {NMFTypeF32,        nmPackFlatF32,      nmUnpackFlatF32},       // [10] 'f'
  {NMFTypeS16,        nmPackFlatS16,      nmUnpackFlatS16},       // [11] 'h'
  {NMFTypeS32,        nmPackFlatS32,      nmUnpackFlatS32},       // [12] 'i'
  {NMFTypeP32,        nmPackFlatP32,      nmUnpackFlatP32},       // [13] 'p'
  {NMFTypeS64,        nmPackFlatS64,      nmUnpackFlatS64},       // [14] 'q'
  {NMFTypeString,     nmPackFlatString,   nmUnpackFlatString},    // [15] 's'
  {NMFTypePad,        nmPackFlatPad,      nmUnpackFlatPad},       // [16] 'x'
  {NMFTypeStruct,     nmPackFlatStruct,   nmUnpackFlatStruct}     // [17] '{'
};

/*!
 * \brief Lookup Flat packer function, given the message field type.
 *
 * \param eFType  Message field type.
 *
 * \return Returns packer function on success, NULL on failure.
 */
static NMPackFunc_T nmLookupFlatPacker(NMFType_T eFType)
{
  int idx;
 
  idx = NMHashFType(eFType);

  if( (idx != NMHashNoIdx) && (idx < arraysize(NMFlatLookupTbl)) )
  {
    if( NMFlatLookupTbl[idx].m_eFType == eFType )
    {
      return NMFlatLookupTbl[idx].m_fnPack;
    }
    else
    {
      NMLIB_ERROR(NM_ECODE_INTERNAL,
        "hashed eFtype='%c'(0x%02x) != tbl[%d].m_eFType='%c'(0x%02x)",
        NMLIB_ASCII_FTYPE(eFType), eFType,
        idx, NMLIB_ASCII_FTYPE(NMFlatLookupTbl[idx].m_eFType),
        NMFlatLookupTbl[idx].m_eFType);
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Lookup Flat unpacker function, given the message field type.
 *
 * \param eFType  Message field type.
 *
 * \return Returns unpacker function on success, NULL on failure.
 */
static NMUnpackFunc_T nmLookupFlatUnpacker(NMFType_T eFType)
{
  int idx;
 
  idx = NMHashFType(eFType);

  if( (idx != NMHashNoIdx) && (idx < arraysize(NMFlatLookupTbl)) )
  {
    if( NMFlatLookupTbl[idx].m_eFType == eFType )
    {
      return NMFlatLookupTbl[idx].m_fnUnpack;
    }
    else
    {
      NMLIB_ERROR(NM_ECODE_INTERNAL,
        "hashed eFtype='%c'(0x%02x) != tbl[%d].m_eFType='%c'(0x%02x)",
        NMLIB_ASCII_FTYPE(eFType), eFType,
        idx, NMLIB_ASCII_FTYPE(NMFlatLookupTbl[idx].m_eFType),
        NMFlatLookupTbl[idx].m_eFType);
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
 * \brief Pack an Flat message.
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
int nmPackFlatMsg(const NMMsgDef_T  *pMsgDef,
                  void              *pStruct,
                  byte_t             buf[],
                  size_t             bufSize,
                  NMEndian_T         eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code

  ctl.m_bNoHdr  = true;

  // pack stream
  n = nmPackFlatStream(pMsgDef, pStruct, buf, bufSize, eEndian, &ctl);

  return n;
}

/*!
 * \brief Pack an Flat message, tracing message packing to stderr.
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
int nmPackFlatMsgDebug(const NMMsgDef_T *pMsgDef,
                       void             *pStruct,
                       byte_t            buf[],
                       size_t            bufSize,
                       NMEndian_T        eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code

  ctl.m_bNoHdr  = true;
  ctl.m_bTrace  = true;

  // trace
  fprintf(stderr, "\n--- Packing Flat Message %s(%u): field_count=%zu\n",
                  pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, pMsgDef->m_uCount);

  // pack stream
  n = nmPackFlatStream(pMsgDef, pStruct, buf, bufSize, eEndian, &ctl);

  // trace final buffer
  if( n > 0 )
  {
    fprintf(stderr, "Output Buffer (%zu bytes):\n", (size_t)n);
    nmPrintBuf(stderr, NULL, buf, (size_t)n, 16, 0);
    fprintf(stderr, "\n");
  }

  return n;
}

/*!
 * \brief Unpack a Flat message.
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
int nmUnpackFlatMsg(const NMMsgDef_T  *pMsgDef,
                    byte_t             buf[],
                    size_t             uMsgLen,
                    void              *pStruct,
                    NMEndian_T         eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code

  ctl.m_bNoHdr  = true;

  // unpack stream
  n = nmUnpackFlatStream(pMsgDef, buf, uMsgLen, pStruct, eEndian, &ctl);

  // return code
  return n;
}

/*!
 * \brief Unpack a Flat message, tracing unpacking to stderr.
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
int nmUnpackFlatMsgDebug(const NMMsgDef_T *pMsgDef,
                         byte_t            buf[],
                         size_t            uMsgLen,
                         void             *pStruct,
                         NMEndian_T        eEndian)
{
  NMCtl_T   ctl = NMCTL_INIT_DECL;
  int       n;      // byte count/error code

  ctl.m_bNoHdr  = true;
  ctl.m_bTrace  = true;

  // trace
  fprintf(stderr, "\n--- Unpacking Flat Message %s(%u): field_count=%zu\n",
                  pMsgDef->m_sMsgName, pMsgDef->m_eMsgId, pMsgDef->m_uCount);
  fprintf(stderr, "Input Buffer (%zu bytes):\n", uMsgLen);
  nmPrintBuf(stderr, NULL, buf, uMsgLen, 16, 0);
  fprintf(stderr, "\n");

  // unpack stream
  n = nmUnpackFlatStream(pMsgDef, buf, uMsgLen, pStruct, eEndian, &ctl);

  // return code
  return n;
}
