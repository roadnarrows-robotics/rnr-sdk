///////////////////////////////////////////////////////////////////////////////
//
// File: netmsgs.h
//
/*!
 * \file
 *
 * $LastChangedDate: 2010-07-31 08:48:56 -0600 (Sat, 31 Jul 2010) $
 * $Rev: 521 $
 *
 * \brief Network Messaging declarations.
 *
 * \note This file must be swig-able to generate a python extension module.
 * 
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010 RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * All Rights Reserved
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
///////////////////////////////////////////////////////////////////////////////


#ifndef _NETMSGS_H
#define _NETMSGS_H

#include <stdio.h>

#include "rnr/rnrconfig.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Base Defines and Types
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \ingroup man_libnetmsgs_ecodes NetMsgs Error Codes
 *
 * NetMsgs package-wide error codes.
 * \{
 */
#define NM_OK                       0 ///< not an error, success

#define NM_ECODE_GEN                1 ///< general, unspecified error
#define NM_ECODE_NOMEM              2 ///< not enough buffer memory available
#define NM_ECODE_ARCH_NOTSUP        3 ///< machine architecture not supported 
#define NM_ECODE_RANGE              4 ///< field size or value out-of-range
#define NM_ECODE_FTYPE              5 ///< unknown field type
#define NM_ECODE_EMSG               6 ///< bad message
#define NM_ECODE_MSGID              7 ///< bad/unknown message id

#define NM_ECODE_INTERNAL           8 ///< internal inconsistency or bug
#define NM_ECODE_BADEC              9 ///< internal inconsistency or bug
#define NM_ECODE_NUMOF             10 ///< number of error codes

/*! \} */

/*!
 * Message Encoding Type Enumeration
 */
typedef enum
{
  NMEncodingFlat,     ///< flat (no field header and fixed size) encoding
  NMEncodingITV,      ///< id-type-value encoding
  NMEncodingCLI,      ///< command-line interface encoding (future)
  NMEncodingNumOf     ///< number of different encodings supported
} NMEncoding_T;

/*!
 * Message Byte Ordering Type Enumeration
 */
typedef enum
{
  NMEndianBig,        ///< Most Significant Byte first
  NMEndianLittle,     ///< Least Significant Byte first
  NMEndianNative      ///< native processor order
} NMEndian_T;

/*!
 * Message Field Type Code Enumeration
 */
typedef enum
{
  NMFTypeNone       = 0,      ///< no field type            -
  NMFTypePadTr      = 0,      ///< translated pad type for packed stream

  // enum             value        message field            C data type
  // ----             -----        -------------            ------------
  NMFTypeBool         = '?',  ///< 8-bit bool 0/1           bool_t
  NMFTypeU8           = 'B',  ///< unsigned 8-bit integer   unsigned char
  NMFTypeByte         = 'B',  ///< U8 alias
  NMFTypeF64          = 'F',  ///< 64-bit IEEE 754 FPN      double
  NMFTypeDouble       = 'F',  ///< F64 alias
  NMFTypeU16          = 'H',  ///< unsigned 16-bit integer  unsigned short
  NMFTypeUShort       = 'H',  ///< U16 alias
  NMFTypeU32          = 'I',  ///< unsigned 32-bit integer  unsigned int
  NMFTypeUInt         = 'I',  ///< U32 alias
  NMFTypeP64          = 'P',  ///< 64-bit pointer           (overloaded) void*
  NMFTypeLongPointer  = 'P',  ///< P64 alias
  NMFTypeU64          = 'Q',  ///< unsigned 64-bit integer  unsigned long long
  NMFTypeULongLong    = 'Q',  ///< U64 alias
  NMFTypeVector       = '[',  ///< one dimensional array    VType[]
  NMFTypeS8           = 'b',  ///< signed 8-bit integer     signed char
  NMFTypeSChar        = 'b',  ///< S8 alias
  NMFTypeChar         = 'c',  ///< 8-bit ASCII characters   char
  NMFTypeF32          = 'f',  ///< 32-bit IEEE 754 FPN      float
  NMFTypeFloat        = 'f',  ///< F32 alias
  NMFTypeS16          = 'h',  ///< signed 16-bit integer    short
  NMFTypeShort        = 'h',  ///< S16 alias
  NMFTypeS32          = 'i',  ///< signed 32-bit integer    int
  NMFTypeInt          = 'i',  ///< S32 alias
  NMFTypeP32          = 'p',  ///< 32-bit pointer           (overloaded) void*
  NMFTypePointer      = 'p',  ///< P32 alias
  NMFTypeS64          = 'q',  ///< signed 64-bit integer    long long
  NMFTypeLongLong     = 'q',  ///< S64 alias
  NMFTypeString       = 's',  ///< string                   char[]
  NMFTypePad          = 'x',  ///< pad internal field type
  NMFTypeStruct       = '{',  ///< structure                struct T

  NMFTypeNumOf        = 18    ///< number of
} NMFType_T;

/*!
 * \brief Returns true if field type is a compoound type.
 *
 * \param ft  Field type.
 */
#define NMFTYPE_IS_COMPOUND(ft) \
  (((ft)==NMFTypeString) || ((ft)==NMFTypeStruct) || ((ft)==NMFTypeVector))

/*!
 * \brief Returns true if field type is a simple type.
 *
 * \param ft  Field type.
 */
#define NMFTYPE_IS_SIMPLE(ft) (!NMFTYPE_IS_COMPOUND(ft))


/*!
 * Message packed field value lengths in bytes.
 */
// base
#define NMFVAL_LEN_CHAR       1 ///< character field length
#define NMFVAL_LEN_U8         1 ///< unsigned 8-bit field value length
#define NMFVAL_LEN_S8         1 ///< signed 8-bit field value length
#define NMFVAL_LEN_BOOL       1 ///< boolean field length
#define NMFVAL_LEN_U16        2 ///< unsigned 16-bit field value length
#define NMFVAL_LEN_S16        2 ///< signed 16-bit field value length
#define NMFVAL_LEN_U32        4 ///< unsigned 32-bit field value length
#define NMFVAL_LEN_S32        4 ///< signed 32-bit field value length
#define NMFVAL_LEN_U64        8 ///< unsigned 64-bit field value length
#define NMFVAL_LEN_S64        8 ///< signed 64-bit field value length
#define NMFVAL_LEN_F32        4 ///< 32-bit floating-point number field val len
#define NMFVAL_LEN_F64        8 ///< 64-bit floating-point number field val len
#define NMFVAL_LEN_P32        4 ///< 32-bit pointer field value length
#define NMFVAL_LEN_P64        8 ///< 64-bit pointer field value length

// derived and aliases
#define NMFVAL_LEN_BYTE       NMFVAL_LEN_U8   ///< unsigned char field length
#define NMFVAL_LEN_SCHAR      NMFVAL_LEN_S8   ///< signed char field length
#define NMFVAL_LEN_USHORT     NMFVAL_LEN_U16  ///< unsigned short field length
#define NMFVAL_LEN_SHORT      NMFVAL_LEN_S16  ///< short field length
#define NMFVAL_LEN_UINT       NMFVAL_LEN_U32  ///< signed int field length
#define NMFVAL_LEN_INT        NMFVAL_LEN_U32  ///< unsigned int field length
#define NMFVAL_LEN_ULONGLONG  NMFVAL_LEN_U64  ///< signed long long field length
#define NMFVAL_LEN_LONGLONG   NMFVAL_LEN_U64  ///< unsigned long long field len
#define NMFVAL_LEN_FLOAT      NMFVAL_LEN_F32  ///< float field length
#define NMFVAL_LEN_DOUBLE     NMFVAL_LEN_F64  ///< double field length
#define NMFVAL_LEN_POINTER    NMFVAL_LEN_P32  ///< 4-byte void* field length
#define NMFVAL_LEN_LONGPOINTER NMFVAL_LEN_P64 ///< 8-byte void* field length

// complex
#define NMFVAL_LEN_STRING     0     ///< char[] variable field length
#define NMFVAL_LEN_MAX_STRING 255   ///< max string length (excluding null)
#define NMFVAL_LEN_STRUCT     0     ///< struct T variable field length
#define NMFVAL_LEN_VECTOR     0     ///< VType[[] variable field length
#define NMFVAL_LEN_MAX_VECTOR 255   ///< max vector field length (num of items)


#define NMFVAL_PAD  ((byte_t)NMFTypePadTr)  ///< pad field value same as tr id 

#define NMMSG_ID_NONE         0     ///< no message id (reserved)
#define NMFID_NONE            0     ///< no field id (reserved)

/*!
 * ITV Messge and Field encodings/decodings.
 */
#define NMITV_MSGID_SIZE      2  ///< 2 byte message id size
#define NMITV_FID_SIZE        1  ///< 1 byte field id size
#define NMITV_FTYPE_SIZE      1  ///< 1 byte field type size
#define NMITV_FCOUNT_SIZE     1  ///< 1 byte field count size

/*! message leading number of bytes for message header (msgid, fcount) */
#define NMITV_MSGHDR_SIZE       (NMITV_MSGID_SIZE + NMITV_FCOUNT_SIZE)

/*! number of bytes in header to identify and quatify field (fid, ftype) */
#define NMITV_FHDR_SIZE_BASE    (NMITV_FID_SIZE + NMITV_FTYPE_SIZE)

/*! number of bytes in simple field header (fid, ftype) */
#define NMITV_FHDR_SIZE_SIMPLE  NMITV_FHDR_SIZE_BASE

/*! number of bytes in string field header (fid, ftype, count) */
#define NMITV_FHDR_SIZE_STRING  (NMITV_FHDR_SIZE_BASE + NMITV_FCOUNT_SIZE)

/*! number of bytes in struct field header (fid, ftype, count) */
#define NMITV_FHDR_SIZE_STRUCT  (NMITV_FHDR_SIZE_BASE + NMITV_FCOUNT_SIZE)

/*! number of bytes in vector field header (fid, ftype, count, vtype) */
#define NMITV_FHDR_SIZE_VECTOR \
  (NMITV_FHDR_SIZE_BASE + NMITV_FCOUNT_SIZE + NMITV_FTYPE_SIZE)

//
// Exclude the following low-level C specific interface from python swig.
//
#ifndef SWIG

//
// Forward declarations and pointers
//
struct                _nm_field_def_struct;               ///< field def struct
struct                _nm_msg_def_struct;                 ///< msg def struct
typedef const struct  _nm_field_def_struct *NMFieldDef_P; ///< field def pointer
typedef const struct  _nm_msg_def_struct   *NMMsgDef_P;   ///< msg def pointer

/*
 * Bit flags used in definitions.
 */
#define NMBITS_HAS_MIN   0x01     ///< field has a minimum value
#define NMBITS_HAS_MAX   0x02     ///< field has a maximum value
#define NMBITS_HAS_CONST 0x04     ///< field has a constant value

/*!
 * This padding field definition specific information.
 */
typedef struct
{
  size_t  m_uCount;   ///< number of pad bytes
} NMFieldThisPad_T;

/*!
 * This A [un]signed 8-bit integer field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  byte_t      m_valMin;     ///< field minimum value constraint
  byte_t      m_valMax;     ///< field maximum value constraint
  byte_t      m_valConst;   ///< field constant value contraint
} NMFieldThisU8_T;

/*!
 * This A [un]signed 16-bit integer field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  ushort_t    m_valMin;     ///< field minimum value constraint
  ushort_t    m_valMax;     ///< field maximum value constraint
  ushort_t    m_valConst;   ///< field constant value contraint
} NMFieldThisU16_T;

/*!
 * This [un]signed 32-bit integer field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  uint_t      m_valMin;     ///< field minimum value constraint
  uint_t      m_valMax;     ///< field maximum value constraint
  uint_t      m_valConst;   ///< field constant value contraint
} NMFieldThisU32_T;

/*!
 * This [un]signed 64-bit integer field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  ulonglong_t m_valMin;     ///< field minimum value constraint
  ulonglong_t m_valMax;     ///< field maximum value constraint
  ulonglong_t m_valConst;   ///< field constant value contraint
} NMFieldThisU64_T;

/*!
 * This 32-bit floating-point number field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  float       m_valMin;     ///< field minimum value constraint
  float       m_valMax;     ///< field maximum value constraint
  float       m_valConst;   ///< field constant value contraint
} NMFieldThisF32_T;

/*!
 * This 64-bit floating-point number field definition specific information.
 */
typedef struct
{
  byte_t      m_bits;       ///< field constraint valid bits
  double      m_valMin;     ///< field minimum value constraint
  double      m_valMax;     ///< field maximum value constraint
  double      m_valConst;   ///< field constant value contraint
} NMFieldThisF64_T;

/*!
 * This string field definition specific information.
 */
typedef struct
{
  size_t      m_uMaxCount;  ///< maximum number of characters sans null char
  const char *m_sConst;     ///< string constant value (NULL if no const)
} NMFieldThisString_T;

/*!
 * This strcuture field definition specific information.
 * (equivalent to a submessage)
 */
typedef NMMsgDef_P NMFieldThisStruct_T;

/*!
 * This vector field definition specific information
 */
typedef struct
{
  size_t              m_uMaxCount;  ///< maximum number of vector elements
  size_t              m_uElemSize;  ///< vector element storage size
  NMFieldDef_P        m_pThisElem;  ///< vector element specific info
} NMFieldThisVector_T;

/*!
 * This field definition specific information.
 */
typedef union
{
  NMFieldThisPad_T    m_pad;      ///< pad field specific info
  NMFieldThisU8_T     m_u8;       ///< [unsigned] 8-bit integer specific info
  NMFieldThisU16_T    m_u16;      ///< [unsigned] 16-bit integer specific info
  NMFieldThisU32_T    m_u32;      ///< [unsigned] 32-bit integer specific info
  NMFieldThisU64_T    m_u64;      ///< [unsigned] 64-bit integer specific info
  NMFieldThisF32_T    m_f32;      ///< 32-bit fpn specific info
  NMFieldThisF64_T    m_f64;      ///< 64-bit fpn specific info
  NMFieldThisString_T m_string;   ///< string field specific info
  NMFieldThisStruct_T m_struct;   ///< struct field specific info
  NMFieldThisVector_T m_vector;   ///< vector field specific info
} NMFieldThis_T;

/*!
 * Message Field Definition Type
 */
typedef struct _nm_field_def_struct
{
  const char         *m_sFName;   ///< field string name
  uint_t              m_eFId;     ///< filed id (message/struct unique)
  NMFType_T           m_eFType;   ///< field type
  size_t              m_uOffset;  ///< member offset in assoc msg structure
  NMFieldThis_T       m_this;     ///< specific field information
} NMFieldDef_T;

/*!
 * Generalized Message Vector Field Structure Type
 */
typedef struct
{
  size_t    m_uCount;   ///< number of items in vector
  union
  {
    void   *m_pAlign;   ///< force alignment
    byte_t *m_buf;      ///< the vector 
  } u;                  ///< aligned buffer union
} NMVec_T;

/*!
 * Message Definition Type
 */
typedef struct _nm_msg_def_struct
{
  const char         *m_sMsgName; ///< message/struct string name
  uint_t              m_eMsgId;   ///< message id (globally unique)
  size_t              m_uCount;   ///< number of message fields
  const NMFieldDef_T *m_pFields;  ///< pointer to array of msg field definitions
} NMMsgDef_T;

#endif // not SWIG

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities Function Prototypes
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern const char *nmStrError(int ecode);

//
// Exclude the following low-level C specific interface from python swig.
//
#ifndef SWIG

extern void nmPrintBuf(FILE        *fp,
                       const char  *sPreface,
                       byte_t       buf[],
                       size_t       uCount,
                       size_t       uNLFreq,
                       uint_t       uCol);

extern void nmPrintBits(FILE         *fp,
                        const char   *sPreface,
                        ulonglong_t   uVal,
                        uint_t        uMsb,
                        uint_t        uCnt);

extern size_t nmGetFieldValSize(NMFType_T eFType);

extern const NMFieldDef_T *nmFindFieldDef(const NMMsgDef_T *pMsgDef,
                                          byte_t            byFId);

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Base Packing Function Prototypes
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int nmPackU8(byte_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackU16(ushort_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackU32(uint_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackU64(ulonglong_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackS64(long long val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackF32(float val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackF64(double val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackP32(void *val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackP64(void *val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);

extern int nmPackBuf(byte_t bufSrc[], size_t uCount,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian);


#ifdef SWIG
#define INLINE_IN_H
#endif

/*!
 * \brief Pack a signed 8-bit byte into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackS8(signed char val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU8((byte_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a signed 16-bit integer into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackS16(short val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU16((ushort_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a signed 32-bit integer into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackS32(int val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU32((uint_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a one byte value into the message buffer.
 *
 * Alias for nmPackU8().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackByte(byte_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU8((byte_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a signed char integer value into the message buffer.
 *
 * Alias for nmPackS8().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackSChar(signed char val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU8((byte_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a 8-bit ASCII character into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackChar(char val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU8((byte_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a boolean value into the message buffer.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackBool(bool_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  val &= 0xff;  // clear unused upper bits
  return nmPackU8((byte_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an unsigned short integer into the message buffer.
 *
 * Alias for nmPackU16().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackUShort(ushort_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU16((ushort_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an signed short integer into the message buffer.
 *
 * Alias for nmPackS16().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackShort(short val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU16((ushort_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an unsigned int integer into the message buffer.
 *
 * Alias for nmPackU32().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackUInt(uint_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU32((uint_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an signed int integer into the message buffer.
 *
 * Alias for nmPackS32().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackInt(int val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU32((uint_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an unsigned long integer into the message buffer.
 *
 * Alias for nmPackU32() or nmPackU64() depending on sizeof(ulong_t).
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackULong(ulong_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return sizeof(ulong_t) == 4?
              nmPackU32((uint_t)val, buf, bufSize, eEndian):
              nmPackU64((ulonglong_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an signed long integer into the message buffer.
 *
 * Alias for nmPackS32() or nmPackS64() depending on sizeof(long).
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackLong(long val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return sizeof(long) == 4?
              nmPackS32((int)val, buf, bufSize, eEndian):
              nmPackS64((long long)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an unsigned long long integer into the message buffer.
 *
 * Alias for nmPackU64().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackULongLong(ulonglong_t val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackU64((ulonglong_t)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack an signed long long integer into the message buffer.
 *
 * Alias for nmPackS64().
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std
 */
INLINE_IN_H int nmPackLongLong(long long val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return nmPackS64((long long)val, buf, bufSize, eEndian);
}

/*!
 * \brief Pack a (overloaded) void* pointer into the message buffer.
 *
 * Alias for nmPackP32() or nmPackP64() depending on sizeof(void*).
 *
 * \note Pointer is always packed in native byte order.
 *
 * \copydoc doc_params_pack_std
 * \copydoc doc_return_pack_std 
 */
INLINE_IN_H int nmPackPointer(void *val,
                    byte_t buf[], size_t bufSize, NMEndian_T eEndian)
{
  return sizeof(void*) == 4?
              nmPackP32(val, buf, bufSize, eEndian):
              nmPackP64(val, buf, bufSize, eEndian);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Base Unpacking Function Prototypes
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int nmUnpackU8(byte_t buf[], size_t bufSize,
                      byte_t *pVal, NMEndian_T eEndian);

extern int nmUnpackU16(byte_t buf[], size_t bufSize,
                      ushort_t *pVal, NMEndian_T eEndian);

extern int nmUnpackU32(byte_t buf[], size_t bufSize,
                    uint_t *pVal, NMEndian_T eEndian);

extern int nmUnpackU64(byte_t buf[], size_t bufSize,
                    ulonglong_t *pVal, NMEndian_T eEndian);

extern int nmUnpackS64(byte_t buf[], size_t bufSize,
                    long long *pVal, NMEndian_T eEndian);

extern int nmUnpackF32(byte_t buf[], size_t bufSize,
                    float *pVal, NMEndian_T eEndian);

extern int nmUnpackF64(byte_t buf[], size_t bufSize,
                    double *pVal, NMEndian_T eEndian);

extern int nmUnpackP32(byte_t buf[], size_t bufSize,
                    void *pVal, NMEndian_T eEndian);

extern int nmUnpackP64(byte_t buf[], size_t bufSize,
                    void *pVal, NMEndian_T eEndian);

extern int nmUnpackBuf(byte_t buf[], size_t bufSize,
                      byte_t bufDst[], size_t uCount,
                      NMEndian_T eEndian);

/*!
 * \brief Unpack a signed 8-bit byte from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
INLINE_IN_H int nmUnpackS8(byte_t buf[], size_t bufSize,
                      signed char *pVal, NMEndian_T eEndian)
{
  return nmUnpackU8(buf, bufSize, (byte_t *)pVal, eEndian);
}

/*!
 * \brief Unpack a signed 16-bit integer from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
INLINE_IN_H int nmUnpackS16(byte_t buf[], size_t bufSize,
                      short *pVal, NMEndian_T eEndian)
{
  return nmUnpackU16(buf, bufSize, (ushort_t *)pVal, eEndian);
}

/*!
 * \brief Unpack a signed 32-bit integer from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
INLINE_IN_H int nmUnpackS32(byte_t buf[], size_t bufSize,
                      int *pVal, NMEndian_T eEndian)
{
  return nmUnpackU32(buf, bufSize, (uint_t *)pVal, eEndian);
}

/*!
 * \brief Unpack one byte from the message buffer.
 *
 * Alias for nmUnpackU8().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackByte(byte_t buf[], size_t bufSize,
                            byte_t *pVal, NMEndian_T eEndian)
{
  return nmUnpackU8(buf, bufSize, (byte_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an signed short integer from the message buffer.
 *
 * Alias for nmUnpackS8().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackSChar(byte_t buf[], size_t bufSize,
                            signed char *pVal, NMEndian_T eEndian)
{
  return nmUnpackU8(buf, bufSize, (byte_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an 8-bit ASCII character from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackChar(byte_t buf[], size_t bufSize,
                            char *pVal, NMEndian_T eEndian)
{
  return nmUnpackU8(buf, bufSize, (byte_t *)pVal, eEndian);
}

/*!
 * \brief Unpack a boolean value from the message buffer.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackBool(byte_t buf[], size_t bufSize,
                            bool_t *pVal, NMEndian_T eEndian)
{
  *pVal = 0;  // clear all sizeof(int) bits
  return nmUnpackU8(buf, bufSize, (byte_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an unsigned short integer from the message buffer.
 *
 * Alias for nmUnpackU16().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackUShort(byte_t buf[], size_t bufSize,
                            ushort_t *pVal, NMEndian_T eEndian)
{
  return nmUnpackU16(buf, bufSize, (ushort_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an signed short integer from the message buffer.
 *
 * Alias for nmUnpackS16().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackShort(byte_t buf[], size_t bufSize,
                            short *pVal, NMEndian_T eEndian)
{
  return nmUnpackU16(buf, bufSize, (ushort_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an unsigned int integer from the message buffer.
 *
 * Alias for nmUnpackU32().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackUInt(byte_t buf[], size_t bufSize,
                            uint_t *pVal, NMEndian_T eEndian)
{
  return nmUnpackU32(buf, bufSize, (uint_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an signed int integer from the message buffer.
 *
 * Alias for nmUnpackS32().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackInt(byte_t buf[], size_t bufSize,
                            int *pVal, NMEndian_T eEndian)
{
  return nmUnpackU32(buf, bufSize, (uint_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an unsigned long integer from the message buffer.
 *
 * Alias for nmUnpackU32() or nmUnpackU64() depending on sizeof(ulong_t).
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackULong(byte_t buf[], size_t bufSize,
                            ulong_t *pVal, NMEndian_T eEndian)
{
  return sizeof(ulong_t) == 4?
              nmUnpackU32(buf, bufSize, (uint_t *)pVal, eEndian):
              nmUnpackU64(buf, bufSize, (ulonglong_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an signed long integer from the message buffer.
 *
 * Alias for nmUnpackS32() or nmUnpackS64() depending on sizeof(long).
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackLong(byte_t buf[], size_t bufSize,
                            long *pVal, NMEndian_T eEndian)
{
  return sizeof(ulong_t) == 4?
              nmUnpackS32(buf, bufSize, (int *)pVal, eEndian):
              nmUnpackS64(buf, bufSize, (long long *)pVal, eEndian);
}

/*!
 * \brief Unpack an unsigned long long integer from the message buffer.
 *
 * Alias for nmUnpackU64().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackULongLong(byte_t buf[], size_t bufSize,
                                ulonglong_t *pVal, NMEndian_T eEndian)
{
  return nmUnpackU64(buf, bufSize, (ulonglong_t *)pVal, eEndian);
}

/*!
 * \brief Unpack an signed long long integer from the message buffer.
 *
 * Alias for nmUnpackS64().
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std
 */
INLINE_IN_H int nmUnpackLongLong(byte_t buf[], size_t bufSize,
                            long long *pVal, NMEndian_T eEndian)
{
  return nmUnpackS64(buf, bufSize, (long long *)pVal, eEndian);
}

/*!
 * \brief Unpack a (overloaded)void* pointer from the message buffer.
 *
 * Alias for nmUnpackP32() or nmUnpackP64() depending on sizeof(void*).
 *
 * \note Pointer is always unpacked in native byte order.
 *
 * \copydoc doc_params_unpack_std
 * \copydoc doc_return_unpack_std 
 */
INLINE_IN_H int nmUnpackPointer(byte_t buf[], size_t bufSize,
                            void *pVal, NMEndian_T eEndian)
{
  return sizeof(void*) == 4?
              nmUnpackP32(buf, bufSize, pVal, NMEndianNative):
              nmUnpackP64(buf, bufSize, pVal, NMEndianNative);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Id-Type-Value Packing and Unpacking Function Prototypes
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int nmPackITVMsg(const NMMsgDef_T  *pMsgDef,
                        void              *pStruct,
                        byte_t             buf[],
                        size_t             bufSize,
                        NMEndian_T         eEndian);

extern int nmPackITVMsgDebug(const NMMsgDef_T  *pMsgDef,
                             void              *pStruct,
                             byte_t             buf[],
                             size_t             bufSize,
                             NMEndian_T         eEndian);

extern int nmUnpackITVMsg(const NMMsgDef_T  *pMsgDef,
                          byte_t             buf[],
                          size_t             uMsgLen,
                          void              *pStruct,
                          NMEndian_T         eEndian);

extern int nmUnpackITVMsgDebug(const NMMsgDef_T  *pMsgDef,
                               byte_t             buf[],
                               size_t             uMsgLen,
                               void              *pStruct,
                               NMEndian_T         eEndian);

extern uint_t nmGetITVMsgId(byte_t buf[], size_t uMsgLen, NMEndian_T eEndian);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Flat Packing and Unpacking Function Prototypes
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int nmPackFlatMsg(const NMMsgDef_T  *pMsgDef,
                         void              *pStruct,
                         byte_t             buf[],
                         size_t             bufSize,
                         NMEndian_T         eEndian);

extern int nmPackFlatMsgDebug(const NMMsgDef_T  *pMsgDef,
                              void              *pStruct,
                              byte_t             buf[],
                              size_t             bufSize,
                              NMEndian_T         eEndian);

extern int nmUnpackFlatMsg(const NMMsgDef_T  *pMsgDef,
                           byte_t             buf[],
                           size_t             uMsgLen,
                           void              *pStruct,
                           NMEndian_T         eEndian);

extern int nmUnpackFlatMsgDebug(const NMMsgDef_T  *pMsgDef,
                                byte_t             buf[],
                                size_t             uMsgLen,
                                void              *pStruct,
                                NMEndian_T         eEndian);

#endif // not SWIG


#ifndef SWIG
C_DECLS_END
#endif


#endif // _NETMSGS_H
