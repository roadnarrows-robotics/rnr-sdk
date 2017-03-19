////////////////////////////////////////////////////////////////////////////////
//
// Package:   netmsgs
//
// Library:   libnetmsgs
//
// File:      nmLibUtils.c
//
/*! \file
 *
 * $LastChangedDate: 2010-07-31 08:48:56 -0600 (Sat, 31 Jul 2010) $
 * $Rev: 521 $
 *
 * \brief NetMsgs library utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/netmsgs.h"

#include "nmLibInternal.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief NetMsgs Error Code String Table.
 *
 * Table is indexed by NetMsgs error codes (see \ref man_libnetmsgs_ecodes).
 * Keep in sync.
 */
static const char *nmEcodeStrTbl[] =
{
  "Ok",                                     ///< [NM_OK]

  "Error",                                  ///< [NM_ECODE_GEN]
  "Insufficient memory available",          ///< [NM_ECODE_NOMEM]
  "Machine architecture not supported",     ///< [NM_ECODE_ARCH_NOTSUP]
  "Value out-of-range",                     ///< [NM_ECODE_RANGE]
  "Unknown field type",                     ///< [NM_ECODE_FTYPE]
  "Bad message format",                     ///< [NM_ECODE_EMSG]
  "Unknown message id",                     ///< [NM_ECODE_MSGID]
  "Internal error",                         ///< [NM_ECODE_INTERNAL]
  "Invalid error code",                     ///< [NM_ECODE_BADEC]

  NULL, 0
};
/*!
 * Field Value Byte Size Lookup Table
 *
 * \warning Keep in asending order by ftype.
 */
static size_t NMFValLenLookupTbl[] =
{
  NMFVAL_LEN_BOOL,  NMFVAL_LEN_U8,    NMFVAL_LEN_F64,   NMFVAL_LEN_U16,
  NMFVAL_LEN_U32,   NMFVAL_LEN_P64,   NMFVAL_LEN_U64,   NMFVAL_LEN_VECTOR,
  NMFVAL_LEN_S8,    NMFVAL_LEN_CHAR,  NMFVAL_LEN_F32,   NMFVAL_LEN_S16,
  NMFVAL_LEN_S32,   NMFVAL_LEN_P32,   NMFVAL_LEN_S64,   NMFVAL_LEN_STRING,
  NMFVAL_LEN_STRUCT
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the error string describing the BotSense error code.
 *
 * The absolute value of the error code is taken prior retrieving the string.
 * An unknown or out-of-range error code will be mapped to
 * \ref NM_ECODE_BADEC.
 *
 * \param ecode BotSense error code.
 *
 * \return Returns the appropriate error code string.
 */
const char *nmStrError(int ecode)
{
  if( ecode < 0 )
  {
    ecode = -ecode;
  }

  if( ecode >= arraysize(nmEcodeStrTbl) )
  {
    ecode = NM_ECODE_BADEC;
  }
  return nmEcodeStrTbl[ecode];
}

/*!
 * \brief Pretty print buffer to opened file stream.
 *
 * \param fp        File pointer.
 * \param sPreface  Optional buffer preface string (set to NULL for no preface).
 * \param buf       Buffer to print.
 * \param uCount    Number of bytes to print.
 * \param uNLFreq   Newline frequency (set to 0 for no newlines).
 * \param uCol      Column alignment number.
 */
void nmPrintBuf(FILE       *fp,
                const char *sPreface,
                byte_t      buf[],
                size_t      uCount,
                size_t      uNLFreq,
                uint_t      uCol)
{
  size_t  i;

  if( sPreface && *sPreface )
  {
    fprintf(fp, "%s:", sPreface);
  }

  for(i=0; i<uCount; ++i)
  {
    if( (uNLFreq > 0) && ((i % uNLFreq) == 0) )
    {
      if( i != 0 )
      {
        fprintf(fp, "\n%*s", uCol, "");
      }
    }
    fprintf(fp, " 0x%02x", buf[i]);
  }
}

/*!
 * \brief Pretty print bits in value.
 *
 * \param fp        File pointer.
 * \param sPreface  Optional bit preface string (set to NULL for no preface).
 * \param uVal      Bits to print.
 * \param uMsb      Starting most significant bit.
 * \param uCnt      Number of bits.
 */
void nmPrintBits(FILE        *fp,
                 const char  *sPreface,
                 ulonglong_t  uVal,
                 uint_t       uMsb,
                 uint_t       uCnt)
{
  uint_t i;

  if( sPreface && *sPreface )
  {
    fprintf(fp, "%s:", sPreface);
  }

  for(i=0; i<uCnt; ++i)
  {
    if( (uMsb % 8) == 7 )
    {
      if( i != 0 )
      {
        printf(" ");
      }
    }
    (uVal >> uMsb) & 0x01?  printf("1"): printf("0");
    --uMsb;
  }
}

/*!
 * \brief Get the field value byte size.
 *
 * \param eFType  Field type.
 *
 * \return Returns field value byte size. Returns 0 if field type is unknown
 * or is variable in length.
 */
size_t nmGetFieldValSize(NMFType_T eFType)
{
  int idx;
 
  idx = NMHashFType(eFType);

  if( (idx != NMHashNoIdx) && (idx < arraysize(NMFValLenLookupTbl)) )
  {
    return NMFValLenLookupTbl[idx];
  }
  else
  {
    return (size_t)0;
  }
}

/*!
 * \brief Find the field definition in the message definition,
 * given the field id.
 *
 * \param pMsgDef   Pointer to message definition.
 * \param byFId     Field Id.
 *
 * \return If the field definition is found, then the pointer to the
 * field definition is returned. Else NULL is return.
 */
const NMFieldDef_T *nmFindFieldDef(const NMMsgDef_T *pMsgDef, byte_t byFId)
{
  size_t  i;

  for(i=0; i<pMsgDef->m_uCount; ++i)
  { 
    if( pMsgDef->m_pFields[i].m_eFId == byFId )
    {
      return &(pMsgDef->m_pFields[i]);
    }
  }

  return NULL;
}
