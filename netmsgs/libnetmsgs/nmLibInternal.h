////////////////////////////////////////////////////////////////////////////////
//
// Package:   netmsgs
//
// Library:   libnetmsgs
//
// File:      nmLibInternal.h
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:32:30 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1578 $
 *
 * \brief Internal intra-library declarations.
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

#ifndef _NMLIBINTERNAL_H
#define _NMLIBINTERNAL_H

#include <stdio.h>
#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/netmsgs.h"

/*!
 * \brief Print debugging info. Undef in release version. See Makefile.
 *
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#ifdef NMLIB_DEBUG_ENABLE // debugging compile in/out switch
#define NMLIB_DEBUG(fmt, ...) \
  fprintf(stderr, "DEBUG: %s[%u]: %s(): " fmt, \
      __FILE__, __LINE__, LOGFUNCNAME, ##__VA_ARGS__)
#else
#define NMLIB_DEBUG(fmt, ...)
#endif // NMLIB_DEBUG_ENABLE

/*!
 * \brief Log libnetmsgs warning.
 *
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_WARNING(fmt, ...) \
  LOGDIAG1("Warning: " fmt, ##__VA_ARGS__)

/*!
 * \brief Log libnetmsgs field warning.
 *
 * \param p         Pointer to field definition (may be NULL).
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_FIELD_WARNING(p, fmt, ...) \
  ((p)!=NULL?: \
    NMLIB_WARNING("%s(%u): " fmt, (p)->m_sFName, (p)->m_eFId, ##__VA_ARGS__): \
    NMLIB_WARNING(fmt, ##__VA_ARGS__))

/*!
 * \brief Log libnetmsgs error.
 *
 * \param ecode     NetMsgs error code.
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_ERROR(ecode, fmt, ...) \
  LOGERROR("%s(ecode=%d): " fmt, \
      nmStrError(ecode), (ecode>=0? ecode: -ecode), ##__VA_ARGS__)

/*!
 * \brief Raise libnetmsgs error (i.e. return from calling function).
 *
 * \param ecode     NetMsgs error code.
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_RAISE_ERROR(ecode, fmt, ...) \
  do \
  { \
    NMLIB_ERROR(ecode, fmt, ##__VA_ARGS__); \
    return ecode>0? -ecode: ecode; \
  } while(0)

/*!
 * \brief Log libnetmsgs field error.
 *
 * Error is logged prior to return.
 *
 * \param ecode     NetMsgs error code.
 * \param p         Pointer to field definition (may be NULL).
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_FIELD_ERROR(ecode, p, fmt, ...) \
  do \
  { \
    if( (p) !=NULL ) \
    { \
      NMLIB_ERROR(ecode, "%s(%u): " fmt, \
                          (p)->m_sFName, (p)->m_eFId, ##__VA_ARGS__); \
    } \
    else \
    { \
      NMLIB_ERROR(ecode, fmt, ##__VA_ARGS__); \
    } \
  } while(0)

/*!
 * \brief Raise libnetmsgs field error (i.e. return from calling function).
 *
 * Error is logged prior to return.
 *
 * \param ecode     NetMsgs error code.
 * \param p         Pointer to field definition (may be NULL).
 * \param fmt       Output format string.
 * \param ...       Variable arguments.    
 */
#define NMLIB_RAISE_FIELD_ERROR(ecode, p, fmt, ...) \
  do \
  { \
    NMLIB_FIELD_ERROR(ecode, p, fmt, ##__VA_ARGS__); \
    return ecode>0? -ecode: ecode; \
  } while(0)

/*!
 * \brief Printable ASCII Field Type.
 *
 * \param ftype Field Id 
 */
#define NMLIB_ASCII_FTYPE(ftype) (isgraph((int)ftype)? ftype: ' ')


/*!
 * \brief Internal Control Structure
 */
typedef struct
{
  byte_t  m_bNoHdr;   ///< do [not] include field header in byte stream
  byte_t  m_bNoExec;  ///< do [not] execute assignment of value
  byte_t  m_bTrace;   ///< do [not] trace packing/unpacking
  byte_t  m_uDepth;   ///< structured message depth
} NMCtl_T;

/*!
 * \brief Default Internal control declartion list.
 */
#define NMCTL_INIT_DECL {false, false, false, 0}

/*!
 * \brief Trace field.
 *
 * \param p     Pointer to field definition (may be NULL).
 * \param buf   Input/output buffer.
 * \param n     Number of bytes in buffer to trace.
 * \param ctl   Internal Control.
 * \param fmt   Field representation format string.
 * \param ...   Field representation variable arguments.
 */
#ifdef NMLIB_DEBUG_ENABLE
#define NMLIB_TRACE_FIELD(p, buf, n, ctl, fmt, ...) \
  do \
  { \
    if( ((n) >= 0) && ((ctl)->m_bTrace) ) \
    { \
      nmTraceField(p, (byte_t *)buf, (size_t)n, \
                      (uint_t)(ctl)->m_uDepth, \
                      fmt, ##__VA_ARGS__); \
    } \
  } while(0)
#else
#define NMLIB_TRACE_FIELD(p, buf, n, ctl, fmt, ...)
#endif // NMLIB_DEBUG_ENABLE

//
// Hash Table Variables
//
extern const int    NMHashOffset;
extern const int    NMHashNoIdx;
extern const byte_t NMHashTbl[];
extern const size_t NMHashNumEntries;

/*!
 * \brief Field Type to Index hash function.
 *
 * \param eFType  Field type.
 *
 * \return Returns index on success, NMHasNoIdx on failure.
 */
INLINE_IN_H int NMHashFType(NMFType_T eFType)
{
  int i = (int)((int)eFType - (int)NMHashOffset);

#ifdef LOG
  if( LOGABLE(LOG_LEVEL_DIAG4) )
  { 
    int idx = ((i >= 0) && (i < (int)NMHashNumEntries))? (int)NMHashTbl[i]:
                                                         NMHashNoIdx;
      LOGDIAG4("hash %c --> %d --> %d", NMLIB_ASCII_FTYPE(eFType), i, idx);
  }
#endif // LOG

  return ((i >= 0) && (i < (int)NMHashNumEntries))? (int)NMHashTbl[i]:
                                                    NMHashNoIdx;
}

/*! Message Field Packer Function Type */
typedef int (*NMPackFunc_T)(const NMFieldDef_T *,
                            void *,
                            byte_t [],
                            size_t,
                            NMEndian_T,
                            NMCtl_T *);

/*! Message Field Unpacker Function Type */
typedef int (*NMUnpackFunc_T)(const NMFieldDef_T *,
                              byte_t [],
                              size_t,
                              void *,
                              NMEndian_T,
                              NMCtl_T *);

/*!
 * Message Field Packer/Unpacker Lookup Table Entry Type
 */
typedef struct
{
  uint_t          m_eFType;     ///< message field type
  NMPackFunc_T    m_fnPack;     ///< packer
  NMUnpackFunc_T  m_fnUnpack;   ///< unpacker
} NMLookupTblEntry_T;


// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

extern int nmSetU8(const NMFieldDef_T *pFieldDef,
                   void               *pValIn,
                   byte_t             *pValOut);

extern int nmSetS8(const NMFieldDef_T *pFieldDef,
                   void               *pValIn,
                   signed char        *pValOut);

extern int nmSetU16(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    ushort_t            *pValOut);

extern int nmSetS16(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    short               *pValOut);

extern int nmSetU32(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    uint_t              *pValOut);

extern int nmSetS32(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    int                 *pValOut);

extern int nmSetU64(const NMFieldDef_T  *pFieldDef,
             void                       *pValIn,
             ulonglong_t                *pValOut);

extern int nmSetS64(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    long long           *pValOut);

extern int nmSetF32(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    float               *pValOut);

extern int nmSetF64(const NMFieldDef_T  *pFieldDef,
                    void                *pValIn,
                    double              *pValOut);

#ifdef NMLIB_DEBUG_ENABLE
extern void nmTraceField(const NMFieldDef_T *pFieldDef,
                         byte_t              buf[],
                         size_t              uCount,
                         uint_t              uDepth,
                         const char          *sFmt,
                         ...);
#endif // NMLIB_DEBUG_ENABLE


#endif // _NMLIBINTERNAL_H
