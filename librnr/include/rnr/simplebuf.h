////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Simple [io] buffer declarations and operations.
 *
 *  The buffer is orgainized as a simple linear buffer with independent read
 *  and write positions. Ideal for multi-tasking.
 *
 * \verbatim
 * [----------buffer-----------------]
 *     ^         ^                   ^
 *     RPos      WPos                BufSize
 * \endverbatim
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/simplebuf.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_SIMPLEBUF_H
#define _RNR_SIMPLEBUF_H

#include <unistd.h>

#include "rnrconfig.h"


//-----------------------------------------------------------------------------
// Defines and Types
//-----------------------------------------------------------------------------

C_DECLS_BEGIN

//
// Defines
//
#define SIMPLEBUF_ABS   0     ///< absolute
#define SIMPLEBUF_REL   1     ///< relative

/*!
 * Simple Read/Write Buffer Structure.
 */
typedef struct
{
  byte_t       *m_pRWBuf;       ///< internal read/write buffer
  size_t        m_nBufSize;     ///< buffer total size
  size_t        m_nRPos;        ///< current read buffer position
  size_t        m_nWPos;        ///< current write buffer position
  bool_t        m_bBufDel;      ///< do [not] delete internal R/W buffer
} SimpleBuf_T;


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

extern SimpleBuf_T *SimpleBufNew();

extern SimpleBuf_T *SimpleBufNewWithBuf(size_t nBufSize);

extern void SimpleBufDelete(SimpleBuf_T *pBuf);

extern void SimpleBufSetBuf(SimpleBuf_T *pBuf, byte_t *pRWBuf,
                            size_t nBufSize, size_t nLen);

extern size_t SimpleBufCopy(SimpleBuf_T *pTgt, SimpleBuf_T *pSrc); 

/*!
 * \brief Clear contents of buffer.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 */
INLINE_IN_H void SimpleBufClear(SimpleBuf_T *pBuf)
{
  pBuf->m_nWPos = (size_t)0;
  pBuf->m_nRPos = (size_t)0;
}

/*!
 * \brief Seek buffer read head to the new read position.
 *
 * \param pBuf  Pointer to simple buffer.
 * \param rpos  New read position.
 * \param how   Seek to absolute position (default) or relative to current
 *              position.
 *
 * \return Returns new read position.
 */
INLINE_IN_H size_t SimpleBufReadSeek(SimpleBuf_T *pBuf, size_t rpos, int how)
{
  if( how == SIMPLEBUF_REL )
  {
    rpos += pBuf->m_nRPos;
  }
  pBuf->m_nRPos = rpos<=pBuf->m_nWPos? rpos: pBuf->m_nWPos;
  return pBuf->m_nRPos;
}

/*!
 * \brief Seek buffer write head to the new write position.
 *
 * \param pBuf  Pointer to simple buffer.
 * \param wpos  New write position.
 * \param how   Seek to absolute position (default) or relative to current
 *              position.
 *
 * \return Returns new write position.
 */
INLINE_IN_H size_t SimpleBufWriteSeek(SimpleBuf_T *pBuf, size_t wpos, int how)
{
  if( how == SIMPLEBUF_REL )
  {
    wpos += pBuf->m_nWPos;
  }
  pBuf->m_nWPos = wpos<=pBuf->m_nBufSize? wpos: pBuf->m_nBufSize;
  return pBuf->m_nWPos;
}

/*!
 * \brief Returns size of buffer.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 *\return Returns \h_ge 0.
 */
INLINE_IN_H size_t SimpleBufHasSize(SimpleBuf_T *pBuf)
{
  return pBuf->m_nBufSize;
}

/*!
 * \brief Test if simple buffer is empty.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns true or false.
 */
INLINE_IN_H bool_t SimpleBufIsEmpty(SimpleBuf_T *pBuf)
{
  return pBuf->m_nRPos>=pBuf->m_nWPos? true: false;
}

/*!
 * \brief Test if simple buffer is full.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns true or false.
 */
INLINE_IN_H bool_t SimpleBufIsFull(SimpleBuf_T *pBuf)
{
  return pBuf->m_nWPos>=pBuf->m_nBufSize? true: false;
}

/*!
 * \brief Returns number of bytes currently in buffer.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns \h_ge 0.
 */
INLINE_IN_H size_t SimpleBufHasLen(SimpleBuf_T *pBuf)
{
  return pBuf->m_nWPos - pBuf->m_nRPos;
}

/*!
 * \brief Returns number of bytes available in buffer for writing.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns \h_ge 0.
 */
INLINE_IN_H size_t SimpleBufHasAvail(SimpleBuf_T *pBuf)
{
  return pBuf->m_nBufSize - pBuf->m_nWPos;
}

/*!
 * \brief Returns pointer to internal I/O buffer at the current read position.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns byte* on success, NULL if buffer empty.
 */
INLINE_IN_H byte_t *SimpleBufGetReadPtr(SimpleBuf_T *pBuf)
{
  return SimpleBufIsEmpty(pBuf)? NULL: pBuf->m_pRWBuf+pBuf->m_nRPos;
}

/*!
 * \brief Returns pointer to internal I/O buffer at the current write position.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns Returns byte* on success, NULL if buffer empty.
 */
INLINE_IN_H byte_t *SimpleBufGetWritePtr(SimpleBuf_T *pBuf)
{
  return SimpleBufIsFull(pBuf)? NULL: pBuf->m_pRWBuf+pBuf->m_nWPos;
}

/*!
 * \brief Gets byte from buffer at current read position, advancing read
 * position.
 *
 * \param pBuf  Pointer to simple buffer.
 *
 * \return Returns Read byte on success, -1 if buffer is empty.
 */
INLINE_IN_H int SimpleBufGetC(SimpleBuf_T *pBuf)
{
  if( !SimpleBufIsEmpty(pBuf) )
  {
    return (int)pBuf->m_pRWBuf[pBuf->m_nRPos++];
  }
  return -1;
}

/*!
 * \brief Puts byte into buffer at current write position, advancing write
 * position.
 *
 * \param pBuf  Pointer to simple buffer.
 * \param c     Byte to put.
 *
 * \return Returns Wrote byte on success, -1 if buffer is full.
 */
INLINE_IN_H int SimpleBufPutC(SimpleBuf_T *pBuf, byte_t c)
{
  if( !SimpleBufIsFull(pBuf) )
  {
    pBuf->m_pRWBuf[pBuf->m_nWPos++] = c;
    return (int)c;
  }
  return -1;
}

C_DECLS_END


#endif // _RNR_SIMPLEBUF_H
