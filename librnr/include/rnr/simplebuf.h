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
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
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
