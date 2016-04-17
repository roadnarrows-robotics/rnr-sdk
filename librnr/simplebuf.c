////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      simplebuf.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Simple [io] buffer declarations and operations.
 *
 *  The buffer is orgainized as a simple linear buffer with independent read
 *  and write positions. Ideal for multi-tasking.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2005-2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com) \n
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
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/simplebuf.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Allocate a new simple buffer with no internal buffer allocation.
 *
 * \return Returns SimpleBuf_T* to newly allocated simple buffer.
 */
SimpleBuf_T *SimpleBufNew()
{
  SimpleBuf_T *pBuf = NEW(SimpleBuf_T);

  pBuf->m_pRWBuf    = NULL;
  pBuf->m_nBufSize  = (size_t)0;
  pBuf->m_nRPos     = (size_t)0;
  pBuf->m_nWPos     = (size_t)0;
  pBuf->m_bBufDel   = false;

  return pBuf;
}

/*!
 * \brief Allocate a new simple buffer with internal buffer of nBufSize.
 *
 * \param nBufSize  Size in bytes of internal buffer to allocate with simple
 *                  buffer.
 *
 * \return Returns SimpleBuf_T* to newly allocated simple buffer.
 */
SimpleBuf_T *SimpleBufNewWithBuf(size_t nBufSize)
{
  SimpleBuf_T *pBuf = NEW(SimpleBuf_T);

  pBuf->m_pRWBuf    = (byte_t *)new(nBufSize);
  pBuf->m_nBufSize  = nBufSize;
  pBuf->m_nRPos     = (size_t)0;
  pBuf->m_nWPos     = (size_t)0;
  pBuf->m_bBufDel   = true;

  return pBuf;
}

/*!
 * \brief Delete a simple buffer along with the internal buffer.
 *
 * Actual deletion on occurs if buffer is owned by this simple buffer.
 *
 * \param pBuf  Pointer to simple buffer to be deleted.
 */
void SimpleBufDelete(SimpleBuf_T *pBuf)
{
  if( pBuf == NULL )
  {
    return;
  }

  if( pBuf->m_bBufDel )
  {
    delete(pBuf->m_pRWBuf);
  }
  delete(pBuf);
}

/*!
 * \brief Set simple buffer's internal read/write buffer.
 *
 * \param pBuf      Pointer to simple buffer.
 * \param pRWBuf    Pointer to buffer to be set as the internal read/write
 *                  buffer.
 * \param nBufSize  Size in bytes of pRWBuf.
 * \param nLen      Current length (bytes) of data in pRWBuf.
 */
void SimpleBufSetBuf(SimpleBuf_T *pBuf, byte_t *pRWBuf,
                     size_t nBufSize, size_t nLen)
{
  if( pBuf == NULL )
  {
    return;
  }

  if( pBuf->m_bBufDel )
  {
    delete(pBuf->m_pRWBuf);
  }
 
  pBuf->m_pRWBuf    = pRWBuf;
  pBuf->m_nBufSize  = nBufSize;
  pBuf->m_nRPos     = (size_t)0;
  pBuf->m_nWPos     = nLen<=nBufSize? nLen: nBufSize;
  pBuf->m_bBufDel   = false;
}

/*!
 * \brief Copy contents of source simple buffer to the end of the target simple
 *  buffer.
 *
 * \param pTgt  Pointer to target simple buffer.
 * \param pSrc  Pointer to source simple buffer.
 *
 * Return Value:
 * \return Returns
 *  Number of bytes copied.
 */
size_t SimpleBufCopy(SimpleBuf_T *pTgt, SimpleBuf_T *pSrc)
{
  size_t  rpos, nBytes;

  if( (pTgt == NULL) || (pSrc == NULL) )
  {
    return (size_t)0;
  }

  for(rpos=pSrc->m_nRPos, nBytes=0;
      (rpos < pSrc->m_nWPos) && (pTgt->m_nWPos < pSrc->m_nBufSize);
      ++rpos, ++nBytes)
  {
    pTgt->m_pRWBuf[pTgt->m_nWPos++] = pSrc->m_pRWBuf[rpos];
  }

  return nBytes;
}
