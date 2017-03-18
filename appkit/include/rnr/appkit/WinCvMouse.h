////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_win
//
// File:      WinCvMouse.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-08 08:22:06 -0600 (Wed, 08 May 2013) $
 * $Rev: 2920 $
 *
 * \brief RoadNarrows Robotics base OpenCV Mouse class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
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
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_WIN_CV_MOUSE_H
#define _RNR_WIN_CV_MOUSE_H

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

#include "rnr/appkit/Win.h"

//
// RoadNarrows Robotics Windowing Declarations
//
namespace rnr
{
  //...........................................................................
  // Class WinCvMouse
  //...........................................................................
 
  /*!
   * \brief Window OpenCV Mouse base class.
   *
   * This class supports mouse operations on an OpenCV widget region of the
   * window.
   */
  class WinCvMouse
  {
  public:
    /*!
     * \brief Default contructor.
     */
    WinCvMouse();

    /*!
     * \brief Destructor.
     */
    virtual ~WinCvMouse();

    /*!
     * \brief Bind the mouse to the given window.
     *
     * Any previously bound mouse instance is unbound. This mouse state is
     * reset.
     *
     * \param pWin  Pointer to binding window.
     */
    void bind(Win *pWin);

    /*!
     * \brief Unbind the mouse instance from the currently bound window.
     */
    void unbind();

    /*!
     * \brief Get the current mouse event.
     *
     * \return Current event.
     */
    int getCurrentEvent()
    {
      return m_nCurrentEvent;
    }
  
    /*!
     * \brief Set the current mouse event.
     *
     * \param nEvent    Event unique identifier.
     */
    void setCurrentEvent(int nEvent)
    {
      m_nCurrentEvent = nEvent;
    }

    /*!
     * \brief Enable mouse drag operations.
     */
    void enableMouseDrag()
    {
      m_bDrag = true;
      m_bDragState = false;
    }

    /*!
     * \brief Disable mouse drag operations.
     */
    void disableMouseDrag()
    {
      m_bDrag = false;
      m_bDragState = false;
    }

    /*!
     * \brief Get the most recently recorded mouse position.
     *
     * \return Mouse point in screen coordinates.
     */
    CvPoint getMousePoint()
    {
      return m_ptMouse;
    }

  protected:
    Win        *m_pWin;             ///< mouse actions are bound to this windo
    int         m_nCurrentEvent;    ///< current (user) menu event
    CvPoint     m_ptMouse;          ///< mouse point
    bool        m_bDrag;            ///< enable mouse drag operation
    bool        m_bDragState;       ///< [not] within drag operation

    /*!
     * \brief Mouse event callback handler.
     *
     * \param event   Mouse event.
     * \param x       Window mouse x coordinate.
     *                A value of -1 indicates out-of-region event.
     * \param y       Window mouse y coordinate.
     *                A value of -1 indicates out-of-region event.
     * \param flags   Extra flags
     * \param param   Application specific parameter (this).
     */
    static void onMouse(int event, int x, int y, int flags, void *param);
  };
    
} // namespace


#endif // _RNR_WIN_CV_MOUSE_H
