////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      StateWinCamera.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief StateWinCamera derived state class interface.
 *
 * The StateWinCamera class uses the GTK windowing system along with the
 * GStreamer interface to a video camera.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _RNR_STATE_WIN_CAMERA_H
#define _RNR_STATE_WIN_CAMERA_H

#include <stdio.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinGtk.h"
#include "rnr/appkit/WinMenuGtk.h"
#include "rnr/appkit/CameraGst.h"
#include "rnr/appkit/SessionWin.h"
#include "rnr/appkit/State.h"
#include "rnr/appkit/StateWin.h"

namespace rnr
{
  //----------------------------------------------------------------------------
  // StateWinCamera Class
  //----------------------------------------------------------------------------
  
  /*!
   * State base class.
   */
  class StateWinCamera : public StateWin
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param nStateId      State id. Must be state machine unique.
     * \param session       State's embedding in (derived) window session.
     * \param strStateName  State name.
     * \param strRefTag     State reference id.
     */
    StateWinCamera(int                nStateId,
                   SessionWin        &session,
                   const std::string &strStateName="",
                   const std::string &strRefTag="") :
        StateWin(nStateId, session, strStateName, strRefTag)
    {
      m_bOneTimeInit  = false;
      m_bMouseEvents  = true;
      m_bKbEvents     = false;
      m_pButtons      = NULL;
    }
  
    /*!
     * \brief List constructor.
     *
     * \param nStateId        State id. Must be state machine unique.
     * \param session         State's embedding in (derived) window session.
     * \param strStateName    State name.
     * \param strRefTag       State reference id.
     * \param listStateEvents Declaration list of allocated state events.
     *                        NULL terminated.
     */
    StateWinCamera(int                nStateId,
                   SessionWin        &session,
                   const std::string &strStateName,
                   const std::string &strRefTag,
                   StateEvent        *listStateEvents[]) :
        StateWin(nStateId, session, strStateName, strRefTag, listStateEvents)
    {
      m_bOneTimeInit  = false;
      m_bMouseEvents  = true;
      m_bKbEvents     = false;
      m_pButtons      = NULL;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~StateWinCamera()
    {
    }
  
    /*!
     * \brief Set button states.
     *
     * State and/or session data determine the state of the buttons.
     */
    virtual void setButtonStates();

  protected:
    CameraGst       m_camera;         ///< the camera

    friend class    StateEvent;       ///< friend

    /*!
     * \brief One-time button menu initialization.
     *
     * This function is called on the "enter state" action.
     */
    virtual void initOnceButtons();

    /*!
     * \brief Build the window gui interface.
     *
     * This function is called on the "enter state" action.
     */
    virtual void buildGuiInterface();
  };

} // namespace rnr


#endif // _RNR_STATE_WIN_CAMERA_H
