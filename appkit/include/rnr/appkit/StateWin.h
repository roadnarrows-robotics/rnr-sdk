////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_win
//
// File:      StateWin.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief GUI window StateWin derived state class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef _RNR_STATE_WIN_H
#define _RNR_STATE_WIN_H

#include <stdio.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinMenu.h"
#include "rnr/appkit/SessionWin.h"
#include "rnr/appkit/State.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  //----------------------------------------------------------------------------
  // StateWin Class
  //----------------------------------------------------------------------------
  
  /*!
   * State base class.
   */
  class StateWin : public State
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
    StateWin(int                nStateId,
             SessionWin        &session,
             const std::string &strStateName="",
             const std::string &strRefTag="") :
        State(nStateId, strStateName, strRefTag),
        m_session(session)
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
    StateWin(int                nStateId,
             SessionWin        &session,
             const std::string &strStateName,
             const std::string &strRefTag,
             StateEvent        *listStateEvents[]) :
        State(nStateId, strStateName, strRefTag, listStateEvents),
        m_session(session)
    {
      m_bOneTimeInit  = false;
      m_bMouseEvents  = true;
      m_bKbEvents     = false;
      m_pButtons      = NULL;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~StateWin()
    {
      if( m_pButtons != NULL )
      {
        delete m_pButtons;
      }
    }
  
    /*!
     * \brief Receive next window event.
     *
     * The receive event may set state internal variables used specifically by
     * the state.
     *
     * \return Returns event id.
     */
    virtual int receiveEvent();

    /*!
     * \brief Execute "enter state" action.
     *
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     */
    virtual void actionEnterState(int nPrevStateId, int nEventId);

    /*!
     * \brief Execute "exit state" action.
     *
     * \param nNextStateId  Next state id.
     * \param nEventId      Received event id.
     */
    virtual void actionExitState(int nNextStateId, int nEventId);

    /*!
     * \brief Set button states.
     *
     * State and/or session data determine the state of the buttons.
     */
    virtual void setButtonStates()
    {
    }

    /*!
     * \brief Get the embedding window session
     *
     * \return SessionWin (derived) object reference.
     */
    SessionWin &session()
    {
      return m_session;
    }

    /*!
     * \brief Enable/disable mouse events.
     *
     * \param bEnable   Enable (true) or disable (false) state.
     */
    void enableMouseEvents(bool bEnable)
    {
      m_bMouseEvents = bEnable;
    }

    /*!
     * \brief Enable/disable keyboard events.
     *
     * \param bEnable   Enable (true) or disable (false) state.
     */
    void enableKbEvents(bool bEnable)
    {
      m_bKbEvents = bEnable;
    }

  protected:
    SessionWin     &m_session;        ///< state id
    bool            m_bOneTimeInit;   ///< state's one-time initializaion
    bool            m_bMouseEvents;   ///< enable/disable mouse events
    bool            m_bKbEvents;      ///< enable/disable keyboard events
    WinButtonMenu  *m_pButtons;       ///< button menu
    WinCvMouse      m_mouse;          ///< mouse

    friend class    StateEvent;       ///< friend

    /*!
     * \brief One-time initialization of the window gui interface.
     *
     * This function is called on the "enter state" action. Another option, is
     * to do these one-time initialization during object construction.
     * However, derived class objects would call the base classes one-time
     * initialization unnecessarily. So, lazy initialization is used to make
     * use of C++ inheritence.
     */
    virtual void initOnceGuiInterface()
    {
      if( !m_bOneTimeInit )
      {
        initOnceButtons();
        m_bOneTimeInit = true;
      }
    }

    /*!
     * \brief One-time button menu initialization.
     *
     * This function is called on the "enter state" action.
     */
    virtual void initOnceButtons()
    {
      // initialize button here
    }

    /*!
     * \brief Build the window gui interface.
     *
     * This function is called on the "enter state" action.
     */
    virtual void buildGuiInterface()
    {
      // create new widgets here
    }

    /*!
     * \brief Show the window gui interface.
     *
     * This function is called on the "enter state" action.
     */
    virtual void showGuiInterface()
    {
      // show all gui widgets in workspace.
      m_session.win().showWorkspace();
    }

    /*!
     * \brief Destroy the window gui interface.
     *
     * This function is called on the "exit state" action.
     */
    virtual void destroyGuiInterface()
    {
      // remove all widgets from workspace
      m_session.win().eraseWorkspace();
    }
  };

} // namespace rnr


#endif // _RNR_STATE_WIN_H
