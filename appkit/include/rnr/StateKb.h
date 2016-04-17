////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      StateKb.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Keyboard StateKb derived state class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows LLC
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

#ifndef _RNR_STATE_KB_H
#define _RNR_STATE_KB_H

#include <stdio.h>
#include <termios.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "rnr/State.h"

namespace rnr
{
  //----------------------------------------------------------------------------
  // StateKb Class
  //----------------------------------------------------------------------------
  
  /*!
   * Keyboard state class.
   *
   * The keyboard state receives non-blocking input from stdin.
   * The events are:
   * \li KbEventError
   * \li KbEventEof
   * \li KbEvnetTimeOut
   * \li 8-bit ascii code
   */
  class StateKb : public State
  {
  public:
    static int            ClassObjRefCnt;         ///< object reference count
    static int            OrigInputStatusFlags;   ///< original status flags
    static struct termios OrigInputTio;           ///< original terminal i/o

    //
    // Specical keyboard event ids.
    //
    static const int KbEventError     = -2;   ///< input error
    static const int KbEventEof       = -1;   ///< end of file
    static const int KbEventTimeOut   = 256;  ///< time out

    /*!
     * \brief Initialization constructor.
     *
     * \param nStateId      State id. Must be state machine unique.
     * \param strStateName  State name.
     * \param strRefTag     State reference id.
     */
    StateKb(int                nStateId,
            const std::string &strStateName="",
            const std::string &strRefTag="") :
        State(nStateId, strStateName, strRefTag)
    {
      if( ++ClassObjRefCnt == 1 )
      {
        configInput();
      }
    }
  
    /*!
     * \brief List constructor.
     *
     * \param nStateId        State id. Must be state machine unique.
     * \param strStateName    State name.
     * \param strRefTag       State reference id.
     * \param listStateEvents Declaration list of allocated state events.
     *                        NULL terminated.
     */
    StateKb(int                nStateId,
            const std::string &strStateName,
            const std::string &strRefTag,
            StateEvent        *listStateEvents[]) :
        State(nStateId, strStateName, strRefTag, listStateEvents)
    {
      if( ++ClassObjRefCnt == 1 )
      {
        configInput();
      }
    }

    /*!
     * \brief Destructor.
     */
    virtual ~StateKb()
    {
      if( --ClassObjRefCnt == 0 )
      {
        restoreInput();
      }
    }
  
    /*!
     * \brief Receive next event.
     *
     * This Keyboard state class receives keyboard input.
     *
     * The receive event may set state internal variables used specifically by
     * the state.
     *
     * \return Returns event id.
     */
    virtual int receiveEvent();

  protected:
    friend class StateEvent;            ///< friend

    /*!
     * \brief Configure stdin input.
     */
    void configInput();

    /*!
     * \brief Restore stdin original configuration.
     */
    void restoreInput();
  };

} // namespace rnr


#endif // _RNR_STATE_KB_H
