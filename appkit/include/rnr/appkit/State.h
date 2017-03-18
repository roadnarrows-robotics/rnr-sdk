////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      State.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief State base class interface.
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

#ifndef _RNR_STATE_H
#define _RNR_STATE_H

#include <stdio.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

namespace rnr
{
  //
  // Forward declarations
  //
  class State;


  //----------------------------------------------------------------------------
  // StateEvent Class
  //----------------------------------------------------------------------------

  class StateEvent
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param nEventId    Event id.
     * \param nEventName  Event name.
     * \param nActionName Action name.
     */
    StateEvent(int                nEventId,
               const std::string &strEventName="",
               const std::string &strActionName="");

    virtual ~StateEvent()
    {
    }

    /*!
     * \brief Evaluate event guard condition.
     *
     * \param pState        Associated state.
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     *
     * \return Returns true if event is accepted, otherwise returns false.
     */
    virtual bool evalGuardExpr(State *pState,
                               int    nPrevStateId,
                               int    nEventId)
    {
      return true;
    }

    /*!
     * \brief Execute action associated with the event.
     *
     * \param pState        Associated state.
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     *
     * \return Returns next state.
     */
    virtual int execAction(State *pState,
                           int    nPrevStateId,
                           int    nEventId);
    
    int getEventId() const
    {
      return m_nEventId;
    }

    std::string getEventName() const
    {
      return m_strEventName;
    }

    std::string getActionName() const
    {
      return m_strActionName;
    }

  protected:
    int         m_nEventId;
    std::string m_strEventName;
    std::string m_strActionName;
  };


  //----------------------------------------------------------------------------
  // State Class
  //----------------------------------------------------------------------------
  
  /*!
   * State base class.
   */
  class State
  {
  public:
    typedef std::map<int,StateEvent*>  TransTblMap_T; ///< transition table type

    //
    // Specical reserved state ids.
    //
    static const int StateIdUndef     = 0;  ///< undefined state
    static const int StateIdStart     = 1;  ///< start state
    static const int StateIdPrev      = 2;  ///< the previous state
    static const int StateIdThis      = 3;  ///< the current (this) state
    static const int StateIdTerminate = 4;  ///< terminate state
    static const int StateIdNumOf     = 5;  ///< number of reserved state id's

    /*!
     * \brief Initialization constructor.
     *
     * \param nStateId      State id. Must be state machine unique.
     * \param strStateName  State name.
     * \param strRefTag     State reference id.
     */
    State(int                nStateId,
          const std::string &strStateName="",
          const std::string &strRefTag="")
    {
      init(nStateId, strStateName, strRefTag);
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
    State(int                nStateId,
          const std::string &strStateName,
          const std::string &strRefTag,
          StateEvent        *listStateEvents[]);

    /*!
     * \brief Destructor.
     */
    virtual ~State();
  
    /*!
     * \brief Add a list of state events to this state.
     *
     * This state owns all of the states events and will automatically delete
     * them when this state is deleted.
     *
     * \param pStateEvent First allocated state event object.
     * \param ...         Subsequent allocated state event object.
     *                    Terminate list with NULL.
     *
     * \return Returns the number of state events added.
     */
    int addStateEvents(StateEvent *pStateEvent, ...);

    /*!
     * \brief Add state to this state.
     *
     * This state machine owns the state and will automatically delete it when
     * this state machine is deleted.
     *
     * \param pStateEvent  Allocated state object.
     */
    void addStateEvent(StateEvent *pStateEvent);

    /*!
     * \brief Delete state event from this state.
     *
     * \param nEventId  Event id.
     *
     * \return Returns true if state event is deleted, false otherwise.
     */
    bool deleteStateEvent(int nEventId);

    /*!
     * \brief Set recieve event time out.
     *
     * \param usecTimeOut Time out in \h_mu seconds. Set to 0 for blocking
     *                    until event with no time out.
     *
     * \return Returns last time out value.
     */
    uint_t setTimeOut(uint_t usecTimeOut)
    {
      uint_t  usecLast = m_usecTimeOut;
      m_usecTimeOut = usecTimeOut;
      return usecLast;
    }

    /*!
     * \brief Get recieve event time out.
     *
     * \return Returns time out value.
     */
    uint_t getTimeOut()
    {
      return m_usecTimeOut;
    }

    /*!
     * \brief Receive next event.
     *
     * Randomly chooses an event id listed in the transition table.
     *
     * The receive event may set state internal variables used specifically by
     * the state.
     *
     * \return Returns event id.
     */
    virtual int receiveEvent();

    /*!
     * \brief Queue the next event to be "recieved".
     *
     * Subsequent call to receiveEvent() should retrieve this event rather
     * than doing the state's standard receive operations.
     */
    virtual void queueNextEvent(int nEventId)
    {
      m_nQueuedEventId  = nEventId;
      m_bHasQueuedEvent = true;
    }
  
    /*!
     * \brief Dispatch received event by executing associated action and 
     * transitioning to the next state.
     *
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     *
     * \return Returns next state.
     */
    virtual int dispatchEvent(int nPrevStateId, int nEventId);

    /*!
     * \brief Execute 'enter state' action.
     *
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     */
    virtual void actionEnterState(int nPrevStateId, int nEventId)
    {
    }

    /*!
     * \brief Execute 'exit state' action.
     *
     * \param nNextStateId  Next state id.
     * \param nEventId      Received event id.
     */
    virtual void actionExitState(int nNextStateId, int nEventId)
    {
    }

    /*!
     * \brief Execute 'default' action.
     *
     * \param nPrevStateId  Previous state id.
     * \param nEventId      Received event id.
     *
     * \return Returns next state (this).
     */
    virtual int actionDefault(int nPrevStateId, int nEventId)
    {
      return m_nStateId;
    }

    /*!
     * \brief Get state id.
     *
     * \return State id.
     */
    int getStateId() const
    {
      return m_nStateId;
    }

    /*!
     * \brief Get state name.
     *
     * \return String.
     */
    std::string getStateName() const
    {
      return m_strStateName;
    }

    /*!
     * \brief Get state reference tag.
     *
     * \return String.
     */
    std::string getRefTag() const
    {
      return m_strRefTag;
    }

    /*!
     * \brief Get state event name.
     *
     * \param nEventId    Event id.
     *
     * \return String.
     */
    std::string getEventName(int nEventId);

    /*!
     * \brief Get state action name.
     *
     * \param nEventId    Event id.
     *
     * \return String.
     */
    std::string getActionName(int nEventId);

    /*!
     * \brief Print out state.
     *
     * \param fp      Output file pointer.
     * \param indent  Left indentation.
     */
    virtual void printState(FILE *fp=stdout, int indent=0);

  protected:
    int           m_nStateId;           ///< state id
    std::string   m_strStateName;       ///< state name
    std::string   m_strRefTag;          ///< state tag (short id string)
    TransTblMap_T m_mapTransTbl;        ///< state transition table
    int           m_nQueuedEventId;     ///< queue state event
    bool          m_bHasQueuedEvent;    ///< [no] queued state event
    uint_t        m_usecTimeOut;        ///< receive event time out (useconds)

    friend class StateEvent;            ///< friend

    /*!
     * \brief Initialize data.
     *
     * \param nStateId      State id. Must be state machine unique.
     * \param strStateName  State name.
     * \param strRefTag     State reference id.
     */
    void init(int                nStateId,
          const std::string &strStateName="",
          const std::string &strRefTag="");

    /*!
     * \brief Get the reserved state name.
     *
     * \param nStateId    State id.
     *
     * \return String.
     */
    virtual std::string getReservedStateName(int nStateId);
  };

} // namespace rnr


#endif // _RNR_STATE_H
