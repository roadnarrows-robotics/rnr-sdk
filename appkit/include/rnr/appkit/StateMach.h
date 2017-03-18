////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      StateMach.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Finite State Machine interface.
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

#ifndef _RNR_STATE_MACH_H
#define _RNR_STATE_MACH_H

#include <stdio.h>

#include <string>
#include <map>
#include <vector>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/State.h"


namespace rnr
{
  //----------------------------------------------------------------------------
  // StateMach Class
  //----------------------------------------------------------------------------

  /*!
   * \brief Finite State Machine Class.
   */
  class StateMach
  {
  public:
    typedef std::map<int,State*>   StateTblMap_T;   ///< state table type

    static const size_t NO_LIMIT  = 0;  ///< no limit on length of recording

    /*!
     * \brief Default contructor.
     *
     * \param nStateMachId        State machine id.
     * \param strStateMachName    State machine name.
     */
    StateMach(int nStateMachId = 0, const std::string &strStateMachName = "sm");

    /*!
     * \brief List contructor.
     *
     * This state machine owns all of the states and will automatically delete
     * them when this state machine is deleted.
     *
     * \param nStateMachId        State machine id.
     * \param strStateMachName    State machine name.
     * \param nStartStateId       State id.
     * \param listStates          Declaration list of allocated states.
     *                            NULL terminated.
     */
    StateMach(int                nStateMachId,
              const std::string &strStateMachName,
              int                nStartStateId,
              State             *listStates[]);

    /*!
     * \brief Destructor.
     */
    virtual ~StateMach();

    /*!
     * \brief Add a list of states to this state machine.
     *
     * This state machine owns all of the states and will automatically delete
     * them when this state machine is deleted.
     *
     * \param pState  First allocated state object.
     * \param ...     Subsequent allocated state object. Terminate list with
     *                NULL.
     *
     * \return Returns the number of states added.
     */
    int addStates(State *pState, ...);

    /*!
     * \brief Add state to this state machine.
     *
     * This state machine owns the state and will automatically delete it when
     * this state machine is deleted.
     *
     * \param pState  Allocated state object.
     */
    void addState(State *pState);

    /*!
     * \brief Delete state from this state machine.
     *
     * \param nStateId  State id.
     *
     * \return Returns true if state is deleted, false otherwise.
     */
    bool deleteState(int nStateId);

    /*!
     * \brief Set state machine's start state id.
     *
     * \param nStartStateId   State id.
     */
    void setStartStateId(int nStartStateId)
    {
      m_nStartStateId = nStartStateId;
    }

    /*!
     * \brief Run the state machine from the start.
     *
     * This function does not return until the terminate state is reached.
     */
    virtual void run();

    /*!
     * \brief Start recording states ids.
     *
     * Any previous recording is erased.
     *
     * Typical uses for recording states are work flows, in which the previous
     * states may need backtracing, given a previous event.
     *
     * \param uLimit      Limit the length of the recording.
     */
    virtual void startRecording(size_t uLimit = NO_LIMIT)
    {
      m_stackStateIds.clear();
      m_uMaxRecording = uLimit;
      m_bIsRecording  = true;
    }

    /*!
     * \brief Stop recording of states ids.
     *
     * The recording is not erased.
     */
    virtual void stopRecording()
    {
      m_bIsRecording = false;
    }

    /*!
     * \brief Resume recording of states ids.
     *
     * The recording is not erased.
     */
    virtual void resumeRecording()
    {
      m_bIsRecording = true;
    }

    /*!
     * \brief Erase recording of states ids.
     *
     * The recording start/stop state is not changed.
     */
    virtual void eraseRecording()
    {
      m_stackStateIds.clear();
    }

    /*!
     * \brief Get state machine id.
     *
     * \return State machine id.
     */
    int getStateMachId() const
    {
      return m_nStateMachId;
    }

    /*!
     * \brief Get state machine name.
     *
     * \return String.
     */
    std::string getStateMachName() const
    {
      return m_strStateMachName;
    }

    /*!
     * \brief Get the current state id.
     *
     * \return State id.
     */
    int getCurrStateId() const
    {
      return m_nCurrStateId;
    }

    /*!
     * \brief Get the previously recorded state id.
     * 
     * State id recording must be enabled. The previous state id never equals
     * the current state id.
     *
     * \return
     * If recording is enabled and there are elements on the stack, then the 
     * state id found on the top of the stack is returned.\n
     * Otherwise \ref State::StateIdUndef is returned.
     */
    int getPrevStateId() const
    {
      if( m_bIsRecording && (m_stackStateIds.size() > 0) )
      {
        return m_stackStateIds[m_stackStateIds.size()-1];
      }
      else
      {
        return State::StateIdUndef;
      }
    }

    /*!
     * \brief Print out state machine.
     *
     * \param fp      Output file pointer.
     * \param indent  Left indentation.
     */
    virtual void printStateMach(FILE *fp=stdout, int indent=0);

  protected:
    int               m_nStateMachId;       ///< state machine id
    std::string       m_strStateMachName;   ///< state machine name
    StateTblMap_T     m_mapStateTbl;        ///< state table
    int               m_nStartStateId;      ///< starting state id
    int               m_nCurrStateId;       ///< current state id
    int               m_nMarkStateId;       ///< mark state id of last run cycle
    std::vector<int>  m_stackStateIds;      ///< stack of previous state ids
    size_t            m_uMaxRecording;      ///< max recording length
    bool              m_bIsRecording;       ///< is [not] recording work flow
  
    /*!
     * \brief Push state id onto stack of state ids.
     *
     * The stack preserves the state workflow history.
     *
     * \note No consecutive states with the same id are pushed onto the stack.
     * That is, graph path cylces of length 1 are not recorded. However, cycle
     * lengths \h_gt 1 are not detected and, so, could be recorded. For example,
     * the state sequence a b a b (length == 2).
     *
     * \param nStateId  State id to be pushed.
     */
    void pushStateId(int nStateid);

    /*!
     * \brief Pop state id from stack of state ids.
     *
     * \return
     * If recording is enabled and there are elements on the stack, then the 
     * state id found on the top of the stack is returned.\n
     * Otherwise \ref State::StateIdUndef is returned.
     */
    int popStateId();

    /*!
     * \brief Log state transition.
     *
     * \todo Moderate logging.
     *
     * \param nCurrStateId    Current state id.
     * \param nEventId        Received event id.
     * \param nNextStateId    Next (new) state id.
     */
    virtual void logTransition(int nCurrStateId,
                               int nEventId,
                               int nNextStateId);
  };

} // namespace rnr


#endif // _RNR_STATE_MACH_H
