////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      StateMach.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Finite State Machine implementation.
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

#include <stdio.h>
#include <stdarg.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StateMach.h"
#include "rnr/appkit/State.h"


using namespace std;
using namespace rnr;

/*!
 * \brief Log state transition at diagnostic level 3.
 *
 * \param nCurrStateId    Current state id.
 * \param nEventId        Received event id.
 * \param nNextStateId    Next (new) state id.
 */
#define LOGTRANSITION(nCurrStateId, nEventId, nNextStateId) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG3) ) \
    { \
      logTransition(nCurrStateId, nEventId, nNextStateId); \
    } \
  } while(0)


//----------------------------------------------------------------------------
// StateMach Class
//----------------------------------------------------------------------------

StateMach::StateMach(int nStateMachId, const string &strStateMachName) :
      m_strStateMachName(strStateMachName)
{
  m_nStateMachId  = nStateMachId;
  m_nStartStateId = State::StateIdStart;
  m_nCurrStateId  = State::StateIdUndef;
  m_nMarkStateId  = State::StateIdUndef;
  m_bIsRecording  = false;
}

StateMach::StateMach(int           nStateMachId,
                     const string &strStateMachName,
                     int           nStartStateId,
                     State        *listStates[]) :
      m_strStateMachName(strStateMachName)
{
  int   i;

  m_nStateMachId  = nStateMachId;
  m_nStartStateId = nStartStateId;
  m_nCurrStateId  = State::StateIdUndef;
  m_nMarkStateId  = State::StateIdUndef;
  m_bIsRecording  = false;

  for(i=0; listStates[i] != NULL; ++i)
  {
    addState(listStates[i]);
  }
}

StateMach::~StateMach()
{
  StateTblMap_T::iterator iter;

  for(iter=m_mapStateTbl.begin(); iter!=m_mapStateTbl.end(); ++iter)
  {
    delete iter->second;
  }

  m_mapStateTbl.clear();
}

int StateMach::addStates(State *pState, ...)
{
  va_list ap;
  int     i;

  va_start(ap, pState);

  for(i=0; pState != NULL; pState = va_arg(ap, State *), ++i)
  {
    addState(pState);
  }

  va_end(ap);

  return i;
}

void StateMach::addState(State *pState)
{
  StateTblMap_T::iterator pos;

  if( pState != NULL )
  {
    if((pos = m_mapStateTbl.find(pState->getStateId())) != m_mapStateTbl.end())
    {
      LOGWARN("%s.%d: State [%s.%d] already exists - replacing.",
        m_strStateMachName.c_str(), m_nStateMachId,
        pos->second->getStateName().c_str(), pos->second->getStateId());
      delete pos->second;
      m_mapStateTbl.erase(pos);
    }

    m_mapStateTbl[pState->getStateId()] = pState;

    LOGDIAG3("%s.%d: [%s.%d] state added.",
          m_strStateMachName.c_str(), m_nStateMachId,
          pState->getStateName().c_str(), pState->getStateId());
  }
}

bool StateMach::deleteState(int nStateId)
{
  StateTblMap_T::iterator pos;

  if( (pos = m_mapStateTbl.find(nStateId)) != m_mapStateTbl.end() )
  {
    delete pos->second;
    m_mapStateTbl.erase(pos);
    return true;
  }
  else
  {
    LOGERROR("%s.%d: No state found in state table with state id=%d.",
        m_strStateMachName.c_str(), m_nStateMachId, nStateId);
    return false;
  }
}

void StateMach::run()
{
  StateTblMap_T::iterator   pos;
  State                    *pState;
  int                       nEventId;
  int                       nNextStateId;

  m_nMarkStateId  = m_nCurrStateId;
  m_nCurrStateId  = m_nStartStateId;

  // run from the start
  while( m_nCurrStateId != State::StateIdTerminate )
  {
    //
    // Found the state in state machine. (Block) receive event and dispatch.
    //
    if( (pos = m_mapStateTbl.find(m_nCurrStateId)) != m_mapStateTbl.end() )
    {
      pState = pos->second;

      // entering this state, execute any 'enter state' action
      if( m_nMarkStateId != m_nCurrStateId )
      {
        pState->actionEnterState(m_nMarkStateId, nEventId);
      }

      // (block) receive event
      nEventId = pState->receiveEvent();

      // dispatch event
      nNextStateId = pState->dispatchEvent(m_nMarkStateId, nEventId);

      // this state
      if( nNextStateId == State::StateIdThis )
      {
        nNextStateId = m_nCurrStateId;
      } 

      // previous state
      else if( nNextStateId == State::StateIdPrev )
      {
        nNextStateId = popStateId();
      }

      // exiting this state, execute any 'exit state' action
      if( m_nCurrStateId != nNextStateId )
      {
        pState->actionExitState(nNextStateId, nEventId);
      }

      // log
      LOGTRANSITION(m_nCurrStateId, nEventId, nNextStateId);

      m_nMarkStateId = m_nCurrStateId;
      m_nCurrStateId = nNextStateId;

      // record state (if enabled)
      pushStateId(m_nMarkStateId);
    }

    //
    // Badly formed state machine. No state associated with state id. Terminate.
    //
    else
    {
      LOGERROR("%s.%d: No state found in state table with state id=%d.",
              m_strStateMachName.c_str(), m_nStateMachId,
              m_nCurrStateId);
      m_nCurrStateId = State::StateIdTerminate;
    }
  }
}

void StateMach::pushStateId(int nStateId)
{
  size_t  n;

  if( m_bIsRecording )
  {
    if( (n = m_stackStateIds.size()) == 0 )
    {
      m_stackStateIds.push_back(nStateId);
    }
    else if( (m_uMaxRecording != NO_LIMIT) && (n >= m_uMaxRecording) )
    {
      m_stackStateIds.erase(m_stackStateIds.begin());
      m_stackStateIds.push_back(nStateId);
    }
    else if( m_stackStateIds[n-1] != nStateId )
    {
      m_stackStateIds.push_back(nStateId);
    }
  }
}

int StateMach::popStateId()
{
  int   nStateId;

  if( m_bIsRecording && (m_stackStateIds.size() > 0) )
  {
    nStateId = m_stackStateIds.back();
    m_stackStateIds.pop_back();
    return nStateId;
  }
  else
  {
    return State::StateIdUndef;
  }
}

void StateMach::printStateMach(FILE *fp, int indent)
{
  StateTblMap_T::iterator iter;

  fprintf(fp, "%*sState Machine: %s.%d\n",
      indent, "", m_strStateMachName.c_str(), m_nStateMachId);

  for(iter = m_mapStateTbl.begin(); iter != m_mapStateTbl.end(); ++iter)
  {
    iter->second->printState(fp, indent+2);
  }
}

void StateMach::logTransition(int nCurrStateId, int nEventId, int nNextStateId)
{
  static const char *sUndef = "#undef";

  StateTblMap_T::iterator pos;
  string                  strCurr;
  string                  strEvent;
  string                  strAction;
  string                  strNext;

  if( (pos = m_mapStateTbl.find(nCurrStateId)) != m_mapStateTbl.end() )
  {
    strCurr   = pos->second->getStateName();
    strEvent  = pos->second->getEventName(nEventId);
    strAction = pos->second->getActionName(nEventId);
  }
  else
  {
    strCurr   = sUndef;
    strEvent  = sUndef;
    strAction = sUndef;
  }

  if( (pos = m_mapStateTbl.find(nNextStateId)) != m_mapStateTbl.end() )
  {
    strNext = pos->second->getStateName();
  }
  else
  {
    strNext = "#undef";
  }

  LOGDIAG3("%s.%d: [%s.%d] -- %s(%s.%d) --> [%s.%d]",
              m_strStateMachName.c_str(), m_nStateMachId,
              strCurr.c_str(), nCurrStateId,
              strAction.c_str(), strEvent.c_str(), nEventId,
              strNext.c_str(), nNextStateId);
}
