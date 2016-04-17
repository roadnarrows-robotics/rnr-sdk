////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      State.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief State base class implementation.
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

#include <stdio.h>
#include <stdarg.h>
#include <libgen.h>

#include <sstream>
#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Random.h"
#include "rnr/State.h"

using namespace std;
using namespace rnr;

//------------------------------------------------------------------------------
// StateEvent Class
//------------------------------------------------------------------------------

StateEvent::StateEvent(int           nEventId,
                       const string &strEventName,
                       const string &strActionName)
{
  m_nEventId = nEventId;

  if( strEventName.empty() )
  {
    stringstream ss;

    ss << "e" << m_nEventId;
    ss >> m_strEventName;
  }
  else
  {
    m_strEventName = strEventName;
  }

  if( strActionName.empty() )
  {
    stringstream  ss;

    ss << "action_" << m_strEventName;
    ss >> m_strActionName;
  }
  else
  {
    m_strActionName = strActionName;
  }
}
 
int StateEvent::execAction(State *pState,
                           int    nPrevStateId,
                           int    nEventId)
{
  return pState->m_nStateId;
}
 

//------------------------------------------------------------------------------
// State Class
//------------------------------------------------------------------------------

State::State(int            nStateId,
             const string  &strStateName,
             const string  &strRefTag,
             StateEvent    *listStateEvents[])
{
  int   i;

  init(nStateId, strStateName, strRefTag);

  for(i=0; listStateEvents[i] != NULL; ++i)
  {
    addStateEvent(listStateEvents[i]);
  }
}

State::~State()
{
  TransTblMap_T::iterator iter;

  for(iter=m_mapTransTbl.begin(); iter!=m_mapTransTbl.end(); ++iter)
  {
    delete iter->second;
  }
  m_mapTransTbl.clear();
}

int State::addStateEvents(StateEvent *pStateEvent, ...)
{
  va_list ap;
  int     i;

  va_start(ap, pStateEvent);

  for(i=0; pStateEvent != NULL; pStateEvent = va_arg(ap, StateEvent *), ++i)
  {
    addStateEvent(pStateEvent);
  }

  va_end(ap);

  return i;
}

void State::addStateEvent(StateEvent *pStateEvent)
{
  TransTblMap_T::iterator pos;

  if( pStateEvent != NULL )
  {
    if( (pos = m_mapTransTbl.find(pStateEvent->getEventId()))
                != m_mapTransTbl.end() )
    {
      LOGWARN("%s: Event %s.%d already exists - replacing.",
        m_strStateName.c_str(),
        pos->second->getEventName().c_str(), pos->second->getEventId());
      delete pos->second;
      m_mapTransTbl.erase(pos);
    }

    m_mapTransTbl[pStateEvent->getEventId()] = pStateEvent;

    LOGDIAG3("%s: %s.%d state event added.",
          m_strStateName.c_str(),
          pStateEvent->getEventName().c_str(), pStateEvent->getEventId());
  }
}

bool State::deleteStateEvent(int nEventId)
{
  TransTblMap_T::iterator pos;

  if( (pos = m_mapTransTbl.find(nEventId)) != m_mapTransTbl.end() )
  {
    delete pos->second;
    m_mapTransTbl.erase(pos);
    return true;
  }
  else
  {
    LOGERROR("%s: No event found in transition table with id=%d.",
        m_strStateName.c_str(), nEventId);
    return false;
  }
}

int State::receiveEvent()
{
  if( m_mapTransTbl.size() > 0 )
  {
    TransTblMap_T::iterator pos;
    Random                  rand;
    int                     i;

    i = rand.randrange(0, (int)m_mapTransTbl.size()-1);

    // no '+' or '+=' operator - so crawl through map
    for(pos = m_mapTransTbl.begin(); i > 0; --i, ++pos);

    return pos->second->getEventId();
  }
  else
  {
    return 0;
  }
}

int State::dispatchEvent(int nPrevStateId, int nEventId)
{
  TransTblMap_T::iterator   pos;
  StateEvent               *pEvent;
  int                       nNextStateId;

  //
  // Find event in transition table and execute. 
  //
  if( (pos = m_mapTransTbl.find(nEventId)) != m_mapTransTbl.end() )
  {
    pEvent = pos->second;

    // evaluate guard expression
    if( pEvent->evalGuardExpr(this, nPrevStateId, nEventId) )
    {
      // execute action
      nNextStateId = pEvent->execAction(this, nPrevStateId, nEventId);
    }

    // evaluated to false
    else
    {
      nNextStateId = m_nStateId;
    }
  }

  //
  // No event in transition table, so execute default action. 
  //
  else
  {
    nNextStateId = actionDefault(nPrevStateId, nEventId);
  }

  // stay is this state
  if( nNextStateId == StateIdThis )
  {
    nNextStateId = m_nStateId;
  }

  return nNextStateId;
}

string State::getEventName(int nEventId)
{
  TransTblMap_T::iterator pos;

  if( (pos = m_mapTransTbl.find(nEventId)) != m_mapTransTbl.end() )
  {
    return pos->second->getEventName();
  }
  else
  {
    return "";
  }
}

string State::getActionName(int nEventId)
{
  TransTblMap_T::iterator pos;

  if( (pos = m_mapTransTbl.find(nEventId)) != m_mapTransTbl.end() )
  {
    return pos->second->getActionName();
  }
  else
  {
    return "";
  }
}


void State::printState(FILE *fp, int indent)
{
  TransTblMap_T::iterator  iter;
  StateEvent              *pEvent;

  fprintf(fp, "%*sState: %s.%d\n",
      indent, "", m_strStateName.c_str(), m_nStateId);

  for(iter  = m_mapTransTbl.begin();
      iter != m_mapTransTbl.end();
      ++iter )
  {
    pEvent = iter->second;
    fprintf(fp, "%*sEvent: %s.%d %s\n",
      indent+2, "",
      pEvent->getEventName().c_str(), pEvent->getEventId(),
      pEvent->getActionName().c_str());
  }
}

void State::init(int           nStateId,
                 const string &strStateName,
                 const string &strRefTag)

{
  m_nStateId        = nStateId;
  m_bHasQueuedEvent = false;
  m_usecTimeOut     = 0;

  if( strStateName.empty() )
  {
    stringstream  ss;

    ss << "S" << m_nStateId;
    ss >> m_strStateName;
  }
  else
  {
    m_strStateName = strStateName;
  }

  if( strRefTag.empty() )
  {
    stringstream  ss;

    ss << m_nStateId;
    ss >> m_strRefTag;
  }
  else
  {
    m_strRefTag = strRefTag;
  }
}

string State::getReservedStateName(int nStateId)
{
  switch( nStateId )
  {
    case StateIdStart:
      return "*Start";
    case StateIdPrev:
      return "Previous";    // if recording, then ...
    case StateIdThis:
      return m_strStateName;
    case StateIdTerminate:
      return "Terminate*";
    case StateIdUndef:
      return "#undef";
    default:
      return "";
  }
}
