////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Unit Test: Test State Machine Class.
//  
/*! \file
 *  
 * $LastChangedDate: 2013-05-03 08:51:57 -0600 (Fri, 03 May 2013) $
 * $Rev: 2905 $
 *
 *  \ingroup appkit_ut
 *  
 *  \brief Unit test for librnr_appkit StateMach, State, StateEvent, and
 *  StateKb classes.
 *
 *  \author Robin Knight (robin.knight@roadnarrows.com)
 *  
 *  \par Copyright:
 *  (C) 2013, RoadNarrows LLC
 *  (http://roadnarrows.com)
 *  All rights reserved.
 */
/*
 * @EulaBegin@
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met: 
//
// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution. 
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are 
// those of the authors and should not be interpreted as representing official 
// policies, either expressed or implied, of the FreeBSD Project.
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Random.h"
#include "rnr/State.h"
#include "rnr/StateKb.h"
#include "rnr/StateMach.h"

#include "gtest/gtest.h"

using namespace ::std;
using namespace ::rnr;

/*!
 *  \ingroup appkit_ut
 *  \defgroup appkit_ut_sm State Machine Tests
 *  \brief Fine-grained testing of the StateMach, State, StateKb, and
 *  StateEvent classes.
 *  \{
 */

static const int    StateIdA            = State::StateIdNumOf;      ///< state A
static const int    StateIdB            = State::StateIdNumOf + 1;  ///< state B

static const int    EventIdPauseResume  = 'p';        ///< pause/resume event id
static const int    EventIdTerminate    = 'q';        ///< terminate event id
static const int    EventIdRestart      = 'r';        ///< restart event id
static const int    EventIdStart        = 's';        ///< start event id
static const int    EventIdSymbol[]     = {' ', '@', '#'};
                                                      ///< tape symbol event ids

static const char  *ColorHead           = "\033[0;31m"; ///< color tape head
static const char  *ColorSymbol[]       = {"", "\033[0;33m", "\033[0;35m"};
                                                        ///< color of symbols
static const char  *ColorEnd[]          = {"", "\033[0m", "\033[0m"};
                                                        ///< end color

static const uint_t TRead               = 500000;   ///< read tape time interval

/*!
 * \brief Test menu.
 */
static const char *TestMenu = 
  "\nTest Menu\n"
  "  'p'     pause/resume state machine\n"
  "  'q'     quite (terminate) state machine\n"
  "  'r'     restart state machine\n"
  "  's'     start state machine\n";


// .............................................................................
// Common State Events
// .............................................................................

/*!
 * \brief Start state event class.
 */
class StateEventStart : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventStart() : StateEvent(EventIdStart, "start") { }

  /*! \brief Destructor. */
  virtual ~StateEventStart() { }

  /*!
   * \brief Execute event action.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    return StateIdA;
  }
};

/*!
 * \brief Restart state event class.
 */
class StateEventRestart : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventRestart() : StateEvent(EventIdRestart, "restart") { }

  /*! \brief Destructor. */
  virtual ~StateEventRestart() { }

  /*!
   * \brief Execute event action.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    printf("%s", TestMenu);
    return State::StateIdStart;
  }
};

/*!
 * \brief Pause/resume state event class.
 */
class StateEventPauseResume : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventPauseResume() : StateEvent(EventIdPauseResume, "pause/resume") { }

  /*! \brief Destructor. */
  virtual ~StateEventPauseResume() { }

  /*!
   * \brief Execute event action.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    if( pState->getTimeOut() == 0 )
    {
      pState->setTimeOut(TRead);
    }
    else
    {
      pState->setTimeOut(0);
    }
    return State::StateIdThis;
  }
};

/*!
 * \brief Terminate state event class.
 */
class StateEventTerminate : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventTerminate() : StateEvent(EventIdTerminate, "terminate") { }

  /*! \brief Destructor. */
  virtual ~StateEventTerminate() { }

  /*!
   * \brief Execute event action.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    return State::StateIdTerminate;
  }
};


// .............................................................................
// (2,3) Turing State
// .............................................................................

/*!
 * \brief The (2,3) Turing state base class.
 */
class State2_3Turing : public StateKb
{
public:
  static vector<int>  Tape;     ///< the tape of symbols
  static int          Head;     ///< the tape read/write head

  /*!
   * \brief Initializatoin constuctor.
   *
   * \param nStateId      State id. Must be state machine unique.
   * \param strStateName  State name.
   * \param strRefTag     State reference id.
   */
  State2_3Turing(int                nStateId,
                 const std::string &strStateName="",
                 const std::string &strRefTag="") :
    StateKb(nStateId, strStateName, strRefTag)
  {
  }

  /*! \brief Destructor. */
  virtual ~State2_3Turing() { }

  /*
   * \brief Read symbol from tape at the current head position.
   *
   * \return Symbol.
   */
  int readSymbol()
  {
    return Tape[Head];
  }

  /*
   * \brief Write symbol to tape at the current head position.
   *
   * \param sym   Symbol.
   */
  void writeSymbol(int sym)
  {
    Tape[Head] = sym;
  }

  /*!
   * \brief Move head left one position on the tape.
   */
  void moveHeadLeft()
  {
    if( Head > 0 )
    {
      --Head;
    }
  }

  /*!
   * \brief Move head right one position on the tape.
   */
  void moveHeadRight()
  {
    if( Head < Tape.size()-1 )
    {
      ++Head;
    }
  }

  /*!
   * \brief Show colorized tape to stdout.
   */
  void showTape()
  {  
    int     sym;
    size_t  i, j;

    for(i=0; i<Tape.size(); ++i)
    {
      sym = Tape[i];

      if( i == Head )
      {
        printf("%s%c%s", ColorHead, sym, ColorEnd[1]);
      }

      else
      {
        for(j=0; j<3; ++j)
        {
          if( sym == EventIdSymbol[j] )
          {
            printf("%s%c%s", ColorSymbol[j], sym, ColorEnd[j]);
            break;
          }
        }
      }
    }

    printf("\n");
    fflush(stdin);
  }

};

vector<int>  State2_3Turing::Tape;    ///< tape of symbols
int          State2_3Turing::Head;    ///< the tape read/write head


// .............................................................................
// Start State
// .............................................................................

/*!
 * \brief Start state event class.
 *
 * The start state is not part of the "official" (2,3) Turing Machine. Rather
 * it is used to (re)initize the tape and then transition to the A state.
 */
class StateStart : public State2_3Turing
{
public:
  /*! \brief Constuctor. */
  StateStart() : State2_3Turing(StateIdStart, "Start", "S")
  {
    addStateEvents(new StateEventStart(),
                   new StateEventRestart(),
                   new StateEventTerminate(),
                   NULL);
  }

  /*! \brief Destructor. */
  virtual ~StateStart() { }

  /*!
   * \brief Execute 'enter state' action.
   *
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   */
  virtual void actionEnterState(int nPrefStateId, int nEventId)
  {
    Random  rand;
    int     n;
    int     sym;
    size_t  i;

    Tape.clear();

    n = rand.randrange(0, 10);

    printf("\n");

    for(i=0; i<39-n/2; ++i)
    {
      Tape.push_back(EventIdSymbol[0]);
    }

    for(i=0; i<n; ++i)
    {
      Tape.push_back(EventIdSymbol[rand.randrange(0, 2)]);
    }

    for(i=Tape.size(); i<80; ++i)
    {
      Tape.push_back(EventIdSymbol[0]);
    }

    Head = 41;

    showTape();
  }

};


// .............................................................................
// (2,3) Turing State A
// .............................................................................

/*!
 * \brief (2,3) Turing Machine's state A, symbol 0 event.
 */
class StateEventASym0 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventASym0() : StateEvent(EventIdSymbol[0], "sym0", "action_P1_R_B") { }

  /*! \brief Destructor. */
  virtual ~StateEventASym0() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head < 79? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 1, move right, enter state B.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[1]);
    ((State2_3Turing*)pState)->moveHeadRight();
    ((State2_3Turing*)pState)->showTape();
    return StateIdB;
  }
};

/*!
 * \brief (2,3) Turing Machine's state A, symbol 1 event.
 */
class StateEventASym1 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventASym1() : StateEvent(EventIdSymbol[1], "sym1", "action_P2_L_A") { }

  /*! \brief Destructor. */
  virtual ~StateEventASym1() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head > 0? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 2, move left, stay in state A.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[2]);
    ((State2_3Turing*)pState)->moveHeadLeft();
    ((State2_3Turing*)pState)->showTape();
    return StateIdA;
  }
};

/*!
 * \brief (2,3) Turing Machine's state A, symbol 2 event.
 */
class StateEventASym2 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventASym2() : StateEvent(EventIdSymbol[2], "sym2", "action_P1_L_A") { }

  /*! \brief Destructor. */
  virtual ~StateEventASym2() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head > 0? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 1, move left, stay in state A.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[1]);
    ((State2_3Turing*)pState)->moveHeadLeft();
    ((State2_3Turing*)pState)->showTape();
    return StateIdA;
  }
};

/*!
 * \brief (2,3) Turing Machine's state A.
 */
class StateA : public State2_3Turing
{
public:
  /*! \brief Constuctor. */
  StateA() : State2_3Turing(StateIdA, "A", "A")
  {
    addStateEvents(new StateEventPauseResume(),
                   new StateEventRestart(),
                   new StateEventTerminate(),
                   new StateEventASym0(),
                   new StateEventASym1(),
                   new StateEventASym2(),
                   NULL);

    setTimeOut(TRead);
  }

  /*! \brief Destructor. */
  virtual ~StateA() { }

  /*!
   * \brief Receive the next event.
   *
   * \return Event id.
   */
  virtual int receiveEvent()
  {
    int nEventId;

    nEventId = StateKb::receiveEvent();

    switch( nEventId )
    {
      case EventIdPauseResume:
      case EventIdRestart:
      case EventIdTerminate:
        return nEventId;
      case KbEventTimeOut:
        return readSymbol();
      default:
        return '?';   // force default action == this state
    }
  }
};


// .............................................................................
// (2,3) Turing State B
// .............................................................................

/*!
 * \brief (2,3) Turing Machine's state B, symbol 0 event.
 */
class StateEventBSym0 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventBSym0() : StateEvent(EventIdSymbol[0], "sym0", "action_P2_L_A") { }

  /*! \brief Destructor. */
  virtual ~StateEventBSym0() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head > 0? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 2, move left, enter state A.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[2]);
    ((State2_3Turing*)pState)->moveHeadLeft();
    ((State2_3Turing*)pState)->showTape();
    return StateIdA;
  }
};

/*!
 * \brief (2,3) Turing Machine's state B, symbol 1 event.
 */
class StateEventBSym1 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventBSym1() : StateEvent(EventIdSymbol[1], "sym1", "action_P2_R_B") { }

  /*! \brief Destructor. */
  virtual ~StateEventBSym1() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head < 79? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 2, move right, stay in state B.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[2]);
    ((State2_3Turing*)pState)->moveHeadRight();
    ((State2_3Turing*)pState)->showTape();
    return StateIdB;
  }
};

/*!
 * \brief (2,3) Turing Machine's state B, symbol 2 event.
 */
class StateEventBSym2 : public StateEvent
{
public:
  /*! \brief Constuctor. */
  StateEventBSym2() : StateEvent(EventIdSymbol[2], "sym2", "action_P0_R_A") { }

  /*! \brief Destructor. */
  virtual ~StateEventBSym2() { }

  /*!
   * \brief Evaluate event guard condition.
   *
   * \param pState        Associated state.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Received event id.
   *
   * \return Returns true if event is accepted, otherwise returns false.
   */
  virtual bool evalGuardExpr(State *pState, int nPrevStateId, int nEventId)
  {
    return ((State2_3Turing*)pState)->Head < 79? true: false;
  }

  /*!
   * \brief Execute event action.
   *
   * Print symbol 0, move right, enter state A.
   *
   * \param pState        State owning this event.
   * \param nPrevStateId  Previous state id.
   * \param nEventId      Event id.
   *
   * \return Next state id.
   */
  virtual int execAction(State *pState, int nPrevStateId, int nEventId)
  {
    ((State2_3Turing*)pState)->writeSymbol(EventIdSymbol[0]);
    ((State2_3Turing*)pState)->moveHeadRight();
    ((State2_3Turing*)pState)->showTape();
    return StateIdA;
  }
};

/*!
 * \brief (2,3) Turing Machine's state B.
 */
class StateB : public State2_3Turing
{
public:

  /*! \brief Constuctor. */
  StateB() : State2_3Turing(StateIdB, "B", "B")
  {
    addStateEvents(new StateEventPauseResume(),
                   new StateEventRestart(),
                   new StateEventTerminate(),
                   new StateEventBSym0(),
                   new StateEventBSym1(),
                   new StateEventBSym2(),
                   NULL);

    setTimeOut(TRead);
  }

  /*! \brief Destructor. */
  virtual ~StateB() { }

  /*!
   * \brief Receive the next event.
   *
   * \return Event id.
   */
  virtual int receiveEvent()
  {
    int nEventId;

    nEventId = StateKb::receiveEvent();

    switch( nEventId )
    {
      case EventIdPauseResume:
      case EventIdRestart:
      case EventIdTerminate:
        return nEventId;
      case KbEventTimeOut:
        return readSymbol();
      default:
        return '?';   // force default action == this state
    }
  }
};


// .............................................................................
// (2,3) Turing State Machine Test
// .............................................................................

/*!
 *  \brief Test State Machine.
 *
 * \par (2,3) Turing Machine:
 *  The 2-state 3-color(symbol) Turing machine (hereinafter (2,3) Turing
 *  machine) might be universal as well.\n\n
 *  See
 *  http://en.wikipedia.org/wiki/Wolfram%27s_2-state_3-symbol_Turing_machine
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testSM()
{
  // manually set rnr log level
  //LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  State *listOfStates[] = {new StateStart(), new StateA(), new StateB(), NULL};

  StateMach sm(0, "(2,3)Turing", State::StateIdStart, listOfStates);

  sm.printStateMach();

  printf("%s", TestMenu);

  sm.run();

  LOG_SET_THRESHOLD(LOG_LEVEL_OFF);

  return 0;
}


#ifndef JENKINS

/*!
 * \brief Test State Machine class.
 *
 * \par The Test:
 * Construct a (2,3) Turing State Machin and run.
 */
TEST(SM, SM2_3Turing)
{
  EXPECT_TRUE( testSM() == 0 );
}

#endif // JENKINS


/*!
 *  \}
 */
