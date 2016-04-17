////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekMonitor.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-05-01 16:17:27 -0600 (Fri, 01 May 2015) $
 * $Rev: 3976 $
 *
 * \brief HekMonitor - Hekateros Monitor Class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#ifdef HEK_HAS_SYS_BOARD
#include "rnr/i2c.h"
#endif // HEK_HAS_SYS_BOARD

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"
#include "Dynamixel/DynaError.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekUno.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekDesc.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekState.h"
#include "Hekateros/hekMonitor.h"
#include "Hekateros/hekUtils.h"

using namespace std;
using namespace hekateros;


// -----------------------------------------------------------------------------
// Private Interface
// -----------------------------------------------------------------------------

#define KILOSEC 1000
#define MEGASEC 1000000
#define GIGASEC 1000000000

// RDK timeval and timespec should be classes and placed in appkit.
struct timespec now()
{
  struct timeval  tv;
  struct timespec ts;

  clock_gettime(CLOCK_REALTIME, &ts);

  return ts;
}

// t1 + t2
struct timespec add(const struct timespec &t1, const struct timespec &t2)
{
  struct timespec t3;

  t3.tv_sec  = t1.tv_sec + t2.tv_sec;
  t3.tv_nsec = t1.tv_nsec + t2.tv_nsec;

  if( t3.tv_nsec >= GIGASEC )
  {
    t3.tv_sec  += 1;
    t3.tv_nsec -= GIGASEC;
  }

  return t3;
}

// t1 - t2
struct timespec sub(const struct timespec &t1, const struct timespec &t2)
{
  struct timespec t3;

  t3.tv_sec  = t1.tv_sec - t2.tv_sec;
  t3.tv_nsec = t1.tv_nsec - t2.tv_nsec;

  if( t3.tv_nsec < 0 )
  {
    t3.tv_sec  -= 1;
    t3.tv_nsec += GIGASEC;
  }

  return t3;
}

// t1 <= t2
bool le(const struct timespec &t1, const struct timespec &t2)
{
  if( t1.tv_sec < t2.tv_sec )
  {
    return true;
  }
  else if( t1.tv_sec > t2.tv_sec )
  {
    return false;
  }
  else if( t1.tv_nsec < t2.tv_nsec )
  {
    return true;
  }
  else if( t1.tv_nsec > t2.tv_nsec )
  {
    return false;
  }
  else
  {
    return true;
  }
}

// time to double
double ttod(const struct timespec &t)
{
  return (double)t.tv_sec + ((double)t.tv_nsec)/((double)GIGASEC);
}


// -----------------------------------------------------------------------------
// Class HekMonitor
// -----------------------------------------------------------------------------

HekMonitor::HekMonitor()
{
  m_eMonState   = MonStateIdInit;
  m_bIsOpen     = false;
  m_bEStopCond  = false;
  m_bAlarmCond  = false;
  m_pDynaChain  = NULL;
  m_pKin        = NULL;

#ifdef HEK_HAS_SYS_BOARD
  // expandsion board hardware
  m_i2c.fd      = -1;
  m_i2c.addr    = 0;
#endif // HEK_HAS_SYS_BOARD

  m_byLimitBits = 0x00;
  m_byEEAuxBits = 0x00;

  // mutual exclusion variables
  pthread_mutex_init(&m_mutexMon, NULL);
  pthread_cond_init(&m_condMon,   NULL);

  // no monitor thread
  m_fHz             = 1.0;
  m_eMonThState     = MonThStateIdExit;
  m_eMonThStateOld  = MonThStateIdInit;
}

HekMonitor::~HekMonitor()
{
  close();

  pthread_cond_destroy(&m_condMon);
  pthread_mutex_destroy(&m_mutexMon);
}

int HekMonitor::open(uint_t        uHekHwVer,
                     const string &strDevArduino,
                     int           nBaudRateArduino)
{
  int   rc;

  if( m_bIsOpen )
  {
    LOGWARN("Monitoring hardware already opened.");
    return HEK_OK;
  }

  if( (rc = m_hwif.open(uHekHwVer, strDevArduino, nBaudRateArduino)) < 0 )
  {
    LOGERROR("Hekateros monitor hardware open failed.");
    return rc;
  }
  else if( (rc = m_hwif.scan()) < 0 )
  {
    LOGERROR("Hekateros monitor hardware scan failed.");
    return rc;
  }

  if( (rc = createMonThread()) < 0 )
  {
    LOGERROR("Hekateros monitor thread creation failed.");
    return rc;
  }

  m_eMonState = MonStateIdInit;
  m_bIsOpen   = true;

  LOGDIAG2("Opened monitoring hardware.");

  return HEK_OK;
}

int HekMonitor::close()
{
  if( m_bIsOpen )
  {
    destroyMonThread();

    m_hwif.close();

    m_eMonState   = MonStateIdInit;
    m_bIsOpen     = false;
    m_bEStopCond  = false;
    m_bAlarmCond  = false;
    m_pDynaChain  = NULL;
    m_pKin        = NULL;

    m_opticalLimits.clear();

#ifdef HEK_HAS_SYS_BOARD
    // expandsion board hardware
    m_i2c.fd            = -1;
    m_i2c.addr          = 0;
#endif // HEK_HAS_SYS_BOARD

    m_byLimitBits       = 0x00;
    m_byEEAuxBits       = 0x00;

    LOGDIAG2("Closed monitoring hardware.");
  }

  return HEK_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Monitoring Thread Control
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int HekMonitor::start(double fHz)
{
  double fPeriod;

  if( !m_bIsOpen )
  {
    LOGERROR("Monitor hardware not open.");
    return -HEK_ECODE_NO_RSRC;
  }
  else if( m_eMonThState != MonThStateIdIdle )
  {
    LOGERROR("Monitor thread not in idle state.");
    return -HEK_ECODE_BAD_OP;
  }
  else if( fHz <= 0.0 )
  {
    LOGERROR("Monitor thread run Hz=%lf invalid.", fHz);
    return -HEK_ECODE_BAD_OP;
  }

  m_fHz   = fHz;
  fPeriod = 1.0/fHz;

  m_tPeriod.tv_sec = (time_t)fPeriod;
  fPeriod -= (double)m_tPeriod.tv_sec;
  m_tPeriod.tv_nsec = (time_t)(fPeriod * GIGASEC);

  m_tStart = now();

  LOGDIAG3("Monitor thread started running at %.1lf Hz.", m_fHz);

  signalMonThread(MonThStateIdRun);

  m_eMonState = MonStateIdRunNormal;

  return HEK_OK;
}

int HekMonitor::stop()
{
  if( !m_bIsOpen )
  {
    LOGERROR("Monitor hardware not open.");
    return -HEK_ECODE_NO_RSRC;
  }
  else if( m_eMonThState != MonThStateIdRun )
  {
    LOGERROR("Monitor thread not in run state.");
    return -HEK_ECODE_BAD_OP;
  }

  LOGDIAG3("Monitor thread stopped - blocked waiting.");

  signalMonThread(MonThStateIdIdle);

  m_eMonState = MonStateIdInit;

  return HEK_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Objects to Monitor
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int HekMonitor::addJointLimitsToMonitor(HekSpecJoint_T *pSpecJoint,
                                        HekRobotJoint  *pJoint)
{
  HekJointOptical opticalLimit;
  int             i;

  for(i = 0; i < HekOptLimitMaxPerJoint; ++i)
  {
    if( pSpecJoint->m_limit[i].m_uBit == HekIOExpUnassigned )
    {
      break;
    }

    opticalLimit.m_uMask    = pSpecJoint->m_limit[i].m_uBit;
    opticalLimit.m_uCurVal  = HekIOExpDark;
    opticalLimit.m_pJoint   = pJoint;

    opticalLimit.m_limit.m_uBit = pSpecJoint->m_limit[i].m_uBit;
    opticalLimit.m_limit.m_fMinEdgePos = 
                                degToRad(pSpecJoint->m_limit[i].m_fMinEdgePos);
    opticalLimit.m_limit.m_fMinBlackPos = 
                                degToRad(pSpecJoint->m_limit[i].m_fMinBlackPos);
    opticalLimit.m_limit.m_fCenterPos = 
                                degToRad(pSpecJoint->m_limit[i].m_fCenterPos);
    opticalLimit.m_limit.m_fMaxBlackPos = 
                                degToRad(pSpecJoint->m_limit[i].m_fMaxBlackPos);
    opticalLimit.m_limit.m_fMaxEdgePos = 
                                degToRad(pSpecJoint->m_limit[i].m_fMaxEdgePos);


    m_opticalLimits[opticalLimit.m_uMask] = opticalLimit;
  }

  return HEK_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Monitor State & Conditions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void HekMonitor::markAlarmCond( bool bAlarmCond)
{
  m_bAlarmCond = bAlarmCond;

  writeAlarmLED(m_bAlarmCond || m_bEStopCond);
}

void HekMonitor::markEStopCond(bool bEStopCond)
{
  m_bEStopCond = bEStopCond;

  if( m_bEStopCond )
  {
    m_bAlarmCond = m_bEStopCond;
  }

  writeAlarmLED(m_bAlarmCond || m_bEStopCond);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Basic I/O
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

byte_t HekMonitor::readLimitSwitches()
{
  byte_t  byVal = 0;

  lock();

  //LOGDIAG3("Reading optical limits.");

  byVal = m_hwif.cmdReadLimits();

  unlock();

  return byVal;
}

byte_t HekMonitor::readEEGpio()
{
  byte_t  byVal = 0;

  lock();

  //LOGDIAG3("Reading optical switch pins");
 
  byVal = m_hwif.cmdReadAux();

  unlock();

  return byVal;
}

int HekMonitor::writeEEGpioPin(byte_t byPin, byte_t byVal)
{
  int   rc = HEK_OK;

  lock();

  rc = m_hwif.cmdWritePin(byPin, byVal);

  unlock();

  return rc;
}

int HekMonitor::configEEGpioPin(byte_t byPin, char cDir)
{
  int   rc = HEK_OK;

  lock();

  rc = m_hwif.cmdConfigPin(byPin, cDir);

  unlock();

  return rc;
}

void HekMonitor::writeAlarmLED(bool bState)
{
  lock();

  m_hwif.cmdSetAlarmLED(bState? 1: 0);

  unlock();
}

void HekMonitor::testInterface()
{
  lock();

  m_hwif.cmdTestInterface();

  unlock();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attibute Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

HekTriState HekMonitor::getJointLimitTriState(HekRobotJoint *pJoint, int nLimit)
{
  uint_t uMask = pJoint->m_byOptLimitMask[nLimit];

  if( uMask == HekIOExpUnassigned )
  {
    return HekTriStateUnknown;
  }
  else if( m_opticalLimits[uMask].m_uCurVal & HekIOExpLight )
  {
    return HekTriStateLight;
  }
  else
  {
    return HekTriStateDark;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Monitor Thread
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int HekMonitor::createMonThread()
{
  int   rc;

  m_eMonThState = MonThStateIdIdle;

  rc = pthread_create(&m_threadMon, NULL, HekMonitor::monThread, (void*)this);
 
  if( rc == 0 )
  {
    rc = HEK_OK;
  }

  else
  {
    LOGSYSERROR("pthread_create()");
    m_eMonThState = MonThStateIdExit;
    rc  = -HEK_ECODE_SYS;
  }

  return rc;
}

void HekMonitor::destroyMonThread()
{
  signalMonThread(MonThStateIdExit);

  usleep(1000);

  pthread_cancel(m_threadMon);
  pthread_join(m_threadMon, NULL);

  LOGDIAG3("Monitor thread canceled.");
}

void HekMonitor::signalMonThread(MonThStateId eNewThState)
{
  lock();

  m_eMonThStateOld  = m_eMonThState;
  m_eMonThState     = eNewThState;
  pthread_cond_signal(&m_condMon);

  unlock();
}

void HekMonitor::idleWait()
{
  lock();

  while( m_eMonThState == MonThStateIdIdle )
  {
    pthread_cond_wait(&m_condMon, &m_mutexMon);
  }
  m_tStart = now();

  unlock();
}

void HekMonitor::runWait()
{
  struct timespec tNow = now();

  lock();

  m_tStart = add(m_tStart, m_tPeriod);

  // tStart <= tNow
  if( le(m_tStart, tNow) )
  {
    LOGWARN("Monitor slipped timing by %lf seconds.",
        ttod(sub(tNow, m_tStart)));
    m_tStart = tNow;
  }
  else
  {
    pthread_cond_timedwait(&m_condMon, &m_mutexMon, &m_tStart);
  }

  unlock();
}

const char *HekMonitor::getThStateName(MonThStateId eMonThState)
{
  switch( eMonThState )
  {
    case MonThStateIdInit:
      return "start";
    case MonThStateIdIdle:
      return "idle";
    case MonThStateIdRun:
      return "run";
    case MonThStateIdExit:
      return "exit";
    default:
      return "unknown";
  }
}

#undef HEK_MON_TEST_IF   ///< define/undef to enable/disable debug test of i/f

void *HekMonitor::monThread(void *pArg)
{
  HekMonitor   *pThis = (HekMonitor *)pArg;
  int           oldstate;
  int           rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
  //pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &oldstate);

  pThis->m_eMonThStateOld = MonThStateIdInit;

  LOGDIAG3("Monitor thread created.");

  while( pThis->m_eMonThState != MonThStateIdExit )
  {
     if( pThis->m_eMonThStateOld != pThis->m_eMonThState )
     {
        LOGDIAG3("Monitor thread: %s -> %s state.",
          pThis->getThStateName(pThis->m_eMonThStateOld),
          pThis->getThStateName(pThis->m_eMonThState));
     }

    switch( pThis->m_eMonThState )
    {
      case MonThStateIdIdle:
        pThis->idleWait();
        break;

      case MonThStateIdRun:
        pThis->runWait();
#ifdef HEK_MON_TEST_IF
        pThis->testInterface();
#endif
        pThis->monJointLimits();
        pThis->monEEAuxIO();
        pThis->monServoAlarms();
        break;

      case MonThStateIdExit:
        break;

      default:
        LOGERROR("%d: Unknown monitor state.", pThis->m_eMonThState);
        pThis->m_eMonThState = MonThStateIdExit;
        break;
    }

    pThis->m_eMonThStateOld = pThis->m_eMonThState;
  }

  pThis->unlock();

  LOGDIAG3("Monitor thread exited.");

  return NULL;
}

void HekMonitor::monJointLimits()
{
  // past limit backoff speed (rads/sec)
  static double  TuneBackOffVel =  degToRad(10.0);

  MapOpticalLimits::iterator  iter;
  byte_t                      byMask;
  byte_t                      byVal;
  HekRobotJoint              *pJoint;
  DynaServo                  *pServo;
  int                         nOdPos;

  if( (m_pDynaChain == NULL) || (m_pKin == NULL) )
  {
    return;
  }

  m_byLimitBits = readLimitSwitches();

  for(iter = m_opticalLimits.begin(); iter != m_opticalLimits.end(); ++iter)
  {
    byMask = iter->first;
    pJoint = iter->second.m_pJoint;

    byVal = m_byLimitBits & byMask;

    //
    // Transitioned from light to dark.
    //
    if( (byVal == HekIOExpDark) && (byVal != iter->second.m_uCurVal) )
    {
      // have power to servos and this limit set to automatically stop
      if( m_bPoweredCond && pJoint->doStopAtLimits() )
      {
        string  strJointName;
        double  fJointCurPos, fJointCurVel;

        strJointName = pJoint->m_strName;

        m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

        if( fJointCurPos < pJoint->m_fMinSoftLimitRads )
        {
          m_pKin->move(strJointName, pJoint->m_fMinSoftLimitRads,
                                  TuneBackOffVel);
        }
        else if( fJointCurPos > pJoint->m_fMaxSoftLimitRads )
        {
          m_pKin->move(strJointName, pJoint->m_fMaxSoftLimitRads,
                                  TuneBackOffVel);
        }
        else
        {
          m_pKin->stop(strJointName);
        }
      }
      else
      {
        // could validate/resync position(s) here
      }
    }

    iter->second.m_uCurVal = byVal;
  }
}

void HekMonitor::monEEAuxIO()
{
}

void HekMonitor::monServoAlarms()
{
  int         iter;
  int         nServoId;
  DynaServo  *pServo;
  bool        bAlarmState = false;

  if( m_pDynaChain == NULL )
  {
    return;
  }

  //
  // Emergency stop forces alarm state.
  //
  if( m_bEStopCond )
  {
    bAlarmState = true;
  }

  //
  // Search all servos for alarms.
  //
  else
  {
    for(nServoId = m_pDynaChain->IterStart(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = m_pDynaChain->IterNext(&iter))
    {
      if( (pServo = m_pDynaChain->GetServo(nServoId)) != NULL )
      {
        if( pServo->GetAlarms() != DYNA_ALARM_NONE )
        {
          bAlarmState = true;
          break;
        }
      }
    }
  }

  //
  // New alarm state. Set LED and save new state.
  //
  if( bAlarmState != m_bAlarmCond )
  {
    writeAlarmLED(bAlarmState);
    m_bAlarmCond = bAlarmState;
  }
}
