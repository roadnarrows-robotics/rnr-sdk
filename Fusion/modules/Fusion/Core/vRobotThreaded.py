################################################################################
#
# vRobotThreaded.py
#

""" Threaded Virtual Robot Module.

The threaded version of vRobot provides a standard threaded interface
for both the Gluon Server play executives and for any derived 
vRobotThreaded class.

The execution cycle is defined as: SenseReact()

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.30

Copyright (C) 2006.  RoadNarrows LLC.
"""

#
# All Rights Reserved
#
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

import time
import threading as thread
import math

import Fusion.Utils.Tools as utils

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vRobot as vRobot

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: vRobotThreaded
#-------------------------------------------------------------------------------
class vRobotThreaded(vRobot.vRobot):
  """ Virtual Robot Threaded Virtual Class. """

  #--
  def vRobotInit(self):
    """ One-time vRobot initialization during object instantiation. """
    vRobot.vRobot.vRobotInit(self);

    # default execution cycle and step sizes
    self.SetExecSizes(0.1, 1.0)

    # thread initialization
    self.BotThreadInit()

  #--
  def __del__(self):
    """ Delete vRobotThreaded instance. """
    
    # Note: Destructor does not always work after the Bot thread has been
    # been created. Some weakness in python and/or the threading library.
    # Some of the class data gets wacked before chance to kill the thread.
    # Hangs for ever.
    #if self.mBotThread and self.mBotThread.isAlive():
    #  self.BotThreadKill()
    pass

  #--
  def SetExecSizes(self, execCycle, stepSize):
    """ Set execution cycle period and step size (seconds).

        Parameters:
          execCycle - execution cycle in seconds
          stepSize  - execution step size in seconds

        Return Value:
          None
    """
    self.mBotExecCycle  = execCycle
    self.mBotStepSize   = stepSize


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobotThreaded Default Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the physical robot.

        Callback hook for vRobotThreaded emergency stopping event.

        Requires override in derived class.
    """
    raise NotImplementedError("%s.DoEStop" % (self.__class__.__name__))

  #--
  def DoLoadInit(self):
    """ Load vRobot Initializer. Any auto-connect operation should be 
        done in the overloaded derived class equivalent.

        Callback hook for vRobotThreaded loading event.
    """
    self.IniInit()

  #--
  def DoStartInit(self):
    """ Start vRobot Initializer.

        Callback hook for vRobotThreaded starting or first-time
        stepping event.
    """
    self.SenseReactInit()

  #--
  def DoHold(self):
    """ Hold vRobot and physical robot at current position.

        Callback hook for vRobotThreaded suspending event.
    """
    self.SenseReactHold()

  #--
  def DoRelease(self):
    """ Release vRobot and physical robot continue operations.

        Callback hook for vRobotThreaded resuming event.
    """
    self.SenseReactRelease()

  #--
  def DoStopDeinit(self):
    """ Stop vRobot De-initializer.

        Callback hook for vRobotThreaded stopping event.
    """
    self.SenseReactDeinit()

  #--
  def DoUnloadDeinit(self):
    """ Unload vRobot De-initializer.

        Callback hook for vRobotThreaded unloading event.
    """
    self.pRobotDisconnect()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobotThreaded Default Sense-React Member Functions.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SenseReactInit(self):
    """ Initialize Sense-React Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    self.pRobotInit()
    self.ShadowInit()
    self.BellumInit()
    self.mOnHold = False

  #--
  def SenseReactHold(self):
    """ Hold Sense-React operations.

        Event: Suspeneding event

        Execution Context: Calling thread
    """
    self.pRobotHold()
    self.mOnHold = True

  #--
  def SenseReactRelease(self):
    """ Release Sense-React operations.

        Event: Resuming event

        Execution Context: Calling thread
    """
    self.pRobotRelease()
    self.mOnHold = False

  #--
  def SenseReactDeinit(self):
    """ De-initialize Sense-React Operations.

        Event: Stopping event

        Execution Context: Calling thread
    """
    pass

  #--
  def SenseReact(self):
    """ Sense-React Behavior.

        Execution Context: Bot thread
    """
    if self.mOnHold:
      return
    if __debug__: self.mDbg.d4print('SenseReact')
    self.ShadowUpdate()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobotThreaded Play Executives
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ExecEStop(self):
    """ Emergency Stop.

        This function may be called at any time.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecEStop')
    if not self.GSIsValidExecState(Gluon.EServerExec.EStop):
      return
    self.SetRobotState(Gluon.EServerState.Errored)
    self.DoEStop()
    self.BotThreadStop()
    self.GSReportErrorStatus('Emergency Stop')

  #--
  def ExecLoad(self):
    """ Load the vRobot.
    
        (Re)Initialize the vRobot configuration. The Bot thread is created.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecLoad')
    if not self.GSIsValidExecState(Gluon.EServerExec.Load):
      return
    self.DoLoadInit()
    self.BotThreadCreate()
    if self.IsCommUp():
      self.SetRobotState(Gluon.EServerState.Ready)
    else:
      self.GSReportNormalStatus('To enable, connect to the %s' % self.HasName())

  #--
  def ExecStart(self):
    """ Start the vRobot sensing and reacting.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecStart')
    if not self.GSIsValidExecState(Gluon.EServerExec.Start):
      return
    self.DoStartInit()
    self.BotThreadRun()

  #--
  def ExecStep(self):
    """ Step vRobot one execution cycle.
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecStep')
    if not self.GSIsValidExecState(Gluon.EServerExec.Step):
      return
    if self.mServerState == Gluon.EServerState.Ready:
      self.DoStartInit()
    else:
      self.DoRelease()
    self.BotThreadStep()

  #--
  def ExecResume(self):
    """ Resume vRobot functions with current shadow and bellum data.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecResume')
    if not self.GSIsValidExecState(Gluon.EServerExec.Resume):
      return
    self.DoRelease()
    self.BotThreadRun()

  #--
  def ExecSuspend(self):
    """ Suspend vRobot activity. All movements and low-level functions
        will be put on hold. The current set of data are kept.
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecSuspend')
    if not self.GSIsValidExecState(Gluon.EServerExec.Suspend):
      return
    self.BotThreadPause()

  #--
  def ExecStop(self):
    """ Stop the current vRobot activities. All sensing and moving are
        stopped. The current shadow and bellum data are lost. The Bot
        thread is restarted with re-initialized data.
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecStop')
    if not self.GSIsValidExecState(Gluon.EServerExec.Stop):
      return
    self.BotThreadStop()
    self.DoStopDeinit()

  #--
  def ExecUnload(self):
    """ Exit vRobot instance, killing the Bot thread and performing any
        other resource cleanup. The connnection is closed.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecUnload')
    if not self.GSIsValidExecState(Gluon.EServerExec.Unload):
      return
    self.BotThreadKill()
    self.DoUnloadDeinit()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobotThreaded Bot Thread Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BotThreadInit(self):
    """ One time initialization of the Bot thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadInit')
    self.mBotSema       = thread.Semaphore()
    self.mBotExecEvent  = thread.Event()
    self.mBotCanLive    = False
    self.mBotGo         = False
    self.mBotStep       = False
    self.mBotStepTime   = 0.0
    self.mBotThread     = None
    self.mTWall         = 0.0
    self.mTExecStart    = 0.0

  #--
  def BotThreadCreate(self):
    """ Create the Bot thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadCreate')
    self.mBotSema.release()     # make sure semaphore is acquireable
    self.mBotSema.acquire()
    self.mBotCanLive  = True
    self.mBotGo       = False
    self.mBotStep     = False
    self.mBotThread   = thread.Thread(target=self.BotThread,
        name='Bot'+self.HasName(), kwargs={})
    self.mBotThread.start()
    self.SetRobotState(Gluon.EServerState.NotReady)

  #--
  def BotThreadRun(self):
    """ Run the Bot thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadRun')
    self.mBotGo   = True
    self.mBotStep = False
    self.SetRobotState(Gluon.EServerState.Running)
    self.mBotSema.release()

  #--
  def BotThreadPause(self):
    """ Pause (suspend) the Bot thread.

        Execution Context: Calling thread, Bot Thread
    """
    if __debug__: self.mDbg.d3print('BotThreadPause')
    self.mBotGo   = False
    self.mBotStep = False
    self.SetRobotState(Gluon.EServerState.Paused)

  #--
  def BotThreadStep(self):
    """ Step the Bot thread one execution cycle.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadStep')
    self.mBotGo       = True
    self.mBotStep     = True
    self.mBotStepTime = self.mBotStepSize
    self.SetRobotState(Gluon.EServerState.Stepping)
    self.mBotSema.release()

  #--
  def BotThreadStop(self):
    """ Stop the Bot thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadStop')
    self.mBotGo   = False
    self.mBotStep = False
    self.SetRobotState(Gluon.EServerState.Ready)

  #--
  def BotThreadKill(self):
    """ Kill the Bot thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('BotThreadKill')
    self.mBotGo      = False
    self.mBotStep    = False
    self.mBotCanLive = False
    self.mBotSema.release()
    self.mBotExecEvent.set()
    if self.mBotThread and self.mBotThread.isAlive():
      self.mBotThread.join(2.0)
    if self.mBotThread and self.mBotThread.isAlive():
      if __debug__: self.mDbg.d3print('BotThreadKill - second try')
      self.mBotGo      = False
      self.mBotStep    = False
      self.mBotCanLive = False
      self.mBotSema.release()
      self.mBotExecEvent.set()
      if self.mBotThread and self.mBotThread.isAlive():
        self.mBotThread.join(2.0)
    if self.mBotThread and self.mBotThread.isAlive():
      if __debug__: self.mDbg.d3print('BotThread failed to die')
    self.SetRobotState(Gluon.EServerState.NotLoaded)

  #--
  def BotThreadCleanup(self):
    """ Clean-up any Bot thread resources.

        Execution Context: Bot thread
    """
    self.mBotCanLive  = False
    self.mBotGo       = False
    self.mBotStep     = False

  #--
  def BotThread(self, args=(), kwargs={}):
    """ The Bot thread. 
    
        This Bot thread provides the shadowed interface to the physical 
        vRobot's robot. It senses the vRobot environment through the
        robot's sensors and reacts to this sensed environment based
        on the current low-level goals through the robot's effectors.
        The Bot thread executes at the opted time step.
    """
    if __debug__: self.mDbg.d3print('BotThread: alive')

    t_wait  = 0.0   # time to wait between execution cycles

    while self.mBotCanLive:
      # Run/Block
      self.mTWall = time.time() # wall clock for outer loop
      self.mBotSema.acquire()   # block forever until Run/Kill
      if not self.mBotCanLive:  # exit thread by falling through
        break

      # Execution cycle start time
      self.mTExecStart = time.time()
      if __debug__: self.mDbg.d4print('Exec start: %f' % self.mTExecStart)

      # Wait for next cycle to begin
      if self.mBotGo:           # running
        self.mBotExecEvent.wait(t_wait)
      if not self.mBotCanLive:  # exit thread by falling through
        break

      # Execution Cycle
      if self.mBotGo:           # running
        self.SenseReact()       #  sense the environment and react

      # Delta and Adjusted times
      dt = time.time() - self.mTExecStart # includes last t_wait
      t_wait = self.mBotExecCycle + t_wait - dt
      if t_wait < 0.0:
        t_wait = 0.0
      # RDK!!! perfhook here

      # Determine what to do after execution cycle
      if self.mBotStep:         # stepping one cycle
        self.mBotStepTime -= dt
        if self.mBotStepTime <= 0.0:
          self.BotThreadPause() #  autopause
      if self.mBotGo:           # running
        self.mBotSema.release() #  release to run new cycle
      else:                     # stop running
        self.DoHold()           #  put a hold on vRobot and physical robot
        t_wait = 0.0            #  run next cycle immediately after resume

    # Thread is dying, but don't do any crying
    self.BotThreadCleanup()
    if __debug__: self.mDbg.d3print('BotThread: dead')


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import sys

  vr = vRobotThreaded(debuglevel=3)
  doquit = False

  def quit():
    global doquit
    vr.BotThreadKill()
    doquit = True

  def tsthelp():
    for k in execs.keys():
      print(k)

  def togglecomm():
    if vr.IsCommUp():
      vr.SetCommStatus(False)
    else:
      vr.SetCommStatus(True)

  execs = {
    'quit':     quit,
    'help':     tsthelp,
    'estop':    vr.ExecEStop,
    'load':     vr.ExecLoad,
    'start':    vr.ExecStart,
    'suspend':  vr.ExecSuspend,
    'resume':   vr.ExecResume,
    'step':     vr.ExecStep,
    'stop':     vr.ExecStop,
    'unload':   vr.ExecUnload,
    'comm':     togglecomm
  }

  def main():
    """ vRobotThreaded Test Main """
    print("Enter executive ('help' for list of commands)")
    while not doquit:
      rsp = input('exec> ')
      for k, op in execs.items():
        if rsp == k:
          op()
          break
    #time.sleep(5)

  # run test
  main()
