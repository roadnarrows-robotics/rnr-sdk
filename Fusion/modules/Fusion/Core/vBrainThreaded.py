################################################################################
#
# vBrainThreaded.py
#

""" Threaded Virtual Brain Module.

The threaded version of vBrain provides a standard threaded interface
for both the Gluon Server play executives and for any derived
vBrainThreaded class.

The execution cycle is defined as: ThinkAct()

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.31

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
import Fusion.Core.vBrain as vBrain


#-------------------------------------------------------------------------------
# CLASS: vBrainThreaded
#-------------------------------------------------------------------------------
class vBrainThreaded(vBrain.vBrain):
  """ Virtual Brain Threaded Virtual Class. """

  #--
  def vBrainInit(self):
    """ One-time vBrain initialization during object instantiation. """
    vBrain.vBrain.vBrainInit(self);

    # default execution cycle and step sizes
    self.SetExecSizes(0.1, 1.0)

    # thread initialization
    self.CogThreadInit()

  #--
  def __del__(self):
    """ Delete vBrain instance. """
    
    # Note: Destructor does not always work after the Brain thread has
    # been created. Some weakness in python and/or the threading library.
    # Some of the class data gets wacked before chance to kill the thread.
    # Hangs for ever.
    #if self.mCogThread and self.mCogThread.isAlive():
    #  self.CogThreadKill()
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
    self.mCogExecCycle  = execCycle
    self.mCogStepSize   = stepSize


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded Default Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the brain.

        Callback hook for vBrainThreaded emergency stopping event.

        Requires override in derived class.
    """
    raise NotImplementedError("%s.DoEStop" % (self.__class__.__name__))

  #--
  def DoLoadInit(self):
    """ Load vBrain Initializer. Any auto-connect operation should be 
        done in the overloaded derived class equivalent.

        Callback hook for vBrainThreaded loading event.
    """
    self.IniInit()
    self.PeerRobotInit()

  #--
  def DoStartInit(self):
    """ Start vBrain Initializer.

        Callback hook for vBrainThreaded starting or first-time
        stepping event.
    """
    self.ThinkActInit()

  #--
  def DoHold(self):
    """ Hold vBrain at current position.

        Callback hook for vBrainThreaded suspending event.
    """
    self.ThinkActHold()

  #--
  def DoRelease(self):
    """ Release vBrain continue operations.

        Callback hook for vBrainThreaded resuming event.
    """
    self.ThinkActRelease()

  #--
  def DoStopDeinit(self):
    """ Stop vBrain De-initializer.

        Callback hook for vBrainThreaded stopping event.
    """
    self.ThinkActDeinit()

  #--
  def DoUnloadDeinit(self):
    """ Unload vBrain De-initializer.

        Callback hook for vBrainThreaded unloading event.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded Default Think-Act Member Functions.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ThinkActInit(self):
    """ Initialize Think-Act Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    self.BrumInit()
    self.mOnHold = False

  #--
  def ThinkActHold(self):
    """ Hold Think-Act operations.

        Event: Suspeneding event

        Execution Context: Calling thread
    """
    self.mOnHold = True

  #--
  def ThinkActRelease(self):
    """ Release Think-Act operations.

        Event: Resuming event

        Execution Context: Calling thread
    """
    self.mOnHold = False

  #--
  def ThinkActDeinit(self):
    """ De-initialize Think-Act Operations.

        Event: Stopping event

        Execution Context: Calling thread
    """
    pass

  #--
  def Think(self):
    """ Think-Act Hierarchy.

        Execution Context: Cog thread
    """
    if self.mOnHold:
      return
    if __debug__: self.mDbg.d4print('ThinkAct')


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded Play Executives
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
    self.SetBrainState(Gluon.EServerState.Errored)
    self.DoEStop()
    self.CogThreadStop()
    self.GSReportErrorStatus('Emergency Stop')

  #--
  def ExecLoad(self):
    """ Load the vBrain.
    
        (Re)Initialize the vBrain configuration. The Cog thread is created.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecLoad')
    if not self.GSIsValidExecState(Gluon.EServerExec.Load):
      return
    self.DoLoadInit()
    self.CogThreadCreate()
    self.SetBrainState(Gluon.EServerState.Ready)

  #--
  def ExecStart(self):
    """ Start the vBrain thinking and acting.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecStart')
    if not self.GSIsValidExecState(Gluon.EServerExec.Start):
      return
    self.DoStartInit()
    self.CogThreadRun()

  #--
  def ExecStep(self):
    """ Step vBrain one execution cycle.
        
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
    self.CogThreadStep()

  #--
  def ExecResume(self):
    """ Resume vBrain controller functions with current brum data.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecResume')
    if not self.GSIsValidExecState(Gluon.EServerExec.Resume):
      return
    self.DoRelease()
    self.CogThreadRun()

  #--
  def ExecSuspend(self):
    """ Suspend vBrain activity. All brain functions will be put on hold. 
        The current data are kept.
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecSuspend')
    if not self.GSIsValidExecState(Gluon.EServerExec.Suspend):
      return
    self.CogThreadPause()

  #--
  def ExecStop(self):
    """ Stop the current vBrain activities. All thinking and acting are
        stopped. The current set of brum data are lost. The Cog thread is
        restarted with re-initialized data.
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecStop')
    if not self.GSIsValidExecState(Gluon.EServerExec.Stop):
      return
    self.CogThreadStop()
    self.DoStopDeinit()

  #--
  def ExecUnload(self):
    """ Exit vBrain instance, killing the Cog thread and performing any
        other resource cleanup.

        Return Value:
          None
    """
    if __debug__: self.mDbg.d2print('ExecUnload')
    if not self.GSIsValidExecState(Gluon.EServerExec.Unload):
      return
    self.CogThreadKill()
    self.DoUnloadDeinit()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # VBrainThreaded Thread Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CogThreadInit(self):
    """ One time initialization of the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadInit')
    self.mCogSema       = thread.Semaphore()
    self.mCogExecEvent  = thread.Event()
    self.mCogCanLive    = False
    self.mCogGo         = False
    self.mCogStep       = False
    self.mCogStepTime   = 0.0
    self.mCogThread     = None
    self.mTWall         = 0.0
    self.mTExecStart    = 0.0

  #--
  def CogThreadCreate(self):
    """ Create the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadCreate')
    self.mCogSema.release()     # make sure semaphore is acquireable
    self.mCogSema.acquire()
    self.mCogCanLive  = True
    self.mCogGo       = False
    self.mCogStep     = False
    self.mCogThread   = thread.Thread(target=self.CogThread, 
        name='Cog'+self.HasName(), kwargs={})
    self.mCogThread.start()
    self.SetBrainState(Gluon.EServerState.NotReady)

  #--
  def CogThreadRun(self):
    """ Run the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadRun')
    self.mCogGo   = True
    self.mCogStep = False
    self.SetBrainState(Gluon.EServerState.Running)
    self.mCogSema.release()

  #--
  def CogThreadPause(self):
    """ Pause (suspend) the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadPause')
    self.mCogGo   = False
    self.mCogStep = False
    self.SetBrainState(Gluon.EServerState.Paused)

  #--
  def CogThreadStep(self):
    """ Step the Cog thread one execution cycle.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadStep')
    self.mCogGo       = True
    self.mCogStep     = True
    self.mCogStepTime = self.mCogStepSize
    self.SetBrainState(Gluon.EServerState.Stepping)
    self.mCogSema.release()

  #--
  def CogThreadStop(self):
    """ Stop the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadStop')
    self.mCogGo   = False
    self.mCogStep = False
    self.SetBrainState(Gluon.EServerState.Ready)

  #--
  def CogThreadKill(self):
    """ Kill the Cog thread.

        Execution Context: Calling thread
    """
    if __debug__: self.mDbg.d3print('CogThreadKill')
    self.mCogGo      = False
    self.mCogStep    = False
    self.mCogCanLive = False
    self.mCogSema.release()
    self.mCogExecEvent.set()
    if self.mCogThread.isAlive():
      self.mCogThread.join(2.0)
    if self.mCogThread.isAlive():
      if __debug__: self.mDbg.d3print('CogThread failed to die')
    self.SetBrainState(Gluon.EServerState.NotLoaded)

  #--
  def CogThreadCleanup(self):
    """ Clean-up any Cog thread resources.

        Execution Context: Cog thread
    """
    self.mCogCanLive  = False
    self.mCogGo       = False
    self.mCogStep     = False

  #--
  def CogThread(self, args=(), kwargs={}):
    """ The Cog thread. 
    
        The Cog thread provides the execution sequencing for the
        vBrain thinking and high-level acting activities.
        The Cog thread executes at the opted time step.
    """
    if __debug__: self.mDbg.d3print('CogThread: alive')

    t_wait  = 0.0   # time to wait between execution cycles

    while self.mCogCanLive:
      # Run/Block
      self.mTWall = time.time()   # wall clock for outer loop
      self.mCogSema.acquire()   # block forever until Run/Kill
      if not self.mCogCanLive:  # exit thread by falling through
        break

      # Execution cycle start time
      self.mTExecStart = time.time()
      if __debug__: self.mDbg.d4print('Exec start: %f' % self.mTExecStart)

      # Wait for next cycle to begin
      if self.mCogGo:           # running
        self.mCogExecEvent.wait(t_wait)

      # Execution Cycle
      if self.mCogGo:           # running
        self.ThinkAct()         #  think-act hierarchical

      # Delta and Adjusted times
      dt = time.time() - self.mTExecStart # includes last t_wait
      t_wait = self.mCogExecCycle + t_wait - dt
      if t_wait < 0.0:
        t_wait = 0.0
      # RDK!!! perfhook here

      # Determine what to do after execution cycle
      if self.mCogStep:         # stepping one cycle
        self.mCogStepTime -= dt
        if self.mCogStepTime <= 0.0:
          self.CogThreadPause() #  autopause
      if self.mCogGo:           # running
        self.mCogSema.release() #  release to run new cycle
      else:                     # stop running
        self.DoHold()           #  put a hold on the vBrain
        t_wait = 0.0            #  run next cycle immediately after resume

    # Thread is dying
    self.CogThreadCleanup()
    if __debug__: self.mDbg.d3print('CogThread: dead')


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import sys

  vb = vBrainThreaded(debuglevel=3)
  doquit = False

  def quit():
    global doquit
    vb.CogThreadKill()
    doquit = True

  def tsthelp():
    for k in execs.iterkeys():
      print k

  execs = {
    'quit':     quit,
    'help':     tsthelp,
    'estop':    vb.ExecEStop,
    'load':     vb.ExecLoad,
    'start':    vb.ExecStart,
    'suspend':  vb.ExecSuspend,
    'resume':   vb.ExecResume,
    'step':     vb.ExecStep,
    'stop':     vb.ExecStop,
    'unload':   vb.ExecUnload,
  }

  def main():
    """ vBrainThreaded Test Main """
    print "Enter executive ('help' for list of commands)"
    while not doquit:
      rsp = raw_input('exec> ')
      for k, op in execs.iteritems():
        if rsp == k:
          op()
          break

  # run test
  main()
