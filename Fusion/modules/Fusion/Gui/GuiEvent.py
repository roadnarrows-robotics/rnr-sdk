################################################################################
#
# GuiEvent.py
#

""" Graphical User Interface Safe Event Module.

    Caveat Emptor

Because of inept design of Tkinter, there doesn't seem to be any clear
way for clean context control mechanisms to 'lock' the Tkinter mainloop()
while allowing background event processing. If the mainloop() truly blocks,
deadlocks can easily ensue.

Note: I have not found a reliable way to synchronize. The wait_variable()
      approach works on Linux but poorly on Windows. Looking through
      online discussions, wait_variable() is unreliable.

Note: This module is not used but is kept as a reminder of what not to do.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.10.09

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

import Tkinter as tk
import mutex
import Fusion.Utils.IVTimer as IVTimer
import time

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiEvent
#-------------------------------------------------------------------------------
class GuiEvent:
  """ GUI Event Class.
  
      This is a safe event class for use within Tkinter mainloop()
      context calls. The mainloop() cannot truly block for any duration,
      otherwise Tkinter calls made in other thread contexts will block,
      possibly leading to deadlock. Safe blocking is defined here as 
      giving the mainloop() context Tkinter event processing capabilities
      while being blocked waiting for an unblocking synchronizing event.

      A GuiEvent object manages an internal flag that can be set to True 
      with the set() method and reset to False with the clear() method.
      The wait() method, which is mainloop() safe, blocks until the flag is
      True.

      The internal flag is initially False.
  """

  #--
  def __init__(self, master):
    """ Initialize the Gui Event object. 

        Parameters:
          master    - Tkinter root or widget object
    """
    self._master  = master
    self._mutex = mutex.mutex()               # equivalent: the flag
    self._mutex.lock(self._cbdummy, 'init')   # equivalent: flag = False

    # wait() event synchronization
    self._varWait   = tk.IntVar(self._master)   # wait variable
    self._isWaiting = True                      # wait state

  #--
  def isSet(self):
    """ Return True if and only if the internal flag is True.
        Otherwise return False.
    """
    return self._mutext.test() == 0   # 0 == unlocked, 1 == locked

  #--
  def set(self):
    """ Set the internal flag to True. All mainloop()'s waiting for it to
        become True are released. Mainloops that call wait() once the flag
        is true will not block at all.
    """
    #print 'Dbg: GuiEvent.set()'
    self._mutex.unlock() # equivalent: flag = True

  #--
  def clear(self):
    """ Reset the internal flag to False. Subsequently, mainloop()'s
        calling wait() will block until set() is called to set the
        internal flag to True again.
    """
    #print 'Dbg: GuiEvent.clear()'
    self._mutex.lock(self._cbdummy, 'clear')  # equivalent: flag = False

  #--
  def _cbdummy(self, why):
    pass

  #--
  def wait(self, timeout=0.0):
    """ Block until the internal flag is True. If the internal flag is True
        on entry, return immediately. Otherwise, block until another thread
        calls set() to set the flag to true, or until the optional timeout
        occurs.

        Parameters:
          timeout - Timeout value in seconds.  When the timeout is set,
                    it should be a floating point number specifying a
                    timeout for the wait() operation in seconds (or 
                    fractions thereof). A timeout value of None or 0.0
                    will block forever until set() is called.
    """
    # The wait timeout minimum granularity is 100msec. If the value is too
    # small, then adjust.
    if timeout and timeout < 0.1:
      timeout = 0.1

    #
    # Loop waiting for 'flag' == True or for any timeout to occur
    #
    self._isWaiting = True
    self._varWait.set(0)
    ivt = IVTimer.IVTimer(0.1, 0.1, self._feed, varstate=0)
    ivt.start()
    while self._varWait.get() == 0: 
      #print 'Dbg: GuiEvent.wait(): waiting'
      self._master.wait_variable(self._varWait) # wait for variable set()
      if timeout:               # timeout specified
        timeout -= 0.1          # decrement the remaining time
        if timeout <= 0.0:      # timeout has expired
          self.clear()          # get back to False, eventually, hopefully
          break
    #print 'Dbg: GuiEvent.wait(): finished'
    self._isWaiting = False
    ivt.cancel()
    # leave mutex locked. equivalent: 'flag' == False

  #--
  def _feed(self, ivt):
    """ Feed the wait()'s wait_variable() """
    #print 'Dbg: GuiEvent._feed(): ivt callback, varstate=%d' % ivt.varstate
    if ivt.varstate == 1:           # already got the lock, stay at 1
      #print 'Dbg: GuiEvent._feed(): still locked'
      pass
    elif self._mutex.testandset():  # lock mutex if posible (atomic)
      #print 'Dbg: GuiEvent._feed(): acquired the lock'
      ivt.varstate = 1
    # feed the variable
    if self._isWaiting:
      #print 'Dbg: GuiEvent._feed(): set(%d)' % ivt.varstate
      try:
        self._varWait.set(ivt.varstate)
      except tk.TclError:
        pass
