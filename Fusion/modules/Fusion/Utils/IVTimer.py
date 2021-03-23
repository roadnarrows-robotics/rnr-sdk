################################################################################
#
# IVTimer.py
#

""" Interval Timer Class

A interval timer class. The threading.Timer() class just didn't cut it.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.09

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

import threading as thread
import time

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: IVTimer
#-------------------------------------------------------------------------------
class IVTimer:
  """ Interval Timer Class """

  #--
  def __init__(self, start, iv, callback, *uargs, **ukwargs):
    """ Initialize IVTimer

        Parameters:
          start     - Make first callback after start seconds. If 0.0, then
                      first callback will be immediate.
          iv        - Make periodic callback at the interval of iv seconds,
                      starting from start time.
          callback  - Callback function (just don't tarry too long).
                      Signature: callback(self) where self is this IVTimer.
          *uargs    - Optional user arguments. Stored in public list args[].
          **ukwargs - Optional user keyword arguments. Keyword=value set as
                      public attributes of same name and value.
    """
    self._tstart    = start           # start time
    self._tiv       = iv              # interval time
    self._telapse   = self._tstart    # total elapse time
    self._cb        = callback        # IVTimer callback
    self._ev        = thread.Event()  # IVTimer timeout event
    self._quit      = False           # IVTimer not canceled

    # set user argument attributes
    self.args = []
    for v in uargs:
      self.args.append(v)

    # set user keyword argument attributes
    self._ukwargs = ukwargs
    for n,v in ukwargs.iteritems():
      setattr(self, n, v)

    # create IVTimer thread
    self._thiv = thread.Thread(target=self._ivthread, name='IVTimer', kwargs={})

  #--
  def __del__(self):
    """ Delete self """
    if self._thiv.isAlive():
      self.cancel()

  #--
  def start(self):
    """ Start interval timer. """
    self._thiv.start()

  #--
  def cancel(self):
    """ Cancel interval timer synchronously, killing the underlining thread.
        Note: Cannot be called from IVTimer callback.
    """
    self._quit = True
    self._ev.set()
    self._thiv.join()

  #--
  def asynccancel(self):
    """ Cancel interval timer asynchronously, killing the underlining thread.
        Note: IVTimer callback safe.
    """
    self._quit = True

  #--
  def getivtime(self):
    """ Return interval time in seconds. """
    return self._tiv

  #--
  def getelapsetime(self):
    """ Return total elapse time in seconds from start of IVTimer. """
    return self._telapse

  #--
  def getuserdata(self):
    """ Return saved user data. """
    return tuple(self.args), self._ukwargs

  #--
  def _ivthread(self, args=(), kwargs=()):
    """ The interval timer thread. """
    self._ev.wait(self._tstart)

    self._telapse = self._tstart

    if not self._quit:
      t = time.time()
      self._cb(self)
      dt = time.time() - t
      self._telapse += dt

      t_wait = self._tiv - dt
      if t_wait < 0.0:
        t_wait = 0.0
    
    while not self._quit:

      self._ev.wait(t_wait)

      if not self._quit:
        t = time.time()
        self._cb(self)
        dt = time.time() - t
        self._telapse += dt

        t_wait = self._tiv - dt
        if t_wait < 0.0:
          t_wait = 0.0


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def cb(ivt):
    print('callback made:', time.time())
    if ivt.args[0] == 0:
      print('callback args:', ivt.args[0], ivt.args[1], ivt.apple, ivt.grape)
      ivt.args[0] = 1
    elif ivt.args[0] == 1:
      args, kwargs = ivt.getuserdata()
      print(args, kwargs)
      ivt.args[0] = 2
    time.sleep(0.40)

  def main():
    """ IVTimer Unit Test Main """
    tstart = 1.0
    tiv = 2.0
    print('\n  press any key to cancel\n')
    iv = IVTimer(tstart, tiv, cb, 0, 6, apple='granny', grape='concorde')
    print('starting IVTimer(%3.1f, %3.1f)' % (tstart, tiv))
    print('starting time:', time.time())
    iv.start()
    key = raw_input()
    iv.cancel()
    print('listing threads')
    print(thread.enumerate())

  # run unit test
  main()
