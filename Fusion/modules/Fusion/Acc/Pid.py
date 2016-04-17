################################################################################
#
# Pid.py
#

""" A Proportional-Integral-Derivative controller module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.11

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

import math
import Fusion.Utils.IVTimer as IVTimer

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: Pid
#-------------------------------------------------------------------------------
class Pid:
  """ PID Controller Class.

      This PID applies a PID to a control variable that applies input to
      a process whose output goal is a specified setpoint.

      PID Model:

                -----------
      --- cv -->| process |--> pv ----@
          ^     -----------           |
          |                           |
          |                           |
          @------[PID]<---------------@
                   ^
                   |
                setpoint

     The PID can be used in two ways:
        Automatically:
          pid = Pid(...)
          pid.start()
          ...
          pid.cancel()

        Manually:
          pid = Pid(...)
          pid.apply(dt, setpoint, pv)
  """

  #--
  def __init__(self, CbInput, CbOutput, cv_init, dt, Kp, Ki, Kd, Wp=1.0, Wd=1.0,
                     cv_min=None, cv_max=None, action='direct', trace=False):
    """ Initialize Pid controller.

        Parameters:
          CbInput   - PID input callback function returns (setpoint, pv)
          CbOutput  - PID output callback function that applies new control
                      variable to process input.
          cv_init   - Initial control variable value.
          dt        - Delta time (seconds) between PID input readings 
                      (process output). Used in auto mode.
          Kp        - PID proportional constant. Set to 0.0 if no P component.
          Ki        - PID integral constant. Set to 0.0 if no I component.
          Kd        - PID differential constant. Set to 0.0 if no D component.
          Wp        - Weight of proportional error.
          Wd        - Weight of differential error.
          cv_min    - Minimum control variable value. None == no minimum.
          cv_max    - Maximum control variable value. None == no maximum.
          action    - Control is 'direct' (positive) or 'reverse' (negative).
          auto      - Do [not] create an interval timer every dt seconds
                      to automatically read input, and generated to PID output.
          trace     - Do [not[ trace PID values to stdout.
    """
    self.CbInput  = CbInput  # PID input (process output)
    self.CbOutput = CbOutput # PID output (process input)

    self.cv     = cv_init   # control variable initial value
    self.dt     = dt        # dt (seconds)
    self.Kp     = Kp        # proportional constant
    self.Ki     = Ki        # integral constant
    self.Kd     = Kd        # differential constant
    self.Wp     = Wp        # proportional error weight
    self.Wd     = Wd        # differential error weight
    self.cv_min = cv_min    # minimum control variable value
    self.cv_max = cv_max    # maximum control variable value
    self.Action = action    # PID direct/reverse action

    self.trace  = trace     # do [not] trace

    self.setpoint = 0.0     # current process setpoint
    self.pv       = 0.0     # current process variable output value

    self.e_last   = 0.0     # last error
    self.t_accum  = 0.0     # accumulative time

    self.P        = 0.0     # proportional
    self.I        = 0.0     # integral
    self.D        = 0.0     # derivative

    self._LoopTimer = None  # interval timer

  #--
  def __del__(self):
    """ Delete self """
    self.cancel()

  #--
  def start(self):
    """ Start control loop interval timer """
    self._LoopTimer = IVTimer.IVTimer(self.dt, self.dt, self._CtlLoop)
    self._LoopTimer.start()

  #--
  def cancel(self):
    """ Cancel control loop. """
    if not self._LoopTimer:
      return
    self._LoopTimer.cancel()
    del self._LoopTimer
    self._LoopTimer = None

  #--
  def apply(self, dt, setpoint, pv):
    """ Apply PID to the control variable given the PID input.

        Parameters:
          dt          - Delta time in seconds.
          setpoint    - Current process setpoint.
          pv          - Current process variable output value.

        Return Value:
          New control variable value.
    """
    self._calc(dt, setpoint, pv)
    return self.cv

  #--
  def _CtlLoop(self, ivt):
    """ Interval timer callback. """
    self.setpoint, self.pv = self.CbInput()
    self._calc(self.dt, self.setpoint, self.pv)
    self.CbOutput(self.cv)
    
  #--
  def _calc(self, dt, setpoint, pv):
    """ Calculate new control variable value.

        Parameters:
          dt          - Delta time in seconds.
          setpoint    - Current process setpoint.
          pv          - Current process variable output value.

        Return Value:
          New control variable value.
    """

    if self.trace:
      print '%6.2fs: setpoint=%.3f, pv=%.3f' % (self.t_accum, setpoint, pv)

    if self.Action == 'reverse':
      e_p = self.Wp * setpoint - pv
      e_i = setpoint - pv
      e_d = self.Wd * setpoint - pv
    else:
      e_p = pv - self.Wp * setpoint
      e_i = pv - setpoint
      e_d = pv - self.Wd * setpoint

    self.P = self.Kp * e_p
    self.I = self.I + self.Ki * e_i * dt
    self.D = self.Kd * (e_d - self.e_last) / dt

    u = self.P + self.I + self.D

    self.e_last   = e_d
    self.t_accum += dt

    self.cv  -= u

    if self.cv_min is not None and self.cv < self.cv_min:
      self.cv = self.cv_min
    elif self.cv_max is not None and self.cv > self.cv_max:
      self.cv = self.cv_max

    if self.trace:
      print '        err=%.3f, pid=%.3f, output=%.3f' % \
          (e_i, u, self.cv)

    return self.cv


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  dt = 0.0
  sp = 0.0
  pv = 0.0
  kp = 0.0
  ki = 0.0
  kd = 0.0
  wp = 0.0
  wd = 0.0
  t_accum = 0.0

  def CbSinPlant(theta):
    global pv
    pv = 100.0 * math.sin(0.1 * theta)

  def CbSinInput():
    return 60.0, pv

  def CbMsdPlant(u):
    """ Mass-spring-damper """
    global pv, kp, dt, t_accum
    t_accum += dt
    s = t_accum
    x_s =  u / (s *s + 10.0 * s + (20.0 + u))
    pv = 1.0/x_s

  def CbMsdInput():
    global pv
    return 1.0, pv

  def main():
    """ Pid Test Main """
    global kp, ki, kd
    #pid = Pid(CbSinInput, CbSinPlant, 0.0, 1.0,
    #            0.10, 0.0, 0,
    #            cv_min=-math.pi/0.2, cv_max=math.pi/0.2,
    #            action='direct', trace=True)
    dt = 0.01
    kp = 300.0
    pid = Pid(CbMsdInput, CbMsdPlant, 0.0, dt,
                kp, 0.0, 0.0,
                action='direct', trace=True)
    print "Press <enter> to terminate..."
    pid.start()
    raw_input()
    pid.cancel()

  # run test
  main()
