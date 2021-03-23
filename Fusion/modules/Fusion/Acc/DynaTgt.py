################################################################################
#
# DynaTgt.py
#

""" Attractor Dynamics Target Visulaizer 

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.02.03

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
import threading as thread
import Tkinter as tk

import Fusion.Utils.IVTimer as IVTimer

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiXYGraph as GuiXYGraph

import Fusion.Khepera.Gui.GuiDlgDynaOpt as GuiDlgDynaOpt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# some math constants
pi          = math.pi
halfpi      = math.pi / 2.0
threehalfpi = math.pi * 3.0 / 2.0
twopi       = math.pi * 2.0
e           = math.e
sqrte       = math.sqrt(math.e)

#
# Dynamics Design Parameter Defaults
#
DynaDesignParamDfts = {
    # Field Parameters
    'dpsi':       8.0,      # delta psi (sample size)
    'k_p':        3.0,      # Schoner: 2.0
    'k_n':        1.5,      # Schoner: 3.5
    'l_coop':     math.radians(40.0), # interaction kernel cooperativity length
    'k_tau':      5.0,      # delta time multiplier
    #'W_m':        0.7,     # h_min,max implicitely code this value
    'h_min':      -0.77,    # target activation monostable regime
    'h_max':      -0.0875,  # target activation bistable regime
    'k_r_h_min':  5.0,      # target activation destabilization rate
    'k_r_h_max':  50.0,     # target activation stabilization rate

    # Target Acquisition
    'TgtSensorAngRange':  math.radians(120.0),  # simulated sensor angular range
    'M0':             0.0,  # zero point target bias. Schoner: 0.25
    'sigma':          0.4,  # Gausian convolver sigma
    'k_lambda_tar_p': 5.0,  # target attractive force-let

    # Obstacle Avoidance
    'k_beta_1': 5.0,        # maximal repulsive force-let strength
    'beta_2':   20.0,       # decay distance repulsive force-let strength

    # Velocity Control
    'c':            100.0,  # velocity control sigmoid steepness
    'k_c_v_tar':    15.0,   # target velocity relaxation rate
    'k_c_v_obs':    10.0,   # obstacle velocity relaxation rate
    'psi_hat_max':  35.0,   # maximum robot speed

    # Execution
    'ExecCycle':    0.1,    # execution cycle time
    'ExecStepSize': 1.0,    # step size
}


#-------------------------------------------------------------------------------
# CLASS: DynaTgt
#-------------------------------------------------------------------------------
class DynaTgt:
  """ Attractor Dynamics Target Visulaizer. """

  #--
  def __init__(self):
    """ Initialize. """
    #
    # Design Parameters
    #
    self.mOpt = DynaDesignParamDfts

    #
    # Robot Data
    #
    self.mRobotLoc = {}
    self.mRobotLoc['theta'] = pi
    self.mRobotLoc['pathspeed'] = 32.0

    # Obstacle Sensors
    self.mObsSensors = {}
    for zeta in [-90.0, -45.0, 0.0, 45.0, 90.0]:
      if zeta < 0.0:
        label = 'right'
      elif zeta == 0.0:
        label = ''
      else:
        label = 'left'
      id = 'obs_front%s%d' % (label, abs(int(zeta)))
      self.mObsSensors[id] = {}
      self.mObsSensors[id]['zeta'] = math.radians(zeta)
      self.mObsSensors[id]['angrange'] = math.radians(30.0)

    #
    # Target Sensors
    #
    self.mTgtSensors = {}
    for ang in [90.0, 45.0, 0.0, -45.0, -90.0]:
      if ang < 0.0:
        label = 'right'
      elif ang == 0.0:
        label = ''
      else:
        label = 'left'
      id = 'sim_front%s%d' % (label, abs(int(ang)))
      self.mTgtSensors[id] = {}
      self.mTgtSensors[id]['zeta'] = math.radians(ang)
      self.mTgtSensors[id]['angrange'] = self.mOpt['TgtSensorAngRange']

    #
    # Other data
    #
    self.mSGraphTitle = 'Target Source Behavior ~ S(psi)'
    self.mUGraphTitle = 'Target Activation Behavior ~ u(psi)'
    self.mTestNum = 0
    self.mTests = [
      {'name':'No Targets',
       'init':self.BrumNull,
       'read':self.SensorsReadNoTgt
      },
      {'name':'Steady State One Target',
       'init':self.BrumNull,
       'read':self.SensorsReadSteady
      },
      {'name':'One Target; See Figure 12',
       'init':self.BrumNull,
       'read':self.SensorsReadOneTgt
      },
      {'name':'Moving Target; See Figure 15',
       'init':self.BrumNull,
       'read':self.SensorsReadMovingTgt
      },
      {'name':'Two Targets; See Figure 16',
       'init':self.BrumNull,
       'read':self.SensorsReadTwoTgt
      },
      {'name':'Memory of One Target; See Figure 17',
       'init':self.BrumNull,
       'read':self.SensorsReadMemOneTgt
      },
      {'name':'Memory, then New Target',
       'init':self.BrumNull,
       'read':self.SensorsReadMemNewTgt
      },
    ]
    self.mDt    = 1.0
    self.mT     = -self.mDt
    self.mStep  = -1
    self.mSema  = thread.Semaphore()
    self.mCmdLock = False
    self.mDoCancel = False
    self._cmdlock()
    self.mLoopTimer = IVTimer.IVTimer(0.0, self.mDt, self.CtlLoop)
    self.mLoopTimer.start()

    #
    # (Cere)Brum
    #
    self.BrumNull()

  #--
  def ReadObsSensors(self):
    dist_min = 1000.0
    for sensor in self.mObsSensors.itervalues():
      sensor['dist'] = 5.0
      if sensor['dist'] < dist_min:
        dist_min = sensor['dist']
    self.mBrum['obs_dist_min'] = dist_min

  #--
  def SensorsReadNoTgt(self):
    for id in self.mTgtSensors:
      self.mTgtSensors[id]['Mi'] = 0.0

  #--
  def SensorsReadSteady(self):
    self.SensorsSim([(pi, 2.5)])

  #--
  def SensorsReadOneTgt(self):
    m = self.mStep * 0.25
    if m > 2.5:
      m = 2.5
    self.SensorsSim([(pi, m)])

  #--
  def SensorsReadMovingTgt(self):
    rho = 90.0 + self.mStep * 10.0
    if rho > 270.0:
      rho = 270.0
    print('rho=', rho)
    rho = math.radians(rho)
    m   = 2.5
    self.SensorsSim([(rho, m)])

  #--
  def SensorsReadTwoTgt(self):
    m = self.mStep * 0.25
    n = self.mStep * 0.20
    if m > 2.5:
      m = 2.5
    if n > 2.0:
      n = 2.0
    self.SensorsSim([(math.radians(112.5), n), (math.radians(270.0), m)])

  #--
  def SensorsReadMemOneTgt(self):
    if self.mStep < 10:
      m = self.mStep * 0.25
    else:
      m = 0.0
    self.SensorsSim([(pi, m)])

  #--
  def SensorsReadMemNewTgt(self):
    if self.mStep < 10:
      m = self.mStep * 0.25
      rho = pi
    elif self.mStep < 15:
      m = 0.0
      rho = pi
    else:
      m = (self.mStep - 15) * 0.25
      rho = pi / 2.0
    if m > 2.5:
      m = 2.5
    self.SensorsSim([(rho, m)])

  #--
  def SensorsSim(self, rhoList):
    theta = self.mRobotLoc['theta']
    angrange = self.mOpt['TgtSensorAngRange']
    w = twopi / (2.0 * angrange)    # 0 at ends of [-angrange/2, angrange/2]
    angrange /= 2.0                 # half angle
    rho, m = rhoList[0]
    #print('  rho=%f, m=%f' % (math.degrees(rho), m) )
    for id in self.mTgtSensors:
      zeta = self.mTgtSensors[id]['zeta']
      psi = math.fabs(rho - theta - zeta)
      if psi <= angrange:
        self.mTgtSensors[id]['Mi'] = m * math.cos(w*psi)
      else:
        self.mTgtSensors[id]['Mi'] = 0.0
      #print('   %s[Mi]=%f' % (id, self.mTgtSensors[id]['Mi']))

    for rho, m in rhoList[1:]:
      #print('  rho=%f, m=%f' % (math.degrees(rho), m) )
      for id in self.mTgtSensors:
        zeta = self.mTgtSensors[id]['zeta']
        psi = math.fabs(rho - theta - zeta)
        if psi <= angrange:
          self.mTgtSensors[id]['Mi'] += m * math.cos(w*psi)
        #print('   %s[Mi]=%f' % (id, self.mTgtSensors[id]['Mi']))

  #--
  def BrumNull(self):
    #
    # (Cere)Brum
    #
    self.mBrum = {}

    # user option Brum dependencies
    self.BrumOpts()

    # Robot
    self.mBrum['R_robot'] = 52.0 / 2.0  # Khepera size

    # Obstacle data
    for id in self.mObsSensors.iterkeys():
      self.mObsSensors[id]['tan(angrange)'] = \
          math.tan(self.mObsSensors[id]['angrange']/2.0)

    # Target data
    self.mBrum['h'] = self.mOpt['h_max']
    self.mBrum['S'] = []
    self.mBrum['u'] = []
    for ang in self.mAngRad:
      self.mBrum['S'].append(0.0)
      self.mBrum['u'].append(self.mBrum['h'])

  #--
  def BrumOpts(self):
    """ 'Fixed' Brum values based on user options. """
    self.mAngDeg = range(0, 361, int(self.mOpt['dpsi']))
    if self.mAngDeg[-1] != 360:
      self.mAngDeg.append(360)

    self.mAngRad = []
    for ang in self.mAngDeg:
      self.mAngRad += [math.radians(ang)]

    self.mBrum['dpsi']        = math.radians(int(self.mOpt['dpsi']))
    self.mBrum['2sq(sigma)']  = 2.0 * self.mOpt['sigma']**2
    self.mBrum['l_coop']      = self.mOpt['l_coop'] / 2.0

  #--
  def CtlLoop(self, ivt):
    self.mSema.acquire()

    if self.mDoCancel:
      self.mSema.release()
      return

    self.mT     += self.mDt
    self.mStep  += 1

    self.mShowTimeStep(self.mStep)

    print()
    print('TimeStep', self.mStep)

    # read obstacle sensors
    self.ReadObsSensors()

    # read target sensors
    self.mTests[self.mTestNum]['read']()

    # per execution cycle up fronts
    self.mBrum['dt']            = 0.1 #self.mDt
    tau = self.mBrum['tau']     = self.mOpt['k_tau'] * self.mBrum['dt']
    self.mBrum['r_h_min']       = 1.0/(self.mOpt['k_r_h_min'] * tau)
    self.mBrum['r_h_max']       = 1.0/(self.mOpt['k_r_h_max'] * tau)
    self.mBrum['lambda_tar_p']  = 1.0/ (self.mOpt['k_lambda_tar_p'] * tau)
    self.mBrum['beta_1']        = self.mOpt['k_beta_1'] * \
                                          self.mBrum['lambda_tar_p']
    self.mBrum['c_v_tar']       = self.mOpt['k_c_v_tar'] * \
                                          self.mBrum['lambda_tar_p']
    self.mBrum['c_v_obs']       = self.mOpt['k_c_v_obs'] * \
                                          self.mBrum['lambda_tar_p']

    self.Sum_f_obs_i()  # obstacle repulsive force-let sum
    self.S_field()      # generate target source field
    self.u_field()      # generate activation field
    self.f_Ns()         # total positive activation
    self.f_Nu()         # total input source activation
    self.f_tar()        # target attractive force-let
    self.mBrum['h'] += self.f_dhdt()  # new activation bias

    dtheta = self.mBrum['dtheta_obs'] + self.mBrum['dtheta_tar']
    dv     = self.f_dvdt()

    print('dtheta_obs=%.3f, dtheta_tar=%.3f' % \
        (math.degrees(self.mBrum['dtheta_obs']), 
         math.degrees(self.mBrum['dtheta_tar'])))
    print('dv=%.3f, dtheta=%.3f' % (dv, math.degrees(dtheta)))
    #print('Ns=%f, Nu=%f, h=%f' % \
    #    (self.mBrum['Ns'], self.mBrum['Nu'], self.mBrum['h']))

    self.mSGrapher.graph(title=self.mSGraphTitle,
        xlabel='psi', ylabel='S', xstep=60, ystep=1,
        xdata=self.mAngDeg, ydata=self.mBrum['S'])

    self.mUGrapher.graph(title=self.mUGraphTitle,
        xlabel='psi', ylabel='u', xstep=60, ystep=1,
        xdata=self.mAngDeg, ydata=self.mBrum['u'])

    self.mSema.release()

  #--
  def lambda_i(self, sensorId):
    """ Obstacle repulsive strength function.

        lambda_i = beta_1 * exp(-d_i / beta_2)
          where
            beta_1  - tunable parameter determines the maximal strength
            beta_2  - tuanable parameter fixes decay constant
            d_i     - distance to obstacle measured from sensor i

        Parameters:
          sensorId  - id of obstacle detecting sensor

        Return Value:
          Scalar repulsive strength of repulsive force-let i
    """
    sensor = self.mObsSensors[sensorId]
    sensor['lambda_i'] = self.mBrum['beta_1'] * \
        math.exp(-sensor['dist']/self.mOpt['beta_2'])
    return sensor['lambda_i']

  #--
  def sigma_i(self, sensorId):
    """ Obstacle angular range of sensor contribution function.

        sigma_i = arctan[tan(angrange/2) + R_robot/(R_robot + d_i)]
          where
            angrange  - angular range of the sensor
            R_robot   - radius of the robot's physical size
            d_i       - distance to obstacle measured from sensor i

        Parameters:
          sensorId  - id of obstacle detecting sensor

        Return Value:
          Angular contribution of repulsive force-let i
    """
    sensor = self.mObsSensors[sensorId]
    sensor['sigma_i'] = math.atan(sensor['tan(angrange)'] + \
        self.mBrum['R_robot'] / (self.mBrum['R_robot'] + sensor['dist']))
    return sensor['sigma_i']

  #--
  def f_obs_i(self, sensorId):
    """ Repulsive obstacle force-let function from sensor i.

        f_obs_i = lambda_i * zeta * exp[-(zeta^2) / 2 * sigma_i ^ 2]
          where
            lambda_i  - repulsive strength function
            zeta      - angular location of sensor i
            sigma_i   - repulsive angular contribution function

        Parameters:
          sensorId  - id of obstacle detecting sensor

        Return Value:
          Repulsive force-let i
    """
    lam = self.lambda_i(sensorId)
    sig = self.sigma_i(sensorId)
    sensor = self.mObsSensors[sensorId]
    sensor['f_obs_i'] = lam * -sensor['zeta'] * \
                                  math.exp(-sensor['zeta']**2/ (2 * sig**2))
    #if __debug__: 
    #  self.mDbg.d4print('f_obs_i[%s]=%.3f' % (sensorId, sensor['f_obs_i']))
    return sensor['f_obs_i']

  #--
  def Sum_f_obs_i(self):
    """ Sum over all of the repulsive obstacle force-lets detected by
        the obstacle detecting sensors.

        f_obs_i = sum( f_obs_i )

        Return Value:
          Repulsive force-let on avoidance direction.
    """
    sum = 0.0
    for sensorId in self.mObsSensors.iterkeys():
      sum += self.f_obs_i(sensorId)
    self.mBrum['dtheta_obs'] = sum
    return sum

  #--
  def f_dvdt(self):
    """ Change in robot velocity w.r.t. time function.

        dv/dt = -c_obs * (v - V_obs) - c_tar * (v - V_tar)
          where
            c_obs - weight [0,1] of obstacle delta v 
            c_tar - weight [0,1] of target delta v 
            v     - current robot velocity (speed)
            V_obs - obstacle velocity as a function of distance
            V_tar - target velocity as a function of activation strength

        Return Value:
          dv/dt
    """
    v     = self.mRobotLoc['pathspeed']
    alpha = math.atan(self.mOpt['c'] * self.f_U()) / pi
    print('alpha', alpha)
    c_obs = self.mBrum['c_v_obs'] * (0.5 + alpha)
    c_tar = self.mBrum['c_v_tar'] * (0.5 - alpha)
    print('c_obs', c_obs, 'c_tar', c_tar)
    V_obs = self.mBrum['obs_dist_min'] * self.mOpt['psi_hat_max']
    V_tar = self.mBrum['u_max'] * self.mOpt['psi_hat_max']
    self.mBrum['dv/dt'] = -c_obs * (v - V_obs) - c_tar * (v - V_tar)
    return self.mBrum['dv/dt']

  #--
  def f_U(self):
    """ Obstacle potential function.

        U = Sum(lambda_i * sq(sigma_i) * exp(-sq(zeta)/2sq(sigma_i) 
              - lamda_i * sq(sigma_i) / sqrt(e)), for i=0,numsensors
          where
            lambda_i  - repulsive strength of sensor i
            zeta      - angular location of sensor i
            sigma_i   - repulsive angular contribution of sensor i

        Return Value:
          U
    """
    U = 0.0
    for sensorId,sensor in self.mObsSensors.iteritems():
      sqsigma   = sensor['sigma_i'] ** 2
      lamsqsig  = sensor['lambda_i'] * sqsigma
      U += lamsqsig * math.exp(-sensor['zeta']**2/(2*sqsigma)) - lamsqsig/sqrte
    self.mBrum['U'] = U
    print('U', U)
    return U

  #--
  def f_H_M(self):
    """ Heaviside of sum of all target sensor measure input strengths.

        H(M) =  Heaviside(Sum(Mi),i=0,numsensors)
          where
            Mi  - measure target strength scalar from sensor i

        Return Value:
          H(M)
    """
    sumM = 0.0
    for id in self.mTgtSensors:
      sumM += self.mTgtSensors[id]['Mi']
    self.mBrum['H(M)'] = self.Heaviside(sumM)
    return self.mBrum['H(M)']

  #--
  def f_S(self, psi):
    """ Target input source strength function.

        S = [Sum(Mi * exp(-sq(psi-psi_i)/2sq(sigma)),i=0,numsensors)
              - M0] * Heaviside(Sum(Mi),i=0,numsensors)
          where
            Mi    - measure target strength scalar from sensor i
            psi_i - angle to sensor i in world coordinates
            sigma - Gausian kernel width

        Parameters:
          psi   - robot angle, ccw from front of robot

        Return Value:
          S
    """
    HofM = self.mBrum['H(M)']
    if HofM == 0.0:
      return 0.0
    theta = self.mRobotLoc['theta']
    sumS = 0.0
    for id in self.mTgtSensors:
      dpsi = psi - theta - self.mTgtSensors[id]['zeta']
      sumS += self.mTgtSensors[id]['Mi'] * \
          math.exp(-(dpsi**2)/self.mBrum['2sq(sigma)'])
    return sumS - self.mOpt['M0']

  #--
  def S_field(self):
    """ Target input source potential field.

        S[i] = S(psi), for psi=0,2pi
    """
    self.f_H_M()
    self.mBrum['S'] = []
    for psi in self.mAngRad:
      self.mBrum['S'].append(self.f_S(psi))

  #--
  def u_field(self):
    """ Target activation potential field.

        u[i] = S(psi) + ifi(psi) + h, for psi=0,2pi
    """
    N = len(self.mAngRad)
    i = 0
    unew = []
    u_max = 0.0
    while i < N:
      psi = self.mAngRad[i]
      u = self.mBrum['S'][i] + self.f_interaction(psi) + self.mBrum['h']
      unew.append(u)
      if u > u_max:
        u_max = u
      i += 1
    self.mBrum['u']     = unew
    self.mBrum['u_max'] = u_max

  #--
  def f_w(self, psi1, psi2):
    """ Target interaction kernel.

        w(dpsi) = k_p  if -l_coop < 2dpsi < l_coop
                = -k_n otherwise

        Parameters:
          psi1    - angle one
          psi2    - angle two

        Return Value:
          w
    """
    l_coop = self.mBrum['l_coop']
    psi1 = math.fmod(psi1, twopi)
    psi2 = math.fmod(psi2, twopi)
    l = math.fmod(psi2 - l_coop + twopi, twopi)
    u = math.fmod(psi2 + l_coop, twopi)
    if l > u:
      if l <= psi1 and psi1 <= twopi:
        return self.mOpt['k_p']
      elif psi1 <= u:
        return self.mOpt['k_p']
    elif l <= psi1 and psi1 <= u:
      return self.mOpt['k_p']
    return -self.mOpt['k_n']

  #--
  def f_interaction(self, psi):
    """ Target intrafield interaction function.
        
        ifi = Integral(w(psi-psi_p)*Omicron(u(psi_p))dpsi_p),dpsi_p=0,2pi

        Parameters:
          psi - angle of interest

        Return Value:
          ifi
    """
    N = len(self.mAngRad)
    dpsi = self.mBrum['dpsi']
    ifi = 0.0
    i = 0
    while i < N:
      psi_p = self.mAngRad[i]
      k = self.f_w(psi, psi_p)
      o = self.Omicron(self.mBrum['u'][i])
      #if self.mStep in [3, 4] and psi == math.radians(176):
      #  print('k=%.3f o=%.3f' % (k, o))
      ifi += k * o * dpsi
      i += 1
    #if self.mStep in [3, 4]:
    #  print('psi=%3f u=%.3f' % (math.degrees(psi), ifi))
    return ifi

  #--
  def f_Ns(self):
    """ Total (positive) input source field activation S function.

        Ns = Integral(S(psi)dpsi),psi=0,2pi

        Return Value:
          Ns
    """
    Ns = 0.0
    for s in self.mBrum['S']:
      Ns += s * self.mBrum['dpsi']
    self.mBrum['Ns'] = Ns
    return Ns

  #--
  def f_Nu(self):
    """ Total positive field activation u function.

        Nu = Integral(Heaviside(u(psi))dpsi),psi=0,2pi

        Return Value:
          Nu
    """
    Nu = 0.0
    for u in self.mBrum['u']:
      Nu += self.Heaviside(u) * self.mBrum['dpsi']
    self.mBrum['Nu'] = Nu
    return Nu

  #--
  def f_dhdt(self):
    """ The change of h (target activation field bias) w.r.t. time function.

        dh/dt = -r_h,min * c_h * (h-h_min) - r_h,max * (1-c_h) * (h-h_max)
          where
            h_min    target activation monostable regime minimum
            h_max    target activation bistable regime maximum
            r_h,min  target activation destabilization rate
            r_h,max  target activation stabilization rate

        Return Value:
          dh/dt
    """
    c_h = (self.Heaviside(self.mBrum['Nu']) - self.Heaviside(self.mBrum['Ns']))\
        * self.Heaviside(self.mBrum['Nu'])
    h = self.mBrum['h']
    dhdt = -self.mBrum['r_h_min'] * c_h * (h - self.mOpt['h_min']) \
           - self.mBrum['r_h_max'] * (1.0 - c_h) * (h - self.mOpt['h_max'])
    #print('Ns=%.2f Nu=%.2f c_h=%.2f dh=%.3f h=%.3f' % \
    #    (self.mBrum['Ns'], self.mBrum['Nu'], c_h, dhdt, h+dhdt))
    return dhdt

  #--
  def f_tar(self):
    """ Target attractive force-let function on target direction. """
    Nu = self.mBrum['Nu']

    if Nu == 0:
      self.mBrum['psi_tar'] = 0.0
      self.mBrum['dtheta_tar'] = 0.0
      return self.mBrum['dtheta_tar']

    lambda_tar_p  = self.mBrum['lambda_tar_p']
    theta         = self.mRobotLoc['theta']
    N             = len(self.mAngRad)
    centroid      = 0.0
    i             = 0
    while i < N:
      psi = self.mAngRad[i]
      u   = self.mBrum['u'][i]
      centroid += psi * self.Heaviside(u) * self.mBrum['dpsi']
      i += 1
    psi_tar = self.mBrum['psi_tar'] = centroid / Nu
    if psi_tar - halfpi < theta and theta <= psi_tar + halfpi:
      self.mBrum['dtheta_tar'] = -lambda_tar_p * (Nu * theta - centroid)
    else:
      self.mBrum['dtheta_tar'] = lambda_tar_p * (Nu * (theta - pi) - centroid)
    return self.mBrum['dtheta_tar']

  #--
  def Heaviside(self, x):
    """ Heavside step function. """
    if x > 0.0:
      return 1.0
    else:
      return 0.0

  #--
  def Omicron(self, x):
    """ Sigmoid threshold function. """
    if x < 0.0:
      return 0.0
    elif x < 1.0:
      return x
    else:
      return 1.0

  #--
  def Integral(self, integrand, lbound, ubound, dx, *params):
    """ Integrate the definite integral by simple trapezoidal algorithm.

        Parameters:
          integrand   - function to integrate over
          lbound      - definite integral lower bound
          ubound      - definite integral upper bound
          dx          - discrete dx step size
          *params     - optional parameters passed on each call to the
                        integrand

        Return Value:
          The integration.
    """
    if params:
      intx = lambda x, params: integrand(x, *params)
    else:
      intx = lambda x, params: integrand(x)
    x0 = lbound
    intx0 = intx(x0, params)
    x1 = x0 + dx
    sum = 0.0
    while x1 <= ubound:
      intx1 = intx(x1, params)
      sum += (intx0 + intx1) / 2.0 * dx
      x0 = x1
      intx0 = intx1
      x1 += dx
    if x1 < ubound + dx:
      dx = ubound - x0
      x1 = ubound
      intx1 = intx(x1, params)
      sum += (intx0 + intx1) / 2.0 * dx
    return sum


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Interface Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
  #--
  def hooks(self, sgrapher, ugrapher, showtimestep, showexperiment):
    self.mSGrapher = sgrapher
    self.mUGrapher = ugrapher
    self.mShowTimeStep = showtimestep
    self.mShowTestName = showexperiment
    self._cleanslate()

  #--
  def start(self):
    """ Start control loop. """
    self.mT     = -self.mDt
    self.mStep  = -1
    self.mTests[self.mTestNum]['init']()
    self._cmdunlock()

  def restart(self):
    self._cmdlock()
    self.start()

  #--
  def next(self):
    self._cmdlock()
    self.mTestNum += 1
    if self.mTestNum >= len(self.mTests):
      self.mTestNum = 0
    self._cleanslate()
    return self.mTestNum

  #--
  def exectest(self, testNum):
    self._cmdlock()
    self.mTestNum = testNum
    self._cleanslate()

  #--
  def pause(self):
    self._cmdlock()

  #--
  def resume(self):
    self._cmdunlock()

  def _cmdlock(self):
    if not self.mCmdLock:
      self.mSema.acquire()
      self.mCmdLock = True

  def _cmdunlock(self):
    self.mCmdLock = False
    self.mSema.release()

  #--
  def _cleanslate(self):
    self.mShowTestName(self.mTests[self.mTestNum]['name'])
    self.mShowTimeStep(0)
    self.mSGrapher.graph(title=self.mSGraphTitle, xlabel='psi', ylabel='S')
    self.mUGrapher.graph(title=self.mUGraphTitle, xlabel='psi', ylabel='u')

  #--
  def gettestlist(self):
    return self.mTests

  #--
  def getopts(self):
    return self.mOpt

  #--
  def setopts(self, opt):
    self.mOpt = opt
    self.BrumOpts()

  #--
  def cancel(self):
    """ Cancel control loop """
    if not self.mLoopTimer:
      return
    self.mDoCancel = True
    self.mSema.release()
    self.mLoopTimer.cancel()
    del self.mLoopTimer
    self.mLoopTimer = None


#-------------------------------------------------------------------------------
# CLASS: GuiViz
#-------------------------------------------------------------------------------
class GuiViz:
  """ Quick GUI visualizer. """
  def __init__(self, dynaTgt):
    self.mDynaTgt     = dynaTgt
    self.mRoot        = None
    self.mRootGeo     = None
    self.mRootFixH    = 0
    self.mRootBorder  = 0
    self.mSGraph      = None
    self.mUGraph      = None
    self.body()
    self.setstate('notstarted')
    self.mDynaTgt.hooks(self.mSGraph, self.mUGraph, self.ShowTimeStep,
                        self.ShowTestName)
    self.mRoot.bind('<Configure>', self.resize)

  #--
  def body(self):
    self.mRoot = tk.Tk()
    self.mRoot.wm_title('DynaTgt - Validation of "Target Representation on'
        ' an Autonomous Vehicle with Low-Level Sensors"')

    canvas_s = tk.Canvas(self.mRoot, width=600, height=200)
    canvas_s.grid(row=0, column=0)

    canvas_u = tk.Canvas(self.mRoot, width=600, height=200)
    canvas_u.grid(row=0, column=1)

    frame = tk.Frame(self.mRoot)
    frame.grid(row=1, column=0, columnspan=2)

    self.mVarMenu = tk.IntVar()
    menu = tk.Menu(self.mRoot)
    testmenu = tk.Menu(menu)
    menu.add_cascade(label='Tests', menu=testmenu)
    testList = self.mDynaTgt.gettestlist()
    n = len(testList)
    i = 0
    while i < n:
      testmenu.add_radiobutton(label=testList[i]['name'],
          command=self.CbTest, value=i, variable=self.mVarMenu)
      i += 1
    menu.add_command(label='Options...', command=self.CbOpt)
    self.mRoot.config(menu=menu)

    fcol = 0
    bttn = tk.Button(frame, text='(Re)Start', width=8, fg=gt.ColorGreen1,
        command=self.CbStart)
    bttn.grid(row=0, column=fcol, sticky=tk.W)

    fcol += 1
    bttn = tk.Button(frame, text='Pause', width=8, fg=gt.ColorRed1,
        command=self.CbGoNoGo)
    bttn.grid(row=0, column=fcol, sticky=tk.W)
    self.mButtonGoNoGo = bttn

    fcol += 1
    bttn = tk.Button(frame, text='Next >', width=8, command=self.CbNext)
    bttn.grid(row=0, column=fcol, sticky=tk.W)

    fcol += 1
    bttn = tk.Button(frame, text='GoodBye', width=8, fg=gt.ColorRed1,
        command=self.mRoot.destroy)
    bttn.grid(row=0, column=fcol, sticky=tk.W)

    fcol += 1
    w = tk.Label(frame, text='     ')
    w.grid(row=0, column=fcol)

    fcol += 1
    w = tk.Label(frame, relief=tk.FLAT, justify=tk.LEFT,
        anchor=tk.W, text='Time Step:', fg=gt.ColorBlue1)
    w.grid(row=0, column=fcol, sticky=tk.W)

    fcol += 1
    self.mVarTimeStep = tk.StringVar()
    w = tk.Label(frame, width=4, relief=tk.SUNKEN, justify=tk.LEFT,
        anchor=tk.W, textvariable=self.mVarTimeStep)
    w.grid(row=0, column=fcol, sticky=tk.W)
    self.mVarTimeStep.set("")

    fcol += 1
    w = tk.Label(frame, text=' ')
    w.grid(row=0, column=fcol)

    fcol += 1
    w = tk.Label(frame, relief=tk.FLAT, justify=tk.LEFT,
        anchor=tk.W, text='Test:', fg=gt.ColorBlue1)
    w.grid(row=0, column=fcol, sticky=tk.W)

    fcol += 1
    self.mVarTestName = tk.StringVar()
    w = tk.Label(frame, width=70, relief=tk.SUNKEN, justify=tk.LEFT,
        anchor=tk.W, textvariable=self.mVarTestName)
    w.grid(row=0, column=fcol, sticky=tk.W)
    self.mVarTestName.set("")

    self.mSGraph = GuiXYGraph.GuiXYGraph(canvas_s) 
    self.mUGraph = GuiXYGraph.GuiXYGraph(canvas_u) 

    self.calcdim(canvas_s, canvas_u, frame)

  #--
  def calcdim(self, canvas_s, canvas_u, frame):
    self.mRoot.update_idletasks()
    self.mRootGeo = gut.geometry(self.mRoot)
    fgeo = gut.geometry(frame)
    self.mRootFixH = self.mRootGeo[1] - 200 #fgeo[1]
    self.mRootBorder = self.mRootGeo[0] - \
      int(canvas_s['width']) - int(canvas_u['width'])

  #--
  def resize(self, event):
    geo = gut.geometry(self.mRoot)
    if geo[0] != self.mRootGeo[0] or geo[1] != self.mRootGeo[1]:
      width = geo[0] - self.mRootBorder
      height = geo[1] - self.mRootBorder - self.mRootFixH
      self.mDynaTgt.pause()
      self.mSGraph.configure(width/2, height)
      self.mUGraph.configure(width/2, height)
      self.mDynaTgt.resume()
      self.mRootGeo = geo

  #--
  def ShowTimeStep(self, step):
    self.mVarTimeStep.set('%d' % step)

  #--
  def ShowTestName(self, desc):
    self.mVarTestName.set(desc)

  #--
  def CbStart(self):
    if self.mGuiState == 'notstarted':
      self.mDynaTgt.start()
    else:
      self.mDynaTgt.restart()
    self.setstate('running')

  #--
  def CbGoNoGo(self):
    if self.mGuiState == 'running':
      self.mDynaTgt.pause()
      self.setstate('paused')
    elif self.mGuiState == 'paused':
      self.mDynaTgt.resume()
      self.setstate('running')

  #--
  def CbNext(self):
    testNum = self.mDynaTgt.next()
    self.mVarMenu.set(testNum)
    self.setstate('notstarted')

  #--
  def CbOpt(self):
    opts = self.mDynaTgt.getopts()
    dlg = GuiDlgDynaOpt.GuiDlgDynaOpt(self.mRoot, DynaDesignParamDfts, opts)
    if dlg.result:
      self.mDynaTgt.setopts(dlg.result)

  #--
  def CbTest(self):
    testNum = self.mVarMenu.get()
    self.mDynaTgt.exectest(testNum)
    self.setstate('notstarted')

  #--
  def setstate(self, newstate):
    self.mGuiState = newstate
    self.mButtonGoNoGo['state'] = tk.NORMAL
    if newstate == 'notstarted':
      self.mButtonGoNoGo['text'] = 'Pause'
      self.mButtonGoNoGo['fg'] = gt.ColorRed1
      self.mButtonGoNoGo['state'] = tk.DISABLED
    elif newstate == 'paused':
      self.mButtonGoNoGo['text'] = 'Resume'
      self.mButtonGoNoGo['fg'] = gt.ColorGreen1
    elif newstate == 'running':
      self.mButtonGoNoGo['text'] = 'Pause'
      self.mButtonGoNoGo['fg'] = gt.ColorRed1

  #--
  def test(self):
    v = self.mDynaTgt.Integral(self.parab, 0.0, 3.0, .25)
    print('I(x^2)|x=0,x=3 =', v)
    v = self.mDynaTgt.Integral(self.parab, 0.0, 3.0, .22)
    print('I(x^2)|x=0,x=3 =', v)
    v = self.mDynaTgt.Integral(self.fsin, 0.0, pi, .1, 1.0)
    print('I(sin(x)|x=0,x=pi =', v)
    v = self.mDynaTgt.Integral(self.fsin, 0.0, pi, .1, 2.0)
    print('I(sin(2x)|x=0,x=pi =', v)

  #--
  def parab(self, x):
    return x ** 2

  #--
  def fsin(self, x, w):
    return math.sin(w*x)


#-------------------------------------------------------------------------------
# Visualizer
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  #--
  def main():
    """ DynaTgt Visiualizer. """
    dynatgt = DynaTgt()
    gui = GuiViz(dynatgt)
    gui.mRoot.mainloop()
    dynatgt.cancel()

  # run test
  main()
