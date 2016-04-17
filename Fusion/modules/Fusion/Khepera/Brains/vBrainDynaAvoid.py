################################################################################
#
# vBrainDynaAvoid.py
#

""" Circling Dynamic Avoidance Virtual Brain Module.

The objective of the DynaAvoidBrain is to travel in a circle of a specified 
radius (the target, albeit moving) while avoiding any obstacles.

The robot under control is a K-Team Khepera II. Built into the robot are
6 forward and sideways facing IR LED proximity sensors (the back 2 are 
ignored by this brain). In addition, as Sharp GP2D120 IR LED sensor has
been attached to a Khepera General I/O module and stacked on the Khepera.
The GP2D120 faces forward. This sensor provides real distance measurements
from 4cm - 30cm. The Khepera also has odometers and speedometers affixed
to both of the robots 2 motor driven wheels. These sensors provide location
sensory information to keep the robot traveling in a circle. The Khepera
is holonimic.

The cognitive algorithms of this brain use a dynamic approach (attractor
dynamics) to fuse the IR LED sensor information into a repulsive field,
while the location sensors, along with the circling goal, provide an
attractive field.

This brain is based on the paper:
  Estela Bicho, Pierre Mallet, and Gregor Shoner, "Target Representation
  on an Autonomous Vehicle with Low-Level Sensors", The International
  Journal of Robotics Research, May 2000


Author: Robin D. Knight
Email:  robin.knight@roadnarrowsbrainics.com
URL:    http://www.roadnarrowsbrainics.com
Date:   2005.12.29

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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
import Fusion.Utils.Pt as pt

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vBrain as vBrain
import Fusion.Core.vBrainThreaded as vBrainThreaded

import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox

import Fusion.Khepera.Robots.KheValues as KheValues
import Fusion.Khepera.Gui.GuiDlgDynaOpt as GuiDlgDynaOpt
import Fusion.Khepera.Gui.GuiWinDynaViz as GuiWinDynaViz


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

BrainMimeType       = 'brain/DynaAvoid'
BrainIniDDSectOpts  = BrainMimeType + '/' + 'options'

# Ini Definition Dictionary.
BrainIniDD = {
  # section
  BrainIniDDSectOpts: ['DynaAvoid brain options',
  {
    # Field Parameters
    'dpsi':               [8.0, 'dynamics delta sampling size (degrees)'],
    'k_p':                [2.0, 'interaction kernal excitatory constant'],
    'k_n':                [3.5, 'interaction kernal inhibitory constant'],
    'l_coop':             [math.radians(40.0), 
                    'interaction kernel cooperativity length (radians)'],
    'k_tau':              [5.0, 'execution cycle time multiplier'],
    #'W_m':               [ 0.7, 'h_min,max implicitely code this value'],
    'h_min':              [-0.77, 'target activation monostable regime limit'],
    'h_max':              [-0.0875, 'target activation bistable regime limit'],
    'k_r_h_min':          [5.0, 'target activation destabilization rate'],
    'k_r_h_max':          [50.0, 'target activation stabilization rate'],

    # Target Acquisition',
    'TgtSensorAngRange':  [math.radians(120.0),
                          'simulated target sensor(s) angular range (radians)'],
    'M0':                 [0.0,  'zero set point target sensor(s) bias.'],
    'sigma':              [0.4,  'Gausian kernel convolver width (radians)'],
    'k_lambda_tar_p':     [10.0, 'target attractive force-let constant'],

    # Obstacle Avoidance
    'k_beta_1':           [5.0, 'maximal repulsive force-let strength'],
    'beta_2':             [20.0, 'decay distance repulsive force-let strength'],

    # Velocity Control
    'c':                  [100.0,  'velocity control sigmoid steepness'],
    'k_c_v_tar':          [15.0,   'target velocity relaxation rate'],
    'k_c_v_obs':          [10.0,   'obstacle velocity relaxation rate'],
    'psi_hat_max':        [35.0,   'maximum robot speed (mm/s)'],

    # Execution
    'ExecCycle':          [0.05, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':       [1.0,   "Execution 'Step' size (seconds)."]
  }]
}

# some math constants
pi          = math.pi
halfpi      = math.pi / 2.0
threehalfpi = math.pi * 3.0 / 2.0
twopi       = math.pi * 2.0
e           = math.e
sqrte       = math.sqrt(math.e)


#-------------------------------------------------------------------------------
# CLASS: vBrainDynaAvoid
#-------------------------------------------------------------------------------
class vBrainDynaAvoid(vBrainThreaded.vBrainThreaded):
  """ Attractive Dynamics Circular Avoidance vBrain Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize Dyna Avoid vBrain instance.

        Parameters:
          client      - Gluon client
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    # base class
    vBrainThreaded.vBrainThreaded.__init__(self,
        serverId=self.HasName(), client=client,
        debuglevel=debuglevel, debugfout=debugfout)

  #--
  def vBrainInit(self):
    """ One-time vBrain initialization during object instantiation. """
    # simulated target sensors
    self.mTgtSensors = {}
    for ang in [-135.0, 90.0, 45.0, 0.0, -45.0, -90.0, 135.0]:
      id = 'tgtsensor(%d)' % int(ang)
      self.mTgtSensors[id] = {}
      self.mTgtSensors[id]['zeta'] = math.radians(ang)

    # standard threaded brain initialization
    vBrainThreaded.vBrainThreaded.vBrainInit(self);

    # brain gui initializations (after vBrainThreaded)
    self.GuiInit()

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainDynaAvoid Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the brain type.

        Return Value:
          The brain MIME type string.
    """
    return BrainMimeType

  #--
  def HasName(self):
    """ Returns the short brain name(s) string.

        Return Value:
          The brain name string.
    """
    return 'DynaAvoidBrain'

  #--
  def IsVersion(self):
    """ Returns the vBrain version(s) string.

        Return Value:
          The brain version(s) string.
    """
    return '1.0'

  #--
  def HasDesc(self):
    """ Returns a short description of this vBrain.

        Return Value:
          Multi-line description string.
    """
    sDesc = """Circling Dynamic Avoidance Brain."""
    return sDesc


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Ini (Re)Initalization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """
    self.mIniDD = BrainIniDD
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vBrain run-time options
    self.IniInitOpt()

    # load think-act operational settings
    self.IniInitThinkAct()

  #--
  def IniInitOpt(self):
    """ Initialized vBrain options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = BrainIniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)

    self.BrumOpts()

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitThinkAct(self):
    """ Initialized think-act operational settings from parsed
        configuration.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded to vRobot Peer Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PeerRobotNull(self):
    """ Set the vBrain peer data to the null set.

        Return Value:
          None
    """
    self.mObsSensorGroups = []    # obsctacle sensor groups
    self.mObsSensors      = {}    # obstacle sensor data
    self.mRobotDiameter   = 0.0   # robots effective diameter

    self.mPeerRobotCallbacks['bellumsetgoals'] = self.PeerRobotCbNoOp
    self.mPeerRobotCallbacks['shadow'] = self.PeerRobotCbNoOpShadow

  #--
  def PeerRobotEstablish(self):
    """ Establish the vBrain to vRobot peer interface when the vRobot
        peer registers with this vBrain.

        Implementation specific.

        Return Value:
          True if vRobot peer is compatible with this vBrain and the
          vRobot has sufficient resources. Else return False.
    """
    # validate robot
    if self.mPeer.IsType() != KheValues.KheMimeType:
      self.GSReportErrorStatus(
          'vBrain does not support vRobot type: %s' % self.mPeer.IsType())
      return False

    # null sensor data
    self.PeerRobotNull()

    # search available sensors for the relevant sensors and mark
    sensorDict = self.mPeer.HasSensorTypes()
    hasSufficient = False

    for id,params in sensorDict.iteritems():
      mimetype = params['mimetype']

      # proximity IR sensor
      if mimetype == KheValues.KheSensorMimeTypeProximity:

        # only work with sensors on the front half of the robot
        if params['zeta'] <= twopi/4.0 or params['zeta'] >= twopi * 3.0 / 4.0:
          self.mObsSensors[id] = {}
          self.mObsSensors[id]['zeta'] = params['zeta']
          self.mObsSensors[id]['angrange'] = params['angrange']
          if mimetype not in self.mObsSensorGroups:
            self.mObsSensorGroups.append(mimetype)
            self.mPeerRobotCallbacks[mimetype] = self.mPeer.ShadowGet
          hasSufficient = True

      # distance measuring IR sensor
      elif mimetype == KheValues.KheSensorMimeTypeGP2D120:
        
        # only work with sensors on the front half of the robot
        if params['zeta'] <= twopi/4.0 or params['zeta'] >= twopi * 3.0 / 4.0:
          self.mObsSensors[id] = {}
          self.mObsSensors[id]['zeta'] = params['zeta']
          self.mObsSensors[id]['angrange'] = params['angrange']
          if mimetype not in self.mObsSensorGroups:
            self.mObsSensorGroups.append(mimetype)
            self.mPeerRobotCallbacks[mimetype] = self.mPeer.ShadowGet
          hasSufficient = True

    # not sufficient sensors for this brain
    if not hasSufficient:
      self.GSReportErrorStatus(
          'vBrain does not have sufficient vRobot %s sensors available' % \
              self.mPeer.IsType())
      return False

    # final sensor prepping 
    for id,data in self.mObsSensors.iteritems():
      # convert any zeta in [3/2pi, 2pi] to [-1/2pi, 0]
      if data['zeta'] >= twopi * 3.0 / 4.0:
        self.mObsSensors[id]['zeta'] = data['zeta'] - twopi

    # robot location callback
    self.mPeerRobotCallbacks['shadow'] = self.mPeer.ShadowGet

    # bellum goal setting callback
    self.mPeerRobotCallbacks['bellumsetgoals'] = self.mPeer.BellumSetGoals

    # gather some physical properties of the vRobot
    props = self.mPeer.HasPhysicalProperties()
    self.mRobotDiameter = props['diameter']['val']

    # update dynamics viz window of obstacle sensor arrangement
    self.GSGuiWinUpdate('VizDynamics', 'cfg', obs_sensors=self.mObsSensors)

    return True

  #--
  def PeerRobotCbNoOpShadow(self, groupId):
    """ A locaton NoOp vRobot peer callback. All agruments are ignored and
        no true peer callback is performed.
        
        Return Value:
          An empty dictionary {}.
        
    """
    if groupId == KheValues.KheSensorMimeTypeOdometer:
      return {'pathdist':0.0, 'theta':0.0, 'x':0.0, 'y':0.0}
    elif groupId == KheValues.KheSensorMimeTypeSpeedometer:
      return {'pathspeed':0.0}
    else:
      return {}


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainDynaAvoid (Cere)Brum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BrumNull(self):
    """ Set the vBrains's high-level states to the null set.

        Return Value:
          Returns new current brum state.
    """
    # user options Brum dependencies
    self.BrumOpts()

    # Part of the 'innate' brum lies in the senors. Set any fix components
    # for the dynamic calculations
    for sensor in self.mObsSensors.itervalues():
      sensor['tan(angrange)'] = math.tan(sensor['angrange']/2.0)
      sensor['f_obs_i'] = 0.0

    # set the 'innate' 
    if self.mRobotDiameter < 1.0:
      self.mBrum['R_robot'] = 0.5
    else:
      self.mBrum['R_robot'] = self.mRobotDiameter / 2.0

    # robot location and speed
    self.mRobotLoc = \
      self.PeerRobotCbNoOpShadow(KheValues.KheSensorMimeTypeOdometer)
    self.mRobotSpeed = \
        self.PeerRobotCbNoOpShadow(KheValues.KheSensorMimeTypeSpeedometer)

    # target data
    self.mBrum['h'] = self.mOpt['h_max']
    self.mBrum['S'] = []
    self.mBrum['u'] = []
    for ang in self.mAngRad:
      self.mBrum['S'].append(0.0)
      self.mBrum['u'].append(self.mBrum['h'])

    return self.mBrum

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

    for sensor in self.mTgtSensors.itervalues():
      sensor['angrange'] = self.mOpt['TgtSensorAngRange']

    # update dynamics viz window of angular sampling points
    self.GSGuiWinUpdate('VizDynamics', 'cfg', ang_deg=self.mAngDeg)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the brain.

        Callback hook for vBrainThreaded emergency stopping event.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Think-Act Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ThinkActInit(self):
    """ Initialize Think-Act Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vBrainThreaded.vBrainThreaded.ThinkActInit(self)
    self.SimTgtsInit()
    self.mPeerRobotCallbacks['bellumsetgoals'](speed=self.mOpt['psi_hat_max'])
    self.GSReportNormalStatus(
        "Travel ccw in a %dmm radius circle centered at %s "
        "while avoiding obstacles." % (int(self.mTgtRadius), self.mTgtCenter))
 
  #--
  def ThinkAct(self):
    """ Apply attractor dynamics for build obstacle repulsive and target
        attractor force fields. Then act on the sum of these fields.

        Execution Context: Cog thread

        Return Value:
          None
    """
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('ThinkAct')

    # retrieve obstacle detecting sensor data
    dist_min = 1000.0
    for group in self.mObsSensorGroups:
      dataset = self.mPeerRobotCallbacks[group](group)
      for id,val in dataset.iteritems():
        if self.mObsSensors.has_key(id):
          self.mObsSensors[id]['dist'] = val[0]
          if val[0] < dist_min:
            dist_min = val[0]
    self.mBrum['obs_dist_min'] = dist_min

    # retrieve robot location and speed
    self.mRobotLoc = \
      self.mPeerRobotCallbacks['shadow'](KheValues.KheSensorMimeTypeOdometer)
    self.mRobotSpeed = \
      self.mPeerRobotCallbacks['shadow'](KheValues.KheSensorMimeTypeSpeedometer)

    # update dynamics viz window of robot's current movement
    self.GSGuiWinUpdate('VizDynamics', 'status',
        speed=self.mRobotSpeed['pathspeed'], theta=self.mRobotLoc['theta'])

    # update simulated targets
    self.SimTgtsUpdate()

    # retrieve simulated target measured sensor data
    for id in self.mTgtSensors:
      self.mTgtSensors[id]['Mi'] = self.SimTgtSensorStrength(id)

    # per execution cycle up fronts
    self.mBrum['dt']            = self.mOpt['ExecCycle']
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

    # build repulsive and attractive fields
    self.Sum_f_obs_i()  # obstacle repulsive force-let sum
    self.S_field()      # generate target source field
    self.u_field()      # generate activation field
    self.f_Ns()         # total source activation
    self.f_Nu()         # total positive activation
    self.f_tar()        # target attractive force-let
    self.mBrum['h'] += self.f_dhdt()  # new activation bias

    # update dynamics viz window of new dynamics
    self.GSGuiWinUpdate('VizDynamics', 'Mi', tgt_sensors=self.mTgtSensors)
    self.GSGuiWinUpdate('VizDynamics', 'f_obs_i', obs_sensors=self.mObsSensors)
    self.GSGuiWinUpdate('VizDynamics', 'S_field', field=self.mBrum['S'])
    self.GSGuiWinUpdate('VizDynamics', 'u_field', field=self.mBrum['u'])

    if __debug__:
      self.mDbg.d4print('Ns=%f, Nu=%f, h=%f' % \
          (self.mBrum['Ns'], self.mBrum['Nu'], self.mBrum['h']))
      self.mDbg.d4print('dtheta_obs=%.3f, dtheta_tar=%.3f' % \
        (math.degrees(self.mBrum['dtheta_obs']), 
         math.degrees(self.mBrum['dtheta_tar'])))
 
    # apply the calculated dynamic attractor fields to set robot speed and
    # direction.
    self.mBrum['dtheta'] = self.mBrum['dtheta_obs'] + self.mBrum['dtheta_tar']
    #self.mBrum['dtheta'] = self.mBrum['dtheta_tar']
    self.mBrum['dspeed'] = 0.0 # RDK!!! future: self.f_dvdt()

    self.mPeerRobotCallbacks['bellumsetgoals'](dtheta=self.mBrum['dtheta'])
                                               #dspeed=self.mBrum['dspeed'])

    # update dynamics viz window of new movement goals
    self.GSGuiWinUpdate('VizDynamics', 'status',
        dtheta=self.mBrum['dtheta'],
        dtheta_obs=self.mBrum['dtheta_obs'],
        dtheta_tar=self.mBrum['dtheta_tar'],
        dspeed=self.mBrum['dspeed'])

    if __debug__:
      self.mDbg.d4print('dspeed=%.3f, dtheta=%.3f' % \
          (self.mBrum['dspeed'], math.degrees(self.mBrum['dtheta'])))

  #--
  def SimTgtsInit(self):
    """ Initialize simulated targets. """
    self.mTgtCenter   = pt.pt(300.0, 0.0)   # center of target circle
    self.mTgtRadius   = 200.0               # target circle radius
    self.mTgtSpread   = math.radians(5.0)   # target angular spread on circle
    self.mTgtMaxGain  = 4.0                 # maximum 'power' emitted
    self.mTgtGain     = self.mTgtMaxGain * 0.5  # max gain reached at .5m
    self.mTargets = {}    # place the initialize the targets
    for n in [0, 1]:
      id = 'tgt_%d' % n
      rho = pi + n * self.mTgtSpread    # rho is relative to center
      x = self.mTgtCenter.x + self.mTgtRadius * math.cos(rho)
      y = self.mTgtCenter.y + self.mTgtRadius * math.sin(rho)
      self.mTargets[id] = {'rho':rho, 'x':x, 'y':y, 'dist':0.0}
    self.mTgtNextRho = math.fmod(rho+self.mTgtSpread, twopi) 

  #--
  def SimTgtsUpdate(self):
    """ Update targets positions. """
    # center - robot metrics
    x_bot = self.mRobotLoc['x']
    y_bot = self.mRobotLoc['y']
    dx = x_bot - self.mTgtCenter.x
    dy = y_bot - self.mTgtCenter.y
    dist_c = math.hypot(dx, dy)       # robot's distance from center
    rho_c = math.acos(dx/dist_c)      # angle from center to robot [0,pi]
    if dy < 0.0:                      # map to [0, twopi]
      rho_c = twopi - rho_c
    if __debug__:
      self.mDbg.d4print('center-robot: dist_c=%.1f, rho_c=%.1f' % \
          (dist_c, math.degrees(rho_c)))

    # update targets 
    for id, tgt in self.mTargets.iteritems():
      # target - robot metrics
      dx = tgt['x'] - x_bot
      dy = tgt['y'] - y_bot
      dist_t = math.hypot(dx, dy) # robot's distance from target

      # angle difference
      drho = self.angdist(tgt['rho'], rho_c)

      # move target if robot is too close or has passed the center-target line
      if dist_t <= self.mBrum['R_robot'] or drho < math.radians(1.0):
        rho = self.mTgtNextRho
        x = self.mTgtCenter.x + self.mTgtRadius * math.cos(rho)
        y = self.mTgtCenter.y + self.mTgtRadius * math.sin(rho)
        tgt['rho'] = rho
        tgt['x'] = x
        tgt['y'] = y
        self.mTgtNextRho = math.fmod(rho+self.mTgtSpread, twopi) 

        # recalculate target - robot distance
        dx = tgt['x'] - x_bot
        dy = tgt['y'] - y_bot
        dist_t = math.hypot(dx, dy)     # robot's distance from target
          
      # update target data
      psi = math.acos(dx/dist_t)    # [0, pi]
      if dy < 0.0:                    # map to [0, twopi]
        psi = twopi - psi
      tgt['dist'] = math.hypot(dx, dy)
      tgt['psi'] = math.fmod(psi, twopi)
      if __debug__: 
        self.mDbg.d4print('%s: tgt-loc=(%.1f, %.1f, %.1f), '
                          'tgt-bot: dist=%.1f, psi=%.1f' % \
                (id, tgt['x'], tgt['y'], math.degrees(tgt['rho']),
                 tgt['dist'], math.degrees(tgt['psi'])))

  #--
  def SimTgtSensorStrength(self, sensorId):
    """ Measure simulated targets strength with simulated target sensor.

        Parameters:
          sensorId  - simulated target sensor id

        Return Value:
          Measured strength scaler Mi.
    """
    theta = self.mRobotLoc['theta']
    zeta = self.mTgtSensors[sensorId]['zeta']
    angrange = self.mTgtSensors[sensorId]['angrange']
    w = twopi / (2.0 * angrange)    # 0 at ends of [-angrange/2, angrange/2]
    angrange /= 2.0                 # half angle
    Mi = 0.0
    for tgt in self.mTargets.itervalues():
      # target angle relative to robot [-180, 180]
      dpsi = tgt['psi'] - theta
      if dpsi > pi:
        dpsi -= twopi
      elif dpsi < -pi:
        dpsi += twopi
      dpsi = math.fabs(dpsi - zeta)
      if dpsi <= angrange:
        # target signal strength obeys inverse law 
        m = self.mTgtGain / (tgt['dist'] * 0.001)
        if m > self.mTgtMaxGain:
          m = self.mTgtMaxGain
        # measure target strength is a function of distance and angular offset
        Mi += m * math.cos(w*dpsi)
    if __debug__: self.mDbg.d4print('%s: Mi=%.3f' % (sensorId, Mi))
    return Mi

  #--
  def Sum_f_obs_i(self):
    """ Sum over all of the repulsive obstacle force-lets detected by
        the obstacle detecting sensors.

        f_obs_i = sum( f_obs_i )

        Return Value:
          Repulsive force-let effecting direction.
    """
    sum = 0.0
    for sensorId in self.mObsSensors.iterkeys():
      sum += self.f_obs_i(sensorId)
    self.mBrum['dtheta_obs'] = sum

    return sum

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
    lambda_i = self.f_lambda_i(sensorId)
    sigma_i = self.f_sigma_i(sensorId)
    sensor = self.mObsSensors[sensorId]
    sensor['f_obs_i'] = lambda_i * -sensor['zeta'] * \
                                  math.exp(-sensor['zeta']**2/(2 * sigma_i**2))
    if __debug__: 
      self.mDbg.d4print('f_obs_i[%s]=%.3f' % (sensorId, sensor['f_obs_i']))
    return sensor['f_obs_i']

  #--
  def f_lambda_i(self, sensorId):
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
  def f_sigma_i(self, sensorId):
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
    v     = self.mRobotSpeed['pathspeed']
    alpha = math.atan(self.mOpt['c'] * self.f_U()) / pi
    #print 'alpha', alpha
    c_obs = self.mBrum['c_v_obs'] * (0.5 + alpha)
    c_tar = self.mBrum['c_v_tar'] * (0.5 - alpha)
    #print 'c_obs', c_obs, 'c_tar', c_tar
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
          psi   - target angle, ccw from front of robot

        Return Value:
          S
    """
    HofM = self.mBrum['H(M)']
    if HofM == 0.0:
      return 0.0
    theta = self.mRobotLoc['theta']
    dang = self.angdist(psi, theta)
    sumS = 0.0
    for id in self.mTgtSensors:
      dpsi = dang - self.mTgtSensors[id]['zeta']
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

        w(dpsi) =  k_p : if -l_coop < 2dpsi < l_coop
                = -k_n : otherwise

        Parameters:
          psi1    - angle one
          psi2    - angle two

        Return Value:
          w
    """
    l_coop = self.mBrum['l_coop'] # 2 already factored in
    dpsi = self.angdist(psi2, psi1)
    if math.fabs(dpsi) <= l_coop:
      return self.mOpt['k_p']
    else:
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
      #  print 'k=%.3f o=%.3f' % (k, o)
      ifi += k * o * dpsi
      i += 1
    #if self.mStep in [3, 4]:
    #  print 'psi=%3f u=%.3f' % (math.degrees(psi), ifi)
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
    #print 'Ns=%.2f Nu=%.2f c_h=%.2f dh=%.3f h=%.3f' % \
    #    (self.mBrum['Ns'], self.mBrum['Nu'], c_h, dhdt, h+dhdt)
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
    moment_x      = 0.0   # moment about the y-axis
    moment_y      = 0.0   # moment about the x-axis
    N             = len(self.mAngRad)
    i             = 0
    while i < N:
      psi = self.mAngRad[i]
      drho = self.Heaviside(self.mBrum['u'][i]) * self.mBrum['dpsi']
      moment_x += math.cos(psi) * drho
      moment_y += math.sin(psi) * drho
      i += 1
    tar_x = moment_x / Nu
    tar_y = moment_y / Nu
    if tar_x == 0.0:
      psi_tar = 0.0
    else:
      psi_tar = math.atan(tar_y/tar_x)  # [-90, 90]
    if tar_x < 0.0 and tar_y < 0.0:   # quadrant III
      psi_tar = pi + psi_tar
    elif tar_x < 0.0: # quadrant II
      psi_tar = pi + psi_tar
    elif tar_y < 0.0: # quadrant IV
      psi_tar = twopi + psi_tar
    psi_tar = math.fmod(psi_tar, twopi)
    self.mBrum['psi_tar'] = psi_tar
    dang = self.angdist(psi_tar, theta)
    self.mBrum['dtheta_tar'] = lambda_tar_p * Nu * dang
    #if psi_tar - halfpi < theta and theta <= psi_tar + halfpi:
      #self.mBrum['dtheta_tar'] = -lambda_tar_p * (Nu * theta - moment)
    #else:
      #self.mBrum['dtheta_tar'] = lambda_tar_p * (Nu * (theta - pi) - moment)
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

  #--
  def angdist(self, phi1, phi2):
    """ Calculates the angle distance phi1 - phi2.

        Parameters:
          phi1  - angle 1 (radians)
          phi2  - angle 2 (radians)

        Return Value:
          Differnce in (-pi, pi].
    """
    dang = phi1 - phi2
    if dang > pi:
      dang = dang - twopi
    elif dang <= -pi:
      dang = twopi + dang
    return dang


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # initialize menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Brain|Brain Options...', 'command', 
        callback=self.GuiCbBrainOptions)
    self.mMenuBarList.add('Brain', 'separator')
    self.mMenuBarList.add('Brain|Dynamics Visualizer', 'command', 
        callback=self.GuiCbBrainVizDynamics)
    self.mMenuBarList.add('Brain|Map Visualizer', 'command', 
        callback=self.GuiCbBrainVizMap)
    self.mMenuBarList.add('Help|About Brain...', 'command', 
        callback=self.GuiCbHelpAbout)

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiCbBrainOptions(self):
    """ 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Brain Options')
    
    section = BrainIniDDSectOpts

    # option defaults
    iniDD   = self.mIniDD
    optDict = iniDD[section][1]
    optDfts = {}
    for option,odata in optDict.iteritems():
      optDfts[option] = odata[0]

    # get parsed ini configuation (guaranteed to exist)
    ini           = self.GSGetIni()
    section       = BrainIniDDSectOpts
    settingNames  = GuiDlgDynaOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgDynaOpt.GuiDlgDynaOpt(parent, optDfts,
                  lastSettings=lastSettings, title=self.HasName()+' Options')

    # Serial connection has been successfully opened
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitOpt()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbBrainVizDynamics(self):
    """ 'Dynamics' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Dynamics Visualizer')
    self.GSGuiWinStart('VizDynamics',   # window ID
        GuiWinDynaViz.GuiWinDynaViz)    # start object
    self.GSGuiWinUpdate('VizDynamics', 'cfg',
        obs_sensors=self.mObsSensors,   # obstacle sensor data
        tgt_sensors=self.mTgtSensors,   # target sensor data
        ang_deg=self.mAngDeg)           # angular sample points 

  #--
  def GuiCbBrainVizMap(self):
    """ 'Map' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Map Visualizer')
    msgbox.WarningBox('Discovered map of explored area.\nNot implemented yet.')

  #--
  def GuiCbHelpAbout(self):
    """ 'About' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Brain')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = self.HasName() + ' v' + self.IsVersion()

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='Circling Dynamic Avoidance Demo',
                    desc="""\
The objective of the DynaAvoidBrain is to travel in a circle of a specified 
radius while avoiding any obstacles. Two simulated moving targets are placed
on the circle to lead the Khepera around and around. 

The robot under control is a K-Team Khepera II. Built into the robot are
6 forward and sideways facing IR LED proximity sensors (the back 2 are 
ignored by this brain). In addition, as Sharp GP2D120 IR LED sensor has
been attached to a Khepera General I/O module and stacked on the Khepera.
The GP2D120 faces forward. This sensor provides real distance measurements
from 4cm - 30cm. The Khepera also has odometers and speedometers affixed
to both of the robots 2 motor driven wheels. These sensors provide location
sensory information to keep the robot traveling in a circle. The Khepera
is holonimic.

The cognitive algorithms of this brain use a dynamic approach (attractor
dynamics) to fuse the IR LED sensor information into a repulsive field,
while the location sensors, along with the circling goal, provide an
attractive field.

This brain is based on the paper:
  Estela Bicho, Pierre Mallet, and Gregor Shoner, "Target Representation
  on an Autonomous Vehicle with Low-Level Sensors", The International
  Journal of Robotics Research, May 2000""",
                    copyright='RoadNarrows LLC\n(C) 2006')

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vKhepera test environment. """
    k = vBrainDynaAvoid(debuglevel=level)
    k.ExecLoad()
    return k

  def tstShortRun(k, sec=10):
    """ Short run test """
    iv = 0.1
    i = iv
    state = k.GetBrainState()
    if state == Gluon.EServerState.Ready:
      k.ExecStart()
    while i < sec:
      print 'thinking', i
      time.sleep(iv)
      i += iv
    k.ExecUnload()

  def main():
    """ vBrainDynaAvoid Test Main """
    k = tstCreate()
    tstShortRun(k, 3)

  # run test
  main()
