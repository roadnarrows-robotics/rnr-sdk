################################################################################
#
# vHemisson.py
#

""" Virtual Hemisson Robot.

Virtual Hemisson Robot shadows the physical Hemisson robot including all
supported I2C modules.

vHemisson in compatabile with Hemisson's HemiOS v1.5RNe+ operating system.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.07

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
import re
import Tkinter as tk

import Fusion.Utils.Tools as utils

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vRobot as vRobot
import Fusion.Core.vRobotThreaded as vRobotThreaded

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiDlgSerConn as GuiDlgSerConn
import Fusion.Gui.GuiWinShell as GuiWinShell
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox

import Fusion.Hemisson.Cmd.HemiCmdBase as HemiBase
import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam
import Fusion.Hemisson.Cmd.HemiCmdTts as HemiTts
import Fusion.Hemisson.Cmd.HemiCmdUss as HemiUss
import Fusion.Hemisson.Cmd.HemiCmdAll as HemiCmdAll

import Fusion.Hemisson.Shells.HemiFullShell as HemiCmdShell

import Fusion.Hemisson.Robots.HemiValues as hvals
import Fusion.Hemisson.Robots.HemiIniDD as HemiIniDD

import Fusion.Hemisson.Gui.GuiDlgHemiOpt as GuiDlgHemiOpt
import Fusion.Hemisson.Gui.GuiDlgHemiCalIrLed as GuiDlgHemiCalIrLed
import Fusion.Hemisson.Gui.GuiDlgHemiModLinCam as GuiDlgHemiModLinCam
import Fusion.Hemisson.Gui.GuiDlgHemiModTts as GuiDlgHemiModTts
import Fusion.Hemisson.Gui.GuiDlgHemiModUss as GuiDlgHemiModUss
import Fusion.Hemisson.Gui.GuiWinHemiVizLinCam as GuiWinHemiVizLinCam
import Fusion.Hemisson.Gui.GuiWinHemiVizTts as GuiWinHemiVizTts
import Fusion.Hemisson.Gui.GuiWinHemiVizUss as GuiWinHemiVizUss


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

twopi   = 2.0 * math.pi

# sensor converted value, raw value indices
CVTVAL  = 0
RAWVAL  = 1

#-------------------------------------------------------------------------------
# CLASS: vHemisson
#-------------------------------------------------------------------------------
class vHemisson(vRobotThreaded.vRobotThreaded):
  """ Virtual Hemisson Robot Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize vHemisson instance.

        Parameters:
          client      - Gluon client
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    # base class
    vRobotThreaded.vRobotThreaded.__init__(self, 
        serverId=self.HasName(), client=client,
        debuglevel=debuglevel, debugfout=debugfout)

  #--
  def vRobotInit(self):
    """ One-time vRobot initialization during object instantiation. """
    # hemisson serial commands
    self.mCmd = HemiCmdAll.HemiCmdAll(dbgobj=self.mDbg)
    self.mCommIsUp = self.mCmd.IsOpen()

    # attached module scan list
    self.mModScan = {}
    self.mTtsHasChanged = False  # work around TTS module bug

    # hemisson gui initializations
    self.GuiInit()

    # common threaded robot initialization (includes ini initialization)
    vRobotThreaded.vRobotThreaded.vRobotInit(self)

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)

    # add tool bar
    self.GSSetServerToolBarList(self.mToolBarList)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the Hemisson robot type.

        Return Value:
          The Hemisson robot MIME type string.
    """
    return hvals.HemiMimeType

  #--
  def HasName(self):
    """ Returns the short vHemisson name(s) string.

        Return Value:
          The robot name(s) string which may include either or both
          the vRobot and physical robot names.
    """
    return 'vHemisson'

  #--
  def IsVersion(self):
    """ Returns the vHemisson version(s) string.

        Return Value:
          The vRobot version(s) string.
    """
    return '1.1'

  #--
  def IsRobotVersion(self):
    """ Returns the Hemisson robot version(s) string.

        Return Value:
          Returns version string if connected.
          Else returns 'unknown'.
    """
    ver = None
    if self.mCommIsUp:
      ver = self.mCmd.CmdGetVersion()
      if ver:
        m = re.match('(HemiOS).*(1\.[5-9][0-9]-RN[e-z])', ver) 
        if m:
          return m.group(2)
        else:
          self.GSReportErrorStatus("Unsupported Hemisson OS: %s" % repr(ver))
          return ver
    return 'unknown'

  #--
  def HasDesc(self):
    """ Returns a short description of this vRobot.

        Return Value:
          Multi-line description string.
    """
    sDesc = """\
Hemisson robot with attached Hemisson modules.
Manufactured by K-Team of Switzerland."""
    return sDesc

  #--
  def HasSensorTypes(self):
    """ Returns the dictionary of the sensors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'sensorId', which is vRobot unique.
    
        Return Value:
          Dictionary of sensor id's and properties in the format:
            {sensorId:{'mimetype':<type>...}, ... }
    """
    sensorDict = {}

    # derived time 'sensor'
    sensorDict['time_stamp'] = {
      'mimetype': hvals.HemiSensorMimeTypeTime,
      'units': 's'
    }
    sensorDict['dt'] = {
      'mimetype': hvals.HemiSensorMimeTypeTime,
      'units': 's'
    }

    # proximity sensors
    angrange = math.radians(HemiBase.HemiIrSensorAngRange)
    n = 0
    for irsensor in HemiBase.HemiIrSensorOrder:
      id = 'prox_' + irsensor
      zeta = math.radians(HemiBase.HemiIrSensorAngle[n])
      sensorDict[id] = {
        'mimetype': hvals.HemiSensorMimeTypeProximity,
        'zeta': zeta,
        'angrange':angrange,
        'range': [HemiBase.HemiIrSensorValMin, HemiBase.HemiIrSensorValMax],
        'rawunits': 'adc-10bit',
        'units': 'mm'
      }
      n += 1

    # ambient sensors
    angrange = math.radians(HemiBase.HemiIrSensorAngRange)
    n = 0
    for irsensor in HemiBase.HemiIrSensorOrder:
      id = 'amb_' + irsensor
      zeta = math.radians(HemiBase.HemiIrSensorAngle[n])
      sensorDict[id] = {
        'mimetype': hvals.HemiSensorMimeTypeAmbient,
        'zeta': zeta,
        'angrange':angrange,
        'range': [HemiBase.HemiIrSensorValMin, HemiBase.HemiIrSensorValMax],
        'rawunits': 'adc-10bit',
        'units': 'mm'
      }
      n += 1

    # speedometers
    sensorDict['speedometer_left'] = {
      'mimetype': hvals.HemiSensorMimeTypeSpeedometer,
      'zeta': math.radians(90),
      'range': [HemiBase.HemiSpeedBackwardMax, HemiBase.HemiSpeedForwardMax],
      'resolution': 1,
      'rawunits': 'dc',
      'units': 'unitless'
    }
    sensorDict['speedometer_right'] = {
      'mimetype': hvals.HemiSensorMimeTypeSpeedometer,
      'zeta': math.radians(270),
      'range': [HemiBase.HemiSpeedBackwardMax, HemiBase.HemiSpeedForwardMax],
      'resolution': 1,
      'rawunits': 'dc',
      'units': 'unitless'
    }
    sensorDict['pathspeed'] = {
      'mimetype': hvals.HemiSensorMimeTypeSpeedometer,
      'units': 'unitless'
    }
  
    # linear camera
    sensorDict['linear_camera'] = {
      'mimetype': hvals.HemiSensorMimeTypeLinCam,
      'zeta': math.radians(0.0),
      'angrange':math.radians(120.0),
      'lines': 1,
      'width': HemiLinCam.LinCamNumPixels,
      'range': [0, 255],
      'resolution': 1,
      'units': 'gray-level'
    }

    # ultrasonic sensor
    sensorDict['uss'] = {
      'mimetype': hvals.HemiSensorMimeTypeUss,
      'zeta': math.radians(0.0),
      'angrange':math.radians(HemiUss.UssAngRange),
      'echoes': HemiUss.UssEchoMaxNum,
      'range': [30.0, 6000.0],
      'resolution': 43.0,
      'units': 'mm'
    }

    return sensorDict

  #--
  def HasEffectorTypes(self):
    """ Returns the dictionary of the effectors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'effectorId', which is vRobot unique.
    
        Return Value:
          Dictionary of effector id's and properties in the format:
            {effectorId:{'mimetype':<type>...}, ... }
    """
    effectorDict = {}

    # motors
    effectorDict['motor_left'] = {
      'mimetype': hvals.HemiEffectorMimeTypeWheelMotor,
      'zeta': math.radians(90),
      'resolution': 1,
      'range': [HemiBase.HemiSpeedBackwardMax, HemiBase.HemiSpeedForwardMax],
      'units': 'unitless'
    }
    effectorDict['motor_right'] = {
      'mimetype': hvals.HemiEffectorMimeTypeWheelMotor,
      'zeta': math.radians(270),
      'resolution': 1,
      'range': [HemiBase.HemiSpeedBackwardMax, HemiBase.HemiSpeedForwardMax],
      'units': 'unitless'
    }

    # LEDs
    effectorDict['led0'] = {
      'mimetype': hvals.HemiEffectorMimeTypeLed,
      'color': 'amber',
      'range': [0, 1],
      'units': 'off/on'
    }
    effectorDict['led1'] = {
      'mimetype': hvals.HemiEffectorMimeTypeLed,
      'color': 'amber',
      'range': [0, 1],
      'units': 'off/on'
    }

    # buzzer
    effectorDict['buzzer'] = {
      'mimetype': hvals.HemiEffectorMimeTypeBuzzer,
      'range': [0, 1],
      'units': 'off/on'
    }

    # text-to-speech
    effectorDict['buzzer'] = {
      'mimetype': hvals.HemiEffectorMimeTypeTts,
      'range': [1, 72],
      'units': 'ascii-chars'
    }

    return effectorDict
  
  #--
  def HasPhysicalProperties(self):
    """ Returns the dictionary of Hemisson physical properties.
    
        Return Value:
          Dictionary of physical properties in the format:
            {property:<val>, ... }
    """
    return {
      'diameter': {'val': HemiBase.HemiBaseDiameter, 'units': 'mm'},
      'baseweight': {'val': HemiBase.HemiBaseWeight, 'units': 'g'},
      'wheelbase': {'val': HemiBase.HemiWheelBase, 'units': 'mm'},
    }

  #--
  def HasBellumGoalControls(self):
    """ Returns the maximal list of goal controls (kwGoals) supported
        by vHemisson. 
    
        Return Value:
          List of goal controls.
    """
    return ['speed_left', 'speed_right']


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Ini (Re)Initialization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """

    self.mIniDD = HemiIniDD.GetIniDD()
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vHemisson run-time options
    self.IniInitOpt()

    # load vHemisson connection settings
    self.IniInitConn()

    # load vHemisson proximity ir led sensor settings
    self.IniInitProximity()

    # load vHemisson ambient ir led sensor settings
    self.IniInitAmbient()

    # load vHemisson linear camera sensor settings
    self.IniInitLinCam()

    # load vHemisson text-to-speech effector settings
    self.IniInitTts()

    # load vHemisson ultrasonic sensor settings
    self.IniInitUss()

    # load sense-react operational settings
    self.IniInitSenseReact()

  #--
  def IniInitOpt(self):
    """ Initialized vHemisson options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = HemiIniDD.IniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)
      if option == 'UseLinCamSensor':
        self.GSGuiWinUpdate('VizLinCam', 'cfg',
          **self.GuiWinModComCfg(option, 'linear_camera')) 
      elif option == 'UseUssSensor':
        self.GSGuiWinUpdate('VizUss', 'cfg',
          **self.GuiWinModComCfg(option, 'uss')) 

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitConn(self):
    """ Initialized vHemisson connection settings from parsed configuation. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = HemiIniDD.IniDDSectConn
    optDict = iniDD[section][1]

    self.mConn = {}

    for option in optDict:
      self.mConn[option] = ini.IniGet(section, option)

  #--
  def IniInitProximity(self):
    """ Initialized vHemisson proximity sensor data from parsed 
        configuration.
    """
    ini         = self.GSGetIni()
    section     = HemiIniDD.IniDDSectProximity

    options = ini.IniGetItems(section)
    calParams = {}

    for option,value in options:
      calParams[option] = value
    self.mCmd.ProximitySensorsSetCalParams(calParams)

  #--
  def IniInitAmbient(self):
    """ Initialized vHemisson ambient sensor data from parsed 
        configuration.
    """
    ini         = self.GSGetIni()
    section     = HemiIniDD.IniDDSectAmbient

    options = ini.IniGetItems(section)
    calParams = {}

    for option,value in options:
      calParams[option] = value
    self.mCmd.AmbientSensorsSetCalParams(calParams)

  #--
  def IniInitLinCam(self):
    """ Initialized linear camera settings from parsed configuration. """
    ini       = self.GSGetIni()
    section   = HemiIniDD.IniDDSectLinCam
    options   = ini.IniGetItems(section)
    settings  = utils.tuples2dict(options)

    if self.IsCommUp() and self.mModScan.has_key('linear_camera'):
      self.GSReportNormalStatus("Initializing Hemisson Linear Camera Module")
      self.mCmd.LinCamCmdSetExposureTime(settings['exposure'])
      self.mCmd.LinCamCmdSetQThreshold(settings['threshold'])
    settings.update(
        self.GuiWinModComCfg('UseLinCamSensor', 'linear_camera'), 
        grab='P')
    self.GSGuiWinUpdate('VizLinCam', 'cfg', **settings)

  #--
  def IniInitTts(self):
    """ Initialized text-to-speech settings from parsed configuration. """
    ini       = self.GSGetIni()
    section   = HemiIniDD.IniDDSectTts
    options   = ini.IniGetItems(section)
    settings  = utils.tuples2dict(options)

    # A work around for a TTS module bug. Changing the settings puts the TTS in 
    # a micky mouse mode. So don't change the TTS power-up settings unless the
    # user or the configuration ask to do it.
    if not self.mTtsHasChanged \
       and settings['gain'] == HemiTts.TtsSpeakerGainDft \
       and settings['pitch'] == HemiTts.TtsVoicePitchDft \
       and settings['rate'] == HemiTts.TtsVoiceRateDft:
      pass

    # configure the TTS module
    elif self.IsCommUp() and self.mModScan.has_key('tts'):
      self.GSReportNormalStatus(
          "Initializing Hemisson Text-To-Speech Module")
      self.mCmd.TtsCmdSetSpeakerGain(settings['gain'])
      self.mCmd.TtsCmdSetVoicePitch(settings['pitch'])
      self.mCmd.TtsCmdSetVoiceRate(settings['rate'])
      self.mTtsHasChanged = True

    # report to any registered window
    settings.update(self.GuiWinModComCfg('UseTtsEffector', 'tts'))
    self.GSGuiWinUpdate('VizTts', 'cfg', **settings)

  #--
  def IniInitUss(self):
    """ Initialized ultrasonic sensor settings from parsed configuration. """
    ini       = self.GSGetIni()
    section   = HemiIniDD.IniDDSectUss
    options   = ini.IniGetItems(section)
    settings  = utils.tuples2dict(options)

    if self.IsCommUp() and self.mModScan.has_key('uss'):
      self.GSReportNormalStatus(
          "Initializing Hemisson UltraSonic Sensor Module")
      self.mCmd.UssCmdSetUnits('C')
      self.mCmd.UssCmdSetMaxRange(settings['range']/10)
    settings.update(self.GuiWinModComCfg('UseUssSensor', 'uss'))
    self.GSGuiWinUpdate('VizUss', 'cfg', **settings)

  #--
  def IniInitSenseReact(self):
    """ Initialized sense-react operational settings from parsed
        configuration.
    """
    # smallest positive distance increment possible per execution cycle 
    mindist = (1 * self.mOpt['ExecCycle'])

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Sensor and Shadow Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ShadowInit(self):
    """ Initialize the shadowed data.

        The Shadow captures the sensory states of the (simulated) robot
        plus any derived states into a dictionary of data. This data can
        be readily accessed by the vRobot's reactive subsystem and by the
        upper layer vBrain functions.

        Return Value:
          None.
    """
    self.GSLock()

    # the shadow
    self.mShadow = {}

    # time 'sensor' group
    groupId = hvals.HemiSensorMimeTypeTime
    self.mShadow[groupId]          = {}
    self.mShadow[groupId]['dt_i']  = 0.0
    self.mShadow[groupId]['t_0']   = 0.0
    self.mShadow[groupId]['t_i']   = 0.0

    # proximity ir led sensors group
    groupId = hvals.HemiSensorMimeTypeProximity
    self.mShadow[groupId] = {}
    for ir in HemiBase.HemiIrSensorOrder:
      self.mShadow[groupId]['prox_'+ir] = (0, 0)

    # ambient ir led sensors group
    groupId = hvals.HemiSensorMimeTypeAmbient
    self.mShadow[groupId]    = {}
    for ir in HemiBase.HemiIrSensorOrder:
      self.mShadow[groupId]['amb_'+ir] = (0, 0)

    # speedometer sensors group
    groupId = hvals.HemiSensorMimeTypeSpeedometer
    self.mShadow[groupId]                      = {}
    self.mShadow[groupId]['speedometer_left']  = 0
    self.mShadow[groupId]['speedometer_right'] = 0
    self.mShadow[groupId]['pathspeed']         = 0.0

    # linear camera sensor group
    groupId = hvals.HemiSensorMimeTypeLinCam
    self.mShadow[groupId]                   = {}
    self.mShadow[groupId]['linear_camera']  = [0] * HemiLinCam.LinCamNumPixels

    # ultrasonic sensor group
    groupId = hvals.HemiSensorMimeTypeUss
    self.mShadow[groupId]        = {}
    self.mShadow[groupId]['uss'] = [0] * HemiUss.UssEchoMaxNum

    self.GSUnlock()

  #--
  def ShadowUpdate(self, groupId='all'):
    """ Automatic update of the shadow data group(s) from the current
        states of the robot. The vRobot automatically updates the shadow
        in it's execution cycle.  Ancillary threads (e.g. windows) and
        vBrain functions may also request updates. 

        Note: all shadow groups are updated to keep synchronicity.

        Parameters:
          groupId  - shadow group Id. (ignored)
        
        Return Value:
          None.
    """
    self.GSLock()

    # time 'sensor' group (always on)
    group = self.mShadow[hvals.HemiSensorMimeTypeTime]
    t_now = time.time()
    group['dt_i'] = t_now - group['t_i']
    group['t_i']  = t_now
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                        (hvals.HemiSensorMimeTypeTime, repr(group)))

    # speedometer sensors group
    if self.mOpt['UseSpeedometerSensors']:
      self.mShadow[hvals.HemiSensorMimeTypeSpeedometer] = \
          self.SenseSpeedometer()

    # proximity ir led sensors group
    if self.mOpt['UseProximitySensors']:
      self.mShadow[hvals.HemiSensorMimeTypeProximity] = self.SenseProximity()

    # ambient ir led sensors group
    if self.mOpt['UseAmbientSensors']:
      self.mShadow[hvals.HemiSensorMimeTypeAmbient] = self.SenseAmbient()

    # linear camera
    if self.mOpt['UseLinCamSensor']:
      self.mShadow[hvals.HemiSensorMimeTypeLinCam]['linear_camera'] = \
          self.SenseLinCam()

    # ultrasonic sensor 
    if self.mOpt['UseUssSensor']:
      self.mShadow[hvals.HemiSensorMimeTypeUss]['uss'] = self.SenseUss()

    self.GSUnlock()

  #--
  def SenseSpeedometer(self):
    """ Read the speedometer data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of read speedometer values
        for each motor.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    readData = None
    if self.IsCommUp(): # communication must be up and the module attached
      readData = self.mCmd.CmdGetSpeed()
    if readData is not None:  # read the data
      sensorData['speedometer_left']  = readData[0]
      sensorData['speedometer_right'] = readData[1]
      sensorData['pathspeed'] = (readData[0] + readData[1]) / 2.0
    else:   # data dropout - fill with default values
      sensorData['speedometer_left']  = 0
      sensorData['speedometer_right'] = 0
      sensorData['pathspeed']         = 0.0
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (hvals.HemiSensorMimeTypeSpeedometer, repr(sensorData)))
    return sensorData

  #--
  def SenseProximity(self):
    """ Read the Proximity IR Sensor data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of sensed proximity sensor values
        where each datum is a 2-tuple (<calib>, <raw>) of calibrated and
        raw data. 

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    if self.IsCommUp(): # communication must be up and the module attached
      sensorData = self.mCmd.CmdReadProximitySensors(incraw=True)
    if not sensorData:  # data dropout - fill with default values
      sensorData = {}
      for ir in HemiBase.HemiIrSensorOrder:
        sensorData['prox_'+ir] = (0, 0)
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (hvals.HemiSensorMimeTypeProximity, repr(sensorData)))
    return sensorData

  #--
  def SenseAmbient(self):
    """ Read the Ambient IR Sensor data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of sensed ambient sensor values
        where each datum is a 2-tuple (<calib>, <raw>) of calibrated and
        raw data. 

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    if self.IsCommUp(): # communication must be up and the module attached
      sensorData = self.mCmd.CmdReadAmbientSensors(incraw=True)
    if not sensorData:  # data dropout - fill with default values
      sensorData = {}
      for ir in HemiBase.HemiIrSensorOrder:
        sensorData['amb_'+ir] = (0, 0)
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (hvals.HemiSensorMimeTypeAmbient, repr(sensorData)))
    return sensorData

  #--
  def SenseLinCam(self):
    """ Read the Linear Camera Sensor data. The read is only attempted if
        communication to the robot is up and the LinCam module is attached.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          List of read linear camera sensor pixels on success,
          empty list of failure.
    """
    # communication must be up and the module attached
    if not self.IsCommUp() or not self.mModScan.has_key('linear_camera'):
      return []
    sensorData = self.mCmd.LinCamCmdGrabPPixels()
    if sensorData is None:
      sensorData = []
    # update any registered window
    self.GSGuiWinUpdate('VizLinCam', 'pixels', sensorData)
    if __debug__:
      self.mDbg.d4print(' %s: %s:' % \
          (hvals.HemiSensorMimeTypeLinCam, sensorData))
    return sensorData

  #--
  def SenseUss(self):
    """ Read the UltraSonic Sensor data. The read is only attempted if
        communication to the robot is up and the USS module is attached.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          List of read USS sensor echoes on success, empty list of failure.
          All distances are in mm.
    """
    # communication must be up and the module attached
    if not self.IsCommUp() or not self.mModScan.has_key('uss'):
      return []
    self.mCmd.UssCmdTakeMeasurement()         # make a measurement
    sensorData = self.mCmd.UssCmdGetEchoSet() # read the sensor
    if sensorData is not None:
      n = len(sensorData)
      i = 0
      while i < n:
        sensorData[i] *= 10.0 # returned data is in cm, convert to mm
        i += 1
    else:
      sensorData = []
    # update any registered window
    self.GSGuiWinUpdate('VizUss', 'pings', sensorData)
    if __debug__:
      self.mDbg.d4print(' %s: %s:' % (hvals.HemiSensorMimeTypeUss, sensorData))
    return sensorData


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Effector Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def EffectSay(self, spokenText, writtenText=None, writtenColor='black'):
    """ Say the given  text.

        Parameters:
          spokenText    - text to say with the TTS
          writtenText   - text to display. Default is spokenText.
          writtenColor  - color of display text. Default is black.

        Return Value:
          None
    """
    if self.IsCommUp() and self.mModScan.has_key('tts'):
      # speak if tts is free
      if self.mCmd.TtsCmdQueryState() == HemiTts.TtsStateNotSpeaking:
        self.mCmd.TtsCmdSay(spokenText)
      # update any registered window
      if not writtenText:
        writtenText = spokenText
      self.GSGuiWinUpdate('VizTts', 'tts', text=writtenText,
          colortag=writtenColor)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot (Cere)Bellum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BellumSetGoals(self, **kwGoals):
    """ Set new robot low-level goals. The old goal set is replaced by
        this new goal set.

        Parameters:
          kwGoals   - argument list of keyword=value goals.
           Keywords:
             'speed_left'  - track to new left motor speed (unitless)
             'speed_right' - track to new right motor speed (unitless)
        
        Return Value:
          Returns new current goal set. 
    """
    for k, v in kwGoals.iteritems():
      if k == 'speed_left':
        speedLeft = v
        if speedLeft < HemiBase.HemiSpeedBackwardMax:
          speedLeft = HemiBase.HemiSpeedBackwardMax
        elif speedLeft > HemiBase.HemiSpeedForwardMax:
          speedLeft = HemiBase.HemiSpeedForwardMax
        self.mBellumGoalSet['speed_left'] = speedLeft
      elif k == 'speed_right':
        speedRight = v
        if speedRight < HemiBase.HemiSpeedBackwardMax:
          speedRight = HemiBase.HemiSpeedBackwardMax
        elif speedRight > HemiBase.HemiSpeedForwardMax:
          speedRight = HemiBase.HemiSpeedForwardMax
        self.mBellumGoalSet['speed_right'] = speedRight
    return self.mBellumGoalSet

  #--
  def BellumNullGoals(self):
    """ Set the vHemisson's low-level goals to the null set
        (i.e. resting state).

        Return Value:
          Returns new current goal set.
    """
    self.mBellumGoalSet['speed_left']  = 0
    self.mBellumGoalSet['speed_right'] = 0
    return self.mBellumGoalSet


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the physical robot.

        Callback on vRobotThreaded emergency stopping event.

        Return Value:
          None
    """
    maxTries  = 5
    tries     = 0
    while tries < maxTries:
      if self.mCmd.CmdStop():
        if __debug__: self.mDbg.d2print(
            'Emergency Stop of Hemisson robot succeeded')
        return
      tries += 1
    if __debug__:
      self.mDbg.d2print('Emergency Stop of Hemisson robot Failed: %s' % \
          self.mCmd.GetErrStr())

  #--
  def DoLoadInit(self):
    """ Load vRobot Initializer.

        Callback on vRobotThreaded loading event.

        Return Value:
          None
    """
    vRobotThreaded.vRobotThreaded.DoLoadInit(self)

    # possibly auto connect
    if not self.IsCommUp() and self.mOpt['AutoConnect']:
      if self.mConn['port'] and self.mConn['baudrate']:
        self.pRobotConnect(self.mConn['port'], self.mConn['baudrate'])


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Sense-React Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SenseReactInit(self):
    """ Initialize Sense-React Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vRobotThreaded.vRobotThreaded.SenseReactInit(self)

    # start 'time' sensor
    group = self.mShadow['sensor/time']
    group['t_0']  = group['t_i'] = time.time()  # time 0
    group['dt_i'] = 0.0                         # delta time

    # speed shadow group
    self.mSpeedGroup = self.mShadow[hvals.HemiSensorMimeTypeSpeedometer]

  #--
  def SenseReact(self):
    """ Sense-React Behavior. Track to current set of goals.
        Goals are speed.

        Execution Context: Bot thread
    """
    # on hold
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('SenseReact')

    # shadow robot sensors
    self.ShadowUpdate()

    # Nomenclature:
    #   subscripts: c = current state, g = goal target, l = left, r = right

    # current direction and speeds
    speed_l_c = self.mSpeedGroup['speedometer_left']
    speed_r_c = self.mSpeedGroup['speedometer_right']

    # goal set
    speed_l_g = self.mBellumGoalSet['speed_left']
    speed_r_g = self.mBellumGoalSet['speed_right']

    if speed_l_c != speed_l_g or speed_r_c != speed_r_g:
      self.mCmd.CmdSetSpeed(speed_l_g, speed_r_g)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Physical Robot Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def pRobotInit(self):
    """ Place the physical Hemisson into a known, initialized state.

        Return Value:
          None.
    """
    if not self.IsCommUp():
      self.GSReportErrorStatus("Cannot initialize the Hemisson: no connection")
      return
    self.mCmd.CmdStop()
    self.pRobotModScan()

  #--
  def pRobotConnect(self, port, baudrate=115200, ):
    """ Connect to the Hemisson robot.

        Parameters:
          port      - serial connection port
          baudrate  - serial connection baudrate

        Return Value:
          None
    """
    self.GSReportNormalStatus("Opening serial connection %s %d-%d-%s-%d..." % \
                     (port, baudrate, 8, 'N', 1))
    try:
      self.mCmd.Open(port, baudrate=baudrate)
      self.SetCommStatus(True)
    except IOError, err:
      s = "%s" % err
      self.GSReportErrorStatus(s)
      self.SetCommStatus(False)
    if self.IsCommUp():
      self.mCmd.FlushInput()
      self.pRobotModScan()

  #--
  def pRobotModScan(self):
    """ Scan for attached modules when communication is established or on 
        demand.

        Return Value:
          None.
    """
    if self.IsCommUp():
      if __debug__:
        self.mDbg.d2print('Scanning for attached modules')
      scan = self.mCmd.CmdScanForModules()
      if scan:
        self.mModScan = {}
        for scaninfo in scan:
          mid = scaninfo[1]
          if   mid == 'L': id = 'linear_camera'
          elif mid == 'T': id = 'tts'
          elif mid == 'U': id = 'uss'
          else: id = 'unknown'
          self.mModScan[id] = {
            'modname':  scaninfo[0],
            'mid':      scaninfo[1],
            'i2c_addr': scaninfo[2],
            'ver':      scaninfo[3],
          }
        for mod, modinfo in self.mModScan.iteritems():
          if mod == 'linear_camera':
            self.IniInitLinCam()
          elif mod == 'tts':
            self.IniInitTts()
          elif mod == 'uss':
            self.IniInitUss()
    else:
      self.mModScan = {}
    for winId, modName in [
        ('VizLinCam', 'linear_camera'), ('VizTts', 'tts'), ('VizUss', 'uss')]:
      if self.mModScan.has_key(modName):
        state = 'detected'
      else:
        state = 'not detected'
      self.GSGuiWinUpdate(winId, 'cfg', module=state)

  #--
  def pRobotDisconnect(self):
    """ Disconnnect communicaton channel with the Hemisson. The Hemisson
        is stopped prior to closing the channel.

        Return Value:
          None.
    """
    if self.IsCommUp():
      self.mCmd.CmdStop()
      self.mCmd.Close()
      self.SetCommStatus(False)
    self.mModScan = {}

  #--
  def pRobotHold(self):
    """ Hold position of physical robot.

        Return Value:
          None.
    """
    if self.IsCommUp():
      self.mCmd.CmdStop()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vHemisson Gui and Support
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize vHemisson GUI. """
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()

    self.mMenuBarList.add('Robot|Robot Options...', 'command', 
        callback=self.GuiCbRobotOptions)
    self.mMenuBarList.add('Robot|Module Options|Linear Camera...', 'command', 
        callback=self.GuiCbRobotOptModLinCam)
    self.mMenuBarList.add('Robot|Module Options|Text-To-Speech...', 'command', 
        callback=self.GuiCbRobotOptModTts)
    self.mMenuBarList.add('Robot|Module Options|UltraSonic Sensor...',
        'command', 
        callback=self.GuiCbRobotOptModUss)

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Connect...', 'command', 
        callback=self.GuiCbRobotConnect,
        disabledStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]})
    self.mMenuBarList.add('Robot|Disconnect', 'command', 
        callback=self.GuiCbRobotDisconnect,
        disabledStates={self.mServerId:[
          Gluon.EServerState.NotLoaded,
          Gluon.EServerState.NotReady
        ]})

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Module Scan...', 'command', 
        callback=self.GuiCbRobotModScan)
    self.mMenuBarList.add('Robot|Calibration|Proximity...', 'command', 
        callback=self.GuiCbRobotCalProximity)
    self.mMenuBarList.add('Robot|Calibration|Ambient...', 'command', 
        callback=self.GuiCbRobotCalAmbient)

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Robot Visualizer', 'command', 
        callback=self.GuiCbRobotViz)
    self.mMenuBarList.add('Robot|Module Visualizer|Linear Camera',
        'command', callback=self.GuiCbRobotVizLinCam)
    self.mMenuBarList.add('Robot|Module Visualizer|Text-To-Speech',
        'command', callback=self.GuiCbRobotVizTts)
    self.mMenuBarList.add('Robot|Module Visualizer|UltraSonic Sensor',
        'command', callback=self.GuiCbRobotVizUss)
    self.mMenuBarList.add('Robot|Shell', 'command', 
        callback=self.GuiCbRobotShell)

    self.mMenuBarList.add('Help|About Robot...', 'command', 
        callback=self.GuiCbHelpAbout)

    # Plugin Toolbar list
    self.mToolBarList = Gluon.GluonToolBarList()
    self.mToolBarList.add('connect',
        self.GuiCbRobotConnect, 
        tooltip='Establish a serial connection with the Hemisson',
        imagefile=gut.GetFusionImageFileName('SerConn.gif'),
        altStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]},
        alttooltip='Disconnect serial connection to the Hemisson',
        altimagefile=gut.GetFusionImageFileName('SerDisc.gif'))

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiWinModComCfg(self, runtimeKey, modName):
    """ Common configuration settings for Module Windows.

        Parameters:
          runtimeKey  - mOpt run-time key specific to the module.
          modName     - mModScan key specific to module

        Return Value:
          {'run_time':<val>, 'module':<val>}
    """
    comcfg = {}
    if runtimeKey == 'UseTtsEffector':  # always enabled
      comcfg['run_time'] = 'enabled'
    elif self.mOpt[runtimeKey]:
      comcfg['run_time'] = 'enabled'
    else:
      comcfg['run_time'] = 'disabled'
    if self.mModScan.has_key(modName):
      comcfg['module'] = 'detected'
    else:
      comcfg['module'] = 'not detected'
    return comcfg
    
  #--
  def GuiCbRobotOptions(self):
    """ 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Options')
    
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectOpts
    settingNames  = GuiDlgHemiOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiOpt.GuiDlgHemiOpt(parent, lastSettings=lastSettings)

    # options have been okay'ed
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
  def GuiCbRobotOptModLinCam(self):
    """ 'LinCam Module Options' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Module Options|Linear Camera')

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectLinCam
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiModLinCam.GuiDlgHemiModLinCam(parent,
                    lastSettings=lastSettings)

    # options have been okay'ed
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitLinCam()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotOptModTts(self):
    """ 'TTS Module Options' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Module Options|Text-To-Speech')

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectTts
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiModTts.GuiDlgHemiModTts(parent, lastSettings=lastSettings)

    # options have been okay'ed
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitTts()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotOptModUss(self):
    """ 'USS Module Options' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Module Options|UltraSonic Sensor')

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectUss
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiModUss.GuiDlgHemiModUss(parent, lastSettings=lastSettings)

    # options have been okay'ed
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitUss()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotConnect(self):
    """ 'Robot|Connect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Connect')
    
    # RDK!!! hack until ToolBar support SM
    if self.IsCommUp():
      self.GuiCbRobotDisconnect()
      return

    portHistory   = []
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini     = self.GSGetIni()
    section = HemiIniDD.IniDDSectConn
    optList = ini.IniGetReItems(section, 'port[0-9]+')
    for opt,val in optList:
      portHistory += [val]
    portHistory.sort()
    settingNames = GuiDlgSerConn.GetSettingNames()
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)

    # get Hemisson serial port supported values 
    listBaudRates = self.mCmd.GetSupportedBaudRates()
    listByteSizes = self.mCmd.GetSupportedByteSizes()
    listParities  = self.mCmd.GetSupportedParities()
    listStopBits  = self.mCmd.GetSupportedStopBits()

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgSerConn.GuiDlgSerConn(parent, self.mCmd.Open,
                                      portHistory=portHistory,
                                      lastSettings=lastSettings,
                                      validBaudRates=listBaudRates,
                                      validByteSizes=listByteSizes,
                                      validParities=listParities,
                                      validStopBits=listStopBits)

    # Serial connection has been successfully opened
    if dlg.result: 

      curOpened = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(curOpened)
      ini.IniSetModifiedItems(section, iniSettings)

      # update port history
      isNewPort = True
      for val in portHistory:
        if val == curOpened['port']:
          isNewPort = False
          break
      if isNewPort:
        ini.IniRemoveReOptions(section, 'port[0-9]+')
        i = 1
        for val in portHistory:
          ini.IniSet(section, "port%d" % i, val)
          i += 1
        ini.IniSet(section, "port%d" % i, curOpened['port'])

      # Re-init settings
      self.IniInitConn()

      # Set new communication status
      self.SetCommStatus(True)

      # flush any residuals
      self.mCmd.FlushInput()

      # Scan for modules, setting data a necessary
      self.pRobotModScan()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotDisconnect(self):
    """ 'Disonnect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Disconnect')
    self.SetCommStatus(False)
    self.mCmd.Close()

  #--
  def GuiCbRobotModScan(self):
    """ 'Module Scan' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Modules|Scan')

    self.pRobotModScan()

    #[('Text-To-Speech', 'T', 196, 6), ('UltraSonic Sensor', 'U', 224, 8)]
    hdr = 'Module                   Id    I2C   Version\n' + \
          '------                   --   ----   -------\n'
    txt = ''
    for modinfo in self.mModScan.itervalues():
      txt += modinfo['modname']
      sp = 26 - len(modinfo['modname'])
      txt += '%*s' %(sp, modinfo['mid'])
      sp = 4
      txt += '%*s0x%02x' %(sp, '', modinfo['i2c_addr'])
      sp = 5
      txt += '%*sv%d\n' %(sp, '', modinfo['ver'])
    if not txt:
      txt = 'Module scan is unavailable.\n'

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName() + ' Attached I' + \
                         gt.UniSuperscript['2'] + 'C Modules',
                    desc=hdr+txt)

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotCalProximity(self):
    """ 'Proximity Sensor Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Calibration|Proximity')

    sensorMimeType  = hvals.HemiSensorMimeTypeProximity
    sensorName      = 'Proximity'

    # get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectProximity
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)
    factSettings  = {
      'KBrightnessMin': HemiBase.HemiIrProxMinKBrightness,
      'KBrightnessMax': HemiBase.HemiIrProxMaxKBrightness,
      'KBrightnessDft': HemiBase.HemiIrProxDftKBrightness,
      'NoiseFloorMin': HemiBase.HemiIrProxMinNoiseFloor,
      'NoiseFloorMax': HemiBase.HemiIrProxMaxNoiseFloor,
      'NoiseFloorDft': HemiBase.HemiIrProxDftNoiseFloor,
      'MaxDist': HemiBase.HemiIrProxMaxDist * 2   # for graphing
    }

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiCalIrLed.GuiDlgHemiCalIrLed(parent, 
                                  sensorMimeType,
                                  sensorName,
                                  self.mCmd.ProximitySensorGetDftCalibration,
                                  factSettings,
                                  lastSettings)

    # options have been okay'ed
    if dlg.result: 

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(dlg.result)
      ini.IniSetModifiedItems(section, iniSettings)

      # re-init settings
      self.IniInitProximity()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotCalAmbient(self):
    """ 'Ambient Sensor Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Calibration|Ambient')

    sensorMimeType  = hvals.HemiSensorMimeTypeAmbient
    sensorName      = 'Ambient'

    # get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = HemiIniDD.IniDDSectAmbient
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)
    factSettings  = {
      'KBrightnessMin': HemiBase.HemiIrAmbMinKBrightness,
      'KBrightnessMax': HemiBase.HemiIrAmbMaxKBrightness,
      'KBrightnessDft': HemiBase.HemiIrAmbDftKBrightness,
      'NoiseFloorMin': HemiBase.HemiIrAmbMinNoiseFloor,
      'NoiseFloorMax': HemiBase.HemiIrAmbMaxNoiseFloor,
      'NoiseFloorDft': HemiBase.HemiIrAmbDftNoiseFloor,
      'MaxDist': HemiBase.HemiIrAmbMaxDist   # for graphing
    }

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgHemiCalIrLed.GuiDlgHemiCalIrLed(parent, 
                                  sensorMimeType,
                                  sensorName,
                                  self.mCmd.AmbientSensorGetDftCalibration,
                                  factSettings,
                                  lastSettings)

    # options have been okay'ed
    if dlg.result: 

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(dlg.result)
      ini.IniSetModifiedItems(section, iniSettings)

      # re-init settings
      self.IniInitAmbient()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotViz(self):
    """ 'Visualize Robot' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Robot Visualizer')
    msgbox.WarningBox('Not implemented yet.')

  #--
  def GuiCbRobotVizLinCam(self):
    """ 'Visualize Linear Camera Module' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Module Vizualizer|Linear Camers')
    win = self.GSGuiWinStart('VizLinCam',         # window ID
        GuiWinHemiVizLinCam.GuiWinHemiVizLinCam,  # start object
        sense_lincam=self.SenseLinCam)            # arguments to start
    ini   = self.GSGetIni()
    settings = utils.tuples2dict(ini.IniGetItems(HemiIniDD.IniDDSectLinCam))
    settings.update(
        self.GuiWinModComCfg('UseLinCamSensor', 'linear_camera'), 
        grab='P')
    self.GSGuiWinUpdate('VizLinCam', 'cfg', **settings)

  #--
  def GuiCbRobotVizTts(self):
    """ 'Visualize TTS Module' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Module Vizualizer|Text-To-Speech')
    win = self.GSGuiWinStart('VizTts',          # window ID
        GuiWinHemiVizTts.GuiWinHemiVizTts,      # start object
        effect_tts=self.EffectSay)              # arguments to start
    ini   = self.GSGetIni()
    settings = utils.tuples2dict(ini.IniGetItems(HemiIniDD.IniDDSectTts))
    settings.update(self.GuiWinModComCfg('UseTtsEffector', 'tts'))
    self.GSGuiWinUpdate('VizTts', 'cfg', **settings)

  #--
  def GuiCbRobotVizUss(self):
    """ 'Visualize USS Module' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Module Vizualizer|UltraSonic Sensor')
    win = self.GSGuiWinStart('VizUss',          # window ID
        GuiWinHemiVizUss.GuiWinHemiVizUss,      # start object
        sense_uss=self.SenseUss)                # arguments to start
    ini   = self.GSGetIni()
    settings = utils.tuples2dict(ini.IniGetItems(HemiIniDD.IniDDSectUss))
    settings.update(self.GuiWinModComCfg('UseUssSensor', 'uss'))
    self.GSGuiWinUpdate('VizUss', 'cfg', **settings)

  #--
  def GuiCbRobotShell(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Shell')
    self.GSGuiWinStart('Shell',                     # window ID
                       GuiWinShell.GuiWinShell,     # start object
                       HemiCmdShell.HemiFullShell,  # arguments to start
                       title='vHemisson Command Shell',
                       robotCmds=self.mCmd)

  #--
  def GuiCbHelpAbout(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Robot')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = self.HasName() + ' v' + self.IsVersion() + '\n' + 'Hemisson v' + \
              self.IsRobotVersion()
    imageFile = gut.GetFusionImageFileName(gt.ImageHemisson)

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='RoadNarrows vHemisson vRobot Demo',
                    desc="""\
vHemisson is a virtual Robot (vRobot) that interfaces with the Hemisson
physical robot. vHemisson provides full access to its built in sensors and
effectors, plus all supported I""" + gt.UniSuperscript['2'] + """C attached modules.

vHemisson provides the full set of Hemisson commands, plus movement goal
tracking.

Since the Hemisson does not have odometers or other position sensors, this
vHemisson only goal is to track to speed.
 * speed_left  - track to left motor absolute speed (unitless)
 * speed_right - track to right motor absolute speed (unitless)""",
                    copyright='RoadNarrows LLC\n(C) 2006',
                    logoImage=imageFile)

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vHemisson unit test environment. """
    k = vHemisson(debuglevel=level)
    port = '/dev/rfcomm0'
    print "Opening port %s..." % port
    k.mCmd.Open(port)
    k.SetCommStatus(True)
    print "Port %s opened" % port
    k.ExecLoad()
    return k
  
  def tstShortRun(k, sec=10):
    """ Short run test """
    iv = 0.1
    i = iv
    print k.BellumGetGoals();
    state = k.GetRobotState()
    if state == Gluon.EServerState.Ready:
      k.ExecStart()
      k.BellumSetGoals(speed_left=5, speed_right=3) # turn slowly right
    while i < sec:
      print k.ShadowGet(hvals.HemiSensorMimeTypeSpeedometer)
      time.sleep(iv)
      i += iv
    print k.ShadowGet(hvals.HemiSensorMimeTypeSpeedometer)
    k.ExecUnload()
  
  def main():
    """ vHemisson Unit Test Main """
    k = tstCreate()
    tstShortRun(k, 10)

  # run unit test
  main()
