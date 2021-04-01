################################################################################
#
# HemiCmdBase.py
#

""" Hemisson Base Command Module

Hemisson base serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.16

Copyright (C) 2004, 2005, 2006.  RoadNarrows LLC.
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

import re

import Fusion.Hemisson.Cmd.HemiSerial as HemiSerial
import Fusion.Hemisson.Cmd._hemiUtils as hutil


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Physical Properties
#
HemiBaseDiameter      = 120.0  # Base diameter in mm
HemiBaseWeight        = 200.0  # Base weight in g
HemiWheelBase         = 112.0  # Wheel base in mm

#
# Robot Limits
#
HemiSpeedForwardMax   =  9    # Maximum forward motor speed
HemiSpeedBackwardMax  = -9    # Maximum backward motor speed
HemiStateOff          =  0    # General off state
HemiStateOn           =  1    # General on state

# Module ID to Module Name Dictionary
HemiModuleNamesDict = { 
  '2':'BasicStamp II',
  'G':'General I/O',
  'D':'LCD',
  'L':'Linear Camera',
  'T':'Text-To-Speech',
  'U':'UltraSonic Sensor'
}

# Proximity/Ambient Sensors Read-Out Order
HemiIrSensorOrder = [
  'front', 'frontright', 'frontleft', 'right', 'left', 'rear', 
  'groundright', 'groundleft'
]

#
# Sensor pointing direction (degrees) counterclockwise from front (0 degrees).
# List is in read-out order.
#
# zeta (degrees)
HemiIrSensorAngle    = [0.0, 315.0, 45.0, 270.0, 90.0, 180.0, 0.0, 0.0]
HemiIrSensorNumOf    = len(HemiIrSensorOrder) # Number of IR sensors
HemiIrSensorMax      = HemiIrSensorNumOf - 1  # Maximum IR sensor number
HemiIrSensorValMin   =    0   # Minimum IR sensor read-out value
HemiIrSensorValMax   =  255   # Maximum IR sensor read-out value
HemiIrSensorAngRange =   30   # Sensor angular range (degrees)
HemiIrSensorInfDist  = 1000.0 # infinite distance for these sensors

#
# Proximity Sensors' Effective (real) operating ranges
#
HemiIrProxMinDist    =   5.0 # Minimum 'real' effective operating distance mm
HemiIrProxMaxDist    = 100.0 # Maximum 'real' effective operating distance mm

#
# Proximity IR LED Reflected Light Default Calibarion Data
# 
# This data is a typical, average response.  Units are (8-bit ADC value, mm).
# (See Hemisson User Manual, section 3.2.6)
#
HemiIrProxDftCal = [
  (HemiIrSensorValMax, HemiIrProxMinDist),
  (50, 20.0), (20, 30.0), (10, 40.0), (5, 50.0), (4, 60.0),
  (3, 70.0), (2, 80.0),
  (HemiIrSensorValMin, HemiIrSensorInfDist)
]

#
# Proximity Sensors' ADC Noise Floor
#
HemiIrProxMinNoiseFloor  =  0   # minimum (above noise floor) sensor value
HemiIrProxMaxNoiseFloor  = 25   # maximum (above noise floor) sensor value
HemiIrProxDftNoiseFloor  =  1   # default (above noise floor) sensor value

#
# Proximity Sensors' Brightness Ratio
#
# The ratio of the average brightness of objects in the environment under
# operating ambient light conditions to the brightness of typical, 
# a well-lit setting.
#
HemiIrProxMinKBrightness = 0.01  # minimum k
HemiIrProxMaxKBrightness = 1.0   # maximum k
HemiIrProxDftKBrightness = 1.0   # default k

#
# Ambients Sensors' Effective (real) operating ranges
#
HemiIrAmbMinDist    =   75.0 # maximum 'real' effective operating distance mm
HemiIrAmbMaxDist    = 1000.0 # maximum 'real' effective operating distance mm

#
# Ambient IR LED Reflected Light Default Calibarion Data
# 
# There are no data found to characterize these sensors in passive, ambient
# mode. The data used here are the scaled down data from the Khepera robot.
# Units are (8-bit ADC value, mm). 
# (See Hemisson User Manule, section 3.2.6 & Khepera User Manual, section 3.1.7)
#
HemiIrAmbDftCal = [
  (HemiIrSensorValMax, HemiIrSensorInfDist),
  (124, HemiIrSensorInfDist), (112, 900.0), (106, 750.0), (100, 550.0),
  (93, 400.0), (62, 200.0), (25, 100.0), (19, HemiIrAmbMinDist),
  (HemiIrSensorValMin, HemiIrAmbMinDist)
]

#
# Ambient Sensors' ADC Noise Floor
#
HemiIrAmbMinNoiseFloor  =   0   # minimum (above noise floor) sensor value
HemiIrAmbMaxNoiseFloor  =  25   # maximum (above noise floor) sensor value
HemiIrAmbDftNoiseFloor  =   1   # default (above noise floor) sensor value

#
# Ambient Sensors' Brightness Ratio
#
# The ratio of the average brightness of objects in the environment under
# operating ambient light conditions to the brightness of a typical, well-lit
# environment
#
HemiIrAmbMinKBrightness = 0.01 # minimum k
HemiIrAmbMaxKBrightness = 1.0  # maximum k
HemiIrAmbDftKBrightness = 1.0  # default k


#-------------------------------------------------------------------------------
# CLASS: HemiCmdBase
#-------------------------------------------------------------------------------
class HemiCmdBase(HemiSerial.HemiSerial):
  """ Hemisson Base Command and Response Class. """

  #--
  def __init__(self, port=None, dbgobj=None):
    """ Initialize a Hemisson serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the Hemisson
        serial port object will be in the closed state.

        Parameters:
          port      - serial port (default: no port)
          dbgobj    - PyDebug object. None will create the object.
    """
    HemiSerial.HemiSerial.__init__(self, port, dbgobj)

    self.Init()

  #--
  def Init(self):
    """ One time initialization during object instantiation. """

    # default IR LED proximity sensors calibration parameters
    self.ProximitySensorsSetDftCalParams()

    # default IR LED ambient sensors calibration parameters
    self.AmbientSensorsSetDftCalParams()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Serial Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdGetVersion(self):
    """ Get Hemisson Base OS version ('B').

        Return Value:
          Return version string on success, None on bad response.
    """
    rsp = self.SendCmd('B')
    if rsp is None: return None
    return self._ParseRspVersion(rsp)

  #--
  def CmdSetSpeed(self, motorLeft, motorRight):
    """ Set Hemisson Base speed ('D').
  
        Parameters:
          motorLeft   -  left motor speed [-9,9]
          motorRight  - right motor speed [-9,9]
  
        Return Value:
          New current speed tuple (motorLeft, motorRight) on success, 
          None on bad response.
    """
    motorLeft  = hutil.cap(motorLeft, HemiSpeedBackwardMax, HemiSpeedForwardMax)
    motorRight = hutil.cap(motorRight,HemiSpeedBackwardMax, HemiSpeedForwardMax)
    rsp = self.SendCmd('D,' + "%d" % motorLeft + ",%d" % motorRight)
    if rsp is None: return None
    return self._ParseRspSpeed(rsp)
  
  #--
  def CmdStop(self):
    """ Stop Hemisson Base movement ('D').
  
        Return Value:
          New current speed tuple (motorLeft, motorRight) on success, 
          None on bad response.
    """
    return self.CmdSetSpeed(0, 0)
   
  #--
  def CmdMoveForward(self, motor):
    """ Move Hemisson Base forward at the given speed ('D').
  
        Parameters:
          motor   - left and right motor speeds [1,9]
  
        Return Value:
          New current speed tuple (motorLeft, motorRight) on success, 
          None on bad response.
    """
    motor = hutil.cap(motor, 1, HemiSpeedForwardMax)
    return self.CmdSetSpeed(motor, motor)
   
  #--
  def CmdMoveBackward(self, motor):
    """ Move Hemisson Base backward at the given speed ('D').
  
        Parameters:
          motor   - left and right motor speeds [1,9]
  
        Return Value:
          New current speed tuple state (motorLeft, motorRight) on success, 
          None on bad response.
    """
    motor = hutil.cap(-motor, HemiSpeedBackwardMax, -1)
    return self.CmdSetSpeed(motor, motor)
   
  #--
  def CmdGetSpeed(self):
    """ Get Hemisson Base current speed ('E').
  
        Return Value:
          Current speed tuple (motorLeft, motorRight) on success, 
          None on bad response.
    """
    rsp = self.SendCmd('E')
    if rsp is None: return None
    return self._ParseRspSpeed(rsp)
  
  #--
  def CmdSetBeep(self, state):
    """ Set Hemisson Base annoying beep state (H).
  
        Parameters:
          state - beep state
                    0 = off
                    1 = on
  
        Return Value:
          New beep state on success, None on bad response.
    """
    state = hutil.cap(state, HemiStateOff, HemiStateOn)
    rsp = self.SendCmd('H' + ",%d" % state)
    if rsp is None: return None
    if len(rsp) != 3: return self.mErr.SetErrBadRsp(rsp)
    try:
      state = int(rsp[2:])
      return state
    except (SyntaxError, NameError, TypeError, ValueError):
      return self.mErr.SetErrBadRsp(rsp)
  
  #--
  def CmdBeep(self):
    """ Turn Hemisson Base annoying beeper on ('H').
  
        Return Value:
          New beep state on success, None on bad response.
    """
    return self.CmdSetBeep(HemiStateOn)
  
  #--
  def CmdBeepOff(self):
    """ Turn Hemisson Base annoying beeper off ('H').
  
        Return Value:
          New beep state on success, None on bad response.
    """
    return self.CmdSetBeep(HemiStateOff)
  
  #--
  def CmdScanForModules(self):
    """ Scan Hemisson Base I2C address space for all connected I2C capable 
        modules ('J').
  
        Return Value:
          List of 4-tuples (ModuleName, ModuleID, I2CAddr, VersionNum)
          for each discovered module.  List is empty if no modules found.
          None on bad response.
    """
    rsp = self.SendCmd('J')
    if rsp is None: return None
    return self._ParseRspI2CScan(rsp)
  
  #--
  def CmdSetBaseLedStates(self, ledLeft, ledRight):
    """ Set Hemisson Base user controllable LED states ('H').
  
        Parameters:
          ledLeft   -  left LED state. 0 = off, 1 = on
          ledRight  - right LED state. 0 = off, 1 = on
  
        Return Value:
          New LED state tuple (ledLeft, ledRight) on success, 
          None on bad response.
    """
    ledLeft  = hutil.cap(ledLeft,  HemiStateOff, HemiStateOn)
    ledRight = hutil.cap(ledRight, HemiStateOff, HemiStateOn)
    rsp = self.SendCmd('L,1,0' + ",%d" % ledLeft + ",%d" % ledRight)
    if rsp is None: return None
    return self._ParseRspLed(rsp)
  
  #--
  def CmdReadProximitySensors(self, incraw=False):
    """ Read Hemisson Base proximity sensors ('N').
  
        Parameters:
          incraw   - do [not] include raw values with return
 
        Return Value:
          Dictionary of labeled proximity sensor values on success. 
          Sensor value range = [0,255] with:
              0 = very far to 
            255 = very close
          None on bad response.
    """
    rsp = self.SendCmd('N')
    if rsp is None: return None
    if len(rsp) < 33: return self.mErr.SetErrBadRsp(rsp)
    sensorVals = hutil.cvtCsvIntStr(rsp[2:])
    sensorDict = {}
    n = 0
    for val in sensorVals:
      id = 'prox_' + HemiIrSensorOrder[n]
      dist = self.ProximitySensorDist(id, val)
      if not incraw:
        sensorDict[id] = dist
      else:
        sensorDict[id] = (dist, val)
      n += 1
    return sensorDict
  
  #--
  def CmdReadAmbientSensors(self, incraw=False):
    """ Read Hemisson Base ambient sensors ('O').
  
        Parameters:
          incraw   - do [not] include raw values with return
 
        Return Value:
          Dictionary of labeled ambient sensor values on success. 
          Sensor value range = [0,255] with: 
              0 = brightest infrared light to
            255 = no light
          None on bad response.
    """
    rsp = self.SendCmd('O')
    if rsp is None: return None
    if len(rsp) < 33: return self.mErr.SetErrBadRsp(rsp)
    sensorVals = hutil.cvtCsvIntStr(rsp[2:])
    sensorDict = {}
    n = 0
    for val in sensorVals:
      id = 'amb_' + HemiIrSensorOrder[n]
      dist = self.AmbientSensorDist(id, val)
      if not incraw:
        sensorDict[id] = dist
      else:
        sensorDict[id] = (dist, val)
      n += 1
    return sensorDict
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Proximity Sensor Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ProximitySensorsGetCalParams(self):
    """ Get the current set of IR LED proximity sensors' calibration
        parameters.

        Per each sensor:
          enabled       - sensor is [not] enabled.
          k_brightness  - brightness ratio. the value 1.0 equals the 
                          factory default of white paper in a well lit
                          environment.
          noise_floor   - the adc noise floor. any value below this is
                          considered unreadable noise.

        Return Values:
          Calibration parameter dictionary:
            { <sensorId>: 
                {'enabled':bool, 'k_brightness':float, 'noise_floor':int},
              ...
            }
    """
    return self.mIrProxCalParams

  #--
  def ProximitySensorsSetCalParams(self, newCal):
    """ Set the current proximity sensors' calibration parameters.

        Parameters:
          newCal - see ProximitySensorsGetCalParams()

        Return Value:
          None
    """
    for id, newParams in newCal.items():
      if id not in self.mIrProxCalParams: # bogus id
        continue
      sensor = self.mIrProxCalParams[id]
      sensor['enabled'] = newParams['enabled']
      sensor['k_brightness'] = newParams['k_brightness']
      if sensor['k_brightness'] < HemiIrProxMinKBrightness:
        sensor['k_brightness'] = HemiIrProxMinKBrightness
      elif sensor['k_brightness'] > HemiIrProxMaxKBrightness:
        sensor['k_brightness'] = HemiIrProxMaxKBrightness
      sensor['noise_floor'] = newParams['noise_floor']
      if sensor['noise_floor'] < HemiIrProxMinNoiseFloor:
        sensor['noise_floor'] = HemiIrProxMinNoiseFloor
      elif sensor['noise_floor'] > HemiIrProxMaxNoiseFloor:
        sensor['noise_floor'] = HemiIrProxMaxNoiseFloor

  #--
  def ProximitySensorsSetDftCalParams(self):
    """ Restores the current proximity sensors' calibration parameters
        to factory default.

        Return Value:
          None
    """
    self.mIrProxCalParams = {}
    for id in HemiIrSensorOrder:
      self.mIrProxCalParams['prox_'+id] = {
        'enabled': True,
        'k_brightness': HemiIrProxDftKBrightness,
        'noise_floor': HemiIrProxDftNoiseFloor
      }

  #--
  def ProximitySensorGetDftCalibration(self):
    """ Get the default calibration data for the proximity IR LED sensor. """
    return {'unitsX':'adc-8bit', 'unitsY':'mm',
            'calData':HemiIrProxDftCal}
 
  #--
  def ProximitySensorDist(self, sensorId, sensorVal):
    """ Map Proximity sensor value to distance in mm. Linear interpolation
        is used on the current (maybe default) sensor calibarion data.

        Parameters:
          sensorId  - sensor ID string
          sensorVal - read sensor value

        Return Value:
          Distance in mm.
    """
    calParams = self.mIrProxCalParams[sensorId]

    # sensor not enabled
    if not calParams['enabled']:
      return HemiIrSensorInfDist

    # below noise floor for this sensor
    elif sensorVal < calParams['noise_floor']:
      return HemiIrSensorInfDist

    # adjust sensor value
    sensorVal /= calParams['k_brightness']
    if sensorVal > HemiIrSensorValMax:
      sensorVal = HemiIrSensorValMax

    # interpolate distance
    caldata = HemiIrProxDftCal
    callen  = len(caldata)
    i = 0
    while i < callen: 
      if caldata[i][0] == sensorVal:
        return caldata[i][1]
      elif caldata[i][0] < sensorVal:
        break;
      i += 1
    j = i - 1
    return caldata[j][1] - float(caldata[j][0] - sensorVal) / \
                           float(caldata[j][0] - caldata[i][0]) * \
                           (caldata[j][1] - caldata[i][1])
  
   # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Ambient Sensor Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def AmbientSensorsGetCalParams(self):
    """ Get the current set of IR LED ambient sensors' calibration
        parameters.

        Per each sensor:
          enabled       - sensor is [not] enabled.
          k_brightness  - brightness ratio. the value 1.0 equals the 
                          factory default of a typical light source.
          noise_floor   - the adc noise floor. any value below this is
                          considered unreadable noise.

        Return Values:
          Calibration parameter dictionary:
            { <sensorId>: 
                {'enabled':bool, 'k_brightness':float, 'noise_floor':int},
              ...
            }
    """
    return self.mIrAmbCalParams

  #--
  def AmbientSensorsSetCalParams(self, newCal):
    """ Set the current ambient sensors' calibration parameters.

        Parameters:
          newCal - see AmbientSensorsGetCalParams()

        Return Value:
          None
    """
    for id, newParams in newCal.items():
      if id not in self.mIrAmbCalParams: # bogus id
        continue
      sensor = self.mIrAmbCalParams[id]
      sensor['enabled'] = newParams['enabled']
      sensor['k_brightness'] = newParams['k_brightness']
      if sensor['k_brightness'] < HemiIrAmbMinKBrightness:
        sensor['k_brightness'] = HemiIrAmbMinKBrightness
      elif sensor['k_brightness'] > HemiIrAmbMaxKBrightness:
        sensor['k_brightness'] = HemiIrAmbMaxKBrightness
      sensor['noise_floor'] = newParams['noise_floor']
      if sensor['noise_floor'] < HemiIrAmbMinNoiseFloor:
        sensor['noise_floor'] = HemiIrAmbMinNoiseFloor
      elif sensor['noise_floor'] > HemiIrAmbMaxNoiseFloor:
        sensor['noise_floor'] = HemiIrAmbMaxNoiseFloor

  #--
  def AmbientSensorsSetDftCalParams(self):
    """ Restores the current ambient sensors' calibration parameters
        to factory default.

        Return Value:
          None
    """
    self.mIrAmbCalParams = {}
    for id in HemiIrSensorOrder:
      self.mIrAmbCalParams['amb_'+id] = {
        'enabled': True,
        'k_brightness': HemiIrAmbDftKBrightness,
        'noise_floor': HemiIrAmbDftNoiseFloor
      }

  #--
  def AmbientSensorGetDftCalibration(self):
    """ Get the default calibration data for the ambient IR LED sensor. """
    return {'unitsX':'adc-8bit', 'unitsY':'mm',
            'calData': HemiIrAmbDftCal}
 
  #--
  def AmbientSensorDist(self, sensorId, sensorVal):
    """ Map Ambient sensor value to distance in mm. Linear interpolation
        is used on the current (maybe default) sensor calibarion data.

        Parameters:
          sensorId  - sensor ID string
          sensorVal - read sensor value

        Return Value:
          Distance in mm.
    """
    calParams = self.mIrAmbCalParams[sensorId]

    # sensor not enabled
    if not calParams['enabled']:
      return HemiIrSensorInfDist

    # below noise floor for this sensor
    elif sensorVal < calParams['noise_floor']:
      return HemiIrSensorInfDist

    # adjust sensor value
    sensorVal /= calParams['k_brightness']
    if sensorVal > HemiIrSensorValMax:
      sensorVal = HemiIrSensorValMax

    # interpolate distance
    caldata = HemiIrAmbDftCal
    callen  = len(caldata)
    i = 0
    while i < callen: 
      if caldata[i][0] == sensorVal:
        return caldata[i][1]
      elif caldata[i][0] < sensorVal:
        break;
      i += 1
    j = i - 1
    return caldata[j][1] - float(caldata[j][0] - sensorVal) / \
                           float(caldata[j][0] - caldata[i][0]) * \
                           (caldata[j][1] - caldata[i][1])

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  # Speed response regular expression
  _reRspSpeed = re.compile('([-]?\d+),([-]?\d+)');
  
  # LED states response regular expression
  _reRspLed = re.compile('[01],[01],([01]),([01])');
  
  # I2C scan response regular expression
  _reRspI2cScan = re.compile('(.),([0-9A-F][0-9A-F]),([0-9A-F][0-9A-F])')
  
  #--
  def _ParseRspVersion(self, rsp):
    """ Return version response string HemiOS_v_x.yy-special. """
    if len(rsp) < 3: return self.mErr.SetErrBadRsp(rsp)
    return rsp[2:]
  
  #--
  def _ParseRspSpeed(self, rsp):
    """ Return set speed response motorLeft, motorRight. """
    if len(rsp) < 5: return self.mErr.SetErrBadRsp(rsp)
    match = self._reRspSpeed.match(rsp[2:])
    if not match: return self.mErr.SetErrBadRsp(rsp)
    return int(match.group(1)), int(match.group(2))
  
  #--
  def _ParseRspLed(self, rsp):
    """ Return set led response ledLeft, ledRight. """
    if len(rsp) != 9: return self.mErr.SetErrBadRsp(rsp)
    match = self._reRspLed.match(rsp[2:])
    if not match: return self.mErr.SetErrBadRsp(rsp)
    return int(match.group(1)), int(match.group(2))
  
  #--
  def _ParseRspI2CScan(self, rsp):
    """ Return list of 4-tuples of scanned modules. """
    if len(rsp) < 1: return self.mErr.SetErrBadRsp(rsp)
    if len(rsp) == 1: return []
    i = 2
    scanlist = []
    while True:
      match = self._reRspI2cScan.match(rsp[i:])
      if not match: return scanlist
      if match.lastindex < 3: return self.mErr.SetErrBadRsp(rsp)
      modId   = match.group(1)
      i2cAddr = hutil.cvtHH(match.group(2))
      modVer  = hutil.cvtHH(match.group(3))
      if modId in HemiModuleNamesDict:
        modName = HemiModuleNamesDict[modId]
      else:
        modName = "Unknown Module"
      scanlist.append((modName, modId, i2cAddr, modVer))
      i += 8
