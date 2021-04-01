################################################################################
#
# KheCmdBase.py
#

""" Khepera II Base Command Module

Khepera II base serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.02

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

import re
import Fusion.Khepera.Cmd.KheSerial as KheSerial
import Fusion.Khepera.Cmd._kheCmdUtils as util


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Robot Physical Sizes
#
KheBaseDiameter         =  70.0  # Base diameter in mm
KheBaseHeight           =  30.0  # Base height in mm
KheBaseWeight           =  80.0  # Base weight in g
KheWheelBase            =  52.0  # Wheel base in mm
KhePayloadWeight        = 250.0  # Payload maximum weight in g

#
# Motor Speed
# 
# Each motor is controlled by Pulse Width Modulation (PWM). The raw units to the
# motor are pulses/10ms. For each pulse/10ms and addition 8 mm/s speed increase
# is sustained.
#
KheSpeedPp10msMax       =  127  # Maximum motor speed in pulses/10ms
KheSpeedMmpsPerPp10ms   =    8  # Speed conversion mm/s per pulse/10ms    
KheSpeedMmpsPosMax      = KheSpeedPp10msMax * KheSpeedMmpsPerPp10ms
                                # Maxiumum positive motor speed in mm/s
KheSpeedMmpsPosMin      = KheSpeedMmpsPerPp10ms
                                # Minimum positive motor speed in mm/s
KheSpeedMmpsRes         = KheSpeedMmpsPerPp10ms
                                # Speed resolution mm/s


#
# Motor Odometry
# 
# Each motor has an independent, 32-bit counter odometer. Each counter tick
# (odometer pulse) corresponds to 0.08mm in distance. The odometers can be set
# to a specific value. The counters will automatically roll-over after the
# maximum counter tick is reached.
#
KheOdometerTickMin      = -(2**31)  # Minimum odometer value in ticks
KheOdometerTickMax      = 2**31 - 1 # Maximum odometer value in ticks
KheOdometerMmpt         = 0.08      # Distance convsersion in mm/tick
KheOdometerMmMin        = KheOdometerTickMin * KheOdometerMmpt # Min in mm
KheOdometerMmMax        = KheOdometerTickMax * KheOdometerMmpt # Max in mm
KhePosTickMin           = -(2**23 - 2) # Minimum target distance in ticks and 
KhePosTickMax           = 2**23 - 2 # Maximum target distance in ticks that the
                                    # Khepera can be commanded to move to 
                                    # (+/-670m)
KhePosMmMin             = KhePosTickMin * KheOdometerMmpt # Min pos in mm
KhePosMmMax             = KhePosTickMax * KheOdometerMmpt # Max pos in mm

#
# Analog to Digital Converter (ADC)
#
# There are 6 ADC channels, 0-2 are used for functions in the base, while user
# channels 3-5 are accessible through the GenIO turret. Each channel has a
# 10-bit, linear, rail-to-rail ADC converting 0V - 4.096V.
# 
KheAdcVoltsMax          = 4.096 # Maximum analog volts before saturation
KheAdcNumBits           = 10    # Number of bits in analog to digital conversion
KheAdcMax               = 2**KheAdcNumBits - 1        # Maximum ADC value
KheAdcVpb               = KheAdcVoltsMax / KheAdcMax  # Volts/bit
KheAdcBpv               = KheAdcMax / KheAdcVoltsMax  # Bits/volt
KheAdcChanMin           = 0     # Minimum ADC channel
KheAdcChanMax           = 5     # Maximum ADC channel
KheAdcUserChanMin       = 3     # Minimum user ADC channel 
KheAdcUserChanMax       = 5     # Maximum user ADC channel 

#
# Base LEDs
#
# There are two user controllable LEDs
#
KheLedMin               = 0     # Minimum LED number
KheLedMax               = 1     # Maximum LED number

#
# General Data
#
KheStateOff             =    0  # General off state
KheStateOn              =    1  # General on state

#
# Infrared Sensors
#
# There are 8 Proximity/Ambient Infrared Light Sensors. Read-out order is 
# clockwise starting on the Khepera's left side. Each sensor has 10-bits of
# resolution. When used as an ambient light detectors, lower values indicates
# closer objects. When used as proximity detectors measuring reflected IR light
# from the IR emitter, closer values have higher values.
#

# Sensor read-out order from Khepera.
KheIrSensorOrder = [
  'frontleft90',  'frontleft45',  'frontleft10',
  'frontright90', 'frontright45', 'frontright10',
  'backright10',  'backleft10'
]

#
# Sensor pointing direction (degrees) counterclockwise from front (0 degrees).
# List is in read-out order.
#
# zeta (degrees)
KheIrSensorAngle    = [90.0, 45.0, 10.0, 270.0, 315.0, 350.0, 190.0, 170.0]
KheIrSensorNumOf    = len(KheIrSensorOrder) # Number of IR sensors
KheIrSensorMax      = KheIrSensorNumOf - 1  # Maximum IR sensor number
KheIrSensorValMin   =    0  # Minimum IR sensor read-out value
KheIrSensorValMax   = 1023  # Maximum IR sensor read-out value
KheIrSensorAngRange = 30    # Sensor angular range (degrees)
KheIrSensorInfDist  = 1000.0  # infinite distance for these sensors

#
# Proximity Sensors' Effective (real) operating ranges
#
KheIrProxMinDist    =   8.0 # Minimum 'real' effective operating distance mm
KheIrProxMaxDist    = 100.0 # Maximum 'real' effective operating distance mm

#
# Proximity IR LED Reflected Light Default Calibarion Data
# 
# This data is the result of placing a sheet of white paper a various 
# distances and reading the sensor results. Units are (10-bit ADC value, mm).
# (See Khepera User Manual, section 3.1.8)
#
KheIrProxDftCalWhiteSheet = [
  (KheIrSensorValMax, KheIrProxMinDist),
  (1000, 10.0), (450, 20.0), (275, 30.0), (190, 40.0), (150, 50.0),
  (125, 60.0), (110, 70.0), (100, 80.0), (90, 90.0), (80, 100.0),
  (KheIrSensorValMin, KheIrSensorInfDist)
]

#
# Proximity Sensors' ADC Noise Floor
#
KheIrProxMinNoiseFloor  =  10   # minimum (above noise floor) sensor value
KheIrProxMaxNoiseFloor  = 200   # maximum (above noise floor) sensor value
KheIrProxDftNoiseFloor  = 100   # default (above noise floor) sensor value

#
# Proximity Sensors' Brightness Ratio
#
# The ratio of the average brightness of objects in the environment under
# operating ambient light conditions to the brightness of white paper in 
# a well-lit setting. (See Khepera User Manual, section 3.1.8)
#
KheIrProxMinKBrightness = 0.01  # minimum k
KheIrProxMaxKBrightness = 1.0   # maximum k
KheIrProxDftKBrightness = 1.0   # default k

#
# Ambients Sensors' Effective (real) operating ranges
#
KheIrAmbMinDist    =   75.0 # maximum 'real' effective operating distance mm
KheIrAmbMaxDist    = 1000.0 # maximum 'real' effective operating distance mm

#
# Ambient IR LED Reflected Light Default Calibarion Data
# 
# This data is the result of placing a 50W light source a various distances
# above the Khepera robot.j Units are (10-bit ADC value, mm).
# (See Khepera User Manual, section 3.1.7)
#
KheIrAmbDftCal50WLight = [
  (KheIrSensorValMax, KheIrSensorInfDist),
  (500, KheIrSensorInfDist), (450, 900.0), (425, 750.0), (400, 550.0),
  (375, 400.0), (250, 200.0), (100, 100.0), (75, KheIrAmbMinDist),
  (KheIrSensorValMin, KheIrAmbMinDist)
]

#
# Ambient Sensors' ADC Noise Floor
#
KheIrAmbMinNoiseFloor  =  10   # minimum (above noise floor) sensor value
KheIrAmbMaxNoiseFloor  = 200   # maximum (above noise floor) sensor value
KheIrAmbDftNoiseFloor  =  80   # default (above noise floor) sensor value

#
# Ambient Sensors' Brightness Ratio
#
# The ratio of the average brightness of objects in the environment under
# operating ambient light conditions to the brightness of a 50W light source
# hanging above the Khepera. (See Khepera User Manual, section 3.1.7)
#
KheIrAmbMinKBrightness = 0.01 # minimum k
KheIrAmbMaxKBrightness = 1.0  # maximum k
KheIrAmbDftKBrightness = 1.0  # default k

#
# Turret Extensions
#
# The Khepera OS supports communication to some turrets via the 'T' command.
#
KheTidMin               =  0    # Minimum turret id
KheTidMax               = 31    # Maximum turret id
KheTidReservedMin       =  0    # Minimum turret id reserved by K-Team
KheTidReservedMax       = 23    # Maximum turret id reserved by K-Team
KheTidUserMin           = 24    # Minimum user available turret id
KheTidUserMax           = 31    # Maximum user available turret id
KheTidDict = {                  # Turret ID dictionary
  'gripper'   :  1,   # Gripper Turret
  'k213'      :  2,   # K213 Camera Turret
  'lasergps'  :  3,   # Laser GPS Turret
  'radiolink' :  4,   # Radio Link Turret
  'cmucam'    : 10    # CMU Camera Turret
}

#
# Extension Bus
#
# The Extension Bus supports read/write byte access to devices at 64 
# relative addresses.
#
KheExtBusAddrMin        =   0   # Minimum extension bus relative address
KheExtBusAddrMax        =  63   # Maximum extension bus relative address
KheExtBusAddrDict = {           # Extension bus address dictionary
  'genio': 32     # General I/O digital input/output address
}


#-------------------------------------------------------------------------------
# Khepera II Base Command Class
#-------------------------------------------------------------------------------
class KheCmdBase(KheSerial.KheSerial):
  """ Khepera II Base Command and Response Class. """

  #--
  def __init__(self, port=None, baudrate=9600, dbgobj=None):
    """ Initialize a Khepera serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the Khepera
        serial port object will be in the closed state.

        Parameters:
          port      - serial port (default: no port)
          baudrate  - baudrate (default: 9600)
          dbgobj    - PyDebug object. None will create the object.
    """
    KheSerial.KheSerial.__init__(self, port, baudrate, dbgobj)

    # default IR LED proximity sensors calibration parameters
    self.ProximitySensorsSetDftCalParams()

    # default IR LED ambient sensors calibration parameters
    self.AmbientSensorsSetDftCalParams()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Serial Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdGetVersion(self):
    """ Get Khepera Base OS version ('B').

        Return Value:
          Return version string tuple (verBios, verProtocol) on success,
          None on error.
    """
    rsp = self.SendCmd('B')
    if rsp is None: return None
    return self._ParseRspVersion(rsp)

  #--
  def CmdSetSpeed(self, motorLeft, motorRight):
    """ Set Khepera Base speed ('D').
  
        Parameters:
          motorLeft   -  left motor speed in mm/s [-1000,1000]
          motorRight  - right motor speed in mm/s [-1000,1000]
  
        Return Value:
          New current speed integer tuple (motorLeft, motorRight) in mm/s on
          success, None on error.
    """
    motorLeft  = util.cap(int(motorLeft/KheSpeedMmpsPerPp10ms),
                          -KheSpeedPp10msMax, KheSpeedPp10msMax)
    motorRight = util.cap(int(motorRight/KheSpeedMmpsPerPp10ms),
                          -KheSpeedPp10msMax, KheSpeedPp10msMax)
    rsp = self.SendCmd('D,%d,%d' % (motorLeft, motorRight))
    if rsp is None: return None
    return motorLeft * KheSpeedMmpsPerPp10ms, motorRight * KheSpeedMmpsPerPp10ms
  
  #--
  def GetNearestSpeed(self, motor):
    """ Determine the actual speed when the Khepera motor resolution
        is factored in.
  
        Parameters:
          motor   -  desired motor speed in mm/s [-1000,1000]
  
        Return Value:
          Nearest possible physical speed at motor resolution.
    """
    motor  = util.cap(int(motor/KheSpeedMmpsPerPp10ms),
                      -KheSpeedPp10msMax, KheSpeedPp10msMax)
    return motor * KheSpeedMmpsPerPp10ms

  #--
  def CmdStop(self):
    """ Stop Khepera Base movement ('D').
  
        Return Value:
          New current speed integer tuple (motorLeft, motorRight) n mm/s on
          success, None on error.
    """
    return self.CmdSetSpeed(0, 0)
   
  #--
  def CmdMoveForward(self, motor):
    """ Move Khepera Base forward at the given speed ('D').
  
        Parameters:
          motor   - left and right motor speeds [0,1000]
  
        Return Value:
          New current speed integer tuple (motorLeft, motorRight) in mm/s
          on success, None on error.
    """
    if motor < 0: motor = -motor
    return self.CmdSetSpeed(motor, motor)
   
  #--
  def CmdMoveBackward(self, motor):
    """ Move Khepera Base backward at the given speed ('D').
  
        Parameters:
          motor   - left and right motor speeds [0,1000]
  
        Return Value:
          New current speed integer tuple state (motorLeft, motorRight) in
          mm/s on success, None on error.
    """
    if motor < 0: motor = -motor
    return self.CmdSetSpeed(-motor, -motor)
   
  #--
  def CmdGetSpeed(self, incraw=False):
    """ Get the current speed of the Khepera Base ('E').
  
        Parameters:
          incraw   - do [not] include raw values with return
  
        Return Value:
          On success, return the current speed tuple,
            (motorLeft, motorRight [, rawLeft, rawRight])
            with motor<x> in mm/s, and raw<x> in pulses/10ms.
          On error, return None
    """
    rsp = self.SendCmd('E')
    if rsp is None: return None
    return self._ParseRspSpeed(rsp, incraw)
  
  #--
  def CmdMoveToPos(self, posLeft, posRight):
    """ Move Khepera to the given odometer positions ('C').
  
        Parameters:
          posLeft  -  left motor position in mm [0.0, 671088.0]
          posRight - right motor position in mm [0.0, 671088.0]
  
        Return Value:
          Target position float tuple (posLeft, posRight) in mm on success,
          None on error.
    """
    posLeft  = util.cap(int(posLeft/KheOdometerMmpt), 
                        KhePosTickMin, KhePosTickMax)
    posRight = util.cap(int(posRight/KheOdometerMmpt), 
                        KhePosTickMin, KhePosTickMax)
    rsp = self.SendCmd('C,%d,%d' % (posLeft, posRight))
    if rsp is None: return None
    return posLeft * KheOdometerMmpt, posRight * KheOdometerMmpt
  
  #--
  def CmdSetOdometry(self, odometerLeft, odometerRight, incraw=False):
    """ Set Khepera Odometry (G).
  
        Parameters:
          odometerLeft  -  left motor odometer position in mm 
                            [0.0, 343597384.0]
          odometerRight - right motor odometer position in mm 
                            [0.0, 343597384.0]
          incraw        - do [not] include raw values with return
  
        Return Value:
          On success, return the current position tuple,
            (odometerLeft, odometerRight [, rawLeft, rawRight])
            with odometer<x> in mm, and raw<x> in ticks.
          On error, return None
    """
    odometerLeft  = util.cap(int(odometerLeft/KheOdometerMmpt),
                          KheOdometerTickMin, KheOdometerTickMax)
    odometerRight = util.cap(int(odometerRight/KheOdometerMmpt),
                          KheOdometerTickMin, KheOdometerTickMax)
    rsp = self.SendCmd('G,%d,%d'  % (odometerLeft, odometerRight))
    if rsp is None: return None
    if incraw:
      return (odometerLeft * KheOdometerMmpt, odometerRight * KheOdometerMmpt,
            odometerLeft, odometerRight)
    else:
      return (odometerLeft * KheOdometerMmpt, odometerRight * KheOdometerMmpt)
  
  #--
  def CmdGetOdometry(self, incraw=False):
    """ Get Khepera Base odometry values ('H').
  
        Parameters:
          incraw   - do [not] include raw values with return
  
        Return Value:
          On success, return the current position tuple,
            (odometerLeft, odometerRight [, rawLeft, rawRight])
            with odometer<x> in mm, and raw<x> in ticks.
          On error, return None
    """
    rsp = self.SendCmd('H')
    if rsp is None: return None
    return self._ParseRspOdometry(rsp, incraw)
  
  #--
  def CmdReadAdc(self, channel):
    """ Read the Analog to Digital Converter Channel ('I').
  
        Return Value:
          10-bit integer value on success, None on error.
    """
    channel = util.cap(channel, KheAdcChanMin, KheAdcChanMax)
    rsp = self.SendCmd('I,%d' % channel)
    if rsp is None: return None
    return self._ParseRspAdc(rsp)
  
  #--
  def CmdSetBaseLedState(self, led, state):
    """ Set Khepera Base user controllable LED state ('L').
  
        Parameters:
          led   -  LED [0, 1]
          state  - LED state. 0 = off, 1 = on
  
        Return Value:
          New LED state on success, None on error.
    """
    led  = util.cap(led,  KheLedMin, KheLedMax)
    state = util.cap(state,  KheStateOff, KheStateOn)
    rsp = self.SendCmd('L,%d,%d' % (led, state))
    if rsp is None: return None
    return state
  
  #--
  def CmdReadProximitySensors(self, incraw=False):
    """ Read Khepera Base proximity sensors ('N').
  
        Parameters:
          incraw   - do [not] include raw values with return
  
        Return Value:
          On success, the dictionary of labeled proximity sensor values.
            Each value is either: dist or (dist, raw) with
            dist in mm, and raw value range in [0,1023],
              0  = very far to 
            1023 = very close
          On error, return None
    """
    rsp = self.SendCmd('N')
    if rsp is None: return None
    if len(rsp) < 1+2*KheIrSensorNumOf:
      return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    sensorVals = util.cvtCsvIntStr(rsp[2:])
    if len(sensorVals) != KheIrSensorNumOf: 
      return self.mErr.SetErrBadRsp(rsp, "Invalid number of sensor values")
    sensorDict = {}
    n = 0
    for val in sensorVals:
      id = 'prox_' + KheIrSensorOrder[n]
      dist = self.ProximitySensorDist(id, val)
      if not incraw:
        sensorDict[id] = dist
      else:
        sensorDict[id] = (dist, val)
      n += 1
    return sensorDict
  
  #--
  def CmdReadAmbientSensors(self, incraw=False):
    """ Read Khepera Base ambient sensors ('O').
  
        Parameters:
          incraw   - do [not] include raw values with return
  
        Return Value:
          On success, the dictionary of labeled ambient sensor values.
            Each value is either: dist or (dist, raw) with
            dist in mm, and raw value range in [0,1023],
                0 = brightest infrared light (closest) to
             1023 = no light (very far)
          On error, return None
    """
    rsp = self.SendCmd('O')
    if rsp is None: return None
    if len(rsp) < 1+2*KheIrSensorNumOf:
      return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    sensorVals = util.cvtCsvIntStr(rsp[2:])
    if len(sensorVals) != KheIrSensorNumOf: 
      return self.mErr.SetErrBadRsp(rsp, "Invalid number of sensor values")
    sensorDict = {}
    n = 0
    for val in sensorVals:
      id = 'amb_' + KheIrSensorOrder[n]
      dist = self.AmbientSensorDist(id, val)
      if not incraw:
        sensorDict[id] = dist
      else:
        sensorDict[id] = (dist, val)
      n += 1
    return sensorDict
  
  #--
  def CmdExtBusRead(self, addr):
    """ Read a byte from the Khepera Base extension bus at the 
        given bus address ('R').
  
        Parameters:
          addr   -  integer bus address [0, 63] or bus address mnemonic.
                    See data KheExtBusAddrDict for list of supported 
                    mnemonics.

        Return Value:
          Byte read on success, None on error.
    """
    addr = self._MakeParamExtBusAddr(addr)
    if addr is None: return None
    rsp = self.SendCmd('R,%d' % (addr))
    if rsp is None: return None
    if len(rsp) < 2: 
      return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    byte = util.cvtInt(rsp[2:])
    if byte is None:
      return self.mErr.SetErrBadRsp(rsp, "Invalid integer value")
    return byte
  
  #--
  def CmdExtBusWrite(self, addr, byte):
    """ Write a byte to the Khepera Base extension bus at the 
        given bus address ('W').
  
        Parameters:
          addr   -  integer bus address [0, 63] or bus address mnemonic.
                    See data KheExtBusAddrDict for list of supported 
                    mnemonics.
          byte   - integer value

        Return Value:
          Byte written on success, None on error.
    """
    addr = self._MakeParamExtBusAddr(addr)
    if addr is None: return None
    if byte < 0 or byte > 0xff:
        return self.mErr.SetErrBadParam('byte', 'Out of range', byte)
    rsp = self.SendCmd('W,%d,%d' % (byte, addr))
    if rsp is None: return None
    return byte
  
  #--
  def CmdTurret(self, tid, tcmd):
    """ Write a command to a Khepera Turret ('T').
  
        Parameters:
          tid   - integer turret id [0, 31] or id mnemonic.
                    See data KheTidDict for list of supported mnemonics.
          tcmd  - turret command string

        Return Value:
          Unparsed turret response string.
    """
    tid = self._MakeParamTurretId(tid)
    if tid is None: return None
    if not tcmd:
        return self.mErr.SetErrBadParam('tcmd', 'No turret command specified')
    rsp = self.SendCmd('T,%d,%s' % (tid, tcmd))
    if rsp is None: return None
    return rsp[2:]
  

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
      if sensor['k_brightness'] < KheIrProxMinKBrightness:
        sensor['k_brightness'] = KheIrProxMinKBrightness
      elif sensor['k_brightness'] > KheIrProxMaxKBrightness:
        sensor['k_brightness'] = KheIrProxMaxKBrightness
      sensor['noise_floor'] = newParams['noise_floor']
      if sensor['noise_floor'] < KheIrProxMinNoiseFloor:
        sensor['noise_floor'] = KheIrProxMinNoiseFloor
      elif sensor['noise_floor'] > KheIrProxMaxNoiseFloor:
        sensor['noise_floor'] = KheIrProxMaxNoiseFloor

  #--
  def ProximitySensorsSetDftCalParams(self):
    """ Restores the current proximity sensors' calibration parameters
        to factory default.

        Return Value:
          None
    """
    self.mIrProxCalParams = {}
    for id in KheIrSensorOrder:
      self.mIrProxCalParams['prox_'+id] = {
        'enabled': True,
        'k_brightness': KheIrProxDftKBrightness,
        'noise_floor': KheIrProxDftNoiseFloor
      }

  #--
  def ProximitySensorGetDftCalibration(self):
    """ Get the default calibration data for the proximity IR LED sensor. """
    return {'unitsX':'adc-10bit', 'unitsY':'mm',
            'calData':KheIrProxDftCalWhiteSheet}
 
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
      return KheIrSensorInfDist

    # below noise floor for this sensor
    elif sensorVal < calParams['noise_floor']:
      return KheIrSensorInfDist

    # adjust sensor value
    sensorVal /= calParams['k_brightness']
    if sensorVal > KheIrSensorValMax:
      sensorVal = KheIrSensorValMax

    # interpolate distance
    caldata = KheIrProxDftCalWhiteSheet
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
                          factory default of a 50W light source.
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
      if sensor['k_brightness'] < KheIrAmbMinKBrightness:
        sensor['k_brightness'] = KheIrAmbMinKBrightness
      elif sensor['k_brightness'] > KheIrAmbMaxKBrightness:
        sensor['k_brightness'] = KheIrAmbMaxKBrightness
      sensor['noise_floor'] = newParams['noise_floor']
      if sensor['noise_floor'] < KheIrAmbMinNoiseFloor:
        sensor['noise_floor'] = KheIrAmbMinNoiseFloor
      elif sensor['noise_floor'] > KheIrAmbMaxNoiseFloor:
        sensor['noise_floor'] = KheIrAmbMaxNoiseFloor

  #--
  def AmbientSensorsSetDftCalParams(self):
    """ Restores the current ambient sensors' calibration parameters
        to factory default.

        Return Value:
          None
    """
    self.mIrAmbCalParams = {}
    for id in KheIrSensorOrder:
      self.mIrAmbCalParams['amb_'+id] = {
        'enabled': True,
        'k_brightness': KheIrAmbDftKBrightness,
        'noise_floor': KheIrAmbDftNoiseFloor
      }

  #--
  def AmbientSensorGetDftCalibration(self):
    """ Get the default calibration data for the ambient IR LED sensor. """
    return {'unitsX':'adc-10bit', 'unitsY':'mm',
            'calData': KheIrAmbDftCal50WLight}
 
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
      return KheIrSensorInfDist

    # below noise floor for this sensor
    elif sensorVal < calParams['noise_floor']:
      return KheIrSensorInfDist

    # adjust sensor value
    sensorVal /= calParams['k_brightness']
    if sensorVal > KheIrSensorValMax:
      sensorVal = KheIrSensorValMax

    # interpolate distance
    caldata = KheIrAmbDftCal50WLight
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
  
  # Odometry response regular expression
  _reRspOdometry = re.compile('([-]?\d+),([-]?\d+)');
  
  # ADC response regular expression
  _reRspAdc = re.compile('(\d+)');
  
  # LED states response regular expression
  _reRspLed = re.compile('[01],[01],([01]),([01])');
  
  #--
  def _ParseRspVersion(self, rsp):
    """ Return version response string tuple (versionBios, versionProtocol). """
    if rsp is None or len(rsp) < 5: 
      return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    verList = rsp.split(',')
    if len(verList) != 3: 
      return self.mErr.SetErrBadRsp(rsp, "Invalid format")
    return verList[1], verList[2]
  
  #--
  def _ParseRspSpeed(self, rsp, incraw):
    """ Return speed response motorLeft, motorRight integer values. """
    if len(rsp) < 5: return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    match = self._reRspSpeed.match(rsp[2:])
    if not match: return self.mErr.SetErrBadRsp(rsp, "Invalid format")
    rawLeft     = int(match.group(1))
    rawRight    = int(match.group(2))
    motorLeft   = rawLeft * KheSpeedMmpsPerPp10ms
    motorRight  = rawRight * KheSpeedMmpsPerPp10ms
    if not incraw:
      return motorLeft, motorRight
    else:
      return motorLeft, motorRight, rawLeft, rawRight
  
  #--
  def _ParseRspOdometry(self, rsp, incraw):
    """ Return odometry response odometerLeft, odometerRight float values. """
    if len(rsp) < 5: return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    match = self._reRspOdometry.match(rsp[2:])
    if not match: return self.mErr.SetErrBadRsp(rsp, "Invalid format")
    rawLeft       = int(match.group(1))
    rawRight      = int(match.group(2))
    odometerLeft  = float(rawLeft) * KheOdometerMmpt 
    odometerRight = float(rawRight) * KheOdometerMmpt 
    if not incraw:
      return odometerLeft, odometerRight
    else:
      return odometerLeft, odometerRight, rawLeft, rawRight
  
  #--
  def _ParseRspAdc(self, rsp):
    """ Return ADC response integer value. """
    if len(rsp) < 3: return self.mErr.SetErrBadRsp(rsp, "Invalid length")
    match = self._reRspAdc.match(rsp[2:])
    if not match: return self.mErr.SetErrBadRsp(rsp, "Invalid format")
    return int(match.group(1))
  
  #--
  def _MakeParamExtBusAddr(self, addr):
    """ Convert extension bus address to integer address. """
    if type(addr) == int:
      if addr < KheExtBusAddrMin or addr > KheExtBusAddrMax:
        return self.mErr.SetErrBadParam('addr', 'Out of range', addr)
      else:
        return addr
    else:
      if addr not in KheExtBusAddrDict:
        return self.mErr.SetErrBadParam('addr', 'Invalid Ext. Bus mnemonic', 
                                          addr)
      else:
        return KheExtBusAddrDict[addr]
  
  #--
  def _MakeParamTurretId(self, tid):
    """ Convert turret id to integer value. """
    if type(tid) == int:
      if tid < KheTidMin or tid > KheTidMax:
        return self.mErr.SetErrBadParam('tid', 'Out of range', tid)
      else:
        return tid
    else:
      if tid not in KheTidDict:
        return self.mErr.SetErrBadParam('tid', 'Invalid Turret Id mnemonic', 
                                          tid)
      else:
        return KheTidDict[tid]
