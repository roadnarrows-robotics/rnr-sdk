################################################################################
#
# KheCmdGP2D120.py
#

""" Khepera II Robot with attached Sharp GP2D120 sensor.

This module provides the interface to the Khepera II robot with a mounted 
General I/O turret. The GenIO turret has an attached Sharp GP2D120 sensor.
The Sharp GP2D120 is an IR LED distance measuring sensor that can measure 
distances between 4 - 30cm.  The sensor is mounted facing forwards.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.03

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
import Fusion.Khepera.Cmd.KheCmdBase as KheCmdBase


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Sharp GP2D120 Factory Data
# 
# From the sensor:
#   V = GP2D120(R_g), where V = Volts, R_g = distance measured by the sensor
# From the Khepera:
#   R = f(ADC(V)), where R = calculated distance in mm, V is output voltage
#   of the GP2D120, ADC is the Analog To Digital Converter of the General I/O
#   Turret, and f() is the linearized function described below.
#
# The Volts vs. Distance data were eyeballed from the Sharp GP2D120 Reference 
# Document.
#
GP2D120FactDataVoltsVsMm = [
  (2.74, 40.0),  (2.02, 60.0),  (1.56, 80.0),  (1.06, 120.0), 
  (0.82, 160.0), (0.66, 200.0), (0.44, 300.0), (0.31, 400.0)
]

#
# Distance Range
#
# Specified and absolute measurable distance ranges in mm. Note that the
# absolute range exceeds the Sharp specification. However, the measurements 
# seem to behave reasonable although at far distances errors > 100% may occur.
#
GP2D120DistSpecMin    =   40.0 # Minimum specified distance in mm
GP2D120DistSpecMax    =  300.0 # Maximum specified distance in mm
GP2D120DistMin        =   30.0 # Minimum measurable distance in mm
GP2D120DistMax        =  400.0 # Maximum measurable distance in mm
GP2D120DistInf        = 1000.0  # 'Infinite distance for this sensor

#
# Other Data
#
GP2D120_k             = 4.20  # linearizing constant
GP2D120GenIOChan      = 3     # channel number of the General I/O Turret 
                              # the GP2D120 output is connected to


#-------------------------------------------------------------------------------
# Khepera II Base + GP2D120 Command Class
#-------------------------------------------------------------------------------
class KheCmdGP2D120(KheCmdBase.KheCmdBase):
  """ Khepera II Robot with GP2D120 Sensor Class
  """

  # Integer Linear Function Parameters
  # Values calibtrated to factory data listed above.
  m_prime = 30443           # slope to integer linear function
  b_prime =   -15           # y-intercept of integer linear function
  k_prime = int(GP2D120_k)  # integer version of k

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
    KheCmdBase.KheCmdBase.__init__(self, port, baudrate, dbgobj)

    # factory calibration data
    self.mCalData   = GP2D120FactDataVoltsVsMm 
    self.mUnitsG    = 'volts'
    self.mUnitsR    = 'mm'


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GP2D120 Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GP2D120Dist(self, adc):
    """ Calculate the distance given the General I/O ADC value of the
        GP2D120 output voltage.

        Parameters:
          adc       - 10-bit ADC value

        Return Value:
          Distance in mm. -1 on invalid calculation (out of range)
    """
    if adc == -self.b_prime: return -1   # divide by zero
    d = float(self.m_prime / (adc + self.b_prime) - self.k_prime)
    if d >= GP2D120DistMin and d <= GP2D120DistMax:
      return d
    else:
      return GP2D120DistInf

  #--
  def GP2D120Calibrate(self, calData, unitsG='adc', unitsR='mm'):
    """ Calibrate the GP2D120 sensor with the given real-world data to find 
        the best fit slope m and y-intercept b values to the linearized 
        function provided by Sharp: m*ADC + b = 1/(R + k), k = 4.2.
        Solving for the distance, the integer linear function is:
          R = m_prime / (ADC + b_prime) - k_prime
            where m_prime = int(1/m), b_prime = int(b/m) k_prime = int(k).

        Parameters:
          calData   - list of calibration data points [(G1,R1), (G2,R2) ...]
                        where Gn is the GP2D120() output and Rn is the
                        distance.
          unitsG    - units of GP2D120() output. Supported values are:
                        'adc'   = Analog to Digital Converter bits (default)
                        'volts' = volts DC
          unitsR    - units of distance R. Supported values are:
                        'mm' = millimeters (default)
                        'cm' = centimeters
                        'in' = inches

        Return Value:
          Returns m_prime, b_prime
    """
    if not calData: 
      raise ValueError("Parameter 'calData' not specified or is empty")
    if len(calData) < 2:
      raise ValueError("Parameter 'calData' must have at least 2 data points")
    if   unitsG == 'adc':   scaleG = None
    elif unitsG == 'volts': scaleG = KheCmdBase.KheAdcBpv # bits/volt
    else:
      raise ValueError("Parameter 'unitsG' value is unknown: %s" \
          % (repr(unitsG)))
    if   unitsR == 'mm': scaleR = None
    elif unitsR == 'cm': scaleR = 10.0
    elif unitsR == 'in': scaleR = 25.4
    else:
      raise ValueError("Parameter 'unitsR' value is unknown: %s" \
          % (repr(unitsR)))

    # linearize data
    calLinData = []
    for pt in calData:
      g = pt[0]
      if scaleG: g *= scaleG
      r = pt[1]
      if scaleR: r *= scaleR
      if r >= GP2D120DistMin and r <= GP2D120DistMax and \
         g >= 0 and g <= KheCmdBase.KheAdcMax:
        calLinData += [(g, 1.0/(r + GP2D120_k))]

    if len(calLinData) < 2:
      raise ValueError(
          "Calibration data must have 2 or more data points in range")

    # new calibration data
    self.mCalData   = calData 
    self.mUnitsG    = unitsG
    self.mUnitsR    = unitsR

    # linear regression fit new coeficients
    m, b = self._Fit(calLinData)
    m_prime = int(1.0 / m)
    b_prime = int(b / m)

    return m_prime, b_prime

  #--
  def GP2D120Compare(self, benchmark, unitsG='adc', unitsR='mm'):
    """ Compare benchmark data to distances calculated using the current 
        (calibrate) integer linear function. 

        Parameters:
          benchmark - list of benchmark data points [(G1,R1), (G2,R2) ...]
                        where Gn is the GP2D120() output and Rn is the
                        distance.
          unitsG    - units of GP2D120() output. Supported values are:
                        'adc'   = Analog to Digital Converter bits (default)
                        'volts' = volts DC
          unitsR    - units of distance R. Supported values are:
                        'mm' = millimeters (default)
                        'cm' = centimeters
                        'in' = inches
        Return Value:
          List of triplet values (Gn, Rn, Cn) where Gn, Rn are the same 
          benchmark values and Cn is the calculated distance in units of Rn.
    """
    triplet = []
    for pt in benchmark:
      adc = pt[0]
      if unitsG == 'volts': adc *= KheCmdBase.KheAdcBpv # bits/volt
      r = self.Dist(adc)
      if unitsR == 'cm': r /= 10.0
      if unitsR == 'in': r /= 25.4
      triplet += [(pt[0], pt[1], int(r))]
    return triplet

  #--
  def GP2D120GetCalibration(self):
    """ Get the current calibration data for the GP2D120 sensor.

        Return Values:
          Dictionary {'unitsG':unitsG, 'unitsR':unitsR, 'calData':data}
          where:
            unitsG  - units of GP2D120() output. Supported values are:
                       'adc'   = Analog to Digital Converter bits (default)
                       'volts' = volts DC
            unitsR  - units of distance R. Supported values are:
                       'mm' = millimeters (default)
                       'cm' = centimeters
                       'in' = inches
            data    - list of benchmarked data points [(G1,R1), (G2,R2) ...]
                       where Gn is the GP2D120() output and Rn is the
                       distance.
    """
    return {'unitsG':self.mUnitsG, 'unitsR':self.mUnitsR, 
            'calData':self.mCalData}

  #--
  def GP2D120GetDefaultCalibration(self):
    """ Get the default calibration data for the GP2D120 sensor.
        (See GP2D120GetCalData())
    """
    return {'unitsG':'volts', 'unitsR':'mm',
            'calData':GP2D120FactDataVoltsVsMm}
 

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdGP2D120MeasureDist(self, incraw=False):
    """ Khepera command to measure the distance using the GP2D120 sensor
        of the nearest object to the front of the Khepera.

        Parameters:
          incraw   - do [not] include raw values with return
  
        Return Value:
          On success, return either the value: dist or (dist, raw) with
            dist in mm, and raw value range in [0,1023],
              0  = very far to 
            1023 = very close
          On error, return 1000.0 [, 0]
        Return Value:
          Integer distance in mm on success, -1 on measurement failure.
    """
    adc = self.CmdReadAdc(GP2D120GenIOChan)
    if adc:
      dist = self.GP2D120Dist(adc)
    else:
      dist = GP2D120DistInf
      adc = 0
    if not incraw:
      return dist
    else:
      return (dist, adc)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _Fit(self, xyList):
    """ Linear Regression
        Best fit a linear line to the given list of (x,y) data points.

        Parameters:
          xyList - list of (x,y) data points

        Return Value:
          Returns slope and y-intercept (m, b) of best fit line.
    """
    m = b = 0.0
    sx = sy = 0.0
    ss = len(xyList)
    for pt in xyList:
      sx += pt[0]
      sy += pt[1]
    sxoss = sx / ss
    st2 = 0.0
    for pt in xyList:
      t = pt[0] - sxoss
      st2 += t * t
      m += t * pt[1]
    m /= st2
    b = (sy - sx * m) / ss
    return m, b
