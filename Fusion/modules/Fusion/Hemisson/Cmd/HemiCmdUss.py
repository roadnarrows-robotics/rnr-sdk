################################################################################
#
# HemiCmdUss.py
#

""" Hemisson UltraSonic Sensor Command Module

Hemisson UltraSonic Sensor serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.26

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

import Fusion.Hemisson.Cmd.HemiSerial as HemiSerial
import Fusion.Hemisson.Cmd._hemiUtils as hutil


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

UssEchoMaxNum       =    17     # Maximum number of echo measurements
UssEchoMin          =     0     # Minimum echo number
UssEchoMax          =    16     # Maximum echo number

UssRangeMin         =     1     # Minimum range
UssRangeMax         = 65535     # Maximum range
UssRangeDft         = 65535     # Default power-up range (11 meters)
UssRangeMinMm       = 43.0      # Maximum useful range in mm
UssRangeMaxMm       = 6000.0    # Miniumu useful range in mm
UssRangeDftMm       = 11000.0   # Default power-up range in mm (not useful)

UssAngRange         = 55.0      # 55 degree angular beam

UssResolutionMin    = 43.0      # Minimum resolution (mm)

UssUnits            = ['C', 'I', 'U'] # centimeters, inches, microseconds
UssUnitsDft         = 'C'       # Default power-up units

#-------------------------------------------------------------------------------
# CLASS: HemiCmdUss
#-------------------------------------------------------------------------------
class HemiCmdUss(HemiSerial.HemiSerial):
  """ Hemisson UltraSonic Sensor Command and Response Class. """

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
    self.mRange = UssRangeDft
    self.mUnits = UssUnitsDft


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Serial Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def UssCmdGetEchoSet(self, numOfEchoes=UssEchoMaxNum):
    """ Get all echos from the last issued distance measurement. 
        The units of the echoes are in the units specified at the
        last measurement time ('A').

        Parameters:
          numOfEchos  - number of the echoes to retrieve [1,17]
  
        Return Value:
          On success, return list of distance measurements of the echoes,
          sorted from closest to farthest distance.
          Each echo value range = [0, 65535] with:
             0   = indicates that no echo was detected
             1+  = distance in units at measurement time
          Return None on bad response.
    """
    numOfEchoes = hutil.cap(numOfEchoes, 1, UssEchoMaxNum)
    rsp = self.SendCmd('A,U,A')
    if rsp is None: return None
    if len(rsp) != 5 + (UssEchoMaxNum * 5): return self.mErr.SetErrBadRsp(rsp)
    rsp = rsp[6:5 +(numOfEchoes * 5)]
    return self._UssParseRspEchoList(rsp)
  
  #--
  def UssCmdGetOneEcho(self, echonum):
    """ Get one echo from the last issued distance measurement. 
        The units of the echo are in the units specified at the
        last measurement time.  ('E').
  
        Parameters:
          echonum  - echo number [0,16]
                       0 = closest
                      16 = farthest
  
        Return Value:
          On success, return distance measurement of the echo.
          Echo value range = [0, 65535] with:
             0   = indicates that no echo was detected
             1+  = distance in units at measurement time
          Return None on bad response.
    """
    echonum = hutil.cap(echonum, UssEchoMin, UssEchoMax)
    rsp = self.SendCmd('A,U,E,' + "%0.2X" % echonum)
    if rsp is None: return None
    if len(rsp) != 13: return self.mErr.SetErrBadRsp(rsp)
    echo = self._UssParseRspEchoList(rsp[9:])
    if echo:
      return echo[0]
    else:
      return None

  #--
  def UssCmdGetLightIntensity(self):
    """ Get the ambient light intensity from the last issued distance 
        measurment. ('L')
  
        Return Value:
          On success, return the ambient light intensity value. 
          Sensor value range = [0, 255] with:
            0-3    = indicates is complete darkness
            248+   = under bright light
          Return None on bad response.
    """
    rsp = self.SendCmd('A,U,L')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp[6:])
  
  #--
  def UssCmdTakeMeasurement(self):
    """ Take an UltraSonic measurement in the current units and at the
        current maximum range. The ambient light intensity will also be 
        sampled. This command is synchronous. No response is returned 
        until the measurement is completed. At most 65ms will elapse at 
        the maximum ranging distance ('M').
  
        Return Value:
          'completed' on success, None on bad response.
    """
    rsp = self.SendCmd('A,U,M')
    if rsp is None: return None
    if len(rsp) != 5: return self.mErr.SetErrBadRsp(rsp)
    return 'completed'
  
  #--
  def UssCmdPing(self, numOfEchoes=UssEchoMaxNum):
    """ Take a measurement and return the distance values echoed. 
        Ping ('M', 'A').

        Parameters:
          numOfEchos  - number of the echoes to retrieve [1,17]
  
        Return Value:
          On success, return list of distance measurements of the echoes,
          sorted from closest to farthest distance.
          Each echo value range = [0, 65535] with:
             0   = indicates that no echo was detected
             1+  = distance in units at measurement time
          Return None on bad response.
    """
    if self.UssCmdTakeMeasurement():
      return self.UssCmdGetEchoSet(numOfEchoes)
    else:
      return None
  
  #--
  def UssCmdSetMaxRange(self, range):
    """ Set the maximum range for the UltraSonic Sensor. The range value
        is in the current units. The maximum range that the USS can support
        is 11 meters, but the effective range is only 6 meters ('R').
  
        Parameters:
          range  - maximum range [1,65535] 
  
        Return Value:
          On success, return the actual range value set in hardware.
          Return None on bad response.
    """
    range = hutil.cap(range, UssRangeMin, UssRangeMax)
    rsp = self.SendCmd('A,U,R,' + "%04X" % range)
    if rsp is None: return None
    if len(rsp) != 10: return self.mErr.SetErrBadRsp(rsp)
    self.mRange = hutil.cvtHHHH(rsp[6:])
    return self.mRange
  
  #--
  def UssCmdSetUnits(self, units):
    """ Set the current units of the next measurment(s) to be taken.
        The echoes values from the last issued measurment are still
        in the old units ('U').
  
        Parameters:
          units  - measurement units
                     'C'   = centimeters
                     'I'   = inches
                     'U'   = microseconds
        Return Value:
          New current units on success, None on bad response.
    """
    try:
      UssUnits.index(units)
    except ValueError:
      return self.mErr.SetErrBadParam('units', units)
    rsp = self.SendCmd('A,U,U,' + "%c" % units)
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    self.mUnits = rsp[6].capitalize()
    return self.mUnits

  #--
  def UssCmdGetSettings(self):
    """ Get USS current settings.
  
        Return Value:
          Dictionary of current settings.
    """
    return {'range': self.mRange, 'units': self.mUnits}
 
  #--
  def UssCmdGetVersion(self):
    """ Get Hemisson USS version ('V').
  
        Return Value:
          Version integer value on success, None on bad response.
    """
    rsp = self.SendCmd('A,U,V')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  #--
  def _UssParseRspEchoList(self, csvstr):
    """ Return message said or None. """
    width = 4
    numvals = len(csvstr) / (width + 1) + 1
    dlist = []
    n = 0
    i = 0
    while i < len(csvstr) and n < numvals:
      val = hutil.cvtHHHH(csvstr[i:i+width])
      if val == None:
        return self.mErr.SetErrBadRsp("bad hex-value: '%s'" % csvstr[i:i+width])
      else:
        dlist += [val]
      i += (width + 1)
      n += 1
    return dlist
