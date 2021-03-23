################################################################################
#
# HemiCmdLinCam.py
#

""" Hemisson Linear Camera Commands Module

Hemisson Linear Camera serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.16

Copyright (C) 2004, 2005. 2006.  RoadNarrows LLC.
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

import Fusion.Hemisson.Cmd.HemiSerial as HemiSerial
import Fusion.Hemisson.Cmd._hemiUtils as hutil


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

LinCamNumPixels       = 102   # Total number of Pixels in linear camera
LinCamNumPPZ          =  34   # Number of Pixels Per Zone
LinCamNumZones        =   3   # Number of Zones
LinCamZoneAll         =   0   # All zones
LinCamZoneLeft        =   1   # Left zone
LinCamZoneMiddle      =   2   # Middle zone
LinCamZoneRight       =   3   # Right zone
LinCamExposureTimeMin =   0   # Minimum exposure time (milliseconds)
LinCamExposureTimeMax = 255   # Maximum exposure time (milliseconds)
LinCamExposureTimeDft =   1   # Default (at power-on) exposure time (ms)
LinCamThresholdMin    =   1   # Minimum threshold value
LinCamThresholdMax    = 255   # Maximum threshold value
LinCamThresholdDft    = 255   # Default (at power-on) threshold value
LinCamStateOff        =   0   # Linear Camera general off state
LinCamStateOn         =   1   # Linear Camera general on state


#-------------------------------------------------------------------------------
# CLASS: HemiCmdLinCam
#-------------------------------------------------------------------------------
class HemiCmdLinCam(HemiSerial.HemiSerial):
  """ Hemisson Linear Camera Command and Response Class.  """

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
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Serial Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def LinCamCmdGrabPPixels(self, zone=LinCamZoneAll):
    """ Grab Hemisson Linear Camera unthresholded pixels ('P').
  
        Parameters:
          zone  - pixel zone 
                    LinCamZoneAll    = all zones  (102 pixels) [DEFAULT]
                    LinCamZoneLeft   = left zone   (34 pixels)
                    LinCamZoneMiddle = middle zone (34 pixels)
                    LinCamZoneRight  = right zone  (34 pixels)
  
        Return Value:
          Integer gray-level, left-to-right pixel list on success.
          Pixel value range = [0,255] with:
              0 = black to
            255 = white
          None on bad response.
    """
    rsp = self.SendCmd('A,L,P,' + "%d" % zone)
    if rsp is None: return None
    return self._LinCamParseRspPixelList(zone, rsp)
   
  #--
  def LinCamCmdGrabQPixels(self, zone=LinCamZoneAll):
    """ Grab Hemisson Linear Camera thresholded pixels ('Q').
  
        Parameters:
          zone  - pixel zone 
                    LinCamZoneAll    = all zones  (102 pixels) [DEFAULT]
                    LinCamZoneLeft   = left zone   (34 pixels)
                    LinCamZoneMiddle = middle zone (34 pixels)
                    LinCamZoneRight  = right zone  (34 pixels)
  
        Return Value:
          Integer black & white, left-to-right pixel list on success.
          Pixel value is either:
              0 = black if under threshold
            255 = white if >= threshold
          None on bad response.
    """
    rsp = self.SendCmd('A,L,Q,' + "%d" % zone)
    if rsp is None: return None
    return self._LinCamParseRspPixelList(zone, rsp)
  
  #--
  def LinCamCmdSetExposureTime(self, exposureTime):
    """ Set Hemisson Linear Camera exposure time ('E').
  
        Parameters:
          exposuretime  - time in milliseconds [0,255]
  
        Return Value:
          New exposure time on success, None on bad response.
    """
    exposureTime = hutil.cap(exposureTime, LinCamExposureTimeMin,
                            LinCamExposureTimeMax)
    rsp = self.SendCmd('A,L,E,' + "%02x" % exposureTime)
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  #--
  def LinCamCmdGetExposureTime(self):
    """ Get Hemisson Linear Camera current exposure time ('F').
  
        Return Value:
          Current exposure time on success, None on bad response.
    """
    rsp = self.SendCmd('A,L,F')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  #--
  def LinCamCmdSetQThreshold(self, threshold):
    """ Set Hemisson Linear Camera pixel threshold value ('T').
  
        Parameters:
          threshold  - pixel threshold [1,255]
  
        Return Value:
          New threshold value on success, None on bad response.
    """
    threshold = hutil.cap(threshold, LinCamThresholdMin, LinCamThresholdMax)
    rsp = self.SendCmd('A,L,T,' + "%02x" % threshold)
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  #--
  def LinCamCmdGetQThreshold(self):
    """ Get Hemisson Linear Camera current pixel threshold value ('U').
  
        Return Value:
          Current threshold value on success, None on bad response.
    """
    rsp = self.SendCmd('A,L,U')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  #--
  def LinCamCmdSetLedState(self, state):
    """ Set Hemisson Linear Camera LED state ('L').
  
        Parameters:
          state - LED state
                    0 = off
                    1 = on
  
        Return Value:
          New LED state on success, None on bad response.
    """
    rsp = self.SendCmd('A,L,L,' + "%d" % state)
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    try:
      state = int(rsp[6:])
      return state
    except (SyntaxError, NameError, TypeError, ValueError):
      return self.mErr.SetErrBadRsp(rsp)
  
  #--
  def LinCamCmdGetVersion(self):
    """ Get Hemisson Linear Camera version ('V').
  
        Return Value:
          Version integer value on success, None on bad response.
    """
    rsp = self.SendCmd('A,L,V')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  #--
  def _LinCamParseRspPixelList(self, zone, rsp):
    """ Return pixel integer list. """
    hdrlen = 8
    if zone > 0:
      numvals = LinCamNumPPZ
    else:
      numvals = LinCamNumPixels
    if len(rsp) != hdrlen + (numvals * 2):
      #print("bad rsp:", repr(rsp))
      return self.mErr.SetErrBadRsp(rsp)
    rsp = rsp[hdrlen:]   # strip off header
    return hutil.cvtCatHHStr(rsp, numvals)
