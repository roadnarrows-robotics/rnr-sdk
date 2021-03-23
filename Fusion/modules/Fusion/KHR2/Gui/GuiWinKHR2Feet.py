################################################################################
#
# GuiWinKHR2Feet
#

""" Graphical User Interface KHR-2 BrainPack Feet Window

Graphical User Interface (GUI) Tkinter window displays current
RoadNarrows BrainPack feet sensor data, plus simple display options.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.10.21

Copyright (C) 2007.  RoadNarrows LLC.
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

import sys
import socket
import errno
import time
import math
import random

import Tkinter as tk

import Fusion.Core.Values as Values

import Fusion.Utils.IVTimer as IVTimer

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiMenuBar as GuiMenuBar
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import Fusion.Gui.GuiWin as GuiWin

import Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient
import Fusion.KHR2.Gui.GuiDlgKHR2Proxy as GuiDlgKHR2Proxy

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# BrainPack Data Classes
BpDataClassRaw     = 0
BpDataClassCooked  = 1

# BrainPack Feet
BpFeet            = ['bpfoot_left', 'bpfoot_right'] # keys
BpFootSensorMax   = 255        # max. sensor data value
BpFootSensorMask  = 0x00ff      # sensor mask
BpFootVecMagMax   = 858        # max calc'd vector length
BpFootWidth       = 32.5        # mm from left to right sensor columns
BpFootHeight      = 86.0        # mm from top to bottom sensor rows
BpFootPtFulcrum   = (BpFootWidth/2.0, BpFootHeight/2.0)
                                # point fulcrum position at center of foot
BpFootNumOfSoleSensors  = 6
BpFootNumOfToeSensors   = 2
BpFootNumOfFootSensors  = BpFootNumOfSoleSensors + BpFootNumOfToeSensors

# foot sensor index order
BpFootSensorUppL  = 0     # foot upper left
BpFootSensorUppR  = 1     # foot upper right
BpFootSensorMidL  = 2     # foot middle left
BpFootSensorMidR  = 3     # foot middle right
BpFootSensorLowL  = 4     # foot lower left
BpFootSensorLowR  = 5     # foot lower right
BpFootSensorToeL  = 6     # foot toe left
BpFootSensorToeR  = 7     # foot toe right


# feet canvas dimensions
_EdgeLeft   = 15    # left edge margin
_EdgeTop    = 10    # top edge margin
_EdgeBottom = 15    # bottom edge margin
_EdgeRight  = 15    # right edge margin

# minimum size
_CanvasMinWidth   = 400 + _EdgeLeft + _EdgeRight
_CanvasMinHeight  = 400 + _EdgeTop + _EdgeBottom

twopi = math.pi * 2.0

#--
def HSVtoRGB(h, s, v):
  """ Convert Hue-Saturation-Value into Red-Green-Blue equivalent.

      Parameters:
        h   - Hue between [0.0, 360.0) degrees. red(0.0) to violet(360.0-).
        s   - Staturation [0.0, 1.0]
        v   - Value [0.0, 1.0]

      Return Value:
        (r, g, b) 3-tuple with each color between [0, 255]
  """
  # achromatic (grey)
  if s == 0.0:
    r = g = b = int(v * 255)
    return (r, g, b)

  # sector 0 to 5
  h /= 60.0
  i = int(h)    # hue whole part
  f = h - i     # hue fraction part of h
  p = v * (1.0 - s)
  q = v * (1.0 - s * f)
  t = v * (1.0 - s * (1.0 - f ))

  if i == 0:
    r = int(v * 255.0)
    g = int(t * 255.0)
    b = int(p * 255.0)
  elif i == 1:
    r = int(q * 255.0)
    g = int(v * 255.0)
    b = int(p * 255.0)
  elif i == 2:
    r = int(p * 255.0)
    g = int(v * 255.0)
    b = int(t * 255.0)
  elif i == 3:
    r = int(p * 255.0)
    g = int(q * 255.0)
    b = int(v * 255.0)
  elif i == 4:
    r = int(t * 255.0)
    g = int(p * 255.0)
    b = int(v * 255.0)
  else: # sector 5
    r = int(v * 255.0)
    g = int(p * 255.0)
    b = int(q * 255.0)

  return (r, g, b)

#--
def ColorMap(val, max):
  f = float(val) / float(max+1)   # [0.0, 1.0)
  #hue = 240.0 - 180.0 * f         # blue(0) to yellow(max)
  hue = 240.0
  value = 0.3 + 0.7 * f           # dark to light
  #value = 1.0
  #value = 1.0 - 0.5 * f           # light to dark
  sat = 0.1 + 0.9 * f           # grey to pure
  #sat = 1.0 - 0.25 * f           # pure to grey
  return "#%02x%02x%02x" % HSVtoRGB(hue, sat, value)

#--
def PsychoColorMap(val, max):
  base = 0x03 
  color = base + val * (0xffffff / max)
  strcolor = "#%02x%02x%02x" % ((color>>16)&0xff, (color>>8)&0xff, color&0xff) 
  return strcolor


#-------------------------------------------------------------------------------
# CLASS: GuiWinKHR2Feet
#-------------------------------------------------------------------------------
class GuiWinKHR2Feet(GuiWin.GuiWin):
  """ GUI Window vKHR2 Feet Log Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the vKHR2 Feet Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Feet options. Options are:
            auto=<bool>       - do [not] automatically update
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = \
                              'vKHR2 RoadNarrows BrainPack Feet Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  def _initMenuBar(self):
    """ Initialize menubar. """
    # File menubar items
    self.mMenuBar.AddMenuItem('File', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('File|Log...', 'command', owner='root',
        command=self.CbLogData)
    self.mMenuBar.AddMenuItem('File|Exit', 'command', owner='root',
        command=self.destroy)
    self.mMenuBar.AddMenuItem('Tools', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Tools|Configure...',
        'command', owner='root', command=self.CbCfgBsProxy)
    self.mMenuBar.AddMenuItem('Tools|IDs',
        'command', owner='root', command=self.CbFootIds)
    self.mMenuBar.AddMenuItem('Tools|Calibrate',
        'command', owner='root', command=self.CbFootCal)
    self.mMenuBar.AddMenuItem('Tools', 'separator')
    self.mMenuBar.AddMenuItem('Tools|I' + gt.UniSuperscript['2']+'C Scan...',
        'command', owner='root', command=self.CbI2CScan)
    self.mMenuBar.AddMenuItem('Tools', 'separator')
    self.mMenuBar.AddMenuItem('Tools|Test Gui',
        'command', owner='root', command=self.CbTestGui)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          vKHR2     - vKHR2 object.
          options   - feet input options.

        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode        = False
    self.mDataClass         = BpDataClassRaw
    self.mBsClient          = BsProxyClient.BsProxyClient()
    self.mBpFootState       = {'bpfoot_left':False, 'bpfoot_right':False}
    self.mI2CDevName        = '/dev/i2c/0'  # default for KoreBot
    self.mTestGui           = False

    # set options from input parameters
    for k,v in options.iteritems():
      if k == 'auto':
        if v:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False
      elif k == 'data_class':
        self.mDataClass = v

    # locals
    self.mIvtPull = IVTimer.IVTimer(1.50, 0.25, self.IVClient, once=True)
    self.mIvtPull.start()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # menu bar
    self.mMenuBar     = GuiMenuBar.GuiMenuBar(self)

    # create the widgets
    self.GuiBody(self)

    # refresh the feet canvas
    self.FeetCanvasRefresh()
    
    # add menu bar
    self._initMenuBar()

  #--
  def show(self):
    """ Show the gui window initialization callback. """
    # calculate important window and widget dimensions used for resizing
    self.CalcDim()

  #--
  def resize(self, event):
    """ Resize callback event. """
    geo = gut.geometry(self)
    # window size has changed, clear but don't redraw until resizing is done
    if geo[gut.W] != self.mWinGeo[gut.W] or geo[gut.H] != self.mWinGeo[gut.H]:
      self.mWinGeo = geo
      self.FeetCanvasClearFeet()
      width = geo[gut.W] - self.mWinBorder
      height = geo[gut.H] - self.mWinBorder - self.mCtlPanelFrameHeight
      self.mFeetCanvas.configure(width=width, height=height)
      self.FeetCanvasParams(width, height)
      width = geo[gut.W] - self.mWinStatusBarBorder
      self.mStatusBar.configure(width=width)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.FeetCanvasDrawFeet()
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
    GuiWin.GuiWin.destroy(self, auto=self.mIsAutoMode,
        data_class=self.mDataClass)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Window Update
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def WinUpdate(self, request, *args, **kwargs):
    """ Update visualization from the current robot state.

        Execution Context: GuiWin server thread

        Parameters:
          request   - the update request 
          *args     - arguments for specific request
          **kwargs  - keyword arguments for specific request

          Specific request and associated parameters:
          'cfg'           - configure BrainPack Feet window.
          'bpfoot_left'   - left foot sensor data
          'bpfoot_right'  - right foot sensor data
          'clear'         - clear current set displayed data
          'connect'       - open connection to bsproxy
          'disconnect'    - close connection to bsproxy

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'cfg':
      self.WinUpdateStatus(**kwargs)
    elif request == 'bpfoot_left':
      self.WinUpdateFeet(request, **kwargs)
    elif request == 'bpfoot_right':
      self.WinUpdateFeet(request, **kwargs)
    elif request == 'clear':
      self.WinUpdateClear()
    elif request == 'connect':
      self.WinUpdateConnect()
    elif request == 'disconnect':
      self.WinUpdateDisconnect()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateFeet(self, whichfoot, raw_data=None, cooked_data=None,
                                      vec_dir=None, vec_mag=None,
                                      device_id=None, version=None):
    """ Update feet sesnor data

        Execution Context: GuiWin server thread

        Parameters:

        Return Value:
          None.
    """
    if self.mDataClass == BpDataClassRaw:
      if raw_data is not None:
        i = 0
        for val in raw_data:
          if val is not None:
            self.mFeetCanvas.itemconfigure(
                self.mGidSensor[whichfoot]['text'][i],
                text='%d' % val) 
            self.mFeetCanvas.itemconfigure(
                self.mGidSensor[whichfoot]['oval'][i],
                fill=ColorMap(val, BpFootSensorMax))
          i += 1
        self.FeetCanvasDrawCenterOfMass(whichfoot, raw_data)
    elif self.mDataClass == BpDataClassCooked:
      if vec_mag is not None and vec_dir is not None:
        self.FeetCanvasDrawForceVector(whichfoot, vec_mag, vec_dir)
      if cooked_data is not None:
        i = 0
        for val in cooked_data:
          if val is not None:
            self.mFeetCanvas.itemconfigure(
                self.mGidSensor[whichfoot]['text'][i],
                text='%d' % val) 
            self.mFeetCanvas.itemconfigure(
                self.mGidSensor[whichfoot]['oval'][i],
                fill=ColorMap(val, BpFootSensorMax))
          i += 1
        self.FeetCanvasDrawCenterOfMass(whichfoot, cooked_data)

  #--
  def WinUpdateStatus(self, **kwargs):
    """ Update status bar with feet data.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              run_time=<str>        - run-time state
              proxy_addr=<str>      - botsense proxy IP address
              proxy_port=<int>      - botsense proxy TCP port
              foot_left_i2c=<int>   - proxied left foot I2C address
              foot_right_i2c=<int>  - proxied right foot I2C address
              data_class=<class>    - data class (raw/cooked)

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.iteritems():
      if k == 'run_time':
        if v == 'update':
          if not self.mBsClient.mBsIsConn:
            items[k] = 'disconnected'
          elif self.mIsAutoMode:
            items[k] = 'automatic'
          else:
            items[k] = 'manual'
        else:
          items[k] = v
        self.mWidgetActiveImage.SetActive(items[k])
      elif k == 'proxy_addr':
        items[k] = v
      elif k == 'proxy_port':
        items[k] = v
      elif k == 'i2c_dev_name':
        items[k] = v
      elif k == 'foot_left_i2c':
        items[k] = v
      elif k == 'foot_right_i2c':
        items[k] = v
      elif k == 'data_class':
        items[k] = v
    self.mStatusBar.Update(**items)

  #--
  def WinUpdateClear(self):
    """ Clear the feet's current set of waypoints and refresh the window.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.FeetCanvasClearFeet()
    self.FeetCanvasDrawFeet()

  #--
  def WinUpdateConnect(self):
    """ Open connection to the BotSense IP Proxy Server.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    if not self.mBsClient.Connect():
      self.CtlPanelStateDisconnected()
      return

    # configure feet proxied device
    for whichfoot in BpFeet:
      i2caddr = self.mBsClient.mBsProxiedDev[whichfoot]['i2c']
      if self.mBpFootState[whichfoot]:
        self.mBsClient.CmdDevOpen(whichfoot, i2caddr, self.mI2CDevName)
    self.CtlPanelStateConnected()

  #--
  def WinUpdateDisconnect(self):
    """ Close connection to the BotSense IP Proxy Server.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.mBsClient.Disconnect()
    self.FeetCanvasRefresh()
    self.CtlPanelStateDisconnected()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Feet Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # current window dimensions
    self.mWinGeo = gut.geometry(self)

    # current control panel dimensions
    cpgeo = gut.geometry(self.mCtlPanelFrame)

    # control panel frame height is fixed
    self.mCtlPanelFrameHeight = cpgeo[gut.H]

    # border around status bar
    self.mWinStatusBarBorder = self.mWinGeo[gut.W] - \
        gut.geometry(self.mStatusBar)[gut.W]

    # minimum width accepted
    minW = _CanvasMinWidth
    if cpgeo[gut.W] > minW:
      minW = cpgeo[gut.W]

    # make height equal width to start with
    minH = minW - _EdgeLeft - _EdgeRight + _EdgeTop + _EdgeBottom

    # window border width and height
    self.mWinBorder = self.mWinGeo[gut.W] - minW

    # set window's minimum size (250 obtained through experimentation)
    self.wm_minsize(
        width=minW+self.mWinBorder,
        height=minH+self.mWinBorder+self.mCtlPanelFrameHeight
    )

  #--
  def GuiBody(self, parent):
    """ Create the window body.

        Parameters:
          parent  - parent to this body

        Return Value:
          None
    """
    row = 0
    column = 0
    self.FeetCanvasInit(parent, row, column)

    row += 1
    self.CtlPanelInit(parent, row, column)

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Feet Canvas Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def FeetCanvasInit(self, parent, row, column):
    """ Initialize Feet canvas.

        Parameters:
          parent  - parent to this canvas
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None.
    """
    self.FeetCanvasParams(_CanvasMinWidth, _CanvasMinHeight)

    self.mFeetCanvas = tk.Canvas(parent,
                                width = self.mFeetCanvasWidth,
                                height = self.mFeetCanvasHeight,
                                bg = '#999999')
    self.mFeetCanvas.grid(row=row, column=column)

    # canvas graphic ids
    self.mGidFeet   = []  # feet
    self.mGidSensor = \
      {'bpfoot_left':
        {'oval':[], 'text':[], 'force_vec':[], 'force_text':[], 'com':[]},
      'bpfoot_right':
        {'oval':[], 'text':[], 'force_vec':[], 'force_text':[], 'com':[]}}

  #--
  def FeetCanvasParams(self, width, height):
    """ Set canvas view parameters.

        Parameters:
          width   - width of canvas (pixels)
          height  - height of canvas (pixels)

        Return Value:
          None
    """
    # some padding
    pad   = 5
    padh  = 5
    padw  = 15

    # screen coordinates
    self.mFeetCanvasWidth   = width
    self.mFeetCanvasHeight  = height
    self.mFootWidth         = \
            (self.mFeetCanvasWidth - _EdgeLeft - _EdgeRight - padw) / 2
    self.mToeHeight         = \
        (self.mFeetCanvasHeight - _EdgeTop - _EdgeBottom) / 5 - padh
    self.mFootHeight        = \
        (self.mFeetCanvasHeight - _EdgeTop - _EdgeBottom) * 4 / 5 - padh

    # foot and toe sensor radius
    self.mFootSensorRadius = self.mFootWidth / 8

    y0_toe             = _EdgeTop
    y1_toe             = _EdgeTop + self.mToeHeight

    x0_foot_left        = _EdgeLeft
    x1_foot_left        = x0_foot_left + self.mFootWidth

    x0_foot_right       = x1_foot_left + padw
    x1_foot_right       = x0_foot_right + self.mFootWidth

    y0_foot            = y1_toe + padh
    y1_foot            = y0_foot + self.mFootHeight

    self.mCoord = {'bpfoot_left':{}, 'bpfoot_right':{}}

    # left foot and toe

    self.mCoord['bpfoot_left']['foot_dim'] = \
            (x0_foot_left, y0_foot, x1_foot_left, y1_foot)

    self.mCoord['bpfoot_left']['foot_midpt'] = \
            (x0_foot_left+self.mFootWidth/2, y0_foot+self.mFootHeight/2)

    self.mCoord['bpfoot_left']['toe_dim'] = \
            (x0_foot_left, y0_toe, x1_foot_left, y1_toe)

    self.mCoord['bpfoot_left']['toe_midpt'] = \
            (x0_foot_left+self.mFootWidth/2, y0_toe+self.mToeHeight/2)

    # left foot and toe sensor mid-lines
    x0_sensor = x0_foot_left + pad + self.mFootSensorRadius
    x1_sensor = x1_foot_left - pad - self.mFootSensorRadius
    y0_sensor_foot = y0_foot + pad + self.mFootSensorRadius 
    y1_sensor_foot = self.mCoord['bpfoot_left']['foot_midpt'][1]
    y2_sensor_foot = y1_foot - pad - self.mFootSensorRadius 
    y0_sensor_toe = self.mCoord['bpfoot_left']['toe_midpt'][1]

    self.mCoord['bpfoot_left']['sensor'] = [None] * BpFootNumOfFootSensors
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorUppL] = \
            (x0_sensor,y0_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorUppR] = \
            (x1_sensor,y0_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorMidL] = \
            (x0_sensor,y1_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorMidR] = \
            (x1_sensor,y1_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorLowL] = \
            (x0_sensor,y2_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorLowR] = \
            (x1_sensor,y2_sensor_foot)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorToeL] = \
            (x0_sensor,y0_sensor_toe)
    self.mCoord['bpfoot_left']['sensor'][BpFootSensorToeR] = \
            (x1_sensor,y0_sensor_toe)

    # right foot and toe

    self.mCoord['bpfoot_right']['foot_dim'] = \
            (x0_foot_right, y0_foot, x1_foot_right, y1_foot)

    self.mCoord['bpfoot_right']['foot_midpt'] = \
            (x0_foot_right+self.mFootWidth/2, y0_foot+self.mFootHeight/2)

    self.mCoord['bpfoot_right']['toe_dim'] = \
            (x0_foot_right, y0_toe, x1_foot_right, y1_toe)

    self.mCoord['bpfoot_right']['toe_midpt'] = \
            (x0_foot_right+self.mFootWidth/2, y0_toe+self.mToeHeight/2)

    # right foot and toe sensor mid-lines
    x0_sensor = x0_foot_right + pad + self.mFootSensorRadius
    x1_sensor = x1_foot_right - pad - self.mFootSensorRadius
    y0_sensor_foot = y0_foot + pad + self.mFootSensorRadius 
    y1_sensor_foot = self.mCoord['bpfoot_right']['foot_midpt'][1]
    y2_sensor_foot = y1_foot - pad - self.mFootSensorRadius 
    y0_sensor_toe = self.mCoord['bpfoot_right']['toe_midpt'][1]

    self.mCoord['bpfoot_right']['sensor'] = [None] * BpFootNumOfFootSensors
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorUppL] = \
            (x0_sensor,y0_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorUppR] = \
            (x1_sensor,y0_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorMidL] = \
            (x0_sensor,y1_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorMidR] = \
            (x1_sensor,y1_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorLowL] = \
            (x0_sensor,y2_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorLowR] = \
            (x1_sensor,y2_sensor_foot)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorToeL] = \
            (x0_sensor,y0_sensor_toe)
    self.mCoord['bpfoot_right']['sensor'][BpFootSensorToeR] = \
            (x1_sensor,y0_sensor_toe)

  #--
  def FeetCanvasClearFeet(self):
    """ Clear the feet map from the Feet canvas. """
    for id in self.mGidFeet:
      self.mFeetCanvas.delete(id)
    self.mGidFeet = []
    for whichfoot in BpFeet:
      for k in self.mGidSensor[whichfoot].iterkeys():
        for id in self.mGidSensor[whichfoot][k]:
          self.mFeetCanvas.delete(id)
        self.mGidSensor[whichfoot][k] = []

  #--
  def FeetCanvasDrawFeet(self):
    self._foot('bpfoot_left')
    self._foot('bpfoot_right')

  #--
  def FeetCanvasDrawForceVector(self, whichfoot, mag, dir):
    for id in self.mGidSensor[whichfoot]['force_vec']:
      self.mFeetCanvas.delete(id)
    self.mGidSensor[whichfoot]['force_vec'] = []
    if self.mDataClass == BpDataClassCooked:
      self._forcevector(whichfoot, mag, dir)

  #--
  def FeetCanvasDrawCenterOfMass(self, whichfoot, sensors):
    for id in self.mGidSensor[whichfoot]['com']:
      self.mFeetCanvas.delete(id)
    self.mGidSensor[whichfoot]['com'] = []
    self._centerofmass(whichfoot, sensors)

  #--
  def FeetCanvasRefresh(self):
    """ Refresh the Feet canvas. """
    self.FeetCanvasClearFeet()
    self.FeetCanvasDrawFeet()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the Feet Control Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    # the frame
    cpframe = tk.Frame(parent, relief=tk.RAISED, borderwidth=1)
    cpframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0

    # control panel title
    w = tk.Label(cpframe, text='vKHR2 BrainPack Feet Control Panel',
        fg=gt.ColorGreen1)
    w.grid(row=row, column=column)

    row += 1
    column = 0

    # subframe
    subframe = tk.Frame(cpframe, relief=tk.FLAT, borderwidth=0)
    subframe.grid(row=row, column=column, padx=1, pady=1, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # auto/manual visual feedback
    autoFiles = []
    for n in [0, 1, 2, 3]:
      filename = gut.GetFusionImageFileName('RalfWalking%d.gif' % n)
      if filename:
        autoFiles += [filename]
    manFile = gut.GetFusionImageFileName('RalfWalkingMan.gif')
    discFile = gut.GetFusionImageFileName('SerDisc.gif')
    w = gut.ActiveImageWidget(subframe, 
            activesets={'disconnected':[discFile],
                        'automatic':autoFiles,
                        'manual':[manFile]},
            activetag='disconnected',
            period=0.25)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mWidgetActiveImage = w

    # read button
    w = tk.Button(subframe, text='Read', fg=gt.ColorBlack, width=8,
        command=self.CbRead)
    GuiToolTip.GuiToolTip(w, text="Read the robot's current position.")
    self.mButtonRead = w
    subcol += 1

    # column spacer
    tk.Label(subframe, width=1, text=' ').grid(row=subrow, column=subcol)

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Go to Manual Feet Updates"
    self.mTtAutoManAutoText = "Go to Automatic Feet Updates"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtAutoMan = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonAutoMan = w
    self.CtlPanelCfgAutoMan()

    subcol += 1

    # draw read button
    self.mButtonRead.grid(row=subrow, column=subcol, sticky=tk.W)

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)
    subcol += 1

    # raw/cooked data button
    w = tk.Button(subframe, fg=gt.ColorBlack, width=8, command=self.CbDataClass)
    w.grid(row=subrow, column=subcol)
    self.mTtDataClass = GuiToolTip.GuiToolTip(w, text="")
    self.mButtonDataClass = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # clear button
    w = tk.Button(subframe, text='Clear', fg=gt.ColorBlack, width=8,
        command=self.CbFeetClear)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Clear the feet data and visualization")

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # connect/disconnect button
    w = tk.Button(subframe, text='Connect', width=8, fg=gt.ColorBttnGo,
                                command=self.CbConn)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtConn= GuiToolTip.GuiToolTip(w,
        text="Connect to BotSense IP Proxy Server.")
    self.mButtonConn= w

    subcol += 1

    # close button
    w = tk.Button(subframe, text='Close', fg=gt.ColorBttnStop, width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0
 
    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 4
    if self.mFeetCanvasWidth > sbwidth:
      sbwidth = self.mFeetCanvasWidth

    # Status Bar
    self.mStatusBar = GuiStatusBar.GuiStatusBar(cpframe,
        [ 
          {'tag': 'run_time',
           'prefix': 'state:',
           'max_width': 11,
           'val': 'disconnected',
           'tooltip': "Visualizer's run-time state."
          },
          {'tag': 'proxy_addr',
           'prefix': 'proxy-addr:',
           'max_width': 32,
           'val': '',
           'tooltip': "BotSense Proxy's IP address."
          },
          {'tag': 'proxy_port',
           'prefix': 'proxy-port:',
           'max_width': 5,
           'val': 0,
           'fmt': '%d',
           'tooltip': "BotSense Proxy's TCP port."
          },
          {'tag': 'i2c_dev_name',
           'prefix': 'proxied-device:',
           'max_width': 12,
           'val': '',
           'tooltip': "Proxied feet I2C device name."
          },
          {'tag': 'foot_left_i2c',
           'prefix': 'left-foot I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "Left foot proxied I2C address."
          },
          {'tag': 'foot_right_i2c',
           'prefix': 'right-foot I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "Right foot proxied I2C address."
          },
          {'tag': 'data_class',
           'prefix': 'data-class:',
           'max_width': 6,
           'val': 'raw',
           'tooltip': "Foot data are raw values or calibrated (cooked)."
          },
        ],
        initWidth=sbwidth,
        maxRows=2)
    self.mStatusBar.grid(row=row, column=column, pady=3, sticky=tk.W)

    # set control panel initial state
    if self.mDataClass == BpDataClassRaw:
      self.CtlPanelStateRaw()
    else:
      self.CtlPanelStateCooked()

  #--
  def CtlPanelCfgAutoMan(self):
    """ Place control panel into automatic/manual update configuration. """
    w = self.mButtonAutoMan
    if self.mIsAutoMode == True:
      w['text']             = 'Manual'
      w['fg']               = gt.ColorBttnStop
      w['activeforeground'] = gt.ColorBttnStop
      self.mTtAutoMan.newtip(self.mTtAutoManManText)
      self.mButtonRead['state'] = tk.DISABLED
    else:
      w['text']             = 'Auto'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtAutoMan.newtip(self.mTtAutoManAutoText)
      self.mButtonRead['state'] = tk.NORMAL

  #--
  def CtlPanelStateRaw(self):
    """ Set Control Panel widgets' states to raw data class. """
    self.mButtonDataClass['text'] = 'Cooked'
    self.mTtDataClass.newtip("Read feet calibrated sensor data")


  #--
  def CtlPanelStateCooked(self):
    """ Set Control Panel widgets' states to cooked data class. """
    self.mButtonDataClass['text'] = 'Raw'
    self.mTtDataClass.newtip("Read feet raw sensor data")

  #--
  def CtlPanelStateConnected(self):
    """ Set Control Panel widgets' states to connected. """
    self.mButtonConn['state']             = tk.NORMAL
    self.mButtonConn['text']              = 'Disconnect'
    self.mButtonConn['fg']                = gt.ColorBttnStop
    self.mButtonConn['activeforeground']  = gt.ColorBttnStop
    self.mTtConn.newtip("Disconnect to BotSense IP Proxy Server.")


  #--
  def CtlPanelStateDisconnected(self):
    """ Set Control Panel widgets' states to disconnected. """
    self.mButtonConn['state']             = tk.NORMAL
    self.mButtonConn['text']              = 'Connect'
    self.mButtonConn['fg']                = gt.ColorBttnGo
    self.mButtonConn['activeforeground']  = gt.ColorBttnGo
    self.mTtConn.newtip("Connect to BotSense IP Proxy Server.")


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CbAutoMan(self):
    """ Automatic/Manual Viz canvas updates toggle callback. """
    if self.mIsAutoMode == True:
      self.mIsAutoMode  = False
    else:
      self.mIsAutoMode  = True
    self.CtlPanelCfgAutoMan()
    self.WinQueueRequest('cfg', run_time="update")

  #--
  def CbRead(self):
    """ Read feet callback. """
    if self.mTestGui:
      self.TestGuiSensorDataManual()
    else:
      self.BsSensorDataPull()

  #--
  def CbDataClass(self):
    """ Raw/Cooked data class callback. """
    if self.mDataClass == BpDataClassRaw:
      newClass = BpDataClassCooked
      self.CtlPanelStateCooked()
      self.WinQueueRequest('cfg', data_class="cooked")
    else:
      newClass = BpDataClassRaw
      self.CtlPanelStateRaw()
      self.WinQueueRequest('cfg', data_class="raw")
    self.mDataClass = newClass
    self.FeetCanvasRefresh()

  #--
  def CbFeetClear(self):
    """ Clear the viz's current set of data callback. """
    self.WinQueueRequest('clear')

  #--
  def CbLogData(self):
    """ Dump waypoints to file callback. """
    ltime = time.localtime()
    initialfile = 'wp_%d%02d%02d_%02d%02d%02d.py' % \
        (ltime[0], ltime[1], ltime[2], ltime[3], ltime[4], ltime[5])
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self, self._cbSave,
                  title='Save vKHR2 Waypoints As',
                  filetypes=[('Python data files', '*.py', 'TEXT'),
                             ('Text files', '*.txt', 'TEXT'),
                             ('All files', '*')],
                  initialfile=initialfile,
                  defaultextension='.py')

  #--
  def _cbSave(self, filename):
    """ Save As callback to actually save the waypoints.
        
        Parameters:
          filename  - file name to write the data
    """
    fp = open(filename, 'w')
    print('#', file=fp)
    print('# vKHR2 Feet Waypoints', file=fp)
    ltime = time.localtime()
    print('# %d.%02d.%02d %02d:%02d:%02d' % \
        (ltime[0], ltime[1], ltime[2], ltime[3], ltime[4], ltime[5]), file=fp)
    print('#\n', file=fp)
    print('# waypoint feet: (x(mm), y(mm), theta(radians))', file=fp)
    print('vKHR2 = [', end='', file=fp)
    n = 0
    sep = ''
    for p in self.mBotFeet:
      if n > 0:
        sep = ','
      if n % 2 == 0:
        sep += '\n  '
      else:
        sep += ' '
      print('%s(%.2f, %.2f, %.7f)' % (sep, p[X], p[Y], p[THETA]),
          end='', file=fp)
      n += 1
    print('\n]', file=fp)
    fp.close()

  #--
  def CbConn(self):
    """ Connect to bsproxy window callback. """
    self.mButtonConn['state'] = tk.DISABLED
    if self.mBsClient.mBsIsConn:
      self.WinQueueRequest('disconnect')
    else:
      self.WinQueueRequest('connect')
    self.WinQueueRequest('cfg', run_time="update")

  #--
  def CbClose(self):
    """ Close window callback. """
    self.IVCancel()
    self.destroy()
 
  #--
  def CbCfgBsProxy(self):
    """ Configure bsproxy callback. """
    keys = GuiDlgKHR2Proxy.GetSettingNames()

    last = {}
    for k in keys:
      if k == 'proxy_addr':
        last[k] = self.mBsClient.mBsProxyAddr
      elif k == 'proxy_port':
        last[k] = self.mBsClient.mBsProxyPort
      elif k == 'i2c_dev_name':
        last[k] = self.mI2CDevName
      elif k == 'bpfoot_left':
        last[k] = {
            'enable':self.mBpFootState['bpfoot_left'], 
            'i2c_addr': self.mBsClient.mBsProxiedDev['bpfoot_left']['i2c']}
      elif k == 'bpfoot_right':
        last[k] = {
            'enable':self.mBpFootState['bpfoot_right'], 
            'i2c_addr': self.mBsClient.mBsProxiedDev['bpfoot_right']['i2c']}

    dlg = GuiDlgKHR2Proxy.GuiDlgKHR2Proxy(self, lastSettings=last)

    if dlg.result:
      for k,v in dlg.result.iteritems():
        if k == 'proxy_addr' and v:
          self.mBsClient.SetProxyAddr(v)
        elif k == 'proxy_port' and v:
          self.mBsClient.SetProxyPort(v)
        elif k == 'i2c_dev_name':
          self.mI2CDevName = v
        elif k == 'bpfoot_left':
          self.mBpFootState['bpfoot_left'] = v['enable']
          self.mBsClient.mBsProxiedDev['bpfoot_left']['i2c'] = v['i2c_addr']
        elif k == 'bpfoot_right':
          self.mBpFootState['bpfoot_right'] = v['enable']
          self.mBsClient.mBsProxiedDev['bpfoot_right']['i2c'] = v['i2c_addr']

      self.FeetCanvasRefresh()
      self.WinQueueRequest('cfg',
          proxy_addr=self.mBsClient.mBsProxyAddr,
          proxy_port=self.mBsClient.mBsProxyPort,
          foot_left_i2c=self.mBsClient.mBsProxiedDev['bpfoot_left']['i2c'],
          foot_right_i2c=self.mBsClient.mBsProxiedDev['bpfoot_right']['i2c'],
          i2c_dev_name=self.mI2CDevName)

      print("Left foot enabled = ", self.mBpFootState['bpfoot_left'])
      print("Right foot enabled = ", self.mBpFootState['bpfoot_right'])

  #--
  def CbI2CScan(self):
    if self.mBsClient.mBsIsConn:
      for proxdev,proxdata in self.mBsClient.mBsProxiedDev.iteritems():
        handle = proxdata['handle']
        if handle is not None:
          break;
      if handle is not None:
        scanned = self.mBsClient.CmdI2CScan(proxdev)
        if not scanned:
          hdr = 'No I' + gt.UniSuperscript['2'] + 'C devices found.'
          txt = ''
        else:
          hdr = "Discovered %d I" % (len(scanned)) + gt.UniSuperscript['2'] + \
              'C Devices.'
          txt = '\n'
          i = 0;
          for dev in scanned:
            txt += "0x%02x " % dev
            i += 1
            if (i % 16) == 0:
              txt += '\n'
      else:
        hdr = 'No opened proxied I' + gt.UniSuperscript['2'] + \
              'C device found to scan.'
        txt = ''
    else:
      hdr = 'No connection with BotSense Proxy Server'
      txt = ''

    GuiDlgAbout.GuiDlgAbout(self, 
                name="Scanned Proxied I" + gt.UniSuperscript['2'] + 'C Devices',
                desc=hdr+txt)

  #--
  def CbFootIds(self):
    hdr = "Foot   Device ID   Firmware Version\n" \
          "----   ---------   ----------------\n"
    txt = ''
    if self.mBsClient.mBsIsConn:
      for whichfoot in BpFeet:
        if self.mBpFootState[whichfoot]:
          ids = self.mBsClient.CmdBpFoot(whichfoot, 'getids')
          if ids:
            if whichfoot == 'bpfoot_left':
              txt += 'Left    '
            else:
              txt += 'Right   '
            txt += "0x%04x          0x%02x\n" % \
                (ids['device_id'], ids['version'])
    GuiDlgAbout.GuiDlgAbout(self, 
                name="BrainPack Foot IDs",
                desc=hdr+txt)

  #--
  def CbFootCal(self):
    if self.mBsClient.mBsIsConn:
      for whichfoot in BpFeet:
        if self.mBpFootState[whichfoot]:
          self.mBsClient.CmdBpFoot(whichfoot, 'setcal')

  #--
  def CbTestGui(self):
    """ Test Gui menu callback. """
    if self.mTestGui:
      self.mTestGui = False
      self.mMenuBar['Tools|Go Live']['label'] = 'Test Gui'
      print('Going live, testing the Gui is off')
    else:
      self.mTestGui = True
      self.mMenuBar['Tools|Test Gui']['label'] = 'Go Live'
      print('Testing the Gui')


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Interval Timer Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IVClient(self, ivt):
    """ BS Client IV callback. """
    if ivt.once:
      # add what you need
      ivt.once = False # finish one-time initializations
      return
    elif not self.mIsAutoMode:
        return
    elif self.mTestGui:
      self.TestGuiSensorDataAuto()
    elif self.mBsClient.mBsIsConn:
      self.BsSensorDataPull()

  #--
  def IVCancel(self):
    if self.mIvtPull:
      self.mIvtPull.cancel()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Sensor Data Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BsSensorDataPull(self):
    for whichfoot in BpFeet:
      if self.mBpFootState[whichfoot]:
        if self.mDataClass == BpDataClassRaw:
          data = self.mBsClient.CmdBpFoot(whichfoot, 'getraw')
        else:
          data = self.mBsClient.CmdBpFoot(whichfoot, 'getcooked')
        if data:
          self.WinQueueRequest(whichfoot, **data)

  #--
  def TestGuiSensorDataAuto(self):
    data = {}
    if self.mDataClass == BpDataClassRaw:
      for whichfoot in BpFeet:
        if not self.mBpFootState[whichfoot]:
          continue
        data[whichfoot] = [0] * BpFootNumOfFootSensors
        for i in range(BpFootNumOfFootSensors):
          v = random.randint(0,BpFootSensorMax)
          data[whichfoot][i] = v
        self.WinQueueRequest(whichfoot, raw_data=data[whichfoot])
    elif self.mDataClass == BpDataClassCooked:
      for whichfoot in BpFeet:
        if not self.mBpFootState[whichfoot]:
          continue
        data[whichfoot] = [0] * BpFootNumOfFootSensors
        for i in range(BpFootNumOfFootSensors):
          v = random.randint(BpFootSensorMax/2,BpFootSensorMax)
          data[whichfoot][i] = v
        vec_dir, vec_mag = self.TestGuiCalcVector(data[whichfoot])
        self.WinQueueRequest(whichfoot, cooked_data=data[whichfoot],
                                        vec_mag=vec_mag, vec_dir=vec_dir)

  #--
  def TestGuiSensorDataManual(self):
    data1 = [0] * BpFootNumOfFootSensors
    data2 = [0] * BpFootNumOfFootSensors
    if self.mDataClass == BpDataClassRaw:
      stepsize = (BpFootSensorMax + 1) / (BpFootNumOfFootSensors * 2)
      for i in range(BpFootNumOfFootSensors):
        v = i * stepsize
        data1[i] = v
      self.WinQueueRequest('bpfoot_left', raw_data=data1)
      offset = stepsize * BpFootNumOfFootSensors
      for i in range(7):
        v = offset + i * stepsize
        data2[i] = v
      data2[7] = BpFootSensorMax
      self.WinQueueRequest('bpfoot_right', raw_data=data2)
    else:
      stepsize = (BpFootSensorMax + 1) / (BpFootNumOfFootSensors * 4)
      for i in range(BpFootNumOfFootSensors):
        v = i * stepsize
        data1[i] = v
      self.WinQueueRequest('bpfoot_left', cooked_data=data1,
          vec_dir=-90, vec_mag=BpFootVecMagMax/2)
      offset = stepsize * BpFootNumOfFootSensors
      for i in range(BpFootNumOfFootSensors):
        v = 128 + i * stepsize
        data2[i] = v
      self.WinQueueRequest('bpfoot_right', cooked_data=data2,
          vec_dir=0, vec_mag=BpFootVecMagMax)

  #--
  def TestGuiCalcVector(self, sensor):
    d1 = 1.7
    d2 = 0.64
    angle = 1.121

    S1X = (-1) * sensor[0] * d1 * math.cos(angle)
    S1Y =        sensor[0] * d1 * math.sin(angle)

    S2X =        sensor[1] * d1 * math.cos(angle)
    S2Y =        sensor[1] * d1 * math.sin(angle)

    S3X = (-1) * sensor[2] * d2 * math.cos(0)
    S3Y =        sensor[2] * d2 * math.sin(0)

    S4X =        sensor[3] * d2 * math.cos(0)
    S4Y =        sensor[3] * d2 * math.sin(0)

    S5X = (-1) * sensor[4] * d1 * math.cos(angle)
    S5Y = (-1) * sensor[4] * d1 * math.sin(angle)

    S6X =        sensor[5] * d1 * math.cos(angle)
    S6Y = (-1) * sensor[5] * d1 * math.sin(angle)

    FX = S1X + S2X + S3X + S4X + S5X + S6X
    FY = S1Y + S2Y + S3Y + S4Y + S5Y + S6Y

    if FX != 0.0:
      mag = int((math.sqrt((FX*FX) + (FY*FY))))
      dir = int(((math.atan(FY/FX) * 180.0) / math.pi))
      if FX < 0:
        dir += 180
      elif FY < 0:
        dir += 360
    else:
      mag = 0
      dir = 0

    return (dir, mag)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Messy Details
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _foot(self, whichfoot):
    """ Paint the Feet canvas background plus sensor values"""

    toedim    = self.mCoord[whichfoot]['toe_dim']
    toemidpt  = self.mCoord[whichfoot]['toe_midpt']
    footdim   = self.mCoord[whichfoot]['foot_dim']
    footmidpt = self.mCoord[whichfoot]['foot_midpt']

    if whichfoot == 'bpfoot_left':
      prefix = 'Left'
    else:
      prefix = 'Right'

    # toe
    id = self.mFeetCanvas.create_rectangle(toedim,
                                          outline=gt.ColorBlack)
    self.mGidFeet += [id]

    dim = toemidpt[0], toedim[1] + 2
    id = self.mFeetCanvas.create_text(dim, fill=gt.ColorBlue1,
          text=prefix+' Toe', anchor=tk.N)
    self.mGidFeet += [id]

    # foot
    id = self.mFeetCanvas.create_rectangle(footdim,
                                          outline=gt.ColorBlack)
    self.mGidFeet += [id]

    id = self.mFeetCanvas.create_text((footmidpt[0], footdim[1]+2),
              fill=gt.ColorBlue1, text=prefix+' Foot', anchor=tk.N)
    self.mGidFeet += [id]

    r = self.mFootSensorRadius

    iSensor = 0

    # foot sensors
    while iSensor < BpFootNumOfFootSensors:
      c = self.mCoord[whichfoot]['sensor'][iSensor]

      # left column sensor
      dim = c[0] - r, c[1] - r, c[0] + r, c[1] + r
      id = self.mFeetCanvas.create_oval(dim, fill=gt.ColorGray2, 
                                                   outline=gt.ColorBlack)
      self.mGidSensor[whichfoot]['oval'] += [id]

      # left sensor column
      if (iSensor % 2) == 0:
        dim = dim[2] + 2, c[1]
        id = self.mFeetCanvas.create_text(dim, fill=gt.ColorWhite,
                text='0', anchor=tk.W)
        self.mGidSensor[whichfoot]['text'] += [id]
      # right sensor column
      else:
        dim = dim[0] - 2, c[1]
        id = self.mFeetCanvas.create_text(dim, fill=gt.ColorWhite,
              text='0', anchor=tk.E)
        self.mGidSensor[whichfoot]['text'] += [id]

      iSensor += 1

  #--
  def _forcevector(self, whichfoot, mag, dir):
    """ Draw a foot's force vector.

        Parameters:
          whichfoot   - left or right key
          mag         - vector length [0,BpFootVecMagMax]
          dir         - vector angle [-180,180] (degrees)
    """
    footdim   = self.mCoord[whichfoot]['foot_dim']
    footmidpt = self.mCoord[whichfoot]['foot_midpt']

    if mag > 0:
      # keep torque vector visualization within displayed foot boundries
      rMax = float(footdim[2] - footmidpt[0]) - 2.0

      # radius in pixels (mulipling with a fudge factor)
      r = int(2.0 * rMax * float(mag)/BpFootVecMagMax)

      # rotate 90 for torque direction
      dir += 90
      dir %= 360

      theta = math.radians(dir)

      x1 = int(footmidpt[0] + math.cos(theta) * r)
      y1 = int(footmidpt[1] - math.sin(theta) * r)
      dim = footmidpt[0], footmidpt[1], x1, y1

      id = self.mFeetCanvas.create_line(dim, width=3, arrow=tk.LAST,
                                      fill=gt.ColorPink1)
      self.mGidSensor[whichfoot]['force_vec'] += [id]

    x0 = footmidpt[0] - 2
    y0 = footmidpt[1] - 2
    x1 = footmidpt[0] + 2
    y1 = footmidpt[1] + 2
    dim = x0, y0, x1, y1
    id = self.mFeetCanvas.create_oval(dim, fill="#9999ff",
                                          outline=gt.ColorBlack)

    self.mGidSensor[whichfoot]['force_vec'] += [id]

  #--
  def _centerofmass(self, whichfoot, sensors):
    """ Draw a foot's center of masses (sole and toes).

        Parameters:
          whichfoot   - left or right key
          sensors     - vector of sensor data
    """
    M     = 0.0   # total 'mass'
    x_com = 0.0   # x's center of mass
    y_com = 0.0   # y's center of mass

    # x center of mass, plus total mass
    sign = 1.0
    for i in range(0,6):
      x_com += sign * sensors[i] * BpFootPtFulcrum[0]
      M  += sensors[i]
      sign *= -1.0

    # y center of mass
    y_com = (sensors[BpFootSensorUppL] + sensors[BpFootSensorUppR] - \
             sensors[BpFootSensorLowL] - sensors[BpFootSensorLowR]) * \
             BpFootPtFulcrum[1]

    # map real-world coordinates to screen
    if M > 0.0:
      x_com /= M
      y_com /= M

      footmidpt = self.mCoord[whichfoot]['foot_midpt']

      maxx = float(footmidpt[0] - self.mCoord[whichfoot]['sensor'][0][0])
      maxy = float(footmidpt[1] - self.mCoord[whichfoot]['sensor'][0][1])

      p = (footmidpt[0] - int(maxx * x_com/BpFootPtFulcrum[0]),
           footmidpt[1] - int(maxy * y_com/BpFootPtFulcrum[1]))
      r = int(20.0 * M/(BpFootSensorMax*6.0))

      x0 = p[0] - r
      y0 = p[1] - r
      x1 = p[0] + r
      y1 = p[1] + r

      dim = x0, y0, x1, y1
      id = self.mFeetCanvas.create_oval(dim, fill=gt.ColorRed1,
                                          outline=gt.ColorBlack)

      self.mGidSensor[whichfoot]['com'] += [id]

    #
    # toe
    # 

    M     = 0.0   # total 'mass'
    x_com = 0.0   # x's center of mass
    y_com = 0.0   # y's center of mass

    # x center of mass, plus total mass
    sign = 1.0
    for i in range(6,8):
      x_com += sign * sensors[i] * BpFootPtFulcrum[0]
      M  += sensors[i]
      sign *= -1.0

    # map real-world coordinates to screen
    if M > 0.0:
      x_com /= M

      toemidpt = self.mCoord[whichfoot]['toe_midpt']

      maxx = float(toemidpt[0] - self.mCoord[whichfoot]['sensor'][0][0])

      p = (toemidpt[0] - int(maxx * x_com/BpFootPtFulcrum[0]), toemidpt[1])
      r = int(10.0 * M/(BpFootSensorMax*2.0))

      x0 = p[0] - r
      y0 = p[1] - r
      x1 = p[0] + r
      y1 = p[1] + r

      dim = x0, y0, x1, y1
      id = self.mFeetCanvas.create_oval(dim, fill=gt.ColorRed1,
                                          outline=gt.ColorBlack)

      self.mGidSensor[whichfoot]['com'] += [id]


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  
  import Fusion.Core.Gluon as Gluon
  import Fusion.Utils.WinUT as WinUT
  #import Fusion.KHR2.Robots.vKHR2 as vKHR2

  #--
  class KHR2FeetUT:
    """ KHR-2 Feet Unit Tester. """

    #--
    def __init__(self, robot, sut):
      self.mRobot   = None
      self.mSut     = sut


  #--
  def main():
    """ GuiWinKHR2Feet Unit Test Main """
    root = tk.Tk()
    sut = GuiWinKHR2Feet(root)    # system under test
    ut = KHR2FeetUT(None, sut)    # unit tester
    root.mainloop()
    sut.IVCancel()

  # run unit test
  main()
