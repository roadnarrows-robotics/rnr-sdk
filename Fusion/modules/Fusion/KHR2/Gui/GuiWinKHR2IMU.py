################################################################################
#
# GuiWinKHR2IMU
#

""" Graphical User Interface KHR-2 BrainPack Inertia Measurement Unit Window

Graphical User Interface (GUI) Tkinter window displays current
RoadNarrows BrainPack Inertia measurement unit sensor data, 
plus simple display options.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.11.13

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

import tkinter as tk

import Fusion.Core.Values as Values

import Fusion.Utils.IVTimer as IVTimer

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiMenuBar as GuiMenuBar
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import Fusion.Gui.GuiXYGraph as GuiGraph
import Fusion.Gui.GuiWin as GuiWin

import Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient
import Fusion.KHR2.Gui.GuiDlgKHR2Proxy as GuiDlgKHR2Proxy

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# BrainPack Data Classes
BpDataClassRaw     = 0
BpDataClassCooked  = 1

# BrainPack IMU
BpIMUSensorMax   = 255        # max. sensor data value
BpIMUSensorMask  = 0x00ff     # sensor mask

# number of sensors 
BpIMUNumOfSensors   = 3

# IMU sensor index order
BpIMUSensorX  = 0     # x accellerometer
BpIMUSensorY  = 1     # y accellerometer
BpIMUSensorZ  = 2     # z accellerometer

BpIMUMinG         = -1.5          # Memsic minimum measurable g acceleration
BpIMUMaxG         = 1.5           # Memsic maximum measurable g acceleration
BpIMUSensitivity  = 1.0/.5        # Memsic g's per volt (500mv/g)
BpIMUVref         = 3.3           # IMU 3.3 reference voltage to Memsic
BpIMUVzero        = BpIMUVref/2.0 # Memsic zero-g voltage reference
BpADCVref         = 5.0           # Atmega16 ADC voltage reference
BpADCMax          = 1023          # max ADC value (10-bit)
BpADCMult         = BpADCVref / float(BpADCMax+1)
                                  # multiplier to get Vin (Memsic Vout)

# IMU canvas dimensions
_GraphXYWidth   = 320   # x-y graph canvas width
_GraphXZWidth   = 320   # x-z graph canvas width
_GraphHeight    = 320   # single graph canvas height
_GraphTotWidth  = _GraphXYWidth + _GraphXZWidth 
_EdgeLeft       = 5     # left edge margin
_EdgeTop        = 5     # top edge margin
_EdgeBottom     = 5     # bottom edge margin
_EdgeRight      = 5     # right edge margin

# minimum size
_CanvasMinWidth   = _GraphXYWidth + _GraphXZWidth + _EdgeLeft + _EdgeRight
_CanvasMinHeight  = _GraphHeight + _EdgeTop + _EdgeBottom

twopi = math.pi * 2.0

#-------------------------------------------------------------------------------
# CLASS: GuiWinKHR2IMU
#-------------------------------------------------------------------------------
class GuiWinKHR2IMU(GuiWin.GuiWin):
  """ GUI Window vKHR2 Inertia Measurement Unit Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the vKHR2 IMU Window.

        Parameters:
          parent      - GUI parent of this window
          options     - IMU options. Options are:
            auto=<bool>       - do [not] automatically update
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = \
                              'vKHR2 RoadNarrows BrainPack IMU Visualizer'
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
        'command', owner='root', command=self.CbIMUIds)
    self.mMenuBar.AddMenuItem('Tools|Calibrate',
        'command', owner='root', command=self.CbIMUCal)
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
          options   - IMU input options.

        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode        = False
    self.mDataClass         = BpDataClassRaw
    self.mBsClient          = BsProxyClient.BsProxyClient()
    self.mBpIMUState        = False
    self.mI2CDevName        = '/dev/i2c/0'  # default for KoreBot
    self.mTestGui           = False

    # set options from input parameters
    for k,v in options.items():
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
      width = geo[gut.W] - self.mWinBorder
      height = geo[gut.H] - self.mWinBorder - self.mCtlPanelFrameHeight
      self.mXYGrapher.configure(width*_GraphXYWidth/_GraphTotWidth, height)
      self.mXZGrapher.configure(width*_GraphXYWidth/_GraphTotWidth, height)
    # resizing done, now redraw
    elif self.mHasResized:
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
          'cfg'           - configure BrainPack IMU window.
          'bpimu'         - IMU sensor data
          'clear'         - clear current set displayed data
          'connect'       - open connection to bsproxy
          'disconnect'    - close connection to bsproxy

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'bpimu':
      self.WinUpdateIMU(**kwargs)
    elif request == 'cfg':
      self.WinUpdateStatus(**kwargs)
    elif request == 'clear':
      self.WinUpdateClear()
    elif request == 'connect':
      self.WinUpdateConnect()
    elif request == 'disconnect':
      self.WinUpdateDisconnect()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateIMU(self, raw_data=None, cooked_data=None,
                         device_id=None, version=None):
    """ Update IMU sensor data

        Execution Context: GuiWin server thread

        Parameters:

        Return Value:
          None.
    """
    if self.mDataClass == BpDataClassRaw:
      if raw_data is not None:
        self.mXYGrapher.newdata(
            xdata=[0, raw_data[BpIMUSensorX]],
            ydata=[0, raw_data[BpIMUSensorY]])
        self.mXZGrapher.newdata(
            xdata=[0, raw_data[BpIMUSensorX]],
            ydata=[0, raw_data[BpIMUSensorZ]])
    elif self.mDataClass == BpDataClassCooked:
      if cooked_data is not None:
        self.mXYGrapher.newdata(
            xdata=[0, cooked_data[BpIMUSensorX]],
            ydata=[0, cooked_data[BpIMUSensorY]])
        self.mXZGrapher.newdata(
            xdata=[0, cooked_data[BpIMUSensorX]],
            ydata=[0, cooked_data[BpIMUSensorZ]])

  #--
  def WinUpdateStatus(self, **kwargs):
    """ Update status bar with IMU data.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              run_time=<str>        - run-time state
              proxy_addr=<str>      - botsense proxy IP address
              proxy_port=<int>      - botsense proxy TCP port
              imu_i2c=<int>         - IMU I2C address
              data_class=<class>    - data class (raw/cooked)

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.items():
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
      elif k == 'imu_i2c':
        items[k] = v
      elif k == 'data_class':
        items[k] = v
    self.mStatusBar.Update(**items)

  #--
  def WinUpdateClear(self):
    """ Clear the IMU's current set of waypoints and refresh the window.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.mXYGrapher.redraw()
    self.mXYGrapher.newdata(xdata=[0], ydata=[0])
    self.mXZGrapher.redraw()
    self.mXZGrapher.newdata(xdata=[0], ydata=[0])

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

    # configure IMU proxied device
    if self.mBpIMUState:
      i2caddr = self.mBsClient.mBsProxiedDev['bpimu']['i2c']
      self.mBsClient.CmdDevOpen('bpimu', i2caddr, self.mI2CDevName)
    self.CtlPanelStateConnected()

  #--
  def WinUpdateDisconnect(self):
    """ Close connection to the BotSense IP Proxy Server.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.mBsClient.Disconnect()
    self.CtlPanelStateDisconnected()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # IMU Window Specifics
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

    self.mWinBorder = self.mWinGeo[gut.W] - _GraphTotWidth

    # set window's minimum size (250 obtained through experimentation)
    self.wm_minsize(
        width=_GraphTotWidth+self.mWinBorder,
        height=_GraphHeight+self.mWinBorder+self.mCtlPanelFrameHeight
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
    col = 0

    # subframe to hold all graphing canvases
    subframe = tk.Frame(parent, relief=tk.FLAT, borderwidth=0)
    subframe.grid(row=row, column=col, padx=3, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # x-y graph
    self.mCanvasXY = tk.Canvas(subframe, 
        width=_GraphXYWidth, height=_GraphHeight)
    self.mCanvasXY.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mXYGrapher = GuiGraph.GuiXYGraph(self.mCanvasXY,
        title='X - Y Acceleration (g)',
        xlabel='x',
        ylabel='y',
        xstep=0.5, domain=(-1.5, 1.5), ystep=0.5, range=(-1.5, 1.5),
        linewidth=3, linecolor=gt.ColorRed1, showpoints=True,
        xdata=[0], ydata=[0])

    subcol += 1

    # x-z graph
    self.mCanvasXZ = tk.Canvas(subframe, 
        width=_GraphXZWidth, height=_GraphHeight)
    self.mCanvasXZ.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mXZGrapher = GuiGraph.GuiXYGraph(self.mCanvasXZ,
        title='X - Z Acceleration (g)',
        xlabel='x',
        ylabel='z',
        xstep=0.5, domain=(-1.5, 1.5), ystep=0.5, range=(-1.5, 1.5),
        linewidth=3, linecolor=gt.ColorRed1, showpoints=True,
        xdata=[0], ydata=[0])

    row += 1

    self.CtlPanelInit(parent, row, col)

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the IMU Control Panel.

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
    w = tk.Label(cpframe, text='vKHR2 BrainPack IMU Control Panel',
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
    self.mTtAutoManManText  = "Go to Manual IMU Updates"
    self.mTtAutoManAutoText = "Go to Automatic IMU Updates"
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
        command=self.CbIMUClear)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Clear the IMU data and visualization")

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
           'tooltip': "Proxied IMU I2C device name."
          },
          {'tag': 'imu_i2c',
           'prefix': 'IMU I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "IMU proxied I2C address."
          },
          {'tag': 'data_class',
           'prefix': 'data-class:',
           'max_width': 6,
           'val': 'raw',
           'tooltip': "IMU data are raw values or calibrated (cooked)."
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
    self.mTtDataClass.newtip("Read IMU calibrated sensor data")


  #--
  def CtlPanelStateCooked(self):
    """ Set Control Panel widgets' states to cooked data class. """
    self.mButtonDataClass['text'] = 'Raw'
    self.mTtDataClass.newtip("Read IMU raw sensor data")

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
    """ Read IMU callback. """
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

  #--
  def CbIMUClear(self):
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
    print('# vKHR2 IMU Waypoints', file=fp)
    ltime = time.localtime()
    print('# %d.%02d.%02d %02d:%02d:%02d' % \
        (ltime[0], ltime[1], ltime[2], ltime[3], ltime[4], ltime[5]), file=fp)
    print('#\n', file=fp)
    print('# waypoint IMU: (x(mm), y(mm), theta(radians))', file=fp)
    print('vKHR2 = [', end='', file=fp)
    n = 0
    sep = ''
    for p in self.mBotIMU:
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
      elif k == 'bpimu':
        last[k] = {'enable':self.mBpIMUState,
                  'i2c_addr': self.mBsClient.mBsProxiedDev[k]['i2c']}

    dlg = GuiDlgKHR2Proxy.GuiDlgKHR2Proxy(self, lastSettings=last)

    if dlg.result:
      for k,v in dlg.result.items():
        if k == 'proxy_addr' and v:
          self.mBsClient.SetProxyAddr(v)
        elif k == 'proxy_port' and v:
          self.mBsClient.SetProxyPort(v)
        elif k == 'i2c_dev_name':
          self.mI2CDevName = v
        elif k == 'bpimu':
          self.mBpIMUState = v['enable']
          self.mBsClient.mBsProxiedDev['bpimu']['i2c'] = v['i2c_addr']

      self.WinQueueRequest('cfg',
          proxy_addr=self.mBsClient.mBsProxyAddr,
          proxy_port=self.mBsClient.mBsProxyPort,
          bpimu=self.mBsClient.mBsProxiedDev['bpimu']['i2c'],
          i2c_dev_name=self.mI2CDevName)

      print("IMU enabled = ", self.mBpIMUState)

  #--
  def CbI2CScan(self):
    if self.mBsClient.mBsIsConn:
      for proxdev,proxdata in self.mBsClient.mBsProxiedDev.items():
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
  def CbIMUIds(self):
    hdr = "Device ID   Firmware Version\n" \
          "---------   ----------------\n"
    txt = ''
    if self.mBsClient.mBsIsConn:
      if self.mBpIMUState:
        ids = self.mBsClient.CmdBpIMU('getids')
        if ids:
          txt += "0x%04x          0x%02x\n" % (ids['device_id'], ids['version'])
    GuiDlgAbout.GuiDlgAbout(self, 
                name="BrainPack IMU IDs",
                desc=hdr+txt)

  #--
  def CbIMUCal(self, orientation=0):
    if self.mBsClient.mBsIsConn:
      if self.mBpIMUState:
        self.mBsClient.CmdBpIMU('setcal', orientation)

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
    if not self.mBpIMUState:
      return
    elif self.mDataClass == BpDataClassRaw:
      rsp = self.mBsClient.CmdBpIMU('getraw')
      k = 'raw_data'
    else:
      rsp = self.mBsClient.CmdBpIMU('getcooked')
      k = 'cooked_data'
    if rsp:
      rsp[k] = self.ADCtoG(rsp[k])
      self.WinQueueRequest('bpimu', **rsp)
      #print(rsp)

  #--
  def ADCtoG(self, adclist):
    """ Convert IMU ADC values to acceleration in g's. """
    g = []
    for adc in adclist:
      adc <<= 2 # only the upper 8-bits of 10-bits are read (low pass filter)
      v_in = float(adc) * BpADCMult   # convert AD to volts (Memsic Vout)
      g += [(v_in - BpIMUVzero) * BpIMUSensitivity] # convert volts to g's
    return g

  #--
  def TestGuiSensorDataAuto(self):
    if not self.mBpIMUState:
      return
    elif self.mDataClass == BpDataClassRaw:
      data = [0] * BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = random.randint(0,BpIMUSensorMax)
        data[i] = v
      self.WinQueueRequest('bpimu', raw_data=self.ADCtoG(data))
    elif self.mDataClass == BpDataClassCooked:
      data = [0] * BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = random.randint(0,BpIMUSensorMax)
        data[i] = v
      self.WinQueueRequest('bpimu', cooked_data=self.ADCtoG(data))

  #--
  def TestGuiSensorDataManual(self):
    data = [0] * BpIMUNumOfSensors
    if self.mDataClass == BpDataClassRaw:
      stepsize = (BpIMUSensorMax + 1) / BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = i * stepsize
        data[i] = v
      self.WinQueueRequest('bpimu', raw_data=self.ADCtoG(data))
    else:
      stepsize = (BpIMUSensorMax + 1) / (BpIMUNumOfSensors * 2)
      for i in range(BpIMUNumOfSensors):
        v = i * stepsize
        data[i] = v
      self.WinQueueRequest('bpimu', cooked_data=self.ADCtoG(data))


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  
  import Fusion.Core.Gluon as Gluon
  import Fusion.Utils.WinUT as WinUT
  #import Fusion.KHR2.Robots.vKHR2 as vKHR2

  #--
  class KHR2IMUUT:
    """ KHR-2 IMU Unit Tester. """

    #--
    def __init__(self, robot, sut):
      self.mRobot   = None
      self.mSut     = sut


  #--
  def main():
    """ GuiWinKHR2IMU Unit Test Main """
    root = tk.Tk()
    sut = GuiWinKHR2IMU(root)    # system under test
    ut = KHR2IMUUT(None, sut)    # unit tester
    root.mainloop()
    sut.IVCancel()

  # run unit test
  main()
