################################################################################
#
# GuiWinKHR2StandTall
#

""" Graphical User Interface KHR-2 Simple PID control to stand tall.

Graphical User Interface (GUI) Tkinter window displays current KHR-2 info
and PID settings.

The Stand Tall algorithm uses PIDs to keep the KHR-2 robot standing upright,
while lateral and varied forces are applied to the robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2008.07.18

Copyright (C) 2008.  RoadNarrows LLC.
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

import Fusion.KHR2.Cmd.BsProxyMsgDef as BsProxyMsgDef
import Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient
import Fusion.KHR2.Gui.GuiDlgKHR2Proxy as GuiDlgKHR2Proxy
import Fusion.KHR2.Gui.GuiDlgKHR2ProxySimple as GuiDlgKHR2ProxySimple

import Fusion.Acc.Pid as Pid


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# Servo Ports
ServoNeck       =  0 
ServoLHipAbd    = 10
ServoLHip       = 11
ServoLKnee      = 12
ServoLAnkle     = 13
ServoLAnkleLat  = 14
ServoRHipAbd    = 16
ServoRHip       = 17
ServoRKnee      = 18
ServoRAnkle     = 19
ServoRAnkleLat  = 20

# Servo Normalized Directions
DirLHipAbd      = 1.0 #-1.0
DirLHip         = 1.0 #-1.0
DirLKnee        =  1.0
DirLAnkle       = 1.0 #-1.0
DirLAnkleLat    = 1.0 #-1.0
DirRHipAbd      =  1.0
DirRHip         = 1.0 #-1.0
DirRKnee        = 1.0 #-1.0
DirRAnkle       =  1.0
DirRAnkleLat    =  1.0

#
# BrainPack Feet
#
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

#
# BrainPack IMU
#

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

#
# BrainPack Sensors
#
BpSensors         = BpFeet + ['bpimu'] # keys

#
# Math
#
twopi = math.pi * 2.0

#-------------------------------------------------------------------------------
# CLASS: GuiWinKHR2StandTall
#-------------------------------------------------------------------------------
class GuiWinKHR2StandTall(GuiWin.GuiWin):
  """ GUI Window vKHR2 Inertia Measurement Unit Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the vKHR2 StandTall Window.

        Parameters:
          parent      - GUI parent of this window
          options     - StandTall options. Options are:
            auto=<bool>        - do [not] automatically start running
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = \
                              'vKHR2 RoadNarrows BrainPack StandTall Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  def _initMenuBar(self):
    """ Initialize menubar. """
    # File menubar items
    self.mMenuBar.AddMenuItem('File', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('File|Log...', 'command', owner='root',
        command=self.CbLogData)
    self.mMenuBar.AddMenuItem('File|Exit', 'command', owner='root',
        command=self.destroy)

    self.mMenuBar.AddMenuItem('Configure', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Configure|RCB3 bsproxy...',
        'command', owner='root', command=self.CbCfgBsProxyRCB3)
    self.mMenuBar.AddMenuItem('Configure|Sensors bsproxy...',
        'command', owner='root', command=self.CbCfgBsProxyI2C)

    self.mMenuBar.AddMenuItem('Tools', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Tools|IDs',
        'command', owner='root', command=self.CbStandTallIds)
    self.mMenuBar.AddMenuItem('Tools|Calibrate',
        'command', owner='root', command=self.CbStandTallCal)
    self.mMenuBar.AddMenuItem('Tools', 'separator')
    self.mMenuBar.AddMenuItem('Tools|I' + gt.UniSuperscript['2']+'C Scan...',
        'command', owner='root', command=self.CbI2CScan)
    self.mMenuBar.AddMenuItem('Tools', 'separator')
    self.mMenuBar.AddMenuItem('Tools|Test Gui',
        'command', owner='root', command=self.CbTestGui)

  #--
  def _mkcallback(self, servo):
    return lambda: self.CbServoEnable(servo=servo)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          vKHR2     - vKHR2 object.
          options   - StandTall input options.

        Return Value:
          None
    """
    # defaults
    self.mIsConn            = False
    self.mIsRunning         = False
    self.mTestGui           = False
    self.mDataClass         = BpDataClassRaw
    self.mBsClientSensors   = BsProxyClient.BsProxyClient()
    self.mBsClientRCB3      = BsProxyClient.BsProxyClient()
    self.mDevNameI2C        = '/dev/i2c/0'    # default for KoreBot
    self.mDevNameRCB3       = '/dev/ttyUSB0'  # default for KoreBot
    self.mBpSensorState     = {'bpfoot_left': False,
                               'bpfoot_right': False,
                               'bpimu': False}

    # set options from input parameters
    for k,v in options.iteritems():
      if k == 'run':
        if v:
          self.mIsRunning = True
        else:
          self.mIsRunning = False
      elif k == 'data_class':
        self.mDataClass = v

    # locals
    #self.mIvtPull = IVTimer.IVTimer(1.50, 0.25, self.IVClient, once=True)
    #self.mIvtPull.start()

    self.mDB = {}

    dbYawPid = []
    for pidvar,pidval in [('Kp', 0.05), ('Ki', 0.0), ('Kd', 0.1),
                          ('CVmin', -20.0), ('CVmax', 20.0)]:
      pdata = {'label': pidvar, pidvar: tk.DoubleVar()}
      pdata[pidvar].set(pidval)
      dbYawPid.append(pdata)

    dbYawServos = []
    for port,label,dir in [
            (ServoLHipAbd, 'LHipAbd', DirLHipAbd),
            (ServoLAnkleLat, 'LAnkleLat', DirLAnkleLat),
            (ServoRHipAbd, 'RHipAbd', DirRHipAbd),
            (ServoRAnkleLat, 'RAnkleLat', DirRAnkleLat)]:
      sdata = {
        'port': port,
        'dir': dir,
        'label': label,
        'var_enable': tk.IntVar(),
        'wlabel_min': None,
        'wentry_min': None,
        'var_min': tk.DoubleVar(),
        'wlabel_max': None,
        'wentry_max': None,
        'var_max': tk.DoubleVar(),
        'wentry_cur': None,
        'var_cur': tk.DoubleVar(),
      }
      sdata['var_enable'].set(1)
      sdata['var_min'].set(-15.0)
      sdata['var_max'].set(15.0)
      sdata['var_cur'].set(0.0)

      dbYawServos.append(sdata)
    self.mDB['Yaw'] = {'pid': dbYawPid, 'servos': dbYawServos}

    dbPitchPid = []
    for pidvar,pidval in [('Kp', 0.05), ('Ki', 0.0), ('Kd', 0.1),
                          ('CVmin', -20.0), ('CVmax', 20.0)]:
      pdata = {'label': pidvar, pidvar: tk.DoubleVar()}
      pdata[pidvar].set(pidval)
      dbPitchPid.append(pdata)

    dbPitchServos = []
    for port,label,dir in [
            (ServoLHip, 'LHip', DirLHip),
            (ServoLKnee, 'LKnee', DirLKnee),
            (ServoLAnkle, 'LAnkle', DirLAnkle),
            (ServoRHip, 'RHip', DirRHip),
            (ServoLKnee, 'RKnee', DirRKnee),
            (ServoLAnkle, 'RAnkle', DirRAnkle)]:
      sdata = {
        'port': port,
        'dir': dir,
        'label': label,
        'var_enable': tk.IntVar(),
        'wlabel_min': None,
        'wentry_min': None,
        'var_min': tk.DoubleVar(),
        'wlabel_max': None,
        'wentry_max': None,
        'var_max': tk.DoubleVar(),
        'wentry_cur': None,
        'var_cur': tk.DoubleVar(),
      }
      sdata['var_enable'].set(1)
      sdata['var_min'].set(-15.0)
      sdata['var_max'].set(15.0)
      sdata['var_cur'].set(0.0)

      dbPitchServos.append(sdata)
    self.mDB['Pitch'] = {'pid': dbPitchPid, 'servos': dbPitchServos}

    for whichfoot in BpFeet:
      pdata = {'labelx': 'CoMx', 'var_comx': tk.DoubleVar(), 
               'labely': 'CoMy', 'var_comy': tk.DoubleVar()} 
      pdata['var_comx'].set(0.0)
      pdata['var_comy'].set(0.0)
      self.mDB[whichfoot] = pdata
    self.mDB['pv_yaw']   = {'label': 'PVyaw',   'var_comx': tk.DoubleVar()}
    self.mDB['pv_yaw']['var_comx'].set(0.0)
    self.mDB['pv_pitch'] = {'label': 'PVpitch', 'var_comy': tk.DoubleVar()}
    self.mDB['pv_pitch']['var_comy'].set(0.0)

    self.PidInit()

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

  #--
  def destroy(self):
    """ Destroy window callback event. """
    GuiWin.GuiWin.destroy(self, auto=self.mIsRunning,
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
          'cfg'           - configure BrainPack StandTall window.
          'bpimu'         - StandTall sensor data
          'clear'         - clear current set displayed data
          'connect'       - open connection to bsproxy
          'disconnect'    - close connection to bsproxy
          'run'           - initialize and run PIDs
          'stop'          - stop PIDs

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'bpimu':
      self.WinUpdateStandTall(**kwargs)
    elif request == 'cfg':
      self.WinUpdateStatus(**kwargs)
    elif request == 'clear':
      self.WinUpdateClear()
    elif request == 'connect':
      self.WinUpdateConnect()
    elif request == 'disconnect':
      self.WinUpdateDisconnect()
    elif request == 'run':
      self.WinUpdateRun()
    elif request == 'stop':
      self.WinUpdateStop()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateStandTall(self, raw_data=None, cooked_data=None,
                         device_id=None, version=None):
    """ Update StandTall sensor data

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
              run_time=<str>            - run-time state
              sensors_proxy_addr=<str>  - sensors botsense proxy IP address
              sensors_proxy_port=<int>  - sensors botsense proxy TCP port
              rcb3_proxy_addr=<str>     - RCB3 botsense proxy IP address
              rcb3_proxy_port=<int>     - RCB3 botsense proxy TCP port
              i2c_dev_name=<str>        - proxied I2C device
              rcb3_dev_name=<str>       - proxied RCB3 device
              bpimu=<int>               - IMU I2C address
              bpfoot_left=<int>         - left foot I2C address
              bpfoot_right=<int>        - right foot I2C address
              data_class=<class>        - data class (raw/cooked)

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.iteritems():
      if k == 'run_time':
        if v == 'update':
          if not self.mBsClientSensors.mBsIsConn:
            items[k] = 'disconnected'
          elif self.mIsRunning:
            items[k] = 'running'
          else:
            items[k] = 'stopped'
        else:
          items[k] = v
        self.mWidgetActiveImage.SetActive(items[k])
      elif k == 'sensors_proxy_addr':
        items[k] = v
      elif k == 'sensors_proxy_port':
        items[k] = v
      elif k == 'rcb3_proxy_addr':
        items[k] = v
      elif k == 'rcb3_proxy_port':
        items[k] = v
      elif k == 'i2c_dev_name':
        items[k] = v
      elif k == 'rcb3_dev_name':
        items[k] = v
      elif k == 'bpimu':
        items[k] = v
      elif k == 'bpfoot_left':
        items[k] = v
      elif k == 'bpfoot_right':
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
    pass

  #--
  def WinUpdateRun(self):
    if not self.mIsConn:
      return
    self.PidStart()
    self.CtlPanelCfgRunStop()

  #--
  def WinUpdateStop(self):
    self.PidStop()
    self.CtlPanelCfgRunStop()

  #--
  def WinUpdateConnect(self):
    """ Open connection to the BotSense IP Proxy Server.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    if not self.mBsClientSensors.Connect() or not self.mBsClientRCB3.Connect():
      self.mBsClientSensors.Disconnect()
      self.mBsClientRCB3.Disconnect()
      self.PidStop()
      self.CtlPanelStateDisconnected()
      return

    # Sensors client
    for whichSensor in BpSensors:
      i2caddr = self.mBsClientSensors.mBsProxiedDev[whichSensor]['i2c']
      if self.mBpSensorState[whichSensor]:
        self.mBsClientSensors.CmdDevOpen(whichSensor, i2caddr, self.mDevNameI2C)

    # RCB3 Client
    self.mBsClientRCB3.CmdDevOpen('rcb3', 0, self.mDevNameRCB3)

    self.mIsConn = True
    self.CtlPanelStateConnected()

  #--
  def WinUpdateDisconnect(self):
    """ Close connection to the BotSense IP Proxy Server.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.PidStop()
    self.mBsClientSensors.Disconnect()
    self.mBsClientRCB3.Disconnect()
    self.mIsRunning == False
    self.mIsConn = False
    self.CtlPanelStateDisconnected()
    self.CtlPanelCfgRunStop()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # IMU Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # current window dimensions
    self.mWinGeo = gut.geometry(self)

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

    self.PidPanelInit(parent, 'Yaw', row, col)

    col += 1

    self.PidPanelInit(parent, 'Pitch', row, col)

    row += 1
    col  = 0

    self.PvPanelInit(parent, row, col, colspan=2)

    row += 1

    self.CtlPanelInit(parent, row, col, colspan=2)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui PID Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PidPanelInit(self, parent, db, row, column):
    """ Create the PID Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    #
    # pid frame
    #
    frame = tk.Frame(parent, relief=tk.RAISED, borderwidth=1, width=400)
    frame.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    frow = 0
    fcol = 0

    # pid panel title
    w = tk.Label(frame, text='vKHR2 %s Control' % db, fg=gt.ColorGreen1)
    w.grid(row=frow, column=fcol)

    frow += 1

    subframe = tk.Frame(frame, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=frow, column=fcol, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # pid parameters title
    w = tk.Label(subframe, text='%s PID Parameters' % db, fg=gt.ColorBlue1)
    w.grid(row=subrow, column=subcol, columnspan=10)

    subrow += 1

    for pdata in self.mDB[db]['pid']:

      w = tk.Label(subframe, text="%s:" % pdata['label'])
      w.grid(row=subrow, column=subcol)

      subcol += 1

      w = tk.Entry(subframe, width=5, textvariable=pdata[pdata['label']])
      w.grid(row=subrow, column=subcol)

      subcol += 1

    frow += 1

    subframe = tk.Frame(frame, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=frow, column=fcol, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # servos title
    w = tk.Label(subframe,
        text='%s PID Controlled Variables: Servo Angles' % db,
        fg=gt.ColorBlue1)
    w.grid(row=subrow, column=subcol, columnspan=10)

    subrow += 1

    for servo in self.mDB[db]['servos']:
      subcol = 0

       # label
      w = tk.Label(subframe, width=12, text=servo['label']+':', 
          anchor=tk.E, fg=gt.ColorBlack)
      w.grid(row=subrow, column=subcol, sticky=tk.N+tk.E)

      subcol += 1

      # enable button
      w = tk.Checkbutton(subframe, command=self._mkcallback(servo),
                       variable=servo['var_enable'])
      w.grid(row=subrow, column=subcol, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(w, 
          text="Enable %s servo." % servo['label'])

      subcol += 1

      # min label
      w = tk.Label(subframe, text='Min:', fg=gt.ColorBlack)
      w.grid(row=subrow, column=subcol, sticky=tk.E)
      servo['wlabel_min'] = w

      subcol += 1

      # servo min address entry
      w = tk.Entry(subframe, width=5, textvariable=servo['var_min'])
      w.grid(row=subrow, column=subcol, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(w,
          text="Minimum %s servo position (degrees)." % servo['label'])
      servo['wentry_min'] = w

      subcol += 1

      # max label
      w = tk.Label(subframe, text='Max:', fg=gt.ColorBlack)
      w.grid(row=subrow, column=subcol, sticky=tk.E)
      servo['wlabel_max'] = w

      subcol += 1

      # servo min address entry
      w = tk.Entry(subframe, width=5, textvariable=servo['var_max'])
      w.grid(row=subrow, column=subcol, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(w,
          text="Maximum %s servo position (degrees)." % servo['label'])
      servo['wentry_max'] = w

      subcol += 1

      # cur label
      w = tk.Label(subframe, text='Cur:', fg=gt.ColorBlack)
      w.grid(row=subrow, column=subcol, sticky=tk.E)

      subcol += 1

      # servo min address entry
      w = tk.Entry(subframe, width=5, textvariable=servo['var_cur'],
          relief=tk.FLAT)
      w.grid(row=subrow, column=subcol, sticky=tk.W, padx=3)
      servo['wentry_cur'] = w

      subrow += 1


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Process Variable Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PvPanelInit(self, parent, row, column, colspan):
    """ Create the Process Variable Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent
          colspan - grid column span

        Return Value:
          None
    """
    #
    # pid frame
    #
    frame = tk.Frame(parent, relief=tk.RAISED, borderwidth=1)
    frame.grid(row=row, column=column, columnspan=colspan,
        padx=3, ipadx=3, ipady=3, sticky=tk.N+tk.W+tk.E)

    frow = 0
    fcol = 0

    # pid panel title
    w = tk.Label(frame, text='vKHR2 PID Process Variables: Center of Masses',
        fg=gt.ColorGreen1)
    w.grid(row=frow, column=fcol, columnspan=6)

    # left foot
    frow     += 1
    whichfoot = 'bpfoot_left'

    w = tk.Label(frame, text='Left Foot CoM', fg=gt.ColorBlue1)
    w.grid(row=frow, column=fcol, columnspan=2)

    frow += 1

    w = tk.Label(frame, text=self.mDB[whichfoot]['labelx']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB[whichfoot]['var_comx'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)

    frow += 1
    fcol  = 0

    w = tk.Label(frame, text=self.mDB[whichfoot]['labely']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB[whichfoot]['var_comy'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)

    # right foot
    frow      = 1
    fcol      = 2
    whichfoot = 'bpfoot_right'

    w = tk.Label(frame, text='Right Foot CoM', fg=gt.ColorBlue1)
    w.grid(row=frow, column=fcol, columnspan=2)

    frow += 1

    w = tk.Label(frame, text=self.mDB[whichfoot]['labelx']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB[whichfoot]['var_comx'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)

    frow += 1
    fcol  = 2

    w = tk.Label(frame, text=self.mDB[whichfoot]['labely']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB[whichfoot]['var_comy'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)


    # yaw,pitch pv 
    frow      = 1
    fcol      = 4

    w = tk.Label(frame, text='Process Variable CoM', fg=gt.ColorBlue1)
    w.grid(row=frow, column=fcol, columnspan=2)

    frow += 1

    w = tk.Label(frame, text=self.mDB['pv_yaw']['label']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB['pv_yaw']['var_comx'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)

    frow += 1
    fcol  = 4

    w = tk.Label(frame, text=self.mDB['pv_pitch']['label']+":",
        fg=gt.ColorBlack)
    w.grid(row=frow, column=fcol, sticky=tk.E)

    fcol += 1

    w = tk.Entry(frame, width=5, textvariable=self.mDB['pv_pitch']['var_comy'],
        relief=tk.FLAT)
    w.grid(row=frow, column=fcol, sticky=tk.W, padx=3)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column, colspan):
    """ Create the Stand Tall Control Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    # the frame
    cpframe = tk.Frame(parent, relief=tk.RAISED, borderwidth=1)
    cpframe.grid(row=row, column=column, columnspan=colspan,
              padx=3, ipadx=3, ipady=3, sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0

    # control panel title
    w = tk.Label(cpframe, text='vKHR2 BrainPack Stand Tall Control Panel',
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

    # run/stop visual feedback
    runFiles = []
    for n in [0, 1, 2, 3]:
      filename = gut.GetFusionImageFileName('RalfWalking%d.gif' % n)
      if filename:
        runFiles += [filename]
    stopFile = gut.GetFusionImageFileName('RalfWalkingMan.gif')
    discFile = gut.GetFusionImageFileName('SerDisc.gif')
    w = gut.ActiveImageWidget(subframe, 
            activesets={'disconnected':[discFile],
                        'running':runFiles,
                        'stopped':[stopFile]},
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

    # run/stop button
    w = tk.Button(subframe, width=8, command=self.CbRunStop)
    self.mTtStopText  = "Stop PID control"
    self.mTtRunText   = "Run PID control"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtRunStop = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonRunStop = w
    self.CtlPanelCfgRunStop()

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
          {'tag': 'sensors_proxy_addr',
           'prefix': 'sensors bsproxy:',
           'max_width': 20,
           'val': '',
           'tooltip': "Sensors BotSense Proxy's IP address."
          },
          {'tag': 'sensors_proxy_port',
           'prefix': ':',
           'max_width': 5,
           'val': 0,
           'fmt': '%d',
           'tooltip': "Sensors BotSense Proxy's TCP port."
          },
          {'tag': 'rcb3_proxy_addr',
           'prefix': 'rcb3 bsproxy:',
           'max_width': 20,
           'val': '',
           'tooltip': "RCB3 BotSense Proxy's IP address."
          },
          {'tag': 'rcb3_proxy_port',
           'prefix': ':',
           'max_width': 5,
           'val': 0,
           'fmt': '%d',
           'tooltip': "RCB3 BotSense Proxy's TCP port."
          },
          {'tag': 'i2c_dev_name',
           'prefix': 'proxied I'+gt.UniSuperscript['2']+'C:',
           'max_width': 12,
           'val': '',
           'tooltip': "Proxied I2C device name."
          },
          {'tag': 'rcb3_dev_name',
            'prefix': 'proxied RCB3:',
           'max_width': 12,
           'val': '',
           'tooltip': "Proxied RCB3 device name."
          },
          {'tag': 'bpimu',
           'prefix': 'IMU I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "IMU proxied I2C address."
          },
          {'tag': 'bpfoot_left',
           'prefix': 'left foot I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "Left foot proxied I2C address."
          },
          {'tag': 'bpfoot_right',
           'prefix': 'right foot I'+gt.UniSuperscript['2']+'C:',
           'max_width': 4,
           'val': 0,
           'fmt': '0x%02x',
           'tooltip': "right foot proxied I2C address."
          },
          {'tag': 'data_class',
           'prefix': 'data-class:',
           'max_width': 6,
           'val': 'raw',
           'tooltip': "IMU data are raw values or calibrated (cooked)."
          },
        ],
        initWidth=800,
        maxRows=3)
    self.mStatusBar.grid(row=row, column=column, pady=3, sticky=tk.W)

    # set control panel initial state
    if self.mDataClass == BpDataClassRaw:
      self.CtlPanelStateRaw()
    else:
      self.CtlPanelStateCooked()

  #--
  def CtlPanelCfgRunStop(self):
    """ Place control panel into run/stop update configuration. """
    w = self.mButtonRunStop
    if self.mIsRunning == True:
      w['text']             = 'Stop'
      w['fg']               = gt.ColorBttnStop
      w['activeforeground'] = gt.ColorBttnStop
      self.mTtRunStop.newtip(self.mTtStopText)
      self.mButtonRead['state'] = tk.NORMAL
    else:
      w['text']             = 'Run'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtRunStop.newtip(self.mTtRunText)
      self.mButtonRead['state'] = tk.NORMAL

  #--
  def CtlPanelStateRaw(self):
    """ Set Control Panel widgets' states to raw data class. """
    self.mButtonDataClass['text'] = 'Cooked'
    self.mTtDataClass.newtip("Read calibrated sensor data")

  #--
  def CtlPanelStateCooked(self):
    """ Set Control Panel widgets' states to cooked data class. """
    self.mButtonDataClass['text'] = 'Raw'
    self.mTtDataClass.newtip("Read raw sensor data")

  #--
  def CtlPanelStateConnected(self):
    """ Set Control Panel widgets' states to connected. """
    self.mButtonConn['state']             = tk.NORMAL
    self.mButtonConn['text']              = 'Disconnect'
    self.mButtonConn['fg']                = gt.ColorBttnStop
    self.mButtonConn['activeforeground']  = gt.ColorBttnStop
    self.mTtConn.newtip("Disconnect to BotSense IP Proxy Servers.")

  #--
  def CtlPanelStateDisconnected(self):
    """ Set Control Panel widgets' states to disconnected. """
    self.mButtonConn['state']             = tk.NORMAL
    self.mButtonConn['text']              = 'Connect'
    self.mButtonConn['fg']                = gt.ColorBttnGo
    self.mButtonConn['activeforeground']  = gt.ColorBttnGo
    self.mTtConn.newtip("Connect to BotSense IP Proxy Servers.")


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CbServoEnable(self, servo=None):
    val = servo['var_enable'].get()
    if val:
      servo['wlabel_min']['state'] = tk.NORMAL
      servo['wentry_min']['state'] = tk.NORMAL
      servo['wlabel_max']['state'] = tk.NORMAL
      servo['wentry_max']['state'] = tk.NORMAL
    else:
      servo['wlabel_min']['state'] = tk.DISABLED
      servo['wentry_min']['state'] = tk.DISABLED
      servo['wlabel_max']['state'] = tk.DISABLED
      servo['wentry_max']['state'] = tk.DISABLED

  #--
  def CbRunStop(self):
    """ Automatic/Manual Viz canvas updates toggle callback. """
    if not self.mIsConn:
      return
    if self.mIsRunning == True:
      self.mIsRunning  = False
      self.WinQueueRequest('stop')
    else:
      self.mIsRunning  = True
      self.WinQueueRequest('run')
    self.WinQueueRequest('cfg', run_time="update")

  #--
  def CbRead(self):
    """ Read IMU callback. """
    if self.mTestGui:
      self.IMUTestGuiSensorDataManual()
    else:
      self.IMUBsSensorDataPull()

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
    if self.mBsClientSensors.mBsIsConn:
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
  def CbCfgBsProxyI2C(self):
    """ Configure I2C bsproxy callback. """
    keys = GuiDlgKHR2Proxy.GetSettingNames()

    last = {}
    for k in keys:
      if k == 'proxy_addr':
        last[k] = self.mBsClientSensors.mBsProxyAddr
      elif k == 'proxy_port':
        last[k] = self.mBsClientSensors.mBsProxyPort
      elif k == 'i2c_dev_name':
        last[k] = self.mDevNameI2C
      elif k in BpSensors:
        last[k] = {
            'enable':self.mBpSensorState[k], 
            'i2c_addr': self.mBsClientSensors.mBsProxiedDev[k]['i2c']}

    dlg = GuiDlgKHR2Proxy.GuiDlgKHR2Proxy(self, lastSettings=last)

    if dlg.result:
      for k,v in dlg.result.iteritems():
        if k == 'proxy_addr' and v:
          self.mBsClientSensors.SetProxyAddr(v)
        elif k == 'proxy_port' and v:
          self.mBsClientSensors.SetProxyPort(v)
        elif k == 'i2c_dev_name':
          self.mDevNameI2C = v
        elif k in BpSensors:
          self.mBpSensorState[k] = v['enable']
          self.mBsClientSensors.mBsProxiedDev[k]['i2c'] = v['i2c_addr']

      self.WinQueueRequest('cfg',
        sensors_proxy_addr=self.mBsClientSensors.mBsProxyAddr,
        sensors_proxy_port=self.mBsClientSensors.mBsProxyPort,
        bpimu=self.mBsClientSensors.mBsProxiedDev['bpimu']['i2c'],
        bpfoot_left=self.mBsClientSensors.mBsProxiedDev['bpfoot_left']['i2c'],
        bpfoot_right=self.mBsClientSensors.mBsProxiedDev['bpfoot_right']['i2c'],
        i2c_dev_name=self.mDevNameI2C)

      print("IMU enabled = ", self.mBpSensorState['bpimu'])
      print("Left foot enabled = ", self.mBpSensorState['bpfoot_left'])
      print("Right foot enabled = ", self.mBpSensorState['bpfoot_right'])

  #--
  def CbCfgBsProxyRCB3(self):
    """ Configure RCB3 bsproxy callback. """
    keys = GuiDlgKHR2ProxySimple.GetSettingNames()

    last = {}
    for k in keys:
      if k == 'proxy_addr':
        last[k] = self.mBsClientRCB3.mBsProxyAddr
      elif k == 'proxy_port':
        last[k] = self.mBsClientRCB3.mBsProxyPort
      elif k == 'dev_name':
        last[k] = self.mDevNameRCB3

    dlg = GuiDlgKHR2ProxySimple.GuiDlgKHR2ProxySimple(self, lastSettings=last)

    if dlg.result:
      for k,v in dlg.result.iteritems():
        if k == 'proxy_addr' and v:
          self.mBsClientRCB3.SetProxyAddr(v)
        elif k == 'proxy_port' and v:
          self.mBsClientRCB3.SetProxyPort(v)
        elif k == 'dev_name':
          self.mDevNameRCB3 = v
      self.WinQueueRequest('cfg',
        rcb3_proxy_addr=self.mBsClientSensors.mBsProxyAddr,
        rcb3_proxy_port=self.mBsClientSensors.mBsProxyPort,
        rcb3_dev_name=self.mDevNameRCB3)


  #--
  def CbI2CScan(self):
    if self.mBsClientSensors.mBsIsConn:
      for proxdev,proxdata in self.mBsClientSensors.mBsProxiedDev.iteritems():
        handle = proxdata['handle']
        if handle is not None:
          break;
      if handle is not None:
        scanned = self.mBsClientSensors.CmdI2CScan(proxdev)
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
  def CbStandTallIds(self):
    hdr = "Device ID   Firmware Version\n" \
          "---------   ----------------\n"
    txt = ''
    if self.mBsClientSensors.mBsIsConn:
      if self.mBpSensorState['bpimu']:
        ids = self.mBsClientSensors.CmdBpIMU('getids')
        if ids:
          txt += "0x%04x          0x%02x\n" % (ids['device_id'], ids['version'])
      for whichfoot in BpFeet:
        if self.mBpSensorState[whichfoot]:
          ids = self.mBsClientSensors.CmdBpFoot(whichfoot, 'getids')
          if ids:
            if whichfoot == 'bpfoot_left':
              txt += 'Left    '
            else:
              txt += 'Right   '
            txt += "0x%04x          0x%02x\n" % \
                (ids['device_id'], ids['version'])
    GuiDlgAbout.GuiDlgAbout(self, 
                name="BrainPack IDs",
                desc=hdr+txt)

  #--
  def CbStandTallCal(self, orientation=0):
    if self.mBsClientSensors.mBsIsConn:
      if self.mBpSensorState['bpimu']:
        self.mBsClientSensors.CmdBpIMU('setcal', orientation)

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
  # PID Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PidInit(self):
    self.mPidYaw    = None
    self.mPidPitch  = None
    self.mCoMLeftX  = 0.0
    self.mCoMLeftY  = 0.0
    self.mCoMRightX = 0.0
    self.mCoMRightY = 0.0

  #--
  def PidStart(self):
    # Yaw (side to side) PID
    pidParam = {}
    for pdata in self.mDB['Yaw']['pid']:
      pidParam[pdata['label']] = pdata[pdata['label']].get()
    self.mPidYaw = Pid.Pid(self.CbCoMInputYaw, self.CbCoMOutputYaw, 
            0.0,                        # cv 
            0.25,                       # dt
            pidParam['Kp'],
            pidParam['Ki'],
            pidParam['Kd'],
            Wp=1.0, Wd=1.0,             # error weights
            cv_min=pidParam['CVmin'],
            cv_max=pidParam['CVmax'],
            trace=True)

    # Pitch (front to back) PID
    pidParam = {}
    for pdata in self.mDB['Pitch']['pid']:
      pidParam[pdata['label']] = pdata[pdata['label']].get()
    self.mPidPitch = Pid.Pid(self.CbCoMInputPitch, self.CbCoMOutputPitch,
            0.0,                        # cv
            0.25,                       # dt
            pidParam['Kp'],
            pidParam['Ki'],
            pidParam['Kd'],
            Wp=1.0, Wd=1.0,             # error weights
            cv_min=pidParam['CVmin'],
            cv_max=pidParam['CVmax'],
            trace=True)
    self.mPidYaw.start()
    self.mPidPitch.start()

  #--
  def PidStop(self):
    if self.mPidYaw:
      self.mPidYaw.cancel()
    if self.mPidPitch:
      self.mPidPitch.cancel()

  #--
  def CoMFoot(self, sensors):
    """ Calculate a foot's center of masses of the sole.

        Parameters:
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

    if M > 0.0:
      return (x_com/M, y_com/M)
    else:
      return (0.0, 0.0)

  #--
  def CbCoMInputYaw(self):
    """ Yaw PID input (setpoint, pv) callback """
    #semaFoot.acquire()
    whichfoot = 'bpfoot_left'
    if self.mBpSensorState[whichfoot]:
      rsp = self.mBsClientSensors.CmdBpFoot(whichfoot, 'getraw')
      if rsp:
        self.mCoMLeftX, self.mCoMLeftY = self.CoMFoot(rsp['raw_data'])
    else:
        self.mCoMLeftX, self.mCoMLeftY = 0.0, 0.0
    self.mDB[whichfoot]['var_comx'].set(self.mCoMLeftX)
    self.mDB[whichfoot]['var_comy'].set(self.mCoMLeftY)
    whichfoot = 'bpfoot_right'
    if self.mBpSensorState[whichfoot]:
      rsp = self.mBsClientSensors.CmdBpFoot('bpfoot_right', 'getraw')
      if rsp:
        self.mCoMRightX, self.mCoMRightY = self.CoMFoot(rsp['raw_data'])
    else:
        self.mCoMRightX, self.mCoMRightY = 0.0, 0.0
    self.mDB[whichfoot]['var_comx'].set(self.mCoMRightX)
    self.mDB[whichfoot]['var_comy'].set(self.mCoMRightY)
    self.mDB['pv_yaw']['var_comx'].set(self.mCoMLeftX - self.mCoMRightX)
    #semaFoot.release()
    return 0.0, self.mCoMLeftX - self.mCoMRightX

  #--
  def CbCoMOutputYaw(self, cv):
    """ Yaw PID output (servo degrees) callback """
    cv = cv / 20.0
    for servo in self.mDB['Yaw']['servos']:
      if servo['var_enable'].get():
        if cv >= 0.0:
          pos = servo['var_max'].get() * cv * servo['dir']
        else:
          pos = servo['var_min'].get() * abs(cv) * servo['dir']
        self.mBsClientRCB3.CmdRCB3Move(servo['port'], 50, pos)
        servo['var_cur'].set(pos)

  #--
  def CbCoMInputPitch(self):
    """ Pitch PID input (setpoint, pv) callback """
    self.mDB['pv_pitch']['var_comy'].set(self.mCoMLeftY + self.mCoMRightY)
    return 0.0, self.mCoMLeftY + self.mCoMRightY

  #--
  def CbCoMOutputPitch(self, cv):
    """ Pitch PID output (servo degrees) callback """
    cv = cv / 20.0
    for servo in self.mDB['Pitch']['servos']:
      if servo['var_enable'].get():
        if cv >= 0.0:
          pos = servo['var_max'].get() * cv * servo['dir']
        else:
          pos = servo['var_min'].get() * abs(cv) * servo['dir']
        self.mBsClientRCB3.CmdRCB3Move(servo['port'], 50, pos)
        servo['var_cur'].set(pos)



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
    elif not self.mIsRunning:
        return
    elif self.mTestGui:
      self.IMUTestGuiSensorDataAuto()
    elif self.mBsClientSensors.mBsIsConn:
      self.IMUBsSensorDataPull()

  #--
  def IVCancel(self):
    #if self.mIvtPull:
    #  self.mIvtPull.cancel()
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Feet Sensor Data Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def FeetBsSensorDataPull(self):
    for whichfoot in BpFeet:
      if self.mBpSensorState[whichfoot]:
        if self.mDataClass == BpDataClassRaw:
          data = self.mBsClientSensors.CmdBpFoot(whichfoot, 'getraw')
        else:
          data = self.mBsClientSensors.CmdBpFoot(whichfoot, 'getcooked')
        if data:
          self.WinQueueRequest(whichfoot, **data)

  #--
  def FeetTestGuiSensorDataAuto(self):
    data = {}
    if self.mDataClass == BpDataClassRaw:
      for whichfoot in BpFeet:
        if not self.mBpSensorState[whichfoot]:
          continue
        data[whichfoot] = [0] * BpFootNumOfFootSensors
        for i in range(BpFootNumOfFootSensors):
          v = random.randint(0,BpFootSensorMax)
          data[whichfoot][i] = v
        self.WinQueueRequest(whichfoot, raw_data=data[whichfoot])
    elif self.mDataClass == BpDataClassCooked:
      for whichfoot in BpFeet:
        if not self.mBpSensorState[whichfoot]:
          continue
        data[whichfoot] = [0] * BpFootNumOfFootSensors
        for i in range(BpFootNumOfFootSensors):
          v = random.randint(BpFootSensorMax/2,BpFootSensorMax)
          data[whichfoot][i] = v
        vec_dir, vec_mag = self.FeetTestGuiCalcVector(data[whichfoot])
        self.WinQueueRequest(whichfoot, cooked_data=data[whichfoot],
                                        vec_mag=vec_mag, vec_dir=vec_dir)

  #--
  def FeetTestGuiSensorDataManual(self):
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
  def FeetTestGuiCalcVector(self, sensor):
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
  # IMU Sensor Data Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IMUBsSensorDataPull(self):
    if not self.mBpSensorState['bpimu']:
      return
    elif self.mDataClass == BpDataClassRaw:
      rsp = self.mBsClientSensors.CmdBpIMU('getraw')
      k = 'raw_data'
    else:
      rsp = self.mBsClientSensors.CmdBpIMU('getcooked')
      k = 'cooked_data'
    if rsp:
      rsp[k] = self.IMUADCtoG(rsp[k])
      self.WinQueueRequest('bpimu', **rsp)
      #print(rsp)

  #--
  def IMUADCtoG(self, adclist):
    """ Convert IMU ADC values to acceleration in g's. """
    g = []
    for adc in adclist:
      adc <<= 2 # only the upper 8-bits of 10-bits are read (low pass filter)
      v_in = float(adc) * BpADCMult   # convert AD to volts (Memsic Vout)
      g += [(v_in - BpIMUVzero) * BpIMUSensitivity] # convert volts to g's
    return g

  #--
  def IMUTestGuiSensorDataAuto(self):
    if not self.mBpSensorState['bpimu']:
      return
    elif self.mDataClass == BpDataClassRaw:
      data = [0] * BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = random.randint(0,BpIMUSensorMax)
        data[i] = v
      self.WinQueueRequest('bpimu', raw_data=self.IMUADCtoG(data))
    elif self.mDataClass == BpDataClassCooked:
      data = [0] * BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = random.randint(0,BpIMUSensorMax)
        data[i] = v
      self.WinQueueRequest('bpimu', cooked_data=self.IMUADCtoG(data))

  #--
  def IMUTestGuiSensorDataManual(self):
    data = [0] * BpIMUNumOfSensors
    if self.mDataClass == BpDataClassRaw:
      stepsize = (BpIMUSensorMax + 1) / BpIMUNumOfSensors
      for i in range(BpIMUNumOfSensors):
        v = i * stepsize
        data[i] = v
      self.WinQueueRequest('bpimu', raw_data=self.IMUADCtoG(data))
    else:
      stepsize = (BpIMUSensorMax + 1) / (BpIMUNumOfSensors * 2)
      for i in range(BpIMUNumOfSensors):
        v = i * stepsize
        data[i] = v
      self.WinQueueRequest('bpimu', cooked_data=self.IMUADCtoG(data))


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  
  import Fusion.Core.Gluon as Gluon
  import Fusion.Utils.WinUT as WinUT
  #import Fusion.KHR2.Robots.vKHR2 as vKHR2

  #--
  class KHR2StandTallUT:
    """ KHR-2 StandTall Unit Tester. """

    #--
    def __init__(self, robot, sut):
      self.mRobot   = None
      self.mSut     = sut


  #--
  def main():
    """ GuiWinKHR2StandTall Unit Test Main """
    root = tk.Tk()
    sut = GuiWinKHR2StandTall(root)    # system under test
    ut = KHR2StandTallUT(None, sut)    # unit tester
    root.mainloop()
    sut.IVCancel()

  # run unit test
  main()
