################################################################################
#
# GuiWinDynaViz.py
#

""" Graphical User Interface Attractor Dynamics Visualizer Window Module

Graphical User Interface (GUI) Tkinter window to visualize attractor
dynamics in action.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.02.17

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

import threading as thread
import math
import Tkinter as tk
import tkFont

import Fusion.Utils.IVTimer as IVTimer

import Fusion.Core.Values as Values

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiWin as GuiWin
import Fusion.Gui.GuiXYGraph as GuiXYGraph

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

_GraphRWidth    = 300 # default width of R graph
_GraphAWidth    = 300 # default width of A graph
_GraphSWidth    = 300 # default width of S graph
_GraphUWidth    = 300 # default width of U graph
_GraphHeight    = 200 # default height of each graph
_GraphTotWidth  = _GraphRWidth + _GraphAWidth # total width
_GraphTotHeight = _GraphHeight * 2  # total height


#-------------------------------------------------------------------------------
# CLASS: GuiWinDynaViz
#-------------------------------------------------------------------------------
class GuiWinDynaViz(GuiWin.GuiWin):
  """ GUI Attractor Dynamics Visualizer Window Class. """

  #--
  def __init__(self, parent, **options):
    """ Initialize the Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Dynamics options. Options are:
            auto=<bool>         - do [not] automatically update
            **winoptions        - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = 'Attractor Dynamics Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          options   - navigator input options.

        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode      = True

    # set options from input parameters
    for k,v in options.iteritems():
      if k == 'auto':
        if v:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False

    # override bad options with defaults

    # locals

    # repulsive force-let spike graph data
    self.mRXData = []   # x: zeta
    self.mRYData = []   # y: repulsive force-let
    self.mRIndex = {}   # order of data

    # attractor force-let spike graph data
    self.mAXData = []   # x: zeta
    self.mAYData = []   # y: attactor force-let
    self.mAIndex = {}   # order of data

    self.mAngDeg  = [0, 180, 360] # x: dynamics angle sampling points list
    self.mS_field = [0, 0, 0]     # y: target source potential field
    self.mu_field = [0, 0, 0]     # y: target activation potential field


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create the widgets
    self.GuiBody(self)

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
      height = geo[gut.H] - self.mWinBorder - self.mWinFixH
      self.mRGrapher.configure(width*_GraphRWidth/_GraphTotWidth, height/2)
      self.mAGrapher.configure(width*_GraphAWidth/_GraphTotWidth, height/2)
      self.mSGrapher.configure(width*_GraphSWidth/_GraphTotWidth, height/2)
      self.mUGrapher.configure(width*_GraphUWidth/_GraphTotWidth, height/2)
      self.mStatusBar.configure(width=width)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
    self.mDoCancel = True
    GuiWin.GuiWin.destroy(self, auto=self.mIsAutoMode)
   

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
          'cfg'     - configure USS window.
                        **kwargs (see WinUpdateCfg())
          'f_obs_i' - update repulsive force-lets
                        **kwargs (see WinUpdateRepellorSpikeGraph())
                        force=<bool>      - force update
          'Mi'      - update attractive force-lets
                        **kwargs (see WinUpdateAttractorSpikeGraph())
                        force=<bool>      - force update
          'S_field' - update source force field
                        field=<list>  - list of field values
                        force=<bool>      - force update
          'u_field' - update attractive force field
                        field=<list>  - list of field values
                        force=<bool>      - force update
          'status' - status of dynamics on robot speed and direction
                        **kwargs (see WinUpdateStatus())
                        force=<bool>      - force update
          'clear'   - clear current data and redraw screen

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    #print("Dbg: %s: WinUpdate: request: %s, %s" % \
    #    (self.mContextName, request, repr(kwargs)))
    if request == 'f_obs_i':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.WinUpdateRepellorSpikeGraph(obs_sensors=kwargs['obs_sensors'])
    elif request == 'Mi':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.WinUpdateAttractorSpikeGraph(tgt_sensors=kwargs['tgt_sensors'])
    elif request == 'S_field':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.mS_field = kwargs['field']
        self.mSGrapher.newdata(xdata=self.mAngDeg, ydata=self.mS_field)
    elif request == 'u_field':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.mu_field = kwargs['field']
        self.mUGrapher.newdata(xdata=self.mAngDeg, ydata=self.mu_field)
    elif request == 'status':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.WinUpdateStatus(**kwargs)
    elif request == 'cfg':
      self.WinUpdateCfg(**kwargs)
    elif request == 'clear':
      self.WinUpdateClear()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateCfg(self, **kwargs):
    """ Update dynamics configuration.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              ang_deg=<list>      - dynamics angle sampling points list
              obs_sensors=<dict>  - dictionary of obstacle sensors
              tgt_sensors=<dict>  - dictionary of target sensors

        Return Value:
          None.
    """
    for k,v in kwargs.iteritems():
      if k == 'ang_deg':
        self.mAngDeg = v
        self.mS_field = [0] * len(v)
        self.mu_field = [0] * len(v)
        self.mSGrapher.newdata(xdata=self.mAngDeg, ydata=self.mS_field)
        self.mUGrapher.newdata(xdata=self.mAngDeg, ydata=self.mu_field)
      elif k == 'obs_sensors':
        self.mRXData = []
        self.mRYData = []
        self.mRIndex = {}
        zetas = []
        for sensor in v.itervalues():
          zetas.append(sensor['zeta'])
        zetas.sort()
        # build spike data
        i = 1
        for zeta in zetas:
          deg = math.degrees(zeta)
          self.mRXData += [deg, deg, deg]
          self.mRYData += [0.0, 0.0, 0.0]
          self.mRIndex[zeta] = i
          i += 3
        self.mRGrapher.newdata(xdata=self.mRXData, ydata=self.mRYData)
      elif k == 'tgt_sensors':
        self.mAXData = []
        self.mAYData = []
        self.mAIndex = {}
        zetas = []
        for sensor in v.itervalues():
          zetas.append(sensor['zeta'])
        zetas.sort()
        # build spike data
        i = 1
        for zeta in zetas:
          deg = math.degrees(zeta)
          self.mAXData += [deg, deg, deg]
          self.mAYData += [0.0, 0.0, 0.0]
          self.mAIndex[zeta] = i
          i += 3
        self.mAGrapher.newdata(xdata=self.mAXData, ydata=self.mAYData)

  #--
  def WinUpdateRepellorSpikeGraph(self, obs_sensors={}):
    """ Update obstacle repellor force-lets spike graph.

        Execution Context: GuiWin server thread

        Parameters:
          obs_sensors=<dict> - dictionary of obstacle sensors read data

        Return Values:
          None
    """
    for sensor in obs_sensors.itervalues():
      if sensor.has_key('f_obs_i'):
        self.mRYData[self.mRIndex[sensor['zeta']]] = sensor['f_obs_i']
      else:
        self.mRYData[self.mRIndex[sensor['zeta']]] = 0.0
    self.mRGrapher.newdata(xdata=self.mRXData, ydata=self.mRYData)

  #--
  def WinUpdateAttractorSpikeGraph(self, tgt_sensors={}):
    """ Update target attractor force-lets spike graph.

        Execution Context: GuiWin server thread

        Parameters:
          tgt_sensors=<dict> - dictionary of target sensors read data

        Return Values:
          None
    """
    for sensor in tgt_sensors.itervalues():
      if sensor.has_key('Mi'):
        self.mAYData[self.mAIndex[sensor['zeta']]] = sensor['Mi']
      else:
        self.mAYData[self.mAIndex[sensor['zeta']]] = 0.0
    self.mAGrapher.newdata(xdata=self.mAXData, ydata=self.mAYData)

  #--
  def WinUpdateStatus(self, **kwargs):
    """ Update status bar with effects of dynamics on robot speed
        and direction.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              speed=<mm/s>          - robot's current speed
              theta=<radians>       - robot's current direction
              dtheta_obs=<radians>  - change in direction due to obstacls
              dtheta_tar=<radians>  - change in direction due to targets
              dtheta=<radians>      - dtheta_obs + dtheta_tar
              dspeed=<mm/s>         - change in speed

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.iteritems():
      if k in ['speed', 'dspeed']:
        items[k] = v / 1000.0       # convert to meters
      elif k in ['theta', 'dtheta_obs', 'dtheta_tar', 'dtheta']:
        items[k] = math.degrees(v)  # convert to degrees [0, 360)
        if items[k] < 0.0:
          items[k] += 360.0
    self.mStatusBar.Update(**items)

  #--
  def WinUpdateClear(self):
    """ Clear dynamics data and graphics.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.mRGrapher.redraw()
    self.mRYData = [0.0] * len(self.mRXData)
    self.mRGrapher.newdata(xdata=self.mRXData, ydata=self.mRYData)

    self.mAGrapher.redraw()
    self.mAYData = [0.0] * len(self.mAXData)
    self.mAGrapher.newdata(xdata=self.mAXData, ydata=self.mAYData)

    self.mSGrapher.redraw()
    self.mS_field = [0.0] * len(self.mAngDeg)
    self.mSGrapher.newdata(xdata=self.mAngDeg, ydata=self.mS_field)

    self.mUGrapher.redraw()
    self.mu_field = [0.0] * len(self.mAngDeg)
    self.mUGrapher.newdata(xdata=self.mAngDeg, ydata=self.mu_field)
    self.WinUpdateStatus(speed=0.0, theta=0.0, dtheta_obs=0.0, dtheta_tar=0.0,
        dtheta=0.0, dspeed=0.0)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Dynamics Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # current window dimensions
    self.mWinGeo = gut.geometry(self)

    # current control panel frame dimension
    cpgeo = gut.geometry(self.mCtlPanelFrame)

    # control panel frame height is fixed
    self.mWinFixH = cpgeo[gut.H]

    self.mWinBorder = self.mWinGeo[gut.W] - _GraphTotWidth
                          #- int(self.mCanvasS['width']) \
                          #- int(self.mCanvasU['width'])

    # set window's minimum size
    self.wm_minsize(width=_GraphTotWidth+self.mWinBorder,
                    height=_GraphTotHeight+self.mWinBorder+self.mWinFixH)

  #--
  def GuiBody(self, parent):
    """ Create the window body. 

        Paremeters:
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

    # obstacle repellor forse-lets graph
    self.mCanvasR = tk.Canvas(subframe, width=_GraphRWidth, height=_GraphHeight)
    self.mCanvasR.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mRGrapher = GuiXYGraph.GuiXYGraph(self.mCanvasR,
        title='Obstacle Repellor Force-lets ~ f_obs,i(' + \
            gt.UniGreek['zeta'] + ')',
        xlabel=gt.UniGreek['zeta'] + ' ~ robot coordinate degrees',
        ylabel='f obs i',
        xstep=15, ystep=1,
        linewidth=3, linecolor=gt.ColorRed1, showpoints=False,
        xdata=self.mRXData, ydata=self.mRYData)

    subcol += 1

    # target atractor forse-lets graph
    self.mCanvasA = tk.Canvas(subframe, width=_GraphAWidth, height=_GraphHeight)
    self.mCanvasA.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mAGrapher = GuiXYGraph.GuiXYGraph(self.mCanvasA,
        title='Target Attractor Force-lets ~ Mi(' + \
            gt.UniGreek['zeta'] + ')',
        xlabel=gt.UniGreek['zeta'] + ' ~ robot coordinate degrees',
        ylabel='Mi',
        xstep=45, ystep=1,
        linewidth=3, linecolor=gt.ColorGreen1, showpoints=False,
        xdata=self.mAXData, ydata=self.mAYData)

    subrow += 1
    subcol = 0

    # target source behavior graph
    self.mCanvasS = tk.Canvas(subframe, width=_GraphSWidth, height=_GraphHeight)
    self.mCanvasS.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mSGrapher = GuiXYGraph.GuiXYGraph(self.mCanvasS,
        title='Target Source Behavior ~ S(' + gt.UniGreek['psi'] + ')',
        xlabel=gt.UniGreek['psi'] + ' ~ world coordinate degrees', ylabel='S',
        xstep=60, ystep=1,
        domain=(0.0, 360.0),
        xdata=self.mAngDeg, ydata=self.mS_field)

    subcol += 1

    # target activation behavior graph
    self.mCanvasU = tk.Canvas(subframe, width=_GraphUWidth, height=_GraphHeight)
    self.mCanvasU.grid(row=subrow, column=subcol, padx=1, pady=1,
               sticky=tk.N+tk.W+tk.E)

    self.mUGrapher = GuiXYGraph.GuiXYGraph(self.mCanvasU,
        title='Target Activation Behavior ~ u(' + gt.UniGreek['psi'] + ')',
        xlabel=gt.UniGreek['psi'] + ' ~ world coordinate degrees',
        ylabel='u', xstep=60, ystep=1,
        domain=(0.0, 360.0),
        xdata=self.mAngDeg, ydata=self.mu_field)

    row += 1

    # control panel
    self.CtlPanelInit(self, 2, 0)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the Viz Control Panel.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    # the frame
    cpframe = tk.Frame(parent, relief=tk.FLAT, borderwidth=0)
    cpframe.grid(row=row, column=column, padx=3, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0

    # control panel title
    w = tk.Label(cpframe, text='Dynamics Control Panel', fg=gt.ColorGreen1)
    w.grid(row=row, column=column)

    row += 1

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
    w = gut.ActiveImageWidget(subframe, 
        activesets={'auto':autoFiles, 'freeze':[manFile]}, period=0.25)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mWidgetActiveImage = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=1, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Freeze Visualization Updates"
    self.mTtAutoManAutoText = "Go to Automatic Visualization Updates"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtAutoMan = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonAutoMan = w
    self.CtlPanelCfgAutoMan()

    subcol += 1

    # column spacer
    tk.Label(subframe, width=5, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # clear button
    w = tk.Button(subframe, text='Clear', fg=gt.ColorBlack, width=8,
        command=self.CbVizClear)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Clear the visualization")

    subcol += 1

    # snapshot button
    w = tk.Button(subframe, text='SnapShot', fg=gt.ColorBlack, width=8,
        command=self.CbSnapShot)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w,
        text="Take a snap shot image (future).")

    subcol += 1

    # column spacer
    tk.Label(subframe, width=5, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # close button
    w = tk.Button(subframe, text='Close', fg=gt.ColorBlack, width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0

    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 4

    # status bar
    self.mStatusBar = GuiStatusBar.GuiStatusBar(cpframe,
        [ 
          {'tag': 'speed',
           'prefix': 'speed:',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.3f',
           'tooltip': "Robot's current speed (m/s)."
          },
          {'tag': 'theta',
           'prefix': gt.UniGreek['theta']+':',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.2f',
           'tooltip': "Robot's current direction (degrees)."
          },
          {'tag': 'dtheta_obs',
           'prefix': 'd'+gt.UniGreek['theta']+'_obs:',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.2f',
           'tooltip': 'Repulsive effects of obstacles on robot direction.\n'
                      'Obstacle d' + gt.UniGreek['theta'] + ' (degrees).'
          },
          {'tag': 'dtheta_tar',
           'prefix': 'd'+gt.UniGreek['theta']+'_tar:',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.2f',
           'tooltip': 'Attractive effects of targets on robot direction.\n'
                      'Target d' + gt.UniGreek['theta'] + ' (degrees).'
          },
          {'tag': 'dtheta',
           'prefix': 'd'+gt.UniGreek['theta']+':',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.2f',
           'tooltip': 'Total d' + gt.UniGreek['theta'] + ' = '
                      'd'+gt.UniGreek['theta']+'_obs + '
                      'd'+gt.UniGreek['theta']+'_tar (degrees).'

          },
          {'tag': 'dspeed',
           'prefix': 'dspeed:',
           'max_width': 7,
           'val': 0.0,
           'fmt': '%7.3f',
           'tooltip': 'Total dspeed from dynamical effects (m/s).\n'
                      '(currently not calculated).'
          },
        ],
        initWidth=sbwidth,
        maxRows=2)
    self.mStatusBar.grid(row=row, column=column, pady=3)

  #--
  def CtlPanelCfgAutoMan(self):
    """ Place control panel into automatic/manual update configuration. """
    w = self.mButtonAutoMan
    if self.mIsAutoMode == True:
      w['text']             = 'Freeze'
      w['fg']               = gt.ColorBttnStop
      w['activeforeground'] = gt.ColorBttnStop
      self.mTtAutoMan.newtip(self.mTtAutoManManText)
      self.mWidgetActiveImage.SetActive('auto')
    else:
      w['text']             = 'Auto'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtAutoMan.newtip(self.mTtAutoManAutoText)
      self.mWidgetActiveImage.SetActive('freeze')


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

  #--
  def CbVizClear(self):
    """ Clear the viz's current set of data callback. """
    self.WinQueueRequest('clear')

  #--
  def CbSnapShot(self):
    """ Take a snap shot image callback. """
    pass

  #--
  def CbClose(self):
    """ Close window callback. """
    self.destroy()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import time
  import random
  import Fusion.Utils.WinUT as WinUT
  import Fusion.Utils.IVTimer as IVTimer

  #--
  class WinUTDynaViz(WinUT.WinUT):
    """ GuiWinDynaViz Unit Test Window """
    #--
    def __init__(self):
      # the unit test
      ut = {
        'Null Test': self.utNullStart,
        'Config Test': self.utCfg,
        'Random Dynamics Test': self.utRandStart
      }

      WinUT.WinUT.__init__(self, title="GuiWinDynaViz Unit Test", ut=ut)

      self.sut_angdeg = range(0, 368, 8)

      self.sut_obs_sensors = {
        'prox_front':
        { 'zeta':0.0, 'angrange':math.radians(60),
            'f_obs_i':0
        },
        'prox_left':
        { 'zeta':math.radians(45), 'angrange':math.radians(60),
            'f_obs_i':0
        },
        'prox_right':
        { 'zeta':math.radians(-45), 'angrange':math.radians(60),
          'f_obs_i':0
        }
      }

      self.sut_tgt_sensors = {}
      for ang in [-135.0, 90.0, 45.0, 0.0, -45.0, -90.0, 135.0]:
        id = 'tgtsensor(%d)' % int(ang)
        self.sut_tgt_sensors[id] = {}
        self.sut_tgt_sensors[id]['zeta'] = math.radians(ang)
        self.sut_tgt_sensors[id]['angrange'] = math.radians(30)
        self.sut_tgt_sensors[id]['Mi'] = 0.0

      self.sut_S_field = [0] * len(self.sut_angdeg)
      self.sut_u_field = [0] * len(self.sut_angdeg)

    #--
    # THE UNIT TEST
    #--

    #--
    def utNullStart(self):
      """ Null UT Start """
      self.wut_showstatus("No nothing.")

    #--
    def utCfg(self):
      """ Null UT Start """
      self.wut_showstatus("Configure sampling points = %s" % \
          repr(self.sut_angdeg))
      self.mSut.WinQueueRequest('cfg', ang_deg=self.sut_angdeg)
      self.wut_showstatus("Configure obstacle sensors = %s" % \
          repr(self.sut_obs_sensors))
      self.mSut.WinQueueRequest('cfg', obs_sensors=self.sut_obs_sensors)
      self.wut_showstatus("Configure target sensors = %s" % \
          repr(self.sut_obs_sensors))
      self.mSut.WinQueueRequest('cfg', tgt_sensors=self.sut_tgt_sensors)
      self.wut_showstatus("Data configured.")

    #--
    def utRandStart(self):
      """ Random Dynamics UT Start """
      self.utCfg()
      self.wut_showstatus("Push random dynamics data.")
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utRandIter, cnt=0) 
      self.mIvt.start()

    #--
    def utRandIter(self, ivt):
      """ Random Dynamics Iterator """
      for data in self.sut_obs_sensors.itervalues():
        data['f_obs_i'] = random.random() * 2.0
      self.mSut.WinQueueRequest('f_obs_i', obs_sensors=self.sut_obs_sensors)
      for data in self.sut_tgt_sensors.itervalues():
        data['Mi'] = random.random() * 3.0
      self.mSut.WinQueueRequest('Mi', tgt_sensors=self.sut_tgt_sensors)
      i = 0
      n = len(self.sut_angdeg)
      while i < n:
        self.sut_S_field[i] = random.random() * 4.0
        self.sut_u_field[i] = random.random() * 4.0
        i += 1
      self.mSut.WinQueueRequest('S_field', field=self.sut_S_field)
      self.mSut.WinQueueRequest('u_field', field=self.sut_u_field)
      dtheta_obs = -1.0 + random.random() * 2.0
      dtheta_tar = -1.0 + random.random() * 2.0
      self.mSut.WinQueueRequest('status',
          speed=42,
          theta=math.radians(45.0)+dtheta_obs+dtheta_tar,
          dtheta_obs=dtheta_obs,
          dtheta_tar=dtheta_tar,
          dtheta=dtheta_obs + dtheta_tar,
          dspeed=random.random() * 300.0)

  #--
  def main():
    """ GuiWinDynaViz Unit Test Main """
    winUT = WinUTDynaViz()
    winSut = GuiWinDynaViz(winUT.wut_this())
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
