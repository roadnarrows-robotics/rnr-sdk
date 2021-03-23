################################################################################
#
# GuiWinKheNav
#

""" Graphical User Interface Khepera Navigator Window

Graphical User Interface (GUI) Tkinter navigator window to allow simple
maneuvering of the vKhepera robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.17

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

import math
import threading as thread
import Tkinter as tk
import tkFont

import Fusion.Core.Values as Values

import Fusion.Utils.IVTimer as IVTimer

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiWin as GuiWin

import Fusion.Khepera.Cmd.KheCmdBase as KheBase
import Fusion.Khepera.Robots.KheValues as KheValues

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

_KheNavGoalsFwdColor  = '#9999ff'   # color of goals forward points vector
_KheNavGoalsBwdColor  = '#ff9999'   # color of goals backward pointing vector
_KheNavGoalsNullColor = gt.ColorRed # color of null goals
_KheNavBotFwdColor    = '#66ff66'   # color of vKhepera forward points vector
_KheNavBotBwdColor    = '#66ff66'   # color of vKhepera backward points vector

# trip canvas dimensions
_EdgeLeft   = 5    # left edge margin
_EdgeTop    = 5    # top edge margin
_EdgeBottom = 5    # bottom edge margin
_EdgeRight  = 5    # right edge margin

# minimum size
_CanvasMinWidth   = 450 + _EdgeLeft + _EdgeRight
_CanvasMinHeight  = 450 + _EdgeTop + _EdgeBottom


#-------------------------------------------------------------------------------
# CLASS: GuiWinKheNav
#-------------------------------------------------------------------------------
class GuiWinKheNav(GuiWin.GuiWin):
  """ GUI Window vKhepera Navigator Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the vKhepera Navigator Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Navigator options. Options are:
            auto=<bool>       - do [not] automatically update
            k_curvature=<val> - constant of curvature
            sense_loc=<cb>    - sense location callback to vRobot
            sense_speed=<cb>  - sense speed callback to vRobot
            bellum_set=<cb>   - set bellum's new goals callback to vRobot
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = 'vKhepera Navigator'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          options   - navigator input options.
    """
    # defaults
    self.mIsAutoMode        = True
    self.mK                 = 1.0
    self.mCbRobotSenseLoc   = None
    self.mCbRobotSenseSpeed = None
    self.mCbRobotBellumGet  = None
    self.mCbRobotBellumSet  = None

    # set options from input parameters
    k = 1.0
    for key,val in options.iteritems():
      if key == 'auto':
        if val:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False
      elif key == 'k_curvature':
        try:
          k = float(val)
        except (NameError, TypeError, ValueError):
          k = 1.0
      elif key == 'sense_loc':
        self.mCbRobotSenseLoc = val
      elif key== 'sense_speed':
        self.mCbRobotSenseSpeed = val
      elif key== 'bellum_set':
        self.mCbRobotBellumSet = val

    # override bad options with defaults
    if k < 0.5:
      k = 0.5
    elif k > 4.0:
      k = 4.0
    self.mK = k

    # locals
    self.mSpeedMax  = float((KheBase.KheSpeedMmpsPosMax/1000) * 1000)
                              # keep it to even 1000's
    self.mGoalSpeed   = 0.0   # goal speed (mm/s)
    self.mGoalTheta   = 0.0   # goal direction (radians)
    self.mActualSpeed = 0.0   # actual speed (mm/s)
    self.mActualTheta = 0.0   # actual direction (radians)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create the widgets
    self.GuiBody(self)

    # refresh the navigator canvas
    self.NavCanvasRefresh()

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
      self.NavCanvasClearGrid()
      self.NavCanvasClearGoalsVector()
      self.NavCanvasClearActualsVector()
      self.mNavCanvasWidth = self.mWinGeo[gut.W] - self.mWinBorder
      self.mNavCanvasHeight = self.mWinGeo[gut.H] - self.mWinBorder \
                              - self.mCtlPanelFrameHeight
      self.mNavOrigin = self.mNavCanvasWidth/2, self.mNavCanvasHeight/2
      if self.mNavCanvasHeight <= self.mNavCanvasWidth:
        self.mNavRadius = self.mNavCanvasHeight/2 - 25
      else:
        self.mNavRadius = self.mNavCanvasWidth/2 - 25
      self.mNavCanvas.configure(width=self.mNavCanvasWidth,
                                height=self.mNavCanvasHeight)
      self.mStatusBar.configure(width=self.mNavCanvasWidth)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.NavCanvasRefresh()
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
    GuiWin.GuiWin.destroy(self, k_curvature=self.mK)


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
          'cfg'   - configure USS window.
                      run_time=<str>    - location's run-time state
          'loc' - show new location 
                      theta=<radians>   - robot orientation from start
                      force=<bool>      - force location update
          'speed' - show new speed 
                      pathspeed=<mm/s>  - current path speed
                      force=<bool>      - force speed update
          'goals'- show new goals 
                      speed=<mm/s>      - goal path speed
                      theta=<radians>   - goal orientation
                      force=<bool>      - force goals update

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'cfg':
      self.WinUpdateStatus(**kwargs)
    elif request == 'loc':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        theta = kwargs.get('theta', None)
        if theta is not None:
          self.WinUpdateActuals(self.mActualSpeed, theta)
          self.WinUpdateStatus(theta=theta)
    elif request == 'speed':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        pathspeed = kwargs.get('pathspeed', None)
        if pathspeed is not None:
          self.WinUpdateActuals(pathspeed, self.mActualTheta)
          self.WinUpdateStatus(pathspeed=pathspeed)
    elif request == 'goals':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        items = {'goal_speed':self.mGoalSpeed, 'goal_theta':self.mGoalTheta}
        for k,v in kwargs.iteritems():
          if k == 'speed':
            items['goal_speed'] = v
          elif k == 'theta':
            items['goal_theta'] = v
        self.WinUpdateGoals(items['goal_speed'], items['goal_theta'])
        self.WinUpdateStatus(**items)
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateActuals(self, speed, theta):
    """ Update vKhepera actual speed and direction data pushed by robot.

        Execution Context: GuiWin server thread

        Parameters:
          speed  - robot speed (mm/s)
          theta  - robot direction (radians)

        Return Value:
          None.
    """
    # update actual data
    self.mActualSpeed = speed
    self.mActualTheta = theta
    r = self._radius(math.fabs(self.mActualSpeed))
    isFwd = self.mActualSpeed >= 0.0

    # draw actuals vector
    self.NavCanvasDrawActualsVector(r, self.mActualTheta, isFwd)

  #--
  def WinUpdateGoals(self, speed, theta):
    """ Update vKhepera goal speed and direction data pushed by robot.

        Parameters:
          speed  - goal speed (mm/s)
          theta  - goal direction (radians)

        Return Value:
          None.
    """
    # update goal data
    self.mGoalSpeed = speed
    self.mGoalTheta = theta
    r = self._radius(math.fabs(self.mGoalSpeed))

    # sliders
    self.mSliderGoalSpeed.set(self.mGoalSpeed)
    self.mSliderGoalTheta.set(math.degrees(theta))

    # draw goals vector
    self.NavCanvasDrawGoalsVector(r, theta, speed >= 0.0)

  #--
  def WinUpdateStatus(self, **kwargs):
    """ Update status bar with trip data.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              run_time=<str>        - location's run-time state
              theta=<radians>       - robot orientation from start
              pathspeed=<mm/s>      - current path speed
              goal_speed=<mm/s>     - goal path speed
              goal_theta=<radians>  - goal orientation

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.iteritems():
      if k in ['pathspeed', 'goal_speed']:  # convert to meters
        items[k] = v / 1000.0
      elif k in ['theta', 'goal_theta']:    # convert to degrees [0,360)
        items[k] = math.degrees(v)
        if items[k] < 0.0:
          items[k] += 360.0
      elif k == 'run_time':
        items[k] = v
    self.mStatusBar.Update(**items)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Navigator Window Specifics
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

    # minimum width accepted
    minW = _CanvasMinWidth
    if cpgeo[gut.W] > minW:
      minW = cpgeo[gut.W]

    # make height equal width to start with
    minH = minW - _EdgeLeft - _EdgeRight + _EdgeTop + _EdgeBottom

    # window border width and height
    self.mWinBorder = self.mWinGeo[gut.W] - minW

    # set window's minimum size (320 obtained through experimentation)
    self.wm_minsize(width=minW+self.mWinBorder,
                    height=minH+self.mWinBorder+self.mCtlPanelFrameHeight)

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
    self.NavCanvasInit(parent, row, column)

    row += 1
    self.CtlPanelInit(parent, row, column)

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Trip Canvas Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def NavCanvasInit(self, parent, row, column):
    """ Initialize Navigator canvas.

        Parameters:
          parent  - parent to this canvas
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None.
    """
    self.mNavCanvasWidth  = _CanvasMinWidth
    self.mNavCanvasHeight = _CanvasMinHeight

    self.mGidGoalsVector    = None  # canvas item id for goals vector
    self.mGidActualsVector  = None  # canvas item id for actual velocity vector

    self.mNavCanvas = tk.Canvas(parent,
                                width = self.mNavCanvasWidth,
                                height = self.mNavCanvasHeight,
                                bg = 'black')
    self.mNavCanvas.grid(row=row, column=column)
    GuiToolTip.GuiToolTip(self.mNavCanvas, #follow_mouse=1,
        text='Left click to set forward speed and direction.\n'
             'Right click to set backward speed and direction.')

    self.mNavOrigin = self.mNavCanvasWidth/2, self.mNavCanvasHeight/2
    self.mNavRadius = self.mNavCanvasHeight/2 - 25
    self.mGidGrid   = []

    self.mNavCanvas.bind("<ButtonPress-1>", self.CbNavCanvasFwd)
    self.mNavCanvas.bind("<ButtonPress-3>", self.CbNavCanvasRev)

  #--
  def NavCanvasPaintGrid(self):
    """ Paint the Navigator canvas background grid. """
    # delete old grid
    #self.NavCanvasClearGrid()

    # origin
    orig = self.mNavOrigin

    # speed marks
    for speed in range(100, int(self.mSpeedMax)+100, 100):
      r = self._radius(speed)
      x0 = int(orig[0] - r)
      y0 = int(orig[1] - r)
      x1 = int(orig[0] + r)
      y1 = int(orig[1] + r)
      dim = x0, y0, x1, y1
      id = self.mNavCanvas.create_oval(dim, outline=gt.ColorWhite)
      self.mGidGrid += [id]
      if (speed/200) * 200 == speed:
        dim = orig[0] - 10, orig[1] - r + 2
        id = self.mNavCanvas.create_text(dim, fill=gt.ColorGreen,
          text='%d' % speed, anchor=tk.E)
        self.mGidGrid += [id]

    # direction marks
    for n in range(0, 180, 30):
      angle = math.radians(n)
      x = math.cos(angle) * self.mNavRadius
      y = math.sin(angle) * self.mNavRadius
      x0 = orig[0] + x
      y0 = orig[1] - y
      x1 = orig[0] - x
      y1 = orig[1] + y
      dim = x0, y0, x1, y1
      id = self.mNavCanvas.create_line(dim, fill=gt.ColorWhite)
      self.mGidGrid += [id]
      if n == 0:
        anchor1 = tk.W
        anchor2 = tk.E
      elif n <= 60:
        anchor1 = tk.SW
        anchor2 = tk.NE
      elif n == 90:
        anchor1 = tk.S
        anchor2 = tk.N
      elif n <= 150:
        anchor1 = tk.SE
        anchor2 = tk.NW
      dim = x0, y0
      id = self.mNavCanvas.create_text(dim, fill=gt.ColorGreen,
          text='%d' % n, anchor=anchor1)
      self.mGidGrid += [id]
      dim = x1, y1
      m = n + 180
      id = self.mNavCanvas.create_text(dim, fill=gt.ColorGreen,
          text='%d' % m, anchor=anchor2)
      self.mGidGrid += [id]
      
      # Khepera
      x0 = orig[0] - 5
      y0 = orig[1] - 5
      x1 = orig[0] + 5
      y1 = orig[1] + 5
      dim = x0, y0, x1, y1
      id = self.mNavCanvas.create_oval(dim, fill=gt.ColorGray2, 
          outline=gt.ColorBlack)
      self.mGidGrid += [id]

  #--
  def NavCanvasClearGrid(self):
    """ Clear the grid from the Navigator canvas. """
    for id in self.mGidGrid:
      self.mNavCanvas.delete(id)
    self.mGidGrid = []

  #--
  def NavCanvasClearGoalsVector(self):
    """ Clear the goals vector from the Navigator canvas. """
    if self.mGidGoalsVector is not None:
      self.mNavCanvas.delete(self.mGidGoalsVector)
      self.mGidGoalsVector = None

  #--
  def NavCanvasClearActualsVector(self):
    """ Clear the actual velocity vector from the Navigator canvas. """
    if self.mGidActualsVector is not None:
      self.mNavCanvas.delete(self.mGidActualsVector)
      self.mGidActualsVector = None

  #--
  def NavCanvasDrawGoalsVector(self, r, theta, isFwd=True):
    """ Draw the goals vector in the Navigator canvas.

        Parameters:
          r     - vector radius (pixel real units)
          theta - vector angle (radians)
          isFwd - sense is [not] in the forward direction

        Return Value:
          None.
    """
    self.NavCanvasClearGoalsVector()
    self.mGidGoalsVector = \
        self.NavCanvasDrawVector(r, theta, isFwd, _KheNavGoalsFwdColor, 
            _KheNavGoalsBwdColor)

  #--
  def NavCanvasDrawActualsVector(self, r, theta, isFwd=True):
    """ Draw the actual velocity vector in the Navigator canvas.

        Parameters:
          r     - vector radius (pixel real units)
          theta - vector angle (radians)
          isFwd - sense is [not] in the forward direction

        Return Value:
          None.
    """
    self.NavCanvasClearActualsVector()
    self.mGidActualsVector = \
        self.NavCanvasDrawVector(r, theta, isFwd, _KheNavBotFwdColor,
            _KheNavBotBwdColor)

  #--
  def NavCanvasDrawVector(self, r, theta, isFwd, fwdcolor, bwdcolor):
    """ Draw a r,theta vector in the Navigator canvas.

        Parameters:
          r         - vector radius (pixel real units)
          theta     - vector angle (radians)
          isFwd     - sense is [not] in the forward direction
          fwdcolor  - color of forwards vector
          bwdcolor  - color of backwards vector

        Return Value:
          Canvas id.
    """
    #print('goalvector', r, theta, isFwd)
    orig = self.mNavOrigin
    if r > 0.0:
      if isFwd:
        color = fwdcolor
        arrow = tk.LAST
      else:
        color = bwdcolor
        arrow = tk.FIRST
      x0 = orig[0]
      y0 = orig[1]
      x1 = int(x0 + math.cos(theta) * r)
      y1 = int(y0 - math.sin(theta) * r)
      dim = x0, y0, x1, y1
      vecId = self.mNavCanvas.create_line(dim, fill=color, arrow=arrow, width=3)
    else:
      x0 = orig[0] - 2
      y0 = orig[1] - 2
      x1 = orig[0] + 2
      y1 = orig[1] + 2
      dim = x0, y0, x1, y1
      vecId = self.mNavCanvas.create_oval(dim, fill=_KheNavGoalsNullColor,
                                          outline=_KheNavGoalsNullColor)
    return vecId

  #--
  def NavCanvasRefresh(self):
    """ Refresh the Navigator canvas. """
    self.NavCanvasClearGrid()
    self.NavCanvasClearGoalsVector()
    self.NavCanvasClearActualsVector()
    self.NavCanvasPaintGrid()

    # redraw goals vector _without_ reseting the current goal
    r = self._radius(math.fabs(self.mGoalSpeed))
    self.NavCanvasDrawGoalsVector(r, self.mGoalTheta, self.mGoalSpeed >= 0.0)

    # sync goal sliders with goal vector
    self.mSliderGoalSpeed.set(self.mGoalSpeed)
    self.mSliderGoalTheta.set(math.degrees(self.mGoalTheta))

    # redraw actual vector
    r = self._radius(math.fabs(self.mActualSpeed))
    self.NavCanvasDrawActualsVector(r, self.mActualTheta, self.mActualSpeed >= 0.0)

  #--
  def NavSetGoalsFromCoord(self, x, y, isFwd):
    """ Set vKhepera goals from the given Navigator canvas coordinates.

        Parameters:
          x     - Navigator canvas x coordinate
          y     - Navigator canvas y coordinate
          isFwd - left (forward) or right (backward) mouse click 

        Return Value:
          None.
    """
    r, theta = self._rect2polar(x, y)
    #print('polar', r, math.degrees(theta))
    if r > self.mNavRadius:
      return
    speed = self._speed(r)
    if not isFwd:
      speed = -speed
    self.NavSetGoals(r, theta, speed)

  #--
  def NavSetGoalsFromFields(self, speed, theta):
    """ Set vKhepera goals from the given Navigator frame fields.

        Parameters:
          speed  - field speed value (mm/s)
          theta  - field direction value (radians)

        Return Value:
          None.
    """
    r = self._radius(math.fabs(speed))
    self.NavSetGoals(r, theta, speed)

  #--
  def NavSetGoals(self, r, theta, speed):
    """ Set vKhepera goals. The physical Khepera will move if connected.

        Parameters:
          r      - radius
          theta  - goal direction (radians)
          speed  - goal speed (mm/s)

        Return Value:
          None.
    """
    # set goals data
    self.mGoalSpeed = speed
    self.mGoalTheta = theta

    # update goals' status, vector, and sliders
    self.WinQueueRequest('goals', speed=self.mGoalSpeed, theta=self.mGoalTheta,
        force=True)

    # now set robot's goals (robot callback will in turn update 'goals')
    if self.mCbRobotBellumSet:
      self.mCbRobotBellumSet(speed=speed, theta=theta)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the Navigator frame.

        Parameters:
          parent  - parent to this frame
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None
    """
    # control panel frame
    cpframe = tk.Frame(parent, relief=tk.FLAT, borderwidth=1)
    cpframe.grid(row=row, column=column, padx=1, ipadx=1,
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0

    # control panel title
    w = tk.Label(cpframe, text='vKhepera Navigator Control Panel',
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
    for n in [0, 0, 1, 2, 3, 4]:
      filename = gut.GetFusionImageFileName('FusionNav%d.gif' % n)
      if filename:
        autoFiles += [filename]
    manFile = gut.GetFusionImageFileName('FusionNavMan.gif')
    w = gut.ActiveImageWidget(subframe, 
        activesets={'auto':autoFiles, 'man':[manFile]}, period=0.25)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mWidgetActiveImage = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=1, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # locate button
    w = tk.Button(subframe, text='Locate', fg=gt.ColorBlack, width=8,
        command=self.CbLocate)
    GuiToolTip.GuiToolTip(w,
      text="Locate the robot's current speed and direction.")
    self.mButtonLocate = w

    subcol += 1

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Go to Manual Navigator Updates"
    self.mTtAutoManAutoText = "Go to Automatic Navigator Updates"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtAutoMan = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonAutoMan = w
    self.CtlPanelCfgAutoMan()

    subcol += 1

    # draw locate button
    self.mButtonLocate.grid(row=subrow, column=subcol, sticky=tk.W)

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # snapshot button
    w = tk.Button(subframe, text='SnapShot', fg=gt.ColorBlack, width=8,
        command=self.CbSnapShot)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Take a snap shot image (future).")

    subcol += 1

    # column spacer
    tk.Label(subframe, width=4, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # close button
    w = tk.Button(subframe, text="Close", width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0

    # subframe
    subframe = tk.Frame(cpframe, relief=tk.FLAT, borderwidth=0)
    subframe.grid(row=row, column=column, padx=1, pady=1, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0
 
    # speed label
    w = tk.Label(subframe, fg=gt.ColorBlack,
        text='Speed:')
    w.grid(row=subrow, column=subcol, sticky=tk.E)

    subcol += 1

    # speed slider
    w = tk.Scale(subframe, width=10, length=256,
          from_ = -KheBase.KheSpeedMmpsPosMax,
          to = KheBase.KheSpeedMmpsPosMax,
          resolution=KheBase.KheSpeedMmpsRes,
          orient=tk.HORIZONTAL, command=self.CbGoalSpeed)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Set robot's speed goal (mm/s).")
    self.mSliderGoalSpeed = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=1, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # park button
    w = tk.Button(subframe, text='Park', width=6, fg=gt.ColorRed1,
        command=self.CbPark)
    w.grid(row=subrow, column=subcol, rowspan=2, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Park robot at 0 degress.")

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # curvature slider
    w = tk.Scale(subframe, width=10, length=70,
          from_=0.5, to=4.0, resolution=0.5,
          orient=tk.VERTICAL, command=self.CbCurvature)
    w.grid(row=subrow, column=subcol, rowspan=2, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Set graphing curvature.")
    self.mSliderK = w
    self.mSliderK.set(self.mK)

    subcol += 1

    # curvature label
    w = tk.Label(subframe, fg=gt.ColorBlack, text='K')
    w.grid(row=subrow, column=subcol, rowspan=2, sticky=tk.E)

    subrow += 1
    subcol = 0

    # theta label
    w = tk.Label(subframe, fg=gt.ColorBlack,
        text='Theta:')
    w.grid(row=subrow, column=subcol, sticky=tk.E)

    subcol += 1

    # theta slider
    w = tk.Scale(subframe, width=10, length=256,
          from_ = 0.0, to = 360.0, resolution=0.5,
          orient=tk.HORIZONTAL, command=self.CbGoalTheta)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Set robot's direction goal (degrees).")
    self.mSliderGoalTheta = w

    row += 1
    column = 0
 
    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 4
    if self.mNavCanvasWidth > sbwidth:
      sbwidth = self.mNavCanvasWidth

    # Status Bar
    self.mStatusBar = GuiStatusBar.GuiStatusBar(cpframe,
        [ 
          {'tag': 'run_time',
           'prefix': 'run-time:',
           'max_width': 8,
           'val': 'unknown',
           'tooltip': "Robot's location run-time state.\n"
                      "See Robot|Robot Options..."
          },
          {'tag': 'goal_speed',
           'prefix': 'goal speed:',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Current speed goal (m/s)."
          },
          {'tag': 'goal_theta',
           'prefix': 'goal ' + gt.UniGreek['theta'] + ':',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Current direction goal (degrees)."
          },
          {'tag': 'pathspeed',
           'prefix': 'speed:',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Actual speed (m/s)."
          },
          {'tag': 'theta',
           'prefix': gt.UniGreek['theta'] + ':',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Actual direction (degrees)."
          }
        ],
        initWidth=sbwidth,
        maxRows=2)
    self.mStatusBar.grid(row=row, column=column, pady=3, sticky=tk.W)

  #--
  def CtlPanelCfgAutoMan(self):
    """ Place control panel into automatic/manual update configuration. """
    w = self.mButtonAutoMan
    if self.mIsAutoMode == True:
      w['text']             = 'Manual'
      w['fg']               = gt.ColorBttnStop
      w['activeforeground'] = gt.ColorBttnStop
      self.mTtAutoMan.newtip(self.mTtAutoManManText)
      self.mButtonLocate['state'] = tk.DISABLED
      self.mWidgetActiveImage.SetActive('auto')
    else:
      w['text']             = 'Auto'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtAutoMan.newtip(self.mTtAutoManAutoText)
      self.mButtonLocate['state'] = tk.NORMAL
      self.mWidgetActiveImage.SetActive('man')


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
  def CbLocate(self):
    """ Force USS ping """
    if self.mCbRobotSenseLoc:
      loc = self.mCbRobotSenseLoc()
      self.WinQueueRequest('loc', force=True, **loc)
    if self.mCbRobotSenseSpeed:
      speed = self.mCbRobotSenseSpeed()
      self.WinQueueRequest('speed', force=True, **speed)

  #--
  def CbSnapShot(self):
    """ Take a snap shot image callback. """
    pass

  #--
  def CbClose(self):
    """ Close window callback. """
    self.destroy()

  #--
  def CbGoalSpeed(self, val):
    """ Set speed goal slider callback. """
    speed = self.mSliderGoalSpeed.get()
    if speed != self.mGoalSpeed:
      self.NavSetGoalsFromFields(speed, self.mGoalTheta)

  #--
  def CbGoalTheta(self, val):
    """ Set theta goal slider callback. """
    theta = self.mSliderGoalTheta.get()
    theta = math.radians(theta)
    if theta != self.mGoalTheta:
      self.NavSetGoalsFromFields(self.mGoalSpeed, theta)

  #--
  def CbPark(self):
    """ Park robot button callback. """
    self.NavSetGoalsFromFields(0.0, 0.0)

  #--
  def CbCurvature(self, val):
    """ Set graph curvature slider callback. """
    self.mK = self.mSliderK.get()
    self.NavCanvasRefresh()

  #--
  def CbNavCanvasFwd(self, event):
    """ Navigator Canvas mouse button-1 press event callback. """
    # screen width and height (current monitor resolution)
    # not needed, but keep as a reference
    #w = self.mNavCanvas.winfo_screenwidth()
    #h = self.mNavCanvas.winfo_screenheight()

    # nav canvas width and height
    # not needed, but keep as a reference
    #wx = self.mNavCanvas.winfo_reqwidth()
    #wy = self.mNavCanvas.winfo_reqheight()
    
    # nav canvas screen origin
    origx = self.mNavCanvas.winfo_rootx()
    origy = self.mNavCanvas.winfo_rooty()

    # mouse screen coordinates
    x = self.mNavCanvas.winfo_pointerx()
    y = self.mNavCanvas.winfo_pointery()

    # convert to navigator coordinates
    mousex = x - origx
    mousey = y - origy
    #print('navmouse', mousex, mousey)

    self.NavSetGoalsFromCoord(mousex, mousey, True)

  #--
  def CbNavCanvasRev(self, event):
    """ Navigator Canvas mouse button-3 press event callback. """
    # nav canvas screen origin
    origx = self.mNavCanvas.winfo_rootx()
    origy = self.mNavCanvas.winfo_rooty()

    # mouse screen coordinates
    x = self.mNavCanvas.winfo_pointerx()
    y = self.mNavCanvas.winfo_pointery()

    # convert to navigator coordinates
    mousex = x - origx
    mousey = y - origy
    #print('navmouse', mousex, mousey)

    self.NavSetGoalsFromCoord(mousex, mousey, False)
 

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Messy Details
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _rect2polar(self, x, y):
    """ Convert navigator canvas rectangular coordinates to 
        polar coordinates.

        Parameters:
          x  - Navigator canvas x coordinate
          y  - Navigator canvas y coordinate

        Return Value:
          (radius, theta) - equivalent vector radius and angle 
                            CCW from x-axis
    """
    x = float(x - self.mNavOrigin[0]) # x larger to the right
    y = float(self.mNavOrigin[1] - y) # y smaller to the top
    #print('x,y', x, y)
    r = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    if x != 0.0:
      theta = math.atan(y/x) 
    else:
      theta = math.radians(90)
    #print('raw theta', theta, math.degrees(theta))

    # fix up angle for different quadrants [0, 360)
    if x == 0 and y < 0.0:      # 270 is strange
      theta = math.radians(270)
    elif x > 0.0 and y < 0.0:   # quadrants III and IV
      theta = math.radians(360) + theta
    elif x < 0.0:               # quadrant II
      theta = math.radians(180) + theta
    return r, theta

  #--
  def _speed(self, r):
    """ Calculate the speed given the radius. The function is more
        sensitve in the lower speeds.

        Parameters:
          r  - radius

        Return Value:
          speed
    """
    if r <= 0.0:
      return 0.0
    elif r >= self.mNavRadius:
      return self.mSpeedMax
    return ((r / self.mNavRadius) * self.mSpeedMax) * \
           math.exp(self.mK * (r / self.mNavRadius - 1.0))

  #--
  def _radius(self, s):
    """ Calculate the radius given the speed. This is the inverse of
        _speed(). Newton's method is used where:
                      c(s) = r * exp(u)  ==> f(r) = r * exp(u) - c(s)

        Parameters:
          s  - speed

        Return Value:
          radius
    """
    if s <= 0.0:
      return 0.0
    elif s >= self.mSpeedMax:
      return self.mNavRadius
    r0 = self.mNavRadius / 2.0    # starting approximation
    cs = (s * self.mNavRadius) / self.mSpeedMax  # c(s)
    #print('cs', cs)
    du = self.mK / self.mNavRadius           # du/dr
    n = 0
    #print('>', r0)
    while n < 15:
      u = self.mK * (r0 / self.mNavRadius - 1.0) # u(r)
      eu = math.exp(u)          # exp(u)
      r1 = r0 - (r0 * eu - cs) / (eu * (1.0 + r0 * du)) # r0 - f(r0)/f'(r0)
      #print(n, r1)
      if math.fabs(r1-r0) < 0.001:
        return r1
      r0 = r1
      n += 1
    #print(n, r1)
    return r1


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import time
  import random
  import Fusion.Core.Gluon as Gluon
  import Fusion.Utils.WinUT as WinUT
  import Fusion.Utils.IVTimer as IVTimer
  import Fusion.Khepera.Robots.vKhepera as vKhepera

  twopi = math.pi * 2.0

  #--
  class WinUTKheNav(WinUT.WinUT):
    """ GuiWinKheNav Unit Test Window. """

    #--
    def __init__(self, robot):
      # the unit test
      ut = {
        'Null Test': self.utNone,
        'Go Test': self.utGo,
        'Perturb Robot Test': self.utPerturbStart
      }

      self.mRobot = robot
      self.mIvtPush = None

      WinUT.WinUT.__init__(self, title="GuiWinKheNav Unit Test", ut=ut)

    #--
    def wut_cancel(self):
      if self.mIvtPush:
        self.mIvtPush.cancel()
      if self.mRobot.GetRobotState() != Gluon.EServerState.NotLoaded:
        self.mRobot.ExecUnload()
      WinUT.WinUT.wut_cancel(self)

    #--
    def sut_start_khepera(self):
      """ Start the vKhepera. """
      self.mRobot.mCmd.Open('/dev/ttyS0', baudrate=115200)
      self.mRobot.SetCommStatus(True)
      self.mRobot.ExecLoad()
      self.mRobot.ExecStart()

    #--
    def sut_push_start(self):
      self.mIvtPush = IVTimer.IVTimer(1.50, 0.25, self.sut_push_iter, once=True)
      self.mIvtPush.start()

    #--
    def sut_push_iter(self, ivt):
      """ Push location data from khepera to trip window ivt callback. """
      if not self.mSut:
        return
      if ivt.once:
        self.mSut.WinQueueRequest('cfg', run_time="enabled")
        ivt.once = False
      else:
        d = self.mRobot.SenseOdometer()
        self.mSut.WinQueueRequest('loc', **d)
        d = self.mRobot.SenseSpeedometer()
        self.mSut.WinQueueRequest('speed', **d)
        d = self.mRobot.BellumGetGoals()
        self.mSut.WinQueueRequest('goals', **d)

    #--
    # THE UNIT TEST
    #--

    #--
    def utNone(self):
      """ Nada """
      self.wut_showstatus('The robot is not connected, nor running.')
      self.wut_showstatus('Just show the navigator window.')
  
    #--
    def utGo(self):
      """ Go """
      self.wut_showstatus('The robot is connected and running.')
      self.sut_start_khepera()
      self.sut_push_start()
  
    #--
    def utPerturbStart(self):
      """ Perturb Goals Unit Test Start. """
      self.wut_showstatus('The robot is connected and running.')
      self.sut_start_khepera()
      self.sut_push_start()
      self.wut_showstatus('Perturb robot to test tracking and recovery.')
      self.mIvt = IVTimer.IVTimer(5.0, 5.0, self.utPerturbIter)
      self.mIvt.start()
  
    #--
    def utPerturbIter(self, ivt):
      """ Perturb Goals Unit Test Iterator"""
      l = random.randint(-256, 256)
      r = random.randint(-256, 256)
      self.wut_showstatus("Perturbed: lmotor=%d, rmotor=%d" % (l, r))
      self.mRobot.mCmd.CmdSetSpeed(l, r)
  
  #--
  def main():
    """ GuiWinKheNav Unit Test Main """
    robot = vKhepera.vKhepera()
    winUT = WinUTKheNav(robot)
    winSut = GuiWinKheNav(winUT.wut_this(),
                          sense_loc=robot.SenseOdometer,
                          sense_speed=robot.SenseSpeedometer,
                          bellum_set=robot.BellumSetGoals)
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
