################################################################################
#
# GuiWinKheTrip
#

""" Graphical User Interface Khepera Trip Log Window

Graphical User Interface (GUI) Tkinter trip window displays current
trip log, plus simple display options.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.10

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

import math
import time

import Tkinter as tk

import Fusion.Core.Values as Values

import Fusion.Utils.Trip as Trip

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import Fusion.Gui.GuiWin as GuiWin

import Fusion.Khepera.Cmd.KheCmdBase as KheBase


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# waypoint triplet indices
X     = Trip.X      # mm
Y     = Trip.Y      # mm
THETA = Trip.THETA  # radians

LOG10   = 'log10'   # log-log scale
LINEAR  = 'linear'  # linear scale

_ILOG   = 0         # index into log functions or data
_ILIN   = 1         # index into linear functions or data

_LogOrig  = 0.001   # log origin
_LogMin   = 0.01    # 10 ^ -2 meters
_LogMax   = 10.0    # 10 ^ 1 meters
_LogMult  = 1000.0  # multiplier log10(min * mult) -> 1.0

_LinMin   =  1.0    # 10 ^ 0 meters
_LinMax   = 10.0    # 10 ^ 1 meters

_MMPM     = 1000.0  # millimeters / meter (doh)

# trip canvas dimensions
_EdgeLeft   = 45    # left edge margin
_EdgeTop    = 10    # top edge margin
_EdgeBottom = 15    # bottom edge margin
_EdgeRight  = 15    # right edge margin

# minimum size
_CanvasMinWidth   = 400 + _EdgeLeft + _EdgeRight
_CanvasMinHeight  = 400 + _EdgeTop + _EdgeBottom

# 1/10th tick functions
_LogTenth = lambda tick,spacing: int(math.log10(tick) * spacing)
_LinTenth = lambda tick,spacing: int(tick/10.0 * spacing)

# math thingies
twopi = math.pi * 2.0


#-------------------------------------------------------------------------------
# CLASS: GuiWinKheTrip
#-------------------------------------------------------------------------------
class GuiWinKheTrip(GuiWin.GuiWin):
  """ GUI Window vKhepera Trip Log Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the vKhepera Trip Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Trip options. Options are:
            auto=<bool>       - do [not] automatically update
            scale=<val>       - map scaling: One of: 'log10' or 'linear'
            range=<m>         - map range [-range, range] in meters. [1-10]
            sense_loc=<cb>    - sense location callback to vRobot
            sense_speed=<cb>  - sense speed callback to vRobot
            reset_loc=<cb>    - reset location callback to vRobot
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = 'vKhepera Trip Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          vKhepera  - vKhepera object.
          options   - trip input options.

        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode        = True
    self.mRangeLog          = _LogMax
    self.mRangeLin          = _LinMax
    self.mCbRobotSenseLoc   = None
    self.mCbRobotSenseSpeed = None
    self.mCbRobotResetLoc   = None

    scale = LOG10
    range = None

    # set options from input parameters
    for k,v in options.iteritems():
      if k == 'auto':
        if v:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False
      elif k == 'scale':
        scale = v
      elif k == 'range':
        try:
          range = float(v)
        except:
          range = None
      elif k == 'sense_loc':
        self.mCbRobotSenseLoc = v
      elif k == 'sense_speed':
        self.mCbRobotSenseSpeed = v
      elif k == 'reset_loc':
        self.mCbRobotResetLoc = v

    # override bad options with defaults
    if scale not in [LOG10, LINEAR]:
      scale = LOG10

    # adjust range to current scale
    if range is not None:
      if scale == LOG10:
        range = float(range)
        if range < _LogOrig:
          self.mRangeLog = _LogMin
        else:
          self.mRangeLog = _LogOrig * 10**math.ceil(math.log10(range*_LogMult))
          if self.mRangeLog > _LogMax:
            self.mRangeLog = _LogMax
      else:  # linear
        range = int(range)
        if range < _LinMin:
          self.mRangeLin = _LinMin
        elif range > _LinMax:
          self.mRangeLin = _LinMax
        else:
          self.mRangeLin = range

    # locals
    self.mScale       = scale           # log10 or linear
    self.mBotTrip     = Trip.trip()     # the robot's world trip
    self.mMinResDist  = KheBase.KheOdometerMmpt / 2.0 # minimum trip resolution
    self.mMinResTheta = twopi / 720.0   # half a degree


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create the widgets
    self.GuiBody(self)

    # refresh the trip canvas
    self.TripCanvasRefresh()

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
      self.TripCanvasClearBot()
      self.TripCanvasClearTrip()
      self.TripCanvasClearGrid()
      width = geo[gut.W] - self.mWinBorder
      height = geo[gut.H] - self.mWinBorder - self.mCtlPanelFrameHeight
      self.mTripCanvas.configure(width=width, height=height)
      self.TripCanvasParams(width, height)
      width = geo[gut.W] - self.mWinStatusBarBorder
      self.mStatusBar.configure(width=width)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.TripCanvasPaintGrid()
      self.TripCanvasDrawTrip()
      self.TripCanvasDrawBot()
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
    if self.mScale == LOG10:
      range = self.mRangeLog
    else:
      range = self.mRangeLin
    GuiWin.GuiWin.destroy(self, auto=self.mIsAutoMode, scale=self.mScale,
        range=range)


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
                      x=<mm>            - x location from start
                      y=<mm>            - y location from start
                      theta=<radians>   - robot orientation from start
                      pathdist=<mm>     - total path distance traveled
                      force=<bool>      - force location update
          'speed' - show new speed 
                      pathspeed=<mm/s>  - current path speed
                      force=<bool>      - force speed update
          'clear' - clear current set of waypoints and redraw screen
          'reset' - reset robot's odometry to zero and clear the trip
                    and map

        Return Values:
          None
    """
    #print("Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'cfg':
      self.WinUpdateStatus(**kwargs)
    elif request == 'loc':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.WinUpdateTrip(kwargs['x'], kwargs['y'], kwargs['theta'])
        self.WinUpdateStatus(**kwargs)
    elif request == 'speed':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.WinUpdateStatus(**kwargs)
    elif request == 'clear':
      self.WinUpdateClear()
    elif request == 'reset':
      self.WinUpdateReset()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))

  #--
  def WinUpdateTrip(self, x, y, theta):
    """ Update trip with new waypoint.

        Execution Context: GuiWin server thread

        Parameters:
          x     - new x location (mm)
          y     - new y location (mm)
          theta - new orientation (radians)

        Return Value:
          None.
    """
    # don't add new waypoint if within L1-distance minimum trip resolution
    # and there has been little orientation change
    if self.mBotTrip.HasNumWaypoints() > 0:
      p = self.mBotTrip.GetWaypoint(Trip.LAST)
      if  math.fabs(x-p[X]) + math.fabs(y-p[Y]) < self.mMinResDist and \
          math.fabs(theta-p[THETA]) < self.mMinResTheta:
        return

    # add new leg to trip
    self.mBotTrip.AddWaypoint(x, y, theta)

    # show trip leg if map not frozen
    if self.mBotTrip.HasNumWaypoints() > 1:
      self.TripCanvasDrawLeg(self.mBotTrip.GetWaypoint(Trip.PENULT),
                             self.mBotTrip.GetWaypoint(Trip.LAST))
    self.MoveBotLoc()

  #--
  def WinUpdateStatus(self, **kwargs):
    """ Update status bar with trip data.

        Execution Context: GuiWin server thread

        Parameters:
          **kwargs   - Status keyword arguments. Specific keyword=val:
              run_time=<str>    - location's run-time state
              x=<mm>            - x location from start
              y=<mm>            - y location from start
              theta=<radians>   - robot orientation from start
              pathdist=<mm>     - total path distance traveled
              pathspeed=<mm/s>  - current path speed

        Return Value:
          None.
    """
    items = {}
    for k,v in kwargs.iteritems():
      if k in ['x', 'y', 'pathdist', 'pathspeed']:  # convert to meters
        items[k] = v / 1000.0
      elif k == 'theta':      # convert to degrees [0,360)
        items[k] = math.degrees(v)
        if items[k] < 0.0:
          items[k] += 360.0
      elif k == 'run_time':
        items[k] = v
    self.mStatusBar.Update(**items)

  #--
  def WinUpdateClear(self):
    """ Clear the trip's current set of waypoints and refresh the window.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    self.TripCanvasClearBot()
    self.TripCanvasClearTrip()
    try:
      self.mBotTrip.DelSubTrip(Trip.FIRST, Trip.LAST)
    except IndexError:
      pass
    self.TripCanvasDrawTrip()
    self.MoveBotLoc()

  #--
  def WinUpdateReset(self):
    """ Reset the robot's odometry and location, clear the trip's current
        set of waypoints and refresh the window.

        Execution Context: GuiWin server thread

        Return Value:
          None.
    """
    if self.mCbRobotResetLoc:
      self.mCbRobotResetLoc()
    self.WinUpdateClear()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Trip Window Specifics
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
    self.TripCanvasInit(parent, row, column)

    row += 1
    self.CtlPanelInit(parent, row, column)

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Trip Canvas Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def TripCanvasInit(self, parent, row, column):
    """ Initialize Trip canvas.

        Parameters:
          parent  - parent to this canvas
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None.
    """
    self.TripCanvasParams(_CanvasMinWidth, _CanvasMinHeight)

    self.mTripCanvas = tk.Canvas(parent,
                                width = self.mTripCanvasWidth,
                                height = self.mTripCanvasHeight,
                                bg = '#999999')
    self.mTripCanvas.grid(row=row, column=column)

    # canvas graphic ids
    self.mGidGrid   = []  # map grid
    self.mGidTrip   = []  # trip
    self.mGidBot    = []  # robot

  #--
  def TripCanvasParams(self, width, height):
    """ Set canvas view parameters.

        Parameters:
          width   - width of canvas (pixels)
          height  - height of canvas (pixels)

        Return Value:
          None
    """
    # screen coordinates
    self.mTripCanvasWidth, self.mTripCanvasHeight = width, height
    self.mGridWidth         = self.mTripCanvasWidth - _EdgeLeft - _EdgeRight
    self.mGridHeight        = self.mTripCanvasHeight - _EdgeTop - _EdgeBottom
    self.mGridMinX          = _EdgeLeft
    self.mGridMinY          = _EdgeTop
    self.mGridMaxX          = self.mGridMinX + self.mGridWidth
    self.mGridMaxY          = self.mGridMinY + self.mGridHeight
    self.mGridOrigin        = self.mGridMinX + self.mGridWidth/2, \
                              self.mGridMinY + self.mGridHeight/2
    self.mGridMajorCntLog   = int(math.log10(self.mRangeLog * _LogMult))
    self.mGridMajorCntLin   = int(self.mRangeLin)
    self.mGridMajorPixLogX  = self.mGridWidth/(2 * self.mGridMajorCntLog)
    self.mGridMajorPixLogY  = self.mGridHeight/(2 * self.mGridMajorCntLog)
    self.mGridMajorPixLinX  = self.mGridWidth/(2 * self.mGridMajorCntLin)
    self.mGridMajorPixLinY  = self.mGridHeight/(2 * self.mGridMajorCntLin)

  #--
  def TripCanvasPaintGrid(self):
    """ Paint the Trip canvas background grid. """
    if self.mScale == LOG10:
      self._tripCanvasPaintGridLog10()
    else:
      self._tripCanvasPaintGridLin()

  #--
  def TripCanvasClearGrid(self):
    """ Clear the Trip canvas background grid. """
    for id in self.mGidGrid:
      self.mTripCanvas.delete(id)
    self.mGidGrid = []

  #--
  def TripCanvasClearTrip(self):
    """ Clear the trip map from the Trip canvas. """
    for id in self.mGidTrip:
      self.mTripCanvas.delete(id)
    self.mGidTrip = []

  #--
  def TripCanvasClearBot(self):
    """ Clear the bot from Trip canvas. """
    for id in self.mGidBot:
      self.mTripCanvas.delete(id)
    self.mGidBot = []

  #--
  def TripCanvasDrawBot(self):
    """ Draw the bot on the Trip canvas. """
    try:
      triplet = self.mBotTrip.GetWaypoint(Trip.LAST)
    except IndexError:
      return
    c = self._world2canvas(triplet)
    dim = c[X] - 5, c[Y] - 5, c[X] + 5, c[Y] + 5
    id = self.mTripCanvas.create_oval(dim, fill=gt.ColorGray2, 
                                                   outline=gt.ColorBlack)
    self.mGidBot += [id]
    theta = c[THETA]
    x = c[X] + 5 * math.cos(theta)
    y = c[Y] - 5 * math.sin(theta)
    id = self.mTripCanvas.create_line((c[X], c[Y], x, y), fill=gt.ColorBlue1) 
    self.mGidBot += [id]

  #--
  def TripCanvasDrawTrip(self, start=Trip.FIRST, end=Trip.LAST):
    """ Draw the trip defined by the existing waypoints on the 
        Trip canvas.

        Parameters:
          start - starting waypoints index
          end   - last waypoints index. If 'last', the last waypoint index
    """
    if self.mBotTrip.HasNumWaypoints() < 2:
      return
    for p1, p2 in self.mBotTrip.iterlegs(start, end):
      self.TripCanvasDrawLeg(p1, p2)

  #--
  def TripCanvasDrawLeg(self, p1, p2):
    """ Draw a trip leg on the Trip canvas.

        Parameters:
          p1    - starting point
          p2    - ending point

        Return Value:
          None
    """
    #print('(%.2f, %.2f) (%.2f, %.2f)' % (p1[X], p1[Y], p2[X], p2[Y]))
    p1 = self._world2canvas(p1)
    p2 = self._world2canvas(p2)
    #print(p1, p2)
    dim = p1[X], p1[Y], p2[X], p2[Y]
    id = self.mTripCanvas.create_line(dim, fill=gt.ColorGreenBlue)
    self.mGidTrip += [id]

  #--
  def TripCanvasRefresh(self):
    """ Refresh the Trip canvas. """
    self.TripCanvasClearBot()
    self.TripCanvasClearTrip()
    self.TripCanvasClearGrid()
    self.TripCanvasPaintGrid()
    self.TripCanvasDrawTrip()
    self.TripCanvasDrawBot()

  #--
  def MoveBotLoc(self):
    """ Move the bot to the last waypoint location. """
    self.TripCanvasClearBot()
    self.TripCanvasDrawBot()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Control Panel 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CtlPanelInit(self, parent, row, column):
    """ Create the Trip Control Panel.

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
    w = tk.Label(cpframe, text='vKhepera Trip Control Panel', fg=gt.ColorGreen1)
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
    for n in [0, 1, 2, 3, 3]:
      filename = gut.GetFusionImageFileName('FusionMap%d.gif' % n)
      if filename:
        autoFiles += [filename]
    manFile = gut.GetFusionImageFileName('FusionMapMan.gif')
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
    GuiToolTip.GuiToolTip(w, text="Locate the robot's current position.")
    self.mButtonLocate = w

    subcol += 1

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Go to Manual Trip Updates"
    self.mTtAutoManAutoText = "Go to Automatic Trip Updates"
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

    # dump button
    w = tk.Button(subframe, text='Dump...', fg=gt.ColorBlack, width=8,
        command=self.CbDump)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w,
        text="Dump the trip data to a file.")

    subcol += 1

    # column spacer
    tk.Label(subframe, width=3, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # close button
    w = tk.Button(subframe, text='Close', fg=gt.ColorBlack, width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0

    # subframe
    subframe = tk.Frame(cpframe, relief=tk.FLAT, borderwidth=0)
    subframe.grid(row=row, column=column, padx=1, pady=1, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    subrow = 0
    subcol = 0

    # range slider label
    w = tk.Label(subframe, text='Range:\n ', width=16, fg=gt.ColorBlack)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    self.mLabelRange = w

    subcol += 1

    # range slider
    w = tk.Scale(subframe, width=10, length=120, from_=1, to=10,
        orient=tk.HORIZONTAL, command=self.CbRange)
    w.grid(row=subrow, column=subcol, sticky=tk.W, pady=1)
    GuiToolTip.GuiToolTip(w, text="Maximum and minimum map domain and range")
    self.mSliderRange = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=1, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # scale button
    w = tk.Button(subframe, fg=gt.ColorBlack, width=8, command=self.CbScale)
    w.grid(row=subrow, column=subcol)
    self.mTtScale = GuiToolTip.GuiToolTip(w, text="")
    self.mButtonScale = w

    subcol += 1

    # column spacer
    tk.Label(subframe, width=2, text=' ').grid(row=subrow, column=subcol)

    subcol += 1

    # clear button
    w = tk.Button(subframe, text='Clear', fg=gt.ColorBlack, width=8,
        command=self.CbTripClear)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Clear the trip data and visualization")

    subcol += 1
 
    # reset button
    w = tk.Button(subframe, text='Reset', fg=gt.ColorBlack, width=8,
        command=self.CbTripReset)
    w.grid(row=subrow, column=subcol, sticky=tk.E)
    GuiToolTip.GuiToolTip(w,
        text="Reset the robot's odometry and\n"
             "clear the trip data and visualization.")

    row += 1
    column = 0
 
    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 4
    if self.mTripCanvasWidth > sbwidth:
      sbwidth = self.mTripCanvasWidth

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
          {'tag': 'x',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Current x location (meters)"
          },
          {'tag': 'y',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Current y location (meters)"
          },
          {'tag': 'theta',
           'prefix': gt.UniGreek['theta'] + ':',
           'max_width': 7,
           'val': 0,
           'fmt': '%5.1f',
           'tooltip': "Current orientation (degrees)"
          },
          {'tag': 'pathdist',
           'prefix': 'dist:',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Total path distance traveled (meters)"
          },
          {'tag': 'pathspeed',
           'prefix': 'speed:',
           'max_width': 7,
           'val': 0,
           'fmt': '%7.3f',
           'tooltip': "Current path speed (meters/s)"
          },
        ],
        initWidth=sbwidth,
        maxRows=2)
    self.mStatusBar.grid(row=row, column=column, pady=3, sticky=tk.W)

    # set control panel initial state
    if self.mScale == LOG10:
      self.CtlPanelStateLog()
    else:
      self.CtlPanelStateLin()

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

  #--
  def CtlPanelStateLog(self):
    """ Set Control Panel widgets' states to log10 scale. """
    self.mButtonScale['text'] = 'Linear'
    self.mTtScale.newtip("Set linear scale")
    self.mSliderRange['from'] = int(math.log10(_LogMin))
    self.mSliderRange['to']   = int(math.log10(_LogMax))
    self.mSliderRange.set(math.log10(self.mRangeLog))
    self.mLabelRange['text'] = 'Map Zoom:\n(meters x 10^n)'

  #--
  def CtlPanelStateLin(self):
    """ Set Control Panel widgets' states to linear scale. """
    self.mButtonScale['text'] = 'Log10'
    self.mTtScale.newtip("Set log10-log10 scale")
    self.mSliderRange['from'] = int(_LinMin)
    self.mSliderRange['to']   = int(_LinMax)
    self.mSliderRange.set(int(self.mRangeLin))
    self.mLabelRange['text'] = 'Map Zoom:\n(meters)'


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
  def CbRange(self, val):
    """ Change the map range [-range, range] callback. """
    if self.mScale == LOG10:
      oldPow10 = math.log10(self.mRangeLog)
      newPow10 = float(val)
      if newPow10 == oldPow10:
        return
      newRangeLog = 10.0 ** newPow10
      newRangeLin = self.mRangeLin
    else:
      newRangeLin = int(val)
      if newRangeLin == self.mRangeLin:
        return
      newRangeLog = self.mRangeLog
    self.mRangeLog = newRangeLog
    self.mRangeLin = newRangeLin
    self.TripCanvasParams(self.mTripCanvasWidth, self.mTripCanvasHeight)
    self.TripCanvasRefresh()

  #--
  def CbScale(self):
    """ Log10/Linear Scale callback. """
    if self.mScale == LOG10:
      newScale = LINEAR
      self.CtlPanelStateLin()
    else:
      newScale = LOG10
      self.CtlPanelStateLog()
    self.mScale = newScale
    self.TripCanvasRefresh()
    self.MoveBotLoc()

  #--
  def CbTripClear(self):
    """ Clear the viz's current set of data callback. """
    self.WinQueueRequest('clear')

  #--
  def CbTripReset(self):
    """ Reset robot's odometry and clear the current set of data callback. """
    self.WinQueueRequest('reset')

  #--
  def CbSnapShot(self):
    """ Take a snap shot image callback. """
    pass

  #--
  def CbDump(self):
    """ Dump waypoints to file callback. """
    ltime = time.localtime()
    initialfile = 'wp_%d%02d%02d_%02d%02d%02d.py' % \
        (ltime[0], ltime[1], ltime[2], ltime[3], ltime[4], ltime[5])
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self, self._cbSave,
                  title='Save vKhepera Waypoints As',
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
    print('# vKhepera Trip Waypoints', file=fp)
    ltime = time.localtime()
    print('# %d.%02d.%02d %02d:%02d:%02d' % \
        (ltime[0], ltime[1], ltime[2], ltime[3], ltime[4], ltime[5]), file=fp)
    print('#\n', file=fp)
    print('# waypoint triplet: (x(mm), y(mm), theta(radians))', file=fp)
    print('vKheperaWaypoints = [', end='', file=fp)
    n = 0
    sep = ''
    for p in self.mBotTrip:
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
  def CbClose(self):
    """ Close window callback. """
    self.destroy()
 

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Messy Details
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _world2canvas(self, p):
    """ Convert world coordinates to canvas coordinates.

        Parameters:
          p - (x, y [,theta]) in world coordinates mm

        Return Value:
          q - (x, y, [,theta]) in canvas coordinates
    """
    q = [0.0, 0.0]  # fake point

    # log 10 scale
    if self.mScale == LOG10:
      for i in [X, Y]:
        v = math.fabs(p[i])
        if v <= 1.0:    # < 1.0 mm
          v = 0.0
        else:
          v = math.log10(v)
        if p[i] < 0.0:
          v = -v
        q[i] = v

      # map into canvas coordinates
      q[X] = int(self.mGridOrigin[X] + q[X] * self.mGridMajorPixLogX)
      q[Y] = int(self.mGridOrigin[Y] - q[Y] * self.mGridMajorPixLogY)

    # linear scale
    else:
      q[X] = int(self.mGridOrigin[X] + (p[X] * self.mGridMajorPixLinX) / _MMPM)
      q[Y] = int(self.mGridOrigin[Y] - (p[Y] * self.mGridMajorPixLinY) / _MMPM)

    # clip (not needed, since Canvas does a nice job of it)

    if len(p) == 3:
      return (q[X], q[Y], p[THETA])
    else:
      return (q[X], q[Y])


  #--
  def _tripCanvasPaintGridLog10(self):
    """ Paint the Trip canvas background log-log grid. """
    # x-y axes
    self._xyaxes()

    # x grid 
    y0 = self.mGridMinY
    y1 = self.mGridMaxY
    for xgrid in range(1, self.mGridMajorCntLog+1):
      # x positive major grid lines
      x = self.mGridOrigin[X] + xgrid * self.mGridMajorPixLogX
      id = self.mTripCanvas.create_line((x, y0, x, y1), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # x positive minor ticks
      self._xlogtick(xgrid)

      # x negative major grid lines
      x = self.mGridOrigin[X] - xgrid * self.mGridMajorPixLogX
      id = self.mTripCanvas.create_line((x, y0, x, y1), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # x negative minor ticks
      self._xlogtick(-xgrid)

    # x axis labels
    self._xloglabels()

    # y grid 
    x0 = self.mGridMinX
    x1 = self.mGridMaxX
    for ygrid in range(1, self.mGridMajorCntLog+1):
      # y positive major grid lines
      y = self.mGridOrigin[Y] - ygrid * self.mGridMajorPixLogY
      id = self.mTripCanvas.create_line((x0, y, x1, y), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # y positive minor ticks
      self._ylogtick(ygrid)

      # y negative major grid lines
      y = self.mGridOrigin[Y] + ygrid * self.mGridMajorPixLogY
      id = self.mTripCanvas.create_line((x0, y, x1, y), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # y negative minor ticks
      self._ylogtick(-ygrid)

    # y axis labels
    self._yloglabels()

  #--
  def _tripCanvasPaintGridLin(self):
    """ Paint the Trip canvas background linear grid. """
    # x-y axes
    self._xyaxes()

    # x grid 
    y0 = self.mGridMinY
    y1 = self.mGridMaxY
    for xgrid in range(1, self.mGridMajorCntLin+1):
      # x positive major grid lines
      x = self.mGridOrigin[X] + xgrid * self.mGridMajorPixLinX
      id = self.mTripCanvas.create_line((x, y0, x, y1), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # x positive minor ticks
      self._xlintick(xgrid)

      # x negative major grid lines
      x = self.mGridOrigin[X] - xgrid * self.mGridMajorPixLinX
      id = self.mTripCanvas.create_line((x, y0, x, y1), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # x negative minor ticks
      self._xlintick(-xgrid)

    # x axis labels
    self._xlinlabels()

    # y grid 
    x0 = self.mGridMinX
    x1 = self.mGridMaxX
    for ygrid in range(1, self.mGridMajorCntLin+1):
      # y positive major grid lines
      y = self.mGridOrigin[Y] - ygrid * self.mGridMajorPixLinY
      id = self.mTripCanvas.create_line((x0, y, x1, y), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # y positive minor ticks
      self._ylintick(ygrid)

      # y negative major grid lines
      y = self.mGridOrigin[Y] + ygrid * self.mGridMajorPixLinY
      id = self.mTripCanvas.create_line((x0, y, x1, y), fill=gt.ColorBlack)
      self.mGidGrid += [id]

      # y negative minor ticks
      self._ylintick(-ygrid)

    # y axis labels
    self._ylinlabels()

  #--
  def _xyaxes(self):
    """ Draw x-y axes. """
    # x axis
    id = self.mTripCanvas.create_line(
                    (self.mGridMinX, self.mGridOrigin[Y],
                      self.mGridMaxX, self.mGridOrigin[Y]),
                    fill=gt.ColorPink1)
    self.mGidGrid += [id]

    # y axis
    id = self.mTripCanvas.create_line(
                    (self.mGridOrigin[X], self.mGridMinY,
                      self.mGridOrigin[X], self.mGridMaxY),
                      fill=gt.ColorPink1)
    self.mGidGrid += [id]

  #--
  def _xlogtick(self, xgrid):
    """ Draw log10 x axis minor ticks starting at major grid line. """
    # pixels / major grid line
    ppmajor = self.mGridMajorPixLogX

    # there is a minimum size before ticks are readable
    if ppmajor < 40:
      return

    # grid direction
    if xgrid > 0:
      sign = 1
    else:
      sign = -1

    # the tick marks
    ytick0 = self.mGridOrigin[Y] - 2
    ytick1 = self.mGridOrigin[Y] + 2
    xoff   = self.mGridOrigin[X] + sign * (abs(xgrid) - 1) * ppmajor
    for x in range(2,10):
      xtick = xoff + sign * _LogTenth(x, ppmajor)
      id = self.mTripCanvas.create_line((xtick, ytick0, xtick, ytick1),
                                         fill=gt.ColorBlack)
      self.mGidGrid += [id]

  #--
  def _ylogtick(self, ygrid):
    """ Draw log10 y axis minor ticks starting at major grid line. """
    # pixels / major grid line
    ppmajor = self.mGridMajorPixLogY

    # grid direction
    if ygrid > 0:
      sign = -1
    else:
      sign = 1

    # the tick marks
    xtick0 = self.mGridOrigin[X] - 2
    xtick1 = self.mGridOrigin[X] + 2
    yoff   = self.mGridOrigin[Y] + sign * (abs(ygrid) - 1) * ppmajor
    for y in range(2,10):
      ytick = yoff + sign * _LogTenth(y, ppmajor)
      id = self.mTripCanvas.create_line((xtick0, ytick, xtick1, ytick),
                                         fill=gt.ColorBlack)
      self.mGidGrid += [id]

  #--
  def _xloglabels(self):
    """ Label log10 x axis. """
    y = self.mGridMaxY + 2
    for xgrid in range(-self.mGridMajorCntLog, self.mGridMajorCntLog+1):
      x = self.mGridOrigin[X] + xgrid * self.mGridMajorPixLogX
      val = math.pow(10, math.fabs(xgrid)) * _LogOrig
      if val < 0.01:
        fmt = '%.3fm'
      elif val < 1.0:
        fmt = '%.2fm'
      else:
        fmt = '%.1fm'
      if xgrid < 0:
        val = -val
      text = fmt % val
      id = self.mTripCanvas.create_text((x, y), fill=gt.ColorBlack, text=text,
                                        anchor=tk.N)
      self.mGidGrid += [id]

  #--
  def _yloglabels(self):
    """ Label log10 y axis. """
    x = self.mGridMinX - 2
    for ygrid in range(-self.mGridMajorCntLog, self.mGridMajorCntLog+1):
      y = self.mGridOrigin[Y] - ygrid * self.mGridMajorPixLogY
      val = math.pow(10, math.fabs(ygrid)) * _LogOrig
      if val < 0.01:
        fmt = '%.3fm'
      elif val < 1.0:
        fmt = '%.2fm'
      else:
        fmt = '%.1fm'
      if ygrid < 0:
        val = -val
      text = fmt % val
      id = self.mTripCanvas.create_text((x, y), fill=gt.ColorBlack, text=text,
                                        anchor=tk.E)
      self.mGidGrid += [id]

  #--
  def _xlintick(self, xgrid):
    """ Draw linear x axis minor ticks starting at major grid line. """
    # pixels / major grid tick
    ppmajor = self.mGridMajorPixLinX

    # there is a minimum size before ticks are readable
    if ppmajor < 20:
      return
    
    # grid direction
    if xgrid > 0:
      sign = 1
    else:
      sign = -1

    # the tick marks
    ytick0 = self.mGridOrigin[Y] - 2
    ytick1 = self.mGridOrigin[Y] + 2
    xoff   = self.mGridOrigin[X] + sign * (abs(xgrid) - 1) * ppmajor
    for x in range(1,10):
      xtick = xoff + sign * _LinTenth(x, ppmajor)
      id = self.mTripCanvas.create_line((xtick, ytick0, xtick, ytick1),
                                         fill=gt.ColorBlack)
      self.mGidGrid += [id]

  #--
  def _ylintick(self, ygrid):
    """ Draw linear y axis minor ticks starting at major grid line. """
    # pixels / major grid tick
    ppmajor = self.mGridMajorPixLinY

    # there is a minimum size before ticks are readable
    if ppmajor < 20:
      return

    # grid direction
    if ygrid > 0:
      sign = -1
    else:
      sign = 1

    # the tick marks
    xtick0 = self.mGridOrigin[X] - 2
    xtick1 = self.mGridOrigin[X] + 2
    yoff   = self.mGridOrigin[Y] + sign * (abs(ygrid) - 1) * ppmajor
    for y in range(1,10):
      ytick = yoff + sign * _LinTenth(y, ppmajor)
      id = self.mTripCanvas.create_line((xtick0, ytick, xtick1, ytick),
                                         fill=gt.ColorBlack)
      self.mGidGrid += [id]

  #--
  def _xlinlabels(self):
    """ Label linear x axis. """
    fmt = '%.1fm'
    if self.mGridMajorPixLinX < _EdgeLeft - 10:
      modlabel = 2
    else:
      modlabel = 1
    y = self.mGridMaxY + 2
    for xgrid in range(-self.mGridMajorCntLin, self.mGridMajorCntLin+1):
      if xgrid % modlabel != 0:
        continue
      x = self.mGridOrigin[X] + xgrid * self.mGridMajorPixLinX
      text = fmt % xgrid
      id = self.mTripCanvas.create_text((x, y), fill=gt.ColorBlack, text=text,
                                        anchor=tk.N)
      self.mGidGrid += [id]

  #--
  def _ylinlabels(self):
    """ Label linear y axis. """
    fmt = '%.1fm'
    x = self.mGridMinX - 2
    for ygrid in range(-self.mGridMajorCntLin, self.mGridMajorCntLin+1):
      y = self.mGridOrigin[Y] + ygrid * self.mGridMajorPixLinY
      text = fmt % ygrid
      id = self.mTripCanvas.create_text((x, y), fill=gt.ColorBlack, text=text,
                                        anchor=tk.E)
      self.mGidGrid += [id]


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import time
  import Fusion.Core.Gluon as Gluon
  import Fusion.Utils.WinUT as WinUT
  import Fusion.Utils.IVTimer as IVTimer
  import Fusion.Khepera.Robots.vKhepera as vKhepera

  #--
  class WinUTKheTrip(WinUT.WinUT):
    """ GuiWinKheTrip Unit Test Window. """

    #--
    def __init__(self, robot):
      # the unit test
      ut = {
        'Null Test': self.utNone,
        'Line Test': self.utLine,
        'Square Test': self.utSquareStart,
        'Pirouette Test': self.utPirouetteStart,
        'Backdoor Test Pattern': self.utTestPatStart
      }

      self.mRobot = robot

      self.mIvtPush = IVTimer.IVTimer(1.50, 0.25, self.sut_push_iter, once=True)
      self.mIvtPush.start()

      WinUT.WinUT.__init__(self, title="GuiWinKheTrip Unit Test", ut=ut)

    #--
    def wut_cancel(self):
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

    #--
    # THE UNIT TEST
    #--

    #--
    def utNone(self):
      """ Nada """
      self.wut_showstatus('Just show the trip window.')
  
    #--
    def utLine(self):
      """ Line Unit Test"""
      self.wut_showstatus('Khepera will draw a straight line.')
      self.sut_start_khepera()
      self.mRobot.BellumSetGoals(speed=16, theta=0.0)
  
    def utSquareStart(self):
      """ Square Unit Test Start"""
      self.wut_showstatus('Khepera will draw a square ccw aproximately 240mm on a side.')
      self.sut_start_khepera()
      self.mIvt = IVTimer.IVTimer(0, 10.0, self.utSquareIter,
          theta = -math.radians(90.0), speed=25) 
      self.mIvt.start()
  
    def utSquareIter(self, ivt):
      """ Square Unit Test Iterator"""
      ivt.theta += math.radians(90.0)
      if ivt.theta > twopi:
        ivt.theta -= twopi
      ivt.speed = 25
      self.wut_showstatus("Heading %.1f degrees at %.1f mm/s" % \
          (math.degrees(ivt.theta), ivt.speed))
      self.mRobot.BellumSetGoals(speed=ivt.speed, theta=ivt.theta)
  
    def utPirouetteStart(self):
      """ Pirouette Unit Test Start"""
      self.wut_showstatus('Khepera will spin on its z-axis cw')
      self.sut_start_khepera()
      theta = math.radians(0.0)
      self.mIvt = IVTimer.IVTimer(0, 1.0, self.utPirouetteIter, theta=theta) 
      self.mIvt.start()
  
    def utPirouetteIter(self, ivt):
      """ Pirouette Unit Test Iterator. """
      ivt.theta -= twopi / 36.0 
      self.wut_showstatus("Step %.2f degrees" % math.degrees(ivt.theta))
      self.mRobot.BellumSetGoals(theta=ivt.theta)
  
    def utTestPatStart(self):
      """ Backdoor Test Pattern Unit Test Start. """
      self.wut_showstatus('Backdoor a test pattern without using the Khepera.')
      self.mIvt = IVTimer.IVTimer(1.0, 1.0, self.utTestPatIter, cnt=0) 
      self.mIvt.start()
      self.wut_showstatus('Showing trip in 1.0 second')
  
    def utTestPatIter(self, ivt):
      """ Backdoor Test Pattern Unit Test Iterator. """
      if ivt.cnt > 9:
        return
      ivt.cnt += 1
      if ivt.cnt == 1:
        self.mSut.mBotTrip.AddSubTrip([
          (0.0, 0.0, 0.0),
          (100.0, 0.0, 0.0),
          (100.0, 100.0, math.radians(90.0)),
          (-100.0, 100.0, math.radians(180.0)),
          (-100.0, -100.0, math.radians(270.0)),
          (200.0, -100.0, math.radians(0.0)),
          (200.0, 200.0, math.radians(90.0)),
          (-200.0, 200.0, math.radians(180.0)),
          (-200.0, -200.0, math.radians(270.0)),
          (900.0, -200.0, math.radians(0.0)),
          (900.0, 900.0, math.radians(90.0)),
          (-900.0, 900.0, math.radians(180.0)),
          (-900.0, -900.0, math.radians(270.0)),
          (1000.0, -900.0, math.radians(0.0)),
          (1000.0, 1000.0, math.radians(90.0)),
        ])
        self.wut_showstatus( 'Rotating trip 45 degrees in 2.0 seconds')
      elif ivt.cnt == 3:
        self.mSut.mBotTrip.rot(math.pi/4.0)
        self.wut_showstatus( 'Translating trip (-500.0, -500.0) in 2.0 seconds')
      elif ivt.cnt == 5:
        self.mSut.mBotTrip.trans(-500.0, -500.0)
        self.wut_showstatus( 'Rotating trip -45 degrees in 2.0 seconds')
      elif ivt.cnt == 7:
        self.mSut.mBotTrip.rot(-math.pi/4.0)
        self.wut_showstatus( 'Translating trip (500.0, 500.0) in 2.0 seconds')
      elif ivt.cnt == 9:
        self.mSut.mBotTrip.trans(500.0, 500.0)
        self.wut_showstatus( 'Done')
      self.mSut.TripCanvasRefresh()

  #--
  def main():
    """ GuiWinKheTrip Unit Test Main """
    robot = vKhepera.vKhepera()
    winUT = WinUTKheTrip(robot)
    winSut = GuiWinKheTrip(winUT.wut_this(),
                          sense_loc=robot.SenseOdometer,
                          sense_speed=robot.SenseSpeedometer,
                          reset_loc=robot.SenseCfgOdometer)
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
