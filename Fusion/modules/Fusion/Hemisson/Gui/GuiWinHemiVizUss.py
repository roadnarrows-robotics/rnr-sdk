################################################################################
#
# GuiWinHemiVizUss.py
#

""" Graphical User Interface Hemisson USS Visualizer Window Module

Graphical User Interface (GUI) Tkinter UltraSonic Sensor visualizer window.
The USS visualizer shows the echoes return of the emitted ultra-sonic pusles
emitted by the UtraSonic Sensor module attached to the vHemisson robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.09.04

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

import Tkinter as tk

import Fusion.Core.Values as Values

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiWin as GuiWin

import Fusion.Hemisson.Cmd.HemiCmdUss as HemiUss
import Fusion.Hemisson.Robots.HemiValues as HemiValues

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Look and Feel
#

# minimum size
_CanvasMinWidth   = 400
_CanvasMinHeight  = 300


#-------------------------------------------------------------------------------
# CLASS: GuiWinHemiUss
#-------------------------------------------------------------------------------
class GuiWinHemiVizUss(GuiWin.GuiWin):
  """ GUI Window vHemisson UltraSonic Sensor Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Visualizer options. Options are:
            auto=<bool>     - do [not] automatically update
            sense_uss=<cb>  - sense USS callback to vRobot
            **winoptions    - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = \
        'vHemisson UltraSonic Sensor Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          **options   - visualizer options.

        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode      = True
    self.mCbRobotSenseUss = None

    # set options from input parameters
    for key,val in options.iteritems():
      if key == 'auto':
        if val:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False
      elif key == 'sense_lincam':
        self.mCbRobotSenseUss = val

    # override bad options with defaults

    # locals

    # the ping blip sets
    self.mBlipSets = {
      'prev': {'blips':[], 'gids':[]},  # previous blip data and graphic ids
      'cur':  {'blips':[], 'gids':[]},  # current blip data and graphic ids
      'colors': [                       # current blip colors from closest out
        'red', 'orange', 'yellow', 'yellowgreen', 
        'green', gt.ColorGreenBlue, 'blue', "#4400cc",
        "#4400aa", "#440088", "#440066", "#330033",
        "#330033", "#330033", "#330033", "#330033",
        "#330033"]
      }


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create the widgets
    self.GuiBody(self)

    # refresh the navigator canvas
    self.UssCanvasRefresh()

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
    if  geo[gut.W] != self.mWinGeo[gut.W] or \
        geo[gut.H] != self.mWinGeo[gut.H]:
      self.mWinGeo = geo
      self.UssCanvasClearGrid()
      self.UssCanvasClearAllBlips()
      self.mUssCanvasWidth = self.mWinGeo[gut.W] - self.mWinBorder
      self.mUssCanvasHeight = self.mWinGeo[gut.H] - self.mWinBorder \
                              - self.mCtlPanelFrameHeight
      self.mUssCanvasOrigin = self.mUssCanvasWidth/2, self.mUssCanvasHeight - 20
      self.mUssCanvasRadius = self.mUssCanvasHeight - 50
      self.mUssCanvas.configure(width=self.mUssCanvasWidth,
                                height=self.mUssCanvasHeight)
      self.mStatusBar.configure(width=self.mUssCanvasWidth)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.UssCanvasPaintGrid()
      self.UssCanvasDrawAllBlips()
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
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
          'cfg'   - configure USS window.
                      run_time=<str>  - module's run-time state
                      module=<str>    - module's detected state
                      range=<mm>      - max range in mm
          'pings' - show new set of echo pings from USS
                      args[0]=[blip,...]  - list of echoes
                      force=<bool>        - force ping update
          'clear' - clear current set of echo pings and redraw screen

        Return Values:
          None
    """
    #print "Dbg: %s: WinUpdate: request: %s" % (self.mContextName, request)
    if request == 'cfg':
      items = {}
      for key,val in kwargs.iteritems():
        if key == 'range':
          self.mUssMaxRangeMm = val
          items[key] = val/1000.0
        elif key in ['run_time', 'module']:
          items[key] = val
      self.UssCanvasRefresh()
      self.mStatusBar.Update(**items)
    elif request == 'pings':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.mBlipSets['prev']['blips'] = self.mBlipSets['cur']['blips']
        self.mBlipSets['cur']['blips'] = args[0]
        self.UssCanvasClearAllBlips()
        self.UssCanvasDrawAllBlips()
    elif request == 'clear':
      self.mBlipSets['prev']['blips'] = self.mBlipSets['cur']['blips']
      self.mBlipSets['cur']['blips'] = []
      self.UssCanvasRefresh()
    else:
      print "%s: WinUpdate: unknown request: %s" % (self.mTitle, request)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # UltraSonic Sensor Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # current window dimensions
    self.mWinGeo = gut.geometry(self)

    # current control panel frame dimensions
    cangeo = gut.geometry(self.mUssCanvas)

    # current control panel frame dimensions
    cpgeo = gut.geometry(self.mCtlPanelFrame)

    # control panel frame height is fixed
    self.mCtlPanelFrameHeight = cpgeo[gut.H]

    if cpgeo[gut.W] > cangeo[gut.W]:
      maxwidth = cpgeo[gut.W]
    else:
      maxwidth = cangeo[gut.W]
      
    # window border width and height
    self.mWinBorder = self.mWinGeo[gut.W] - maxwidth

    # set window's minimum size
    self.wm_minsize(width=self.mWinGeo[gut.W], height=self.mWinGeo[gut.H])

  #--
  def GuiBody(self, parent):
    """ Create the window body.

        Paremeters:
          parent  - parent to this body
          
        Return Value:
          None
    """
    row = 0
    column = 0
    self.UssCanvasInit(parent, row, column)

    row += 1
    self.CtlPanelInit(parent, row, column)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # USS Ping Canvas Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def UssCanvasInit(self, parent, row, column):
    """ Initialize UltraSonic Sensor canvas.

        Parameters:
          parent  - parent to this canvas
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None.
    """
    self.mUssCanvasWidth      = _CanvasMinWidth
    self.mUssCanvasHeight     = _CanvasMinHeight

    self.mUssCanvas = tk.Canvas(parent,
                                width = self.mUssCanvasWidth,
                                height = self.mUssCanvasHeight,
                                bg = 'black')
    self.mUssCanvas.grid(row=row, column=column)

    self.mUssCanvasOrigin = self.mUssCanvasWidth/2, self.mUssCanvasHeight - 20
    self.mUssCanvasRadius = self.mUssCanvasHeight - 50
    self.mUssMaxRangeMm   = HemiUss.UssRangeDftMm
    self.mGidGrid         = []

  #--
  def UssCanvasPaintGrid(self):
    """ Paint the UltraSonic Sensor canvas background grid. """
    # origin
    x_orig, y_orig = self.mUssCanvasOrigin

   # x axis
    x0 = x_orig
    y0 = y_orig
    x1 = x0
    y1 = 10
    dim = x0, y0, x1, y1
    id = self.mUssCanvas.create_line(dim, fill='white', arrow=tk.LAST)
    self.mGidGrid += [id]

    # x axis label
    dim = x0+10, y1+1
    id = self.mUssCanvas.create_text(dim, fill='white', text='x', anchor=tk.NW)
    self.mGidGrid += [id]

    # y axis
    x0 = self.mUssCanvasWidth - 10
    y0 = y_orig
    x1 = 20
    y1 = y0
    dim = x0, y0, x1, y1
    id = self.mUssCanvas.create_line(dim, fill='white', arrow=tk.LAST)
    self.mGidGrid += [id]

    # y axis label
    dim = x1+2, y0-10
    id = self.mUssCanvas.create_text(dim, fill='white', text='y', anchor=tk.SW)
    self.mGidGrid += [id]

    radius = self.mUssCanvasRadius

    # tick marks and labels
    tick = 1
    numticks = math.ceil(self.mUssMaxRangeMm/1000.0)
    delta_r = radius / numticks
    deg_start = 90.0 - 27.5
    deg_extent = 55.0
    rad_start = math.radians(deg_start)
    while tick <= numticks:
      # tick mark
      r = tick * delta_r
      dim = x_orig-r, y_orig-r, x_orig+r, y_orig+r
      id = self.mUssCanvas.create_arc(dim, start=deg_start, extent=deg_extent,
                                  outline='white')
      self.mGidGrid += [id]
      # angle callout
      if tick == 1:
        # callout right interior arrow
        x0 = x_orig + int(r * math.cos(rad_start))
        y0 = y_orig - int(r * math.sin(rad_start))
        x1 = x0 - 4 * math.sin(rad_start)
        y1 = y0 - 4 * math.cos(rad_start)
        id = self.mUssCanvas.create_line(x0, y0, x1, y1, fill='white',
            arrow=tk.FIRST)
        self.mGidGrid += [id]
        # callout left interior arrow
        x0 = x_orig - int(r * math.cos(rad_start))
        y0 = y_orig - int(r * math.sin(rad_start))
        x1 = x0 + 4 * math.sin(rad_start)
        y1 = y0 - 4 * math.cos(rad_start)
        id = self.mUssCanvas.create_line(x0, y0, x1, y1, fill='white',
            arrow=tk.FIRST)
        self.mGidGrid += [id]
        # callout connector and text
        x0 = x_orig + int(r * math.cos(rad_start)) + 2
        y0 = y_orig - int(r * math.sin(rad_start)) + 2
        x1 = x0 + 5
        y1 = y0 + 10
        id = self.mUssCanvas.create_line(x0, y0, x1, y1, fill='white')
        self.mGidGrid += [id]
        id = self.mUssCanvas.create_text(x1, y1, fill='white', text='55 deg',
            anchor=tk.NW)
        self.mGidGrid += [id]

      # tick mark text
      x0 = x_orig + int(r * math.cos(rad_start)) + 5
      y0 = y_orig - int(r * math.sin(rad_start))
      dim = x0, y0
      id = self.mUssCanvas.create_text(dim, fill='white', text='%dm' % tick, 
          anchor=tk.W)
      self.mGidGrid += [id]

      tick += 1

    # Hemisson
    x0 = x_orig - 7
    y0 = y_orig - 5
    x1 = x_orig + 7
    y1 = y_orig + 15
    dim = x0, y0, x1, y1
    id = self.mUssCanvas.create_oval(dim, fill='blue', outline='yellow')
    self.mGidGrid += [id]
    dim = x1+3, y_orig+3
    id = self.mUssCanvas.create_text(dim, fill='white', text='Hemisson',
          anchor=tk.NW)
    self.mGidGrid += [id]

  #--
  def UssCanvasClearGrid(self):
    """ Clear the grid from the UltraSonic Sensor canvas. """
    for id in self.mGidGrid:
      self.mUssCanvas.delete(id)
    self.mGidGrid = []

  #--
  def UssCanvasClearAllBlips(self):
    """ Clear all blips from the USS Canvas.  """
    for id in self.mBlipSets['cur']['gids']:
      self.mUssCanvas.delete(id)
    self.mBlipSets['cur']['gids'] = []
    for id in self.mBlipSets['prev']['gids']:
      self.mUssCanvas.delete(id)
    self.mBlipSets['prev']['gids'] = []

  #--
  def UssCanvasDrawAllBlips(self):
    """ Draw all current blips on the USS Canvas.  """
    # reverse draw for correct colors
    n = len(self.mBlipSets['cur']['blips']) - 1
    if n > HemiUss.UssEchoMax:
      n = HemiUss.UssEchoMax
    while n >= 0:
      val = self.mBlipSets['cur']['blips'][n]
      if val > 0.0 and val <= self.mUssMaxRangeMm:
        self.UssCanvasDrawBlip(n, val)
      n -= 1

  #--
  def UssCanvasDrawBlip(self, n, val):
    """ Draw all single blip on the USS Canvas.  """
    x_orig, y_orig = self.mUssCanvasOrigin
    color = self.mBlipSets['colors'][n]
    r = self.mUssCanvasRadius *  val / self.mUssMaxRangeMm
    deg_start = 90.0 - 27.5
    deg_extent = 55.0
    rad_start = math.radians(deg_start)
    dim = x_orig-r, y_orig-r, x_orig+r, y_orig+r
    id = self.mUssCanvas.create_arc(dim, start=deg_start, extent=deg_extent, 
                                 style='arc', outline=color, width=3)
    self.mBlipSets['cur']['gids'] += [id]
    x0 = x_orig - int(r * math.cos(rad_start)) - 5
    y0 = y_orig - int(r * math.sin(rad_start))
    dim = x0, y0
    id = self.mUssCanvas.create_text(dim, fill=color,
          text='%6.3f' % (float(val) / 1000.0),
          anchor=tk.E)
    self.mBlipSets['cur']['gids'] += [id]

  #--
  def UssCanvasRefresh(self):
    """ Refresh the UltraSonic Sensor canvas. """
    self.UssCanvasClearGrid()
    self.UssCanvasClearAllBlips()
    self.UssCanvasPaintGrid()
    self.UssCanvasDrawAllBlips()


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
    # control panel frame
    cpframe = tk.Frame(parent, relief=tk.FLAT, borderwidth=0)
    cpframe.grid(row=row, column=column, padx=3, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe

    row = 0
    column = 0

    # control panel title
    w = tk.Label(cpframe, text='USS Control Panel', fg=gt.ColorGreen1)
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
    for n in [0, 0, 1, 2, 3]:
      filename = gut.GetFusionImageFileName('FusionBat%d.gif' % n)
      if filename:
        autoFiles += [filename]
    manFile = gut.GetFusionImageFileName('FusionBatMan.gif')
    w = gut.ActiveImageWidget(subframe, 
        activesets={'auto':autoFiles, 'man':[manFile]}, period=0.25)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mWidgetActiveImage = w

    # ping button
    w = tk.Button(subframe, text='Ping', fg=gt.ColorBlack, width=8,
        command=self.CbPing)
    GuiToolTip.GuiToolTip(w, text="Force UltraSonic Sensor ping")
    self.mButtonPing = w

    subcol += 1

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Go to Manual Visualization Updates"
    self.mTtAutoManAutoText = "Go to Automatic Visualization Updates"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtAutoMan = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonAutoMan = w
    self.CtlPanelCfgAutoMan()

    subcol += 1

    # draw ping button
    self.mButtonPing.grid(row=subrow, column=subcol, sticky=tk.W)

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

    # close button
    w = tk.Button(subframe, text='Close', fg=gt.ColorBlack, width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, padx=1, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0

    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(subframe)[gut.W]

    self.mStatusBar = GuiStatusBar.GuiStatusBar(cpframe,
        [ 
          {'tag': 'run_time',
           'prefix': 'run-time:',
           'max_width': 8,
           'val': 'unknown',
           'tooltip': "Module's run-time state.\nSee Robot|Robot Options..."
          },
          {'tag': 'module',
           'max_width': 12,
           'val': 'unknown',
           'tooltip': "Module's detected presence on the Hemisson"
          },
          {'tag': 'range',
           'max_width': 6,
           'val': HemiUss.UssRangeDftMm/1000.0,
           'fmt': '%6.3f',
           'tooltip': "Maximum useful range in meters [%6.3f, %6.3f]" % \
           (HemiUss.UssRangeMinMm/1000.0, HemiUss.UssRangeMaxMm/1000.0) 
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
      w['text']             = 'Manual'
      w['fg']               = gt.ColorBttnStop
      w['activeforeground'] = gt.ColorBttnStop
      self.mTtAutoMan.newtip(self.mTtAutoManManText)
      self.mButtonPing['state'] = tk.DISABLED
      self.mWidgetActiveImage.SetActive('auto')
    else:
      w['text']             = 'Auto'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtAutoMan.newtip(self.mTtAutoManAutoText)
      self.mButtonPing['state'] = tk.NORMAL
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
  def CbPing(self):
    """ Force USS ping """
    if self.mCbRobotSenseUss:
      self.WinQueueRequest('pings', self.mCbRobotSenseUss(), force=True)

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
  class WinUTVizUss(WinUT.WinUT):
    """ GuiWinHemiVizUss Unit Test Window """
    #--
    def __init__(self):
      # the unit test
      ut = {
        'Null Test': self.utNullStart,
        'Calibrate Pings Test': self.utCalibStart,
        'Random Pings Test': self.utRandStart
      }

      WinUT.WinUT.__init__(self, title="GuiWinHemiVizUss Unit Test", ut=ut)

    #--
    # THE UNIT TEST
    #--

    #--
    def utNullStart(self):
      """ Null UT Start """
      self.wut_showstatus("No automatic push of data.")

    #--
    def utRandStart(self):
      """ Random Pixels UT Start """
      self.wut_showstatus("Push random ping values up to 6 meters.")
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utRandIter, cnt=0) 
      self.mIvt.start()

    #--
    def utRandIter(self, ivt):
      """ Random Pixels Iterator """
      if ivt.cnt == 0:
        self.mSut.WinQueueRequest('cfg',
                        run_time='enabled',
                        module='detected',
                        range=6000)
        ivt.cnt = 1
      else:
        blips = []
        dist = 0.0
        n = 0
        while n < 17:
          delta = random.random() * (self.mSut.mUssMaxRangeMm - dist)
          if delta < 30.0:
            break
          dist += delta
          blips += [dist]
          n += 1
        self.mSut.WinQueueRequest('pings', blips)

    #--
    def utCalibStart(self):
      """ Calibrate Pings UT Start """
      self.wut_showstatus("Calibrate pings.")
      self.mSut.WinQueueRequest('cfg', run_time='enabled', module='detected')
      self.mIvt = IVTimer.IVTimer(2.0, 2.0, self.utCalibIter, calset=0)
      self.mIvt.start()

    #--
    def utCalibIter(self, ivt):
      """ Calibrate Pings Iterator """
      if not self.mSut.mIsAutoMode: # don't interfere with manual testing
        return
      if ivt.calset == 0:
        self.wut_showstatus("Calibrate pings at 0.5 meter intervals to 12m.")
        self.mSut.WinQueueRequest('cfg', range=12000)
        self.mSut.WinQueueRequest('pings',
          [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 
          5000, 5500, 6000, 6500, 7000, 7500, 8000, 11000, 12000])
      elif ivt.calset == 1:
        self.wut_showstatus("Calibrate pings at 0.5 meter intervals to 4m.")
        self.mSut.WinQueueRequest('cfg', range=4000)
        self.mSut.WinQueueRequest('pings',
          [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 
          5000, 5500, 6000, 6500, 7000, 7500, 8000, 11000])
      elif ivt.calset == 2:
        self.wut_showstatus("Calibrate pings at limit of ping resolution "
                            " of 0.043ms to 12m")
        self.mSut.WinQueueRequest('cfg', range=12000)
        blips = []
        dist = 43
        n = 0
        while n < 17:
          blips += [dist]
          dist += 43
          n += 1
        self.mSut.WinQueueRequest('pings', blips)
      elif ivt.calset == 3:
        self.wut_showstatus("Calibrate pings at limit of ping resolution "
                            " of 0.043ms to 2m")
        self.mSut.WinQueueRequest('cfg', range=2000)
        blips = []
        dist = 43
        n = 0
        while n < 17:
          blips += [dist]
          dist += 43
          n += 1
        self.mSut.WinQueueRequest('pings', blips)
      ivt.calset = (ivt.calset + 1) % 4


  def PhakeUss():
    """ Fake Hemisson USS reader. """
    return [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000,
            9000, 10000, 11000, 12000]

  #--
  def main():
    """ GuiWinHemiUss Unit Test Main """
    winUT = WinUTVizUss()
    winSut = GuiWinHemiVizUss(winUT.wut_this(), sense_lincam=PhakeUss)
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
