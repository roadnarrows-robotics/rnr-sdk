################################################################################
#
# GuiWinHemiVizLinCam
#

""" Graphical User Interface Hemisson Linear Camera Visualizer Window

Graphical User Interface (GUI) Tkinter Linear Camera visualizer window.
The Linear Camera vizualizer displays the captured pixel gray-scale
values from the Linear Camera module attached to the vHemisson robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.09.12

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
import threading as thread
import time

import tkinter as tk
import tkinter.font

import Fusion.Core.Values as Values
import Fusion.Core.Gluon as Gluon

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiStatusBar as GuiStatusBar
import Fusion.Gui.GuiWin as GuiWin

import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam
import Fusion.Hemisson.Robots.HemiValues as HemiValues


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

_NumOfPixels        = HemiLinCam.LinCamNumPixels
_NumOfPixelRowsDft  =  8
_NumOfPixelRowsMax  = 64

# viz canvas dimensions
_EdgeLeft   =  2   # left edge margin
_EdgeTop    =  2   # top edge margin
_EdgeBottom =  2   # bottom edge margin
_EdgeRight  =  2   # right edge margin

# cursor's fixed width
_CursorWidth       = 10

# minimum size
_CanvasMinWidth   = _NumOfPixels * 6 + 1 + _EdgeLeft + _EdgeRight +\
                        _CursorWidth * 2
_CanvasMinHeight  = _NumOfPixelRowsMax * 6 + 1 + _EdgeTop + _EdgeBottom


#-------------------------------------------------------------------------------
# CLASS: GuiWinHemiVizLinCam
#-------------------------------------------------------------------------------
class GuiWinHemiVizLinCam(GuiWin.GuiWin):
  """ GUI Window vHemisson Linear Camera Sensor Visualizer Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the Window.

        Parameters:
          parent      - GUI parent of this window
          options     - Visualizer options. Options are:
            auto=<bool>       - do [not] automatically update
            num_rows=<n>      - number of rows to display
            sense_lincam=<cb> - sense linear camera callback to vRobot
            **winoptions      - GuiWin core options
    """
    # first initializations
    self.Init(**options)

    # create the window
    options[Values.FusionCWinKeyTitle] = 'vHemisson Linear Camers Visualizer'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, **options):
    """ First initializations.

        Parameters:
          options   - visualizer options.
        
        Return Value:
          None
    """
    # defaults
    self.mIsAutoMode          = True
    self.mNumRows             = _NumOfPixelRowsDft
    self.mCbRobotSenseLinCam  = None

    # set options from input parameters
    for key,val in options.items():
      if key == 'auto':
        if val:
          self.mIsAutoMode = True
        else:
          self.mIsAutoMode = False
      elif key == 'num_rows':
        try:
          self.mNumRows = int(val)
        except (ValueError, TypeError):
          self.mNumRows = _NumOfPixelRowsDft
      elif key == 'sense_lincam':
        self.mCbRobotSenseLinCam = val

    # override bad options with defaults
    if self.mNumRows <= 0:
      self.mNumRows = _NumOfPixelRowsDft
    elif self.mNumRows > _NumOfPixelRowsMax:
      self.mNumRows = _NumOfPixelRowsMax

    # locals
    self.mPixels = []
    row = 0
    while row < _NumOfPixelRowsMax:
      self.mPixels.append([0] * _NumOfPixels)
      row += 1
    self.mRowIndex  = 0     # next pixel fill row


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    # create the widgets
    self.GuiBody(self)

    # refresh the vizualization
    self.VizCanvasRefresh()

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
      self.VizCanvasClearAllPixels()
      self.VizCanvasClearGrid()
      width = geo[gut.W] - self.mWinBorder
      height = geo[gut.H] - self.mWinBorder - self.mCtlPanelFrameHeight
      self.mVizCanvas.configure(width=width, height=height)
      self.VizCanvasParams(width, height)
      self.mStatusBar.configure(width=width)
      self.mHasResized = True
    # resizing done, now redraw
    elif self.mHasResized:
      self.VizCanvasPaintGrid()
      self.VizCanvasPaintAllPixels()
      self.VizCanvasPaintCursor()
      self.mHasResized = False

  #--
  def destroy(self):
    """ Destroy window callback event. """
    GuiWin.GuiWin.destroy(self, auto=self.mIsAutoMode, num_rows=self.mNumRows)


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
          *kwargs   - keyword arguments for specific request

          Specific request and associated parameters:
          'cfg'     - configure LinCam window.
                        run_time=<str>  - module's run-time state
                        module=<str>    - module's detected state
                        exposure=<ms>   - exposure time (ms)
                        threshold=<val> - pixel threshold for 'Q'
                        grab={'P'|'Q'}  - pixel capture method with
                                            'P' = unthresholded
                                            'Q' = thresholded
          'pixels'  - show line of pixels values from Linear Camera
                        arg[0]=[pix,...]  - list of pixel values
                        force=<bool>      - force pixel update
          'num_rows' - set the number of display rows
                        arg[0]=<int>  - number of rows
          'clear'   - clear current set of pixels pings and redraw screen

        Return Values:
          None
 
    """
    #print("%s: WinUpdate: request: %s" % (self.mContextName, request))
    if request == 'cfg':
      items = {}
      for key,val in kwargs.items():
        if key in ['run_time', 'module', 'exposure', 'threshold', 'grab']:
          items[key] = val
      self.mStatusBar.Update(**items)
    elif request == 'pixels':
      if self.mIsAutoMode == True or kwargs.get('force', False):
        self.mPixels[self.mRowIndex] = args[0]
        self.VizCanvasPaintPixelRow(self.mRowIndex)
        self.mRowIndex = (self.mRowIndex +1) % self.mNumRows
        self.VizCanvasPaintCursor()
    elif request == 'num_rows':
      nRows = args[0]
      if nRows == self.mNumRows:
        return
      self.VizCanvasClearAllPixels()
      self.VizCanvasClearGrid()
      if nRows <= self.mRowIndex:
        self.mRowIndex = nRows - 1
      row = self.mRowIndex
      while row < self.mNumRows:
        self.mPixels[row] = [0] * _NumOfPixels
        row += 1
      self.mNumRows = nRows # set after clear
      self.VizCanvasParams(self.mVizCanvasWidth, self.mVizCanvasHeight)
      self.VizCanvasPaintGrid()
      self.VizCanvasPaintAllPixels()
      self.VizCanvasPaintCursor()
    elif request == 'clear':
      self.mPixels = [[0] * _NumOfPixels] * _NumOfPixelRowsMax
      self.mRowIndex = 0
      self.VizCanvasRefresh()
    else:
      print("%s: WinUpdate: unknown request: %s" % (self.mTitle, request))


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Viz Window Specifics
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

    # window border width and height
    self.mWinBorder = self.mWinGeo[gut.W] - int(self.mVizCanvas['width'])

    # set window's minimum size (250 obtained through experimentation)
    self.wm_minsize(
        width=_CanvasMinWidth+self.mWinBorder,
        height=_CanvasMinHeight+self.mWinBorder+self.mCtlPanelFrameHeight
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
    self.VizCanvasInit(parent, row, column)

    row += 1
    self.CtlPanelInit(parent, row, column)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Viz Canvas Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def VizCanvasInit(self, parent, row, column):
    """ Initialize Viz canvas.

        Parameters:
          parent  - parent to this canvas
          row     - grid row in parent
          column  - grid column in parent

        Return Value:
          None.
    """
    self.VizCanvasParams(_CanvasMinWidth, _CanvasMinHeight)

    self.mVizCanvas = tk.Canvas(parent,
                                width = self.mVizCanvasWidth,
                                height = self.mVizCanvasHeight,
                                bg = '#999999')
    self.mVizCanvas.grid(row=row, column=column)

    # canvas graphic id sets
    self.mGidGrid   = []  # pixel grid
    self.mGidPix    = []  # pixel blocks
    row = 0
    while row < _NumOfPixelRowsMax:
      self.mGidPix.append([None] * _NumOfPixels)
      row += 1
    self.mGidCursor = []  # insertion cursor

  #--
  def VizCanvasParams(self, width, height):
    """ Set canvas view parameters.

        Parameters:
          width   - width of canvas (pixels)
          height  - height of canvas (pixels)

        Return Value:
          None
    """
    # screen coordinates
    self.mVizCanvasWidth  = width
    self.mVizCanvasHeight = height
    self.mGridWidth       = self.mVizCanvasWidth - _EdgeLeft - _EdgeRight - \
                              _CursorWidth * 2
    self.mGridHeight      = self.mVizCanvasHeight - _EdgeTop - _EdgeBottom

    # pixel width and height
    self.mPixWidth        = (self.mGridWidth - _NumOfPixels - 1)/ _NumOfPixels
    self.mPixHeight       = (self.mGridHeight - self.mNumRows - 1) / \
                              self.mNumRows

    # re-adjust for roundoff errors
    self.mGridWidth       = (self.mPixWidth + 1) * _NumOfPixels + 1
    self.mGridHeight      = (self.mPixHeight + 1) * self.mNumRows + 1

    # minimums and maximums
    self.mGridMinX        = (self.mVizCanvasWidth - self.mGridWidth)/2
    self.mGridMinY        = (self.mVizCanvasHeight - self.mGridHeight)/2
    self.mGridMaxX        = self.mGridMinX + self.mGridWidth
    self.mGridMaxY        = self.mGridMinY + self.mGridHeight

  #--
  def VizCanvasPaintGrid(self):
    """ Paint the Viz canvas background grid. """
    row = 0
    x0 = self.mGridMinX
    x1 = self.mGridMaxX
    while row < self.mNumRows + 1:
      y = self.mGridMinY + row * (self.mPixHeight + 1)
      id = self.mVizCanvas.create_line((x0, y, x1, y), fill=gt.ColorBlue)
      self.mGidGrid += [id]
      row += 1

    n = 0
    y0 = self.mGridMinY
    y1 = self.mGridMaxY
    while n < _NumOfPixels + 1:
      x = self.mGridMinX + (self.mPixWidth + 1) * n
      id = self.mVizCanvas.create_line((x, y0, x, y1), fill=gt.ColorBlue)
      self.mGidGrid += [id]
      n += 1

  #--
  def VizCanvasClearGrid(self):
    """ Clear the Viz canvas background grid. """
    for id in self.mGidGrid:
      self.mVizCanvas.delete(id)
    self.mGidGrid = []

  #--
  def VizCanvasPaintAllPixels(self):
    """ Paint all rows of pixels values. """
    row = 0
    while row < self.mNumRows:
      self.VizCanvasPaintPixelRow(row)
      row += 1

  #--
  def VizCanvasClearAllPixels(self):
    """ Clear all rows of displayed pixels. """
    row = 0
    while row < self.mNumRows:
      self.VizCanvasClearPixelRow(row)
      row += 1

  #--
  def VizCanvasPaintPixelRow(self, row):
    """ Paint a row of pixel gray-scale values """
    m = len(self.mPixels[row])
    n = 0
    while n < _NumOfPixels and n < m:
      pixel = abs(self.mPixels[row][n])
      if pixel > 255: pixel = 255
      rect = self._pixel2canvas(row, n)
      pixColor = "#%02x%02x%02x" % (pixel, pixel, pixel)
      id = self.mGidPix[row][n]
      if id is not None: # replace
        self.mVizCanvas.itemconfigure(id, fill=pixColor, outline=pixColor)
      else:       # create
        id = self.mVizCanvas.create_rectangle(rect,
              fill=pixColor, outline=pixColor)
        self.mGidPix[row][n] = id
      n += 1

  #--
  def VizCanvasClearPixelRow(self, row):
    """ Clear a row of displayed pixels. """
    for id in self.mGidPix[row]:
      if id >= 0:
        self.mVizCanvas.delete(id)
    self.mGidPix[row] = [None] * _NumOfPixels

  #--
  def VizCanvasPaintCursor(self):
    """ Mark next to-be-filled pixel row. """
    for id in self.mGidCursor:
      self.mVizCanvas.delete(id)
    self.mGidCursor = []
    y = self.mGridMinY + self.mRowIndex * (self.mPixHeight + 1)
    x = self.mGridMinX - _CursorWidth
    id = self.mVizCanvas.create_polygon(
        x, y,
        self.mGridMinX-1, y + self.mPixHeight/2,
        x, y + self.mPixHeight + 1,
        fill=gt.ColorPink1, outline=gt.ColorGold1)
    self.mGidCursor += [id]
    x = self.mGridMaxX + _CursorWidth
    id = self.mVizCanvas.create_polygon(
        x, y,
        self.mGridMaxX+1, y + self.mPixHeight/2,
        x, y + self.mPixHeight + 1,
        fill=gt.ColorPink1, outline=gt.ColorPink1)
    self.mGidCursor += [id]

  #--
  def VizCanvasRefresh(self):
    """ Refresh the Viz canvas. """
    self.VizCanvasClearAllPixels()
    self.VizCanvasClearGrid()
    self.VizCanvasPaintGrid()
    self.VizCanvasPaintAllPixels()
    self.VizCanvasPaintCursor()

  #--
  def _pixel2canvas(self, row, n):
    """ Convert pixel number to canvas rectangle coordinates.

        Parameters:
          row - pixel row number
          n   - pixel number

        Return Value:
          x0, y0, x1, y1
    """
    x0 = self.mGridMinX + (self.mPixWidth + 1) * n + 1
    y0 = self.mGridMinY + (self.mPixHeight + 1) * row + 1
    x1 = x0 + self.mPixWidth - 1
    y1 = y0 + self.mPixHeight - 1
    return x0, y0, x1, y1


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
    w = tk.Label(cpframe, text='Linear Camera Control Panel', fg=gt.ColorGreen1)
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
    for n in [0, 0, 1, 2]:
      filename = gut.GetFusionImageFileName('FusionCamera%d.gif' % n)
      if filename:
        autoFiles += [filename]
    manFile = gut.GetFusionImageFileName('FusionCameraMan.gif')
    w = gut.ActiveImageWidget(subframe, 
        activesets={'auto':autoFiles, 'man':[manFile]}, period=0.25)
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mWidgetActiveImage = w

    subcol += 1

    # capture button
    w = tk.Button(subframe, text='Capture', fg=gt.ColorBlack, width=8,
        command=self.CbCapture)
    GuiToolTip.GuiToolTip(w, text="Force Linear Camera line capture")
    self.mButtonCapture = w

    # automatic/manual updates button
    w = tk.Button(subframe, width=8, command=self.CbAutoMan)
    self.mTtAutoManManText  = "Go to Manual Visualization Updates"
    self.mTtAutoManAutoText = "Go to Automatic Visualization Updates"
    w.grid(row=subrow, column=subcol, sticky=tk.W)
    self.mTtAutoMan = GuiToolTip.GuiToolTip(w, 'no tip')
    self.mButtonAutoMan = w
    self.CtlPanelCfgAutoMan()

    subcol += 1

    # draw capture button
    self.mButtonCapture.grid(row=subrow, column=subcol, sticky=tk.W)

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

    # range slider
    w = tk.Scale(subframe, width=10, length=120, from_=1, to=_NumOfPixelRowsMax,
        orient=tk.HORIZONTAL, label='Number of Rows')
    w.grid(row=subrow, column=subcol,
        sticky=tk.W, pady=1)
    GuiToolTip.GuiToolTip(w, text="Number of pixels rows (history) to show.")
    w.set(self.mNumRows)
    self.mSliderNumRows = w
    w.bind("<ButtonRelease-1>", self.CbNumRows)

    subcol += 1

    # close button
    w = tk.Button(subframe, text='Close', fg=gt.ColorBlack, width=8,
        activeforeground=gt.ColorBttnStop, command=self.CbClose)
    w.grid(row=subrow, column=subcol, sticky=tk.E, padx=5)
    GuiToolTip.GuiToolTip(w, text="Close this window.")

    row += 1
    column = 0

    # status bar width
    self.update_idletasks()
    sbwidth = gut.geometry(cpframe)[gut.W] - 4

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
          {'tag': 'exposure',
           'max_width': 3,
           'val': HemiLinCam.LinCamExposureTimeDft,
           'tooltip': "Exposure time in milliseconds [%d, %d]" % \
           (HemiLinCam.LinCamExposureTimeMin, HemiLinCam.LinCamExposureTimeMax) 
          },
          {'tag':'threshold',
            'max_width': 4,
            'val': HemiLinCam.LinCamThresholdDft,
            'fmt': '0x%02X',
            'tooltip': "'Q' grab pixel threshold [0x%02x, 0x%02x]" % \
              (HemiLinCam.LinCamThresholdMin, HemiLinCam.LinCamThresholdMax) 
          },
          {'tag':'grab',
            'max_width': 1,
            'val': 'P',
            'tooltip': "Frame grab method\n"
                       " 'P' - unthresholded\n"
                       " 'Q' - thresholded",
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
      self.mButtonCapture['state'] = tk.DISABLED
      self.mWidgetActiveImage.SetActive('auto')
    else:
      w['text']             = 'Auto'
      w['fg']               = gt.ColorBttnGo
      w['activeforeground'] = gt.ColorBttnGo
      self.mTtAutoMan.newtip(self.mTtAutoManAutoText)
      self.mButtonCapture['state'] = tk.NORMAL
      self.mWidgetActiveImage.SetActive('man')


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CbAutoMan(self):
    """ Automatic/Manual updates toggle callback. """
    if self.mIsAutoMode == True:
      self.mIsAutoMode  = False
    else:
      self.mIsAutoMode  = True
    self.CtlPanelCfgAutoMan()

  #--
  def CbCapture(self):
    """ Force Linear Camera capture """
    if self.mCbRobotSenseLinCam:
      self.WinQueueRequest('pixels', self.mCbRobotSenseLinCam(), force=True)

  #--
  def CbVizClear(self):
    """ Clear the viz's current set of data callback. """
    self.WinQueueRequest('clear')

  #--
  def CbSnapShot(self):
    """ Take a snap shot image callback. """
    pass

  #--
  def CbNumRows(self, event):
    """ Number of pixel rows slider callback. """
    self.WinQueueRequest('num_rows',self.mSliderNumRows.get())

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
  import math
  import Fusion.Utils.WinUT as WinUT
  import Fusion.Utils.IVTimer as IVTimer

  #--
  class WinUTVizLinCam(WinUT.WinUT):
    """ GuiWinHemiVizLinCam Unit Test Window """

    #--
    def __init__(self):
      # the unit test
      ut = {
        'Null Test': self.utNullStart,
        'Random Pixels Test': self.utRandPixelsStart,
        'Moving Blob Test': self.utBlobStart,
        'Full Pixel Scale Test': self.utScalePixelsStart
      }

      WinUT.WinUT.__init__(self, title="GuiWinHemiVizLinCam Unit Test", ut=ut)

    #--
    # THE UNIT TEST
    #--

    #--
    def utNullStart(self):
      """ Null UT Start """
      self.wut_showstatus("No automatic push of data.")

    #--
    def utRandPixelsStart(self):
      """ Random Pixels UT Start """
      self.wut_showstatus("Push random pixels values.")
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utRandPixelsIter, cnt=0) 
      self.mIvt.start()

    #--
    def utRandPixelsIter(self, ivt):
      """ Random Pixels Iterator """
      if ivt.cnt == 0:
        self.mSut.WinQueueRequest('cfg',
                        run_time='enabled',
                        module='detected',
                        exposure=0,
                        threshold=129,
                        grab='P')
        ivt.cnt = 1
      else:
        n = 0
        pixList = []
        while n < _NumOfPixels:
          pixList += [random.randint(0,255)]
          n += 1
        self.mSut.WinQueueRequest('pixels', pixList)

    #--
    def utScalePixelsStart(self):
      """ Full Scale Pixels UT Start """
      self.wut_showstatus("Push black to white full scale pixels values.")
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utScalePixelsIter, cnt=0) 
      self.mIvt.start()

    #--
    def utScalePixelsIter(self, ivt):
      """ Full Scale Pixels Iterator """
      if ivt.cnt == 0:
        self.mSut.WinQueueRequest('cfg',
                        run_time='enabled',
                        module='detected',
                        exposure=5,
                        threshold=0xff,
                        grab='P')
        ivt.cnt = 1
      else:
        self.mSut.WinQueueRequest('pixels', PhakeLinCam())

    #--
    def utBlobStart(self):
      """ Pixel Blob UT Start """
      self.wut_showstatus("Push pixel blob left to right.")
      self.mSut.WinQueueRequest('cfg', run_time='enabled', module='detected')
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utBlobIter, pixelstart=0) 
      self.mIvt.start()

    #--
    def utBlobIter(self, ivt):
      """ Pixel Blob UT Start """
      pixelend = ivt.pixelstart + 4
      if pixelend > 101:
        pixelend = 101
      n = 0
      pixList = []
      while n < _NumOfPixels:
        if n >= ivt.pixelstart and n <= pixelend:
          pixList += [random.randint(180,255)]
        else:
          pixList += [0]
        n += 1
      self.mSut.WinQueueRequest('pixels', pixList)
      ivt.pixelstart += 1
      if ivt.pixelstart > 101:
        ivt.pixelstart = 0


  #--
  def PhakeLinCam():
    """ Fake Hemisson linear camera reader. """
    step = 256.0/101.0
    n = 0
    pixList = []
    while n < _NumOfPixels:
      pix = math.floor(step * n)
      if pix > 255:
        pix = 255
      pixList += [pix]
      n += 1
    return pixList

  #--
  def main():
    """ GuiWinHemiVizLinCam Unit Test Main """
    winUT = WinUTVizLinCam()
    winSut = GuiWinHemiVizLinCam(winUT.wut_this(), sense_lincam=PhakeLinCam)
    winUT.wut_mark_sut(winSut)
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  main()
