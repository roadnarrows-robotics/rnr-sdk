################################################################################
#
# GuiWinOpticFlowViz
#

""" Graphical User Interface Hemisson Optic Flow Visualizatin Window

Graphical User Interface (GUI) Tkinter visualization window displays
the optic flow state graphically.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.08

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

import Tkinter as tk
import tkFont

import Fusion.Core.Values as Values
import Fusion.Core.Gluon as Gluon

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiWin as GuiWin
import Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs

import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam
import Fusion.Hemisson.Robots.HemiValues as HemiValues


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

_NumPixels = HemiLinCam.LinCamNumPixels

# viz canvas dimensions
_EdgeLeft   = 5    # left edge margin
_EdgeTop    = 5    # top edge margin
_EdgeBottom = 5    # bottom edge margin
_EdgeRight  = 5    # right edge margin

_BlobHeightRatio      = 0.20  # ratio of blob height to grid height

# minimum size
_CanvasMinWidth   = _NumPixels * 6 + 1 + _EdgeLeft + _EdgeRight
_CanvasMinHeight  = 10 + 2 + 4 + _EdgeTop + _EdgeBottom


#-------------------------------------------------------------------------------
# CLASS: GuiWinOpticFlowViz
#-------------------------------------------------------------------------------
class GuiWinOpticFlowViz(GuiWin.GuiWin):
  """ GUI Window Optic Flow Visualization Class """

  #--
  def __init__(self, parent, vBrain, **options):
    """ Initialize the Window.

        Parameters:
          parent      - GUI parent of this window
          vBrain      - vBrain object supporting optic flow
          options     - Navigator options. Options are:
            **winoptions  - GuiWin core options
    """
    # first initializations
    self.Init(vBrain, **options)

    # create the window
    options[Values.FusionCWinKeyTitle] = 'Optic Flow Visualization'
    GuiWin.GuiWin.__init__(self, parent, **options)

  #--
  def Init(self, vBrain, **options):
    """ First initializations.

        Parameters:
          options   - visualization input options.
    """
    # defaults

    # set options from input parameters
    for k,v in options.iteritems():
      pass

    # override bad options with defaults

    # adjust range to current scale

    # locals
    self.mBrain   = vBrain            # the vBrain
    self.mPixels  = [0] * _NumPixels  # the pixels
    self.mBlobs   = {}                # blobs
    self.mBoiId    = ''               # Blob of Interest id

    # mutex
    self.mVizSema = thread.Semaphore()


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
    if geo[0] != self.mWinGeo[0] or geo[1] != self.mWinGeo[1]:
      width = geo[0] - self.mWinBorder
      height = geo[1] - self.mWinBorder - self.mCtlPanelFrameHeight
      self.mVizSema.acquire()
      self.mVizCanvas.configure(width=width, height=height)
      self.VizCanvasParams(width, height)
      self.VizCanvasRefresh()
      self.mVizSema.release()
      self.mWinGeo = geo


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Window Update
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def WinUpdate(self, what):
    """ Update visualization from the current brain state. """
    if self.mFreeze:
      return
    self.mVizSema.acquire()
    if what == 'raw_pixels' or what == 'all':
      rawPixels = self.mBrain.mBrum['raw_pixels']
      self.mPixels = rawPixels
      self.VizCanvasPaintPixels()
    if what == 'cooked_pixels' or what == 'all':
      cookedPixels = self.mBrain.mBrum['cooked_pixels']
      self.mPixels = cookedPixels
      self.VizCanvasPaintPixels()
    if what == 'blobs' or what == 'all':
      blobs = self.mBrain.mBrum['blobs']
      self.VizCanvasClearBoi()
      self.VizCanvasClearBlobs()
      self.mBlobs = blobs
      self.VizCanvasPaintBlobs()
      self.VizCanvasPaintBoi()
    if what == 'boi_id' or what == 'all':
      boi_id = self.mBrain.mBrum['boi_id']
      self.VizCanvasClearBoi()
      self.mBoiId = boi_id
      self.VizCanvasPaintBoi()
    self.mVizSema.release()


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
    self.mWinBorder = self.mWinGeo[0] - int(self.mVizCanvas['width'])

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
    self._bodyCtlPanel(parent, row, column)


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
    self.mGidPix    = [-1] * _NumPixels # pixels
    self.mGidBlob   = []  # blobs
    self.mGidBoi    = []  # blob of interest

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
    self.mBlobHeight      = int(
      (self.mVizCanvasHeight - _EdgeTop - _EdgeBottom - 4) * _BlobHeightRatio)
    self.mGridWidth       = self.mVizCanvasWidth - _EdgeLeft - _EdgeRight
    self.mGridHeight      = self.mVizCanvasHeight - _EdgeTop - _EdgeBottom - \
                            self.mBlobHeight - 4
    self.mGridMinX        = _EdgeLeft
    self.mGridMinY        = _EdgeTop
    self.mGridMaxX        = self.mGridMinX + self.mGridWidth
    self.mGridMaxY        = self.mGridMinY + self.mGridHeight
    self.mPixWidth        = (self.mGridWidth - _NumPixels - 1)/ _NumPixels
    self.mPixHeight       = self.mGridHeight - 2
    self.mBlobOrigY       = self.mGridMaxY + 2

    # adjust for roundoff errors
    self.mGridWidth       = (self.mPixWidth + 1) * _NumPixels + 1
    self.mGridMaxX        = self.mGridMinX + self.mGridWidth

  #--
  def VizCanvasPaintGrid(self):
    """ Paint the Viz canvas background grid. """
    x0 = self.mGridMinX
    y0 = self.mGridMinY
    x1 = self.mGridMaxX
    y1 = self.mGridMaxY
    id = self.mVizCanvas.create_line((x0, y0, x1, y0), fill=gt.ColorBlue)
    self.mGidGrid += [id]
    id = self.mVizCanvas.create_line((x0, y1, x1, y1), fill=gt.ColorBlue)
    self.mGidGrid += [id]

    n = 0
    while n < _NumPixels + 1:
      x = x0 + (self.mPixWidth + 1) * n
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
  def VizCanvasPaintPixels(self):
    """ Paint the Viz current pixels in grid. """
    n = 0
    while n < _NumPixels:
      pixel = abs(self.mPixels[n])
      if pixel > 255: pixel = 255
      rect = self._pixel2canvas(n)
      pixColor = "#%02x%02x%02x" % (pixel, pixel, pixel)
      id = self.mGidPix[n]
      # replace
      if id >= 0:
        self.mVizCanvas.itemconfigure(id, fill=pixColor, outline=pixColor)
      else:
        id = self.mVizCanvas.create_rectangle(rect, fill=pixColor, 
          outline=pixColor)
        self.mGidPix += [id]
      n += 1

  #--
  def VizCanvasClearPixels(self):
    """ Clear the Viz canvas pixels. """
    for id in self.mGidPix:
      if id >= 0:
        self.mVizCanvas.delete(id)
    self.mGidPix = [-1] * _NumPixels

  #--
  def VizCanvasPaintBlobs(self):
    """ Paint the Viz current blob list. """
    for bid, blob in self.mBlobs.iteritems():
      if bid != self.mBoiId:
        rect = self._blob2canvas(blob['start'], blob['end'])
        id = self.mVizCanvas.create_rectangle(rect, fill=gt.ColorGray1, 
                outline=gt.ColorGray1)
        self.mGidBlob += [id]

  #--
  def VizCanvasClearBlobs(self):
    """ Clear the Viz canvas blobs. """
    for id in self.mGidBlob:
      self.mVizCanvas.delete(id)
    self.mGidBlob = []

  #--
  def VizCanvasPaintBoi(self):
    """ Paint the Viz current blob list. """
    blob = self.mBlobs.get(self.mBoiId, None)
    if not blob:
      return

    # bar at the bottom
    rect = self._blob2canvas(blob['start'], blob['end'])
    id = self.mVizCanvas.create_rectangle(rect, fill='#804000',
          outline='#804000')
    self.mGidBoi += [id]
    y1 = rect[3]  # save for center line

    # pixel fill
    n = blob['start']
    while n <= blob['end']:
      pixel = abs(self.mPixels[n])
      if pixel > 255: pixel = 255
      rect = self._pixel2canvas(n)
      pixColor = "#%02x%02x00" % (pixel, pixel/2)
      id = self.mVizCanvas.create_rectangle(rect, fill=pixColor, 
          outline=pixColor)
      self.mGidBoi += [id]
      n += 1

    # center line
    c = blob['center']
    rect = self._pixel2canvas(int(c))
    x = (rect[2] + rect[0]) / 2
    x += int((self.mPixWidth + 1) * (c - int(c)))
    y0 = rect[1] - 2
    id = self.mVizCanvas.create_line((x, y0, x, y1), fill=gt.ColorRed1, width=3)
    self.mGidBoi += [id]

  #--
  def VizCanvasClearBoi(self):
    """ Clear the Viz canvas blobs. """
    for id in self.mGidBoi:
      self.mVizCanvas.delete(id)
    self.mGidBoi = []

  #--
  def VizCanvasRefresh(self):
    """ Refresh the Viz canvas. """
    self.VizCanvasClearBoi()
    self.VizCanvasClearBlobs()
    self.VizCanvasClearPixels()
    self.VizCanvasClearGrid()
    self.VizCanvasPaintGrid()
    self.VizCanvasPaintPixels()
    self.VizCanvasPaintBlobs()
    self.VizCanvasPaintBoi()

  #--
  def _pixel2canvas(self, n):
    """ Convert pixel number to canvas rectangle coordinates.

        Parameters:
          n - pixel number

        Return Value:
          x0, y0, x1, y1
    """
    x0 = self.mGridMinX + (self.mPixWidth + 1) * n + 1
    y0 = self.mGridMinY + 1
    x1 = x0 + self.mPixWidth - 1
    y1 = self.mGridMaxY - 1
    return x0, y0, x1, y1

  #--
  def _blob2canvas(self, start, end):
    """ Convert pixel number to canvas rectangle coordinates.

        Parameters:
          start - starting pixel number
          end   - ending pixel number

        Return Value:
          x0, y0, x1, y1
    """
    x0 = self.mGridMinX + (self.mPixWidth + 1) * start + 1
    y0 = self.mBlobOrigY
    x1 = self.mGridMinX + (self.mPixWidth + 1) * end + 1 + self.mPixWidth
    y1 = y0 + self.mBlobHeight
    return x0, y0, x1, y1


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui Window Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CbUpdate(self):
    """ Force Visualization Update """
    self.Update('all')

  #--
  def CbFreezeCont(self):
    """ Freeze/Continue Viz canvas updates callback. """
    w = self.mButtonFreezeCont
    if self.mFreeze == True:
      self.mFreeze = False
      w['text']   = 'Freeze'
      w['fg']     = gt.ColorBttnStop
      self.mTtFreezeCont.newtip("Freeze visualiztion updates")
      self.Update('all')
    else:
      self.mFreeze = True
      w['text']   = 'Continue'
      w['fg']     = gt.ColorBttnGo
      self.mTtFreezeCont.newtip("Continue visualization updates")

  #--
  def CbVizClear(self):
    """ Clear the viz's current set of waypoints callback. """
    self.mVizSema.acquire()
    self.VizCanvasClearBoi()
    self.VizCanvasClearBlobs()
    self.VizCanvasClearPixels()
    self.VizCanvasClearGrid()
    self.VizCanvasPaintGrid()
    self.mVizSema.release()

  #--
  def CbSnapShot(self):
    """ Take a snap shot image callback. """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Messy Stuff
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _bodyCtlPanel(self, parent, row, column):
    """ Create the Viz Control Panel.

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
    w = tk.Label(cpframe, text='Optic Flow Control Panel', fg=gt.ColorGreen1)
    w.grid(row=row, column=column, columnspan=6)

    row += 1

    # update button
    w = tk.Button(cpframe, text='Update', fg=gt.ColorBlack, width=8,
        command=self.CbUpdate)
    w.grid(row=row, column=column)
    GuiToolTip.GuiToolTip(w, text="Force visualization update")

    column += 1

    # freeze/thaw button
    w = tk.Button(cpframe, text='Freeze', fg=gt.ColorBttnStop, width=8,
        command=self.CbFreezeCont)
    w.grid(row=row, column=column, sticky=tk.SE)
    self.mTtFreezeCont = GuiToolTip.GuiToolTip(w, 
        text="Freeze visualization updates")
    self.mFreeze = False
    self.mButtonFreezeCont = w

    column += 1

    # clear button
    w = tk.Button(cpframe, text='Clear', fg=gt.ColorBlack, width=8,
        command=self.CbVizClear)
    w.grid(row=row, column=column, sticky=tk.E)
    GuiToolTip.GuiToolTip(w, text="Clear the visualization")

    column += 1

    # snapshot button
    w = tk.Button(cpframe, text='SnapShot', fg=gt.ColorBlack, width=8,
        command=self.CbSnapShot)
    w.grid(row=row, column=column, sticky=tk.SE)
    GuiToolTip.GuiToolTip(w,
        text="Take a snap shot image (future).")


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import Fusion.Hemisson.Brains.vBrainOpticFlow as vBrain
  import time

  #--
  def main():
    """ GuiWinOpticFlowViz Test Main """
    brain = vBrain.vBrainOpticFlow()
    n = 0
    pixList = []
    while n < _NumPixels:
      if n < 10:
        pixList += [255 - n * 16]
      else:
        pixList += [255 - n * 2]
      n += 1
    brain.mBrum['cooked_pixels'] = pixList
    brain.mBrum['blobs']['a'] = {'start':0, 'end':9}
    brain.mBrum['blobs']['b'] = {'start':54, 'end':54}
    brain.mBrum['blobs']['c'] = {'start':99, 'end':101}
    brain.mBrum['boi_id'] = 'a'
    root = tk.Tk()
    win = GuiWinOpticFlowViz(root, brain)
    root.mainloop()

  # run test
  main()
