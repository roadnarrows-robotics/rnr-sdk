################################################################################
#
# GuiXYGraph.py
#

""" Graphical User Interface X-Y Graph Module.

Graphical User Interface (GUI) Tkinter simple x-y graphing module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.25

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
import tkinter as tk

import Fusion.Utils.Pt as pt

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# flag to force real text width and height base calcualations
_DidOneTimes = False

# character characteristics
_CharW  =  9  # max text character width  (pixels)
_CharH  = 15  # max text character height (pixels)

# canvas grid margins 
_EdgeLeft   = _CharW * 8    # left edge margin
_EdgeTop    = _CharH * 2    # top edge margin
_EdgeBottom = _CharH * 3    # bottom edge margin
_EdgeRight  = _CharW * 2    # right edge margin


#-------------------------------------------------------------------------------
# CLASS: GuiXYGraph
#-------------------------------------------------------------------------------
class GuiXYGraph:
  """ GUI X-Y Graphing Class """

  #--
  def __init__(self, canvas, **kwopts):

    """ Initialize x-y graph instance.

        Parameters:
          canvas    - the graphing canvas (tk.Canvas() instance)
          **kwopts: 
            xdata       - list of x data. Default: []
            ydata       - list of y data cooresponding 1-1 with the x data
                          Default: []
            fofx        - y=f(x) data generator. If specified, this function
                          takes precedence over the ydata list.
                          Default: None
            title       - title of this graph. Default: 'y = f(x)'
            xlabel      - x-axis label. Default: 'x'
            ylabel      - y-axis label. Default: 'y'
            xstep       - x-axis grid step size. Default: 1
            ystep       - y-axis grid step size. Default: 1
            linewidth   - graph line width. Default: 1
            linecolor   - graph line color. Default: ColorBlue1
            showpoints: - do [not] show graph (x,y) points. Default: True
            domain:     - do [not] auto limit x domain. if 'auto', then xmin
                          and xmax are determined by the xdata.
                          Otherwise specify (xmin, xmax). Default: 'auto'
            range:      - do [not] auto limit y range. if 'auto', then ymin
                          and ymax are determined by the ydata/fofx.
                          Otherwise specify (ymin, ymax). Default: 'auto'
    """
    # the graphing canvas
    self.mCanvas = canvas

    # force idletask to determine size, etc.
    self.mCanvas.update_idletasks()

    # canvas geometry
    geo = gut.geometry(self.mCanvas)

    # canvas width and height
    self.mCanvasWidth   = geo[gut.W]
    self.mCanvasHeight  = geo[gut.H]

    # canvas graphic ids
    self.mGidsXGrid   = []
    self.mGidsYGrid   = []
    self.mGidsData    = []
    self.mGidsLabels  = []

    # determine real text dimensions used by this canvas
    if not _DidOneTimes:
      self._onetimes()

    # initialize grid dimension data
    self._griddim(geo[gut.W], geo[gut.H])

    # initialize graph data
    self.mXMin    = 0.0
    self.mXMax    = 0.0
    self.mYMin    = 0.0
    self.mYMax    = 0.0
    self.mDomSpan = 0.0
    self.mRanSpan = 0.0
    self.mData    = []

    # graph the data
    self.graph(**kwopts)

  #--
  def configure(self, width, height):
    """ (Re)configure the graph canvas to the new width and height.

        Parameters:
          width   - new graph canvas width in pixels
          height  - new graph canvas height in pixels

        Return Value:
          None
    """
    self.erase()
    self.mCanvas.configure(width=width, height=height)
    self._griddim(width, height)
    # race condition can leave droppings, erase again (kludge)
    self.erase()
    self._graph()

  #--
  def redraw(self):
    """ Redraw complete x-y graph, including title, labels, etc.

        Return Vaue:
          None
    """
    self.erase()
    self._graph()

  #--
  def erase(self):
    """ Erase the x-y graph from canvas. """
    self._erasexgrid()
    self._eraseygrid()
    self._erasedata()
    self._eraselabels()

  #--
  def graph(self, **kwopts):
    """ Graph the x-y data. Any old graph will be automatically erased.

        Parameters:
          See __init__().

        Return Value:
          None
    """
    # graphing options
    self.mOpts = {
      'xdata': [],
      'ydata': [],
      'fofx': None,
      'title':'y = f(x)',
      'xlabel': 'x',
      'ylabel': 'y',
      'xstep': 1,
      'ystep': 1,
      'linewidth': 1,
      'linecolor': gt.ColorBlue1,
      'showpoints': True,
      'domain': 'auto',
      'range': 'auto',
    }

    for key in kwopts:
      if key in self.mOpts:
        self.mOpts[key] = kwopts[key]
      else:
        raise KeyError('unknown x-y graph option: %s' % repr(key))

    # erase any old graph and grid
    self.erase()

    # initialize new graph data
    self.mXMin    = 0.0
    self.mXMax    = 0.0
    self.mYMin    = 0.0
    self.mYMax    = 0.0
    self.mDomSpan = 0.0
    self.mRanSpan = 0.0
    self.mData    = []

    # new graph data
    self.mData = self._gendata(self.mOpts['xdata'], self.mOpts['ydata'])

    # find domain and range of data
    self._domain()      # x domain of data
    self._range()       # y range of data

    # now really graph the data and grid
    self._graph()

  #--
  def newdata(self, xdata=[], ydata=[]):
    """ Graph new x-y data with current graphing options including
        current axes and title labels.

        Parmeters:
          xdata     - list of x data
          ydata     - list of y data cooresponding 1-1 with the x data.
                      (any preexisting fofx() overrides this list).
    """
    # new raw data
    self.mOpts['xdata'] = xdata
    self.mOpts['ydata'] = ydata

    # new graphing data
    self.mData = self._gendata(self.mOpts['xdata'], self.mOpts['ydata'])

    # find scopes of new data
    hasNewDomain  = self._domain()  # x domain of data (sets XMin, XMax)
    hasNewRange   = self._range()   # y range of dat (sets YMin, YMax)

    self._erasedata()   # erase old x-y graph data

    # new domain and/or range, so re-grid
    if hasNewDomain or hasNewRange:
      self._erasexgrid()    # erase x grid 
      self._eraseygrid()    # erase y grid
      self._xgrid()         # draw the x-grid (and x-axis if in domain)
      self._ygrid()         # draw the y-grid (and y-axis if in range)

    # draw the new x-y graph data
    self._graphdata()

  #--
  def _erasexgrid(self):
    """ Erase x grid and ticks. """
    for id in self.mGidsXGrid:
      self.mCanvas.delete(id)
    self.mGidsXGrid = []

  #--
  def _eraseygrid(self):
    """ Erase y grid and ticks. """
    for id in self.mGidsYGrid:
      self.mCanvas.delete(id)
    self.mGidsYGrid = []

  #--
  def _erasedata(self):
    """ Erase x-y data. """
    for id in self.mGidsData:
      self.mCanvas.delete(id)
    self.mGidsData = []

  #--
  def _eraselabels(self):
    """ Erase graph labels. """
    for id in self.mGidsLabels:
      self.mCanvas.delete(id)
    self.mGidsLabels = []

  #--
  def _graph(self):
    """ The graphing grunt. """
    self._xgrid()       # draw the x-grid (and x-axis if in domain)
    self._ygrid()       # draw the y-grid (and x-axis if in range)
    self._xaxislabel()  # draw the x-axis label
    self._yaxislabel()  # draw the y-axis label
    self._title()       # draw the graph title
    self._graphdata()   # draw the graph data

  #--
  def _gendata(self, xdata, ydata):
    """ Generate the x-y data points. """
    data = []
    if self.mOpts['fofx']:
      for x in xdata:
        data.append(pt.pt(x, self.mOpts['fofx'](x)))
    else:
      m = len(xdata)
      n = len(ydata)
      if m > n:
        m = n
      i = 0
      while i < m:
        data.append(pt.pt(xdata[i], ydata[i]))
        i += 1
    return data

  #--
  def _xgrid(self):
    """ Draw x grid lines and x grid labels. """

    # nothing to grid
    if self.mXMin == self.mXMax:
      return

    # current x grid label end pixel position
    labelend = 0

    # x grid line y start and end positions
    ytick0 = self.mGridMinY-2
    ytick1 = self.mGridMaxY+2

    # x grid starting world value
    x = math.ceil(self.mXMin/self.mOpts['xstep']) * self.mOpts['xstep']

    # make the grid lines
    while x <= self.mXMax:
      # map to grid coordinates (don't care about y)
      p = self._world2grid(pt.pt(x, 0.0))

      # color the x origin grid line differently
      if x == 0.0:
        color = gt.ColorPink1
      else:
        color = gt.ColorBlack

      # grid line
      id = self.mCanvas.create_line((p.x, ytick0, p.x, ytick1), fill=color)
      self.mGidsXGrid += [id]

      # label the grid line
      text = self._gridlabelfmt(x) % x
      id = self.mCanvas.create_text((p.x, ytick1+1), fill=color,
              text=text, anchor=tk.N)
      rect = self.mCanvas.bbox(id)

      # no overlap with previous label, the label is good
      if rect[0] > labelend:
        self.mGidsXGrid += [id]
        labelend = rect[2]

      # this label overlapped with previous, so delete
      else:
        self.mCanvas.delete(id)

      # next step
      x += self.mOpts['xstep']

  #--
  def _ygrid(self):
    """ Draw y grid lines and y grid labels. """

    # nothing to grid
    if self.mYMin == self.mYMax:
      return

    # pixels per grid
    ppg = self.mGridHeight / (self.mRanSpan / self.mOpts['ystep'])

    # label modulus
    labelmod = int(float(_CharH) / (ppg + 1.0) + 1)

    # current label count
    labelcnt = 0

    # y grid line x start and end positions
    xtick0 = self.mGridMinX-2
    xtick1 = self.mGridMaxX+2

    # y grid starting world value
    y = math.ceil(self.mYMin/self.mOpts['ystep']) * self.mOpts['ystep']

    # make the grid lines
    while y <= self.mYMax:
      # map to grid coordinates (don't care about x)
      p = self._world2grid(pt.pt(0.0, y))

      # color the y origin grid line differently
      if y == 0.0:
        color = gt.ColorPink1
      else:
        color = gt.ColorBlack

      # the grid line
      id = self.mCanvas.create_line((xtick0, p.y, xtick1, p.y), fill=color)
      self.mGidsYGrid += [id]

      # label the grid line if there is space
      if labelcnt % labelmod == 0:
        text = self._gridlabelfmt(y) % y
        id = self.mCanvas.create_text((xtick0-1, p.y), fill=color,
            text=text, anchor=tk.E)
        self.mGidsYGrid += [id]

        # remember longest y annotation
        rect = self.mCanvas.bbox(id)
        textwid = rect[2] - rect[0]
        if textwid > self.mGridYLabelMaxW:
          self.mGridYLabelMaxW = textwid

      # bump the label cnt
      labelcnt += 1

      # next step
      y += self.mOpts['ystep']

  #--
  def _gridlabelfmt(self, val):
    """ Return grid label text format. """
    val = math.fabs(val)
    if val >= 10000.0:
      return '%.1e'
    elif val >= 1.0:
      return '%.1f'
    elif val >= 0.01:
      return '%.2f'
    elif val >= 0.0001:
      return '%.4f'
    elif val == 0.0:
      return '%.1f'
    else:
      return '%.1e'

  #--
  def _xaxislabel(self):
    """ Draw x-axis label. """
    if not self.mOpts['xlabel']:
      return
    labels = self.mOpts['xlabel'].split('\n')
    nlines = len(labels) 
    xmid = (self.mGridMinX + self.mGridMaxX) / 2
    y = (self.mCanvasHeight + self.mGridMaxY + _CharH)/2 - \
        (nlines * _CharH) / 2
    for label in labels:
      id = self.mCanvas.create_text((xmid, y), fill=gt.ColorBlue1, 
            text=label, anchor=tk.N)
      self.mGidsLabels += [id]
      y += _CharH

  #--
  def _yaxislabel(self):
    """ Draw y-axis label. """
    if not self.mOpts['ylabel']:
      return
    labels = self.mOpts['ylabel'].split('\n')
    nlines = len(labels)
    x = self.mGridMinX - self.mGridYLabelMaxW
    x = x / 2 - (nlines * _CharW) / 2
    ymid = (self.mGridMaxY + self.mGridMinY) / 2
    for label in labels:
      y = ymid - len(label) / 2 * _CharH
      for c in label:
        id = self.mCanvas.create_text((x, y), fill=self.mOpts['linecolor'],
            text=c, anchor=tk.N)
        self.mGidsLabels += [id]
        y += _CharH
      x += _CharW

  #--
  def _title(self):
    """ Draw graph title. """
    if not self.mOpts['title']:
      return
    xmid = (self.mGridMinX + self.mGridMaxX) / 2
    y = self.mGridMinY/2 - _CharH/2
    id = self.mCanvas.create_text((xmid, y), fill=gt.ColorGreen1, 
            text=self.mOpts['title'], anchor=tk.N)
    self.mGidsLabels += [id]

  #--
  def _graphdata(self):
    """ Graph the data. """
    n = len(self.mData)
    if n <= 0:
      return
    p_w = self.mData[0]           # point in world coordinates
    p_g = self._world2grid(p_w)   # point in grid coordinates
    pIsIn = self._gib(p_w)        # point is [not] in graphing scope
    if self.mOpts['showpoints'] and pIsIn:
      id = self.mCanvas.create_oval((p_g.x-2, p_g.y-2, p_g.x+2, p_g.y+2),
          fill=gt.ColorOrange, outline=gt.ColorOrange)
      self.mGidsData += [id]
    i = 1
    while i < n:
      q_w = self.mData[i]
      q_g = self._world2grid(q_w)
      qIsIn = self._gib(q_w)
      if pIsIn and qIsIn:
        id = self.mCanvas.create_line((p_g.x, p_g.y, q_g.x, q_g.y), 
          fill=self.mOpts['linecolor'], width=self.mOpts['linewidth'])
        self.mGidsData += [id]
      elif pIsIn and not qIsIn:
        r_w = self._intercept(p_w, q_w)
        r_g = self._world2grid(r_w)
        id = self.mCanvas.create_line((p_g.x, p_g.y, r_g.x, r_g.y), 
          fill=self.mOpts['linecolor'], width=self.mOpts['linewidth'])
        self.mGidsData += [id]
      elif not pIsIn and qIsIn:
        r_w = self._intercept(q_w, p_w)
        r_g = self._world2grid(r_w)
        id = self.mCanvas.create_line((r_g.x, r_g.y, q_g.x, q_g.y), 
          fill=self.mOpts['linecolor'], width=self.mOpts['linewidth'])
        self.mGidsData += [id]
      if self.mOpts['showpoints'] and qIsIn:
        id = self.mCanvas.create_oval((q_g.x-2, q_g.y-2, q_g.x+2, q_g.y+2),
          fill=gt.ColorOrange, outline=gt.ColorOrange)
        self.mGidsData += [id]
      p_w = q_w
      p_g = q_g
      pIsIn = qIsIn
      i += 1

  #--
  def _gib(self, p):
    """ Checks to see if point p is Graphically In bounds. """
    return p.x >= self.mXMin and p.x <= self.mXMax and \
           p.y >= self.mYMin and p.y <= self.mYMax

  #--
  def _intercept(self, pin, pout):
    """ Find graphing box intercept of line defined by the two points 
        located inside and outside of the box, respectively.
    """
    dx = pout.x - pin.x
    if dx == 0.0: # vertical line
      if pout.y > self.mYMax:
        return pt.pt(pin.x, self.mYMax)
      else:
        return pt.pt(pin.x, self.mYMin)
    dy = pout.y - pin.y
    if dy == 0.0: # horizontal line
      if pout.x > self.mXMax:
        return pt.pt(self.mXMax, pin.y)
      else:
        return pt.pt(self.mXMin, pin.y)
    m = dy/dx
    b = pin.y - m * pin.x
    if pout.x > self.mXMax:
      y = m * self.mXMax + b
      if y >= self.mYMin and y <= self.mYMax:
        return pt.pt(self.mXMax, y)
    elif pout.x < self.mXMin:
      y = m * self.mXMin + b
      if y >= self.mYMin and y <= self.mYMax:
        return pt.pt(self.mXMin, y)
    if pout.y > self.mYMax:
      x = (self.mYMax - b)/m
      return pt.pt(x, self.mYMax)
    else:
      x = (self.mYMin - b)/m
      return pt.pt(x, self.mYMin)

  #--
  def _domain(self):
    """ Determine the data domain minimums, maximums, and spans.

        Return Value:
          True if domain has changed, else false
    """
    # domain is the same
    hasNewDomain = False

    # initialize min and max x
    if not self.mData:    # no data
      xmin = xmax = 0
    elif self.mOpts['domain'] == 'auto':  # automatic
      xmin = xmax = self.mData[0].x
      for p in self.mData:
        if p.x < xmin:
          xmin = p.x
        if p.x > xmax:
          xmax = p.x
    else: # fixed
      xmin = self.mOpts['domain'][0]
      xmax = self.mOpts['domain'][1]

    # adjust for step size
    xmin = math.floor(float(xmin)/float(self.mOpts['xstep'])) \
                    * self.mOpts['xstep']
    xmax = math.ceil(float(xmax)/float(self.mOpts['xstep'])) \
                    * self.mOpts['xstep']

    # heuristically determine if new domain forces a new grid
    if self.mOpts['domain'] == 'auto':
      # grow domain if needed
      if xmin < self.mXMin:
        hasNewDomain = True
      else:
        xmin = self.mXMin    # keep old min
      if xmax > self.mXMax:
        hasNewDomain = True
      else:
        xmax = self.mXMax    # keep old max
      # RDK need heuristic to reduce domain (decay or min step or percent?)

    # set new min and max
    self.mXMin = xmin
    self.mXMax = xmax

    # set domain span
    self.mDomSpan = math.fabs(self.mXMax - self.mXMin)
    if self.mDomSpan == 0.0:
      self.mDomSpan = self.mXMin + self.mOpts['xstep']

    return hasNewDomain

  #--
  def _range(self):
    """ Determine the data range minimums, maximums, and spans.

        Return Value:
          True if range has changed, else false
    """
    # range is the same
    hasNewRange = False

    # initialize min and max x
    if not self.mData:  # no data
      ymin = ymax = 0
    elif self.mOpts['range'] == 'auto': # automatic
      ymin = ymax = self.mData[0].y
      for p in self.mData:
        if p.y < ymin:
          ymin = p.y
        if p.y > ymax:
          ymax = p.y
    else: # fixed
      ymin = self.mOpts['range'][0]
      ymax = self.mOpts['range'][1]

    # adjust for step size
    ymin = math.floor(float(ymin)/float(self.mOpts['ystep'])) \
                      * self.mOpts['ystep']
    ymax = math.ceil(float(ymax)/float(self.mOpts['ystep'])) \
                      * self.mOpts['ystep']

    # heuristically determine if new range forces a new grid
    if self.mOpts['range'] == 'auto':
      # grow range if needed
      if ymin < self.mYMin:
        hasNewRange = True
      else:
        ymin = self.mYMin    # keep old min
      if ymax > self.mYMax:
        hasNewRange = True
      else:
        ymax = self.mYMax    # keep old max
      # RDK need heuristic to reduce range (decay or min step or percent?)

    # set new min and max
    self.mYMin = ymin
    self.mYMax = ymax

    # set range span
    self.mRanSpan = math.fabs(self.mYMax - self.mYMin)
    if self.mRanSpan == 0.0:
      self.mRanSpan = self.mYMin + self.mOpts['ystep']

    return hasNewRange

  #--
  def _world2grid(self, p):
    """ Map world coordinates to canvas grid coordinates. """
    x = self.mGridMinX + \
        (p.x - self.mXMin) / self.mDomSpan * self.mGridWidth
    y = self.mGridMaxY - \
        (p.y - self.mYMin) / self.mRanSpan  * self.mGridHeight
    return pt.pt(int(x),int(y))

  #--
  def _onetimes(self):
    """ One time, run time calculations for global data. """
    global _DidOneTimes
    global _CharW, _CharH
    global _EdgeLeft, _EdgeTop, _EdgeBottom, _EdgeRight

    # maximum character width and height
    _CharW = self._textwidth('M')
    _CharH = self._textheight('Mp')

    # canvas grid margins 
    _EdgeLeft   = self._textwidth('MM 0.0e+00')
    _EdgeTop    = _CharH * 2 
    _EdgeBottom = _CharH * 3 
    _EdgeRight  = _CharW * 2

    _DidOneTimes = True

  #--
  def _griddim(self, width, height):
    """ Initialize grid related dimension parameters. """
    # canvas width and height
    self.mCanvasWidth   = width
    self.mCanvasHeight  = height

    # grid dimensions
    self.mGridWidth         = self.mCanvasWidth - _EdgeLeft - _EdgeRight
    self.mGridHeight        = self.mCanvasHeight - _EdgeTop - _EdgeBottom
    self.mGridMinX          = _EdgeLeft
    self.mGridMinY          = _EdgeTop
    self.mGridMaxX          = self.mGridMinX + self.mGridWidth
    self.mGridMaxY          = self.mGridMinY + self.mGridHeight
    self.mGridYLabelMaxW    = 0

  #--
  def _textwidth(self, text):
    """ Calculate width of text in pixels. """
    id = self.mCanvas.create_text((1, 1), 
            text=text, anchor=tk.NW)
    rect = self.mCanvas.bbox(id)
    w = rect[2] - rect[0]
    self.mCanvas.delete(id)
    return w

  #--
  def _textheight(self, text):
    """ Calculate height of text in pixels. """
    id = self.mCanvas.create_text((1, 1), 
            text=text, anchor=tk.NW)
    rect = self.mCanvas.bbox(id)
    h = rect[3] - rect[1]
    self.mCanvas.delete(id)
    return h


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  root        = None
  graph       = None
  rootgeo     = None
  rootframeh  = 0
  rootborder  = 0
  val         = 0

  #--
  def calcdim(canvas, frame):
    global root, rootgeo, rootframeh, rootborder
    root.update_idletasks()
    rootgeo = gut.geometry(root)
    fgeo = gut.geometry(frame)
    rootframeh = fgeo[1]
    rootborder = rootgeo[0] - int(canvas['width'])

  #--
  def resize(event):
    global root, graph, rootgeo, rootframeh, rootborder
    geo = gut.geometry(root)
    if geo[0] != rootgeo[0] or geo[1] != rootgeo[1]:
      width = geo[0] - rootborder
      height = geo[1] - rootborder - rootframeh
      graph.configure(width, height)
      rootgeo = geo
    
  #--
  def parabola(x):
    return math.pow(x, 2.0)

  def p3(x):
    return math.pow(x, 3.0)

  def degcos(x):
    return math.cos(math.radians(x))

  #--
  def cbNext():
    global graph, val
    if val == 0:
      graph.graph(title='Raw Data',
              xdata=[-3.2, -1.4, 0.0, 0.5, 2.0, 5.6], 
              ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 9.1],
              xstep=1.0, ystep=2.0, xlabel='weed\n(lbs)', ylabel='wacky\ndacky')
      print(val, 'raw data graph')
    elif val == 1:
      graph.newdata(xdata=[-4.0, -1.4, 0.0, 0.5, 2.0, 6.0], 
                    ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 9.1])
      print(val, 'new raw data - should have same domain and range')
    elif val == 2:
      graph.newdata(xdata=[-4.0, -1.4, 0.0, 0.5, 2.0, 6.0], 
                    ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 10.0])
      print(val, 'new raw data - should have same domain and range')
    elif val == 3:
      graph.newdata(xdata=[-4.4, -1.4, 0.0, 0.5, 2.0, 6.0], 
                    ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 10.0])
      print(val, 'new raw data - should have new -x domain')
    elif val == 4:
      graph.newdata(xdata=[-4.4, -1.4, 0.0, 0.5, 2.0, 7.0], 
                    ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 10.0])
      print(val, 'new raw data - should have new +x domain')
    elif val == 5:
      graph.newdata(xdata=[-4.4, -1.4, 0.0, 0.5, 2.0, 7.0], 
                    ydata=[-12.0, 4.9, 0.0, -3.1, 6.8, 11.0])
      print(val, 'new raw data - should have new +y range')
    elif val == 6:
      graph.newdata(xdata=[-4.4, -1.4, 0.0, 0.5, 2.0, 7.0], 
                    ydata=[-12.1, 4.9, 0.0, -3.1, 6.8, 10.0])
      print(val, 'new raw data - should have new -y range')
    elif val == 7:
      graph.graph(title='y = x ^ 2',
              xdata=[0.0, 0.5, 1.0, 1.5, 2.0],
              fofx=parabola,
              xstep=1.0, ystep=1.0, linewidth=3, linecolor=gt.ColorGreen1,
              showpoints=False)
      print(val, 'graph x^2')
    elif val == 8:
      graph.graph(title='y = x ^ 3',
              xdata=[-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0],
              fofx=p3,
              xstep=1.0, ystep=1.0, linecolor=gt.ColorRed1,
              domain=(-2.0, 2.0), range=(-4.0, 4.0))
      print(val, 'graph x^3')
    elif val == 9:
      graph.graph(title='y = cos(x)',
              xdata=[180, 135, 90, 45, 0, -45, -90, -135, -180],
              fofx=degcos,
              xstep=15.0, ystep=.5, linecolor=gt.ColorRed1, linewidth=2,
              domain=(-150, 60))
      print(val, 'graph cos(x)')
    elif val == 10:
      graph.graph(title='null data')
      print(val, 'graph no data')
    val = (val + 1) % 11

  #--
  def main():
    """ GuiXYGraph Test Main """
    global root, graph, rootgeo
    root = tk.Tk()
    canvas = tk.Canvas(root, width=200, height=200)
    canvas.grid(row=0, column=0)
    frame = tk.Frame(root)
    frame.grid(row=1, column=0)
    bttn = tk.Button(frame, text='Next', command=cbNext)
    bttn.grid(row=0, column=0)
    bttn = tk.Button(frame, text='GoodBye', command=root.destroy)
    bttn.grid(row=0, column=1)
    graph = GuiXYGraph(canvas)
    calcdim(canvas, frame)
    root.bind('<Configure>', resize)

    root.mainloop()

  # run test
  main()
