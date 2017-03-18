###############################################################################
#
# Module:   Laelaps.VelPlot
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     VelPlot.py
#
## \file 
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $
## $Rev: 4293 $
##
## \brief Motor velocity plot class.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright
##   \h_copy 2015-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##
# @EulaBegin@
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
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import math
import getopt

import numpy as np

import matplotlib
matplotlib.use('TkAgg')

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, \
                                              NavigationToolbar2TkAgg
from matplotlib.figure import Figure

import matplotlib.pyplot as mplPlot
import matplotlib.lines as mplLine
import matplotlib.animation as mplAnim
import matplotlib.colors as mplColors

if sys.version_info[0] < 3:
  from Tkinter import *
  from Tkconstants import *
  from tkFileDialog import *
  import tkFont
else:
  from tkinter import *
  from Tkconstants import *
  from tkFileDialog import *
  import tkFont

import Laelaps.Utils as ut

_spkey = lambda key: 'sp_'+key


# ------------------------------------------------------------------------------
# Class VelPlotData
# ------------------------------------------------------------------------------

class VelPlotData:
  def __init__(self, ax, maxt=4, dt=0.02):
    self.ax = ax
    self.dt = dt
    self.maxt = maxt

    self.cntEnabled = 0
    self.enabled    = {}
    self.lines      = {}
    self.vdata      = {}
    self.setpoints  = {}

    self.tdata = [0]

    self.ax.set_ylim(-100.1, 100.1)
    self.ax.set_xlim(0, self.maxt)

  def addVel(self, key, color):
    # current velocity
    self.enabled[key] = False
    self.vdata[key] = [0]
    self.lines[key] = mplLine.Line2D(self.tdata, self.vdata[key])
    self.lines[key].set_color(mplColors.hex2color(color))
    self.lines[key].set_linestyle('-')
    self.ax.add_line(self.lines[key])

    # velocity setpoint
    key_sp = _spkey(key)
    self.enabled[key_sp] = False
    self.vdata[key_sp] = [0]
    self.setpoints[key_sp] = 0
    self.lines[key_sp] = mplLine.Line2D(self.tdata, self.vdata[key_sp])
    self.lines[key_sp].set_color(mplColors.hex2color(color))
    self.lines[key_sp].set_linestyle('--')
    self.lines[key_sp].set_linewidth(2.0)
    self.ax.add_line(self.lines[key_sp])

  def update(self, y):
    if self.cntEnabled <= 0 or len(self.lines) == 0:
      return tuple(self.lines.values())
    lastt = self.tdata[-1]
    if lastt > self.tdata[0] + self.maxt: # reset the arrays
      self.tdata.pop(0)
      for key in self.vdata.keys():
        if self.enabled[key]:
          self.vdata[key].pop(0)
      self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
      self.ax.figure.canvas.draw()
    t = self.tdata[-1] + self.dt
    self.tdata.append(t)
    for key in self.vdata.keys():
      if self.enabled[key]:
        if key.find('sp_') >= 0:
          self.vdata[key].append(self.setpoints[key])
        else:
          self.vdata[key].append(y[key])
        self.lines[key].set_data(self.tdata, self.vdata[key])
    return tuple(self.lines.values())

  def emitter(self):
    while True:
      v = {}
      for k,s in self.enabled.iteritems():
        if s and k.find('sp_') == -1:
          v[k] = 200.0 * np.random.rand(1) - 100.0
      yield v
      #v = np.random.rand(1)
      #yield 200.0 * np.random.rand(1) - 100.0
      #if v > p:
      #  yield 0.
      #else:
      #  yield 100.0 * np.random.rand(1)

  def enable(self, key, setpoint):
    if not self.enabled[key]:
      self.vdata[key] = [0] * len(self.tdata)
      self.enabled[key] = True

      key_sp = _spkey(key)
      self.setpoints[key_sp] = setpoint
      self.vdata[key_sp] = [setpoint] * len(self.tdata)
      self.enabled[key_sp] = True

      self.cntEnabled += 2

  def disable(self, key):
    if self.enabled[key]:
      self.enabled[key] = False

      key_sp = _spkey(key)
      self.enabled[key_sp] = False

      self.cntEnabled -= 2

  def setpoint(self, key, setpoint):
    key_sp = _spkey(key)
    self.setpoints[key_sp] = setpoint


# ------------------------------------------------------------------------------
# Class VelPlot
# ------------------------------------------------------------------------------

class VelPlot:
  def __init__(self, wCanvas, width, height, motorNames, colors):
        #
    # Create matplot subplots.
    # Note: Size are in ridiculus inches units. 
    #
    dpi   = 100 # does not have to be real
    color = mplColors.hex2color("#ffcccc")

    fig = mplPlot.Figure(figsize=(width/dpi, height/dpi), dpi=dpi, 
        facecolor=color)

    ax = fig.add_subplot(111, xlabel="t (seconds)", ylabel='v (percent)')

    self.m_rtData = VelPlotData(ax, dt=0.01)

    for motor in motorNames:
      self.m_rtData.addVel(motor, colors[motor])

    canvas = FigureCanvasTkAgg(fig, master=wCanvas)
    canvas.get_tk_widget().grid(row=0, column=0)

    # pass a generator in "emitter" to produce data for the update func
    ani = mplAnim.FuncAnimation(fig, self.m_rtData.update,
                self.m_rtData.emitter, interval=10, blit=True)

    canvas.show()

  def enable(self, motors, setpoints):
    if type(motors) == str:
      self.m_rtData.enable(motors, setpoints)
    elif type(motors) == list or type(motors) == tuple:
      for motor in motors:
        self.m_rtData.enable(motor, setpoints[motor])
    else:
      print "BUG: Bad paramaters:", repr(motors), repr(setpoints)

  def disable(self, motors):
    if type(motors) == str:
      self.m_rtData.disable(motors)
    elif type(motors) == list or type(motors) == tuple:
      for motor in motors:
        self.m_rtData.disable(motor)
    else:
      print "BUG: Bad paramaters:", repr(motors), repr(setpoints)

  def setpoint(self, motors, setpoints):
    if type(motors) == str:
      self.m_rtData.setpoint(motors, setpoints)
    elif type(motors) == list or type(motors) == tuple:
      for motor in motors:
        self.m_rtData.setpoint(motor, setpoints[motor])
    else:
      print "BUG: Bad paramaters:", repr(motors), repr(setpoints)
