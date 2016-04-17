################################################################################
#
# GuiDlgKheCalIrLed.py
#

""" Graphical User Interface vKhepera IR LED Calibration Dialog Module

Graphical User Interface (GUI) Tkinter vKhepera calibration dialog
module for the built-in Khepera proximity/ambient IR LED sensors.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.27

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

import Tkinter as tk
import tkSimpleDialog

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiXYGraph as GuiXYGraph


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames(sensorName):
  """ Return list of per sensor dictionary keys of selected settings 
      and results.
  """
  return ['enabled', 'k_brightness', 'noise_floor']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgKheCalIrLed
#-------------------------------------------------------------------------------
class GuiDlgKheCalIrLed(tkSimpleDialog.Dialog):
  """ vKhepera IR LED Sensors Calibration Dialog Class

      The result on dialog exit:
        On ok success, result dictionary:
          {<sensorId>: 
              {'enabled':bool, 'k_brightness':float, 'noise_floor':int},
           <sensorId>: ...
           ...
          }
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, sensorMimeType, sensorName, cbCalData,
                     factSettings, lastSettings):
    """ Initialize the Debug Dialog.

        Parameters:
          guiParent       - this dialog's parent GUI object
          sensorMimeType  - MIME type of sensor
          sensorName      - sensor name string
          cbCalData       - callback to get calibration data
          factSettings    - factory limits and defaults
          lastSettings    - settings of last configurations.
                            See result on exit.
    """
    self.result           = None
    self.mSensorMimeType  = sensorMimeType
    self.mSensorName      = sensorName
    self.mCbCalData       = cbCalData
    self.mFactory         = factSettings
    self.mSettings        = lastSettings

    self.mHasDim          = False

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def body(self, master):
    """Create the dialog body."""
    # dialog title
    self.wm_title('vKhepera %s IR LED Sensors Calibration' % self.mSensorName)

    row    = 0
    column = 0

    # calibration graph canvas
    self._bodyCalCanvas(master, row, column)

    # control panel frame
    row +=1 
    self._bodyCtlPanel(master, row, column)

    # graph current calibration data
    self._graph(self.mSliderK.get(), self.mSliderNF.get())

    # bind resizer
    self.bind('<Configure>', self.resize)

  #--
  def _bodyCalCanvas(self, master, row, column):
    """Create calibration x-y graph canvas. """
    # calibration graph canvas
    self.mCalCanvas = tk.Canvas(master, relief=tk.SUNKEN, borderwidth=0,
                                width=500, height=400)
    self.mCalCanvas.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
                         sticky=tk.N+tk.W+tk.E)

    # factory data
    factdata = self.mCbCalData()
    self.mUnitsY = factdata['unitsY']
    self.mUnitsX = factdata['unitsX']
    self.mFactCalData = factdata['calData']

    # graph title
    self.mGraphTitle = self.mSensorName + ' IR LED Calibration  (' + \
        self.mSensorMimeType + ')'

    # tailor y-axis step size
    if self.mFactory['MaxDist'] >= 300.0:
      self.mYStep = 100
    else:
      self.mYStep = 10

    # start with empty graph
    self.mXYGraph = GuiXYGraph.GuiXYGraph(self.mCalCanvas)

  #--
  def _bodyCtlPanel(self, master, row, column):
    """Create Control Panel subdialog frame."""
    # control panel frame
    cpframe = tk.Frame(master, relief=tk.FLAT, borderwidth=1)
    cpframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    self.mCtlPanelFrame = cpframe
    
    row = 0
    column = 0

    # sensor sets subframe
    ssframe = tk.Frame(cpframe, relief=tk.RAISED, borderwidth=1)
    ssframe.grid(row=row, column=column, padx=1, pady=1, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.S)

    row = 0
    column = 0

    # sensor sets subpanel title
    w = tk.Label(ssframe, text='Calibration Set Inclusion', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=11, sticky=tk.N)

    row += 1

    ids = self.mSettings.keys()
    ids.sort()
    for id in ids:
      self.mSettings[id]['incvar'] = tk.IntVar()
      w = tk.Checkbutton(ssframe, variable=self.mSettings[id]['incvar'],
          command=self.CbSensorInc, anchor=tk.W)
      w.grid(row=row, column=column, sticky=tk.W)
      w.select()

      column += 1
      w = tk.Label(ssframe, text=id)
      w.grid(row=row, column=column, sticky=tk.W)
      self.mSettings[id]['widgets'] = [w]

      column += 1
      w = tk.Label(ssframe, text=' ', width=2)
      w.grid(row=row, column=column, sticky=tk.E)

      column += 1
      w = tk.Label(ssframe, text='enabled:', anchor=tk.E)
      w.grid(row=row, column=column, sticky=tk.E)
      self.mSettings[id]['widgets'] += [w]

      column += 1
      self.mSettings[id]['envar'] = tk.StringVar()
      w = tk.Label(ssframe, width=5, textvariable=self.mSettings[id]['envar'],
          anchor=tk.W, fg=gt.ColorGreen1, relief=tk.SUNKEN)
      w.grid(row=row, column=column, sticky=tk.W)
      self.mSettings[id]['widgets'] += [w]

      column += 1
      w = tk.Label(ssframe, text=' ', width=1)
      w.grid(row=row, column=column, sticky=tk.E)

      column += 1
      w = tk.Label(ssframe, text='k_brightness:', anchor=tk.E)
      w.grid(row=row, column=column, sticky=tk.E)
      self.mSettings[id]['widgets'] += [w]

      column += 1
      self.mSettings[id]['kbvar'] = tk.DoubleVar()
      w = tk.Label(ssframe, width=5, textvariable=self.mSettings[id]['kbvar'],
          anchor=tk.W, fg=gt.ColorGreen1, relief=tk.SUNKEN)
      w.grid(row=row, column=column, sticky=tk.W)
      self.mSettings[id]['widgets'] += [w]

      column += 1
      w = tk.Label(ssframe, text=' ', width=1)
      w.grid(row=row, column=column, sticky=tk.E)

      column += 1
      w = tk.Label(ssframe, text='noise_floor:', anchor=tk.E)
      w.grid(row=row, column=column, sticky=tk.E)
      self.mSettings[id]['widgets'] += [w]

      column += 1
      self.mSettings[id]['nfvar'] = tk.IntVar()
      w = tk.Label(ssframe, width=5, textvariable=self.mSettings[id]['nfvar'],
          anchor=tk.W, fg=gt.ColorGreen1, relief=tk.SUNKEN)
      w.grid(row=row, column=column, sticky=tk.W)
      self.mSettings[id]['widgets'] += [w]

      row += 1
      column = 0

    self.SetSet()

    row = 1
    column = 0

    # sensor control subframe
    scframe = tk.Frame(cpframe, relief=tk.RAISED, borderwidth=1)
    scframe.grid(row=row, column=column, padx=1, pady=1, ipadx=1, ipady=1, 
               sticky=tk.N+tk.W+tk.E)

    row = 0
    column = 0

    # sensor control subpanel title
    w = tk.Label(scframe, text='Calibration Control', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=5, sticky=tk.N)

    row += 1

    # k-brightness slider
    w = tk.Scale(scframe, width=10, length=250,
        from_=self.mFactory['KBrightnessMin'],
        to=self.mFactory['KBrightnessMax'],
        resolution=0.01, orient=tk.HORIZONTAL, command=self.CbK,
        label='k_brightness Ratio')
    w.grid(row=row, column=column, sticky=tk.SW, pady=1)
    GuiToolTip.GuiToolTip(w, 
        text="Set %s k_brightness ratio.\nWhite paper = 1.0" % self.mSensorName)
    self.mSliderK = w
    self.mSliderK.set(self.mFactory['KBrightnessDft'])

    row += 1

    # Noise Floor slider
    w = tk.Scale(scframe, width=10, length=250,
        from_=self.mFactory['NoiseFloorMin'],
        to=self.mFactory['NoiseFloorMax'],
        resolution=1, orient=tk.HORIZONTAL, command=self.CbNF,
        label='Sensor ADC Noise Floor')
    w.grid(row=row, column=column, sticky=tk.SW, pady=1)
    GuiToolTip.GuiToolTip(w, 
        text="Set sensor noise floor. Any ADC values < noise floor are "
        "considered noise which will map to the sensor's 'infinite' distance.")
    self.mSliderNF = w
    self.mSliderNF.set(self.mFactory['NoiseFloorDft'])

    row = 1
    column += 1

    # column spacer
    tk.Label(scframe, text=' ', width=2).grid(row=row, column=column)

    column += 1

    # enable/disable
    self.mVarEnable = tk.IntVar()
    w = tk.Checkbutton(scframe, variable=self.mVarEnable,
        text='Enable Sensors', command=self.CbSensorEnable, anchor=tk.W)
    w.grid(row=row, column=column, sticky=tk.W+tk.S)
    GuiToolTip.GuiToolTip(w,
        'Enable/disable selected sensors from robot sensing operations.') 
    w.select()

    row += 1

    # factory defaults button
    w = tk.Button(scframe, text='Factory', fg=gt.ColorBlack, width=8,
                  command=self.CbFact)
    w.grid(row=row, column=column, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="Set calibration to factory defaults.")

    row = 1
    column += 1

    # column spacer
    tk.Label(scframe, text=' ', width=2).grid(row=row, column=column)

    column += 1

    # apply button
    w = tk.Button(scframe, text='Update\nSelected\nSensors',
        fg=gt.ColorBlack, width=8, height=4, command=self.CbUpdate)
    w.grid(row=row, column=column, rowspan=2)
    GuiToolTip.GuiToolTip(w,
        text="Update calibration to selected sensors data.")

  #-- 
  def SetSet(self):
    """ Initial sensor set values. """
    for sensor in self.mSettings.itervalues():
      if sensor['incvar'].get() == 0:
        state = tk.DISABLED
      else:
        state = tk.NORMAL
      for label in sensor['widgets']:
        label['state'] = state
      if sensor['enabled']:
        sensor['envar'].set('True')
      else:
        sensor['envar'].set('False')
      sensor['kbvar'].set(sensor['k_brightness'])
      sensor['nfvar'].set(sensor['noise_floor'])
        
  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort.
        Note: Cannot do the calculations within the body() or incorrect
              dialog sizes will result. Must wait until parent dialog
              finishes drawing.
    """
    # force idletask to determine size
    self.update_idletasks()

    # current window dimensions
    self.mDlgGeo = gut.geometry(self)

    # current control panel dimensions
    cpgeo = gut.geometry(self.mCtlPanelFrame)

    # calibration canvas dimensions
    ccw = int(self.mCalCanvas['width'])
    cch = int(self.mCalCanvas['height'])

    # control panel frame height is fixed (includes 'Ok' and 'Cancel'
    self.mDlgFixH = self.mDlgGeo[gut.H] - cch

    # window border width and height
    self.mWinBorder = self.mDlgGeo[gut.W] - ccw

    # set window's minimum size 
    self.wm_minsize(
        width=500+self.mWinBorder,
        height=200+self.mWinBorder+self.mDlgFixH
    )

  #--
  def resize(self, event):
    """ Resize callback event. """
    # first time through: calculate important window and widget dimensions 
    # used for resizing
    if not self.mHasDim:
      self.CalcDim()
      self.mHasDim = True
      return

    # real resize event
    geo = gut.geometry(self)
    if geo[gut.W] != self.mDlgGeo[gut.W] or geo[gut.H] != self.mDlgGeo[gut.H]:
      width = geo[gut.W] - self.mWinBorder
      height = geo[gut.H] - self.mWinBorder - self.mDlgFixH
      self.mXYGraph.configure(width=width, height=height)
      self.mDlgGeo = geo

  #--
  def ok(self, event=None):
    """Dialog OK button callback."""

    if not self.validate():
      return

    self.withdraw()
    self.update_idletasks()

    self.apply()

    self.cancel() # exit

    return

  #--
  def validate(self):
    """Validate dialog settings."""
    self.result = {}
    for id,sensor in self.mSettings.iteritems():
      self.result[id] = {}
      if sensor['envar'].get() == 'True':
        self.result[id]['enabled'] = True
      else:
        self.result[id]['enabled'] = False
      self.result[id]['k_brightness'] = sensor['kbvar'].get()
      self.result[id]['noise_floor'] = sensor['nfvar'].get()
    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def CbK(self, val):
    """ K-Brightness Slider callback. """
    self._graph(self.mSliderK.get(), self.mSliderNF.get())

  #--
  def CbNF(self, val):
    """ Noise Floor Slider callback. """
    self._graph(self.mSliderK.get(), self.mSliderNF.get())

  #--
  def CbSensorInc(self):
    """ Sensor Set Inclusion Checkbox callback. """
    for sensor in self.mSettings.itervalues():
      if sensor['incvar'].get() == 0:
        state = tk.DISABLED
      else:
        state = tk.NORMAL
      for label in sensor['widgets']:
        label['state'] = state

  #--
  def CbSensorEnable(self):
    """ Sensors Control Enable Checkbox callback """
    pass

  #--
  def CbFact(self):
    """ Factory Defaults Button callback. """
    self.mSliderK.set(self.mFactory['KBrightnessDft'])
    self.mSliderNF.set(self.mFactory['NoiseFloorDft'])
    self.mVarEnable.set(1)
    self._graph(self.mFactory['KBrightnessDft'],
                self.mFactory['NoiseFloorDft'])

  #--
  def CbUpdate(self):
    """ Update Parameters to Inclusion Set Button callback. """
    enable = self.mVarEnable.get()
    k = self.mSliderK.get()
    nf = self.mSliderNF.get()
    for sensor in self.mSettings.itervalues():
      if sensor['incvar'].get():
        if enable:
          sensor['envar'].set('True')
        else:
          sensor['envar'].set('False')
        sensor['kbvar'].set(k)
        sensor['nfvar'].set(nf)

  #--
  def _graph(self, k, nf):
    """ Graph the calibration data.

        Parameters:
          k - k_brightness ratio
    """
    xData = []
    yData = []
    for x, y in self.mFactCalData:
      if x >= nf:
        d = y * k
        xData.append(x)
        yData.append(d)
    
    self.mXYGraph.graph(title=self.mGraphTitle,
                        xdata=xData, ydata=yData,
                        xstep=100, ystep=self.mYStep,
                        xlabel='Raw  ~  %s' % self.mUnitsX,
                        ylabel='Distance ~ %s' % self.mUnitsY)


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import  Fusion.Khepera.Cmd.KheCmdBase as KheCmdBase

  def main():
    """ GuiDlgKheCalIrLed Test Main """
    factSettings  = {
      'KBrightnessMin': KheCmdBase.KheIrProxMinKBrightness,
      'KBrightnessMax': KheCmdBase.KheIrProxMaxKBrightness,
      'KBrightnessDft': KheCmdBase.KheIrProxDftKBrightness,
      'NoiseFloorMin': KheCmdBase.KheIrProxMinNoiseFloor,
      'NoiseFloorMax': KheCmdBase.KheIrProxMaxNoiseFloor,
      'NoiseFloorDft': KheCmdBase.KheIrProxDftNoiseFloor,
      'MaxDist': KheCmdBase.KheIrProxMaxDist * 2   # for graphing
    }
    
    cmd = KheCmdBase.KheCmdBase()
    root = tk.Tk()
    dlg = GuiDlgKheCalIrLed(root,
                            'sensor/reflective-irled',
                            'Proximity',
                            cmd.ProximitySensorGetDftCalibration,
                            factSettings,
                            cmd.ProximitySensorsGetCalParams())
 
    if dlg.result:
      print 'ok:', dlg.result
    else:
      print 'cancel'

  # run test
  main()
