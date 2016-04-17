################################################################################
#
# GuiDlgHemiOpt.py
#

""" Graphical User Interface vHemisson Options Dialog Module

Graphical User Interface (GUI) Tkinter vHemisson options dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.07

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

import  Tkinter as tk
import  tkSimpleDialog
import  tkFont
import  Fusion.Gui.GuiToolTip as GuiToolTip
import  Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['UseProximitySensors', 'UseAmbientSensors', 'UseLinCamSensor',
          'UseUssSensor', 'UseSpeedometerSensors', 'AutoConnect',
          'ExecCycle', 'ExecStepSize']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgHemiOpt
#-------------------------------------------------------------------------------
class GuiDlgHemiOpt(tkSimpleDialog.Dialog):
  """ vHemisson Run-Time Options Dialog Class

      The result on dialog exit:
        On ok success, result dictionary:
          {'UseProximitySensors':truefalse,
           'UseAmbientSensors':truefalse, 
           'UseLinCamSensor':truefalse,
           'UseUssSensor':truefalse,
           'UseSpeedometerSensors':truefalse,
           'AutoConnect':truefalse,
           'ExecCycle':seconds,
           'ExecStepSize':seconds}
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, lastSettings={}):
    """ Initialize the Debug Dialog.

        Parameters:
          guiParent     - this dialog's parent GUI object
          lastSettings  - settings of last configurations.
                          See Return Value.
    """
    self.result         = None
    self.mLastSettings  = lastSettings

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if self.mLastSettings.has_key(key) and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    elif key == 'ExecCycle' or key == 'ExecStepSize':
      return 0.10
    else:
      return False

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('vHemisson Options')

    font = tkFont.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Sensor Select Frame
    row +=1 
    self._bodySensorFrame(master, row, column)

    # Execution Select Frame
    column += 1
    self._bodyExecFrame(master, row, column)

    # Status bar
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=2, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select vHemisson options.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodySensorFrame(self, master, row, column):
    """Create Sensor subdialog frame."""
    self.mSensorGroups = [
      ['UseProximitySensors',   'Proximity IR LED Sensors', tk.IntVar(),
      'Enable/disable Hemisson built-in infrared LED sensors in '
      'reflective transmit/receive mode.'],
      ['UseAmbientSensors',     'Ambient IR LED Sensors', tk.IntVar(),
      'Enable/disable Hemisson built-in infrared LED sensors in '
      'passive ambient receive mode.'],
      ['UseSpeedometerSensors', 'Speedometery', tk.IntVar(),
      'Enable/disable Hemisson built-in left and right wheel dc speedometers.'],
      ['UseLinCamSensor',    'Linear Camera Module Sensor', tk.IntVar(),
      'Enable/disable Hemisson Linear Camera Sensor Module.\n'
      'The module must also be attached.'],
      ['UseUssSensor',    'UltraSonic Sensor Module', tk.IntVar(),
      'Enable/disable Hemisson USS Module Sensor.\n'
      'The module must also be attached.'],
    ]

    senframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    senframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    row = 0
    column = 0
    w = tk.Label(senframe, text='Sensor Groups:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

    column = 0

    i = 0
    for data in self.mSensorGroups:
      row += 1
      if i == 3:
        row -= 3
        column += 1
      val = self._lastSetting(data[0])
      if val == True:
        val = 1
      else:
        val = 0
      data[2].set(val)
      b = tk.Checkbutton(senframe,
                         text=data[1],
                         variable=data[2])
      b.grid(row=row, column=column, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(b, data[3])
      i += 1

  #--
  def _bodyExecFrame(self, master, row, column):
    """Create Exec subdialog frame."""
    self.mVarExecCycle    = tk.DoubleVar()
    self.mVarExecStepSize = tk.DoubleVar()
    self.mVarAutoConnect  = tk.IntVar()

    execframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    execframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    
    row = 0
    column = 0
    w = tk.Label(execframe, text='Execution Control Options:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    column = 0
    w = tk.Label(execframe, text='Execution Cycle:')
    w.grid(row=row, column=column, sticky=tk.E)

    column += 1
    self.mEntryExecCycle = tk.Entry(execframe, width=8,
                                    textvariable=self.mVarExecCycle) 
    self.mEntryExecCycle.grid(row=row, column=column, columnspan=2,
        sticky=tk.N+tk.S+tk.W+tk.E)
    val = self._lastSetting('ExecCycle')
    self.mEntryExecCycle.delete(0, tk.END)
    s = '%-.2f' % val
    self.mEntryExecCycle.insert(0, s)
    GuiToolTip.GuiToolTip(self.mEntryExecCycle,
        text='Core execution sense/react cycle period (seconds)')

    row += 1
    column = 0
    w = tk.Label(execframe, text='Step Size:')
    w.grid(row=row, column=column, sticky=tk.E)

    column += 1
    self.mEntryExecStepSize = tk.Entry(execframe, width=8,
                                    textvariable=self.mVarExecStepSize) 
    self.mEntryExecStepSize.grid(row=row, column=column, columnspan=2,
        sticky=tk.N+tk.S+tk.W+tk.E)
    val = self._lastSetting('ExecStepSize')
    self.mEntryExecStepSize.delete(0, tk.END)
    s = '%-.2f' % val
    self.mEntryExecStepSize.insert(0, s)
    GuiToolTip.GuiToolTip(self.mEntryExecStepSize,
        text="Size (seconds) per 'Step'")

    row +=1
    column = 0
    val = self._lastSetting('AutoConnect')
    if val == True:
      val = 1
    else:
      val = 0
    self.mVarAutoConnect.set(val)
    b = tk.Checkbutton(execframe, text='AutoConnect', 
                       variable=self.mVarAutoConnect)
    b.grid(row=row, column=column, columnspan=2, sticky=tk.W)
    GuiToolTip.GuiToolTip(b,
        text="Do [not] autoconnect to Hemisson if port and baudrate are known.")

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

    try:
      execCycle = self.mVarExecCycle.get()
    except ValueError, err:
      self.ShowStatus("Execution Cycle: field has invalid value: %s" % \
          repr(self.mEntryExecCycle.get()),
          fg=gt.ColorFgStatusError)
      self.mEntryExecCycle.focus_set()
      return False
    if execCycle <= 0.0001 or execCycle >= 100.0:
      self.ShowStatus("Execution Cycle: field out of range "
          "[%-.4f, %-.1f]: '%-.2f'" % \
          (0.0001, 100.0, execCycle),
          fg=gt.ColorFgStatusError)
      self.mEntryExecCycle.focus_set()
      return False

    try:
      execStepSize = self.mVarExecStepSize.get()
    except ValueError, err:
      self.ShowStatus("Step Size: field has invalid value: %s" % \
          repr(self.mEntryExecStepSize.get()),
          fg=gt.ColorFgStatusError)
      self.mEntryExecStepSize.focus_set()
      return False
    if execStepSize <= 0.0001 or execStepSize >= 100.0:
      self.ShowStatus("Step Size: field out of range "
          "[%-.4f, %-.1f]: '%-.2f'" % \
          (0.0001, 100.0, execStepSize),
          fg=gt.ColorFgStatusError)
      self.mEntryExecStepSize.focus_set()
      return False

    autoConnect   = self.mVarAutoConnect.get()
    if autoConnect == 1:
      autoConnect = True
    else:
      autoConnect = False

    self.result = {'ExecCycle':execCycle, 'ExecStepSize':execStepSize,
                   'AutoConnect':autoConnect}

    for data in self.mSensorGroups:
      val = data[2].get()
      if val == 1:
        val = True
      else:
        val = False
      self.result[data[0]] = val

    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def ShowStatus(self, text='', fg=gt.ColorFgStatusOk):
    """Show connection dialog status."""
    self.mEntryStatus['state'] = tk.NORMAL
    self.mEntryStatus['fg'] = fg
    self.mEntryStatus.delete(0, tk.END)
    self.mEntryStatus.insert(0, text)
    self.mEntryStatus['state'] = 'readonly'


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgHemiOpt Test Main """
    root = tk.Tk()
    dlg = GuiDlgHemiOpt(root)
    if dlg.result:
      print 'ok:', dlg.result
    else:
      print 'cancel'

  # run test
  main()
