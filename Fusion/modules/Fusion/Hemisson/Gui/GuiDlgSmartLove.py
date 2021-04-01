################################################################################
#
# GuiDlgSmartLove.py
#

""" Graphical User Interface vBrainyBerg Options Dialog Module

Standard Graphical User Interface (GUI) Tkinter options dialog module
for Hemisson brains using the Braitenburg brain base and the SmartLove 
brain implant.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2008.06.01

Copyright (C) 2008.  RoadNarrows LLC.
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
import tkinter.simpledialog
import tkinter.font
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Field Parameters (keep as list to preserve order)
#
_SmartLoveOptBiasGroup = [
  { 'key':      'bias_left',
    'label':    'Left Motor',
    'type':     'slider',
    'tooltip':  "Left motor's bias.",
    'load':     int,
    'store':    int,
    'min':      0,
    'max':      9,
    'res':      1
  },
  { 'key':      'bias_right',
    'label':    "Right MotorBias",
    'type':     'slider',
    'tooltip':  "Right motor's bias.",
    'load':     int,
    'store':    int,
    'min':      0,
    'max':      9,
    'res':      1
  },
]

#
# NN Parameters (keep as list to preserve order)
#
_SmartLoveOptNNGroup = [
  { 'key':      'nn_vis_threshold',
    'label':    'Pixel Noise Threshold',
    'type':     'slider',
    'tooltip':  'Camera pixels noise floor threshold.',
    'load':     int,
    'store':    int,
    'min':      0,
    'max':      255,
    'res':      1
  },
  { 'key':      'nn_vis_scaler',
    'label':    'A x ' + gt.UniGreek['Sigma'] + 'out_optic_i',
    'type':     'entry',
    'tooltip':  'Scalar A multiplier of sum of output function of all '
                'of the vision neurons on a given side.',
    'load':     float,
    'store':    float,
  },
  { 'key':      'nn_mem_scaler',
    'label':    'B x ' + gt.UniGreek['Sigma'] + 'out_memory_i',
    'type':     'entry',
    'tooltip':  'Scalar B multiplier of sum of output function of all '
                'of the memory neurons on a given side.',
    'load':     float,
    'store':    float,
  },
  { 'key':      'nn_mem_num',
    'label':    'Number of Memory Neurons',
    'type':     'slider',
    'tooltip':  "Number of memory neurons. "
                "Half for the left side, half for the right.",
    'load':     int,
    'store':    int,
    'min':      2,
    'max':      32,
    'res':      2
  },
]

#
# Execution Parameters (keep as list to preserve order)
#
_SmartLoveOptExecGroup = [
  { 'key':      'ExecCycle',
    'label':    'Execution Cycle',
    'type':     'entry',
    'tooltip':  'Brain execution think/act cycle period (seconds)',
    'load':     float,
    'store':    float,
  },
  { 'key':      'ExecStepSize',
    'label':    'Step Size',
    'type':     'entry',
    'tooltip':  "Size (seconds) per 'Step'",
    'load':     float,
    'store':    float,
  },
]

#
# The Groups of Options
#
_SmartLoveOptGroups = [
  { 'group':    _SmartLoveOptBiasGroup,
    'heading':  'Motor Bias Values',
    'row':      0,
    'column':   0,
    'rspan':    1,
    'cspan':    1,
  },
  { 'group':    _SmartLoveOptNNGroup,
    'heading':  'Neural Network Configuration',
    'row':      0,
    'column':   1,
    'rspan':    1,
    'cspan':    1,
  },
  { 'group':    _SmartLoveOptExecGroup,
    'heading':  'Execution Control Options',
    'row':      0,
    'column':   2,
    'rspan':    1,
    'cspan':    1,
  },
]

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  keyList = []
  for group in _SmartLoveOptGroups:
    for field in group['group']:
      keyList.append(field['key'])
  return keyList


#-------------------------------------------------------------------------------
# CLASS: GuiDlgSmartLove
#-------------------------------------------------------------------------------
class GuiDlgSmartLove(tkinter.simpledialog.Dialog):
  """ vBrainyBerg Run-Time Options Dialog Class

      The result on dialog exit:
        On ok success, result dictionary of option values.
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, dftSettings, lastSettings={}, title=None):
    """ Initialize the Debug Dialog.

        Parameters:
          guiParent     - this dialog's parent GUI object
          lastSettings  - settings of last configurations.
                          See result.
    """
    self.result         = None
    self.mDftSettings   = dftSettings
    self.mLastSettings  = lastSettings
    if title:
      self.mTitle = title
    else:
      self.mTitle = 'Attactor OpticFlowmics Design Options'

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if key in self.mLastSettings and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    else:
      return self.mDftSettings[key]

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Option groups
    maxrow=0
    maxcol=0
    for group in _SmartLoveOptGroups:
      self._bodyGroupFrame(master, 
          group['row'], 
          group['column'],
          group['rspan'],
          group['cspan'],
          group['heading'], 
          group['group'])
      if group['row'] > maxrow:
        maxrow = group['row']
      if group['column'] > maxcol:
        maxcol = group['column']

    # Status bar
    row = maxrow + 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=maxcol+1, 
        padx=3, pady=5, sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Optic Flow parameter tuning.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyGroupFrame(self, master, row, column, rowspan, columnspan,
                            heading, group):
    """Create field group subdialog frame."""
    gframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    gframe.grid(row=row, column=column, rowspan=rowspan, columnspan=columnspan, 
        padx=3, pady=3, ipadx=1, ipady=1, sticky=tk.N+tk.W+tk.S+tk.E)

    row = 0
    column = 0
    w = tk.Label(gframe, text=heading+':', fg=gt.ColorBlue, anchor=tk.N)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    for field in group:
      row += 1
      if field['type'] == 'entry':
        field['var'] = tk.DoubleVar()
        w = tk.Label(gframe, text=field['label']+':', fg=gt.ColorBlack)
        w.grid(row=row, column=0, sticky=tk.E, padx=3)
        w = tk.Entry(gframe, textvariable=field['var'], width=12)
        w.grid(row=row, column=1, sticky=tk.W)
        GuiToolTip.GuiToolTip(w, field['tooltip'])
        val = self._lastSetting(field['key'])
        val = field['load'](val)
        field['var'].set(val)
      elif field['type'] == 'slider':
        w = tk.Scale(gframe, width=10, length=250, orient=tk.HORIZONTAL,
                      label=field['label'],
                      from_=field['min'],
                      to=field['max'],
                      resolution=field['res'])
        w.grid(row=row, column=column, sticky=tk.W)
        GuiToolTip.GuiToolTip(w, field['tooltip'])
        val = self._lastSetting(field['key'])
        val = field['load'](val)
        field['var'] = w
        field['var'].set(val)

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

    for group in _SmartLoveOptGroups:
      if not self._validateGroup(group['group']):
        result = None
        return False

    return True

  #--
  def _validateGroup(self, group):
    """ Validate dialog group settings. """
    for field in group:
      try:
        val = field['var'].get()
      except (ValueError, TypeError) as msg:
        self.ShowStatus('Field %s: %s' % (repr(field['label']), msg), 
            fg=gt.ColorFgStatusError)
        return False
      try:
        val = field['store'](val)
      except (ValueError, TypeError) as msg:
        self.ShowStatus('Field %s: %s' % (repr(field['label']), msg), 
            fg=gt.ColorFgStatusError)
        return False
      self.result[field['key']] = val
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

  #--
  def buttonbox(self):
    """ Add new buttons overriding the standard buttons. """
    box = tk.Frame(self)

    w = tk.Button(box, text="OK", width=10, command=self.ok,
        default=tk.ACTIVE)
    w.pack(side=tk.LEFT, padx=5, pady=5)

    w = tk.Button(box, text="Cancel", width=10, command=self.cancel)
    w.pack(side=tk.LEFT, padx=5, pady=5)

    w = tk.Button(box, text="Defaults", width=10, command=self.CbDefaults) 
    w.pack(side=tk.LEFT, padx=5, pady=5)

    self.bind("<Return>", self.ok)
    box.pack()

  #--
  def CbDefaults(self, event=None):
    for group in _SmartLoveOptGroups:
      for field in group['group']:
        val = self.mDftSettings[field['key']]
        val = field['load'](val)
        field['var'].set(val)


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  #--
  def main():
    """ GuiDlgSmartLove Test Main """
    root = tk.Tk()
    dfts = {
      'bias_left':          9,
      'bias_right':         9,
      'nn_vis_threshold':   1,
      'nn_vis_scaler':      1.0,
      'nn_mem_scaler':      1.0,
      'nn_mem_num':         4,
      'ExecCycle':          0.1,
      'ExecStepSize':       1.0,
    }

    dlg = GuiDlgSmartLove(root, dfts)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
