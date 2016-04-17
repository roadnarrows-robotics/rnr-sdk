################################################################################
#
# GuiDlgDynaOpt.py
#

""" Graphical User Interface vBrainDyna<x> Options Dialog Module

Standard Graphical User Interface (GUI) Tkinter options dialog module
for Khepera brains using a non-linear dynamic approach (attractor
dynamics).

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.02.06

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
import tkSimpleDialog
import tkFont
import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#
# Field Parameters (keep as list to preserve order)
#
_DynaOptFieldGroup = [
  { 'key':      'dpsi',
    'label':    'd' + gt.UniGreek['psi'],
    'tooltip':  'dynamics delta sampling size (degrees)',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_p',
    'label':    'k_p',
    'tooltip':  'interaction kernel excitatory constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_n',
    'label':    'k_n',
    'tooltip':  'interaction kernel inhibitory constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'l_coop',
    'label':    'l_coop',
    'tooltip':  'interaction kernel cooperativity length (degrees)',
    'load':     math.degrees,
    'store':    math.radians,
  },
  { 'key':      'k_tau',
    'label':    'k_' + gt.UniGreek['tau'],
    'tooltip':  'execution cycle time multiplier',
    'load':     float,
    'store':    float,
  },
  { 'key':      'h_min',
    'label':    'h_min',
    'tooltip':  'target field activation limit constant for the monostable '
                'regime (destabilizing memory)',
    'load':     float,
    'store':    float,
  },
  { 'key':      'h_max',
    'label':    'h_max',
    'tooltip':  'target field activation limit constant for the bistable regime'
                ' (enabling memory)',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_r_h_min',
    'label':    'k_r_h,min',
    'tooltip':  'target field activation destabilizing rate constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_r_h_max',
    'label':    'k_r_h,max',
    'tooltip':  'target field activation stabilizing rate constant',
    'load':     float,
    'store':    float,
  },
]

#
# Target Acquisition Parameters (keep as list to preserve order)
#
_DynaOptTgtGroup = [
  { 'key':      'TgtSensorAngRange',
    'label':    'angrange',
    'tooltip':  'simulated target sensor(s) angular range (degrees)',
    'load':     math.degrees,
    'store':    math.radians,
  },
  { 'key':      'M0',
    'label':    'M0',
    'tooltip':  'zero set point target sensory bias constant', 
    'load':     float,
    'store':    float,
  },
  { 'key':      'sigma',
    'label':    gt.UniGreek['sigma'],
    'tooltip': "Gausian kernel width " + gt.UniGreek['sigma'] + ' (degrees) ' +
               'convolved with the discrete target sensory input',
    'load':     math.degrees,
    'store':    math.radians,
  },
  { 'key':      'k_lambda_tar_p',
    'label':    'k_' + gt.UniGreek['lambda'] + "'_tar",
    'tooltip':  'target attractive force-let constant',
    'load':     float,
    'store':    float,
  },
]

#
# Obstacle Avoidance Parameters (keep as list to preserve order)
#
_DynaOptObsGroup = [
  { 'key':      'k_beta_1',
    'label':    'k_' + gt.UniGreek['beta'] + '1',
    'tooltip':  'maximal strength of obstacle repulsive force-let',
    'load':     float,
    'store':    float,
  },
  { 'key':      'beta_2',
    'label':    gt.UniGreek['beta'] + '2',
    'tooltip':  'decay distance constant of obstacle repulsive force-let',
    'load':     float,
    'store':    float,
  },
]

#
# Velocity Control Parameters (keep as list to preserve order)
#
_DynaOptVelGroup = [
  { 'key':      'c',
    'label':    'c',
    'tooltip':  'velocity control sigmoid constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_c_v_tar',
    'label':    'k_c_v,tar',
    'tooltip':  'target velocity relaxation rate constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'k_c_v_obs',
    'label':    'k_c_v,obs',
    'tooltip':  'obstacle velocity relaxation rate constant',
    'load':     float,
    'store':    float,
  },
  { 'key':      'psi_hat_max',
    'label':    gt.UniGreek['psi'] + '^_max',
    'tooltip':  'maximum robot speed (mm/s)',
    'load':     float,
    'store':    float,
  },
]

#
# Execution Parameters (keep as list to preserve order)
#
_DynaOptExecGroup = [
  { 'key':      'ExecCycle',
    'label':    'Execution Cycle',
    'tooltip':  'Dynamic brain execution think/act cycle period (seconds)',
    'load':     float,
    'store':    float,
  },
  { 'key':      'ExecStepSize',
    'label':    'Step Size',
    'tooltip':  "Size (seconds) per 'Step'",
    'load':     float,
    'store':    float,
  },
]

#
# The Groups of Options
#
_DynaOptGroups = [
  { 'group':    _DynaOptFieldGroup,
    'heading':  'Field Parameters',
    'row':      0,
    'column':   0,
    'rspan':    2,
    'cspan':    1,
  },
  { 'group':    _DynaOptTgtGroup,
    'heading':  'Target Parameters',
    'row':      0,
    'column':   1,
    'rspan':    1,
    'cspan':    1,
  },
  { 'group':    _DynaOptObsGroup,
    'heading':  'Obstacle Parameters',
    'row':      1,
    'column':   1,
    'rspan':    1,
    'cspan':    1,
  },
  { 'group':    _DynaOptVelGroup,
    'heading':  'Velocity Control',
    'row':      0,
    'column':   2,
    'rspan':    1,
    'cspan':    1,
  },
  { 'group':    _DynaOptExecGroup,
    'heading':  'Execution Control Options',
    'row':      1,
    'column':   2,
    'rspan':    1,
    'cspan':    1,
  },
]

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  keyList = []
  for group in _DynaOptGroups:
    for field in group['group']:
      keyList.append(field['key'])
  return keyList


#-------------------------------------------------------------------------------
# CLASS: GuiDlgDynaOpt
#-------------------------------------------------------------------------------
class GuiDlgDynaOpt(tkSimpleDialog.Dialog):
  """ vBrainDyna<x> Run-Time Options Dialog Class

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
      self.mTitle = 'Attactor Dynamics Design Options'

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if self.mLastSettings.has_key(key) and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    else:
      return self.mDftSettings[key]

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    font = tkFont.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Option groups
    maxrow=0
    maxcol=0
    for group in _DynaOptGroups:
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
    self.ShowStatus('Dynamic parameter tuning.', fg=gt.ColorFgStatusPrompt)

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
      field['var'] = tk.DoubleVar()
      w = tk.Label(gframe, text=field['label']+':', fg=gt.ColorBlack)
      w.grid(row=row, column=0, sticky=tk.E, padx=3)
      w = tk.Entry(gframe, textvariable=field['var'], width=12)
      w.grid(row=row, column=1, sticky=tk.W)
      GuiToolTip.GuiToolTip(w, field['tooltip'])
      val = self._lastSetting(field['key'])
      val = field['load'](val)
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

    for group in _DynaOptGroups:
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
      except (ValueError, TypeError), msg:
        self.ShowStatus('Field %s: %s' % (repr(field['label']), msg), 
            fg=gt.ColorFgStatusError)
        return False
      try:
        val = field['store'](val)
      except (ValueError, TypeError), msg:
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
    for group in _DynaOptGroups:
      for field in group['group']:
        val = self.mDftSettings[field['key']]
        val = field['load'](val)
        field['var'].set(val)

#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  from Fusion.Acc.DynaTgt import DynaDesignParamDfts
  def main():
    """ GuiDlgDynaOpt Test Main """
    root = tk.Tk()
    dlg = GuiDlgDynaOpt(root, DynaDesignParamDfts)
    if dlg.result:
      print 'ok:', dlg.result
    else:
      print 'cancel'

  # run test
  main()
