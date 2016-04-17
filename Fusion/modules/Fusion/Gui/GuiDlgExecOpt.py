################################################################################
#
# GuiDlgExecOpt.py
#

""" Graphical User Interface Simple Execution Options Dialog Module

Graphical User Interface (GUI) Tkinter simple execution options dialog
module. It can be used to define execution cycle and step size options

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.06

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
  return ['ExecCycle', 'ExecStepSize']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgExecOpt
#-------------------------------------------------------------------------------
class GuiDlgExecOpt(tkSimpleDialog.Dialog):
  """ Simple dialog to set options for execution cycle time and 'Step' size.

      The result on dialog exit:
        On ok success, result dictionary:
           {'ExecCycle':seconds, 'ExecStepSize':seconds}
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, lastSettings={}, title='Execution Options'):
    """ Initialize the Dialog.

        Parameters:
          guiParent     - this dialog's parent GUI object
          lastSettings  - settings of last configurations.
                          See Return Value.
          title         - title of this dialog
    """
    self.result         = None
    self.mLastSettings  = lastSettings
    self.mTitle         = title

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if self.mLastSettings.has_key(key) and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    elif key == 'ExecCycle':
      return 0.10
    elif key == 'ExecStepSize':
      return 1.0
    else:
      return None

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    font = tkFont.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Execution Select Frame
    self._bodyExecFrame(master, row, column)

    # Status bar
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font, width=30,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select execution options.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

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
        text='Core execution sense/act cycle period (seconds)')

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

    self.result = {'ExecCycle':execCycle, 'ExecStepSize':execStepSize}

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
    """ GuiDlgExecOpt Test Main """
    root = tk.Tk()
    dlg = GuiDlgExecOpt(root)
    if dlg.result:
      print 'ok:', dlg.result
    else:
      print 'cancel'

  # run test
  main()
