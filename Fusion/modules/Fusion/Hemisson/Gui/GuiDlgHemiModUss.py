################################################################################
#
# GuiDlgHemiModUss.py
#

""" Graphical User Interface Hemisson UltraSonic Sensor Dialog Module

Graphical User Interface (GUI) Tkinter Hemisson Linear Cameera Module
options dialog module.

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

import tkinter as tk
import tkinter.simpledialog
import tkinter.font

import Fusion.Gui.GuiToolTip as GuiToolTip
import Fusion.Gui.GuiTypes as gt

import Fusion.Hemisson.Cmd.HemiCmdUss as HemiUss

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['range']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgHemiModUss
#-------------------------------------------------------------------------------
class GuiDlgHemiModUss(tkinter.simpledialog.Dialog):
  """ Hemisson UltraSonic Sensor Module Dialog Class.

      The result on dialog exit:
        On ok success, result dictionary:
           {'range':meters}
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, lastSettings={}):
    """ Initialize the Dialog.

        Parameters:
          guiParent     - this dialog's parent GUI object
          lastSettings  - settings of last configurations.
                          See Return Value.
    """
    self.result         = None
    self.mLastSettings  = lastSettings

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if key in self.mLastSettings and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    else:
      return 0

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('vHemisson UltraSonic Sensor Module Settings Dialog')

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Settings Frame
    self._bodySetFrame(master, row, column)

    # Status bar
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font, width=30,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select settings.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodySetFrame(self, master, row, column):
    """Create Exec subdialog frame."""
    setframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    setframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    
    row = 0
    column = 0
    w = tk.Label(setframe, text='UltraSonic Sensor Settings:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    w = tk.Scale(setframe, width=10, length=250,
          label='Maximum Range',
          from_=int(HemiUss.UssResolutionMin),
          to=6000,
          resolution=int(HemiUss.UssResolutionMin),
          orient=tk.HORIZONTAL)
    w.grid(row=row, column=column, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text="UltraSonic Sensor maximum range (mm)")
    self.mSliderRange = w
    self.mSliderRange.set(self._lastSetting('range'))

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
    self.result = {
      'range': self.mSliderRange.get()
    }

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
    """ GuiDlgHemiModUss Test Main """
    root = tk.Tk()
    lastsettings = {'range': 6000}
    dlg = GuiDlgHemiModUss(root, lastSettings=lastsettings)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
