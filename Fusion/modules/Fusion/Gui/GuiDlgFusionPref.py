################################################################################
#
# GuiDlgFusionPref.py
#

""" Graphical User Interface Fusion Preferences Dialog Module

Graphical User Interface (GUI) Tkinter Fusion preferneces dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.01

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

import  tkinter as tk
import  tkinter.simpledialog
import  tkinter.font

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiToolTip as GuiToolTip

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['AutoSave', 'AutoSavePlugins', 'AutoPlugin']

#-------------------------------------------------------------------------------
# CLASS: GuiDlgFusionPref
#-------------------------------------------------------------------------------
class GuiDlgFusionPref(tkinter.simpledialog.Dialog):
  """ Fusion Preferences Dialog Class

      The result on dialog exit:
        On ok success, returns dictionary of preferences settings
          {'AutoSave':autoSave, 'AutoSavePlugins':autoSavePlugins,
           'AutoPlugin':autoPlugin}
        On cancel, returns None
    """

  #--
  def __init__(self, guiParent, lastSettings={}):
    """ Initialize the Fusion Preferences Dialog.

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
    elif key == 'AutoPlugin':
      return False
    else:
      return True

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('Fusion Preferences')

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Ini Control Frame
    self._bodyIniFrame(master, row, column)

    # Plugin Control Frame
    column += 1
    self._bodyPluginFrame(master, row, column)

    # status bar
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=2, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select preferences settings.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyIniFrame(self, master, row, column):
    """Create Ini Control subdialog frame."""
    self.mVarAutoSave = tk.IntVar()

    if self._lastSetting('AutoSave'):
      self.mVarAutoSave.set(1)
    else:
      self.mVarAutoSave.set(0)

    iniframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    iniframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E+tk.S)

    row = 0
    column = 0
    w = tk.Label(iniframe, text='Ini Configuration:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

    row += 1
    b = tk.Checkbutton(iniframe,
                       text='AutoSave',
                       variable=self.mVarAutoSave)
    b.grid(row=row, column=column, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(b,
        text="Enable to autosave ini configuration on exit")

    # spacer
    row += 1
    w = tk.Label(iniframe, text=' ', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

  #--
  def _bodyPluginFrame(self, master, row, column):
    """Create Plugin Control subdialog frame."""
    self.mVarAutoSavePlugins  = tk.IntVar()
    self.mVarAutoPlugin       = tk.IntVar()

    if self._lastSetting('AutoSavePlugins'):
      self.mVarAutoSavePlugins.set(1)
    else:
      self.mVarAutoSavePlugins.set(0)

    if self._lastSetting('AutoPlugin'):
      self.mVarAutoPlugin.set(1)
    else:
      self.mVarAutoPlugin.set(0)

    plugframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    plugframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    row = 0
    column = 0
    w = tk.Label(plugframe, text='Plugin Configuration:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

    row += 1
    b = tk.Checkbutton(plugframe,
                       text='AutoSavePlugins',
                       variable=self.mVarAutoSavePlugins)
    b.grid(row=row, column=column, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(b,
        text="Enable to autosave session's vBrain and vRobot plugins to"
              "'BrainPlugin' and 'RobotPlugin'")

    row += 1
    b = tk.Checkbutton(plugframe,
                       text='AutoPlugin',
                       variable=self.mVarAutoPlugin)
    b.grid(row=row, column=column, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(b,
        text="Enable to auto-plugin the ini specfied "
              "'BrainPlugin' and 'RobotPlugin' plugins")

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

  def validate(self):
    """Validate dialog settings."""

    if self.mVarAutoSave.get():
      autoSave = True
    else:
      autoSave = False
    if self.mVarAutoSavePlugins.get():
      autoSavePlugins = True
    else:
      autoSavePlugins = False
    if self.mVarAutoPlugin.get():
      autoPlugin = True
    else:
      autoPlugin = False

    self.result = {'AutoSave':autoSave, 'AutoSavePlugins':autoSavePlugins,
                   'AutoPlugin':autoPlugin}

    return True

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
    """ GuiDlgFusionPref Test Main """
    root = tk.Tk()
    dlg = GuiDlgFusionPref(root)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
