################################################################################
#
# GuiDlgDebug.py
#

""" Graphical User Interface Debug Dialog Module

Graphical User Interface (GUI) Tkinter debug dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.11

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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
import  Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import  Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['DebugLevelFusion', 'DebugLevelRobot', 'DebugLevelBrain',
          'DebugFileName']

#-------------------------------------------------------------------------------
# CLASS: GuiDlgDebug
#-------------------------------------------------------------------------------
class GuiDlgDebug(tkSimpleDialog.Dialog):
  """ Debug Dialog Class

      The result on dialog exit:
        On ok success, returns dictionary of debug settings
            {'DebugLevelFusion':debuglevel, 'DebugLevelRobot':debuglevel,
             'DebugLevelBrain':debuglevel, 'DebugFileName':filename}
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
    elif key == 'DebugFileName':
      return '<stdout>'
    elif key == 'DebugLevelFusion' or \
         key == 'DebugLevelRobot' or \
         key == 'DebugLevelBrain':
      return 0  # off
    else:
      return None

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('Debug Settings')

    font = tkFont.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Debug Level Select Frame
    row +=1 
    self._bodyDebugLevelFrame(master, row, column)

    # Debug Output File Select Frame
    row += 1
    self._bodyDebugFileFrame(master, row, column)

    row += 1
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select debug settings.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyDebugLevelFrame(self, master, row, column):
    """Create Debug Level subdialog frame."""
    self.mVarDLFusion = tk.IntVar()
    self.mVarDLFusion.set(-1)
    self.mVarDLRobot  = tk.IntVar()
    self.mVarDLRobot.set(-1)
    self.mVarDLBrain  = tk.IntVar()
    self.mVarDLBrain.set(-1)

    dlframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    dlframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    GuiToolTip.GuiToolTip(dlframe,
        text='Select debug levels for Fusion, vRobot, and vBrain.')

    row = 0
    column = 0
    w = tk.Label(dlframe, text='Debug Levels:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

    for dtxt, dobj, dvar in [('Fusion', 'Fusion', self.mVarDLFusion),
                             ('vBrain', 'Brain',  self.mVarDLBrain),
                             ('vRobot', 'Robot',  self.mVarDLRobot)]:
      row += 1
      column = 0
      w = tk.Label(dlframe, text='%s:' % dtxt)
      w.grid(row=row, column=0, sticky=tk.E)
      dl = 0
      for dllabel in ['off', '1', '2', '3', '4', '5']:
        column += 1
        b = tk.Radiobutton(dlframe,
                           text=dllabel,
                           variable=dvar,
                           value=dl)
        b.grid(row=row, column=column, sticky=tk.W, padx=3)
        if dl == self._lastSetting('DebugLevel'+dobj):
          b.select()
          dvar.set(dl)
        dl += 1

  #--
  def _bodyDebugFileFrame(self, master, row, column):
    """Create Debug Output File subdialog frame."""
    self.mVarDebugFile  = tk.StringVar()
    self.mVarStdFile    = tk.StringVar()

    lastFileName = self._lastSetting('DebugFileName')
    
    dfframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    dfframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    
    row = 0
    column = 0
    w = tk.Label(dfframe, text='Debug Output File:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    column = 0
    b = tk.Radiobutton(dfframe, text='<stdout>',
                                command=self.CbStdOut,
                                variable=self.mVarStdFile,
                                value='<stdout>')
    b.grid(row=row, column=column, sticky=tk.E, padx=3)
    if lastFileName == '<stdout>':
      b.select()
    GuiToolTip.GuiToolTip(b, text='Send debug information to standard output.')

    column += 1
    b = tk.Radiobutton(dfframe, text='<stderr>',
                                command=self.CbStdErr,
                                variable=self.mVarStdFile,
                                value='<stderr>')
    b.grid(row=row, column=column, sticky=tk.E, padx=3)
    if lastFileName == '<stderr>':
      b.select()
    GuiToolTip.GuiToolTip(b, text='Send debug information to standard error.')

    row += 1
    column = 0
    self.mEntryDbgFileName = tk.Entry(dfframe, width=35,
                                      textvariable=self.mVarDebugFile) 
    self.mEntryDbgFileName.grid(row=row, column=column, columnspan=2,
        sticky=tk.N+tk.S+tk.W+tk.E)
    GuiToolTip.GuiToolTip(self.mEntryDbgFileName,
        text='Display and/or specify debug ouput file.')

    column += 2
    b = tk.Button(dfframe, text='Browse...', command=self.CbBrowse)
    b.grid(row=row, column=column, sticky=tk.W)
    GuiToolTip.GuiToolTip(b,
        text='Browse file system for debug output file.')

    self._setDebugFileName(lastFileName)

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

    dlFusion  = self.mVarDLFusion.get()
    dlRobot   = self.mVarDLRobot.get()
    dlBrain   = self.mVarDLBrain.get()
    debugfile = self.mVarDebugFile.get()

    if dlFusion == -1:
      self.ShowStatus("No Fusion debug level specified.",
          fg=gt.ColorFgStatusError)
      return False
    if dlRobot == -1:
      self.ShowStatus("No vRobot debug level specified.",
          fg=gt.ColorFgStatusError)
      return False
    if dlBrain == -1:
      self.ShowStatus("No vBrain debug level specified.",
          fg=gt.ColorFgStatusError)
      return False
    if not debugfile:
      self.ShowStatus("No debug output file specified.",
          fg=gt.ColorFgStatusError)
      return False

    self.result = {'DebugLevelFusion':dlFusion, 'DebugLevelRobot':dlRobot,
                   'DebugLevelBrain':dlBrain, 'DebugFileName':debugfile}
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

  #--
  def CbBrowse(self): 
    """Browse callback. """
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self,
                  title='Debug Output File',
                  filetypes=[('Text files', '*.txt', 'TEXT'),
                             ('Text debug files', '*.dbg', 'TEXT'),
                             ('All files', '*')],
                  defaultextension='.txt')
    if dlg.result:
      if dlg.result['status'] == 'notsaved':
        self._setDebugFileName(dlg.result['filename'])
      elif dlg.result['status'] == 'error':
        pass

  #--
  def CbStdOut(self): 
    self._setDebugFileName('<stdout>')

  #--
  def CbStdErr(self): 
    self._setDebugFileName('<stderr>')

  #--
  def _setDebugFileName(self, fileName):
    """ Set debug file name ."""
    self.mEntryDbgFileName.delete(0, tk.END)
    self.mEntryDbgFileName.insert(0, fileName)


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgDebug Test Main """
    root = tk.Tk()
    dlg = GuiDlgDebug(root)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
