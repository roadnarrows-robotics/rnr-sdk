################################################################################
#
# GuiDlgKHR1Opt.py
#

""" Graphical User Interface vKHR1sson Options Dialog Module

Graphical User Interface (GUI) Tkinter vKHR1sson options dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.01.07

Copyright (C) 2007.  RoadNarrows LLC.
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
import  Fusion.Gui.GuiToolTip as GuiToolTip
import  Fusion.Gui.GuiTypes as gt

import Fusion.KHR1.Cmd.KHR1CmdBase as KHR1CmdBase


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['board_ids', 'servo_version',
          'ExecCycle', 'ExecStepSize']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgKHR1Opt
#-------------------------------------------------------------------------------
class GuiDlgKHR1Opt(tkinter.simpledialog.Dialog):
  """ vKHR1 Run-Time Options Dialog Class

      The result on dialog exit:
        On ok success, result dictionary:
          {'board_ids': <listOfBoardIds>,
           'servo_version':{'red' | 'blue'}, 
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

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    if key in self.mLastSettings and self.mLastSettings[key] is not None:
      return self.mLastSettings[key]
    elif key == 'ExecCycle' or key == 'ExecStepSize':
      return 0.10
    else:
      return False

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('vKHR1 Options')

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # KHR-1 Select Frame
    row +=1 
    self._bodyKHR1Frame(master, row, column)

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
    self.ShowStatus('Select vKHR1 options.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyKHR1Frame(self, master, row, column):
    """Create KHR1 subdialog frame."""
    subframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    row = 0
    column = 0
    w = tk.Label(subframe, text='KHR-1 Robot Options:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=3, sticky=tk.NW)

    bidList = self._lastSetting('board_ids')
    servoVer  = self._lastSetting('servo_version')

    self.mVarBid0     = tk.IntVar()
    self.mVarBid1     = tk.IntVar()
    self.mVarServoVer = tk.StringVar()

    row += 1
    column = 0
    w = tk.Label(subframe, text='First RCB-1 ID:')
    w.grid(row=row, column=column, sticky=tk.E)

    column += 1
    w = tk.Entry(subframe, width=2, textvariable=self.mVarBid0) 
    w.grid(row=row, column=column, sticky=tk.W)
    w.delete(0, tk.END)
    if len(bidList) > 0:
      self.mVarBid0.set(bidList[0])
    GuiToolTip.GuiToolTip(w,
        text='First Board ID. Much match RCB-1 assigned configuration.')
    self.mEntryBid0 = w

    row += 1
    column = 0
    w = tk.Label(subframe, text='Second RCB-1 ID:')
    w.grid(row=row, column=column, sticky=tk.E)

    column += 1
    w = tk.Entry(subframe, width=2, textvariable=self.mVarBid1) 
    w.grid(row=row, column=column, sticky=tk.W)
    w.delete(0, tk.END)
    if len(bidList) > 1:
      self.mVarBid1.set(bidList[1])
    GuiToolTip.GuiToolTip(w,
        text='Second Board ID. Much match RCB-1 assigned configuration.')
    self.mEntryBid1 = w

    row += 1
    column = 0
    w = tk.Label(subframe, text='Servo Version:')
    w.grid(row=row, column=column, sticky=tk.E)

    column += 1
    w = tk.Label(subframe, width=5, relief=tk.SUNKEN,
        textvariable=self.mVarServoVer) 
    w.grid(row=row, column=column, sticky=tk.W)
    GuiToolTip.GuiToolTip(w, text='Kondo Servo Version')
    self.mLabelServoVer = w

    column += 1
    mb = tk.Menubutton(subframe, text='v', relief=tk.RAISED)
    mb.grid(row=row, column=column, stick=tk.W)
    GuiToolTip.GuiToolTip(mb, text='Select servo version.')
    mb.menu = tk.Menu(mb, tearoff=0)
    for ver, cmd in [
        (KHR1CmdBase.RCB1ServoVersionBlue, self.CbServoVerBlue),
        (KHR1CmdBase.RCB1ServoVersionRed, self.CbServoVerRed)
      ]:
      mb.menu.add_command(label=ver, command=cmd)
      if ver == servoVer:
        cmd()
    mb.config(menu=mb.menu)

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
    except ValueError as err:
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
    except ValueError as err:
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

    try:
      bid0 = self.mVarBid0.get()
    except ValueError as err:
      self.ShowStatus("First Board ID: field has invalid value: %s" % \
          repr(self.mEntryBid0.get()),
          fg=gt.ColorFgStatusError)
      self.mEntryBid0.focus_set()
      return False
    if bid0 < KHR1CmdBase.RCB1BoardIdMin or bid0 > KHR1CmdBase.RCB1BoardIdMax:
      self.ShowStatus("First Board ID: field out of range [%d, %d]: '%d'" % \
          (KHR1CmdBase.RCB1BoardIdMin, KHR1CmdBase.RCB1BoardIdMax, bid0),
          fg=gt.ColorFgStatusError)
      self.mEntryBid0.focus_set()
      return False

    if self.mEntryBid1.get() == '':
      bid1 = -1
    else:
      try:
        bid1 = self.mVarBid1.get()
      except ValueError as err:
        self.ShowStatus("Second Board ID: field has invalid value: %s" % \
          repr(self.mEntryBid1.get()),
          fg=gt.ColorFgStatusError)
        self.mEntryBid1.focus_set()
        return False
      if bid0 < KHR1CmdBase.RCB1BoardIdMin or bid0 > KHR1CmdBase.RCB1BoardIdMax:
        self.ShowStatus("Second Board ID: field out of range [%d, %d]: '%d'" % \
          (KHR1CmdBase.RCB1BoardIdMin, KHR1CmdBase.RCB1BoardIdMax, bid0),
          fg=gt.ColorFgStatusError)
        self.mEntryBid1.focus_set()
        return False

    bidList = []

    bidList += [bid0]
    if bid1 >= KHR1CmdBase.RCB1BoardIdMin:
      bidList += [bid1]

    self.result = {'ExecCycle':execCycle, 'ExecStepSize':execStepSize,
        'AutoConnect':autoConnect, 'board_ids': bidList,
        'servo_version': self.mVarServoVer.get()}

    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def CbServoVerBlue(self):
    self.mLabelServoVer['fg'] = gt.ColorBlue1
    self.mVarServoVer.set(KHR1CmdBase.RCB1ServoVersionBlue)

  #--
  def CbServoVerRed(self):
    self.mLabelServoVer['fg'] = gt.ColorRed1
    self.mVarServoVer.set(KHR1CmdBase.RCB1ServoVersionRed)

  #--
  def ShowStatus(self, text='', fg=gt.ColorFgStatusOk):
    """Show connection dialog status."""
    self.mEntryStatus['state'] = tk.NORMAL
    self.mEntryStatus['fg'] = fg
    self.mEntryStatus.delete(0, tk.END)
    self.mEntryStatus.insert(0, text)
    self.mEntryStatus['state'] = 'readonly'


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  #--
  def main():
    """ GuiDlgHemiOpt Unit Test Main """
    root = tk.Tk()
    dlg = GuiDlgKHR1Opt(root, 
        lastSettings={'board_ids':[0, 1], 'servo_version':'blue'})
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run unit test
  main()
