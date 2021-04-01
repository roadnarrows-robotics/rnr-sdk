################################################################################
#
# GuiDlgKHR1ServoChan.py
#

""" Graphical User Interface KHR-1 Actiive Servo Channel Dialog.

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

import tkinter as tk
import tkinter.font
import tkinter.simpledialog

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiToolTip as GuiToolTip

import Fusion.KHR1.Cmd.KHR1CmdBase as KHR1CmdBase


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['active_channels']


#-------------------------------------------------------------------------------
# CLASS: GuiDlgKHR1ServoChan
#-------------------------------------------------------------------------------
class GuiDlgKHR1ServoChan(tkinter.simpledialog.Dialog):
  """ Options Dialog for KHR-1 Active Servo Channel Class.

      The result on dialog exit:
        On ok success, result dictionary:
           {'active_channels':<activeServos>}
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, title='KHR-1 Active Servo Channels Dialog',
                                lastSettings={}, defaultSettings={}):
    """ Initialize the Debug Dialog.

        Parameters:
          guiParent       - This dialog's parent GUI object.
          lastSettings    - Settings of last configurations. See Return Value.
          defaultSettings - Default settings.
          title           - Title of this dialog.
    """
    self.result         = None
    self.mLastSettings  = lastSettings
    self.mDftSettings   = defaultSettings
    self.mTitle         = title

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    pass

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)
    self.mDlgInfo = []

    row    = 0
    column = 0
    
    # list twelve channels per subframe
    chanStart = 0
    chanEnd   = KHR1CmdBase.RCB1NumOfChannels-1
    while chanStart < KHR1CmdBase.KHR1NumOfChannelsDft:
      self.BodyChan(master, row, column, chanStart, chanEnd)
      column += 1
      chanStart = chanStart + KHR1CmdBase.RCB1NumOfChannels
      chanEnd = chanEnd + KHR1CmdBase.RCB1NumOfChannels

    # set current active
    self.ChanPopulate(**self.mLastSettings['active_channels'])

    # Status bar
    row += 1
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font, width=30,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=0, columnspan=column+1,
        padx=3, pady=5, sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select active channels and assign mnemonics.',
        fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def BodyChan(self, master, row, column, chanStart, chanEnd):
    """Create active channels subdialog frame."""
    subframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    
    subrow = 0
    subcol = 0
    
    w = tk.Label(subframe, fg=gt.ColorBlue,
        text="Channels %d - %d" % (chanStart, chanEnd))
    w.grid(row=subrow, column=subrow, columnspan=3)

    subrow += 1

    for chan in range(chanStart, chanEnd+1):

      info = {
        'isactive': tk.IntVar(),
        'mnem': tk.StringVar(),
      }

      # force argument passing
      def cb(chan=chan): # bind with current value, not end of loop value
        self.CbActivateToggle(chan)

      info['isactive'].set(0)
      w = tk.Checkbutton(subframe, variable=info['isactive'],
          command=cb)
      w.grid(row=subrow, column=subcol, sticky=tk.W)
      GuiToolTip.GuiToolTip(w, text="Channel is [not] active for KHR-1.")

      w = tk.Label(subframe, width=11, state=tk.DISABLED,
          text="Channel %2d:" % chan)
      w.grid(row=subrow, column=subcol+1, sticky=tk.W)
      info['wlabel'] = w

      info['mnem'].set('')
      w = tk.Entry(subframe, width=20, state=tk.DISABLED, 
                            textvariable=info['mnem'])
      w.grid(row=subrow, column=subcol+2, sticky=tk.W)
      GuiToolTip.GuiToolTip(w, text="Set channel mnemonic.")
      info['wentry'] = w

      self.mDlgInfo += [info]
      subrow += 1

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
    newActive = {}
    chan = 0
    for info in self.mDlgInfo:
      if info['isactive'].get():
        mnem = info['mnem'].get()
        if not mnem:
          self.ShowStatus("No mnemonic for channel %d specified." % chan,
                          fg=gt.ColorFgStatusError)
          info['wentry'].focus_set()
          return False
        elif mnem in newActive:
          self.ShowStatus("Mnemonic for channel %d is not unique." % chan,
                          fg=gt.ColorFgStatusError)
          info['wentry'].focus_set()
          return False
        newActive[mnem] = chan
      chan += 1

    if len(newActive) == 0:
      self.ShowStatus(
          "At least one servo must be specified, or what's the point?",
          fg=gt.ColorFgStatusError)
      return False

    self.result = {
      'active_channels': newActive
    }

    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

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
  def ChanPopulate(self, **activeServos):
    """ Populate active channels.

        Parameters:
          **activeServos  - Keyword arguments <mnem>=<chanNum> specifying
                            active KHR-1 servos.

        Return Value:
          None
    """
    for mnem, chan in activeServos.items():
      self.ChanActivate(chan, mnem)
    actChanList = list(activeServos.values())
    for chan in range(0, KHR1CmdBase.KHR1NumOfChannelsDft):
      if chan not in actChanList:
        self.ChanDeactivate(chan)

  #--
  def ChanActivate(self, chan, mnem):
    """ Activate a given channel servo dialog. """
    self.mDlgInfo[chan]['isactive'].set(1)
    self.mDlgInfo[chan]['wlabel']['state'] = tk.NORMAL
    self.mDlgInfo[chan]['wentry']['state'] = tk.NORMAL
    self.mDlgInfo[chan]['wentry'].delete(0, tk.END)
    self.mDlgInfo[chan]['mnem'].set(mnem)

  #--
  def ChanDeactivate(self, chan):
    """ Deactivate a given channel servo dialog. """
    self.mDlgInfo[chan]['isactive'].set(0)
    self.mDlgInfo[chan]['wlabel']['state'] = tk.DISABLED
    self.mDlgInfo[chan]['wentry']['state'] = tk.DISABLED

  #--
  def CbActivateToggle(self, chan):
    """ Toggle channel between active and deactive.
    """
    if self.mDlgInfo[chan]['isactive'].get():
      self.ChanActivate(chan, self.mDlgInfo[chan]['mnem'].get())
    else:
      self.ChanDeactivate(chan)

  #--
  def CbDefaults(self, event=None):
    self.ChanPopulate(**self.mDftSettings['active_channels'])
 
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
    """ GuiDlgKHR1ServoChan Unit Test Main """
    root = tk.Tk()
    dlg = GuiDlgKHR1ServoChan(root,
        lastSettings={'active_channels':KHR1CmdBase.KHR1FacDftActiveServos},
        defaultSettings={'active_channels':KHR1CmdBase.KHR1FacDftActiveServos})
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run unit test
  main()
