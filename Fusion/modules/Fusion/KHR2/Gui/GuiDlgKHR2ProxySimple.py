################################################################################
#
# GuiDlgKHR2ProxySimple.py
#

""" Graphical User Interface KHR-2 BotSense Proxy Simple Dialog Module

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2008.07.19

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

import  tkinter as tk
import  tkinter.simpledialog
import  tkinter.font

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiToolTip as GuiToolTip

import  Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['proxy_addr', 'proxy_port', 'dev_name']

#-------------------------------------------------------------------------------
# CLASS: GuiDlgKHR2ProxySimple
#-------------------------------------------------------------------------------
class GuiDlgKHR2ProxySimple(tkinter.simpledialog.Dialog):
  """ Fusion Simple BotSense IP Proxy Preferences Dialog Class

      The result on dialog exit:
        On ok success, returns dictionary of preferences settings
          {'proxy_addr':str, 'proxy_port':int, 'dev_name':str}
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
  def _getSetting(self, key, subkey=None):
    """ Gets last configured setting parameter. """
    if key == 'proxy_addr':
      return self.mLastSettings.get(key, '')
    elif key == 'proxy_port':
      return self.mLastSettings.get(key, 0)
    elif key == 'dev_name':
      return self.mLastSettings.get(key, '')
    else:
      return 0

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('BotSense IP Proxy Server Preferences')

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Proxy Control Frame
    self._bodyProxyFrame(master, row, column)

    row += 1

    # status bar
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=2, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show status
    self.ShowStatus('Select preferences settings.', fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyProxyFrame(self, master, row, column):
    """Create BotSense IP Proxy Server subdialog frame."""
    self.mVarProxyAddr  = tk.StringVar()
    self.mVarProxyPort  = tk.IntVar()
    self.mVarDevName    = tk.StringVar()


    self.mVarProxyAddr.set(self._getSetting('proxy_addr'))
    self.mVarProxyPort.set(self._getSetting('proxy_port'))
    self.mVarDevName.set(self._getSetting('dev_name'))

    subframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)

    row = 0
    col = 0

    # subframe label
    w = tk.Label(subframe, text='Proxy Server', fg=gt.ColorBlue)
    w.grid(row=row, column=col, columnspan=2, sticky=tk.N)

    row += 1

    # address
    w = tk.Label(subframe, text='IP Address:',
        fg=gt.ColorBlack)
    w.grid(row=row, column=col, sticky=tk.E)

    col += 1

    # addr entry
    w = tk.Entry(subframe, width=32, textvariable=self.mVarProxyAddr)
    w.grid(row=row, column=col, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(w,
        text="BotSense Proxy Server IP address.\n"
            "Address can be a dotted IP address\nor a DNS name.")

    row += 1
    col  = 0

    # address
    w = tk.Label(subframe, text='TCP Port:',
        fg=gt.ColorBlack)
    w.grid(row=row, column=col, sticky=tk.E)

    col += 1

    # addr entry
    w = tk.Entry(subframe, width=6, textvariable=self.mVarProxyPort)
    w.grid(row=row, column=col, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(w,
        text="BotSense Proxy Server TCP port number.\n"
            "The server accepts new connections on this port.")

    row += 1
    col  = 0

    # device device name
    w = tk.Label(subframe,
        width=32,
        text='Proxied Device:',
        fg=gt.ColorBlack)
    w.grid(row=row, column=col, sticky=tk.N+tk.E)

    col += 1

    # device name entry
    w = tk.Entry(subframe, width=24, textvariable=self.mVarDevName)
    w.grid(row=row, column=col, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(w,
      text="Proxied device name where BrainPack sensors are attached.")

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
    self.result = {}
    self.result['proxy_addr'] = self.mVarProxyAddr.get()
    self.result['proxy_port'] = self.mVarProxyPort.get()
    self.result['dev_name'] = self.mVarDevName.get()
    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def ShowStatus(self, text='', fg=gt.ColorFgStatusOk):
    """Show configuration dialog status."""
    self.mEntryStatus['state'] = tk.NORMAL
    self.mEntryStatus['fg'] = fg
    self.mEntryStatus.delete(0, tk.END)
    self.mEntryStatus.insert(0, text)
    self.mEntryStatus['state'] = 'readonly'


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgKHR2ProxySimple Unit Test Main """
    root = tk.Tk()
    last =  {
              'dev_name': '/dev/ooga'
            }
    dlg = GuiDlgKHR2ProxySimple(root, lastSettings=last)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
