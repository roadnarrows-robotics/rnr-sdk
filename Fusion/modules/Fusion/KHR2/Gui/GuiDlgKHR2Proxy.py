################################################################################
#
# GuiDlgKHR2Proxy.py
#

""" Graphical User Interface KHR-2 BotSense Proxy Dialog Module

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.10.22

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

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiToolTip as GuiToolTip

import  Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['proxy_addr', 'proxy_port', 'i2c_dev_name'] + \
          BsProxyClient.BsProxiedDevAll

#-------------------------------------------------------------------------------
# CLASS: GuiDlgKHR2Proxy
#-------------------------------------------------------------------------------
class GuiDlgKHR2Proxy(tkinter.simpledialog.Dialog):
  """ Fusion Preferences Dialog Class

      The result on dialog exit:
        On ok success, returns dictionary of preferences settings
          {'proxy_addr':str, 'proxy_port':int, 'i2c_dev_name':str,
           <proxdev>:{'enable':bool, 'i2c_addr':int}, ...}
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
    elif key == 'i2c_dev_name':
      return self.mLastSettings.get(key, '')
    elif key in BsProxyClient.BsProxiedDevAll:
      proxdata = self.mLastSettings.get(key, {'enable':False, 'i2c_addr':0})
      return proxdata[subkey]
    else:
      return 0

  #--
  def _getProxDevLabel(self, proxdev):
    """ Gets label string for proxied device. """
    if proxdev == 'bpfoot_left':
      return "Left Foot"
    elif proxdev == 'bpfoot_right':
      return "Right Foot"
    elif proxdev == 'bpimu':
      return "IMU"
    elif proxdev == 'bphand_left':
      return "Left Hand"
    elif proxdev == 'bphand_right':
      return "Right Hand"
    elif proxdev == 'bpcompass':
      return "Compass"
    else:
      return proxdev+":"

  #--
  def _mkcallback(self, proxdev):
    return lambda: self.CbEnable(proxdev=proxdev)

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('BotSense IP Proxy Server Preferences')

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Feet Control Frame
    self._bodyProxDevFrame(master, row, column)

    row += 1

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

    # set state
    for proxdev in self.mProxiedDev.keys():
      self.CbEnable(proxdev=proxdev)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyProxDevFrame(self, master, row, column):
    """Create Proxied I2C Control subdialog frame."""
    self.mProxiedDev = {}

    for proxdev in BsProxyClient.BsProxiedDevAll:
      if proxdev in self.mLastSettings:
        self.mProxiedDev[proxdev] = {}
        self.mProxiedDev[proxdev]['var_enable']   = tk.IntVar()
        self.mProxiedDev[proxdev]['var_i2c_addr'] = tk.IntVar()
        if self._getSetting(proxdev, 'enable'):
          self.mProxiedDev[proxdev]['var_enable'].set(1)
        else:
          self.mProxiedDev[proxdev]['var_enable'].set(0)
        i2c_addr = self._getSetting(proxdev, 'i2c_addr')
        self.mProxiedDev[proxdev]['var_i2c_addr'].set(i2c_addr)
        self.mProxiedDev[proxdev]['label'] = self._getProxDevLabel(proxdev)

    self.mVarI2CDevName   = tk.StringVar()
    self.mVarI2CDevName.set(self._getSetting('i2c_dev_name'))

    subframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    subframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E+tk.S)

    row = 0
    col = 0

    # subframe label
    w = tk.Label(subframe, text='Proxied I'+gt.UniSuperscript['2']+'C Devices',
        fg=gt.ColorBlue)
    w.grid(row=row, column=col, columnspan=4, sticky=tk.N)

    row += 1

    ##### proxied devices ####
    keys = list(self.mProxiedDev.keys())
    keys.sort()
    for proxdev in keys:
      proxdata = self.mProxiedDev[proxdev]

      i2cframe = tk.Frame(subframe, relief=tk.RIDGE, borderwidth=1)
      i2cframe.grid(row=row, column=col, columnspan=2, padx=3, ipadx=3, ipady=3,
               sticky=tk.N+tk.W+tk.E+tk.S)

      i2crow = 0
      i2ccol = 0

      # label
      w = tk.Label(i2cframe, width=12, text=proxdata['label']+':', 
          anchor=tk.E, fg=gt.ColorBlack)
      w.grid(row=i2crow, column=i2ccol, sticky=tk.N+tk.E)

      i2ccol += 1

      # enable button
      w = tk.Checkbutton(i2cframe, command=self._mkcallback(proxdev),
                       variable=proxdata['var_enable'])
      w.grid(row=i2crow, column=i2ccol, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(w, 
          text="Enable BrainPack %s sensor." % proxdata['label'])

      i2ccol += 1

      # i2c label
      w = tk.Label(i2cframe, text='I'+gt.UniSuperscript['2']+'C Address:',
                fg=gt.ColorBlack)
      w.grid(row=i2crow, column=i2ccol, sticky=tk.E)
      self.mProxiedDev[proxdev]['wlabel_i2c_addr'] = w

      i2ccol += 1

      # i2c address entry
      w = tk.Entry(i2cframe, width=5, textvariable=proxdata['var_i2c_addr'])
      w.grid(row=i2crow, column=i2ccol, sticky=tk.W, padx=3)
      GuiToolTip.GuiToolTip(w,
          text="BrainPack %s I2C address." % proxdata['label'])
      self.mProxiedDev[proxdev]['wentry_i2c_addr'] = w

      row += 1

    ##### i2c device name ####

    row += 1
    col = 0

    # device device name
    w = tk.Label(subframe,
        width=12,
        text='I'+gt.UniSuperscript['2']+'C Device:',
        fg=gt.ColorBlack)
    w.grid(row=row, column=col, sticky=tk.N+tk.E)

    col += 1

    # device name entry
    w = tk.Entry(subframe, width=24, textvariable=self.mVarI2CDevName)
    w.grid(row=row, column=col, sticky=tk.W, padx=3)
    GuiToolTip.GuiToolTip(w,
      text="Proxied I2C Bus device name where BrainPack sensors are attached.")

  #--
  def _bodyProxyFrame(self, master, row, column):
    """Create BotSense IP Proxy Server subdialog frame."""
    self.mVarProxyAddr  = tk.StringVar()
    self.mVarProxyPort  = tk.IntVar()

    self.mVarProxyAddr.set(self._getSetting('proxy_addr'))
    self.mVarProxyPort.set(self._getSetting('proxy_port'))

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

    for proxdev,proxdata in self.mProxiedDev.items():
      self.result[proxdev] = {}
      if proxdata['var_enable'].get():
        self.result[proxdev]['enable'] = True
      else:
        self.result[proxdev]['enable'] = False
      self.result[proxdev]['i2c_addr'] = proxdata['var_i2c_addr'].get()

    self.result['i2c_dev_name'] = self.mVarI2CDevName.get()

    self.result['proxy_addr'] = self.mVarProxyAddr.get()
    self.result['proxy_port'] = self.mVarProxyPort.get()

    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def CbEnable(self, proxdev=None):
    val = self.mProxiedDev[proxdev]['var_enable'].get()
    if val:
      self.mProxiedDev[proxdev]['wlabel_i2c_addr']['state'] = tk.NORMAL
      self.mProxiedDev[proxdev]['wentry_i2c_addr']['state'] = tk.NORMAL
    else:
      self.mProxiedDev[proxdev]['wlabel_i2c_addr']['state'] = tk.DISABLED
      self.mProxiedDev[proxdev]['wentry_i2c_addr']['state'] = tk.DISABLED

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
    """ GuiDlgKHR2Proxy Unit Test Main """
    root = tk.Tk()
    last =  {
              'bpfoot_left': {'enable':True, 'i2c_addr':0x32},
              'bpfoot_right': {'enable':False, 'i2c_addr':0x33},
              'bpimu': {'enable':True, 'i2c_addr':0x71}
            }
    dlg = GuiDlgKHR2Proxy(root, lastSettings=last)
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
