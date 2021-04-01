################################################################################
#
# GuiDlgSerConn.py
#

""" Graphical User Interface Serial Connection Dialog Module

Graphical User Interface (GUI) Tkinter serial connection dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.05

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

import  time
import  serial as ser
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
  return ['port', 'baudrate', 'bytesize', 'parity', 'stopbits']

#-------------------------------------------------------------------------------
# CLASS: GuiDlgSerConn
#-------------------------------------------------------------------------------
class GuiDlgSerConn(tkinter.simpledialog.Dialog):
  """ Serial Connection Dialog Class

      The result on dialog exit:
        On ok success, returns dictionary of connection settings:
          {'port':port, 'baudrate':baudrate, 'bytesize':bytesize,
           'parity':parity, 'stopbits':stopbits)
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, openCmd, 
                     portHistory=[], 
                     lastSettings={},
                     validBaudRates=ser.Serial.BAUDRATES, 
                     validByteSizes=ser.Serial.BYTESIZES,
                     validParities=ser.Serial.PARITIES, 
                     validStopBits=ser.Serial.STOPBITS):
    """ Initialize the Connection Dialog.

        Parameters:
          guiParent       - this dialog's parent GUI object
          openCmd         - command to open port. Expected interface:
                              openCmd(port, baudrate=baudrate,
                                      bytesize=bytesize, parity=parity,
                                      stopbits=stopbits)
                              with the keyword values conforming to the 
                              python serial package.
                            This command should raise IOError if the open
                            fails.
          portHistory     - list of previosly opened and/or preferred 
                              ports (default: empty)
          lastSettings    - settings of last opened connection
                              See Return Value.
          validBaudRates  - valid list of supported baudrates.
                              (default: all host supported)
          validByteSizes  - valid list of supported byte sizes. 
                              (default: all host supported)
          validParities   - valid list of supported parities sizes. 
                              (default: all host supported)
          validStopBits   - valid list of supported stop bits. 
                              (default: all host supported)
    """
    self.mOpenCmd         = openCmd
    self.mPortHistory     = portHistory
    self.mLastSettings    = lastSettings
    self.mValidBaudRates  = validBaudRates
    self.mValidByteSizes  = validByteSizes
    self.mValidParities   = validParities
    self.mValidStopBits   = validStopBits

    self.result = None

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last opened parameter. """
    if key == 'port':
      pass
    elif key == 'baudrate':
      if len(self.mValidBaudRates) == 1:
        return self.mValidBaudRates[0]
    elif key == 'bytesize':
      if len(self.mValidByteSizes) == 1:
        return self.mValidByteSizes[0]
    elif key == 'parity':
      if len(self.mValidParities) == 1:
        return self.mValidParities[0]
    elif key == 'stopbits':
      if len(self.mValidStopBits) == 1:
        return self.mValidStopBits[0]
    if not self.mLastSettings:
      return None
    elif key not in self.mLastSettings:
      return None
    else:
      return self.mLastSettings[key]

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('Serial Connection')

    row    = 0
    column = 0

    # column fudge factor, since grid() can't do the job
    colfudge = len(self.mValidByteSizes)
    if len(self.mValidParities) > colfudge:
      colfudge = len(self.mValidParities)
    if len(self.mValidStopBits) > colfudge:
      colfudge = len(self.mValidStopBits)

    font = tkinter.font.Font(master, font=gt.FontHelv10Bold)

    # Current selection
    self.mEntryCurSel = tk.Entry(master, relief=tk.RAISED, font=font,
        state='readonly')
    self.mEntryCurSel.grid(row=row, column=column, columnspan=3, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Port Select Frame
    row +=1 
    self._bodyPortFrame(master, row, column)

    # BaudRate Select Frame
    column += 1
    self._bodyBaudRateFrame(master, row, column)

    # ByteSize Select Frame
    column += 1
    self._bodyByteSizeFrame(master, row, column, colfudge)

    # Parity Select Frame
    row += 1
    self._bodyParityFrame(master, row, column, colfudge)

    # StopBits Select Frame
    row += 1
    self._bodyStopBitsFrame(master, row, column, colfudge)

    # Status line
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=3, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Put focus on entry port line
    self.mEntryPort.focus_set()

    # Show current selection
    self.ShowCurSelection()

    # Show status
    self.ShowStatus('Select the serial connection configuration',
        fg=gt.ColorFgStatusPrompt)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _bodyPortFrame(self, master, row, column):
    """Create Port subdialog frame."""
    self.mVarPort = tk.StringVar()

    portframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    portframe.grid(row=row, column=column, rowspan=3, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W)
    GuiToolTip.GuiToolTip(portframe, #follow_mouse=1,
        text='Select or specifiy the connection port')

    row = 0
    w = tk.Label(portframe, text='Serial Port:', fg=gt.ColorBlue)
    w.grid(row=row, column=0, sticky=tk.NW)

    row += 1
    scrollbar = tk.Scrollbar(portframe, orient=tk.VERTICAL)
    self.mListboxPorts = tk.Listbox(portframe, selectmode=tk.BROWSE,
        yscrollcommand=scrollbar.set, height=7, width=30)
    scrollbar.config(command=self.mListboxPorts.yview)
    self.mListboxPorts.bind("<ButtonRelease-1>", self.CbPort)
    
    idx = 0
    for port in self.mPortHistory:
      self.mListboxPorts.insert(tk.END, port)
      if port == self._lastSetting('port'):
        self.mListboxPorts.select_set(idx)
        self.mVarPort.set(port)
      idx += 1

    self.mListboxPorts.grid(row=row, column=0, sticky=tk.W+tk.N+tk.S+tk.E)
    scrollbar.grid(row=row, column=1, sticky=tk.W+tk.N+tk.S)

    row += 1
    self.mEntryPort = tk.Entry(portframe,
                              textvariable=self.mVarPort) 
    self.mEntryPort.grid(row=row, columnspan=2, 
        sticky=tk.N+tk.S+tk.W+tk.E, pady=1)
    self.mEntryPort.bind("<KeyRelease>", self.CbEntryPort)

  #--
  def _bodyBaudRateFrame(self, master, row, column):
    """Create BaudRate subdialog frame."""
    self.mVarBaudRate = tk.IntVar()
    self.mVarBaudRate.set(-1)

    brframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    brframe.grid(row=row, column=column, rowspan=3, padx=3, pady=3, ipadx=3,
                  ipady=3, sticky=tk.N+tk.W+tk.S)
    GuiToolTip.GuiToolTip(brframe, #follow_mouse=1,
        text='Select from the supported baudrates')

    row = 0
    w = tk.Label(brframe, text='Baud Rate:', fg=gt.ColorBlue)
    w.grid(row=row, column=0, sticky=tk.W+tk.N)

    row += 1
    scrollbar = tk.Scrollbar(brframe, orient=tk.VERTICAL)
    self.mListboxBaudRates = tk.Listbox(brframe, selectmode=tk.BROWSE,
        yscrollcommand=scrollbar.set, height=8, width=10)
    scrollbar.config(command=self.mListboxBaudRates.yview)
    self.mListboxBaudRates.bind("<ButtonRelease-1>", self.CbBaudRate)
    
    idx = 0
    for baudrate in self.mValidBaudRates:
      self.mListboxBaudRates.insert(tk.END, baudrate)
      if baudrate == self._lastSetting('baudrate'):
        self.mListboxBaudRates.select_set(idx)
        self.mVarBaudRate.set(baudrate)
      idx += 1

    self.mListboxBaudRates.grid(row=row, column=0, sticky=tk.W+tk.N+tk.S+tk.E)
    scrollbar.grid(row=row, column=1, sticky=tk.W+tk.N+tk.S)

  #--
  def _bodyByteSizeFrame(self, master, row, column, colfudge):
    """Create ByteSize subdialog frame."""
    self.mVarByteSize = tk.IntVar()
    self.mVarByteSize.set(-1)

    bsframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    bsframe.grid(row=row, column=column, padx=3, pady=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    GuiToolTip.GuiToolTip(bsframe, #follow_mouse=1,
        text='Select from the supported data byte sizes')

    row = 0
    column = 0
    w = tk.Label(bsframe, text='Byte Size:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    for bytesize in self.mValidByteSizes:
      label = repr(bytesize)
      if len(label) < 4:
        label = '%s%*s' % (label, 4-len(label), ' ')
      b = tk.Radiobutton(bsframe,
                         text=label,
                         variable=self.mVarByteSize,
                         value=bytesize,
                         command=self.CbByteSize)
      b.grid(row=row, column=column, sticky=tk.W, padx=3)
      column += 1
      if bytesize == self._lastSetting('bytesize'):
        b.select()
        self.mVarByteSize.set(bytesize)

    if column < colfudge:
      self._columnfudge(bsframe, row, column, colfudge-column)

  #--
  def _bodyParityFrame(self, master, row, column, colfudge):
    """Create Parity subdialog frame."""
    self.mVarParity = tk.StringVar()

    parframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    parframe.grid(row=row, column=column, padx=3, pady=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    GuiToolTip.GuiToolTip(parframe, #follow_mouse=1,
        text='Select from the supported data parities')

    row = 0
    column = 0
    w = tk.Label(parframe, text='Parity:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    for parity in self.mValidParities:
      if parity == 'N':
        label = 'none'
      elif parity == 'O':
        label = 'odd'
      elif parity == 'E':
        label = 'even'
      else:
        label = repr(parity)
      if len(label) < 4:
        label = '%s%*s' % (label, 4-len(label), ' ')
      b = tk.Radiobutton(parframe,
                         text=label,
                         variable=self.mVarParity,
                         value=parity,
                         command=self.CbParity)
      b.grid(row=row, column=column, padx=3, sticky=tk.W)
      column += 1
      if parity == self._lastSetting('parity'):
        b.select()
        self.mVarParity.set(parity)

    if column < colfudge:
      self._columnfudge(parframe, row, column, colfudge-column)

  #--
  def _bodyStopBitsFrame(self, master, row, column, colfudge):
    """Create StopBits subdialog frame."""
    self.mVarStopBits = tk.IntVar()
    self.mVarStopBits.set(-1)

    sbframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    sbframe.grid(row=row, column=column, padx=3, pady=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    GuiToolTip.GuiToolTip(sbframe, #follow_mouse=1,
        text='Select from the supported stop bits')

    row = 0
    column = 0
    w = tk.Label(sbframe, text='Stop Bits:', fg=gt.ColorBlue)
    w.grid(row=row, column=column, columnspan=2, sticky=tk.NW)

    row += 1
    for stopbits in self.mValidStopBits:
      label = repr(stopbits)
      if len(label) < 4:
        label = '%s%*s' % (label, 4-len(label), ' ')
      b = tk.Radiobutton(sbframe,
                         text=label,
                         variable=self.mVarStopBits,
                         value=stopbits,
                         command=self.CbStopBits)
      b.grid(row=row, column=column, padx=3, sticky=tk.W)
      column += 1
      if stopbits == self._lastSetting('stopbits'):
        b.select()
        self.mVarStopBits.set(stopbits)

    if column < colfudge:
      self._columnfudge(sbframe, row, column, colfudge-column)

  #--
  def _columnfudge(self, parent, row, column, ncols):
    """ Add more columns so left-right grid alignment works. """
    while ncols > 0:
      l = tk.Label(parent, text='          ')
      l.grid(row=row, column=column, padx=3)
      column += 1
      ncols -= 1

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

    port      = self.mVarPort.get()
    baudrate  = self.mVarBaudRate.get()
    bytesize  = self.mVarByteSize.get()
    parity    = self.mVarParity.get()
    stopbits  = self.mVarStopBits.get()

    if not port:
      self.ShowStatus("No port specified.", fg=gt.ColorFgStatusError)
      return False
    if baudrate == -1:
      self.ShowStatus("No baudrate specified.", fg=gt.ColorFgStatusError)
      return False
    if bytesize == -1:
      self.ShowStatus("No bytesize specified.", fg=gt.ColorFgStatusError)
      return False
    if not parity:
      self.ShowStatus("No parity specified.", fg=gt.ColorFgStatusError)
      return False
    if stopbits == -1:
      self.ShowStatus("No stopbits specified.", fg=gt.ColorFgStatusError)
      return False

    # show status
    self.ShowStatus("Opening serial connection %s %d-%d-%s-%d..." % \
                     (port, baudrate, bytesize, parity, stopbits))
    time.sleep(0.1)

    try:
      self.mOpenCmd(port, baudrate=baudrate, bytesize=bytesize, parity=parity,
                    stopbits=stopbits)
      self.result = {'port':port, 'baudrate':baudrate, 'bytesize':bytesize,
                     'parity':parity, 'stopbits':stopbits}
      return True
    except IOError as err:
      s = "%s" % err
      self.ShowStatus(s, fg=gt.ColorFgStatusError)
      return False

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
  def ShowCurSelection(self):
    """Show current connection selection."""
    fg        = gt.ColorBlack
    port      = self.mVarPort.get()
    baudrate  = str(self.mVarBaudRate.get())
    bytesize  = str(self.mVarByteSize.get())
    parity    = self.mVarParity.get()
    stopbits  = str(self.mVarStopBits.get())
    if not port:          port = "?port?";          fg = gt.ColorRed
    if baudrate == '-1':  baudrate = "?baudrate?";  fg = gt.ColorRed
    if bytesize == '-1':  bytesize = "?bytesize?";  fg = gt.ColorRed
    if not parity:        parity = "?parity?";      fg = gt.ColorRed
    if stopbits == '-1':  stopbits = "?stopbits?";  fg = gt.ColorRed
    s = "    %s %s-%s-%s-%s" % \
        (port, baudrate, bytesize, parity, stopbits)
    self.mEntryCurSel['state'] = tk.NORMAL
    self.mEntryCurSel['fg'] = fg
    self.mEntryCurSel.delete(0, tk.END)
    self.mEntryCurSel.insert(0, s)
    self.mEntryCurSel['state'] = 'readonly'

  #--
  def CbPort(self, event): 
    """Port Listbox widget event callback. """
    items = self.mListboxPorts.curselection()
    if len(items) == 0:
      return
    try:
      idx = int(items[0])
    except ValueError:
      return
    port = self.mListboxPorts.get(idx)
    self.mEntryPort.delete(0, tk.END)
    self.mEntryPort.insert(0, port)
    self.ShowCurSelection()

  #--
  def CbEntryPort(self, event): 
    """Port Entry widget event callback. """
    self.ShowCurSelection()

  #--
  def CbBaudRate(self, event): 
    """BaudRate widget event callback. """
    items = self.mListboxBaudRates.curselection()
    if len(items) == 0:
      return
    try:
      idx = int(items[0])
    except ValueError:
      return
    self.mVarBaudRate.set(int(self.mListboxBaudRates.get(idx)))
    self.ShowCurSelection()

  #--
  def CbByteSize(self): 
    """ByteSize widget callback. """
    self.ShowCurSelection()

  #--
  def CbParity(self): 
    """Parity widget callback. """
    self.ShowCurSelection()

  #--
  def CbStopBits(self): 
    """StopBits widget callback. """
    self.ShowCurSelection()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def _opencmd(port, baudrate=2400, bytesize=8, parity='N', stopbits=2):
    print('_opencmd(%s,%d,%d,%s,%d)' % \
            (repr(port), baudrate, bytesize, repr(parity), stopbits))
    open(port)

  def main():
    """ GuiDlgSerConn Test Main """
    root = tk.Tk()
    dlg = GuiDlgSerConn(root, _opencmd, 
      portHistory=['/dev/ttyS0', '/dev/ttyS1', 'com1', 'com2', '/dev/ttyUB0',
                   '/dev/ttyUB1', '/dev/tty10', 'com3', 'com4', 'com5'],
      lastSettings={'port':'/dev/ttyUB0', 'baudrate':19200, 'parity':'E'})
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
