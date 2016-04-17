################################################################################
#
# GuiDlgPlugin.py
#

""" Graphical User Interface Module Plugin Dialog Module

Graphical User Interface (GUI) Tkinter module plugin dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.12

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

import  os
import  sys
import  Tkinter as tk
import  tkSimpleDialog
import  tkFont

import  Fusion.Utils.Tools as utils

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiDlgOpen as GuiDlgOpen
import  Fusion.Gui.GuiDlgEntryBox as GuiDlgEntryBox
import  Fusion.Gui.GuiToolTip as GuiToolTip

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

def GetSettingNames():
  """ Return list of dictionary keys of selected settings and results. """
  return ['pluginPath',
          'pluginEPointName',
          'pluginMod',
          'pluginEPoint',
          'initialdir'
         ]

#-------------------------------------------------------------------------------
# CLASS: GuiDlgPlugin
#-------------------------------------------------------------------------------
class GuiDlgPlugin(tkSimpleDialog.Dialog):
  """ Module Plugin Dialog Class """

  #--
  def __init__(self, guiParent, pluginType,
                     dirList=[],
                     initialdir='.',
                     title='Plugin Modules'):
    """ Initialize the Dialog.

        Parameters:
          guiParent     - this dialog's parent GUI object.
          pluginType    - type of plugin found in __plugin__.py
          dirList       - list of known plugin directories.
          initialdir    - initial starting directory for browsing.
          title         - dialog title.

        Return Value:
          On ok success, returns dictionary of settings. Entries:
            'pluginPath'        - file path string to module 
            'pluginEPointName'  - module entry point name, if any
            'pluginMod'         - reference to imported module
            'pluginEPoint'      - reference to entry point in module, if any
            'initialdir'        - new initial browse directory after dialog
          On cancel, returns None
    """
    self.result       = None

    self.mParent      = guiParent
    self.mPluginType  = pluginType
    self.mDirList     = dirList
    self.mInitialDir  = initialdir
    self.mTitle       = title

    self.mEntryPoints = []    # list of entry points names
    self.mEPointName  = None  # name of choosen entry point, if any

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last opened parameter. """
    return None

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    font = tkFont.Font(master, font=gt.FontHelv10Bold)

    row    = 0
    column = 0

    # Plugin Directory List Select Frame
    self._bodyDirFrame(master, row, column)

    # Plugin Module Select Frame
    column += 1
    self._bodyModFrame(master, row, column)

    # Current Selection/Browse
    row += 1
    column = 0
    self._bodyCurBrowseFrame(master, row, column, 2)

    # Status line
    row += 1
    column = 0
    self.mEntryStatus = tk.Entry(master, relief=tk.FLAT, font=font,
        state='readonly')
    self.mEntryStatus.grid(row=row, column=column, columnspan=2, padx=3, pady=5,
        sticky=tk.W+tk.E)

    # Show current selection
    self.ShowCurSelection()

    # Show status
    self.ShowStatus('Select %s plugin' % self.mPluginType,
        fg=gt.ColorFgStatusPrompt)

    # first resize event (it is automatic) will calculate dialog dimensions
    self.mHasDim = False

    # bind resizer
    self.bind('<Configure>', self.CbResize )

  #--
  def _CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort.
        Note: Cannot do the calculations within the body() or incorrect
              dialog sizes will result. Must wait until parent dialog
              finishes drawing.
    """
    # force idletask to determine size
    self.update_idletasks()

    # current dialog dimensions (pixels)
    self.mDlgGeo = gut.geometry(self)

    # Listboxes' dimensions (pixels)
    geoLbDirs = gut.geometry(self.mListboxDirs)
    geoLbMods = gut.geometry(self.mListboxMods)
    geoLbInfo = gut.geometry(self.mListboxInfo)

    # total widths all the listboxes (pixels)
    wTotLbs = geoLbDirs[0] + geoLbMods[0] + geoLbInfo[0]

    # listbox fix offsets (pixels)
    self.mLbWOffset   = self.mDlgGeo[0] - wTotLbs
    self.mLbHOffset   = self.mDlgGeo[1] - geoLbDirs[1]

    # keep listbox ratios
    self.mWRatioLbDirs = float(geoLbDirs[0]) / float(wTotLbs)
    self.mWRatioLbMods = float(geoLbMods[0]) / float(wTotLbs)
    self.mWRatioLbInfo = float(geoLbInfo[0]) / float(wTotLbs)

    # pixels per character ratios (listboxes use the same font)
    wCharLb = float(self.mListboxDirs['width'])
    hCharLb = float(self.mListboxDirs['height'])
    self.mPixPerWChar = float(geoLbDirs[0])/wCharLb
    self.mPixPerHChar = float(geoLbDirs[1])/hCharLb

    # current selection entry width offset
    geo = gut.geometry(self.mEntryCurSel)
    self.mEcWOffset = self.mDlgGeo[0] - geo[0]

  #--
  def CbResize(self, event):
    """ Resize callback event. """

    # first time through: calculate important window and widget dimensions 
    # used for resizing
    if not self.mHasDim:
      self._CalcDim()
      self.mHasDim = True
      return

    # real resize event
    geo = gut.geometry(self)
    if geo[0] != self.mDlgGeo[0] or geo[1] != self.mDlgGeo[1]:
      
      lbTotWidth  = float(geo[0] - self.mLbWOffset)
      lbHeight    = float(geo[1] - self.mLbHOffset)

      hchar = int(lbHeight / self.mPixPerHChar)

      # resize dirs listbox
      wchar = int((lbTotWidth * self.mWRatioLbDirs) / self.mPixPerWChar)
      self.mListboxDirs['width']  = wchar
      self.mListboxDirs['height'] = hchar

      # resize mods listbox
      wchar = int((lbTotWidth * self.mWRatioLbMods) / self.mPixPerWChar)
      self.mListboxMods['width']  = wchar
      self.mListboxMods['height'] = hchar

      # resize info listbox
      wchar = int((lbTotWidth * self.mWRatioLbInfo) / self.mPixPerWChar)
      self.mListboxInfo['width']  = wchar
      self.mListboxInfo['height'] = hchar

      # resize current selection entry
      wchar = int(float(geo[0] - self.mEcWOffset) / self.mPixPerWChar)
      self.mEntryCurSel['width'] = wchar

  #--
  def _bodyDirFrame(self, master, row, column):
    """Create Plugin Directory List subdialog frame."""
    self.mVarDir = tk.StringVar()

    # Frame
    dirframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    dirframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W)
    GuiToolTip.GuiToolTip(dirframe, #follow_mouse=1,
        text='Select plugin directory')

    # Title Label
    row = 0
    column = 0
    w = tk.Label(dirframe, text='%s Plugin Directories:' % self.mPluginType,
                 fg=gt.ColorBlue)
    w.grid(row=row, column=column, sticky=tk.NW)

    # Directory Listbox
    row += 1
    vscrollbar = tk.Scrollbar(dirframe, orient=tk.VERTICAL)
    hscrollbar = tk.Scrollbar(dirframe, orient=tk.HORIZONTAL)
    self.mListboxDirs = tk.Listbox(dirframe, selectmode=tk.BROWSE,
                                  height=10, width=40,
                                  yscrollcommand=vscrollbar.set, 
                                  xscrollcommand=hscrollbar.set) 
    vscrollbar.config(command=self.mListboxDirs.yview)
    hscrollbar.config(command=self.mListboxDirs.xview)
    self.mListboxDirs.bind("<ButtonRelease-1>", self.CbDir)
    
    idx = 0
    for dname in self.mDirList:
      dname = utils.canonicalpath(dname)   # expand $var, ~, strip '//', etc
      self.mListboxDirs.insert(tk.END, dname)
      idx += 1
    if idx == 0:
      self.mListboxDirs.insert(tk.END, '** No directories **')

    self.mListboxDirs.grid(row=row, column=0, sticky=tk.W+tk.N+tk.S+tk.E)
    column += 1
    vscrollbar.grid(row=row, column=column, sticky=tk.W+tk.N+tk.S)
    row +=1
    column = 0
    hscrollbar.grid(row=row, column=column, sticky=tk.S+tk.W+tk.E)

  #--
  def _bodyModFrame(self, master, row, column):
    """Create Plugin Module List subdialog frame."""
    self.mVarMod = tk.StringVar()

    # Frame
    modframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    modframe.grid(row=row, column=column, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W)

    # Title Label
    row = 0
    column = 0
    w = tk.Label(modframe, text='%s Plugin Modules:' % self.mPluginType,
                           fg=gt.ColorBlue)
    w.grid(row=row, column=column, sticky=tk.NW)

    vscrollbar = tk.Scrollbar(modframe, orient=tk.VERTICAL)

    # Module Name Listbox
    row += 1
    hscrollbar = tk.Scrollbar(modframe, orient=tk.HORIZONTAL)
    self.mListboxMods = tk.Listbox(modframe, selectmode=tk.BROWSE,
                                  height=10, width=30,
                                  yscrollcommand=vscrollbar.set, 
                                  xscrollcommand=hscrollbar.set) 
    vscrollbar.config(command=self.CbModYView)
    hscrollbar.config(command=self.mListboxMods.xview)
    self.mListboxMods.bind("<ButtonRelease-1>", self.CbMod)
    GuiToolTip.GuiToolTip(self.mListboxMods,
        text='Select %s plugin module.' % self.mPluginType)
    
    self.mListboxMods.insert(tk.END, '** No Modules Selected **')

    self.mListboxMods.grid(row=row, column=column, sticky=tk.W+tk.N+tk.S+tk.E)
    hscrollbar.grid(row=row+1, column=column, sticky=tk.S+tk.W+tk.E)

    # Module Info Listbox
    column += 1
    hscrollbar = tk.Scrollbar(modframe, orient=tk.HORIZONTAL)
    self.mListboxInfo = tk.Listbox(modframe, selectmode=tk.BROWSE,
                                  height=10, width=40,
                                  yscrollcommand=vscrollbar.set, 
                                  xscrollcommand=hscrollbar.set) 
    hscrollbar.config(command=self.mListboxInfo.xview)
    GuiToolTip.GuiToolTip(self.mListboxInfo,
        text='Displays %s plugin module information.' % self.mPluginType)
    #self.mListboxInfo.bind("<ButtonRelease-1>", self.CbInfo)
    
    self.mListboxInfo.grid(row=row, column=column, sticky=tk.W+tk.N+tk.S+tk.E)
    hscrollbar.grid(row=row+1, column=column, sticky=tk.S+tk.W+tk.E)

    column += 1
    vscrollbar.grid(row=row, column=column, sticky=tk.W+tk.N+tk.S)

  #--
  def _bodyCurBrowseFrame(self, master, row, column, columnspan):
    """Create Current Selection/Browse subdialog frame."""
    # Frame
    cbframe = tk.Frame(master, relief=tk.RAISED, borderwidth=1)
    cbframe.grid(row=row, column=column, columnspan=columnspan,
                  padx=3, ipadx=3, ipady=3, sticky=tk.N+tk.W+tk.E)

    font = tkFont.Font(cbframe, font=gt.FontHelv10Bold)

    row = 0
    column = 0
    self.mEntryCurSel = tk.Entry(cbframe, width=105, font=font,
                                 relief=tk.RAISED, state='readonly')
    self.mEntryCurSel.grid(row=row, column=column, padx=3, pady=5,
        sticky=tk.W+tk.E)
    GuiToolTip.GuiToolTip(self.mEntryCurSel, text='Displays current selection.')

    column += 1
    b = tk.Button(cbframe, text='Browse...', command=self.CbBrowse)
    b.grid(row=row, column=column, sticky=tk.E)
    GuiToolTip.GuiToolTip(b,
        text='Browse file system for %s plugin module.' % self.mPluginType)

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

    # settings
    dir         = self.mVarDir.get()
    mod         = self.mVarMod.get()
    epointname  = self.mEPointName
    initialdir  = self.mInitialDir

    if not initialdir:
      initialdir = '.'

    # setting validations
    if not dir:
      self.ShowStatus("No directory specified.", fg=gt.ColorFgStatusError)
      return False
    if not mod:
      self.ShowStatus("No module.", fg=gt.ColorFgStatusError)
      return False

    pluginpath = dir + os.path.sep + mod
    modbase, modext = os.path.splitext(mod)

    # split off any '.py' extension
    if not modbase:
      self.ShowStatus("Badly formed module name: %s" % mod,
          fg=gt.ColorFgStatusError)
      return False

    # Try importing plugin module
    try:
      plugin = utils.importmodule(pluginpath, modbase)
    except ImportError, err:
      self.ShowStatus('%s: %s' % (self.mVarDir.get(), err),
                      fg=gt.ColorFgStatusError)
      return False
    except:
      self.ShowStatus('%s: %s' % (self.mVarDir.get(), sys.exc_info()[0]),
                      fg=gt.ColorFgStatusError)
      return False

    # Try attaching to any entry point
    if epointname:
      try:
        epoint = eval('plugin.'+epointname)
      except AttributeError:
        utils.delmodule(plugin)
        del plugin
        self.ShowStatus('%s: %s attribute not found' % (pluginpath, epointname),
            fg=gt.ColorFgStatusError)
        return False
    else:
      epoint = None

    # Got it
    self.result = {'pluginPath':pluginpath, 'pluginEPointName':epointname,
                   'pluginMod':plugin, 'pluginEPoint':epoint,
                   'initialdir':initialdir}
    return True

  #--
  def apply(self):
    """Apply dialog data and settings."""
    pass

  #--
  def ListModules(self):
    self.ClearMods()
    pluginpath = self.mVarDir.get() + os.path.sep + '__plugins__.py'
    try:
      plugin = utils.importmodule(pluginpath, '__plugins__')
    except ImportError, err:
      self.ShowStatus('%s: %s' % (self.mVarDir.get(), err),
                      fg=gt.ColorFgStatusError)
      return
    except:
      self.ShowStatus('%s: %s' % (self.mVarDir.get(), sys.exc_info()[0]),
                      fg=gt.ColorFgStatusError)
      return
    try:
      modList = eval('plugin.'+self.mPluginType)
    except AttributeError:
      utils.delmodule(plugin)
      del plugin
      self.ShowStatus('%s: %s attribute not found' % \
          (pluginpath, self.mPluginType),
          fg=gt.ColorFgStatusError)
      return
    num = self.SetMods(modList)
    if num == 0:
      self.ShowStatus('%s: No modules listed in %s' % \
          (pluginpath, self.mPluginType),
          fg=gt.ColorFgStatusError)
    utils.delmodule(plugin)
    del plugin

  #-- 
  def ClearMods(self):
    """ Clear plugin module name and info lists. """
    self.mListboxMods.delete(0, tk.END)
    self.mListboxInfo['state'] = tk.NORMAL
    self.mListboxInfo.delete(0, tk.END)
    self.mListboxInfo['state'] = tk.DISABLED
    self.mEntryPoints = []

  #-- 
  def SetMods(self, modList):
    """ Set plugin module name and info lists. """
    idx = 0
    self.mListboxInfo['state'] = tk.NORMAL
    for modEntry in modList:
      self.mListboxMods.insert(tk.END, modEntry[0])
      self.mEntryPoints.append(modEntry[1])
      self.mListboxInfo.insert(tk.END, modEntry[2])
      idx += 1
    self.mListboxInfo['state'] = tk.DISABLED
    return idx

  #--
  def ShowStatus(self, text='', fg=gt.ColorFgStatusOk):
    """Show dialog status."""
    self.mEntryStatus['state'] = tk.NORMAL
    self.mEntryStatus['fg'] = fg
    self.mEntryStatus.delete(0, tk.END)
    self.mEntryStatus.insert(0, text)
    self.mEntryStatus['state'] = 'readonly'

  #--
  def ShowCurSelection(self):
    """Show current selection."""
    s = ''
    dir         = self.mVarDir.get()
    mod         = self.mVarMod.get()
    epointname  = self.mEPointName
    if dir:
      s = '[' + dir
      if mod:
        s += os.path.sep
        s += mod
        if epointname:
          s += ', ' + epointname
      s += ']'
    self.mEntryCurSel['state'] = tk.NORMAL
    self.mEntryCurSel['fg'] = gt.ColorBlack
    self.mEntryCurSel.delete(0, tk.END)
    self.mEntryCurSel.insert(0, s)
    self.mEntryCurSel['state'] = 'readonly'

  #--
  def CbDir(self, event): 
    """Directories Listbox widget event callback. """
    items = self.mListboxDirs.curselection()
    if len(items) == 0:
      return
    try:
      idx = int(items[0])
    except ValueError:
      return
    dir = self.mListboxDirs.get(idx)
    if dir[0:2] == '**':  # ignore
      return
    self.mVarDir.set(dir)
    self.mVarMod.set('')
    self.ShowStatus('Select plugin module', fg=gt.ColorFgStatusPrompt)
    self.ListModules()
    self.ShowCurSelection()

  #--
  def CbMod(self, event): 
    """Module Names Listbox widget event callback. """
    items = self.mListboxMods.curselection()
    if len(items) == 0:
      return
    try:
      idx = int(items[0])
    except ValueError:
      return
    mod = self.mListboxMods.get(idx)
    if mod[0:2] == '**':  # ignore
      return
    self.mVarMod.set(mod)
    self.mEPointName = self.mEntryPoints[idx]
    self.ShowStatus('Ok')
    self.ShowCurSelection()

  #--
  def CbModYView(self, move, offset): 
    """ Vertical scroll callback. """
    self.mListboxMods.yview(move, offset)
    self.mListboxInfo.yview(move, offset)

  #--
  def CbBrowse(self):
    """ Browse button callback. """
    dlg = GuiDlgOpen.GuiDlgOpen(self.mParent, openCmd=None,
                    title='Open %s Python File' % self.mPluginType,
                    filetypes=[('Python Files', '*.py', 'TEXT'),
                               ('All files', '*')],
                    defaultextension='.py',
                    initialdir=self.mInitialDir)
    if dlg.result:
      dname = os.path.dirname(dlg.result)
      fname = os.path.basename(dlg.result)
      self.mVarDir.set(dname)
      self.mInitialDir = dname
      self.mVarMod.set(fname)
      self.mEPointName = None
      self.ClearMods()
      self.ShowCurSelection()
      if fname == "__plugins__.py":
        self.mListboxDirs.insert(tk.END, dname)
      else:
        dlg = GuiDlgEntryBox.GuiDlgEntryBox(self.mParent,
                             title='Enter %s Entry Point' % self.mPluginType,
                             fieldname='Entry Point')
        self.mEPointName = dlg.result
        self.ShowCurSelection()
      self.ShowStatus('Ok')


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  #--
  def main():
    """ GuiDlgPlugin Unit Test Main """
    root = tk.Tk()
    dlg = GuiDlgPlugin(root, 'vRobots', 
      dirList=['/prj/fusion/Fusion-1.0/Fusion/KheGP2D120',
               '/prj/fusion/Fusion-1.0/Fusion/tests/fish',
               '$HOME/src', '~/src/brains'],
      title='vRobot Plugins')
    if dlg.result:
      print 'ok:', dlg.result
      help(dlg.result['pluginMod'])
    else:
      print 'cancel'

  # run unit test
  main()
