################################################################################
#
# GuiDlgEntryBox.py
#

""" Graphical User Interface EntryBox Dialog Module

Graphical User Interface (GUI) Tkinter entry box dialog module. The
EntryBox retreives one field value from the user.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.13

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
import  Fusion.Gui.GuiTypes as gt

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiDlgEntryBox
#-------------------------------------------------------------------------------
class GuiDlgEntryBox(tkSimpleDialog.Dialog):
  """ EntryBox Dialog Class

      The result on dialog exit:
        On ok success, returns entry field string value.
        On cancel, returns None
  """

  #--
  def __init__(self, guiParent, fieldname='Field', title='Entry'):
    """ Initialize the EntryBox Dialog.

        Parameters:
          guiParent - this dialog's parent GUI object
          fieldname - name of field
          title     - title of this dialog
    """
    self.result     = None
    
    self.mTitle     = title
    self.mFieldName = fieldname

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def _lastSetting(self, key):
    """ Gets last configured setting parameter. """
    return None

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    self.mVarEntry = tk.StringVar()

    row    = 0
    column = 0

    # Field Label
    w = tk.Label(master, text=self.mFieldName+':', fg=gt.ColorBlue)
    w.grid(row=row, column=column, sticky=tk.E)

    # Field Entry
    column += 1
    self.mEntry = tk.Entry(master, textvariable=self.mVarEntry, width=20)
    self.mEntry.grid(row=row, column=column, padx=3, pady=5, sticky=tk.W)

    # Put focus on Field Entry 
    self.mEntry.focus_set()

    # fix size dialog (not resizeable)
    self.resizable(0,0)

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

    fieldVal = self.mVarEntry.get()
    if fieldVal:
      self.result = fieldVal
    else:
      self.result = None
    return True

  def apply(self):
    """Apply dialog data and settings."""
    pass


#-------------------------------------------------------------------------------
# CLASS: Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgEntryBox Test Main """
    root = tk.Tk()
    dlg = GuiDlgEntryBox(root, fieldname='Widgy Fidgy', title='Test Entry Box')
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
