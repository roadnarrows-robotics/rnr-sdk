################################################################################
#
# GuiDlgOpen.py
#

""" Graphical User Interface File Open Dialog Module

Graphical User Interface (GUI) Tkinter open dialog module.

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

import  sys
import  tkinter as tk
import  tkinter.filedialog
import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiDlgOpen
#-------------------------------------------------------------------------------
class GuiDlgOpen(tkinter.filedialog.Open):
  """ Open File Dialog Class

        The result on dialog exit: 
          On ok success, result = [opened] filename
          On cancel, result is None
  """

  #--
  def __init__(self, guiParent, openCmd=None, **options):
    """ Initialize the dialog.

        Parameters:
          guiParent - this dialog's parent GUI object
          openCmd   - optional command to open file. If specified, this
                      command should raise an IOError exception if the
                      open fails.
          options   - Any combination of:
                        filetypes
                          this option gives the filetypes to be listed 
                          in the 'type' listbox. Format:
                            filetypes:
                              [typetuple,...]
                            typetuple:
                              (desc_str, filter_str, type_str)
                            desc_str:
                              'descriptive string'
                            filter_str:
                              'filter filter ...'
                            filter: one of:
                              '<file>', '*.<ext>, '*'
                            type_str (opt):
                              'type' (e.g. 'TEXT', 'GIFF')
                        defaultextension
                          extension that will be appended to the filename
                          if the user enters a filename without an
                          extension.
                        initialdirectory
                          specifies the directory to be displayed when
                          the dialog pops up.
                        initialfile
                          specifies a filename to be displayed in the 
                          dialog when it pops up
                        title
                          specifies a string to display as the title
                          of the dialog box other than the default.
    """
    self.result   = None
    self.mOpenCmd = openCmd

    tkinter.filedialog.Open.__init__(self, guiParent, **options)

    self.mOpenFile = self.show()

    if self.mOpenFile and self.Open():
      self.result = self.mOpenFile

  #--
  def Open(self):
    """ Open the file (optional). """
    if not self.mOpenCmd:
      return True
    try:
      self.mOpenCmd(self.mOpenFile)
      return True
    except IOError as err:
      gut.ErrorBox(err)
      return False


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgOpen Test Main """
    root = tk.Tk()
    dlg = GuiDlgOpen(root, openCmd=None,
                  title='Open Test',
                  filetypes=[('Text Files', '*.txt', 'TEXT'),
                             ('Standard output', '<stdout>', 'STREAM'),
                             ('All files', '*')],
                  defaultextension='.txt')
    if dlg.result:
      print('ok:', dlg.result)
    else:
      print('cancel')

  # run test
  main()
