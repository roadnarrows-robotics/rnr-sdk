################################################################################
#
# GuiDlgAskYesOrNo.py
#

""" Graphical User Interface Ask Yes Or No Dialog Module

Graphical User Interface (GUI) Tkinter yes or no dialog module.

Note: The tkMessageBox.askyesno(question) is buggy in python 2.3.
      It is supposedly fixed in 2.4.

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

import  Tkinter as tk
import  tkSimpleDialog
import  tkFont

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiDlgAskYesOrNo
#-------------------------------------------------------------------------------
class GuiDlgAskYesOrNo(tkSimpleDialog.Dialog):
  """ Ask Yes Or No Dialog Class.
  
      The result on dialog exit:
        'yes', 'no', ['cancel']
  """

  #--
  def __init__(self, guiParent, title, question, addcancel=False):
    """ Initialize the dialog.

        Parameters:
          guiParent - This dialog's parent GUI object.
          title     - Dialog's title.
          question  - The question to answer yes or no to.
          addcancel - Do [not] add "Cancel" button option.
    """
    self.mTitle     = title
    self.mQuestion  = question
    self.mAddCancel = addcancel

    tkSimpleDialog.Dialog.__init__(self, guiParent)

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    row = 0
    column = 0

    frame = tk.Frame(master)
    frame.grid(row=row, column=column, sticky=tk.W+tk.E)

    imageFile = gut.GetFusionImageFileName(gt.ImageWarning)

    if imageFile:
      w = gut.ImageWidget(frame, imageFile)
      w.grid(row=row, column=column, padx=5, pady=5)
      column += 1

    w = tk.Label(frame, fg=gt.ColorBlue, justify=tk.CENTER, text=self.mQuestion)
    w.grid(row=row, column=column, padx=5, pady=5)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def destroy(self):
    # default result on window exit with button push
    if self.result is None:
      if self.mAddCancel:
        self.result = 'cancel'
      else:
        self.result = 'no'
    tkSimpleDialog.Dialog.destroy(self)

  #--
  def cbyes(self, event=None):
    """ The answer is yes. """
    self.withdraw()
    self.update_idletasks()
    self.result = 'yes'
    self.destroy()
    return 

  #--
  def cbno(self, event=None):
    """ The answer is no. """
    self.withdraw()
    self.update_idletasks()
    self.result = 'no'
    self.destroy() 
    return 

  #--
  def cbcancel(self, event=None):
    """ The answer is withdrawn. """
    self.withdraw()
    self.update_idletasks()
    self.result = 'cancel'
    self.destroy() 
    return 

  #--
  def buttonbox(self):
    """ Add new buttons overriding the standard buttons. """

    box = tk.Frame(self)

    w = tk.Button(box, text="Yes", width=10, command=self.cbyes, 
        default=tk.ACTIVE)
    w.pack(side=tk.LEFT, padx=5, pady=5)

    w = tk.Button(box, text="No", width=10, command=self.cbno)
    w.pack(side=tk.LEFT, padx=5, pady=5)

    if self.mAddCancel:
      w = tk.Button(box, text="Cancel", width=10, command=self.cbcancel)
      w.pack(side=tk.LEFT, padx=5, pady=5)

    self.bind("<Return>", self.cbyes)

    box.pack()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgAskYesOrNo Unit Test Main """
    root = tk.Tk()
    dlg = GuiDlgAskYesOrNo(root,
                    'Maybe yes, Maybe no',
                    'Do you wanna go out with me?', addcancel=True)
    print dlg.result
    if dlg.result == 'yes':
      return

    dlg = GuiDlgAskYesOrNo(root,
            'No???',
            'But I am a real nice guy.\nYou like hairy backs do you not?')
    print dlg.result

  # run unit test
  main()
