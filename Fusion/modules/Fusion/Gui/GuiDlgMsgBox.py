################################################################################
#
# GuiDlgMsgBox.py
#

""" Graphical User Interface Message Box Dialog Module

Graphical User Interface (GUI) Tkinter message box dialog module.

Note: The tkMessageBox does work, but the layout is poor. So, with 
      disgust I've added this dialog.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.02

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

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# Types of message boxes
INFO      = 'info'
WARNING   = 'warning'
ERROR     = 'error'


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

#--
def InfoBox(msg, justify=tk.CENTER):
  """ Show an information box.

      Parameters:
        msg     - the informational message
        justify - left, center, right
  """
  GuiDlgMsgBox(INFO, 'Fusion Informational', msg, justify=justify)

#--
def WarningBox(msg, justify=tk.CENTER):
  """ Show a warning box.

      Parameters:
        msg     - the warning message
        justify - left, center, right
  """
  GuiDlgMsgBox(WARNING, 'Fusion Warning', msg, justify=justify)

#--
def ErrorBox(msg, justify=tk.CENTER):
  """ Show an error box.

      Parameters:
        msg     - the error message
        justify - left, center, right
  """
  GuiDlgMsgBox(ERROR, 'Fusion Error', msg, justify=justify)


#-------------------------------------------------------------------------------
# CLASS: GuiDlgMsgBox
#-------------------------------------------------------------------------------
class GuiDlgMsgBox(tkSimpleDialog.Dialog):
  """ Info/Warning/Error Message Box Dialog Class.
  """

  #--
  def __init__(self, type, title, message, justify=tk.CENTER):
    """ Initialize the dialog.

        Parameters:
          type      - 'info', 'warning', or 'error'
          title     - message box title
          message   - the message
          justify   - left, center, right
    """
    self.mType    = type
    self.mTitle   = title
    self.mMessage = message
    self.mJustify = justify
    tkSimpleDialog.Dialog.__init__(self, None)

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title(self.mTitle)

    if self.mJustify == tk.CENTER:
      sticky = tk.W+tk.E
    elif self.mJustify == tk.LEFT:
      sticky = tk.W
    elif self.mJustify == tk.RIGHT:
      sticky = tk.E
    else:
      sticky = tk.N
    row = 0
    column = 0

    frame = tk.Frame(master)
    frame.grid(row=row, column=column, sticky=tk.W+tk.E)

    self.drawimage(frame, row, column)

    w = tk.Label(frame, fg=gt.ColorBlue, justify=self.mJustify, 
        text=self.mMessage)
    w.grid(row=0, column=1, padx=5, pady=5, sticky=sticky)

    # fix size dialog (not resizeable)
    self.resizable(0,0)
  
  #--
  def drawimage(self, master, row, column):
    if self.mType == 'info':
      imageBaseName = gt.ImageInfo
    elif self.mType == 'warning':
      imageBaseName = gt.ImageWarning
    elif self.mType == 'error':
      imageBaseName = gt.ImageError
    else:
      return
    
    imageFile = gut.GetFusionImageFileName(imageBaseName)

    if imageFile:
      w = gut.ImageWidget(master, imageFile)
      w.grid(row=row, column=column, sticky=tk.W)

  #--
  def buttonbox(self):
    """ Add only the 'ok' button box by overriding standard buttons. """

    box = tk.Frame(self)

    w = tk.Button(box, text="OK", width=10, command=self.ok, default=tk.ACTIVE)
    w.pack(side=tk.LEFT, padx=5, pady=5)

    self.bind("<Return>", self.ok)

    box.pack()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def main():
    """ GuiDlgMsgBox Test Main """
    root = tk.Tk()
    root.wm_title('My root')
    GuiDlgMsgBox('info', 'Fusion Informational',
        'this is the message.\nwith a long variable name\n'
        'abcdefghijklmnopqrstuvwxyz-nowiknowmyabcswhatdoyouthinkofme?')
    GuiDlgMsgBox('warning', 'Fusion Warning',
        'Danger, Will Robinson! Danger!')
    GuiDlgMsgBox('error', 'Fusion Error',
        'I understand. To err is human')

  # run test
  main()
