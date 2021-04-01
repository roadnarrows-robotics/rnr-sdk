################################################################################
#
# GuiDlgAbout.py
#

""" Graphical User Interface About Dialog Module

Graphical User Interface (GUI) Tkinter about dialog module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.15

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

import  tkinter as tk
import  tkinter.simpledialog
import  tkinter.font
import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiDlgAbout
#-------------------------------------------------------------------------------
class GuiDlgAbout(tkinter.simpledialog.Dialog):
  """ About Dialog Class """

  #--
  def __init__(self, guiParent, name, version='',
                                mimeType='', descTitle='', desc='',
                                copyright='', logoImage=None):
    """ Initialize the dialog.

        Parameters:
          guiParent - this dialog's parent GUI object
          name      - name of entity in which this is about (required)
          version   - version(s) string of this entity (may have '\\n'
                      formatting characters) (optional)
          mimeType  - MIME type of this entity (optional)
          descTitle - title of description (optional)
          desc      - decription string (may have '\\n' formatting
                      characters) (optional)
          copyright - any copyright string (may have '\\n' formatting
                      characters) (optional)
          logoImage - logo (optional)
    """
    self.mName      = name
    self.mVersion   = version
    self.mMimeType  = mimeType
    self.mDescTitle = descTitle
    self.mDesc      = desc
    self.mCopyRight = copyright
    self.mLogoImage = logoImage

    tkinter.simpledialog.Dialog.__init__(self, guiParent)

  #--
  def body(self, master):
    """Create the dialog body."""

    self.wm_title('About %s' % self.mName)

    row = 0
    column = 0

    frame = tk.Frame(master)
    frame.grid(row=row, column=column, sticky=tk.W+tk.E)

    # Logo Image
    if self.mLogoImage:
      w = gut.ImageWidget(frame, self.mLogoImage)
      w.grid(row=row, column=column, sticky=tk.W)
      column += 1

    # Name, MIME type, and Version
    text = self.mName
    if self.mMimeType:
      text += ' (%s)' % self.mMimeType
    if self.mVersion:
      text += '\n\n' + self.mVersion
    w = tk.Label(frame, fg=gt.ColorBlue, justify=tk.CENTER, text=text)
    w.grid(row=0, column=1, padx=5, pady=5)

    column = 0

    # Description Title
    if self.mDescTitle:
      row += 1
      # spacer
      w = tk.Label(master, text=' ')
      w.grid(row=row, column=column, padx=0, pady=0)
      # description
      row += 1
      row = self._AddText(master, self.mDescTitle, row, column, tk.CENTER)

    # Description
    if self.mDesc:
      row += 1
      # spacer
      w = tk.Label(master, text=' ')
      w.grid(row=row, column=column, padx=0, pady=0)
      # description
      row += 1
      row = self._AddText(master, self.mDesc, row, column, tk.LEFT)

    # CopyRight
    if self.mCopyRight:
      row += 1
      # spacer
      w = tk.Label(master, text=' ')
      w.grid(row=row, column=column, padx=0, pady=0)
      # copyright
      row += 1
      self._AddText(master, self.mCopyRight, row, column, tk.CENTER)

    # fix size dialog (not resizeable)
    self.resizable(0,0)

  #--
  def _AddText(self, master, text, row, column, justify):
    """ Add multi-line text. """
    if justify == tk.CENTER:
      sticky = tk.W+tk.E
    elif justify == tk.LEFT:
      sticky = tk.W
    elif justify == tk.RIGHT:
      sticky = tk.E
    else:
      sticky = tk.N

    textList = text.split('\n')
    for text in textList:
      w = tk.Label(master, fg=gt.ColorBlue, justify=justify, text=text,
          font=gt.FontCour10Bold)
      w.grid(row=row, column=column, padx=3, pady=0, sticky=sticky)
      row += 1
    return row

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
    """ GuiDlgAbout Test Main """
    root = tk.Tk()
    images = ['/prj/fusion/Fusion-1.0/Fusion/Gui/Images/RNRLogo.gif', None]
    for image in images:
      dlg = GuiDlgAbout(root,
                      name='vZeus',
                      version='1000000.0',
                      mimeType='god/ceo',
                      descTitle='Zeus the Humanizer',
                      desc="""Well, I am of course the king of the greek gods.
But did you know that I am also the god of love? That's right.
I loved women and men equally. Okay, I loved swans, cows, and
other creatures too. But now I am replaced by the true
God of Love.""",
                      copyright='RoadNarrows LLC\n(C) 2005',
                      logoImage=image)

  # run test
  main()
