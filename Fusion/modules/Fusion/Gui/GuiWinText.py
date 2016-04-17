################################################################################
#
# GuiWinText.py
#

""" Graphical User Interface Generic Text Window

Graphical User Interface (GUI) Tkinter window to display and manipulate
a text window.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.26

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
import  tkFont

import  Fusion.Core.Values as Values

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiWin as GuiWin
import  Fusion.Gui.GuiMenuBar as GuiMenuBar
import  Fusion.Gui.GuiTextBar as GuiTextBar
import  Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: GuiWinText
#-------------------------------------------------------------------------------
class GuiWinText(GuiWin.GuiWin):
  """ GUI Text Window Class """

  #--
  def __init__(self, parent, title='Text Window',
                     width=80, height=25, 
                     maxHistory=4000, fontTuple=None, lineNums=True,
                     restState=tk.DISABLED, wrap=tk.NONE, **options):
    """ Initialize the Text Window.

        Parameters:
          parent      - GUI parent of this window
          width       - textbar view width in number of text characters 
          height      - statubar view height in number of text lines
          maxHistory  - maximum number of lines to hold in textbar
          fontTuple   - 3 tuple of (familiy, size, style)
          lineNums    - do [not] number lines
          restState   - 'resting' state of textbar.
          wrap        - wrap method. One of: NONE, CHAR, WORD
          options     - GuiWin core options
    """
    # text window options
    self.mTextOpt = {}
    self.mTextOpt['width']      = width
    self.mTextOpt['height']     = height
    self.mTextOpt['maxHistory'] = maxHistory
    self.mTextOpt['fontTuple']  = fontTuple
    self.mTextOpt['lineNums']   = lineNums
    self.mTextOpt['restState']  = restState
    self.mTextOpt['wrap']       = wrap

    # create the window
    options[Values.FusionCWinKeyTitle] = title
    GuiWin.GuiWin.__init__(self, parent, **options)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # GuiWin Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    self.mMenuBar     = GuiMenuBar.GuiMenuBar(self)
    
    txtframe = tk.Frame(self, relief=tk.RAISED, borderwidth=1)
    txtframe.grid(row=0, column=0, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.W+tk.E)
    self.mTextBar     = GuiTextBar.GuiTextBar(txtframe,
                          width=self.mTextOpt['width'],
                          height=self.mTextOpt['height'],
                          maxHistory=self.mTextOpt['maxHistory'],
                          fontTuple=self.mTextOpt['fontTuple'],
                          lineNums=self.mTextOpt['lineNums'],
                          restState=self.mTextOpt['restState'],
                          wrap=self.mTextOpt['wrap'])
    # menu bar items
    self._initMenuBar(self.mTextOpt['restState'])

  #--
  def show(self):
    """ Show the gui window initialization callback. """
    # calculate important window and widget dimensions used for resizing
    self.CalcDim()

    # put focus on text window
    self.mTextBar.mText.focus_set()

  #--
  def resize(self, event):
    """ Resize callback event. """
    geo = gut.geometry(self)
    if geo[gut.W] != self.mWinGeo[gut.W] or geo[gut.H] != self.mWinGeo[gut.H]:
      self.mTextBar.configure(units='pixels', 
        width=geo[gut.W]-self.mTextWOffset,
        height=geo[gut.H]-self.mTextHOffset)
      self.mWinGeo = geo


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Text Window Specifics
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # current window dimensions
    self.mWinGeo = gut.geometry(self)

    # text widget dimensions
    textgeo = gut.geometry(self.mTextBar.mText)

    # text widget offsets from window
    self.mTextWOffset = self.mWinGeo[0] - textgeo[0]
    self.mTextHOffset = self.mWinGeo[1] - textgeo[1]

  #--
  def _initMenuBar(self, restState):
    """ Initialize menubar. """
    # File menubar items
    self.mMenuBar.AddMenuItem('File', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('File|Save As...', 'command', owner='root',
        command=self.CbSaveAs)
    self.mMenuBar.AddMenuItem('File', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('File|Exit', 'command', owner='root',
        command=self.destroy)

    # Edit menubar items
    self.mMenuBar.AddMenuItem('Edit|Copy', 'command', owner='root',
        command=self.CbEditCopy)
    # this text window is editable
    if restState == tk.NORMAL:
      self.mMenuBar.AddMenuItem('Edit|Paste', 'command', owner='root',
          command=self.CbEditPaste)
      self.mMenuBar.AddMenuItem('Edit|Cut', 'command', owner='root',
          command=self.CbEditCut)
    self.mMenuBar.AddMenuItem('Edit', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('Edit|Select All', 'command', owner='root',
          command=self.CbEditSelectAll)

  #--
  def GetTextWidget(self):
    """ Get the text widget.
        
        Return Value:
          Tkinter.Text().
    """
    return self.mTextBar.mText

  #--
  def TagAdd(self, tag, **kw):
    """ Add display attribute tag.
        
        Parameters:
          tag - tag name
          kw  - dictionary of keyword,value pairs

        Return Value:
          None.
    """
    self.mTextBar.TagAdd(tag, **kw)

  #--
  def TagDel(self, tag):
    """ Delete display attribute tag.
        
        Parameters:
          tag - tag name

        Return Value:
          None.
    """
    self.mTextBar.TagDel(tag)

  #--
  def TextAdd(self, text, tags=()):
    """ Add text to the end of the TextBar.
        
        Parameters:
          text - text string
          tags - display attribute tag or tags tuple

        Return Value:
          None.
    """
    self.mTextBar.TextAdd(text, tags)

  #--
  def TextDump(self):
    """ Dump window text into a string.

        Return Value:
          A string of the concatenated text including '\n''s.
    """
    return self.mTextBar.TextDump()

  def Save(self, filename):
    """ Save window text as a flat text file.

        Parameters:
          filename  - filename to save text.

        Return Value:
          None.
    """
    fp = open(filename, 'w')
    fp.write(self.TextDump())
    fp.close()

  #--
  def CbSaveAs(self):
    """ Save As callback. """
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self, self.Save,
                  title='Save Text As',
                  filetypes=[('Text files', '*.txt', 'TEXT'),
                             ('All files', '*')],
                  defaultextension='.txt')

  #--
  def CbEditCopy(self):
    """ Edit|Copy callback. """
    self.mTextBar.EditCopy()

  #--
  def CbEditPaste(self):
    """ Edit|Paste callback. """
    self.mTextBar.EditPaste()

  #--
  def CbEditCut(self):
    """ Edit|Cut callback. """
    self.mTextBar.EditCut()

  #--
  def CbEditSelectAll(self):
    """ Edit|SelectAll callback. """
    self.mTextBar.EditSelectAll()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  #--
  def main():
    """ GuiWinText Unit Test Main """
    root = tk.Tk()
    win = GuiWinText(root, restState=tk.NORMAL)
    root.mainloop()

  # run unit test
  main()
