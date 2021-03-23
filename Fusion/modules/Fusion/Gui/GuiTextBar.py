################################################################################
#
# GuiTextBar.py
#

""" Graphical User Interface TextBar Module

Graphical User Interface (GUI) Tkinter textbar module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.08

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
import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

NoMaxHistory  = 0

#-------------------------------------------------------------------------------
# CLASS: GuiTextBar
#-------------------------------------------------------------------------------
class GuiTextBar:
  """ GUI TextBar Class

      The TextBar is a wrapper class around the Tkinter.Text widget
      to provide simple, auto-configuration operation in a wide
      set of applications including: prompt lines, text histories,
      file contents, text widnows, and shell windows.
  """

  #--
  def __init__(self, parent, width=80, height=1, maxHistory=100,
                     fontTuple=None, lineNums=True, restState=tk.DISABLED,
                     wrap=tk.NONE):
    """ Initialize the TextBar.

        Parameters:
          parent      - parent GUI object
          width       - textbar view width in number of text characters 
          height      - statubar view height in number of text lines
          maxHistory  - maximum number of lines to hold in textbar
                          NoMaxHistory(0) = no maximum
          fontTuple   - 3 tuple of (familiy, size, style)
          lineNums    - do [not] number lines
          restState   - 'resting' state of textbar.
          wrap        - wrap method. One of: NONE, CHAR, WORD
    """
    # text widget width and height in characters 
    if width < 1:
      width = 1
    if height < 1:
      height = 1

    if maxHistory <= 0:
      maxHistory = NoMaxHistory

    # to scroll or not to scroll
    if maxHistory == NoMaxHistory or maxHistory > height:
      haveScrolling = True
    else:
      maxHistory    =  height
      haveScrolling = False

    # text font type
    if fontTuple:
       self.mFontTuple = fontTuple
    else:
       self.mFontTuple = gt.FontHelv10Bold

    self.mMaxHistory  = maxHistory    # maximum lines in history
    self.mLineNums    = lineNums      # do [not] number lines
    self.mRestState   = restState     # Text widget 'rest' state
    self.mLineCnt     = 1             # next line count

    # draw body
    self.body(parent, width, height, haveScrolling, wrap)

  #--
  def body(self, parent, width, height, haveScrolling, wrap):
    """ Draw dialog.

        Parameters:
          parent        - parent GUI object
          width         - textbar view width in number of text characters 
          height        - statubar view height in number of text lines
          haveScrolling - textbar does [not] have scrolling
          wrap          - wrap method
    """
    self.mFont = tkFont.Font(parent, font=self.mFontTuple)

    # core attributes
    kwargs = {
      'font':self.mFont,
      'exportselection':True,
      'wrap':wrap,
      'width':width,
      'height':height,
      'state':self.mRestState
    }
    
    if haveScrolling:
      self.mVScrollBar = tk.Scrollbar(parent, orient=tk.VERTICAL)
      kwargs['yscrollcommand'] = self.mVScrollBar.set
      if wrap == tk.NONE:
        self.mHScrollBar = tk.Scrollbar(parent, orient=tk.HORIZONTAL)
        kwargs['xscrollcommand'] = self.mHScrollBar.set

    self.mText = tk.Text(parent, kwargs)
    self.mText.grid(row=0, column=0, sticky=tk.W+tk.N+tk.S+tk.E)

    if haveScrolling:
      self.mVScrollBar.config(command=self.mText.yview)
      self.mVScrollBar.grid(row=0, column=1, sticky=tk.W+tk.N+tk.S)
      if wrap == tk.NONE:
        self.mHScrollBar.config(command=self.mText.xview)
        self.mHScrollBar.grid(row=1, column=0, sticky=tk.W+tk.N+tk.E)

    self.mText.tag_config('prompt', foreground=gt.ColorFgStatusPrompt)
    self.mText.tag_config('normal', foreground=gt.ColorFgStatusOk)
    self.mText.tag_config('error', foreground=gt.ColorFgStatusError)

  #--
  def GetTextWidget(self):
    """ Get the text widget associated with this textbar.
        
        Return Value:
          Tkinter.Text().
    """
    return self.mText

  #--
  def TagAdd(self, tag, **kw):
    """ Add display attribute tag.
        
        Parameters:
          tag - tag name
          kw  - dictionary of keyword,value pairs

        Return Value:
          None.
    """
    self.mText.tag_config(tag, **kw)

  #--
  def TagDel(self, tag):
    """ Delete display attribute tag.
        
        Parameters:
          tag - tag name

        Return Value:
          None.
    """
    self.mText.tag_delete(tag)

  #--
  def TextAdd(self, text, tags=()):
    """ Add text to the end of the TextBar.
        
        Parameters:
          text - text string
          tags - display attribute tag or tags tuple

        Return Value:
          None.
    """
    self.mText['state'] = tk.NORMAL
    self.mText.insert(tk.END, text, tags)
    self.mText['state'] = self.mRestState
    self._clearOldHist()
    if not self.mText.tag_nextrange(tk.SEL, 1.0): # only move if no selection
      self.mText.see(tk.END)

  #--
  def TextDump(self):
    """ Dump window text into a string.

        Return Value:
          A string of the concatenated text including '\n''s.
    """
    tupList = self.mText.dump(1.0, tk.END, text=True)
    text = ''
    for tup in tupList:
      text += tup[1]
    return text

  #--
  def EditCopy(self):
    """ Edit-Copy text selection into clipboard. """
    range = self.mText.tag_nextrange(tk.SEL, 1.0)
    if range:
      start = range[0]
      stop = range[1]
      self.mText.clipboard_clear()
      self.mText.clipboard_append(self.mText.get(start, stop))

  #--
  def EditPaste(self):
    """ Edit-Paste clipboard text at current cursor position. """
    text = self.mText.selection_get(selection='CLIPBOARD')
    if text:
      self.TextAdd(text)

  #--
  def EditCut(self):
    """ Edit-Cut text selection. """
    self.EditCopy()
    self.mText['state'] = tk.NORMAL
    self.mText.delete(tk.SEL_FIRST, tk.SEL_LAST)
    self.mText['state'] = self.mRestState

  #--
  def EditSelectAll(self):
    """ Edit-SelectAll text. """
    self.mText.tag_remove(tk.SEL, 1.0, tk.END)
    self.mText.tag_add(tk.SEL, 1.0, tk.END)

  #--
  def CbSelection(self, event):
    """ Text selection callback to place selected text into clipboard.

        To install callback:
          self.mText.bind("<ButtonRelease-1>", self.CbSelection)
    """
    range = self.mText.tag_nextrange(tk.SEL, 1.0)
    if range:
      start = range[0]
      stop = range[1]
      self.mText.clipboard_clear()
      self.mText.clipboard_append(self.mText.get(start, stop))

  #--
  def configure(self, units='text', width=None, height=None):
    """ Configure new TextBar width and height.
        
        Note: The width, height dimensions are to size the text widget.
              the calling routine must make allowances for any scroll
              bars in the parent window.

        Parameters:
          units   - dimension units: one of: 'text', 'pixels'
          width   - new width in the given units. Default: current width
          height  - new height in the given units. Default: current height

        Return Value:
          New width and height in 'text' units.
    """
    curWidth = int(self.mText['width'])
    curHeight = int(self.mText['height'])
    if units == 'text':
      if not width:
        newWidth = curWidth
      else:
        newWidth = width
      if not height:
        newHeight = curHeight
      else:
        newHeight = height
    elif units == 'pixels':
      textgeo = gut.geometry(self.mText)
      if not width:
        newWidth = curWidth
      else:
        # pixels/charwidth
        pixperwchar = float(textgeo[0])/float(curWidth)
        newWidth = int(float(width) / pixperwchar)
      if not height:
        newHeight = curHeight
      else:
        # pixels/charheight
        pixperhchar = float(textgeo[1])/float(curHeight)
        newHeight = int(float(height) / pixperhchar)
    else:
      raise ValueError('unknown units: %s' % repr(units))
    if curWidth != newWidth:
      self.mText['width'] = newWidth
    if curHeight != newHeight:
      self.mText['height'] = newHeight
    return newWidth, newHeight

  #--
  def ShowPrompt(self, pmsg):
    """ Show prompt text.
        
        Parameters:
          pmsg - prompt message string

        Return Value:
          None.
    """
    self._statusAddLine('> %s' % pmsg, tag='prompt')

  #--
  def ShowNormalStatus(self, msg):
    """ Show normal status text message.
        
        Parameters:
          msg - message string

        Return Value:
          None.
    """
    self._statusAddLine(msg, tag='normal')

  #--
  def ShowErrorStatus(self, emsg):
    """ Show error status text message.
        
        Parameters:
          emsg - error message string

        Return Value:
          None.
    """
    self._statusAddLine('Error: %s' % emsg, tag='error')

  #--
  def ShowStatus(self, msg, tag):
    """ Show status text message with the given attribute tag.
        
        Parameters:
          msg - message string
          tag - tag name

        Return Value:
          None.
    """
    self._statusAddLine(msg, tag)

  #--
  def _statusAddLine(self, text, tag=gt.ColorFgStatusOk):
    """ The workhorse for the status text functions.

        Parameters:
          text  - status text with no '\n' characters
          tag   - status display attribute tag

        Return Value:
          None.
    """
    self.mText['state'] = tk.NORMAL
    row, col = self.rowcol(self.textendpoint())
    if col > 0:
      self.mText.insert(tk.END, '\n')
    if self.mLineNums:
      self.mText.insert(tk.END, '%0*d: %s' % (6, self.mLineCnt, text), tag)
    else:
      self.mText.insert(tk.END, '%s' % (text), tag)
    self.mText['state'] = self.mRestState
    self.mLineCnt += 1
    self._clearOldHist()
    if not self.mText.tag_nextrange(tk.SEL, 1.0): # only move if no selection
      self.mText.see(tk.END)

  #--
  def _clearOldHist(self):
    """ Clear any old history (text) that exceeds the maximum history. """
    if self.mMaxHistory == NoMaxHistory:
      return
    row, col = self.rowcol(tk.END)
    if row-1 > self.mMaxHistory:
      row = row - self.mMaxHistory
      idxDel = '%d.0' % row
      self.mText['state'] = tk.NORMAL
      self.mText.delete(1.0, idxDel)
      self.mText['state'] = self.mRestState

  #--
  def textendpoint(self):
    """ Return the true end of text index. """
    # the end index maps to one line past the very last text line
    sIdxEnd = self.mText.index(tk.END)
    # back up one line to the last text line
    try:
      iIdxLastLine = int(float(sIdxEnd)) - 1
    except ValueError:      # Windows version sometimes goes nuts here
      return '1.0'
    # now return index (string) of the end of text in that last line
    return self.mText.index('%d.0lineend' % iIdxLastLine)

  #--
  def lineendpoint(self, index):
    """ Return the end of line index at the given index. """
    sIdx = self.mText.index(index)
    try:
      sRow, sCol = sIdx.split('.')
    except ValueError:      # Windows version sometimes goes nuts here
      print('Trace: ValueError: %s' % repr(sIdx))
      sRow = '0'
    return self.mText.index('%s.0lineend' % sRow)
    
  #--
  def rowcol(self, index):
    """ Convert index into integer 2-tuple (row, column). """
    sIdx = self.mText.index(index)
    try:
      sRow, sCol = sIdx.split('.')
    except ValueError:      # Windows version sometimes goes nuts here
      print('Trace: ValueError: %s' % repr(sIdx))
      return 2, 0
    try:
      row = int(sRow)
      col = int(sCol)
      return row, col
    except ValueError:
      print('Trace: ValueError: %s' % repr(sIdx))
      return 2, 0

  #--
  def _textDelLine(self, tag):
    range = self.mText.tag_nextrange(tag, 1.0)
    if range:
      self.mText['state'] = tk.NORMAL
      self.mText.delete(range[0], range[1])
      self.mText['state'] = self.mRestState
      self.mLineCnt -= 1

  #--
  def _textDump(self):
    """ Dump all tagged text. """
    for i in range(self.mTagStart, self.mTagEnd+1):
      self._textPrintTagged(i)

  #--
  def _textPrintTagged(self, tag):
    """ Print all text with the given tag. """
    ranges = self.mText.tag_ranges(tag)
    for i in range(0, len(ranges), 2):
      start = ranges[i]
      stop = ranges[i+1]
      print('tag %s: %s' % (tag, repr(self.mText.get(start, stop))))


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  #--
  def main():
    """ GuiTextBar Unit Test Main """
    root = tk.Tk()
    tb = GuiTextBar(root, width=30, height=4, maxHistory=10)
  
    tb.TagAdd("blue", foreground='blue')
    tb.TagAdd("black", foreground='black')
    tb.TagAdd("red", foreground='red')
    tb.TagAdd("green", foreground='#009900')
    tb.TagAdd("orange", foreground='#996600')
    tb.TagAdd("taggy", background="yellow", foreground="red")
  
    tb.ShowStatus('BLACK BERRIES', tag='black')
    tb.ShowStatus('gREEN TOMATOS', tag='green')
    tb.ShowStatus('BLUE BERRIES', tag='blue')
    tb.ShowStatus('RED BEETS', tag='red')
    tb.ShowStatus('ORANgE pERSSIMONS', tag='orange')
    tb.ShowStatus('BANANA', tag='taggy')
  
    tb.ShowNormalStatus('This line is displaying a normal status message.')
    tb.ShowErrorStatus('And this line is displaying an error status message.')
    tb.ShowPrompt('Ready to exit')
  
    for i in range(9, 12):
      tb.TextAdd('line %d\n' % i)

    tb.configure(width=40, height=8)

    root.mainloop()

  # run unit test
  main()
