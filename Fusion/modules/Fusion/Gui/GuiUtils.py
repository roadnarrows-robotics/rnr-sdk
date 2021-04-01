################################################################################
#
# GuiUtils.py
#

""" Gui Utilities Module

General purpose and common utilites to support the GUI.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.31

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

import os
import stat
import threading as thread

import tkinter as tk

import Fusion.Core.Values as Values
import Fusion.Utils.Tools as utils
import Fusion.Gui.GuiTypes as gt


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# geometry indices
W     = 0   # width
H     = 1   # height
XOFF  = 2   # x offset from left
YOFF  = 3   # y offset from top


#-------------------------------------------------------------------------------
# Funtions and Classes
#-------------------------------------------------------------------------------

#--
def geometry(widget):
  """ Return widget's current geometry integer 4-tuple.

      Parameter:
        widget  - GUI widget

      Return Value:
        4-tuple (width, height, xoffset, yoffset)
  """
  return cvt_geometry(widget.winfo_geometry())

#--
def cvt_geometry(geostr):
  """ Convert geometry string to integer 4-tuple.

      Parameter:
        geostr  - Geometry string '(widthxheight+xoffset+yoffset)

      Return Value:
        4-tuple (width, height, xoffset, yoffset)
  """
  w = h = xoff = yoff = 0
  if not geostr:
    return (w, h, xoff, yoff)
  off = geostr.split('+')
  dim = off[0].split('x')
  if len(dim) == 2:     # ['width', 'height']
    w = int(dim[0])
    h = int(dim[1])
  if len(off) == 3:     # ['', 'xoffset', 'yoffset']
    xoff = int(off[1])
    yoff = int(off[2])
  return (w, h, xoff, yoff)

#--
def GetFusionImageFileName(imageFile):
  """ Get Fusion fully qualified image filename.

      Parameters:
        imageFile - tail component of image filename

      Return Value:
        Fully qualified image filename or None.
  """
  if Values.FusionImageDir:
    imageFile = utils.makefilename(Values.FusionImageDir, imageFile)
  else:
    return None
  try:
    mode = os.stat(imageFile)[stat.ST_MODE]
  except OSError as msg:
    return None
  if stat.S_ISREG(mode):
    return imageFile
  else:
    None

#-------------------------------------------------------------------------------
# CLASS: ImageWidget
#-------------------------------------------------------------------------------
class ImageWidget(tk.Canvas):
  """ Simple Image Widget Class """

  def __init__(self, master, imageFile, cnf={}, **kw):
    """ Initialize ImageWidget.

        Parameters:
          master    - master widget
          imageFile - image filename
          cnf       - Tkinter.Canvas widget standard cnf values
          **kw      - Tkinter.Canvas widget standard keyword argments

        Return Value:
          widget
    """
    # load and build the image
    self.mImg = tk.PhotoImage(file=imageFile)
    imgWidth  = self.mImg.width() + 2
    imgHeight = self.mImg.height() + 2

    # canvas widget to show the image
    tk.Canvas.__init__(self, master, cnf=cnf,
                  relief=tk.FLAT, borderwidth=0,
                  height=imgHeight, width=imgWidth,
                  **kw)
    self.create_image(1, 1, anchor=tk.NW, image=self.mImg)

# RDK!!! StatusLine()
# RDK!!! ShowStatus()

#-------------------------------------------------------------------------------
# CLASS: ActiveImageWidget
#-------------------------------------------------------------------------------
class ActiveImageWidget(tk.Canvas):
  """ Active Image Widget Class """

  #--
  def __init__(self, master, 
                     filenames=[], activesets={}, activetag=None, period=0.125,
                     cnf={}, **kw):
    """ Initialize the Active Image widget The display of the images in the
        specified list cycle at the given frames/second rate.

        Parameters:
          master      - master widget
          filenames   - list of image file path names
            or
          activesets  - dictionary of active image sets
                          entry format: [file, ...]
          activetag   - set active the image set marked by this tag 
          period      - seconds/frame period
          cnf         - Tkinter.Canvas widget standard cnf values
          **kw        - Tkinter.Canvas widget standard keyword argments
    """
    self.mImgSets         = {}
    self.mImgPeriod       = int(period * 1000) # millisecs
    self.mImgWidth        = 1
    self.mImgHeight       = 1

    if filenames:
      self._BuildImageSet('#DEFAULT#', filenames)
    for tag, imgset in activesets.items():
      self._BuildImageSet(tag, imgset)

    # fudge size for edges, find center
    self.mImgWidth  += 2
    self.mImgHeight += 2
    self.mCenterPt  = (self.mImgWidth/2, self.mImgHeight/2)

    # canvas widget to show the image
    tk.Canvas.__init__(self, master, cnf=cnf, 
                relief=tk.FLAT, borderwidth=0,
                height=self.mImgHeight, width=self.mImgWidth,
                **kw)

    # active image set tag
    if len(self.mImgSets) == 1: # only one set, so it must be the active set
      self.mActiveTag = list(self.mImgSets.keys())[0]
    elif activetag:             # active set specified in init
      self.mActiveTag = activetag
    else:                       # default to first set
      self.mActiveTag = list(self.mImgSets.keys())[0]

    self.mIndex     = 0                   # starting index
    self.mCurImg    = (None, -1)          # no currently displayed image
    self.mGidImg    = None                # no canvas graphics id
    self._mIterator = None                # iterator
    self.mAfterId   = self.after_idle(self._CbIterStart)

  #--
  def _BuildImageSet(self, tag, filenames):
    """ Build the image set(s) """
    self.mImgSets[tag] = []
    for filename in filenames:
      img = tk.PhotoImage(file=filename)
      width = img.width()
      height = img.height()
      self.mImgSets[tag] += [img]
      if width > self.mImgWidth:
        self.mImgWidth = width
      if height > self.mImgHeight:
        self.mImgHeight = height

  #--
  def destroy(self):
    """ Destroy this widget. """
    self.after_cancel(self.mAfterId)
    tk.Canvas.destroy(self)

  #--
  def SetActive(self, tag):
    """ Set active image set marked by the tag.

        Parameters:
          tag   - set tag

        Return Values:
          None
    """
    self.mActiveTag = tag

  def _CbIterStart(self):
    """ Start image sequencer iterator. 
    
        Note: This callback is called only at an idle time of the mainloop,
        guaranteeing widget validity.
    """
    self.mAfterId = self.after(self.mImgPeriod, self._CbIterNext)

  #--
  def _CbIterNext(self):
    """ Image sequencing iterator. """
    if self.mCurImg != (self.mActiveTag, self.mIndex):
      self.mCurImg = (self.mActiveTag, self.mIndex)
      # new active set can upset indexing, so check
      if self.mCurImg[1] >= len(self.mImgSets[self.mCurImg[0]]):
        self.mCurImg = (self.mCurImg[0], 0)
        self.mIndex = 0
      # delete old image from canvas
      if self.mGidImg is not None:
        try:
          self.delete(self.mGidImg)
        except RuntimeError:
          self.mGidImg = None
      # mainloop may not have been created or is in process of being destroyed
      # so protect call
      try:
        self.mGidImg = self.create_image(self.mCenterPt, anchor=tk.CENTER,
            image=self.mImgSets[self.mCurImg[0]][self.mCurImg[1]])
      except RuntimeError:
        pass
    # bump image set index
    self.mIndex = (self.mIndex + 1) % len(self.mImgSets[self.mCurImg[0]])
    self.mAfterId = self.after(self.mImgPeriod, self._CbIterNext)
