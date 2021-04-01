################################################################################
#
# WinUT.py
#

""" Unit Test Window Module

Simple and Handy Unit Test Window Harness for Fusion Module Unit
Testing.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.12.05

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

import tkinter as tk
import Fusion.Utils.IVTimer as IVTimer
import Fusion.Gui.GuiTextBar as GuiTextBar


#-------------------------------------------------------------------------------
# CLASS: WinUT
#-------------------------------------------------------------------------------
class WinUT:
  """ Handy and Simple Unit Test Window Harness Class.

      Derive from this class and add specific UT functions.
  """

  #--
  def __init__(self, title="Unit Test Window", ut={}):
    """ Initialize the Window.

        Parameters:
          title   - Title of this window.
          ut      - Unit test dictionary: {'Menu name': calback, ... }
    """
    if not ut:
      ut = {'<dummy test>': self.utDummyStart}

    root = tk.Tk()
    root.wm_title(title)
    mb = tk.Menubutton(root, text="Select a Unit Test", bg="#00cccc",
      relief=tk.RAISED)
    mb.grid(row=0, column=0, stick=tk.W)
    mb.menu = tk.Menu(mb, tearoff=0)
    for k,v in ut.items():
      mb.menu.add_command(label=k, command=v)
    mb.config(menu=mb.menu)
    b = tk.Button(root, text="Quit", bg="#990000", fg="#ffffff",
      command=root.destroy)
    b.grid(row=0, column=1, stick=tk.E)

    frame = tk.Frame(root, relief=tk.SUNKEN)
    frame.grid(row=1, column=0, columnspan=2, 
      padx=3, pady=5, sticky=tk.W+tk.E)
    self.mStatusPane = GuiTextBar.GuiTextBar(frame, width=100, height=4,
        maxHistory=1000)
    self.mStatusPane.TagAdd("blue", foreground='blue')
    self.mStatusPane.TagAdd("black", foreground='black')
    self.mStatusPane.TagAdd("red", foreground='red')
    self.mStatusPane.TagAdd("green", foreground='#009900')
    self.mStatusPane.TagAdd("orange", foreground='#996600')

    self.mRoot = root   # this Unit Test Window's 'widget'
    self.mSut = None    # System Under Test 
    self.mIvt = None    # handy interval timer

    self.wut_showstatus("Ready", fg='green')

  #--
  def wut_this(self):
    """ Return this UT window's widget. """
    return self.mRoot

  #--
  def wut_mark_sut(self, sut):
    """ Mark UT window's SUT window. """
    self.mSut = sut
    self.mRoot.tkraise()

  #--
  def wut_cancel(self):
    """ Cancel any unit test residules. """
    if self.mIvt:
      self.mIvt.cancel()

  #--
  def wut_showstatus(self, msg, fg='black'):
    """ Show UT status message. """
    self.mStatusPane.ShowStatus(msg, tag=fg)

  #--
  def utDummyStart(self):
    """ Dummy UT Start ."""
    self.wut_showstatus("Started dummy UT.")
    self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utDummyIter, cnt=0) 
    self.mIvt.start()

  #--
  def utDummyIter(self, ivt):
    """ Dummy UT Iterator. """
    self.wut_showstatus("Dummy UT: pass %d" % ivt.cnt)
    ivt.cnt += 1
 

#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import Fusion.Gui.GuiWinText as GuiWinText

  class MyWinUT(WinUT):
    def __init__(self):
      ut = {
          'My Test 1': self.utTest1Start,
          'My Test 2': self.utTest2Start
      }
      WinUT.__init__(self, title="My Unit Test Window", ut=ut)

    def utTest1Start(self):
      self.wut_showstatus("Nothing to run for my test 1", fg='red')

    def utTest2Start(self):
      self.wut_showstatus("Started my test 2.")
      self.mIvt = IVTimer.IVTimer(0.5, 0.5, self.utTest2Iter,
          firsttime=True, cnt=0) 
      self.mIvt.start()

    def utTest2Iter(self, ivt):
      if ivt.firsttime:
        msg = "First time for my test 2."
        ivt.firsttime = False
      else:
        msg = "my test 2, pass #%d" % ivt.cnt
      self.wut_showstatus(msg)
      if self.mSut:
        self.mSut.TextAdd(msg+'\n')
      ivt.cnt += 1

  #--
  def main(how='base'):
    """ WinUT Unit Test Main. """
    if how == 'sut':
      winUT = MyWinUT()
      winSut = GuiWinText.GuiWinText(winUT.wut_this(), title="Text Window UT")
      winUT.wut_mark_sut(winSut)
    elif how == 'derived':
      winUT = MyWinUT()
    else: # 'base':
      winUT = WinUT("WinUT Unit Test Window")
    winUT.wut_this().mainloop()
    winUT.wut_cancel()

  # run unit test
  #main(how='base')
  #main(how='derived')
  main(how='sut')
