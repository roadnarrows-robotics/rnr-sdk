################################################################################
#
# GuiWin.py
#

""" Graphical User Interface Fusion Child Window Base Class

Graphical User Interface (GUI) Tkinter Fusion child window base class.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.01.15

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
import  tkFont
import  Queue
import  threading as thread
import  time

import  Fusion.Core.Values as Values

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiEvent as GuiEvent
import  Fusion.Gui.GuiMenuBar as GuiMenuBar
import  Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# Window update server queue size (number of items)
WinServerQueueSize  = 8

#
# Builtin Window Requests
#
WinRequestDie     = '##DIE##'     # kill server thread
WinRequestResize  = '##RESIZE##'  # resize window


#-------------------------------------------------------------------------------
# CLASS: GuiWin
#-------------------------------------------------------------------------------
class GuiWin(tk.Toplevel):
  """ GUI Base Window Class """

  #--
  def __init__(self, parent, **options):
    """ Initialize the Gui Base Window.

        Parameters:
          parent    - GUI parent of this window
          options   - GuiWin core keyword options. Options are:
            title=<str>           - Title of this window.
                                    DEFAULT: 'Fusion Window'
            ondestroy=<func>      - Callback when this window is destroyed.
                                    DEFAULT: None
            geometry=<str>        - '<w>x<h>+<x>+<y>' desired geometry of this
                                    window. DEFAULT: ''
            create_server=<bool>  - Do [not] create request server thread.
                                    DEFAULT: True (create)
            qsize=<int>           - Size of server queue.
                                    DEFAULT: 8
  
            winid        - unique window id string
    """
    # defaults
    self.mTitle     = 'Fusion Window'
    self.mWinId     = None
    self.mOnDestroy = None
    geometry        = ''
    create_server   = True
    self.mQSize     = WinServerQueueSize

    # set options
    for k,v in options.iteritems():
      if k == Values.FusionCWinKeyOnDestroy:
        self.mOnDestroy = v
      elif k == Values.FusionCWinKeyTitle:
        self.mTitle = v
      elif k == Values.FusionCWinKeyGeometry:
        geometry = v
      elif k == Values.FusionCWinKeyWinId:
        self.mWinId = v
      elif k == Values.FusionCWinKeyCreateServer:
        create_server = v
      elif k == Values.FusionCWinKeyQSize:
        self.mQSize = v

    # create toplevel window with the title
    tk.Toplevel.__init__(self, master=parent)
    self.wm_title(self.mTitle)
    self.withdraw()  # hide the window until we know the geometry

    # window destroy event
    self.protocol('WM_DELETE_WINDOW', self.destroy)

    # draw gui body (invisibly)
    self.body()

    # force idletask to determine size, etc.
    self.update_idletasks()

    # offset this window from parent and show
    self.mWinGeo = geo = gut.geometry(parent)
    self.wm_geometry('+%d+%d' % (geo[2]+geo[0]/2, geo[3]+50))
    self.deiconify()

    # show and tell time
    self.show()

    # bind resizer
    self.bind('<Configure>', self.resize)

    # set new geometry 
    if geometry:
      self.wm_geometry(geometry)

    # public attributes
    self.isAlive      = True  # use to check if window still exists
    self.result       = None  # use if results are of window are returned
    self.mHasResized  = False # window has [not] resized
    
    # private attributes
    self.mHasQMeter   = False # no request queue monitoring meter
    if create_server:         # create window request server
      self.mContextName = self.mTitle.replace(' ', '') + '_Server'
      self._WinServerCreate()
    else:                     # no request server
      self.mContextName = self.mTitle.replace(' ', '') + '_Caller'
      self.mEvent       = None  # no GUI Event object
      self.mWinQueue    = None  # no request queue
      self.mWinThread   = None  # no request server thread

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    pass

  #--
  def show(self):
    """ Show the gui window initialization callback. """
    pass

  #--
  def resize(self, event):
    """ Resize callback event.

        Note: This Tkinter callback is call many times with the same
              geometry, so moderate the resizing action.
    """
    geo = gut.geometry(self)
    # window size has changed, clear but don't redraw until resizing is done
    if geo[0] != self.mWinGeo[0] or geo[1] != self.mWinGeo[1]:
      self.mWinGeo = geo
      self.mHasResized = True   # window has resized
    # resizing done, now redraw
    elif self.mHasResized:
      self.mHasResized = False   # window has not resized

  #--
  def destroy(self, **options):
    """ Destroy window callback event.

        Override this function to pass in specific window options to
        pass to any registered ondestroy callback.

        Parameters:
          **options   - keyword options passed back to any registered
                        ondestroy callback.
    """
    #print('Dbg: %s: destroy()' % self.mContextName)
    self.isAlive = False                  # this window is now defunct
    if self.mWinThread and self.mWinThread.isAlive():  # kiil thread
      self.WinQueueRequest(WinRequestDie) # queue death
    self._ondestroy(**options)            # make any ondestroy callback
    while self.mWinThread and self.mWinThread.isAlive():  # RDK hack busy loop
      self.WinQueueRequest(WinRequestDie) # queue death
    self._destroythis()                   # destroy this Tkinter window

  #--
  def _ondestroy(self, **options):
    """ On destroy, inform parent with options to save. """
    if self.mOnDestroy: # registered callback present
      # add geometry
      options[Values.FusionCWinKeyGeometry] = self.winfo_geometry()
      # make callback
      self.mOnDestroy(self.mWinId, self, **options)

  #--
  def _destroythis(self):
    """ Wrapper to simply destroy this window.
        
        Note: This function must be run in a Tkinter mainloop() context.
    """
    try:
      tk.Toplevel.destroy(self)
    except tk.TclError:    # already destroyed
      pass


#-------------------------------------------------------------------------------
# Window Update Reqeust Member Functions
#-------------------------------------------------------------------------------

  #--
  def _WinServerCreate(self):
    """ Create the update request server thread. """
    self.mEvent     = GuiEvent.GuiEvent(self)  # GUI Event object (not used)
    self.mWinQueue  = Queue.Queue(self.mQSize) # request queue
    self.mWinThread = thread.Thread(target=self._WinServer,
                                    name=self.mContextName, kwargs={})
    self.mWinThread.start()

  #--
  def _WinServer(self, args=(), kwargs={}):
    """ The Window Update Request Server thread.  """
    # process request loop
    #print("Dbg: Starting %s" % self.mWinThread.getName())
    while self.isAlive: 
      try:
        qargs, qkwargs = self.mWinQueue.get(True, 0.5)  # block with timeout
      except Queue.Empty:
        continue
      if len(qargs) > 0 and qargs[0] == WinRequestDie:       # die request
          #print("Dbg: Breaking %s" % self.mWinThread.getName())
          break;
      elif len(qargs) > 0 and qargs[0] == WinRequestResize:  # resize request
          #self.OnResize()    # RDK future
          pass                # RDK for now
      elif self.isAlive:
        if self.mHasQMeter:
          self._ShowQMeter(self.mWinQueue.qsize())
        self.WinUpdate(*qargs, **qkwargs)               # service request
    #print("Dbg: Quitting %s" % self.mWinThread.getName())

  #--
  def WinQueueRequest(self, *qargs, **qkwargs):
    """ Queue update request for window server thread to process.

        Execution Context: Calling thread

        Parameters:
          *qargs    - queued WinUpdate() arguments
          *qkwargs  - queued WinUpdate() keyword=value arguments

        Return Value:
          True if the request was successfully queued.
          False if the request could not be queued.
    """
    #print("Dbg: %s: WinQueueRequest(%s...)" % \
    #    (self.mContextName, len(qargs[0])>0 and repr(qargs[0]) or ''))
    if not self.mWinThread or not self.mWinThread.isAlive():
      return
    try:
      self.mWinQueue.put((qargs, qkwargs), True, 0.1) # try queueing w/ timeout
      if self.mHasQMeter:
        self._ShowQMeter(self.mWinQueue.qsize())
      return True
    except Queue.Full:
      if self.mHasQMeter:
        self._ShowQMeter(self.mWinQueue.qsize()+1)
      #print("Dbg: %s: WinQueueRequest(...): Full" % self.mContextName)
      return False

  #--
  def WinUpdate(self, *qargs, **qkwargs):
    """ Process window update request. 

        Override this function in derived class for derived window specific
        updates.

        Execution Context: GuiWin server thread iff scheduled from
                           WinQueueRequest().

        Notes:
          * This function is not thread-safe.
          * If the Tkinter.mainloop() is blocked and WinUpdate() makes 
            Tkinter calls, then this function will also block.

        Unless the derived window is very simple, call the function
        indirectly by starting the window's request server and queueing
        update requests via WinQueueRequest().

        Parameters:
          *qargs    - WinUpdate() arguments (window specific)
          *qkwargs  - WinUpdate() keyword=value arguments (window specific)

        Return Value:
          None
    """
    #print("Dbg: %s: WinUpdate(...)" % (self.mContextName))
    pass
 
  #--
  def CreateQMeter(self, parent, row=0, column=0):
    """ Create optional request queue monitoring meter. If this meter
        is created, then request queue puts/gets will be automatically
        displayed on the parent widget.

        Parameters:
          parent  - contruct the queue meter widget with the given parent
          row     - grid row
          column  - grid column

        Return Value:
          None
    """
    width   = 6 + self.mQSize * 12 +  5
    height  = 16
    w = tk.Canvas(parent, width=width, height=height)
    w.grid(row=row, column=column)
    self.mMeterWidget = w
    self.mMeterGids = []

    # set color bands
    self.mMeterColors = [gt.ColorGreenDark] # queue empty
    i = 1.0
    while i <= self.mQSize:
      if i <= self.mQSize * 0.25:
        self.mMeterColors.append('#00cc00') # medium green
      elif i <= self.mQSize * 0.50:
        self.mMeterColors.append('#99ff00') # yellowish green
      elif i <= self.mQSize * 0.75:
        self.mMeterColors.append('yellow')  # yellow
      else:
        self.mMeterColors.append('#ffcc44') # yellowish orange
      i += 1.0
    self.mMeterColors.append('red')         # queue overflow

    # pre-build canvas items
    x0, y0, y1 = 1, 1, height-1
    for color in self.mMeterColors:
      if color in [gt.ColorGreenDark, 'red']:
        x1 = x0 + 4
      else:
        x1 = x0 + 10
      self.mMeterGids += [self.mMeterWidget.create_rectangle(x0, y0, x1, y1,
                                           outline='black')]
      x0 = x1 + 2

    # show empty queue
    self.mHasQMeter = True
    self._ShowQMeter(0)
 
  #--
  def _ShowQMeter(self, nQueued):
    """ Show the queue monitoring meter at the given queued compacity.
        This function is automatically called by the queue put/get
        window functions.

        Parameters:
          nQueued   - number of queued items in request queue.

        Return Value:
          None
    """
    i = 0
    while i < len(self.mMeterColors):
      if i <= nQueued:
        color = self.mMeterColors[i]
      else:
        color = 'gray'
      self.mMeterWidget.itemconfigure(self.mMeterGids[i], fill=color)
      i += 1
 

#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import sys
  import time
  import random
  
  #--
  def killroot(root):
    try:
      root.destroy()
      pass
    except tk.TclError:
      print('TclError: root.destroy()', file=sys.stderr)
      pass

  #--
  def fakequeueing(win, count):
    """ may cause errors, but okay for unit testing """
    try:
      if win.isAlive:
        win._ShowQMeter(count)
    except tk.TclError:
      print('TclError: _ShowQMeter()', file=sys.stderr)
      pass
    count = (count + 1) % (WinServerQueueSize + 2)
    try:
      if win.isAlive:
        win.AlarmFakeId = win.after(1000, fakequeueing, win, count)
    except tk.TclError:
      print('TclError: win.after()', file=sys.stderr)
      pass

  #--
  def main(killtime=None):
    """ GuiWin Unit Test Main """
    global root
    root = tk.Tk()
    if killtime:
      root.after(killtime, killroot, root)
    win = GuiWin(root)
    win.CreateQMeter(win, 0, 0)
    win.AlarmFakeId = win.after(1000, fakequeueing, win, 0)
    print("Unit Test: Cycle thru QMeter forever until window is destroyed")
    root.mainloop()

  #--
  def mainloop():
    while True:
      try:
        raw_input("Press <Enter> or <Ctrl-D> ")
      except EOFError:
        print("<EOF>")
        break
      t = random.randint(500, 3000)
      print("\n*** Create GuiWin for %dmsec***" % t)
      main(killtime=t)

  # run unit test
  #mainloop()
  main()
