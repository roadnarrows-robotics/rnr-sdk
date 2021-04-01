#
# Test Tkinter mainloop() interactions
#

import tkinter as tk
import time
import threading as thread
import random
import queue

import Fusion.Gui.GuiUtils as gut

#
# Builtin Window Requests
#
WinRequestDie = '##DIE##'

class GuiWin(tk.Toplevel):

  #--
  def __init__(self, parent, **options):
    """ Initialize the Gui Base Window.

        Parameters:
          parent    - GUI parent of this window
          options   - GuiWin core keyword options. Options are:
            title=<str>   - title of this window
            server_create=<bool> - do [not] create request server thread.
                            DEFAULT: True
    """
    # defaults
    title           = 'Fusion Window'
    geometry        = ''
    server_create   = True


    # set options
    for k,v in options.items():
      if k == 'title':
        title = v
      elif k == 'geometry':
        geometry = v
      elif k == 'geometry':
        server_create = v

    # create toplevel window with the title
    tk.Toplevel.__init__(self, master=None)

    self.wm_title(title)
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
    self.isAlive   = True    # use to check if window still exists
    self.result    = None    # use if results are of window are returned

    # create window request server
    if server_create:
      self.mContextName = title.replace(' ', '') + '_Server'
      self._WinServerCreate()
    else:
      self.mContextName = title.replace(' ', '') + '_Caller'

  #--
  def _WinServerCreate(self):
    """ Create the Tkinter mainloop() thread. """
    self.mWinQueue  = queue.Queue(4)
    self.mWinThread = thread.Thread(target=self._WinServer,
                                    name=self.mContextName, kwargs={})
    self.mWinThread.start()

  #--
  def _WinServer(self, args=(), kwargs={}):
    """ Window Server thread.  """
    print("Starting %s" % self.mWinThread.getName())
    while self.isAlive: 
      try:
        qargs, qkwargs = self.mWinQueue.get(True, 0.5)  # block with timeout
      except queue.Empty:
        continue
      if len(qargs) > 0 and qargs[0] == WinRequestDie:
        break;
      else:
        self.ShowMeter(self.mWinQueue.qsize())
        self.WinUpdate(*qargs, **qkwargs) 
    print("Quitting %s" % self.mWinThread.getName())

  #--
  def WinQueueRequest(self, *qargs, **qkwargs):
    try:
      self.mWinQueue.put((qargs, qkwargs), True, 0.1)
      self.ShowMeter(self.mWinQueue.qsize())
      return True
    except queue.Full:
      self.ShowMeter(self.mWinQueue.qsize()+1)
      return False

  #--
  def WinUpdate(self, *qargs, **qkwargs):
    """ Not thread safe... """
    obj = qargs[0]
    color = qkwargs.get('color', 'white')
    print("%s: WinUpdate(%s,%s)" % (self.mContextName, obj, color))
    if obj == 'oval':
      self.CanvasOval(color=color)
    elif obj == 'circle':
      self.CanvasCircle(color=color)
    elif obj == 'square':
      self.CanvasSquare(color=color)
    elif obj == 'lines':
      self.CanvasLines(color=color)
    elif obj == 'all':
      self.CanvasErase()
      self.CanvasOval()
      self.CanvasCircle()
      self.CanvasSquare()
      self.CanvasLines()
    elif obj == 'erase':
      self.CanvasErase()
    else:
      print("%s: unknown object: %s" % (self.mContextName, obj))

  #--
  def body(self):
    """ Initialize the gui window initialization callback. """
    pass

  #--
  def show(self):
    """ Show the gui window initialization callback. """
    self.mCanvasW = 300
    self.mCanvasH = 100
    w = tk.Canvas(self, width=self.mCanvasW, height=self.mCanvasH, bg='#999999')
    w.grid(row=0, column=0, columnspan=4)
    self.mCanvas = w
    self.mGidList = []

    w = tk.Canvas(self, width=59, height=16)
    w.grid(row=1, column=0)
    self.mMeterWidget = w
    self.mMeterGids = []
    self.mMeterColors = ['#009900', 'green', 'yellowgreen', 'yellow', 'orange',
                         'red']
    x0, y0, y1 = 1, 1, 15
    for color in self.mMeterColors:
      if color in ['#009900', 'red']:
        x1 = x0 + 4
      else:
        x1 = x0 + 10
      self.mMeterGids += [self.mMeterWidget.create_rectangle(x0, y0, x1, y1,
                                           outline='black')]
      x0 = x1 + 2
    self.ShowMeter(0)

    w = tk.Button(self, text='Clear', command=self.CbClear)
    w.grid(row=1, column=1)
    w = tk.Button(self, text='DrawAll', command=self.CbDrawAll)
    w.grid(row=1, column=2)
    w = tk.Button(self, text='Blast', command=self.CbBlast)
    w.grid(row=1, column=3)

  #--
  def resize(self, event):
    """ Resize callback event. """
    geo = gut.geometry(self)
    if geo[0] != self.mWinGeo[0] or geo[1] != self.mWinGeo[1]:
      self.mWinGeo = geo

  #--
  def destroy(self):
    """ Destroy window callback event. """
    print('Destroying', self.wm_title())
    self.isAlive = False
    self.WinQueueRequest(WinRequestDie)
    self.ondestroy()
    self.destroythis()

  #--
  def ondestroy(self, **options):
    """ On destroy, inform parent with options to save. """
    pass

  #--
  def destroythis(self):
    """ Wrapper to simply destroy this window. """
    try:
      tk.Toplevel.destroy(self)
    except tk.TclError:    # already destroyed
      pass

  #--
  def ShowMeter(self, nQueued):
    i = 0
    while i < len(self.mMeterColors):
      if i <= nQueued:
        color = self.mMeterColors[i]
      else:
        color = 'gray'
      self.mMeterWidget.itemconfigure(self.mMeterGids[i], fill=color)
      i += 1

  #--
  def CanvasErase(self):
    for gid in self.mGidList:
      self.mCanvas.delete(gid)
    self.mGidList = []

  #--
  def CanvasOval(self, color='blue'):
    x1 = self.mCanvasW-1
    y1 = self.mCanvasH-1
    gid = self.mCanvas.create_oval(1, 1, x1, y1, fill=color)
    self.mGidList += [gid]

  #--
  def CanvasCircle(self, color='red'):
    r = (self.mCanvasH-1)/2
    x0 = self.mCanvasW/2 - r 
    y0 = self.mCanvasH/2 - r
    x1 = x0 + 2 * r
    y1 = y0 + 2 * r
    gid = self.mCanvas.create_oval(x0, y0, x1, y1, fill=color)
    self.mGidList += [gid]

  #--
  def CanvasSquare(self, color='green'):
    l = 10
    x0 = random.randint(0, self.mCanvasW-l-1)
    y0 = random.randint(0, self.mCanvasH-l-1)
    x1 = x0 + l
    y1 = y0 + l
    gid = self.mCanvas.create_rectangle(x0, y0, x1, y1, fill=color)
    self.mGidList += [gid]

  #--
  def CanvasLines(self, color='pink'):
    i = 0
    while i < 10:
      x0 = random.randint(0, self.mCanvasW)
      y0 = random.randint(0, self.mCanvasH)
      x1 = random.randint(0, self.mCanvasW)
      y1 = random.randint(0, self.mCanvasH)
      gid = self.mCanvas.create_line(x0, y0, x1, y1, fill=color)
      self.mGidList += [gid]
      i += 1

  #--
  def CbBlast(self):
    print("%s: CbBlast" % self.wm_title())
    self.WinQueueRequest('erase')
    self.WinQueueRequest('oval')
    self.WinQueueRequest('circle')
    self.WinQueueRequest('square')
    self.WinQueueRequest('lines')

  #--
  def CbClear(self):
    print("%s: CbClear" % self.wm_title())
    self.WinQueueRequest('erase')

  #--
  def CbDrawAll(self):
    print("%s: CbDrawAll" % self.wm_title())
    self.WinQueueRequest('all')


#------------------------------
Root      = None
Win1      = None
AuxThread = None
AuxSema   = thread.Semaphore()
AuxCmd    = ''

def drawrequest():
  global Win1
  objList = ['oval', 'circle', 'square', 'lines', 'erase', 'all']
  colorList = ['red', 'orange', 'yellow', 'yellowgreen', 'green',
      'blue', 'cyan', 'magenta', 'pink', 'gray', 'white']
  obj = random.choice(objList)
  color=random.choice(colorList)
  print("drawrequest: %s, %s" % (obj, color))
  Win1.WinQueueRequest(obj, color=color)

def drawblast(n=20):
  print("** drawblast(%d) **" % n)
  i = 0
  while i < n:
    drawrequest()
    i += 1

def rootclose():
  global Root
  Root.destroy()

def startroot():
  global Root
  print('Starting root')
  Root = tk.Tk()
  Root.wm_title("root")
  w = tk.Button(Root, text='Send', command=drawrequest)
  w.grid(row=0, column=0)
  w = tk.Button(Root, text='Blast', command=drawblast)
  w.grid(row=0, column=1)
  w = tk.Button(Root, text='ThBlast', command=auxblast)
  w.grid(row=0, column=2)
  w = tk.Button(Root, text='Close', command=rootclose)
  w.grid(row=0, column=3)

def startaux():
  global AuxThread
  print('Starting AuxThread')
  AuxThread = thread.Thread(target=auxmain, name="AuxThread", kwargs={})
  AuxSema.acquire()
  AuxThread.start()

def auxmain(args=(), kwargs={}):
  global AuxSema, AuxCmd
  while True:
    AuxSema.acquire()
    if AuxCmd == 'blast':
      drawblast(50)
    elif AuxCmd == 'die':
      break

def auxblast():
  global AuxThread, AuxSema, AuxCmd
  AuxCmd = 'blast'
  AuxSema.release()

def auxquit():
  global AuxThread, AuxSema, AuxCmd
  AuxCmd = 'die'
  AuxSema.release()

startroot()
print('Starting window #1')
Win1 = GuiWin(Root, title="WinOne")
startaux()
print('Entering root.mainloop()')
Root.mainloop()
print("Root exit")
auxquit()
print("AuxThread exit")
