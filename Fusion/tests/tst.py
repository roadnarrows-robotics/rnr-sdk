import Tkinter as tk
import tkFont
import os
import sys
import time
import Fusion.Khepera.Robots.vKhepera as vk
import Fusion.Core.Reactor as fusion
import Fusion.Gui.GuiTypes as gt

def insterr(code):
  try:
    x = y
  except Exception, inst:
    print(type(inst))    # the exception instance
    print(inst.args)     # arguments stored in .args
    print(inst)      

def newmodule(modname, filename=None):
  null_module = 'Fusion.Utils.null_module'
  import Fusion.Utils.null_module
  sys.modules[modname] = sys.modules[null_module]
  sys.modules[modname].__name__ = modname
  if filename:
    sys.modules[modname].__file__ = filename
  del sys.modules[null_module]
  del Fusion.Utils.null_module
  return sys.modules[modname]

def delmodule(module):
  modname = module.__name__
  if sys.modules.has_key(modname):
    del sys.modules[modname]
  print(module)
  #exec('del %s' % module)

def importmodule(filename, modname):
  module = newmodule(modname, filename)
  try:
    execfile(filename, module.__dict__, module.__dict__)
  except IOError, err:
    del sys.modules[modname]
    raise ImportError, err
  return module 

def importfile(filename):
  dname = os.path.dirname(filename)
  fname = os.path.basename(filename)
  if fname[-3:] == '.py':
    fname = fname[:-3]
  sys.path.insert(0, dname)
  try:
    mod = __import__(fname)
  finally:
    sys.path = sys.path[1:]
  return mod

def dlgconn():
  #reload(gluon)
  reload(vk)
  reload(fusion)
  r = vk.vKhepera(debuglevel=2)
  f = fusion.Fusion(vRobot=r, debuglevel=2)
  return r, f

def gtseplabel(mbPath=None):
  print(gt.MBMakeSepLabel(mbPath))

def dofusion():
  khe = vKhepera.vKhepera()
  return Fusion.Fusion(vRobot=khe)


def cb1(): print('cb1')
def cb2(): print('cb2')
def cb3(): print('cb3')
def cb4(): print('cb4')
def cbtravel(): print('cbtravel')

def mbmakefulltree(mb):
  mbmakeroottree(mb)
  mbmaketraveltree(mb)

def mbmakeroottree(mb):
  mb.AddMenuItem('File|Open...', 'command', owner='root', command=cb1)
  mb.AddMenuItem('File|New...', 'command', owner='root', command=cb1)
  mb.AddMenuItem('File|Save', 'command', owner='root', command=cb1)
  mb.AddMenuItem('File|Save As...', 'command', owner='root', command=cb1)
  seplabel = mb.AddMenuItem('File', 'separator', owner='root')
  mb.AddMenuItem('File|Exit', 'command', owner='root', command=cb1)

  mb.AddMenuItem('View', 'cascade', owner='root', command=cb2)

  mb.AddMenuItem('Go|Run', 'command', owner='root', command=cb2)
  mb.AddMenuItem('Go|Pause', 'command', owner='root', command=cb2)
  mb.AddMenuItem('Go|Step', 'command', owner='root', command=cb2)
  mb.AddMenuItem('Go|Stop', 'command', owner='root', command=cb2)
  mb.AddMenuItem('Go', 'separator', owner='root')
  mb.AddMenuItem('Go|Load', 'command', owner='root', command=cb2)

  mb.AddMenuItem('Help|Fusion', 'command', owner='root', command=cb3)
  mb.AddMenuItem('Help', 'separator', owner='root')
  mb.AddMenuItem('Help|About', 'command', owner='root', command=cb3)

def mbmaketraveltree(mb):
  mb.InsMenuItem('Help', 'Falls|Niagara...', 'command', 
      owner='travel', command=cbtravel)
  mb.AddMenuItem('Falls|Angel...', 'command', 
      owner='travel', command=cbtravel)
  mb.AddMenuItem('Falls', 'separator', owner='travel')
  mb.AddMenuItem('Falls|Africa', 'cascade', owner='travel')
  mb.AddMenuItem('Falls|Africa|Victoria', 'checkbutton', 
      owner='travel', command=cbtravel)
  mb.AddMenuItem('Falls|Africa|Augrabies', 'radiobutton', 
      owner='travel', command=cbtravel)

  mb.InsMenuItem('Help', 'Travel', 'command', owner='travel', command=cbtravel)

  mb.AppMenuItem('Falls', 'Lakes|Erie', 'command', 
      owner='travel', command=cbtravel)
  mb.AddMenuItem('Lakes|Ontario', 'command', owner='travel', command=cbtravel)
  mb.AddMenuItem('Lakes|Superior', 'command', owner='travel', command=cbtravel)
  mb.AddMenuItem('Lakes|Huron', 'command', owner='travel', command=cbtravel)
  mb.AddMenuItem('Lakes|Michigan', 'command', owner='travel', command=cbtravel)

  seplabel = mb.AppMenuItem('File|Save As...', '', 'separator', owner='travel')
  mb.AppMenuItem(seplabel, 'Travel Profile', 'command', 
      owner='travel', command=cbtravel)

def mbprinttree(menuTree=None, indent=0):
  if menuTree is None: menuTree = self.mMenuTree
  #print(menuTree)
  for item in menuTree['items']:
    if item['type'] == 'cascade':
      print("%*s%s|" % (indent, ' ', item['label']))
      PrintMenuTree(item, indent+2)
    else:
      print("%*s%s (%s)" % (indent, ' ', item['label'], item['type']))

def mbcolormenu(mb, color):
  for p in mb:
    try:
      mb[p]['foreground'] = color
    except tk.TclError:
      pass

def tkprintmenudata(menu):
  k = 0
  j = -1
  i = 0
  while i > j and k < 10:
    mtype = menu.type(i)
    print(i, mtype, end='')
    if mtype != 'tearoff' and mtype != 'separator':
      print(menu.entrycget(i, 'label'), end='')
    print()
    j = i
    i = menu.index(i+1)
    k += 1

class B:
  class estate:
    s1 = 1
    s2 = 2
    sname = {s1: 's1', s2: 's2'}

  def __init__(self):
    self.state = self.estate.s1
    self.i = 99
    self.ddata = 98
  def b(self):
    print('b', self.state, self.estate.sname[self.state])

class D(B):
  def __init__(self):
    self.i = 100
    self.ddata = 101
    B.__init__(self)
  def d(self):
    print('d', self.i)

class E:
  class EE:
    ready = 1
    go = 2
    stop = 3
  def __init__(self):
    self.i = self.EE.ready
  def e(self):
    print('.', self.i)

def k(kw):
  for k,v in kw.iteritems():
    if v is None: print(k, 'only')
    else: print(k, v)

class Enum:
  def __init__(self, names):
    self.names = {}
    for num, name in enumerate(names.split()):
      setattr(self, name, num)
      self.names[num] = name
  def name(self, num):
    return self.names[num]

# want: mb['Go|Run']['label'] = 'NoGo'
class helper:
  def __init__(self):
    self.i = 42
  def __getitem__(self, option):
    print('helper.__getitem__', self.i, option)
  def _SetI(self, i):
    self.i = i

class hamburger:
  def __init__(self):
    self.h = helper()
    self.cnt = 0
  def __getitem__(self, name):
    print('hamburger.__getitem__', repr(name))
    self.h._SetI(self.cnt)
    self.cnt += 1
    return self.h
  def __setitem__(self, name, val):
    print('__setitem__', val)


class RevIter:
  def __init__(self, r):
    self.r = r
    self.index  = len(r.data)
  def __del__(self):
    print('__del__')
  def __iter__(self):
    return self
  def next(self):
    if self.index == 0:
      raise StopIteration
    self.index = self.index - 1
    return self.r.data[self.index]

class Reverse:
  "Iterator for looping over a sequence backwards"
  def __init__(self, data):
    self.data = data
    self.datalen = len(data)
  def __iter__(self):
    return RevIter(self)
  def next(self):
    print('Reverse.next')
    raise StopIteration

def revc(r):
  for c in r:
    print(c, end='')

def revd(r):
  for c in r:
    print(c, end='')
    for d in r:
      print(d, end='')

def reve(r):
  i = 0
  for e in r:
    print(c, end='')

class bar:
  IDX0 = 0
  IDX1 = 1
  def __init__(self):
    self.lst = [55, 66]
  def pr(self):
    print(self.lst[self.IDX0])
    print(self.lst[self.IDX1])

def tstgrid():
  root = tk.Tk()
  label1 = tk.Label(root, text='Height')
  label1.grid(sticky=tk.E)
  label2 = tk.Label(root, text='Width')
  label2.grid(sticky=tk.E)

  entry1 = tk.Entry(root)
  entry1.grid(row=0, column=1)
  entry2 = tk.Entry(root)
  entry2.grid(row=1, column=1)
                    
  checkbutton = tk.Checkbutton(root, text='Preserve Aspect')
  checkbutton.grid(columnspan=2, sticky=tk.W)


  image = tk.Label(root, text='prettypix')
  image.grid(row=0, column=2, columnspan=2, rowspan=2,
             sticky=tk.W+tk.E+tk.N+tk.S, padx=5, pady=5)

  button1 = tk.Button(root, text='Zoom In')
  button1.grid(row=2, column=2)
  button2 = tk.Button(root, text='Zoom Out')
  button2.grid(row=2, column=3)

def tstgrid2():
  root = tk.Tk()

  menu = tk.Menu(root)
  root.config(menu=menu)
  menu.insert(1, 'command', label='menuitem', command=cb1)

  """
  frame = tk.Frame(root)
  frame.grid(rowspan=2, padx=3, ipadx=3, sticky=tk.W)
  label1 = tk.Label(frame, text='Height')
  label1.grid(sticky=tk.E)
  label2 = tk.Label(frame, text='Width')
  label2.grid(sticky=tk.E)
  """

  entry1 = tk.Entry(root)
  entry1.grid(row=0, column=1)
  entry2 = tk.Entry(root)
  entry2.grid(row=1, column=1)
                    
  """
  checkbutton = tk.Checkbutton(root, text='Preserve Aspect')
  checkbutton.grid(columnspan=2, sticky=tk.W)


  image = tk.Label(root, text='prettypix')
  image.grid(row=0, column=2, columnspan=2, rowspan=2,
             sticky=tk.W+tk.E+tk.N+tk.S, padx=5, pady=5)

  button1 = tk.Button(root, text='Zoom In')
  button1.grid(row=2, column=2)
  button2 = tk.Button(root, text='Zoom Out')
  button2.grid(row=2, column=3)
  """


def statusbar():
  root = tk.Tk()

  vscrollbar = tk.Scrollbar(root, orient=tk.VERTICAL)
  hscrollbar = tk.Scrollbar(root, orient=tk.HORIZONTAL)

  family='helvetica'
  fontStatus = tkFont.Font(family=family, size=10)#, weight=tkFont.BOLD)
  print(fontStatus.actual())
  print(fontStatus.metrics())
  if not fontStatus:
    print('not found:', end='')
    return
  linespace = fontStatus.metrics()['linespace']
  em = fontStatus.measure('M')

  """
  em = "M"
  try:
    font = ImageFont.truetype("/usr/X11/lib/X11/fonts/TTF/Vera.ttf", 10)
  except IOError:
    font = ImageFont.load_default()
  fw, fh = font.getsize(em)
  """

  canvas = tk.Canvas(root,
    yscrollcommand=vscrollbar.set, 
    xscrollcommand=hscrollbar.set, 
    height=4*linespace, width=80*em)

  vscrollbar.config(command=canvas.yview)
  hscrollbar.config(command=canvas.xview)

  canvas.grid(row=0, column=0, sticky=tk.W+tk.N+tk.S+tk.E)

  vscrollbar.grid(row=0, column=1, sticky=tk.W+tk.N+tk.S)
  hscrollbar.grid(row=1, column=0, sticky=tk.W+tk.N+tk.E)

  orig = 0, 0
  ids = []
  n = 0

  pos = orig
  id = canvas.create_text(pos, fill='black', anchor=tk.NW, font=fontStatus,
      text='BLACK BERRIES')
  ids += [id]

  n += 1
  pos = orig[0], orig[1] + n * linespace 
  id = canvas.create_text(pos, fill='#009900', anchor=tk.NW,
      text='gREEN TOMATOS')
  ids += [id]

  n += 1
  pos = orig[0], orig[1] + n * linespace 
  id = canvas.create_text(pos, fill='blue', anchor=tk.NW,
      text='BLUE BERRIES')
  ids += [id]

  n += 1
  pos = orig[0], orig[1] + n * linespace 
  id = canvas.create_text(pos, fill='red', anchor=tk.NW,
      text='RED BEETS')
  ids += [id]

  n += 1
  pos = orig[0], orig[1] + n * linespace 
  id = canvas.create_text(pos, fill='orange', anchor=tk.NW,
      text='ORANgE pERSSIMONS')
  ids += [id]

def textbar():
  root = tk.Tk()

  vscrollbar = tk.Scrollbar(root, orient=tk.VERTICAL)
  hscrollbar = tk.Scrollbar(root, orient=tk.HORIZONTAL)

  family='helvetica'
  fontStatus = tkFont.Font(family=family, size=10)#, weight=tkFont.BOLD)
  linespace = fontStatus.metrics()['linespace']
  em = fontStatus.measure('M')

  wtext = tk.Text(root,
    yscrollcommand=vscrollbar.set, 
    xscrollcommand=hscrollbar.set, 
    height=4, width=80)

  vscrollbar.config(command=wtext.yview)
  hscrollbar.config(command=wtext.xview)

  wtext.grid(row=0, column=0, sticky=tk.W+tk.N+tk.S+tk.E)

  vscrollbar.grid(row=0, column=1, sticky=tk.W+tk.N+tk.S)
  hscrollbar.grid(row=1, column=0, sticky=tk.W+tk.N+tk.E)

  wtext['state'] = tk.DISABLED

  wtext.tag_config("n", background="yellow", foreground="red")
  wtext.tag_config("a", foreground="blue")

  wtext.tag_config("blue", foreground='blue')
  wtext.tag_config("black", foreground='black')
  wtext.tag_config("red", foreground='red')
  wtext.tag_config("green", foreground='#009900')
  wtext.tag_config("orange", foreground='#996600')

  textadd(wtext, 'BLACK BERRIES')
  textadd(wtext, 'gREEN TOMATOS', tag='green')
  textadd(wtext, 'BLUE BERRIES', tag='blue')
  textadd(wtext, 'RED BEETS', tag='red')
  textadd(wtext, 'ORANgE pERSSIMONS', tag='orange')

  return wtext

wtextmax = 8
wtextlinecnt = 0
wtextstart  = 0
wtextend  = 0
def textadd(wtext, text, tag='black'):
  global wtextmax, wtextlinecnt, wtextstart, wtextend
  wtext['state'] = tk.NORMAL
  if wtextlinecnt > 0:
    wtext.insert(tk.END, '\n', wtextend)
    wtextend += 1
  wtext.insert(tk.END, text, (tag, wtextend))
  wtextlinecnt += 1
  wtext['state'] = tk.DISABLED
  while wtextlinecnt > wtextmax:
    deltag(wtext, wtextstart)
    wtextstart += 1
  wtext.see(tk.END)

def textdump(wtext):
  global wtextlinecnt, wtextstart, wtextend
  for i in range(wtextstart, wtextend+1):
    texttag(wtext, i)

def texttag(wtext, tag):
  ranges = wtext.tag_ranges(tag)
  for i in range(0, len(ranges), 2):
    start = ranges[i]
    stop = ranges[i+1]
    print(i, tag, repr(wtext.get(start, stop)))

def firsttag(wtext, tag):
  range = wtext.tag_nextrange(tag, 1.0)
  if range:
    start = range[0]
    stop = range[1]
    print(tag, repr(wtext.get(start, stop)))

def deltag(wtext, tag):
  global wtextmax, wtextlinecnt, wtextstart, wtextend
  range = wtext.tag_nextrange(tag, 1.0)
  if range:
    start = range[0]
    stop = range[1]
    wtext['state'] = tk.NORMAL
    wtext.delete(start, stop)
    wtext['state'] = tk.DISABLED
    wtextlinecnt -= 1

class s:
  #def __init__(self, args=(), kwargs={}):
  def __init__(self, args=(), **kwargs):
    for arg in args:
      print(repr(arg))
    for k,v in kwargs.iteritems():
      print(repr(k), '=', repr(v))
    self.d = {'this':self.f1, 'that':self.f2}
  def f1(self):
    print('f1')
  def f2(self):
    print('f2')

class testApp2:
  def __init__( self, master ):
    self.ma = master
    self.f = tk.Frame( self.ma )
    self.f.pack()
    self.cv = tk.Canvas(self.f, width=25, height=25, bg='red')
    self.cv.pack()
    self.b1 = tk.Button( self.f, text='hello', command=None )
    self.b1.pack(side='bottom')
    self.ma.aspect(minNumer=400, minDenom=400, maxNumer=500, maxDenom=500)
    self.ma.bind('<Configure>', self.resize )

  def resize( self, event ):
    print('(%d, %d)' % (event.width, event.height))
    #self.cv.configure( width = event.width-4, height = event.height-4 )

def resizer():
  root = Tk()
  app = testApp2(root)
  root.mainloop()

def endpoint(tx):
  e = tx.index(tk.END)
  l = int(float(e)) - 1
  return tx.index('%d.0lineend' % l)

def iterlines(s):
  print('*', repr(s), '*')
  start = 0
  while start < len(s):
    idx = s.find('\n', start)
    if idx >= 0:
      print(repr(s[start:idx+1]))
    else:
      print(repr(s[start:]))
      break
    start = idx + 1

class B:
  val = 33
  def __init__(self):
    self.me = 'yeou'
  def foo(self):
    print(self.me)
  def bar(self):
    print(self.val)

