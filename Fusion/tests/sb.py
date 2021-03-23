# status bar primatives
import Tkinter as tk
import Fusion.Gui.GuiUtils as gut

SB = None

class sb(tk.Frame):
  def __init__(self, master=None, **kwargs):
    tk.Frame.__init__(self, master=master, **kwargs)
    colors = ['#8888ff', '#88ff88', '#ff8888']
    self.l = []
    self.can = tk.Canvas(self, height=2, width=200, borderwidth=0, bg='#440088')
    self.can.grid(row=0, column=0, columnspan=5)
    for n in range(0,3):
      self.l.append(tk.Label(self, text='label%d'%n, bg=colors[n]))
      self.l[n].grid(row=1, column=n)
    self.update_idletasks()
    self.fixedwidth = 0
    for w in self.l:
      self.fixedwidth += gut.geometry(w)[gut.W]
    self.pad = None
    self.can.bind('<Configure>', self.resize)

  def configure(self, cnf=None, **kw):
    print('configure', repr(kw))
    tk.Frame.configure(self, cnf=cnf, **kw)
    width = kw.get('width', 0)
    if width == 0:
      return
    if width > self.fixedwidth:
      if not self.pad:
        self.pad = tk.Label(self, width=width-self.fixedwidth,
                    anchor=tk.E, bitmap='gray12', relief=tk.SUNKEN,
                    bg='#cccccc')
        self.pad.grid(row=1, column=100)
      else:
        self.pad['width'] = width-self.fixedwidth

  def resize(self, event):
    print('resize self=%dx%d' % (event.width, event.height))
    print('resize', 'master=', self.master.winfo_geometry())
    width = gut.geometry(self.master)[gut.W]
    self.configure(width=width)
  def x(self):
    self.pad['width'] = 44


def main():
  global SB
  root = tk.Tk()
  SB = sb(root)
  SB.grid(row=0, column=0)

def movedown():
  global l1, l2, l3
  l3.grid(row=1, column=0)
