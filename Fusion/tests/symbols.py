import Tkinter as tk
from tkFont import Font as Font
import Fusion.Gui.GuiTypes as gt

root = tk.Tk()

#root.font = Font(root, font = 
#    (u'\u30d2\u30e9\u30ae\u30ce\u89d2\u30b4 Pro W3', 12))
root.font = Font(root, font=gt.FontHelv12Bold)
print(root.font.actual())

myFrame = tk.Frame(root)
root.resizable(1, 1)
root.rowconfigure(0,weight=1)
root.columnconfigure(0,weight=1)
myFrame.grid()
myFrame.rowconfigure(0,weight=1)
myFrame.columnconfigure(0,weight=1)

EntryGreek = tk.Entry(myFrame, borderwidth = 1, font=root.font, width=100)
#testString = u"\u8146, \u500e, \u440c"

# greek
greek     = ''
v = gt.UniGreek.values()
v.sort()
for u in v:
  greek += u
EntryGreek.insert(0, greek)
EntryGreek.grid()
#print(repr(EntryGreek.get()))

EntrySS = tk.Entry(myFrame, borderwidth = 1, font=root.font, width=100)
script = ''
for k,v in gt.UniSubscript.iteritems():
  script += k
  script += v
script += '     '
for k,v in gt.UniSuperscript.iteritems():
  script += k
  script += v
EntrySS.insert(0, script)
EntrySS.grid(row=1)

root.mainloop()
