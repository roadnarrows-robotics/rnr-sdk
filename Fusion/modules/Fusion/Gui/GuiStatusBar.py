################################################################################
#
# GuiStatusBar.py
#

""" Graphical User Interface StatusBar Module

Graphical User Interface (GUI) Tkinter statusbar module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.10.11

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
import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiToolTip as GuiToolTip

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: GuiStatusBar
#-------------------------------------------------------------------------------
class GuiStatusBar(tk.Frame):
  """ GUI StatusBar Class

      A StatusBar encapsulates in a single line a set of status items
      displayed as read-only text. Any or all of the status items may be
      updated as needed to reflect the current status.
  """

  #--
  def __init__(self, master=None, itemList=[], initWidth=None, maxRows=1, 
                      colorLabel=gt.ColorBlue1, colorField='black', **kw):
    """ GUI StatusBar initialization.

        Parameters:
          master      - GUI parent of this statusbar widget
          itemList    - List of status dictionary items [item,...].
                        Each item description:
                          {'tag': <str>,
                           'prefix': <str>,
                           'max_width':<int>,
                           'val:':<val>,
                           'tooltip':<str>}
                        with
                          tag       - Item id and default prefix name.
                                      Required.
                          prefix    - Prefix string to field value.
                                      An empty string '' equals no prefix.
                                      Default: '<tag>: '
                          max_width - Maximum text width of item value.
                                      Default: 2
                          val       - Initial item value. Default: ''
                          fmt       - Value format string. Default: None
                          tooltip   - Item's tooltip message. Default: None
          initWidth   - Status bar initial width (pixels). If None, then
                        status bar will one row of be fixed size.
          maxRows     - Status bar maximum number of display rows.
          colorLabel  - Status bar color of labels
          colorField  - Status bar color of fields
          **kw        - Standard frame keyword=value options.
    """
    # status bar container frame
    kw['relief'] = kw.get('releif', tk.RAISED)      # default
    kw['borderwidth'] = kw.get('borderwidth', 1)    # default
    tk.Frame.__init__(self, master=master, **kw)    # base class

    # look n feel
    self.mColorLabel = colorLabel
    self.mColorField = colorField

    # status bar width fixed overhead (internal x padding plus border width)
    self._oh = kw['borderwidth'] * 2

    # overline accent
    self._overline = tk.Canvas(self, height=2, width=2, borderwidth=0,
                                bg=self.mColorLabel)
    self._overline.grid(row=0, column=0, columnspan=2)

    # status bar info icon
    w = tk.Label(self, relief=tk.FLAT, bitmap='info')
    w.grid(row=1, column=0, rowspan=maxRows, sticky=tk.W, padx=2)

    # add info icon width to fixed overhead
    self.update_idletasks()  # force widget draw to get geo
    self._oh += gut.geometry(w)[gut.W]

    # overhead fudge factor - internal paddings, borders, etc
    self._oh += 8

    # initial width is the maximum width (can also be set in configure())
    if initWidth:
      self._maxwidth    = initWidth
    else:
      self._maxwidth    = 0   # status bar max width will be width of first row

    # other internal working variables
    self._maxrows = maxRows    # maximum status bar rows
    self._endrow  = 0          # current status bar end row

    #
    # Status bar container frames, one frame per status bar row. Having a
    # frame per row eliminates grid alignment hassles between rows.
    #
    self._frame = []
    for row in range(0, self._maxrows):
      self._frame += [{}]
      self._frame[row]['wframe']  = None      # frame container widget
      self._frame[row]['width']   = self._oh  # current width (pixels)
      self._frame[row]['tags']    = []        # ordered items on this frame
      self._frame[row]['wpad']    = None      # right pad widget
      self._frame[row]['wpad_viz']= False     # pad is [not] visible 

    # calculate item width coefficients for width estimation
    self._CalcCoef()

    #
    # Draw Status bar.
    #
    self._item = {}
    for item in itemList:
      try:
        tag = item['tag']
      except KeyError:
        raise KeyError('GuiStatusBar: new item requires a tag')
      del item['tag']   # remove 'tag' key from copied item initial values
      self.AppendItem(tag, **item)

    # maximum width inherits width of first frame 
    if self._maxwidth == 0:
      self._maxwidth = self._frame[0]['width']

    # resize overline accent
    self._Accent()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Status Bar Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    
  #--
  def AppendItem(self, tag, **item):
    """ Append new status item at the end of the StatusBar.

        Parameters:
          tag     - New item tag and default prefix name.
          **items - Status bar item options:
              prefix    - Prefix string to field value.
                          An empty string '' equals no prefix.
                          Default: '<tag>: '
              max_width - Maximum text width of item value.
                          Default: 2
              val       - Initial item value. Default: ''
              fmt       - Value format string. Default: None
              tooltip   - Item's tooltip message. Default: None

        Return Value:
          None
    """
    item_width  = self._EstWidth(tag, **item)
    row_width   = self._frame[self._endrow]['width']
    if self._maxwidth > 0 and row_width+item_width > self._maxwidth \
        and self._endrow < self._maxrows-1: 
      self._endrow += 1
    self._CreateItem(tag, self._endrow, len(self._frame[self._endrow]['tags']),
                        **item)

  #--
  def InsertItem(self, tag, row, index, **item):
    """ Insert the new status item on the given StatusBar row before the
        given row item index.

        Parameters:
          tag     - New item tag and default prefix name.
          row     - Status bar row [0,maxrows-1]
          index   - Item index on row.
          **items - Status bar item options (see AppendItem())

        Return Value:
          None
    """
    # create item
    self._CreateItem(tag, row, index, **item)

    # move other items to the right of insertion point
    col = index * 2 + 2
    for otag in self._frame[row]['tags'][index+1:]:
      self._item[otag]['wlabel'].grid(row=0, column=col, sticky=tk.W, padx=0)
      self._item[otag]['wfield'].grid(row=0, column=col+1, sticky=tk.W, ipadx=2)
      col += 2

    # bump status bar end row
    if row > self._endrow:
      self._endrow = row

    return self._item[tag]['item_width']

  #--
  def MoveItem(self, tag, row, index):
    """ Move an existing item from its current StatusBar location to the new
        position at the given StatusBar row before the given row item index.

        Parameters:
          tag     - Existing item tag and default prefix name.
          row     - Status bar row [0,maxrows-1]
          index   - Item index on row.

        Return Value:
          None
    """
    # save key item data
    item = {}
    for k in ['prefix', 'max_width', 'val', 'fmt', 'tooltip']:
      item[k] = self._item[tag][k]
    # destroy item from old location
    self._DestroyItem(tag)
    # insert item back into new location
    self.InsertItem(tag, row, index, **item)

  #--
  def DelItem(self, tag):
    """ Delete status item.

        Parameters:
          tag   - Existing item tag.

        Return Value:
          None
    """
    try:
      item = self._item[tag]
    except KeyError:
      raise KeyError('GuiStatusBar: no item found: %s' % (repr(tag)))
    row = item['row']
    self._DestroyItem(tag)
    self._ReflowItems('out', startrow=row)

  #--
  def Update(self, **items):
    """ Update status item values.

        Parameters:
          **items   - tag=<val> [,tag=<val> ...]

        Return Value:
          None
    """
    for tag, val in items.iteritems():
      try:
        item = self._item[tag]
      except KeyError:
        raise KeyError('GuiStatusBar: no item found: %s' % (repr(tag)))
      item['val'] = val
      item['wfield']['text'] = item['formatter'](val)


  #--
  def GetItemTags(self):
    """ Return a list of all of the status bar item tags. """
    return self._item.keys()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Tkinter.Frame Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def configure(self, **kw):
    """ (Re)Configure the status bar layout.

        Parameters:
          **kw      - Keyword configuration

        Return Value:
          None
    """
    # get configuration
    if not kw:
      return {
          'width': self._maxwidth,
          'colorLabel': self.mColorLabel,
          'colorField': self.mColorField,
          'maxRows': self._maxrows,
          'items': self.GetItemTags()}

    # set configuration
    # RDK: setting Frame's configure() clears all widgets. Why?
    #tk.Frame.configure(self, **kw)
    newWidth = kw.get('width', 0)
    if newWidth == 0:
      return
    oldWidth = self._maxwidth
    self._maxwidth = newWidth - 8   # 8 pixels of overhead (by experiment)
    if oldWidth == newWidth:
      return
    elif oldWidth == 0 or oldWidth > newWidth:
      self._ReflowItems('in')
    else:
      self._ReflowItems('out')


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _CalcCoef(self, _staticvar={'#done_once#':0}):
    """ Calculate item coefficients used to estimate item's width (pixels).

        Parameters:
          _staticvar  - Class 'static' variables. A mutable object like a
                        dictionary element as default argument will act like
                        a static variable since its address is fixed.

        Return Value:
          None
    """
    # 
    # Each item component (label and field) width is a linear function:
    #   w = k * wchar + c
    # where
    #   k is a fixed scaler, c is the fixed overhead and wchar is the widget
    #   width in the number of text characters.
    # Sample two points to determine k and c
    #
    if not _staticvar['#done_once#']:
      wlabel = tk.Label(self, height=1, width=2,
                      anchor=tk.E, relief=tk.RIDGE,
                      state=tk.NORMAL, fg=gt.ColorBlue1)
      wfield = tk.Label(self, height=1, width=2,
                      anchor=tk.E, relief=tk.SUNKEN, state=tk.NORMAL)
      wlabel.grid(row=1, column=1, sticky=tk.W, padx=0)
      wfield.grid(row=1, column=2, sticky=tk.W, padx=0)
      self.update_idletasks()
      w11 = gut.geometry(wlabel)[gut.W]
      w21 = gut.geometry(wfield)[gut.W]

      wlabel['width'] = 11  # 2nd wchar value
      wfield['width'] = 11  # 2nd wchar value
      self.update_idletasks()
      w12 = gut.geometry(wlabel)[gut.W]
      w22 = gut.geometry(wfield)[gut.W]
      wlabel.destroy()
      wfield.destroy()

      _staticvar['_k1'] = (w12 - w11) / 9.0
      _staticvar['_c1'] = w11 - 2 * _staticvar['_k1']

      _staticvar['_k2'] = (w22 - w21) / 9.0
      _staticvar['_c2'] = w21 - 2 * _staticvar['_k2']

      _staticvar['#done_once#'] = 1

    # set class instance attributes
    for k,v in _staticvar.iteritems():
      if k != '#done_once#':
        setattr(self, k, v)

  #--
  def _EstWidth(self, tag, **item):
    """ Estimate width of item.

        Parameters:
          tag     - Item tag and default prefix name.
          **items - Status bar item options (see AppendItem())

        Return Value:
          Item's width in pixels.
    """
    w_label = len(item.get('prefix', tag+':'))
    w_field = item.get('max_width', 2)
    # label width + field width + border
    return int(self._k1 * w_label + self._c1 + \
               self._k2 * w_field + self._c2 + \
               4)

  #--
  def _ReflowItems(self, dir, startrow=0):
    """ Reflow StatusBar items.

        Parameters:
          dir       - One of: 'in' 'out' where:
                      'in'  - Status bar's width has shrunked.
                      'out' - Status bar's width has expaneded or less
                              items exist.
          startrow  - Status bar's starting row for the reflow.

        Return Value:
          None
    """
    if dir == 'in':
      for row in range(startrow, self._maxrows-1):
        if self._frame[row]['tags']:
          pos = self._oh + self._item[self._frame[row]['tags'][0]]['item_width']
        else:
          return
        tags = self._frame[row]['tags'][1:]   # moveable item tags
        n = 0
        for tag in tags:
          pos += self._item[tag]['item_width']
          if pos > self._maxwidth:
            break;
          n += 1
        index = 0
        while n < len(tags):
          self.MoveItem(tags[n], row+1, index)
          index += 1
          n += 1
    elif dir == 'out':
      for row in range(startrow, self._endrow):
        index = len(self._frame[row]['tags'])
        tags = self._frame[row+1]['tags']   # make copy
        for tag in tags:
          width = self._frame[row]['width']
          item_width = self._item[tag]['item_width']
          if width + item_width <= self._maxwidth:
            self.MoveItem(tag, row, index)
            index += 1
          else:
            break;
    for row in range(startrow, self._endrow+1):
      self._Pad(row)
    self._Accent()

  #--
  def _CreateItem(self, tag, row, index, **item):
    """ Create a new StatusBar item.

        Parameters:
          tag     - New item tag and default prefix name.
          row     - Status bar row [0,maxrows-1]
          index   - Item index on row.
          **items - Status bar item options (see AppendItem())

        Return Value:
          None
    """
    # new item db
    self._item[tag] = {}

    # intial item data or defaults
    prefix    = '  '+item.get('prefix', tag+':')
    max_width = item.get('max_width', 2)
    val       = item.get('val', '')
    fmt       = item.get('fmt', None)
    tooltip   = item.get('tooltip', None)

    # container frame data
    frame = self._frame[row]

    # auto-build container frame if not present
    if not frame['wframe']:
      frame['wframe'] = tk.Frame(self, relief=tk.FLAT,
                                borderwidth=0)
      frame['wframe'].grid(row=row+1, column=1)

    # field label
    # rdk
    #wlabel = tk.Label(frame['wframe'], height=1, width=len(prefix)+2,
    #                text=prefix, anchor=tk.E, relief=tk.RIDGE,
    #                state=tk.NORMAL, fg=self.mColorLabel)
    # rdk
    wlabel = tk.Label(frame['wframe'], height=1,
                    text=prefix, anchor=tk.E, relief=tk.RIDGE,
                    state=tk.NORMAL, fg=self.mColorLabel)

    # formatted field value
    # rdk: add left/right justification to formatting
    if fmt:
      fmtstr = '%*s' % (max_width, '') + fmt
      formatter = lambda v: fmtstr % (v)
    elif type(val) != str:
      formatter = lambda v: '%*s' % (max_width, repr(v))
    else:
      formatter = lambda v: '%*s' % (max_width, v)

    # field
    wfield = tk.Label(frame['wframe'], height=1, width=max_width,
                    text=formatter(val),
                    anchor=tk.E,
                    relief=tk.SUNKEN, state=tk.NORMAL,
                    fg=self.mColorField)

    # field tool tip
    if tooltip:
      wtt = GuiToolTip.GuiToolTip(wfield, text=tooltip)
    else:
      wtt = None

    # forget padding
    self._PadForget(row)

    # show
    col = index * 2
    wlabel.grid(row=0, column=col, sticky=tk.W, padx=0)
    wfield.grid(row=0, column=col+1, sticky=tk.W, ipadx=2)

    # status bar item actual width (pixels)
    self.update_idletasks()  # force widget draw to get geo
    item_width =  gut.geometry(wlabel)[gut.W] + gut.geometry(wfield)[gut.W]

    # item data
    self._item[tag]['row']        = row         # 'points' to _frame[]
    self._item[tag]['prefix']     = prefix      # item label prefix
    self._item[tag]['wlabel']     = wlabel      # item label widget
    self._item[tag]['fmt']        = fmt         # field format
    self._item[tag]['formatter']  = formatter   # field formatter
    self._item[tag]['max_width']  = max_width   # field max width (char)
    self._item[tag]['val']        = val         # field initial value
    self._item[tag]['tooltip']    = tooltip     # tool tip text
    self._item[tag]['wfield']     = wfield      # item field widget
    self._item[tag]['wtt']        = wtt         # item tool tip widget
    self._item[tag]['item_width'] = item_width  # item width (pixels)

    # update frame data
    frame['width'] += self._item[tag]['item_width'] # status bar width 
    frame['tags'].insert(index, tag)                # ordered list of items

    # re-pad out row
    self._Pad(row)

  #--
  def _DestroyItem(self, tag):
    """ Update status item values.

        Parameters:
          tag   - Existing item tag.

        Return Value:
          None
    """
    self._item[tag]['wlabel'].destroy()
    self._item[tag]['wfield'].destroy()
    if self._item[tag]['wtt']:
      del self._item[tag]['wtt']

    row = self._item[tag]['row']

    # update frame data
    self._frame[row]['width'] -= self._item[tag]['item_width']
    self._frame[row]['tags'].remove(tag)

    # no more items on expandable row - destroy
    if row > 0 and len(self._frame[row]['tags']) == 0:
      if self._frame[row]['wpad']:
        self._frame[row]['wpad'].destroy()
      self._frame[row]['wpad'] = None
      self._frame[row]['wpad_viz'] = False
      self._frame[row]['wframe'].destroy()
      self._frame[row]['wframe'] = None
      self._frame[row]['width'] = self._oh
      self._endrow -= 1 # assumes end row is row deleted, else bug
    else:
      self._Pad(row)

    # delete item
    del self._item[tag]

  #-- 
  def _Pad(self, row):
    """ Pad out row to force left justification.

        Parameters:
          row   - Status bar row.

        Return Value:
          None
    """
    if not self._frame[row]['wframe']:
      return

    pad = self._maxwidth - self._frame[row]['width']

    # adjust pad widget to fill out status bar
    if pad >= 2:
      # auto-create pad widget
      if not self._frame[row]['wpad']:
        self._frame[row]['wpad'] = tk.Label(self._frame[row]['wframe'],
                    width=pad,
                    anchor=tk.E, bitmap='gray12', relief=tk.SUNKEN,
                    bg='#cccccc')
        # keep at end
        self._frame[row]['wpad'].grid(row=0, column=100, sticky=tk.W)
        self._frame[row]['wpad_viz'] = True
      # pad out
      else:
        self._frame[row]['wpad']['width'] = pad
        # make visible
        if not self._frame[row]['wpad_viz']:
          self._frame[row]['wpad'].grid(row=0, column=100, sticky=tk.W)
          self._frame[row]['wpad_viz'] = True

    # no need to pad
    elif self._frame[row]['wpad']:
      self._frame[row]['wpad'].destroy()
      self._frame[row]['wpad'] = None
      self._frame[row]['wpad_viz'] = False

  #--
  def _PadForget(self, row):
    """ Forget (i.e. make invisible) the padding widget on the StatusBar
        row.

        Parameters:
          row   - Status bar row.

        Return Value:
          None
    """
    if not self._frame[row]['wframe']:
      return
    elif not self._frame[row]['wpad']:
      return
    else:
      self._frame[row]['wpad'].grid_forget()
      self._frame[row]['wpad_viz'] = False

  #--
  def _Accent(self):
    """ Adjust any StatusBar accents.

        Return Value:
          None
    """
    self._overline.configure(width=self._maxwidth)


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import random

  class mywin(tk.Tk):
    def __init__(self):
      tk.Tk.__init__(self)
      self.body()
      self.CalcDim()
      self.bind('<Configure>', self.configure)

    def CalcDim(self):
      self.update_idletasks()  # force draw to get geo

      # current window dimensions
      wingeo = gut.geometry(self)

      # status bar width
      fgeo = gut.geometry(self.f)

      # window border width and height
      #self.mWinBorder = wingeo[gut.W] - sbgeo[gut.W]


    def configure(self, event):
      geo = gut.geometry(self)
      self.sb.configure(width=geo[gut.W])

    def body(self):
      f = tk.Frame(self)
      f.grid(row=0, column=0, sticky=tk.W)
      row = col = 0
      w = tk.Button(f, text="Conn", command=self.cbconn)
      w.grid(row=row, column=col, sticky=tk.W)
      col += 1
      w = tk.Button(f, text="RandHex", command=self.cbhex)
      w.grid(row=row, column=col, sticky=tk.W)
      col += 1
      w = tk.Button(f, text="RandStr", command=self.cbstr)
      w.grid(row=row, column=col, sticky=tk.W)
      col += 1
      w = tk.Button(f, text="All", command=self.cball)
      w.grid(row=row, column=col, sticky=tk.W)
      col += 1
      w = tk.Button(f, text="Add", command=self.cbadd)
      w.grid(row=row, column=col, sticky=tk.W)
      col += 1
      w = tk.Button(f, text="Del", command=self.cbdel)
      w.grid(row=row, column=col, sticky=tk.W)
      self.f = f

      self.update_idletasks()  # force draw to get geo
      fgeo = gut.geometry(self.f)

      self.sb = GuiStatusBar(self,
        itemList = [
          {'tag': 'Connection',
           'max_width': 12,
            'val': 'disconnected',
           'tooltip': 'Connection status'
          },
          {'tag':'hex',
            'max_width': 4,
            'val': 255,
            'fmt': '0x%02x',
            'tooltip': "hex format"
          },
          {'tag':'str',
            'max_width': 10,
            'fmt': 'my %s',
            'tooltip': "str format"
          },
          {
            'tag': 'ott',
            'prefix': '',
            'max_width': 10,
            'val': 'fixed'
          },
        ],
        initWidth=fgeo[gut.W],
        maxRows=2)
      self.sb.grid(row=1, column=0, sticky=tk.W+tk.E)

    #--
    def cbconn(self):
      tbl = ['disconnected', '115200-8-N-1', '9600-8-E-2']
      s = random.choice(tbl)
      print('UT: Update(Connection=%s)' % s)
      self.sb.Update(Connection=s)
  
    #--
    def cbhex(self):
      i = random.randint(0,255)
      print('UT: Update(hex=0x%x)' % i)
      self.sb.Update(hex=i)
  
    #--
    def cbstr(self):
      tbl = ['apples', 'bananas', 'cherries', 'dates', 'figs']
      s = random.choice(tbl)
      print('UT: Update(str=%s)' % s)
      self.sb.Update(str=s)
  
    #--
    def cball(self):
      print("UT: Update(Connection='disconnected', hex=0, str='sweets')")
      self.sb.Update(Connection='disconnected', hex=0, str='sweets')
  
    #--
    def cbadd(self):
      a = ['b', 'g', 'k', 'm', 's', 't', 'z']
      b = ['a', 'e', 'o']
      c = ['d', 'f', 'p', 'r', 'w']
      tag = '#'+random.choice(a)+random.choice(b)+random.choice(c)
      val = random.random()
      fmt = '%6.4f'
      print('UT: AppendItem(%s,val=%f,fmt=%s)' % (tag, val, fmt))
      self.sb.AppendItem(tag, val=val, fmt=fmt, max_width=6)
  
    #--
    def cbdel(self):
      alltags = self.sb.GetItemTags()
      deltags = []
      for tag in alltags:
        if tag[0] == '#':
          deltags += [tag]
      if deltags:
        tag = random.choice(deltags)
        print('UT: DelItem(%s)' % (tag))
        self.sb.DelItem(tag)

  #--
  def main():
    """ GuiStatusBar Unit Test Main """
    win = mywin()
    print(win.sb.configure())
    win.mainloop()

  # run unit test
  main()
