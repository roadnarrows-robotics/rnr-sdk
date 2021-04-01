#! /usr/bin/env python3

###############################################################################
#
# Package:  RoadNarrows Robotics Laelaps Robotic Mobile Platform Package
#
# Script:   laelaps_motors
#
# File: laelaps_motors
#
## \file 
##
## $LastChangedDate$
## $Rev$
##
## \brief Graphical interface to Laelaps motors for tuning and debugging.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright
##   \h_copy 2015-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##
# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# 
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import math
import getopt

if sys.version_info[0] < 3:
  from tkinter import *
  from tkinter.constants import *
  from tkinter.filedialog import *
  import tkinter.font
else:
  from tkinter import *
  from tkinter.constants import *
  from tkinter.filedialog import *
  import tkinter.font

import webbrowser

import Laelaps.RoboClaw as Roboclaw
import Laelaps.SysConf as SysConf
import Laelaps.Utils as Utils
import Laelaps.VelPlot as VelPlot
import Laelaps.PidParams as PidParams


# ------------------------------------------------------------------------------
# Globals
# ------------------------------------------------------------------------------

## \brief Application version. Update as needed. 
AppVersion = '1.0.0'

## \brief Laelaps wiki URL
LaelapsWikiUrl = "https://github.com/roadnarrows-robotics/laelaps/wiki"

## \brief Image search paths.
ImagePath = [
  "/prj/share/Laelaps/images",
  "/usr/local/share/Laelaps/images",
  "/prj/share/appkit/images",
  "/usr/local/share/appkit/images"
]

## \brief Common UI colors.
UIColors = {
  'normal':       'black',
  'ok':           '#008800',
  'focus':        '#0000aa',
  'warning':      '#aa6600',
  'error':        '#cc0000',
  'left_front':   '#aa0000',
  'right_front':  '#aa6600',
  'left_rear':    '#0066aa',
  'right_rear':   '#0000aa',
}

# \brief Tri-state values.
TriState = {
  'none': -1,
  'off':  0,
  'on':   1
}

_ctlrkey = lambda pos: str.lower(pos) + "_ctlr"
_motkey  = lambda pos: str.lower(pos) + "_motor"

#
## \brief Concatenate two dictionaries to make a third.
#
def dictConcat(d1, d2):
  e = {}
  e.update(d1)
  e.update(d2)
  return e


# ------------------------------------------------------------------------------
# Class window
# ------------------------------------------------------------------------------

##
## \brief Window class supporting application.
##
class window(Frame):

  #
  ## \brief Constructor.
  ##
  ## \param master  Window parent master widget.
  ## \param cnf     Configuration dictionary.
  ## \param kw      Keyword options.
  #
  def __init__(self, master=None, cnf={}, **kw):
    self.m_wMaster = master

    self.perfStart()

    # intialize window data
    kw = self.initData(kw)

    # initialize parent object frame
    Frame.__init__(self, master=master, cnf=cnf, **kw)

    # window title
    self.master.title("Laelaps Motors")

    self.m_imageLoader = Utils.ImageLoader(py_pkg='Laelaps.images',
                                       image_paths=ImagePath)

    self.m_icons['app_icon'] = self.m_imageLoader.load("icons/LaelapsIcon.png")
    #RDK self.master.tk.call('wm', 'iconphoto', self.master._w,
    #RDK                    self.m_icons['app_icon'])

    #print('DBG', self.m_var)

    # create and show widgets
    self.createWidgets()
    self.grid(row=0, column=0, padx=5, pady=5)

    self.update_idletasks()
    #width = self.winfo_width()
    #print('DBG: self.width =', width)

    # RDK Comment out when done. Debug
    self.dbgSeedFields()

  def perfStart(self):
    self.t0 = time.time()

  def perfMark(self, msg):
    t1 = time.time()
    print("%.4f: %s" % (t1 - self.t0, msg))
    self.t0 = t1

  #
  ## \brief Initialize class state data.
  ##
  ## Any keywords for this application specific window that are not supported 
  ## by the Frame Tkinter class must be removed.
  ##
  ## \param kw      Keyword options.
  ##
  ## \return Modified keywords sans this specific class.
  #
  def initData(self, kw):
    self.m_botName        = "laelaps"
    #RDK self.m_cfgPanel       = PanelConfigDlg.ConfigDft.copy() # panel config
    self.m_icons          = {}    # must keep loaded icons referenced
    self.m_wBttn          = {}    # button widgets
    self.m_plotVel        = None  # no plot yet

    # fixed names, addresses, positions
    self.m_motorCtlrPos   = ['Front', 'Rear']
    self.m_motorCtlrAddr  = {'Front': SysConf.MotorCtlrAddrFront,
                             'Rear':  SysConf.MotorCtlrAddrRear}
    self.m_motorPos       = ['Left', 'Right']
    self.m_powertrain     = {
      'front_ctlr': {'left_motor': 'left_front', 'right_motor': 'right_front'},
      'rear_ctlr':  {'left_motor': 'left_rear', 'right_motor': 'right_rear'}
    }

    # variable db
    self.m_var = {
      'front_ctlr': {'changes': False, 'left_motor': {}, 'right_motor': {}},
      'rear_ctlr':  {'changes': False, 'left_motor': {}, 'right_motor': {}}
    }

    # override panel configuration
    #RDK if 'config' in kw:
    #RDK   self.m_cfgPanel = kw['config'].copy()
    #RDK   del kw['config']
    #RDK # or read configuration
    #RDK else:
    #RDK   self.readPanelConfig()

    return kw

  def dbgSeedFields(self):
    for ctlrpos in self.m_motorCtlrPos:
      ctlrkey = _ctlrkey(ctlrpos)
      addr    = self.m_motorCtlrAddr[ctlrpos]

      self.m_var[ctlrkey]['addr'].set("0x%02x" % (addr))
      self.m_var[ctlrkey]['baudrate'].set(SysConf.MotorCtlrBaudRate)
      self.m_var[ctlrkey]['model'].set("2x15a")
      self.m_var[ctlrkey]['version'].set("4.1.6")
      self.m_var[ctlrkey]['batt_max'].set(33.6)
      self.m_var[ctlrkey]['batt_min'].set(8.1)
      self.m_var[ctlrkey]['logic_max'].set(33.6)
      self.m_var[ctlrkey]['logic_min'].set(5.0)
      self.m_var[ctlrkey]['battery'].set(11.9)
      self.m_var[ctlrkey]['logic'].set(12.0)
      self.m_var[ctlrkey]['temp'].set(26.4)
      self.m_var[ctlrkey]['alarm_temp']['w']['image'] = \
          self.m_icons['led_green']
      self.m_var[ctlrkey]['alarm_temp']['val'] = TriState['off']
      self.m_var[ctlrkey]['alarm_batt_high']['w']['image'] = \
          self.m_icons['led_red']
      self.m_var[ctlrkey]['alarm_batt_high']['val'] = TriState['on']

      for motorpos in self.m_motorPos:
        motorkey = _motkey(motorpos)

        self.m_var[ctlrkey][motorkey]['enc_type'].set("Quadrature")
        self.setPidParams(ctlrkey, motorkey, 1.0, 0.25, 0.5, 100000, 1000.0)

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    # preload some of the needed images
    self.m_icons['rn_logo']   = self.m_imageLoader.load("RNLogo48.png");
    self.m_icons['laelaps_logo'] = \
        self.m_imageLoader.load("icon_laelaps_logo.png");
    self.m_icons['led_dark']  = self.m_imageLoader.load("icon_led_dark_16.png")
    self.m_icons['led_green'] = self.m_imageLoader.load("icon_led_green_16.png")
    self.m_icons['led_red']   = self.m_imageLoader.load("icon_led_red_16.png")
    self.m_icons['linked_h'] = \
        self.m_imageLoader.load('icon_chain_h_linked_16.png')
    self.m_icons['unlinked_h'] = \
        self.m_imageLoader.load('icon_chain_h_unlinked_16.png')
    self.m_icons['linked_v'] = \
        self.m_imageLoader.load('icon_chain_v_linked_16.png')
    self.m_icons['unlinked_v'] = \
        self.m_imageLoader.load('icon_chain_v_unlinked_16.png')
    self.perfMark("Preloaded images")

    self.createMenu()
    self.update_idletasks()
    self.perfMark("Created menus")

    self.createHeading()
    self.update_idletasks()
    self.perfMark("Created heading")

    self.createLeftButtons()
    self.update_idletasks()
    self.perfMark("Created left buttons")

    self.createCenterPanel()
    self.update_idletasks()
    self.perfMark("Created center panel")

    self.createStatusBar()
    self.update_idletasks()
    self.perfMark("Created status bar")

    #RDK self.updateButtonState(self.m_keysFewMoves, 'disabled')

  #
  ## \brief Create menu
  #
  def createMenu(self):
    # top menu bar
    self.m_wMenuBar = Menu(self)

    # file menu
    self.m_wMenuFile = Menu(self.m_wMenuBar, tearoff=0)
    self.m_wMenuFile.add_command(label="Save...", command=self.notimpl)
    self.m_wMenuFile.add_separator()
    self.m_wMenuFile.add_command(label="Quit", command=self.destroy)
    self.m_wMenuBar.add_cascade(label="File", menu=self.m_wMenuFile)

    # move menu
    self.m_wMenuMove = Menu(self.m_wMenuBar, tearoff=0)
    self.m_wMenuBar.add_cascade(label="Move", menu=self.m_wMenuMove)

    # help menu
    self.m_wMenuHelp = Menu(self.m_wMenuBar, tearoff=0)
    self.m_wMenuHelp.add_command(label="Online Laelaps Wiki",
        command=lambda aurl=LaelapsWikiUrl:webbrowser.open_new(LaelapsWikiUrl))
    self.m_wMenuHelp.add_separator()
    self.m_wMenuHelp.add_command(label="About...", command=self.about)
    self.m_wMenuBar.add_cascade(label="Help", menu=self.m_wMenuHelp)

    self.m_wMaster.config(menu=self.m_wMenuBar)

  #
  ## \brief Create top gui heading.
  #
  def createHeading(self):
    # rn logo
    w = Label(self)
    if self.m_icons['rn_logo']:
      w['image'] = self.m_icons['rn_logo']
      w['anchor'] = W
    else:
      w['text'] = 'rn'
      w['anchor'] = W
      w['width'] = 5
    w.grid(row=0, column=0, sticky=W)
    
    # top heading
    w = Label(self)
    w['font']   = ('Helvetica', 16)
    w['text']   = "Laelaps Motors"
    w['anchor'] = CENTER
    w['justify'] = CENTER
    w.grid(row=0, column=1, sticky=E+W)
    self.m_wTopHeading = w

    # hek logo
    w = Label(self)
    if self.m_icons['laelaps_logo']:
      w['image'] = self.m_icons['laelaps_logo']
      w['anchor'] = E
    else:
      w['text'] = 'mot'
      w['anchor'] = E
      w['width'] = 5
    w.grid(row=0, column=2, sticky=E)
    
  #
  ## \brief Create gui left hand side buttons.
  #
  def createLeftButtons(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 0
    wframe['relief'] = 'ridge'
    wframe.grid(row=1, column=0, padx=1, pady=20, sticky=N+W+E)

    row = 0

    # apply
    row += 1
    w = self.createButton(wframe, "Apply", "icons/icon_check.png",
                            self.apply)
    w.grid(row=row, column=0, sticky=N+E+W)

    # stop
    row += 1
    w = self.createButton(wframe, "Stop",
                                  "icons/icon_stop2.png", self.stop)
    w.grid(row=row, column=0, sticky=N+E+W)

    # save
    row += 1
    w = self.createButton(wframe, "Save", "icons/icon_floppy.png",
                            self.save)
    w.grid(row=row, column=0, sticky=N+E+W)

    # info
    row += 1
    w = self.createButton(wframe, "About",
                                  "icons/icon_info.png", self.about)
    w.grid(row=row, column=0, sticky=N+E+W)

    # quit
    row += 1
    w = self.createButton(wframe, "Quit", "icons/icon_exit.png", self.destroy,
                                  fg='red')
    w.grid(row=row, column=0, sticky=N+E+W)

  #
  ## \brief Create robot status and joint state center panel.
  #
  def createCenterPanel(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 0
    wframe['relief'] = 'ridge'
    wframe.grid(row=1, column=1, columnspan=2,
        padx=1, pady=3, sticky=N+W+E)

    self.createStatusPanel(wframe, 0, 0)
    self.perfMark("Created status panel")

    self.createVelTuningPanel(wframe, 0, 1)
    self.perfMark("Created velocity tuning panel")

    self.update_idletasks()
    width = wframe.winfo_width()

    self.createPlotPanel(wframe, 1, 0, width)
    self.perfMark("Created plot panel")

  #
  ## \brief Create motor status panel.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  #
  def createStatusPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    row = 0
    col = 0
        
    for name in self.m_motorCtlrPos:
      self.createMotorCtlrPanel(wframe, row, col, name)
      row += 1      

  #
  ## \brief Create motor controller panel.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  ## \param ctlrpos Motor controller position string.
  #
  def createMotorCtlrPanel(self, parent, row, col, ctlrpos):
    wframe = LabelFrame(parent)
    wframe['text']  = "%s Motor Controller State" % (ctlrpos)
    wframe['font']  = ('Helvetica', 12)
    wframe['fg']    = UIColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=1, sticky=N+W+E)

    addr = self.m_motorCtlrAddr[ctlrpos]

    ctlrkey = _ctlrkey(ctlrpos)

    # common widget and grid options
    wcfgLabel = {'anchor': E, 'justify': RIGHT}
    gcfgLabel = {'padx': (5,2), 'pady': 2, 'sticky': E}
    wcfgValue = {'anchor': W, 'justify': LEFT, 'relief': 'solid'}
    gcfgValue = {'padx': (0,2), 'pady': 1, 'sticky': W}

    col = 0

    # packet address label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Address:'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # packet address
    self.m_var[ctlrkey]['addr'] = StringVar(); 
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['addr'], 'width': 8},
          wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    # baudrate label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Baudrate:'}, wcfgLabel),
        dictConcat({'row': 1, 'column': col}, gcfgLabel))

    # baudrate
    self.m_var[ctlrkey]['baudrate'] = IntVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['baudrate'],
          'width': 8}, wcfgValue),
        dictConcat({'row': 1, 'column': col+1}, gcfgValue))

    col += 2

    # motor controller model label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Model:'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # motor controller model
    self.m_var[ctlrkey]['model'] = StringVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['model'], 'width': 10},
          wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    # firmware version label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Version:'}, wcfgLabel),
        dictConcat({'row': 1, 'column': col}, gcfgLabel))

    # firmware version
    self.m_var[ctlrkey]['version'] = StringVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['version'],
          'width': 10}, wcfgValue),
        dictConcat({'row': 1, 'column': col+1}, gcfgValue))

    col += 2

    # maximum battery voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Max Battery (V):'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # maximum battery voltage
    self.m_var[ctlrkey]['batt_max'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['batt_max'],
          'width': 5}, wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    # minimum battery voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Min Battery (V):'}, wcfgLabel),
        dictConcat({'row': 1, 'column': col}, gcfgLabel))

    # minimum battery voltage
    self.m_var[ctlrkey]['batt_min'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['batt_min'],
          'width': 5}, wcfgValue),
        dictConcat({'row': 1, 'column': col+1}, gcfgValue))

    col += 2

    # maximum logic voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Max Logic (V):'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # maximum logic voltage
    self.m_var[ctlrkey]['logic_max'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['logic_max'],
          'width': 5}, wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    # minimum logic voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Min Logic (V):'}, wcfgLabel),
        dictConcat({'row': 1, 'column': col}, gcfgLabel))

    # minimum logic voltage
    self.m_var[ctlrkey]['logic_min'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['logic_min'],
          'width': 5}, wcfgValue),
        dictConcat({'row': 1, 'column': col+1}, gcfgValue))

    col += 2

    # battery voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Battery (V):'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # battery voltage
    self.m_var[ctlrkey]['battery'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['battery'], 'width': 5},
          wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    # logic voltage label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Logic (V):'}, wcfgLabel),
        dictConcat({'row': 1, 'column': col}, gcfgLabel))

    # logic voltage
    self.m_var[ctlrkey]['logic'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['logic'],
          'width': 5}, wcfgValue),
        dictConcat({'row': 1, 'column': col+1}, gcfgValue))

    col += 2

    # temperature label
    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Temp (C):'}, wcfgLabel),
        dictConcat({'row': 0, 'column': col}, gcfgLabel))

    # temperature
    self.m_var[ctlrkey]['temp'] = DoubleVar()
    self.makeWidget(wframe, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey]['temp'],
          'width': 6}, wcfgValue),
        dictConcat({'row': 0, 'column': col+1}, gcfgValue))

    col += 2

    wframe = Frame(wframe)
    wframe.grid(row=3, column=0, columnspan=col, padx=1, pady=5, sticky=N+W+E)

    row = 0

    for motorpos in self.m_motorPos:
      self.createMotorPanel(wframe, row, 0, ctlrkey, motorpos)
      row += 1

    wframe = Frame(wframe)
    wframe.grid(row=row, column=0, columnspan=col, padx=1, pady=5, sticky=N+W+E)

    self.createAlarmsPanel(wframe, 0, 0, ctlrkey)

  #
  ## \brief Create motor controller alarm panel.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrkey   Motor controller db key.
  ##
  def createAlarmsPanel(self, parent, row, col, ctlrkey):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)

    row = 0
    col = 0
 
    self.makeWidget(wframe, Label,
        {'text': "Alarms:",
          #'font':  ('Helvetica', 10),
          'fg':    UIColors['focus'],
          'borderwidth': 2},
        {'row': row, 'column': col, 'padx': 1, 'pady': 2, 'sticky': E})

    col += 1

    # Left motor over current alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Left Motor\nOver Current', 'alarm_lmoc')

    col += 1

    # Right motor over current alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Right Motor\nOver Current', 'alarm_rmoc')

    col += 1

    # Battery low voltage alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Battery\nLow Volt', 'alarm_batt_low')

    col += 1

    # Battery high voltage alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Battery\nHigh Volt', 'alarm_batt_high')

    col += 1

    # Logic low voltage alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Logic\nLow Volt', 'alarm_logic_low')

    col += 1

    # Logic high voltage alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Logic\nHigh Volt', 'alarm_logic_high')

    col += 1

    # Temperature alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Over\nTemperature', 'alarm_temp')

    col += 1

    # Emergency stop alarm
    self.createAlarmWidget(wframe, row, col, ctlrkey,
        'Emergency\nStopped', 'alarm_estop')

  #
  ## \brief Create alarm widgets and initalize alarm db.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrkey   Motor controller db key.
  ## \param text      Alarm label text.
  ## \param key       Alarm db key.
  #
  def createAlarmWidget(self, parent, row, col, ctlrkey, text, key):
    wframe = Frame(parent)
    wframe['borderwidth'] = 1
    wframe['relief']      = 'solid'
    wframe.grid(row=row, column=col, padx=0, pady=1, sticky=N+W+E)

    row = 0
    col = 0

    self.makeWidget(wframe, Label,
      {'text': text, 'justify': CENTER, 'anchor': CENTER},
      {'row':row, 'column':col, 'padx':(1, 1), 'pady':2, 'sticky':W+S+E})

    w = self.makeWidget(wframe, Label,
      {'image': self.m_icons['led_dark'], 'justify': CENTER, 'anchor': CENTER},
      {'row':row, 'column':col+1, 'padx':1, 'pady':(2,4), 'sticky':W+S+E})

    self.m_var[ctlrkey][key] = {'w': w, 'val': TriState['none']}


  #
  ## \brief Create motor controller motor panel.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrkey   Motor controller db key.
  ## \param motorpos  Motor position string.
  #
  def createMotorPanel(self, parent, row, col, ctlrkey, motorpos):
    self.makeWidget(parent, Label,
        {'text': "%s Motor:" % (motorpos),
          'font':  ('Helvetica', 10),
          'fg':    UIColors['focus'],
          'borderwidth': 2},
        {'row': row, 'column': col, 'padx': 1, 'pady': 2, 'sticky': N+W+E})

    motorkey = _motkey(motorpos)

    # common widget and grid options
    wcfgLabel = {'anchor': E, 'justify': RIGHT}
    gcfgLabel = {'padx': (5,2), 'pady': 2, 'sticky': E}
    wcfgValue = {'anchor': W, 'justify': LEFT, 'relief': 'solid'}
    gcfgValue = {'padx': (0,2), 'pady': 1, 'sticky': W}

    col = 1

    # encoder type label
    self.makeWidget(parent, Label,
        dictConcat({'text': 'Encoder Type:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    # encoder type
    self.m_var[ctlrkey][motorkey]['enc_type'] = StringVar()
    self.makeWidget(parent, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey][motorkey]['enc_type'],
          'width': 10}, wcfgValue),
        dictConcat({'row': row, 'column': col+1}, gcfgValue))

    col += 2

    # amps label
    self.makeWidget(parent, Label,
        dictConcat({'text': 'Amps:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    # amps
    self.m_var[ctlrkey][motorkey]['amps'] = DoubleVar()
    self.makeWidget(parent, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey][motorkey]['amps'],
          'width': 8}, wcfgValue),
        dictConcat({'row': row, 'column': col+1}, gcfgValue))

    col += 2

    # encoder label
    self.makeWidget(parent, Label,
        dictConcat({'text': 'Encoder:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    # encoder
    self.m_var[ctlrkey][motorkey]['encoder'] = IntVar()
    self.makeWidget(parent, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey][motorkey]['encoder'],
          'width': 12}, wcfgValue),
        dictConcat({'row': row, 'column': col+1}, gcfgValue))

    col += 2

    # speed label
    self.makeWidget(parent, Label,
        dictConcat({'text': 'Speed (QPPS):'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    # speed
    self.m_var[ctlrkey][motorkey]['speed'] = IntVar()
    self.makeWidget(parent, Label,
        dictConcat({'textvariable': self.m_var[ctlrkey][motorkey]['speed'],
          'width': 8}, wcfgValue),
        dictConcat({'row': row, 'column': col+1}, gcfgValue))

    col += 2

  #
  ## \brief Create velocity tuning panel.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  #
  def createVelTuningPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['relief'] = 'flat'
    wframe.grid(row=row, column=col, padx=1, pady=1, sticky=N+W+E)

    row = 0
    col = 0
        
    for ctlrpos in self.m_motorCtlrPos:
      self.createMotorsTuningPanel(wframe, row, col, ctlrpos)
      col += 1      

  #
  ## \brief Create motor controller motors tuning panel.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrpos   Motor controller position name.
  #
  def createMotorsTuningPanel(self, parent, row, col, ctlrpos):
    wframe = LabelFrame(parent)
    wframe['text']  = "%s Velocity Tuning" % (ctlrpos)
    wframe['font']  = ('Helvetica', 12)
    wframe['fg']    = UIColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=5, sticky=N+W+E)

    ctlrkey = _ctlrkey(ctlrpos)

    wcfgLabel = {'anchor': E, 'justify': RIGHT}
    gcfgLabel = {'padx': (5,2), 'pady': 2, 'sticky': E}
    wcfgValue = {'anchor': W, 'justify': LEFT, 'relief': 'solid'}
    gcfgValue = {'padx': (0,5), 'pady': 1, 'sticky': W}

    row = 0
    col = 0

    # PID subframe
    self.createPidParamsPanel(wframe, row, col, ctlrkey)

    # Setpoint subframe
    row = 1
    col = 0

    self.createSetpointsPanel(wframe, row, col, ctlrkey)

  #
  ## \brief Create motor controller motors PID parameters panel.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrkey   Motor controller db key.
  #
  def createPidParamsPanel(self, parent, row, col, ctlrkey):
    wframe = LabelFrame(parent)
    wframe['text'] = 'PID Parameters'
    #wframe['font'] = ('Helvetica', 10),
    wframe['fg']   = UIColors['focus'],
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=(5,1), pady=(5,0), sticky=N+W+E)

    wcfgLabel = {'anchor': E, 'justify': RIGHT}
    gcfgLabel = {'padx': (5,2), 'pady': 2, 'sticky': E}
    wcfgValue = {'anchor': W, 'justify': LEFT, 'relief': 'solid'}
    gcfgValue = {'padx': (0,5), 'pady': 1, 'sticky': W}

    #
    # PID labels
    #
    row = 1
    col = 0

    self.makeWidget(wframe, Label,
        dictConcat({'text': 'P:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    row += 1

    self.makeWidget(wframe, Label,
        dictConcat({'text': 'I:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    row += 1

    self.makeWidget(wframe, Label,
        dictConcat({'text': 'D:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    row += 1

    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Max QPPS:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    row += 1

    self.makeWidget(wframe, Label,
        dictConcat({'text': 'Max Accel:'}, wcfgLabel),
        dictConcat({'row': row, 'column': col}, gcfgLabel))

    col = 1

    #
    # Motor PID fields
    #
    for motorpos in self.m_motorPos:
      motorkey = _motkey(motorpos)

      row = 0

      self.makeWidget(wframe, Label,
        {'text': motorpos+' Motor', 'anchor': W, 'justify': LEFT},
        {'row': row, 'column': col, 'padx': (0,1), 'pady': 2, 'sticky': W})

      # P
      row += 1
      key = 'vel_pid_p'

      self.m_var[ctlrkey][motorkey][key] = DoubleVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        dictConcat({'row': row, 'column': col}, gcfgValue))

      # I
      row += 1
      key = 'vel_pid_i'

      self.m_var[ctlrkey][motorkey][key] = DoubleVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        dictConcat({'row': row, 'column': col}, gcfgValue))

      # D
      row += 1
      key = 'vel_pid_d'

      self.m_var[ctlrkey][motorkey][key] = DoubleVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        dictConcat({'row': row, 'column': col}, gcfgValue))

      # QPPS
      row += 1
      key = 'vel_pid_qpps'

      self.m_var[ctlrkey][motorkey][key] = DoubleVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        dictConcat({'row': row, 'column': col}, gcfgValue))

      # acceleration
      row += 1
      key = 'vel_pid_max_accel'

      self.m_var[ctlrkey][motorkey][key] = DoubleVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        dictConcat({'row': row, 'column': col}, gcfgValue))

      row += 1
      col += 1

      # current or new parameters in controller
      self.m_var[ctlrkey][motorkey]['vel_pid_params'] = PidParams.VelPidParams()

    # upwards arrow with left turn 
    self.makeWidget(wframe, Label,
      dictConcat({'text': '\u21b0'}, wcfgLabel),
      {'row': 1, 'column': col, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # spacer
    self.makeWidget(wframe, Label,
      dictConcat({'text': ''}, wcfgLabel),
      {'row': 2, 'column': col, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # link/unlink button
    def cblink(ck): return lambda: self.cbLinkCtlrPids(ck)
    icon_key = 'linked_v'
    w = Button(wframe)
    if self.m_icons[icon_key]:
      w['image']    = self.m_icons[icon_key]
      w['padx']     = 0
      w['pady']     = 0
      w['anchor']   = W
      w['width']    = 16
    else:
      w['text']     = 'link'
      w['anchor']   = CENTER
      w['width']    = 6
    self.m_var[ctlrkey]['pid_linked'] = {'w': w, 'val': True};
    w['command']    = cblink(ctlrkey)
    w.grid(row=3, column=col)

    # spacer
    self.makeWidget(wframe, Label,
      dictConcat({'text': ''}, wcfgLabel),
      {'row': 4, 'column': col, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # downward arrow with left turn
    self.makeWidget(wframe, Label,
      dictConcat({'text': '\u21b2'}, wcfgLabel),
      {'row': 5, 'column': col, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

  #
  ## \brief Create motor controller motor velocity setpoints panel.
  ##
  ## \param parent    Parent widget.
  ## \param row       Row in parent widget.
  ## \param col       Column in parent widget.
  ## \param ctlrkey   Motor controller db key.
  #
  def createSetpointsPanel(self, parent, row, col, ctlrkey):
    wframe = LabelFrame(parent)
    wframe['text'] = 'Setpoints'
    #wframe['font'] = ('Helvetica', 10),
    wframe['fg']   = UIColors['focus'],
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=(5,1), pady=(5,0),
            sticky=N+W+E+S)

    wcfgLabel = {'anchor': E, 'justify': RIGHT}
    gcfgLabel = {'padx': (5,2), 'pady': 2, 'sticky': E}
    wcfgValue = {'anchor': W, 'justify': LEFT, 'relief': 'solid'}
    gcfgValue = {'padx': (0,5), 'pady': 1, 'sticky': W}

    row = 0

    def cbspd(ck, mk, fk): return lambda: self.cbVelSetpoint(ck, mk, fk, 0)
    def cbpct(ck, mk, fk): return lambda v: self.cbVelSetpoint(ck, mk, fk, v)

    for motorpos in self.m_motorPos:
      motorkey = _motkey(motorpos)

      col = 0

      self.makeWidget(wframe, Label,
        dictConcat({'text': motorpos+' Motor:'}, wcfgLabel),
        {'row': row, 'column': col, 'padx': (2,2), 'pady': (1,0), 'sticky': W})

      col += 1

      key = 'setpoint_speed'

      self.m_var[ctlrkey][motorkey][key] = IntVar()

      self.makeWidget(wframe, Entry,
        {'textvariable': self.m_var[ctlrkey][motorkey][key],
          'validate': 'focusout',
          'validatecommand': cbspd(ctlrkey, motorkey, key),
          'width': 8, 'justify': LEFT, 'relief': 'sunken'},
        {'row': row, 'column': col,
          'padx': (2,2), 'pady': (1,0), 'sticky': E})

      row += 1
      col = 0
      key = 'setpoint_percent'

      self.m_var[ctlrkey][motorkey][key] = IntVar()

      # slider
      self.makeWidget(wframe, Scale,
        {'variable': self.m_var[ctlrkey][motorkey][key],
          'command': cbpct(ctlrkey, motorkey, key),
         'from_': -100, 'to': 100, 'resolution': 1, 'orient': HORIZONTAL,
         'tickinterval': 50, 'length': 210},
        {'row': row, 'column': col, 'columnspan': 2,
          'padx': (2,2), 'pady': (0,0), 'sticky': W})

      row += 1

      # spacer
      if motorpos == 'Left':
        self.makeWidget(wframe, Label,
            dictConcat({'text': ''}, wcfgLabel),
        {'row': row, 'column': col, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

      row += 1

    # upwards arrow with left turn 
    self.makeWidget(wframe, Label,
      dictConcat({'text': '\u21b0'}, wcfgLabel),
      {'row': 0, 'column': 2, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # spacer
    self.makeWidget(wframe, Label,
      dictConcat({'text': ''}, wcfgLabel),
      {'row': 1, 'column': 2, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # link/unlink button
    def cblink(ck): return lambda: self.cbLinkCtlrSetpoints(ck)
    icon_key = 'linked_v'
    w = Button(wframe)
    if self.m_icons[icon_key]:
      w['image']    = self.m_icons[icon_key]
      w['padx']     = 0
      w['pady']     = 0
      w['anchor']   = W
      w['width']    = 16
    else:
      w['text']     = 'link'
      w['anchor']   = CENTER
      w['width']    = 6
    self.m_var[ctlrkey]['setpoint_linked'] = {'w': w, 'val': True};
    w['command']    = cblink(ctlrkey)
    w.grid(row=2, column=2)

    # spacer
    self.makeWidget(wframe, Label,
      dictConcat({'text': ''}, wcfgLabel),
      {'row': 3, 'column': 2, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

    # downward arrow with left turn
    self.makeWidget(wframe, Label,
      dictConcat({'text': '\u21b2'}, wcfgLabel),
      {'row': 4, 'column': 2, 'padx': (2,2), 'pady': (1,1), 'sticky': W})

  #
  ## \brief Create real-time plot panel.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  ## \param width   Total available width of plot panel in pixels.
  #
  def createPlotPanel(self, parent, row, col, width):
    wframe = LabelFrame(parent)
    wframe['text']  = "Real-Time Velocity Plot"
    wframe['font']  = ('Helvetica', 12)
    wframe['fg']    = UIColors['focus']
    wframe['borderwidth'] = 2
    wframe['relief']      = 'ridge'
    wframe.grid(row=row, column=col, columnspan=2, padx=1, pady=1, sticky=N+W+E)

    subwidth = self.createPlotControls(wframe, 0, 0)
    self.perfMark("Created plot controls")

    self.createPlotCanvas(wframe, 0, 1, width-subwidth)
    self.perfMark("Created plot canvas")

  #
  ## \brief Create real-time plot controls.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  ##
  ## \return Total available width of control frame in pixels.
  #
  def createPlotControls(self, parent, row, col):
    wframe = Frame(parent)
    wframe['borderwidth'] = 0
    wframe['relief']      = 'flat'
    wframe.grid(row=row, column=col, padx=1, pady=1, sticky=N+W+E)

    row = 0
    col = 0

    def cben(ck, mk, fk): return lambda: self.cbEnDisVelPlot(ck, mk, fk)

    ctlrkey   = "front_ctlr"
    motorkey  = "left_motor"
    fieldkey  = "plotvel"

    # left front graph color 
    self.makeWidget(wframe, Label,
        {'text': '     ', 'bg': UIColors['left_front'],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col, 'padx': (5,1), 'pady': (1,1), 'sticky': W})

    # left front motor checkbutton
    self.m_var[ctlrkey][motorkey][fieldkey] = IntVar(); 

    self.makeWidget(wframe, Checkbutton,
        {'text': 'Left Front', 'command': cben(ctlrkey, motorkey, fieldkey),
          'variable': self.m_var[ctlrkey][motorkey][fieldkey],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col+1, 'padx': (1,5), 'pady': (1,1), 'sticky': W})

    row += 1

    motorkey  = "right_motor"

    # right front graph color 
    self.makeWidget(wframe, Label,
        {'text': '     ', 'bg': UIColors['right_front'],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col, 'padx': (5,1), 'pady': (1,1), 'sticky': W})

    # right front motor checkbutton
    self.m_var[ctlrkey][motorkey][fieldkey] = IntVar(); 

    self.makeWidget(wframe, Checkbutton,
        {'text': 'Right Front', 'command': cben(ctlrkey, motorkey, fieldkey),
          'variable': self.m_var[ctlrkey][motorkey][fieldkey],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col+1, 'padx': (1,5), 'pady': (1,1), 'sticky': W})

    row += 1

    ctlrkey   = "rear_ctlr"
    motorkey  = "left_motor"

    # left rear graph color 
    self.makeWidget(wframe, Label,
        {'text': '     ', 'bg': UIColors['left_rear'],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col, 'padx': (5,1), 'pady': (1,1), 'sticky': W})

    # left rear motor checkbutton
    self.m_var[ctlrkey][motorkey][fieldkey] = IntVar(); 

    self.makeWidget(wframe, Checkbutton,
        {'text': 'Left Rear', 'command': cben(ctlrkey, motorkey, fieldkey),
          'variable': self.m_var[ctlrkey][motorkey][fieldkey],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col+1, 'padx': (1,5), 'pady': (1,1), 'sticky': W})

    row += 1

    motorkey  = "right_motor"

    # right rear graph color 
    self.makeWidget(wframe, Label,
        {'text': '     ', 'bg': UIColors['right_rear'],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col, 'padx': (5,1), 'pady': (1,1), 'sticky': W})

    # right rear motor checkbutton
    self.m_var[ctlrkey][motorkey][fieldkey] = IntVar(); 

    self.makeWidget(wframe, Checkbutton,
        {'text': 'Right Rear', 'command': cben(ctlrkey, motorkey, fieldkey),
          'variable': self.m_var[ctlrkey][motorkey][fieldkey],
          'anchor': W, 'justify': LEFT},
      {'row': row, 'column': col+1, 'padx': (1,5), 'pady': (1,1), 'sticky': W})

    row += 1

    self.update_idletasks()
    width = wframe.winfo_width()
    #print('DBG: plotcontrols width =', width)

    return width

  #
  ## \brief Create real-time plot canvas.
  ##
  ## \param parent  Parent widget.
  ## \param row     Row in parent widget.
  ## \param col     Column in parent widget.
  ## \param width   Total available width of canvas in pixels.
  #
  def createPlotCanvas(self, parent, row, col, width):
    wframe = Frame(parent)
    wframe['borderwidth'] = 0
    wframe['relief']      = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=1, sticky=N+W+E)

    # width and height in pixels, subbing out margins
    width  -= 40
    height  = 400

    wCanvas = Canvas(wframe, height=height, width=width)
    wCanvas.grid(row=0, column=0)

    self.m_plotVel = VelPlot.VelPlot(wCanvas, width, height, 
        ['left_front', 'right_front', 'left_rear', 'right_rear'],
        UIColors)

    self.update_idletasks()

  #
  ## \brief Make widgets from widget description table.
  ##
  ## The description table is a list of wdesc widget description dictionaries.
  ## Each wdesc dictionary must have the following keys and values:
  ##  'widget': W - W is a Tkinter widget class (e.g. Label, button,
  ##                  listbox, etc).
  ##  'wcfg': D   - D is a dictionary of widget-specific configuration options.
  ##  'gcfg': D   - D is a dictionary of grid options.
  ##
  ## \param parent    Parent widget.
  ## \param wdescTbl  List of [wdesc, wdesc,...].
  #
  def makeWidgets(self, parent, wdescTbl):
    for wdesc in wdescTbl:
      self.makeWidget(parent, wdesc['widget'], wdesc['wcfg'], wdesc['gcfg'])

  #
  ## \brief Make widget.
  ##
  ## \param parent  Parent widget.
  ## \param widget  Tkinter widget class.
  ## \param wcfg    Dictionary of widget-specific configuration options.
  ## \param gcfg    Dictionary of grid options.
  ##
  ## \return Returns made widget.
  #
  def makeWidget(self, parent, widget, wcfg, gcfg):
    w = widget(parent, **wcfg)
    w.grid(**gcfg)
    return w

  #
  ## \brief Create gui status bar at bottom of gui window.
  #
  def createStatusBar(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=2, column=0, columnspan=3, padx=1, pady=3, sticky=N+E+W+S)

    self.m_varStatus = StringVar()
    self.m_varStatus.set("Calibration required.")
    self.m_wStatusBar = Entry(wframe)
    self.m_wStatusBar['width']    = wframe['width']
    self.m_wStatusBar['relief']   = 'flat'
    self.m_wStatusBar['textvar']  = self.m_varStatus
    self.m_wStatusBar['fg']       = UIColors['normal']
    self.m_wStatusBar['state']    = 'readonly'
    self.m_wStatusBar.grid(row=0, column=0, padx=3, pady=3, sticky=N+E+W+S)

  #
  ## \brief Update button activation states.
  #
  def updateButtonState(self, keys, state):
    for key in keys:
      self.m_wBttn[key]['state'] = state

  #
  ## \brief Create button.
  ##
  ## \param parent    Parent widget.
  ## \param text      Button text.
  ## \param imagefile Image file name. None for no image.
  ## \param command   Callback for button push.
  ## \param fg        Foreground text color.
  ##
  ## \return Button widget.
  #
  def createButton(self, parent, text, imagefile, command, fg='black'):
    key = str.lower(text.replace("\n", "_"))
    self.m_icons[key] = self.m_imageLoader.load(imagefile)
    w = Button(parent)
    w['text']     = text
    if self.m_icons[key]:
      w['image']    = self.m_icons[key]
      w['compound'] = LEFT
      w['padx']     = 0
      w['pady']     = 0
      w['anchor']   = W
      w['width']    = 105
    else:
      w['anchor']   = CENTER
      w['width']    = 10
    w['fg']       = fg
    w['command']  = command
    self.m_wBttn[key] = w
    return self.m_wBttn[key]

  #
  ## \brief Destroy window callback.
  #
  def destroy(self):
    self.quit()

  #
  ## \brief Not implemented callback.
  #
  def notimpl(self):
    emsg = "Window function not implemented yet."
    self.showError(emsg)
    print(emsg)

  #
  ## \brief Apply tuning tweaks to controllers callback.
  #
  def apply(self):
    pass
  
  #
  ## \brief Save tuning to tuning file and controller EEPROM callback.
  #
  def save(self):
    pass
  
  #
  ## \brief Stop Laelaps
  #
  def stop(self):
    pass
  
  #
  ## \brief Show about dialog callback.
  #
  def about(self):
    prodInfo = self.getProductInfo()
    dlg = AboutDlg(master=self, info=prodInfo, app_ver=AppVersion)
  
  def cbLinkAllPids(self):
    pass

  def cbLinkCtlrPids(self, ck):
    fk = 'pid_linked'
    if self.m_var[ck][fk]['val']:
      self.m_var[ck][fk]['w']['image'] = self.m_icons['unlinked_v']
      self.m_var[ck][fk]['val'] = False
    else:
      self.m_var[ck][fk]['w']['image'] = self.m_icons['linked_v']
      self.m_var[ck][fk]['val'] = True

  def cbLinkAllSetpoints(self):
    pass

  def cbLinkCtlrSetpoints(self, ck):
    fk = 'setpoint_linked'
    if self.m_var[ck][fk]['val']:
      self.m_var[ck][fk]['w']['image'] = self.m_icons['unlinked_v']
      self.m_var[ck][fk]['val'] = False
    else:
      self.m_var[ck][fk]['w']['image'] = self.m_icons['linked_v']
      self.m_var[ck][fk]['val'] = True

  #
  ## \brief Setpoint changed callback.
  ##
  ## \param ck  Controller key.
  ## \param mk  Motor key.
  ## \param fk  Field key.
  ## \param v   Value (ignored).
  ##
  ## \return True
  #
  def cbVelSetpoint(self, ck, mk, fk, v):
    if self.m_plotVel is None:
      return True
    val = self.m_var[ck][mk][fk].get()
    pk = self.m_powertrain[ck][mk]
    self.m_plotVel.setpoint(pk, val)
    return True

  #
  ## \brief Enable/disable velocity plot.
  ##
  ## \param ck  Controller key.
  ## \param mk  Motor key.
  ## \param fk  Field key.
  #
  def cbEnDisVelPlot(self, ck, mk, fk):
    if self.m_plotVel is None:
      return
    name = self.m_powertrain[ck][mk]
    val = self.m_var[ck][mk][fk].get()
    sp = self.m_var[ck][mk]['setpoint_speed'].get()
    if val:
      self.m_plotVel.enable(name, sp)
    else:
      self.m_plotVel.disable(name)

  def setPidParams(self, ck, mk, Kp, Ki, Kd, maxQpps, maxAccel):
    # UI fields
    self.m_var[ck][mk]['vel_pid_p'].set(Kp)
    self.m_var[ck][mk]['vel_pid_i'].set(Ki)
    self.m_var[ck][mk]['vel_pid_d'].set(Kd)
    self.m_var[ck][mk]['vel_pid_qpps'].set(maxQpps)
    self.m_var[ck][mk]['vel_pid_max_accel'].set(maxAccel)

    # new/current PID parameters
    self.m_var[ck][mk]['vel_pid_params'].m_Kp = Kp
    self.m_var[ck][mk]['vel_pid_params'].m_Ki = Ki
    self.m_var[ck][mk]['vel_pid_params'].m_Kd = Kd
    self.m_var[ck][mk]['vel_pid_params'].m_maxQpps  = maxQpps
    self.m_var[ck][mk]['vel_pid_params'].m_maxAccel = maxAccel

  #
  ## \brief Get product information.
  ## 
  ## \return Returns product information on success, None on failure.
  #
  def getProductInfo(self):
    prodInfo = None
    try:
      rospy.wait_for_service("laelaps_control/get_product_info", timeout=5)
    except rospy.ROSException as e:
      self.showError('Get product info: ' + e.message + '.')
    else:
      try:
        get_product_info = rospy.ServiceProxy(
                                          'laelaps_control/get_product_info',
                                          GetProductInfo)
        rsp = get_product_info()
        prodInfo = rsp.i
      except rospy.ServiceException as e:
        self.showError("Get product info request failed: %s." % (e.message))
    return prodInfo

  #
  ## \brief Get Laelaps name.
  ## 
  ## \return Returns name.
  #
  def getLaelapsName(self):
    name = "laelaps"
    try:
      rospy.wait_for_service("laelaps_control/get_product_info", timeout=5)
    except rospy.ROSException as e:
      rospy.logerr("Get product info: %s." % (e.message))
    else:
      try:
        get_product_info = rospy.ServiceProxy(
                                          'laelaps_control/get_product_info',
                                          GetProductInfo)
        rsp = get_product_info()
        name = rsp.i.hostname
      except rospy.ServiceException as e:
        rospy.logerr("Get product info request failed: %s." % (e.message))
    return name

  #
  ## \brief Update motor status.
  ##
  #
  def updateStatus(self):
    pass

  #
  ## \brief Show information message on status bar.
  ##
  ## \param msg   Info message string.
  #
  def showInfo(self, msg):
    self.m_wStatusBar["state"] = "normal"
    self.m_wStatusBar["fg"]    = UIColors['normal']
    self.m_varStatus.set(msg)
    self.m_wStatusBar["state"] = "readonly"

  #
  ## \brief Show error message on status bar.
  ##
  ## \param msg   Error message string.
  #
  def showError(self, msg):
    self.m_wStatusBar["state"] = "normal"
    self.m_wStatusBar["fg"]    = UIColors['error']
    self.m_varStatus.set(msg)
    self.m_wStatusBar["state"] = "readonly"

  #
  ## \brief Show text on read-only entry.
  ##
  ## \param w     Entry widget.
  ## \param var   Bound entry variable.
  ## \param val   Variable value.
  ## \param fg    Text foreground color.
  #
  def showEntry(self, w, var, val, fg='black'):
    w['state'] = 'normal'
    w['fg']    = fg
    var.set(val)
    w['state'] = 'readonly'

  #
  ## \brief Map alignment value to justify equivalent.
  ##
  ## \param align   Alignment.
  ## 
  ## \return Tk justify.
  #
  def alignToJustify(self, align):
    if align == W:
      return LEFT
    elif align == E:
      return RIGHT
    else:
      return CENTER

  #
  ## \brief Final window initializations.
  ##
  ## Both the window data and widgets, along with ROS node application, have
  ## been fully initialized.
  #
  def finalInits(self):
    self.m_botName   = self.getLaelapsName()
    self.master.title("Laelaps Control Panel - %s" % (self.m_botName))
    self.m_wTopHeading['text'] = "Laelaps Control Panel - %s" % \
        (self.m_botName)


# ------------------------------------------------------------------------------
# Exception Class usage
# ------------------------------------------------------------------------------

##
## \brief Unit test command-line exception class.
##
## Raise usage excpetion.
##
class usage(Exception):

  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.msg = msg


# ------------------------------------------------------------------------------
# Class application
# ------------------------------------------------------------------------------

##
## \brief Laelaps control panel.
##
class application():

  #
  ## \brief Constructor.
  #
  def __init__(self):
    self._Argv0 = os.path.basename(__file__)
    self.m_win = None

  #
  ## \brief Print usage error.
  ##
  ## \param emsg  Error message string.
  #
  def printUsageErr(self, emsg):
    if emsg:
      print("%s: %s" % (self._Argv0, emsg))
    else:
      print("%s: error" % (self._Argv0))
    print("Try '%s --help' for more information." % (self._Argv0))

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print(\
"""
usage: %s [OPTIONS]
       %s --help

Options and arguments:
-h, --help                : Display this help and exit.
"""  % (self._Argv0, self._Argv0))
 
  #
  ## \brief Get command-line options
  ##  
  ## \param argv          Argument list. If not None, then overrides
  ##                      command-line arguments.
  ## \param [out] kwargs  Keyword argument list.  
  ##
  ## \return Parsed keyword arguments.
  #
  def getOptions(self, argv=None, **kwargs):
    if argv is None:
      argv = sys.argv

    self._Argv0 = os.path.basename(kwargs.get('argv0', __file__))

    # defaults
    kwargs['debug'] = False

    # parse command-line options
    try:
      opts, args = getopt.getopt(argv[1:], "?h",
          ['help', ''])
    except getopt.error as msg:
      raise usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        self.printUsage()
        sys.exit(0)

    #if len(args) < 1:
    #  self.printUsageErr("No input xml file specified")
    #  sys.exit(2)
    #else:
    #  kwargs['filename'] = args[0]

    return kwargs

  #
  ## \brief Initialize interface to hek_robot.
  #
  def initRobot(self):
    self.m_win.showInfo("Initializing interface to Laelaps.")

    self.m_win.showInfo("Laelaps motor interface initialized.")

  #
  ## \brief Run application.
  ##    
  ## \param argv    Optional argument list to override command-line arguments.
  ## \param kwargs  Optional keyword argument list.
  ##
  ## \return Exit code.
  #
  def run(self, argv=None, **kwargs):
  
    # parse command-line options and arguments
    try:
      kwargs = self.getOptions(argv, **kwargs)
    except usage as e:
      print(e.msg)
      return 2

    # create root 
    root = Tk()

    # create application window
    self.m_win = window(master=root)

    # destroy window on 'x'
    root.protocol('WM_DELETE_WINDOW', root.destroy)

    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)

    # initialize robot interface
    self.initRobot()

    # go for it
    self.m_win.mainloop()

    return 0


# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
