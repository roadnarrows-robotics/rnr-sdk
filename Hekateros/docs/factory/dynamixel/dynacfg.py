#! /usr/bin/env python

###############################################################################
#
# Package:  Hekateros
#
# File: dynacfg.py
#
## \file 
##
## $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
## $Rev: 3748 $
##
## \brief Configure Hekateros servo.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2013-2017. RoadNarrows LLC.\n
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
import getopt

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

from PIL import Image, ImageTk

# ------------------------------------------------------------------------------
# Globals
# ------------------------------------------------------------------------------

## \brief Application version. Update as needed. 
appVersion = '1.0.0'

## \brief Image search paths.
imagePath = [
  "/prj/pkg/Hekateros/share/images",
  "/usr/local/share/Hekateros/images",
  "/prj/pkg/appkit/share/images",
  "/usr/local/share/appkit/images"
]


hekScriptPath = "/prj/pkg/Hekateros/docs/factory/dynamixel"

hekProducts = ['4S', '4L', '5S', '5L']

hekDynaScripts = {
  '4S':
    { 'base_rot':     None,
      'shoulder_m':   'shoulder_m.scr', 
      'shoulder_s':   'shoulder_s.scr', 
      'elbow':        'elbow.scr', 
      'wrist_pitch':  'wrist_pitch.scr', 
      'wrist_rot':    'wrist_rot.scr', 
      'gripper':      'gripper.scr'
    },
  '4L':
    { 'base_rot':     None,
      'shoulder_m':   'shoulder_m.scr', 
      'shoulder_s':   'shoulder_s.scr', 
      'elbow':        'elbow.scr', 
      'wrist_pitch':  'wrist_pitch.scr', 
      'wrist_rot':    'wrist_rot.scr', 
      'gripper':      'gripper.scr'
    },
  '5S':
    { 'base_rot':     'base_rot.scr', 
      'shoulder_m':   'shoulder_m.scr', 
      'shoulder_s':   'shoulder_s.scr', 
      'elbow':        'elbow.scr', 
      'wrist_pitch':  'wrist_pitch.scr', 
      'wrist_rot':    'wrist_rot.scr', 
      'gripper':      'gripper.scr'
    },
  '5L':
    { 'base_rot':     'base_rot.scr', 
      'shoulder_m':   'shoulder_m.scr', 
      'shoulder_s':   'shoulder_s.scr', 
      'elbow':        'elbow.scr', 
      'wrist_pitch':  'wrist_pitch.scr', 
      'wrist_rot':    'wrist_rot.scr', 
      'gripper':      'gripper.scr'
    }
}

hekBaudRates = [9600, 19200, 57600, 115200, 200000, 400000, 500000, 1000000]

## \brief Common foreground colors.
fgColors = {
  'normal':   'black',
  'ok':       '#008800',
  'focus':    '#0000aa',
  'warning':  '#aa6600',
  'error':    '#cc0000'
}

# ------------------------------------------------------------------------------
# Utilities
# ------------------------------------------------------------------------------

#
## \brief Load icon image from file name.
##
## \param filename    Icon file name.
##
## \return Returns icon widget on success, None on failure.
#
def loadIcon(filename):
  # no file name
  if filename is None or len(filename) == 0:
    return None;
  # absolute file name
  if filename[0] == os.path.sep:
    try:
      return ImageTk.PhotoImage(Image.open(filename))
    except IOError:
      return None
  # relative file name - search path for file
  for path in imagePath:
    fqname = path + os.path.sep + filename
    try:
      return ImageTk.PhotoImage(Image.open(fqname))
    except IOError:
      continue
  return None

#
## Round to nearest 100th.
#
def round100th(x):
  return math.floor((x + 0.005) * 100.0) / 100.0

#
## Round to nearest 10th.
#
def round10th(x):
  return math.floor((x + 0.05) * 10.0) / 10.0

#
## Degrees to radians.
#
def degToRad(deg):
  return deg / 180.0 * math.pi

#
## Radians to degrees.
#
def radToDeg(rad):
  return rad / math.pi * 180.0



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
    # intialize window data
    kw = self.initData(kw)

    Frame.__init__(self, master=master, cnf=cnf, **kw)
    self.master.title("Hekateros Factory Servo Configuration")
    self.grid(row=0, column=0, padx=5, pady=5)

    # craete and show widgets
    self.createWidgets()

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
    self.m_icons = {}
    self.m_debug = False;
    self.m_uri = '/dev/ttyUSB0'
    self.m_baudrate = 57600
    self.m_product = None
    self.m_joint = None

    if kw.has_key('debug'):
      self.m_debug = kw['debug']
      del kw['debug']
    if kw.has_key('arm'):
      self.m_product = kw['arm']
      del kw['arm']
    if kw.has_key('joint'):
      self.m_joint = kw['joint']
      del kw['joint']
    if kw.has_key('uri'):
      self.m_uri = kw['uri']
      del kw['uri']
    if kw.has_key('baudrate'):
      self.m_baudrate = kw['baudrate']
      del kw['baudrate']

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    self.createHeading(self, 0, 0)
    self.createCfgPanel(self, 1, 0)
    self.createStatusBar(self, 2, 0)

  #
  ## \brief Create top gui heading.
  #
  def createHeading(self, parent, row, col):
    wframe = Frame(parent)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)
    self.m_wJointStateFrame = wframe

    # rn logo
    w = Label(wframe)
    self.m_icons['rn_logo'] = loadIcon("RNLogo48.png");
    if self.m_icons['rn_logo']:
      w['image'] = self.m_icons['rn_logo']
    else:
      w['text'] = 'rn'
      w['anchor'] = W
      w['width'] = 5
    w.grid(row=0, column=0, sticky=W)
    
    # top heading
    w = Label(wframe)
    w['font']   = ('Helvetica', 16)
    w['text']   = 'Hekateros Factory Servo Configuration'
    w['anchor'] = CENTER
    w['width'] = 40
    w.grid(row=0, column=1, sticky=E+W)

    # hek logo
    w = Label(wframe)
    self.m_icons['hek_logo'] = loadIcon("icons/icon_hek_logo.png");
    if self.m_icons['hek_logo']:
      w['image'] = self.m_icons['hek_logo']
      w['anchor'] = E
    else:
      w['text'] = 'hek'
      w['anchor'] = E
      w['width'] = 5
    w.grid(row=0, column=2, sticky=E)
 
  #
  ## \brief Create joint state lower center panel headers.
  ##
  ## \param parent  Parent widget
  #
  def createCfgPanel(self, parent, row, col):
    wframe = Frame(parent)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=row, column=col, padx=1, pady=3, sticky=N+W+E)
    self.m_wJointStateFrame = wframe

    # heading
    w = Label(wframe)
    w['font'] =('Helvetica', 12)
    w['text'] = 'Servo Configuration'
    w['anchor'] = CENTER
    w.grid(row=0, column=0, columnspan=10, sticky=E+W)

    width = 12
    padx  = 8
    pady  = 3
    row   = 1
    col   = 0

    # column labels, line 1
    for text in [' ', 'Target', 'Current', 'Current', ' ']:
      w = Label(wframe, width=width, padx=padx, pady=pady, anchor=CENTER,
                text=text)
      w.grid(row=row, column=col, pady=0, sticky=W+E)
      col += 1

    row += 1
    col  = 0

    # column labels, line 2
    for text in ['Joint Servo', 'Servo Id', 'Servo Id', 'Baudrate',
            'Configure']:
      w = Label(wframe, width=width, padx=padx, pady=0, anchor=CENTER,
                text=text)
      w.grid(row=row, column=col, sticky=W+E)
      col += 1

  #
  ## \brief Create gui status bar at bottom of gui window.
  #
  def createStatusBar(self, parent, row, col):
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
    self.m_wStatusBar['fg']       = fgColors['normal']
    self.m_wStatusBar['state']    = 'readonly'
    self.m_wStatusBar.grid(row=0, column=0, padx=3, pady=3, sticky=N+E+W+S)




# ------------------------------------------------------------------------------
# Class application
# ------------------------------------------------------------------------------

##
## \brief Configure Hekateros servo.
##
class application():

  ## \brief Unit test constructor.
  def __init__(self):
    self._Argv0 = __file__
    self.m_win = None

  ## \brief Print usage error.
  ##
  ## \param emsg  Error message string.
  def printUsageErr(self, emsg):
    if emsg:
      print "%s: %s" % (self._Argv0, emsg)
    else:
      print "%s: error" % (self._Argv0)
    print "Try '%s --help' for more information." % (self._Argv0)

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print \
"""
usage: %s [OPTIONS] <arm> <joint>
       %s --help

Options and arguments:
-u, --uri=<device_uri>    : Dynamixel serial device uri.
                              SYNTAX:  [botsense://[hostname][:port]]/device
                              DEFAULT:  /dev/ttyUSB0
-b, --baudrate=<baud>     : Dynamixel serial device baud rate.
                              DEFAULT:  1000000
-h, --help                : Display this help and exit.

<arm>                     : Hekateros product id. One of: 4S 4L 5S 5L
<joint>                   : Hekateros joint servo. One of:
                              base_rot shoulder_m shoulder_s elbow
                              wrist_pitch wrist_rot gripper
"""  % (self._Argv0, self._Argv0)
 
  ## \brief Get command-line options
  ##  
  ## \param argv          Argument list. If not None, then overrides
  ##                      command-line arguments.
  ## \param [out] kwargs  Keyword argument list.  
  def getOptions(self, argv=None, **kwargs):
    if argv is None:
      argv = sys.argv

    self._Argv0 = kwargs.get('argv0', __file__)

    # defaults
    kwargs['debug']     = 0
    kwargs['uri']       = '/dev/ttyUSB0'
    kwargs['baudrate']  = 1000000

    # parse command-line options
    try:
      opts, args = getopt.getopt(argv[1:], "?hu:b:",
            ['help', 'uri=', 'baudrate='])
    except getopt.error, msg:
      raise usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        self.printUsage()
        sys.exit(0)
      elif opt in ('-u', '--uri'):
        kwargs['uri'] = optarg
      elif opt in ('-b', '--baudrate'):
        kwargs['baudrate'] = optarg

    if len(args) < 1:
      self.printUsageErr("No arm product specified.")
      sys.exit(2)
    else:
      kwargs['arm'] = args[0]

    if len(args) < 2:
      self.printUsageErr("No servo id specified.")
      sys.exit(2)
    else:
      kwargs['joint'] = args[1]

    return kwargs

  ## \brief Run application.
  ##    
  ## \param argv    Optional argument list to override command-line arguments.
  ## \param kwargs  Optional keyword argument list.
  def run(self, argv=None, **kwargs):
  
    # parse command-line options and arguments
    kwargs = self.getOptions(argv, **kwargs)

    if hekDynaScripts.has_key(kwargs['arm']):
      scripts = hekDynaScripts[kwargs['arm']]
    else:
      self.printUsageErr("%s: Unknown hekateros product key." % (kwargs['arm']))
      return 2
      
    if scripts.has_key(kwargs['joint']):
      cfg_script = scripts[kwargs['joint']]
    else:
      self.printUsageErr("%s: Unknown joint." % (kwargs['joint']))
      return 2
      
    # create root 
    root = Tk()

    # create application window
    self.m_win = window(master=root, **kwargs)

    root.protocol('WM_DELETE_WINDOW', root.destroy)

    # go for it
    self.m_win.mainloop()

    return 0

    fqscript = hekScriptPath + os.path.sep + cfg_script

    cmd = "dynashell --uri=%s --baudrate=%s --script=%s" % \
            (kwargs['uri'], kwargs['baudrate'], fqscript)

    ec = os.system(cmd)

    return 0


# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
