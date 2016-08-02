#! /usr/bin/env python

###############################################################################
#
# Package:   RoadNarrows Robotics Eudoxus Console
#
# Link:      https://github.com/roadnarrows-robotics/eudoxus
#
# File: eudoxus_console
#
## \file 
##
## $LastChangedDate: 2016-03-18 09:57:27 -0600 (Fri, 18 Mar 2016) $
## $Rev: 4354 $
##
## \brief Graphical user interface console to control the Eudoxus init.d
## daemons and user services.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import math
import subprocess
import psutil
import re
import threading
import getopt

from Tkinter import *
from Tkconstants import *
from tkFileDialog import *
import tkFont

# running as su does not necessary have all paths setup - so fix up here
sys.path.insert(0, "/usr/local/lib/python2.7/site-packages")
sys.path.insert(0, "/prj/lib/python2.7/site-packages")

from Eudoxus.Utils import *

## \brief Application version. Update as needed. 
appVersion = '1.0.0'

## \brief Additional image search paths.
imagePath = [
  "/prj/share/Eudoxus/images",
  "/prj/share/Eudoxus/images/icons",
  "/prj/share/appkit/images",
  "/prj/share/appkit/images/icons",
  "/usr/local/share/Eudoxus/images",
  "/usr/local/share/Eudoxus/images/icons"
  "/usr/local/share/appkit/images",
  "/usr/local/share/appkit/images/icons"
]

## \brief Common foreground colors.
fgColors = {
  'normal':   'black',
  'ok':       '#008800',
  'focus':    '#0000aa',
  'warning':  '#aa6600',
  'error':    '#cc0000'
}

## \brief Status text foreground colors.
statusText = {
  'unknown': fgColors['normal'],
  'running': fgColors['ok'],
  'stopped': fgColors['error']
}

# pre-compiled regular expressions
reDone              = re.compile(r"done", re.IGNORECASE)
reFail              = re.compile(r"fail", re.IGNORECASE)
reDoneDone          = re.compile(r"done.*done", re.DOTALL | re.IGNORECASE)
reFailDone          = re.compile(r"fail.*done", re.DOTALL | re.IGNORECASE)
reRunning           = re.compile(r"\s*is\s*running", re.IGNORECASE)
reNotRunning        = re.compile(r"\s*is\s*not\s*running", re.IGNORECASE)
rePython            = re.compile(r".*python", re.IGNORECASE)
reRosLaunch         = re.compile(r".*roslaunch", re.IGNORECASE)
reRosOpenni2        = re.compile(r".*openni2\.launch", re.IGNORECASE)
reRosImageView      = re.compile(r".*image_view", re.IGNORECASE)
reRosImageViewImage = re.compile(r"image:=/camera/depth/image", re.IGNORECASE)


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

    self.m_imageLoader = ImageLoader(py_pkg='Eudoxus.images',
                                      image_paths=imagePath)

    Frame.__init__(self, master=master, cnf=cnf, **kw)
    self.master.title("Eudoxus Console")

    self.m_icons['app_icon'] = \
                    self.m_imageLoader.loadImage("icons/EudoxusInitIcon.png")
    if self.m_icons['app_icon'] is not None:
      self.master.tk.call('wm', 'iconphoto', self.master._w,
        self.m_icons['app_icon'])

    # craete and show widgets
    self.createWidgets()

    self.grid(row=0, column=0, padx=5, pady=5)

    self.after(100, self.autoRefresh)

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
    self.m_debug          = False # default debug level
    self.m_icons          = {}    # must keep loaded icons referenced
    self.m_wBttn          = {}    # button widgets
    self.m_svcKeys        = [
      'eudoxus_roscore',  'eudoxus_shutter',
      'openni2_launch',   'image_view']
    self.m_svc = {
      'eudoxus_roscore': {
        'desc':   'ROS Master, Parameter Server, rosout logging node.',
        'type':   'init.d',
        'status': 'unknown',
      },
      'eudoxus_shutter': {
        'desc':   'Eudoxus user button monitor.',
        'type':   'init.d',
        'status': 'unknown',
      },
      'openni2_launch': {
        'desc':   'ROS OpenNI2 camera drivers and RGBD launch files.',
        'type':   'user',
        'status': 'unknown',
      },
      'image_view': {
        'desc':   'ROS depth disparity viewer.',
        'type':   'user',
        'status': 'unknown',
      }
    }
    self.m_lock = threading.Lock()

    if kw.has_key('debug'):
      self.m_debug = kw['debug']
      del kw['debug']

    # variables only used for debugging
    if self.m_debug:
      pass

    return kw

  #
  ## \brief Create gui widgets with supporting data and show.
  #
  def createWidgets(self):
    self.createHeading()
    self.createLeftButtons()
    self.createCenterPanel()
    self.createRightButtons()
    self.update_idletasks()
    self.createStatusBar()

  #
  ## \brief Create top gui heading.
  #
  def createHeading(self):
    # rn logo
    w = Label(self)
    self.m_icons['rn_logo'] = self.m_imageLoader.loadImage("RNLogo48.png");
    if self.m_icons['rn_logo']:
      w['image'] = self.m_icons['rn_logo']
    else:
      w['text'] = 'rn'
      w['anchor'] = W
      w['width'] = 5
    w.grid(row=0, column=0, sticky=W)
    
    # top heading
    w = Label(self)
    w['font']   = ('Helvetica', 16)
    w['text']   = 'Eudoxus Console'
    w['anchor'] = CENTER
    w.grid(row=0, column=1, sticky=E+W)

    # eudoxus logo
    w = Label(self)
    self.m_icons['eudoxus_logo'] = \
        self.m_imageLoader.loadImage("icon_eudoxus_logo.png");
    if self.m_icons['eudoxus_logo']:
      w['image'] = self.m_icons['eudoxus_logo']
      w['anchor'] = E
    else:
      w['text'] = 'eudoxus'
      w['anchor'] = E
      w['width'] = 5
    w.grid(row=0, column=2, sticky=E)
 
  #
  ## \brief Create gui left hand side buttons.
  #
  def createLeftButtons(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=1, column=0, padx=1, pady=3, sticky=N+W+E)

    row = 0

    # start 
    w = self.createButton(wframe, "Start", "icon_play.png",
                            self.cbStartServices)
    w['state'] = 'disabled'
    w.grid(row=row, column=0, sticky=N+E+W)

    row += 1

    # stop 
    w = self.createButton(wframe, "Stop", "icon_stop.png",
                            self.cbStopServices)
    w['state'] = 'disabled'
    w.grid(row=row, column=0, sticky=N+E+W)

    row += 1

    # restart 
    w = self.createButton(wframe, "Restart", "icon_resume.png",
                            self.cbRestartServices)
    w['state'] = 'disabled'
    w.grid(row=row, column=0, sticky=N+E+W)

    row += 1

  #
  ## \brief Create gui center panel.
  #
  def createCenterPanel(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=1, column=1, padx=1, pady=3, sticky=N+W+E)

    row = 0
    col = 0

    for text in ['Sel', 'Status', 'Service', 'Type', 'Description']:

      w = Label(wframe, text=text, foreground=fgColors['focus'])
      w['font'] = ('Helvetica', 10, "bold")
      w.grid(row=row, column=col, padx=3, pady=3, stick=W)

      col += 1

    row     += 1
    colspan = len(self.m_svcKeys)
    bg0     = '#ffffcc'
    bg1     = wframe['bg']
    bg      = bg0

    self.m_service = { }

    for key in self.m_svcKeys:
      col = 0
      service = { }

      service['sel'] = IntVar(0)

      w = Checkbutton(wframe, bg=bg, variable=service['sel'],
          command=self.cbSelect)
      w.grid(row=row, column=col, padx=1, pady=3, sticky=W)

      col += 1

      service['status'] = StringVar()
      service['status'].set("unknown")

      w = Label(wframe, bg=bg, textvariable=service['status'])
      w['fg']     = statusText[service['status'].get()]
      w['width']  = 8
      w.grid(row=row, column=col, padx=1, pady=3, stick=W)
      service['wstatus'] = w

      col += 1

      w = Label(wframe, bg=bg, anchor=W, justify=LEFT, text=key)
      w.grid(row=row, column=col, padx=1, pady=3, stick=W+E)

      col += 1

      w = Label(wframe, bg=bg, anchor=W, justify=LEFT,
          text=self.m_svc[key]['type'])
      w.grid(row=row, column=col, padx=1, pady=3, stick=W+E)

      col += 1

      w = Label(wframe, bg=bg, anchor=W, justify=LEFT,
          text=self.m_svc[key]['desc'])
      w.grid(row=row, column=col, padx=1, pady=3, stick=W+E)

      col += 1

      self.m_service[key] = service

      if bg == bg0:
        bg = bg1
      else:
        bg = bg0

      row += 1

  #
  ## \brief Create gui right hand side buttons.
  #
  def createRightButtons(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=1, column=2, padx=1, pady=3, sticky=N+W+E)

    row = 0

    # refresh 
    w = self.createButton(wframe, "Refresh", "icon_refresh.png",
                            self.cbRefreshStatus)
    w.grid(row=row, column=0, sticky=N+E+W)

    row += 1
    # save 
    w = self.createButton(wframe, "Save", "icon_floppy.png",
                            self.cbSave)
    w['state'] = 'disabled'
    w.grid(row=row, column=0, sticky=N+E+W)

    row += 1

    # save 
    w = self.createButton(wframe, "Quit", "icon_exit.png",
                            self.destroy)
    w.grid(row=row, column=0, sticky=N+E+W)

  #
  ## \brief Create gui multi-line status bar at bottom of gui window.
  #
  def createStatusBar(self):
    wframe = Frame(self)
    wframe['borderwidth'] = 2
    wframe['relief'] = 'ridge'
    wframe.grid(row=2, column=0, columnspan=3, padx=1, pady=3, sticky=N+E+W+S)

    w = Scrollbar(wframe)
    w.grid(row=0, column=1, sticky=N+S)
    self.m_wScrollBar = w;

    self.m_wStatusBar = Text(wframe)
    self.m_wStatusBar['width']    = 105
    self.m_wStatusBar['height']   = 10
    self.m_wStatusBar['wrap']     = WORD
    self.m_wStatusBar['relief']   = 'flat'
    self.m_wStatusBar['fg']       = fgColors['normal']
    self.m_wStatusBar['state']    = 'disabled'
    self.m_wStatusBar.grid(row=0, column=0, padx=3, pady=3, sticky=N+E+W+S)

    # attach
    self.m_wStatusBar['yscrollcommand'] = self.m_wScrollBar.set
    self.m_wScrollBar['command'] = self.m_wStatusBar.yview

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
    self.m_icons[key] = self.m_imageLoader.loadImage(imagefile)
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
  ## \brief Clear all select checkboxes.
  #
  def clearSelect(self):
    for key in self.m_svcKeys:
      self.m_service[key]['sel'].set(0)
    self.cbSelect()

  #
  ## \brief Checkbox change state callback.
  #
  def cbSelect(self):
    bttns = ['start', 'stop', 'restart']
    nselected = 0
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        nselected += 1
    if nselected > 0:
      state = 'normal'
    else:
      state = 'disabled'
    for key in bttns:
      self.m_wBttn[key]['state'] = state

  #
  ## \brief Start selected services callback.
  #
  def cbStartServices(self):
    self.showSbInfo("Starting selected services...\n")
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Starting {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf = self.execStart(key)
        self.showSbResult(text, pf)
    #self.clearSelect()
    self.refresh()

  #
  ## \brief Stop selected services callback.
  #
  def cbStopServices(self):
    self.showSbInfo("Stopping selected services...\n")
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Stopping {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf = self.execStop(key)
        self.showSbResult(text, pf)
    #self.clearSelect()
    self.refresh()

  #
  ## \brief Restart selected services callback.
  #
  def cbRestartServices(self):
    self.showSbInfo("Restarting selected services...\n")
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Restarting {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf = self.execRestart(key)
        self.showSbResult(text, pf)
    #self.clearSelect()
    self.refresh()

  #
  ## \brief Refresh services status callback.
  #
  def cbRefreshStatus(self):
    self.showSbInfo("Refreshing status of all services...\n")
    for key in self.m_svcKeys:
      text = "  Checking {0} status".format(key)
      self.showSbInfo(text)
      self.update_idletasks()
      status = self.execStatus(key)
      self.showSbStatus(text, status)
      self.setStatus(key, status)

  #
  ## \brief Save new settings callback.
  #
  def cbSave(self):
    self.showSbInfo("Saving new settings...\n")

  #
  ## \brief Destroy window callback.
  #
  def destroy(self):
    self.quit()

  #
  ## \brief Show information text on status bar.
  ##
  ## \param text  Info text string.
  #
  def showSbInfo(self, text):
    self.m_wStatusBar["state"] = "normal"
    idx0 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.insert(END, text)
    idx1 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.tag_add("norm", idx0, idx1)
    self.m_wStatusBar.tag_config("norm", foreground=fgColors['normal'])
    self.m_wStatusBar.see(END)
    self.m_wStatusBar["state"] = "disabled"

  #
  ## \brief Show error text on status bar.
  ##
  ## \param text  Error text string.
  #
  def showSbError(self, text):
    self.m_wStatusBar["state"] = "normal"
    idx0 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.insert(END, text)
    idx1 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.tag_add("err", idx0, idx1)
    self.m_wStatusBar.tag_config("err", foreground=fgColors['error'])
    self.m_wStatusBar.see(END)
    self.m_wStatusBar["state"] = "disabled"

  #
  ## \brief Show ok text on status bar.
  ##
  ## \param text  Ok text string.
  #
  def showSbOk(self, text):
    self.m_wStatusBar["state"] = "normal"
    idx0 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.insert(END, text)
    idx1 = self.m_wStatusBar.index(INSERT)
    self.m_wStatusBar.tag_add("ok", idx0, idx1)
    self.m_wStatusBar.tag_config("ok", foreground=fgColors['ok'])
    self.m_wStatusBar.see(END)
    self.m_wStatusBar["state"] = "disabled"

  #
  ## \brief Show operation result on status bar.
  ##
  ## \param text    Prefix text already displayed on current status bar line.
  ## \param success Operation was [not] a success.
  #
  def showSbResult(self, text, success):
    n = self.m_wStatusBar['width'] - len(text)
    if success:
      rc = '[ok]'
      self.showSbOk("%*s\n" % (n, rc))
    else:
      rc = '[failed]'
      self.showSbError("%*s\n" % (n, rc))

  #
  ## \brief Show service status result on status bar.
  ##
  ## \param text    Prefix text already displayed on current status bar line.
  ## \param status  Service status. One of: 'running' 'stopped' 'unknown'
  #
  def showSbStatus(self, text, status):
    n = self.m_wStatusBar['width'] - len(text)
    if status == 'running':
      rc = '[running]'
      self.showSbOk("%*s\n" % (n, rc))
    elif status == 'stopped':
      rc = '[stopped]'
      self.showSbError("%*s\n" % (n, rc))
    else:
      rc = '[unknown]'
      self.showSbInfo("%*s\n" % (n, rc))

  #
  ## \brief Set service status field.
  ##
  ## \param service Service (key).
  ## \param status  Service status. One of: 'running' 'stopped' 'unknown'
  #
  def setStatus(self, service, status):
    self.m_service[service]['status'].set(status)
    self.m_service[service]['wstatus']['fg'] = statusText[status]

  def autoRefresh(self):
    self.refresh()
    self.after(50000, self.autoRefresh)

  #
  ## \brief Refresh status of all services.
  #
  def refresh(self):
    for key in self.m_svcKeys:
      self.execStatus(key)
      self.setStatus(key, self.m_svc[key]['status'])

  #
  ## \brief Execute 'service <service> start' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns True on success, False on failure.
  #
  def execStart(self, service):
    s = ''
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "start"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError, inst:
      self.m_lock.release()
      return False
    self.m_lock.release()
    if reFail.search(s):
      return False
    else:
      return True

  #
  ## \brief Execute 'service <service> stop' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns True on success, False on failure.
  #
  def execStop(self, service):
    s = ''
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "stop"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError, inst:
      self.m_lock.release()
      return False
    self.m_lock.release()
    if reFail.search(s):
      return False
    else:
      return True

  #
  ## \brief Execute 'service <service> restart' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns True on success, False on failure.
  #
  def execRestart(self, service):
    s = ''
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "restart"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError, inst:
      self.m_lock.release()
      return False
    self.m_lock.release()
    if reDoneDone.search(s):
      return True
    elif reFailDone.search(s):
      return True
    else:
      return False

  #
  ## \brief Execute determination of service status.
  ##
  ## \param service Service (key).
  ##
  ## \return Service status. One of: 'running' 'stopped' 'unknown'
  #
  def execStatus(self, service):
    if self.m_svc[service]['type'] == 'init.d':
      return self.execStatusInitd(service)
    else:
      return self.execStatusUser(service)

  #
  ## \brief Execute 'service <service> status' subprocess.
  ##
  ## \param service Service (key).
  ##
  ## \return Service status. One of: 'running' 'stopped' 'unknown'
  #
  def execStatusInitd(self, service):  
    s = ''
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "status"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError, inst:
      s = inst.output
    self.m_lock.release()
    if reRunning.search(s):
      self.m_svc[service]['status'] = 'running'
    elif reNotRunning.search(s):
      self.m_svc[service]['status'] = 'stopped'
    else:
      self.m_svc[service]['status'] = 'unknown'
    return self.m_svc[service]['status']

  #
  ## \brief Execute user service status.
  ##
  ## \param service Service (key).
  ##
  ## \return Service status. One of: 'running' 'stopped' 'unknown'
  #
  def execStatusUser(self, service):  
    hasLock = self.m_lock.acquire()
    self.m_svc[service]['status'] = 'stopped'
    for p in psutil.process_iter():
      try:
        argv = p.cmdline
        #print argv
      except psutil.NoSuchProcess:
        pass
      else:
        if service == 'openni2_launch':
          if self.findProcess(argv, [rePython, reRosLaunch, reRosOpenni2]):
            self.m_svc[service]['status'] = 'running'
        elif service == 'image_view':
          if self.findProcess(argv, [reRosImageView, reRosImageViewImage]):
            self.m_svc[service]['status'] = 'running'
        else:
          self.m_svc[service]['status'] = 'unknown'
    self.m_lock.release()
    return self.m_svc[service]['status']

  # /usr/bin/python /opt/ros/indigo/bin/roslaunch openni2_launch openni2.launch

  def findProcess(self, argv, reList):
    i = 0
    b = False
    for re in reList:
      b = False
      for i in range(i, len(argv)):
        if re.search(argv[i]):
          b = True;
          i += 1
          break
      if not b:
        break
    return b


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
## \brief Eudoxus control panel.
##
class application():

  #
  ## \brief Constructor.
  #
  def __init__(self):
    self._Argv0 = __file__
    self.m_win = None

  #
  ## \brief Print usage error.
  ##
  ## \param emsg  Error message string.
  #
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
usage: %s [OPTIONS]
       %s --help

Options and arguments:

-h, --help                : Display this help and exit.
"""  % (self._Argv0, self._Argv0)
 
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

    self._Argv0 = kwargs.get('argv0', __file__)

    # defaults
    kwargs['debug'] = False # Set to False when finished debugging.

    # parse command-line options
    try:
      opts, args = getopt.getopt(argv[1:], "?h",
          ['help', ''])
    except getopt.error, msg:
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
    except usage, e:
      print e.msg
      return 2

    # create root 
    root = Tk()

    # create application window
    self.m_win = window(master=root, **kwargs)

    root.protocol('WM_DELETE_WINDOW', root.destroy)

    # loop
    self.m_win.mainloop()

    return 0


# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
