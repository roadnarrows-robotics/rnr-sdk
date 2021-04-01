#! /usr/bin/env python3

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
## \copyright
##   \h_copy 2016-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##
# @EulaBegin@
# @EulaEnd@
#
###############################################################################

import sys
import os
import time
import datetime
import math
import shlex
import subprocess
import psutil
import re
import threading
import getopt

from tkinter import *
from tkinter.constants import *
from tkinter.filedialog import *
import tkinter.font

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

# /usr/bin/python /opt/ros/indigo/bin/roslaunch openni2_launch openni2.launch
rePython            = re.compile(r".*python", re.IGNORECASE)
reRosLaunch         = re.compile(r".*roslaunch", re.IGNORECASE)
reRosOpenni2Pkg     = re.compile(r".*openni2_launch", re.IGNORECASE)
reRosOpenni2Launch  = re.compile(r".*openni2\.launch", re.IGNORECASE)

# /opt/ros/indigo/lib/image_view/image_view image:=/camera/depth/image
reRosImageView      = re.compile(r".*image_view", re.IGNORECASE)
reRosImageViewImage = re.compile(r"image:=/camera/depth/image", re.IGNORECASE)


# make absolute path from list of path components
mkpath = lambda *p: os.path.normpath(os.path.join(os.path.sep, *p))

#
# The psutil package changed interfaces sometime between v1.x and v3.x.
#
if callable(psutil.Popen.cmdline): # new psutil versions
  procCmdLine = lambda p: p.cmdline()
else:                 # old psutil versions
  procCmdLine = lambda p: p.cmdline


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
        'desc':     'ROS Master, Parameter Server, rosout logging node.',
        'type':     'init.d',
        'cmd':      'service eudoxus_roscore {0}',
        'status':   'unknown',
        'relist':   [],
        'subproc':  None,
      },
      'eudoxus_shutter': {
        'desc':     'Eudoxus user button monitor.',
        'type':     'init.d',
        'cmd':      'service eudoxus_shutter {0}',
        'status':   'unknown',
        'relist':   [],
        'subproc':  None,
      },
      'openni2_launch': {
        'desc':     'ROS OpenNI2 camera drivers and RGBD launch files.',
        'type':     'user',
        'cmd':      'roslaunch openni2_launch openni2.launch',
        'status':   'unknown',
        'relist':   [reRosLaunch, reRosOpenni2Pkg, reRosOpenni2Launch],
        'subproc':  None,
      },
      'image_view': {
        'desc':     'ROS depth disparity viewer.',
        'type':     'user',
        'cmd':      'rosrun image_view image_view image:=/camera/depth/image',
        'status':   'unknown',
        'relist':   [reRosImageView, reRosImageViewImage],
        'subproc':  None,
      }
    }
    
    # environment
    self.setenv()
    self.printenv()

    self.m_lock = threading.Lock()

    if 'debug' in kw:
      self.m_debug = kw['debug']
      del kw['debug']

    # variables only used for debugging
    if self.m_debug:
      pass

    return kw

  #
  ## \brief Set up execution environment.
  #
  def setenv(self):
    pathsep = os.path.pathsep   # ':'
    sep     = os.path.sep       # '/'

    self.m_env = dict(os.environ)

    rosdistro = self.m_env.get('ROS_DISTRO', 'indigo')
    rosroot   = self.m_env.get('ROS_ROOT',
                              mkpath('opt', 'ros', rosdistro, 'share', 'ros'))
    rosprefix = rosroot[0:rosroot.find(rosdistro)]
    rospkg    = self.m_env.get('ROS_PACKAGE_PATH', '')
    rnrprefix = mkpath('opt', 'rnr_ros') # default
    for path in rospkg.split(pathsep):
      s = path[0:path.find(rosdistro)]
      if s != rosprefix:
        rnrprefix = s
        break

    ros = mkpath(rosprefix, rosdistro)
    rnr = mkpath(rnrprefix, rosdistro)

    # fixup PATH - not inherited user environment
    path = self.m_env.get('PATH', '')
    for p in [
        mkpath(ros, 'bin'),
        mkpath('usr', 'local', 'bin'),
        mkpath('prj', 'bin')]:
      path = p + pathsep + path
    self.m_env['PATH'] = path

    # fixup PYTHONPATH - not inherited user environment
    python = 'python2.7'
    path = self.m_env.get('PYTHONPATH', '')
    for p in [
        mkpath('usr', 'local', 'lib', python),
        mkpath('prj', 'lib', python, 'site-packages'),
        mkpath(ros, 'lib', python, 'dist-packages'),
        mkpath(rnr, 'devel', 'lib', python, 'dist-packages')]:
      path = p + os.path.pathsep + path
    self.m_env['PYTHONPATH'] = path

    # fixup LD_LIBRARY_PATH - not inherited user environment
    path = self.m_env.get('LD_LIBRARY_PATH', '')
    for p in [
        mkpath(ros, 'lib'),
        mkpath(rnr, 'devel', 'lib')]:
      path = p + os.path.pathsep + path
    self.m_env['LD_LIBRARY_PATH'] = path

  #
  ## \brief (Debug) Print relevant environment values.
  #
  def printenv(self):
    for var in ['LD_LIBRARY_PATH', 'PATH', 'PYTHONPATH',
                'ROSLISP_PACKAGE_DIRECTORIES', 'ROS_DISTRO', 'ROS_ETC_DIR',
                'ROS_MASTER_URI', 'ROS_PACKAGE_PATH', 'ROS_ROOT']:
      val = self.m_env.get(var, '')
      print("{0}={1}".format(var, val))

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
    now = datetime.datetime.today().strftime("%Y.%m.%d %H:%M:%S")
    self.showSbInfo("[{0}] Starting selected services...\n".format(now))
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Starting {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf,emsg = self.execStart(key)
        self.showSbResult(text, pf, emsg)
        self.showSbInfo('    ('+self.m_svc[key]['cmd'].format('start')+')\n')
    #self.clearSelect()
    self.refresh()

  #
  ## \brief Stop selected services callback.
  #
  def cbStopServices(self):
    now = datetime.datetime.today().strftime("%Y.%m.%d %H:%M:%S")
    self.showSbInfo("[{0}] Stopping selected services...\n".format(now))
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Stopping {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf,emsg = self.execStop(key)
        self.showSbResult(text, pf, emsg)
        self.showSbInfo('    ('+self.m_svc[key]['cmd'].format('stop')+')\n')
    #self.clearSelect()
    self.refresh()

  #
  ## \brief Restart selected services callback.
  #
  def cbRestartServices(self):
    now = datetime.datetime.today().strftime("%Y.%m.%d %H:%M:%S")
    self.showSbInfo("[{0}] Restarting selected services...\n".format(now))
    for key in self.m_svcKeys:
      if self.m_service[key]['sel'].get():
        text = "  Restarting {0} service".format(key)
        self.showSbInfo(text)
        self.update_idletasks()
        pf,emsg = self.execRestart(key)
        self.showSbResult(text, pf, emsg)
        self.showSbInfo('    ('+self.m_svc[key]['cmd'].format('restart')+')\n')
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
  def showSbResult(self, text, success, emsg=None):
    n = self.m_wStatusBar['width'] - len(text)
    if success:
      result = '[ok]'
      self.showSbOk("%*s\n" % (n, result))
    else:
      if emsg is not None and len(emsg) > 0:
        result = '[' + emsg + ']'
      else:
        result = '[failed]'
      self.showSbError("%*s\n" % (n, result))

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
    #RDK self.after(1000, self.autoRefresh)
    self.after(1000, self.autoRefresh)

  #
  ## \brief Refresh status of all services.
  #
  def refresh(self):
    for key in self.m_svcKeys:
      self.execStatus(key)
      self.setStatus(key, self.m_svc[key]['status'])

  #
  ## \brief Start service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStart(self, service):
    if self.m_svc[service]['type'] == 'init.d':
      return self.execStartInitd(service)
    else:
      return self.execStartUser(service)

  #
  ## \brief Execute init.d 'service <service> start' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStartInitd(self, service):
    pf    = False
    emsg  = None
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "start"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as inst:
      emsg = inst.message
    else:
      if not reFail.search(s):
        pf = True
    self.m_lock.release()
    return pf,emsg

  #
  ## \brief Execute user service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStartUser(self, service):
    pf    = False
    emsg  = None
    hasLock = self.m_lock.acquire()
    if self.findProcess(service, self.m_svc[service]['relist']) is None:
      args = shlex.split(self.m_svc[service]['cmd'])
      try:
        self.m_svc[service]['subproc'] = subprocess.Popen(args, env=self.m_env)
        pf = True
      except OSError as inst:
        self.m_svc[service]['subproc'] = None
        emsg = "{0}: {1}(errno={2}".format(args[0], inst.strerror, inst.errno)
    else:
      emsg = 'already running'
    self.m_lock.release()
    return pf,emsg

  #
  ## \brief Stop service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStop(self, service):
    if self.m_svc[service]['type'] == 'init.d':
      return self.execStopInitd(service)
    else:
      return self.execStopUser(service)

  #
  ## \brief Execute init.d 'service <service> stop' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStopInitd(self, service):
    pf    = False
    emsg  = None
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "stop"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as inst:
      emsg = inst.message
    else:
      if not reFail.search(s):
        pf = True
    self.m_lock.release()
    return pf,emsg

  #
  ## \brief Kill user service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execStopUser(self, service):
    pf    = False
    emsg  = None
    hasLock = self.m_lock.acquire()
    p = self.findProcess(service, self.m_svc[service]['relist'])
    if p is not None:
      try:
        self.kill(p)
        self.m_svc[service]['subproc'] = None
        pf = True
      except OSError as inst:
        emsg = "{0}: {1}(errno={2}".format(args[0], inst.strerror, inst.errno)
    else:
      emsg = 'not running'
    self.m_lock.release()
    return pf,emsg

  #
  ## \brief Kill process and all of its children (bwhaaaa).
  ##
  ## \param process psutil process.
  #
  def kill(self, process):
    for proc in process.get_children(recursive=True):
      proc.kill()
    process.kill()

  #
  ## \brief Restart service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execRestart(self, service):
    if self.m_svc[service]['type'] == 'init.d':
      return self.execRestartInitd(service)
    else:
      return self.execRestartUser(service)

  #
  ## \brief Execute init.d 'service <service> restart' subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execRestartInitd(self, service):
    pf    = False
    emsg  = None
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "restart"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as inst:
      emsg = inst.message
    else:
      if reDoneDone.search(s):
        pf = True
      elif reFailDone.search(s):
        pf = True
    self.m_lock.release()
    return pf,emsg

  #
  ## \brief Kill and re-execute user service subprocess.
  ##
  ## \param service Service (key).
  ## 
  ## \return Returns (pf, emsg) where pf is True on success, False on failure,
  ## emsg is either an error message or None.
  #
  def execRestartUser(self, service):
    self.execStopUser(service)
    time.sleep(5)
    return self.execStartUser(service)

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
    hasLock = self.m_lock.acquire()
    try:
      s = subprocess.check_output(["service", service, "status"],
                                  stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as inst:
      s = inst.output
    else:
      if reRunning.search(s):
        self.m_svc[service]['status'] = 'running'
      elif reNotRunning.search(s):
        self.m_svc[service]['status'] = 'stopped'
      else:
        self.m_svc[service]['status'] = 'unknown'
    self.m_lock.release()
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
    if self.findProcess(service, self.m_svc[service]['relist']) is not None:
      self.m_svc[service]['status'] = 'running'
    else:
      self.m_svc[service]['status'] = 'stopped'
    self.m_lock.release()
    return self.m_svc[service]['status']

  #
  ## \brief Find running process with command line that matches list of
  ## regular expressions.
  ##
  ## \param service   Service (key).
  ## \param reList    List of regular expressions.
  ##
  ## \return Returns Popen object or None.
  #
  def findProcess(self, service, reList):
    for p in psutil.process_iter():
      try:
        if self.matchProcess(service, procCmdLine(p), reList):
          return p
      except psutil.NoSuchProcess:
        pass
    return None

  #
  ## \brief Match command line arguments against list of regular expressions.
  ##
  ## All regular expressions must match.
  ##
  ## \param service   Service (key).
  ## \param cmdline   List of command line arguments.
  ## \param reList    List of regular expressions.
  ##
  ## \return Returns True if matched, False otherwise.
  #
  def matchProcess(self, service, cmdline, reList):
    i = 0
    b = False
    for re in reList:
      b = False
      for j in range(i, len(cmdline)):
        if re.search(cmdline[j]):
          b = True;
          break
      if not b:
        break
      i = j + 1
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
      print("%s: %s" % (self._Argv0, emsg))
    else:
      print("%s: error" % (self._Argv0))
    print("Try '%s --help' for more information." % (self._Argv0))

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print(
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

    self._Argv0 = kwargs.get('argv0', __file__)

    # defaults
    kwargs['debug'] = False # Set to False when finished debugging.

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
