#! /usr/bin/env python
################################################################################
#
# TShell.py
#

""" User Template for a command-line shell to the robot.

Instructions:
  Copy this file to your development area, renaming it appropriately.
  Search for the string 'USERACTION' for things you should do and
  USER<x> for things to change.

This shell template is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.01

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

import sys
import os
import time
import re
import signal

try:
  import readline   # optional module
except ImportError:
  pass

import Fusion.Utils.Tools as utils

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiWinShell as gterm


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# USERACTION: add any specific global data here.

ReStart     = '^\s*(\S+)'         # command's start regular expression
ReEnd       = '\s*(?:#.*|$)'      # command's end regular expression
ReArgStr    = '\s+(\S+)'          # string argument
ReArgInt    = '\s+([-+]?\d+)'     # integer argument
ReArgPosInt = '\s+(\d+)'          # strictly positive integer argument
ReArgFloat  = '\s+([-+]?\d+\.?\d*|[-+]?\d*\.?\d+)'  # float argument
ReArgOnOff  = '\s+(on|off)'       # on or off argument
ReArgIncRaw = '\s+(incraw)'       # include raw data

MaxScriptDepth  = 5               # maximum script file depth


# USERACTION: change USERSHELL to your class name.
#-------------------------------------------------------------------------------
# CLASS: USERSHELL
#-------------------------------------------------------------------------------
class USERSHELL:
  """ User Shell Class. """
  """ User Template for Robot Command-Line Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Raw Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of key=value arguments
            argv0       - shell command file name. Default: this file name
            script      - command option script file name. Default: None
            port        - command option serial port device name.
                          Default: None
            baudrate    - command option serial port baudrate.
                          Default: 9600
            shVersion   - shell version string. Default: '1.0'
            shName      - shell name. Default: 'USERSHELLNAME'
            shPS1       - primary prompt string. Default: 'USERPS1$ '
            shPS2       - secondary prompt string. Default: 'USERPS2> '
            helpExample - help example string. Default: ''
            robotCmds   - robot commands object. Default: DummyCmds()
            fin         - user input stream. Default: stdin
            fout        - user output stream. Default: stdout
            closeonexit - do [not] close serial connection on exit.
                          Default: True
 
    """
    # initialize defaults
    self.InitDefaults()

    # set arguments
    self.SetArgs(**kwargs)

    # initialize command dictionary
    self.InitCmdDict()

    # core data initialization
    self.mPortIsOpen    = False             # track port state 
    self.mScriptFile    = None              # command script file
    self.mDoQuit        = False             # do [not] quit this shell
    self.mIsInteractive = True              # shell is user interactive
    self.mScriptDepth   = 0                 # current script file depth

    # set gui colors
    self.SetStyles()

  #--
  def InitDefaults(self):
    """ Initialize defaults. """
    self.mArgv0       = os.path.basename(__file__)
    self.mOptScript   = None        # script command-line option
    self.mOptPort     = None        # USERACTION: comm specific option
    self.mOptBaudRate = 9600        # USERACTION: comm specific option
    self.mShVersion   = '1.0'       # USERACTION: update as needed
    self.mShName      = 'USERSHELLNAME'
    self.mShPS1       = 'USERPS1$ ' # USERACTION: primary prompt string
    self.mShPS2       = 'USERPS2> ' # USERACTION: secondary prompt string
    self.mHelpExample = ''
    self.mRobotCmds   = None        # USERACTION: change to your cmd object
    self.mFin         = sys.stdin
    self.mFout        = sys.stdout
    self.mCloseOnExit = True

    # USERACTION: add other data here

  #--
  def SetArgs(self, **kwargs):
    """ Process argument list.

        Parameters:
          kwargs  - dictionary of key=value arguments
    """
    for key, value in kwargs.iteritems():
      if key == 'argv0':
        self.mArgv0 = os.path.basename(value)
      elif key == 'shVersion':
        self.mShVersion = value
      elif key == 'shName':
        self.mShName = value
      elif key == 'shPS1':
        self.mShPS1 = value
      elif key == 'shPS2':
        self.mShPS2 = value
      elif key == 'helpExample':
        self.mHelpExample = value
      elif key == 'robotCmds':
        self.mRobotCmds = value
      elif key == 'script':
        self.mOptScript = value
      elif key == 'port':
        self.mOptPort = value
      elif key == 'baudrate':
        self.mOptBaudRate = value
      elif key == 'fin':
        self.mFin = value
      elif key == 'fout':
        self.mFout = value
      elif key == 'closeonexit':
        self.mCloseOnExit = value
    # USERACTION: add other kwargs here

  #--
  def SetStyles(self):
    """ Set text styles if shell attached to a GUI. """
    gterm.SendAddStyle('dft', fg=gt.ColorFgShellDft)
    gterm.SendAddStyle('err', fg=gt.ColorFgShellErr)
    gterm.SendAddStyle('pun', fg=gt.ColorFgShellPun)
    gterm.SendAddStyle('key', fg=gt.ColorFgShellKey)
    gterm.SendAddStyle('num', fg=gt.ColorFgShellNum)
    gterm.SendAddStyle('str', fg=gt.ColorFgShellStr)
    gterm.SendAddStyle('boo', fg=gt.ColorFgShellBool)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # The Command Dictionary (append to this dictionary)
  #   Per each command:
  #     'help'    help string list
  #     'refmt'   regular expression [list] used to parse user input line
  #     'cvt'     argument conversion function list
  #     'prereq'  command prerequisite function list
  #     'exec'    command execution function
  #     'rsp'     response function (optional)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  # USERACTION: Add all shell commands to this dictionary.
  def InitCmdDict(self):
    """ Initialize Command Dictionary. """
    self.mCmdDict = {
      'help': {
        'help':   ['', 'Print this list of commands.'],
        'refmt':  ReStart + ReEnd,
        'cvt':    [],
        'prereq': [],
        'exec':   self.CmdHelp
      },
      # USERACTION: open establish connection with the robot. modify for
      # USERACTION: specific robot. This example is for a serial connection.
      'open': {
        'help':   ['<port> [<baudrate>]',
                   "Open serial connection to the robot.",
                   'parameters:',
                   '  <port>     - port name (device)',
                   '  <baudrate> - supported baudrate (optional)',
                   '               (default: 9600)'],
        'refmt':  [ReStart + ReArgStr + ReArgPosInt + ReEnd,
                   ReStart + ReArgStr + ReEnd],
        'cvt':    [self.CvtNull, self.CvtInt],
        'prereq': [],
        'exec':   self.CmdOpen
      },
      'close': {
        'help':   ['', "Close connection to the robot."],
        'refmt':  ReStart + ReEnd,
        'cvt':    [],
        'prereq': [],
        'exec':   self.CmdClose
      },
      'rawcmd': {
        'help':   ['<cmd>',
                   "Send raw command to robot.",
                   'parameters:',
                   '  <cmd> - raw command'],
        'refmt':  ReStart + ReArgStr + ReEnd,
        'cvt':    [self.CvtNull],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdRawCmd
      },
      'flush': {
        'help':   ['', "Flush input."],
        'refmt':  ReStart + ReEnd,
        'cvt':    [],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdFlush
      },
      'err': {
        'help':   ['', 
                   "Print last robot communication error."],
        'refmt':  ReStart + ReEnd,
        'cvt':    [],
        'prereq': [],
        'exec':   self.CmdErr
      },
      'debug': {
        'help':   ['<level>',
                   "Turn connection debugging on/off.",
                   'parameters:',
                   '  <level>  - debugging level 0=off, 1-5'],
        'refmt':  ReStart + ReArgPosInt + ReEnd,
        'cvt':    [self.CvtInt],
        'prereq': [],
        'exec':   self.CmdDebug
      },
      'script': {
        'help':   ['<file>',
                   "Run script file through this shell.",
                   'parameters:',
                   '  <file>  - script file file'],
        'refmt':  ReStart + ReArgStr + ReEnd,
        'cvt':    [self.CvtNull],
        'prereq': [],
        'exec':   self.CmdScript
      },
      'quit': {
        'help':   ['', "Quit this shell."],
        'refmt':  ReStart + ReEnd,
        'cvt':    [],
        'prereq': [],
        'exec':   self.CmdQuit
      },
      'wait': {
        'help':   ['<seconds>',
                   "Wait until the given seconds have elapsed.",
                   'parameters:',
                   '  <seconds> - seconds > 0.0'],
        'refmt':  ReStart + ReArgFloat + ReEnd,
        'cvt':    [self.CvtFloat],
        'prereq': [],
        'exec':   self.CmdWait
      }
    }
  
  def AddToCmdDict(self, newdict):
    """ Add a new command dictionary to the existing dictionary.

        Parameters:
          newdict   - new dictionary

        Return Value:
          None.
    """
    for k,v in newdict.iteritems():
      if self.mCmdDict.has_key(k):
        self.PError("BUG: Adding duplicate key to CmdDict: key = %s" % repr(k))
        return
      else:
        self.mCmdDict[k] = v


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Raw Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CmdOpen(self, port, baudrate=9600):
    """ Open Serial Port Shell Command. """
    try:
      # USERACTION: establish communication with robot, change as needed
      self.mRobotCmds.Open(port, baudrate=baudrate)
    except IOError:
      return 'err', self.mRobotCmds.GetErrStr() # USERACTION: modify as needed
    self.mPortIsOpen = True
    return 'ok', "port %s opened at %d baud" % (port, baudrate)
  
  def CmdClose(self):
    """ Close Serial Port Shell Command. """
    # USERACTION: close communication with robot, change as needed
    self.mRobotCmds.Close()
    self.mPortIsOpen = False
    return 'ok', 'connection closed'
  
  def CmdRawCmd(self, cmd):
    """ Raw Robot Shell Command. """
    # USERACTION: send raw robot command, change as needed
    return 'ok', repr(self.mRobotCmds.SendCmd(cmd))
  
  def CmdFlush(self):
    """ Flush Serial Input Shell Command. """
    # USERACTION: flush robot input, change/delete as needed
    self.mRobotCmds.FlushInput()
    return 'ok', ''
  
  def CmdErr(self):
    """ Return Last Robot Error Shell Command. """
    # USERACTION: get last robot communication error, change as needed
    return 'ok', self.mRobotCmds.GetErrStr()
  
  def CmdDebug(self, dl):
    """ Enable/Disable Serial Port Debugging Shell Command. """
    # USERACTION: set communication debug level, change as needed
    self.mRobotCmds.SetDebugLevel(dl)
    return 'ok', 'debugging level %d' % dl
  
  def CmdScript(self, scriptFile):
    """ Run Script Shell Command. """
    if self.mScriptDepth > MaxScriptDepth:
      return 'err', 'script depth exceeds max: %d' % MaxScriptDepth
    try:
      self.mScriptFile = open(scriptFile, 'r')
      self.mIsInteractive = False
    except IOError, err:
      return 'err', err
    self.mScriptDepth += 1
    self.RunLoopScript()
    self.mScriptDepth -= 1
    return 'ok', ''
  
  def CmdQuit(self):
    """ Quit Shell Command. """
    # USERACTION: close communication with robot, change as needed
    self.mRobotCmds.Close()
    self.mDoQuit = True
    return 'ok', "good-bye"
  
  def CmdWait(self, seconds):
    """ Wait Shell Shell Command. """
    time.sleep(seconds)
    return 'ok', ''
  
  def CmdHelp(self):
    """ Help Shell Command. """
    banner = "%s v%s Help" % (self.mArgv0, self.mShVersion)
    self.Print(banner)
    self.Print('-' * len(banner))
    self.Print("""
Enter any of the commands listed below to execute the desired action(s).
Partial command matching is done to find the unique command substring. 
""")
    if self.mHelpExample:
      self.Print(self.mHelpExample)
      self.Print('')
  
    cmdList = self.mCmdDict.keys()
    cmdList.sort()
    col2 = 10
    self.Print("Command List:")
    for cmd in cmdList:
      s = ' ' + cmd
      col = len(s)
      self.Write(s)
      if col < col2: sep =  ' ' * (col2 - col)
      else: sep = ' '
      helpstrs = self.mCmdDict[cmd]['help']
      self.Print('%susage: %s %s' % (sep, cmd, helpstrs[0]))
      sep = ' ' * (col2 + 4)
      for str in helpstrs[1:]:
        self.Print(sep + str)
    return 'ok', ''


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Common Parameter Conversions 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CvtNull(self, s):
    """ Convert Argument NoOp. """
    return 'ok', s
  
  def CvtTrue(self, s):
    """ Blindly Convert Argument into True. """
    return 'ok', True
  
  def CvtInt(self, s, base=10):
    """ Convert Argument into Integer. """
    try:
      val = int(s, base)
      return 'ok', val
    except (SyntaxError, NameError, TypeError, ValueError):
      return 'err', "bad integer: %s" % (s)
  
  def CvtOnOff(self, s):
    """ Convert {on | off} into Integer. """
    if s == 'on':
      return 'ok', 1
    elif s == 'off':
      return 'ok', 0
    else:
      return 'err', "not 'on' or 'off'"
  
  def CvtFloat(self, s):
    """ Convert Argument into Float. """
    try:
      val = float(s)
      return 'ok', val
    except (SyntaxError, NameError, TypeError, ValueError):
      return 'err', "bad float: %s" % (s)
  
  def CvtIntList(self, s):
    """ Convert Argument into Integer List. """
    intList = []
    for sArg in s.split():
      if len(sArg) > 2 and (sArg[0:2] == '0x' or sArg[0:2] == '0X'):
        rc, i = self.CvtInt(sArg[2:], 16)
      else:
        rc, i = self.CvtInt(sArg, 10)
      if rc == 'err':
        return err, i
      if i < 0 or i > 0xff:
        return 'err', 'byte out of range: %s' % (sArg)
      intList += [i]
    return 'ok', intList


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Common Shell Prerequisites 
  #   All prerequisite checkers must return either True (ok) or False.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def PreReqOpen(self, cmd):
    """ 'Serial Port Must Be Open' Prerequisite. """
    # USERACTION: test if communication is up, change as needed
    b = self.mRobotCmds.IsOpen()
    if not b:
      if self.mPortIsOpen:
        self.Print('Port closed\n')
      self.RspErr("command requires port to be open: '%s'" % (cmd))
    else:
      if not self.mPortIsOpen:
        # USERACTION: get communication channel name, change as needed
        self.Print("Port '%s' opened.\n" % (self.mRobotCmds.GetPort()))
    self.mPortIsOpen = b
    return self.mPortIsOpen


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Response Post-Processors
  #   All response post-processors must print any formatted data to fout.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def PError(self, *args):
    """ Print error to stderr. """
    for arg in args:
      print(arg, file=sys.stderr, end='')
    print('', file=sys.stderr)

  def Print(self, *args):
    """ Print to output stream. """
    # do not use 'print(..., file=sys.stdout)' calls - locks up threads
    # reading pipes
    for arg in args:
      if type(arg) == str:
        self.mFout.write(arg+' ')
      else:
        self.mFout.write(repr(arg)+' ')
    self.mFout.write('\n')
    self.mFout.flush()

  def Write(self, s):
    """ Write to output stream. """
    self.mFout.write(s)
    self.mFout.flush()
 
  def RspRepr(self, rsp):
    """ Safe Response Formatter. """
    self.Print(repr(rsp))
  
  def RspErr(self, err):
    """ Error Response Formatter. """
    gterm.SendSetStyle('err')
    self.Print("error: %s" % err)
    gterm.SendSetStyle('dft')

  def RspPrettyPrint(self, data):
    """ Pretty Print Response Formatter. """
    if gterm.ThisShellHasAGui():
      self._RspGuiPrettyPrint(0, data)
      gterm.SendSetStyle('dft')
      self.Write('\n')
    else:
      self.Print(data)

  def _RspGuiPrettyPrint(self, indent, data):
    """ GUI Pretty Print Response Formatter Workhorse.  """
    if type(data) == int or type(data) == float:
      gterm.SendSetStyle('num')
      self.Write('%*s%s' % (indent, '', repr(data)))
    elif type(data) == str:
      gterm.SendSetStyle('str')
      self.Write('%*s%s' % (indent, '', data))
    elif type(data) == bool:
      gterm.SendSetStyle('boo')
      self.Write('%*s%s' % (indent, '', repr(data)))
    elif type(data) == tuple:
      gterm.SendSetStyle('pun')
      self.Write('%*s(' % (indent, ''))
      n = 0
      for v in data:
        if n > 0:
          gterm.SendSetStyle('pun')
          self.Write(', ')
        self._RspGuiPrettyPrint(0, v)
        n += 1
      gterm.SendSetStyle('pun')
      self.Write(')')
    elif type(data) == list:
      gterm.SendSetStyle('pun')
      self.Write('%*s[' % (indent, ''))
      n = 0
      for v in data:
        if n > 0:
          gterm.SendSetStyle('pun')
          self.Write(',')
        self.Write('\n')
        self._RspGuiPrettyPrint(indent+2, v)
        n += 1
      gterm.SendSetStyle('pun')
      self.Write('\n%*s]' % (indent, ''))
    elif type(data) == dict:
      gterm.SendSetStyle('pun')
      self.Write('%*s{' % (indent, ''))
      keys = data.keys()
      keys.sort()
      n = 0
      for k in keys:
        if n > 0:
          gterm.SendSetStyle('pun')
          self.Write(',')
        self.Write('\n')
        gterm.SendSetStyle('key')
        if type(k) == str:
          self.Write('%*s%s' %(indent+2, '', k))
        else:
          self.Write('%*s%s' %(indent+2, '', repr(k)))
        gterm.SendSetStyle('pun')
        self.Write(':')
        self._RspGuiPrettyPrint(indent+2, data[k])
        n += 1
      gterm.SendSetStyle('pun')
      self.Write('\n%*s}' % (indent, ''))
    else:
      gterm.SendSetStyle('dft')
      self.Write('%*s%s' % (indent, '', repr(data)))

  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parse Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  def ParseCmd(self, inputLine):
    """ Parse Input Command Line. """
    if not inputLine:
      return None
    inputCmd = self.ParseGetInputCmd(inputLine)
    if not inputCmd:
      return None
    if inputCmd[0] == '#':
      return None
    n = len(inputCmd)
    matches = []
    for cmd in self.mCmdDict.iterkeys():
      if len(cmd) >= n and cmd[:n] == inputCmd:
        matches += [cmd]
    if len(matches) == 0:
      self.RspErr("command not found: '%s'" % (inputCmd))
      return None
    elif len(matches) > 1:
      self.RspErr("ambiguous command: '%s' matches %s" % (inputCmd, matches))
      return None
    return self.ParseCvtInput(matches[0], inputLine)
  
  def ParseGetInputCmd(self, inputLine):
    """ Get the Input Command (first argument). """
    mo = re.search('\s*(\S+)', inputLine)
    if not mo:
      return None
    return mo.group(1) 
  
  def ParseCvtInput(self, cmd, inputLine):
    """ Convert Input Command Arguments into Executable Format. """
    if type(self.mCmdDict[cmd]['refmt']) == list:
      for refmt in self.mCmdDict[cmd]['refmt']:
        inputArgs = re.findall(refmt, inputLine)
        if inputArgs: break
    else:
      inputArgs = re.findall(self.mCmdDict[cmd]['refmt'], inputLine)
    if not inputArgs:
      self.RspErr("invalid syntax: '%s'" % (inputLine))
      return None
    inputArgs = inputArgs[0]   # get first element tuple in list '[(...)]'
    cmdArgs = [cmd]
    if type(inputArgs) == tuple:
      n = 0
      for arg in inputArgs[1:]:
        rc, cvtArg = self.mCmdDict[cmd]['cvt'][n](arg)
        if rc == 'err':
          self.RspErr(cvtArg)
          return None
        else:
          cmdArgs += [cvtArg]
        n += 1
    return cmdArgs
  
  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Execution Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  def ExecCmd(self, cmdArgs):
    """ Execute Parsed Input Command. """
    if not self.ExecChkPreReqs(cmdArgs[0]):
      return
    cmdfn = self.mCmdDict[cmdArgs[0]]['exec']
    nArgs = len(cmdArgs)
    if not self.mIsInteractive: 
      for arg in cmdArgs:
        if type(arg) == str:
          self.Write(arg+' ')
        else:
          self.Write(repr(arg)+' ')
      self.Write('\n')
    if nArgs == 1:
      rc, rval = cmdfn()
    elif nArgs == 2:
      rc, rval = cmdfn(cmdArgs[1])
    elif nArgs == 3:
      rc, rval = cmdfn(cmdArgs[1], cmdArgs[2])
    elif nArgs == 4:
      rc, rval = cmdfn(cmdArgs[1], cmdArgs[2], cmdArgs[3])
    else:
      self.PError("BUG: too many args: '%s'" % (cmdArgs))
      return
    if rc == 'err':
      self.RspErr("%s" % (rval))
    elif rval != '':
      if self.mCmdDict[cmdArgs[0]].has_key('rsp'):
        self.mCmdDict[cmdArgs[0]]['rsp'](rval)
      else:
        self.RspPrettyPrint(rval)
  
  def ExecChkPreReqs(self, cmd):
    """ Check All Prerequisites Prior to Command Execution. """
    for prereq in self.mCmdDict[cmd]['prereq']:
      if not prereq(cmd):
        return False
    return True


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Run Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def Run(self):
    """ Run the shell. """
    ec = self.RunInit()
    if ec == 0:
      ec = self.RunLoop()
    return ec

  #--
  def RunInit(self):
    """ Run Initialization. """
    self.mDoQuit        = False
    self.mIsInteractive = True

    # try setting up a user interrupt handler. this will only work if the
    # shell is running in the main thread. Otherwise, ignore.
    try:
      signal.signal(signal.SIGINT, self.ExitHandler)
    except ValueError:
      pass

    if not self.mRobotCmds:
      # USERACTION: Add default robot command object here
      self.mRobotCmds = DummyCmd()

    if self.mOptPort:
      try:
        # USERACTION: establish communication with robot, change as needed
        self.mRobotCmds.Open(self.mOptPort, baudrate=self.mOptBaudRate)
      except IOError, err:
        PrintUsageErr("%s" % err)
        return 2
    if self.mOptScript:
      try:
        self.mScriptFile = open(self.mOptScript, 'r')
        self.mIsInteractive = False
      except IOError, err:
        PrintUsageErr("%s" % err)
        return 2
    return 0
    
  #--
  def RunLoop(self):
    """ The Run Loop. """
    if self.mIsInteractive:
      self.RunLoopUser()
    else:
      self.RunLoopScript()
    return 0
  
  def RunLoopUser(self):
    """ Run Loop for User Interactive Shell. """
    self.Print("USERSHELL %s Command Shell, Version %s" % \
        (self.mShName, self.mShVersion))
    self.Print(" (type 'help' for list of commands)")
    self.Print("")
    # USERACTION: change as needed
    if self.mRobotCmds.IsOpen():
      # USERACTION: change as needed
      self.Print("Port '%s' open.\n" % (self.mRobotCmds.GetPort()))
      self.mPortIsOpen = True
    while not self.mDoQuit:
      inputLine = self.GetUserInput(self.mShPS1)
      if inputLine:
        cmdArgs = self.ParseCmd(inputLine)
        if cmdArgs:
          self.ExecCmd(cmdArgs)
  
  def RunLoopScript(self):
    """ Run Loop for Command Script. """
    inputLine = self.mScriptFile.readline()
    while not self.mDoQuit and inputLine:
      cmdArgs = self.ParseCmd(inputLine)
      if cmdArgs:
        self.ExecCmd(cmdArgs)
      inputLine = self.mScriptFile.readline()
    self.mScriptFile.close()
    self.CmdClose()
  
  def GetUserInput(self, prompt):
    """ Get User Input. """
    try:
      inputLine = utils.user_input(prompt, fin=self.mFin, fout=self.mFout)
      return inputLine
    except KeyboardInterrupt:
      self.ExitHandler(0, 'Received Keyboard Interurrupt')
    except  EOFError:
      self.ExitHandler(0, 'Received EOF')
    except IOError, msg:
      self.ExitHandler(0, msg)
  
  def ExitHandler(self, signal, eventmsg):
    """ SIGINT, and other bad tidings, exit handler """
    self.PError('Shell', self.mShName, eventmsg)
    self.Print(self.CmdQuit())
  

#-------------------------------------------------------------------------------
# CLASS: DummyCmd
#-------------------------------------------------------------------------------
class DummyCmd:
  """ Robot Dummy Commands Class. """
  def Open(self, port, baudrate=9600): pass
  def Close(self): pass
  def IsOpen(self): return True
  def GetErrStr(self): return ''
  def SendCmd(self, cmd): return 'rawrsp'
  def FlushInput(self): pass
  def SetDebugLevel(self, dl): pass
  def GetPort(self): return '/dev/dummy'


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

import sys
import getopt

_Argv0   = __file__
_ShName  = 'USERSHELL'

#--
class Usage(Exception):
  """ Command-Line Options Usage Exception Class. """
  def __init__(self, msg):
    self.msg = msg

#--
def PrintUsageErr(emsg):
  """ Print Error Usage Message. """
  if emsg:
    print("%s: %s" % (_Argv0, emsg))
  else:
    print("%s: error" % (_Argv0))
  print("Try '%s --help' for more information." % (_Argv0))

#--
def PrintUsage():
  """ Print Shell Command-Line Usage Message """
  print("usage: %s [options]..." % (_Argv0))
  print("USERSHELL %s Command Shell." % (_ShName))
  print("""Options and arguments:
  -p, --port <port>          : Open on this serial port.
  -b, --baudrate <baudrate>  : Open with this baudrate.
  -s, --script <file>        : Read commands from file.

  -h, --help                 : Display this help and exit.

Environment variables:
PYTHONPATH                   : a ':'-seperated list of additional directories
                               to search for python packages and modules.
  """)
  
#--
def main(shclass=None, argv=None, **kwargs):
  """ Raw Main. """
  global _Argv0, _ShName

  if not shclass:
    shclass = USERSHELL

  if argv is None:
    argv = sys.argv

  if kwargs.has_key('argv0'):
    _Argv0 = kwargs['argv0']
  else:
    _Argv0   = __file__
  if kwargs.has_key('shName'):
    _ShName = kwargs['shName']
  else:
    _ShName  = 'USERSHELLNAME'

  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hs:p:b:",
                                ['help', 'script=', 'port=', 'baudrate='])
    except getopt.error, msg:
      raise Usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        PrintUsage()
        return 0
      elif opt in ('-s', '--script'):
        kwargs['script'] = optarg
      elif opt in ('-p', '--port'):
        kwargs['port'] = optarg
      elif opt in ('-b', '--baudrate'):
        try:
          kwargs['baudrate'] = int(optarg)
        except ValueError, msg:
          raise Usage(msg)
  except Usage, err:
    PrintUsageErr(err.msg)
    return 2

  sh = shclass(**kwargs)

  return sh.Run()

#--
if __name__ == "__main__":
  sys.exit( main(shclass=USERSHELL, argv=None) )
