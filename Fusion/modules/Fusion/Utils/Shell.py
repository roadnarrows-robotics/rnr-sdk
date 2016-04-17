################################################################################
#
# Shell.py
#

""" Fusion Base Shell Module

Fusion standard Shell base class module.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.12.14

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

#
# Standard Regular Expression for Syntax Parser
#
ReStart     = '^\s*(\S+)'         # command's start regular expression
ReEnd       = '\s*(?:#.*|$)'      # command's end regular expression
ReX         = lambda x: '\s+('+x+')' # general argument(s) regular expression
ReArgStr    = '\s+(\S+)'          # single non-whitespace string argument
ReArgInt    = '\s+([-+]?\d+)'     # integer argument
ReArgPosInt = '\s+(\d+)'          # strictly positive integer argument
ReArgFloat  = '\s+([-+]?\d+\.?\d*|[-+]?\d*\.?\d+)'  # float argument
ReArgOnOff  = '\s+(on|off)'       # on or off argument
ReArgAll    = '\s+([^#]+)'        # all characters between cmd and comment/eol

#
# Other
#
PrettyPrintMaxCol = 80    # maximum column for pretty printing. Note that this
                          # should change dynamically based on terminal window
                          # width (future)

#-------------------------------------------------------------------------------
# CLASS: KHR1RawShell
#-------------------------------------------------------------------------------
class Shell:
  """ Shell Base Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of key=value arguments
            argv0       - Shell command file name. Default: this file name
            shVersion   - Shell version string. Default: '1.0'
            shName      - Shell name. Default: 'fusionshell'
            shPS1       - Primary prompt string. Default: 'fsh$ '
            shPS2       - Secondary prompt string. Default: 'fsh> '
            script      - Command option script file name. Default: None
            maxDepth    - Maximum script file depth. Default: 5
            helpExample - Help example string. Default: ''
            fin         - User input stream. Default: stdin
            fout        - User output stream. Default: stdout
 
    """
    # option defaults
    self.mOpts = {
      'argv0': os.path.basename(__file__),
      'shName': 'FusionShell', 'shVersion': '1.0',
      'shPS1': 'fsh$ ', 'shPS2': 'fsh> ',
      'script': None, 'maxDepth': 5,
      'helpExample': '',
      'fin': sys.stdin, 'fout': sys.stdout
    }

    # set options from arguments
    self.Config(**kwargs)

    # initialize command dictionary
    self.InitCmdDict()

    # set gui colors
    self.SetStyles()

    # status data initialization
    self.mScriptFile    = None      # current command script file
    self.mScriptDepth   = 0         # current script file depth
    self.mDoQuit        = False     # do [not] quit this shell
    self.mIsInteractive = True      # shell is in user interactive mode

  #--
  def Config(self, **kwargs):
    """ Configure options from argument list.

        Parameters:
          kwargs  - dictionary of option arguments

        Return Value:
          None
    """
    for key, value in kwargs.iteritems():
      if self.mOpts.has_key(key):
        self.mOpts[key] = value

    self.mOpts['argv0'] = os.path.basename(self.mOpts['argv0'])

  #--
  def SetStyles(self):
    """ Set text styles if shell attached to a GUI. """
    gterm.SendAddStyle('dft', fg=gt.ColorFgShellDft)        # default
    gterm.SendAddStyle('err', fg=gt.ColorFgShellErr)        # error
    gterm.SendAddStyle('pun', fg=gt.ColorFgShellPun)        # punctuation
    gterm.SendAddStyle('key', fg=gt.ColorFgShellKey)        # keyword
    gterm.SendAddStyle('num', fg=gt.ColorFgShellNum)        # numeric
    gterm.SendAddStyle('str', fg=gt.ColorFgShellStr)        # string
    gterm.SendAddStyle('boo', fg=gt.ColorFgShellBool)       # boolean
    gterm.SendAddStyle('hel', fg=gt.ColorFgShellHelp)       # help
    gterm.SendAddStyle('hul', fg=gt.ColorFgShellHelp, ul=1) # help emphasis


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # The Command Dictionary (append to this dictionary)
  #   Per each command:
  #     'help'    help dictionary
  #       'desc'    description string or list of strings
  #       'usage'   usage string or list of strings
  #       'args'  arguments string or list of strings
  #       'rvals'   return value description string or list of strings
  #     'parse'   list of syntax parse dictionaries. each dictionary:
  #       'refmt'   regular expression used to parse user input line 
  #       'cvt'     argument conversion function list (one per argument,
  #                 parsed, with last converter repeated if more arguments)
  #     'prereq'  command prerequisite function list
  #     'exec'    command execution function
  #     'rsp'     response post-processor function (optional)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def InitCmdDict(self):
    """ Initialize Command Dictionary.

        Return Value:
          None
    """
    self.mCmdDict = {
      'help': {
        'help': {
          'desc': 'Print (full) help for all commands or for a specific '
                  'command.',
          'usage': [
            'help [<cmd>]',
            'help -l[ist]',
          ],
          'args': {
            '<cmd>': "One of the supported shell commands.",
          },
          'rvals': 'None'
        },
        'parse': [
          { 'refmt': ReStart + ReX('-l|-list') + ReEnd,
            'cvt':   [self.CvtStr]
          },
          { 'refmt': ReStart + ReArgStr + ReEnd,
            'cvt':   [self.CvtStr]
          },
          { 'refmt': ReStart + ReEnd,
            'cvt':   []
          },
        ],
        'prereq': [],
        'exec':   self.CmdHelp
      },
      'script': {
        'help': {
          'desc': "Run a script file through this shell.",
          'usage': 'script [<file>]',
          'args': {
            '<file>': 'Script file name.',
          },
          'rvals': 'None'
        },
        'parse': [
          { 'refmt':  ReStart + ReArgStr + ReEnd,
            'cvt':    [self.CvtStr],
          },
        ],
        'prereq': [],
        'exec':   self.CmdScript
      },
      'echo': {
        'help': {
          'desc': "Display a line of text.",
          'usage': 'echo [<string> [string ...]]',
          'args': {
            '<string>': 'String of text.',
          },
          'rvals': 'Line of text.'
        },
        'parse': [
          { 'refmt':  ReStart + ReArgAll + ReEnd,
            'cvt':    [self.CvtStr],
          },
        ],
        'prereq': [],
        'exec':   self.CmdEcho
      },
      'quit': {
        'help': {
          'desc': "Close resources and quit this shell.",
          'usage': 'quit',
          'rvals': 'good-bye'
        },
        'parse': [
          { 'refmt':  ReStart + ReEnd,
            'cvt':    [],
          },
        ],
        'prereq': [],
        'exec':   self.CmdQuit
      },
      'wait': {
        'help': {
          'desc': "Wait until the given seconds have elapsed.",
          'usage':  'wait <seconds>',
          'args': {
            '<seconds>': "Seconds > 0.0"
          },
          'rvals': 'None'
        },
        'parse': [
          { 'refmt':  ReStart + ReArgFloat + ReEnd,
            'cvt':    [self.CvtFloat],
          },
        ],
        'prereq': [],
        'exec':   self.CmdWait
      }
    }
  
  #--
  def AddToCmdDict(self, newdict):
    """ Add a new command dictionary to the existing dictionary. A new
        command will replace a command with the same name.

        Parameters:
          newdict   - New command dictionary.

        Return Value:
          None.
    """
    for k,v in newdict.iteritems():
      self.mCmdDict[k] = v


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Core Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdScript(self, scriptFile):
    """ Run Script Shell Command. """
    if self.mScriptDepth > self.mOpts['maxDepth']:
      return 'err', 'script depth exceeds max: %d' % self.mOpts['maxDepth']
    try:
      self.mScriptFile = open(scriptFile, 'r')
      self.mIsInteractive = False
    except IOError, err:
      return 'err', err
    self.mScriptDepth += 1
    self.RunLoopScript()
    self.mScriptDepth -= 1
    if self.mScriptDepth == 0:
      self.mIsInteractive = True
    return 'ok', ''
  
  #--
  def CmdEcho(self, str=''):
    """ Echo Shell Shell Command. """
    return 'ok', str
  
  #--
  def CmdQuit(self):
    """ Quit Shell Command. """
    self.OnQuit()
    self.mDoQuit = True
    return 'ok', "good-bye"
  
  #--
  def CmdWait(self, seconds):
    """ Wait Shell Command. """
    time.sleep(seconds)
    return 'ok', ''
  
  #--
  def CmdHelp(self, cmd=None):
    """ Help Shell Command. """
    # list of commands help
    if cmd in ['-l', '-list']:
      cmdList = self.mCmdDict.keys()
      cmdList.sort()
      s = ''
      for cmd in cmdList:
        s += cmd + ' '
      gterm.SendSetStyle('hel')
      self.HelpPrintBlock('', s)
      gterm.SendSetStyle('dft')
      return 'ok', ''

    # specific command help
    elif cmd:
      if self.mCmdDict.has_key(cmd):
        self.HelpPrintCmd(cmd)
        return 'ok', ''
      else:
        return 'err', 'help %s not found' % repr(cmd)

    # full help
    gterm.SendSetStyle('hul')
    banner = "%s Command Shell Help, v%s" % \
        (self.mOpts['shName'], self.mOpts['shVersion'])
    self.Print(banner)
    gterm.SendSetStyle('hel')
    self.Print('')
    self.Print('Module: %s' % self.mOpts['argv0'])
    self.Print("""
Enter any of the commands listed below to execute the desired action(s).
Partial command matching is done to find the unique command substring. 
""")
    if self.mOpts['helpExample']:
      self.Print(self.mOpts['helpExample'])
      self.Print('')
  
    cmdList = self.mCmdDict.keys()
    cmdList.sort()
    self.Print("Command Set:")
    for cmd in cmdList:
      self.HelpPrintCmd(cmd)
    return 'ok', ''

  #--
  def HelpPrintCmd(self, cmd):
    """ Print specific command help.

        Parameters:
          cmd - Supported shell command name.

        Return Value:
          None
    """
    help = self.mCmdDict[cmd]['help']

    # header
    gterm.SendSetStyle('hul')
    self.Write(cmd)
    if len(cmd) < 27:   # purely esthetics
      gterm.SendSetStyle('dft')
      s = ' ' * (79-2*len(cmd))
      self.Write(s)
      gterm.SendSetStyle('hul')
      self.Write(cmd)
    self.Write('\n')

    # body
    gterm.SendSetStyle('hel')
    if help.has_key('desc'):
      self.Print('  Description:')
      self.HelpPrintBlock("    ", help['desc'])
    if help.has_key('usage'):
      self.Print('  Usage:')
      self.HelpPrintBlock("    ", help['usage'])
    if help.has_key('args'):
      self.HelpPrintCmdArgs(help['args'])
    if help.has_key('rvals'):
      self.Print("  Return Values:")
      self.HelpPrintBlock("    ", help['rvals'])
    self.Print('')
    gterm.SendSetStyle('dft')

  #--
  def HelpPrintCmdArgs(self, args):
    """ Print command arguments help.

        Parameters:
          args - Dictionary of command arguments.

        Return Value:
          None
    """
    if not args:
      return
    s = '  Arguments:'
    self.Print(s)
    maxlen = 0
    keys = args.keys()
    keys.sort()
    for opt in keys:
      if len(opt) > maxlen:
        maxlen = len(opt)
    for opt in keys:
      pad = ' ' * (maxlen - len(opt))
      preface = "    %s%s - " % (opt, pad)
      self.HelpPrintBlock(preface, args[opt])

  #--
  def HelpPrintBlock(self, preface, strlist, newline=False):
    """ Print help text block.

        Parameters:
          options - Dictionary of command options.

        Return Value:
          None
    """
    col = indent = len(preface)
    if type(strlist) != list:
      strlist = [strlist]
    s = preface
    if newline:
      s += '\n'
      s += ' ' * indent
    sep = ''
    for str in strlist:
      s += sep
      if sep == '\n':
        s += ' ' * indent
        col = indent
      l = len(str)
      while l > 0:
        if l < 79 - col:
          s += str
          l = 0
        else:
          i = str.rfind(' ', 0, 79-col)
          if i == -1:
            i = len(s)
          s += str[0:i]
          str = str[i+1:]
          s += '\n'
          s += ' ' * indent
          col = indent
          l = len(str)
      sep = '\n'
    self.Print(s)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Core Parameter Converters 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CvtStr(self, s):
    """ Convert String Argument into String (NoOp). """
    return 'ok', s
  
  #--
  def CvtTrue(self, s):
    """ Blindly Convert Argument into True. """
    return 'ok', True
  
  #--
  def CvtOnOff(self, s):
    """ Convert {on | off} into Integer. """
    if s == 'on':
      return 'ok', 1
    elif s == 'off':
      return 'ok', 0
    else:
      return 'err', "not 'on' or 'off'"
  
  #--
  def CvtFloat(self, s):
    """ Convert Argument into Float. """
    try:
      val = float(s)
      return 'ok', val
    except (SyntaxError, NameError, TypeError, ValueError):
      return 'err', "bad float: %s" % (s)
  
  #--
  def CvtInt(self, s, base=10):
    """ Convert Argument into Integer. """
    try:
      val = int(s, base)
      return 'ok', val
    except (SyntaxError, NameError, TypeError, ValueError):
      return 'err', "bad integer: %s" % (s)
  
  #--
  def CvtIntList(self, s, min=0, max=0xff):
    """ Convert Argument into Integer List. """
    intList = []
    for sArg in s.split():
      if len(sArg) > 2 and (sArg[0:2] == '0x' or sArg[0:2] == '0X'):
        base = 16
      else:
        base = 10
      rc, i = self.CvtInt(sArg, base)
      if rc == 'err':
        return 'err', i
      if i < min or i > max:
        if base == 10:
          return 'err', '%s is out of range [%d,%d]' % (sArg, min, max)
        else:
          return 'err', '%s is out of range [0x%x,0x%x]' % (sArg, min, max)
      intList += [i]
    return 'ok', intList


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Core Shell Prerequisites 
  #   All prerequisite checkers must return either True (ok) or False.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Response Post-Processors
  #   All response post-processors must print any formatted data to fout.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def RspRepr(self, rsp):
    """ Safe Response Formatter. """
    self.Print(repr(rsp))
  
  #--
  def RspErr(self, err):
    """ Error Response Formatter. """
    gterm.SendSetStyle('err')
    self.Print("error: %s" % err)
    gterm.SendSetStyle('dft')

  #--
  def RspPrettyPrint(self, data):
    """ Pretty Print Response Formatter. """
    if gterm.ThisShellHasAGui():
      self._RspGuiPrettyPrint(0, 0, data)
      gterm.SendSetStyle('dft')
      self.Write('\n')
    else:
      self.Print(data)

  #--
  def _RspGuiPrettyPrint(self, col, indent, data):
    """ GUI Pretty Print Response Formatter Workhorse.  """
    # number
    if type(data) == int or type(data) == float:
      gterm.SendSetStyle('num')
      s = repr(data)
      if col + len(s) > PrettyPrintMaxCol-2:
        self.Write('\n')
        col = self.Write('%*s' % (indent, ''))
      col += self.Write('%s' % (s))
    # string
    elif type(data) == str:
      gterm.SendSetStyle('str')
      if col + len(data) > PrettyPrintMaxCol-2:
        self.Write('\n')
        col = self.Write('%*s' % (indent, ''))
      col += self.Write('%s' % (data))
    # boolean
    elif type(data) == bool:
      gterm.SendSetStyle('boo')
      s = repr(data)
      if col + len(s) > PrettyPrintMaxCol-2:
        self.Write('\n')
        col = self.Write('%*s' % (indent, ''))
      col += self.Write('%s' % (s))
    # tuple
    elif type(data) == tuple:
      gterm.SendSetStyle('pun')
      if col < indent:
        col += self.Write('%*s' % (indent-col, ''))
      col += self.Write('(')
      n = 0
      for v in data:
        if n > 0:
          gterm.SendSetStyle('pun')
          col += self.Write(', ')
        col = self._RspGuiPrettyPrint(col, indent+1, v)
        n += 1
      gterm.SendSetStyle('pun')
      col += self.Write(')')
    # list
    elif type(data) == list:
      gterm.SendSetStyle('pun')
      if col < indent:
        col += self.Write('%*s' % (indent-col, ''))
      self.Write('[\n')
      col = self.Write('%*s' % (indent+2, ''))
      n = 0
      for v in data:
        if n > 0:
          gterm.SendSetStyle('pun')
          col += self.Write(', ')
        col = self._RspGuiPrettyPrint(col, indent+2, v)
        n += 1
      gterm.SendSetStyle('pun')
      self.Write('\n')
      col = self.Write('%*s]' % (indent, ''))
    # dictionary
    elif type(data) == dict:
      gterm.SendSetStyle('pun')
      if col < indent:
        col += self.Write('%*s' % (indent-col, ''))
      col += self.Write('{ ')
      keys = data.keys()
      keys.sort()
      n = 0
      for k in keys:
        if n > 0:
          gterm.SendSetStyle('pun')
          col += self.Write(',')
        self.Write('\n')
        col = 0
        gterm.SendSetStyle('key')
        if type(k) == str:
          col += self.Write('%*s%s' %(indent+2, '', k))
        else:
          col += self.Write('%*s%s' %(indent+2, '', repr(k)))
        gterm.SendSetStyle('pun')
        col += self.Write(':')
        self._RspGuiPrettyPrint(col, indent+2, data[k])
        n += 1
      gterm.SendSetStyle('pun')
      self.Write('\n')
      col = self.Write('%*s}' % (indent, ''))
    # default
    else:
      gterm.SendSetStyle('dft')
      col += self.Write('%*s%s' % (indent, '', repr(data)))
    return col

  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Derived Shell Callbacks (Override as Necessary)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def OnRunInit(self):
    """ On Run Initialization Callback.
    
        Return Value:
          None
    """
    pass

  #--
  def OnRunStart(self):
    """ On Run Start Callback.

        Return Value:
          None
    """
    pass

  #--
  def OnQuit(self):
    """ On Quit Callback.

        Return Value:
          None
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Output Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  #--
  def PError(self, *args):
    """ Print error to stderr. """
    for arg in args:
      print >>sys.stderr, arg,
    print >>sys.stderr

  #--
  def Print(self, *args):
    """ Print to output stream. """
    # do not use 'print >>fout' calls - it locks up threads reading pipes
    sep = ''
    for arg in args:
      if type(arg) == str:
        self.mOpts['fout'].write(arg+sep)
      else:
        self.mOpts['fout'].write(repr(arg)+sep)
      sep = ' '
    self.mOpts['fout'].write('\n')
    self.mOpts['fout'].flush()

  #--
  def Write(self, s):
    """ Write to output stream. """
    self.mOpts['fout'].write(s)
    self.mOpts['fout'].flush()
    return len(s)
 
  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parse Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  #--
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
  
  #--
  def ParseGetInputCmd(self, inputLine):
    """ Get the Input Command (first argument). """
    mo = re.search('\s*(\S+)', inputLine)
    if not mo:
      return None
    return mo.group(1) 
  
  #--
  def ParseCvtInput(self, cmd, inputLine):
    """ Convert Input Command Arguments into Executable Format. """
    # match command to regular expression
    for parser in self.mCmdDict[cmd]['parse']:
      inputArgs = re.findall(parser['refmt'], inputLine)
      if inputArgs:
        break

    # no match
    if not inputArgs:
      self.RspErr("invalid syntax: '%s'" % (inputLine))
      return None

    # convert matched input arguments, building command
    inputArgs = inputArgs[0]   # get first element tuple in list '[(...)]'
    cmdArgs = [cmd]
    if type(inputArgs) == tuple:
      n = 0
      last = len(parser['cvt']) - 1   # last converter
      for arg in inputArgs[1:]:
        if n <= last:
          rc, cvtArg = parser['cvt'][n](arg)
        else:
          rc, cvtArg = parser['cvt'][last](arg)
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
  
  #--
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
    rc, rval = cmdfn(*cmdArgs[1:])
    if rc == 'err':
      self.RspErr("%s" % (rval))
    elif rval != '':
      if self.mCmdDict[cmdArgs[0]].has_key('rsp'):
        self.mCmdDict[cmdArgs[0]]['rsp'](rval)
      else:
        self.RspPrettyPrint(rval)
  
  #--
  def ExecChkPreReqs(self, cmd):
    """ Check All Prerequisites Prior to Command Execution. """
    for prereq in self.mCmdDict[cmd]['prereq']:
      if not prereq(cmd):
        return False
    return True


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Run Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
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

    # make callback to derived shell
    self.OnRunInit()

    if self.mOpts['script']:
      try:
        self.mScriptFile = open(self.mOpts['script'], 'r')
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
  
  #--
  def RunLoopUser(self):
    """ Run Loop for User Interactive Shell. """
    self.Print("%s Command Shell, Version %s" % \
        (self.mOpts['shName'], self.mOpts['shVersion']))
    self.Print(" (type 'help' for list of commands)")
    self.Print("")
    self.OnRunStart()
    while not self.mDoQuit:
      inputLine = self.GetUserInput(self.mOpts['shPS1'])
      if inputLine:
        cmdArgs = self.ParseCmd(inputLine)
        if cmdArgs:
          self.ExecCmd(cmdArgs)
  
  #--
  def RunLoopScript(self):
    """ Run Loop for Command Script. """
    inputLine = self.mScriptFile.readline()
    while not self.mDoQuit and inputLine:
      cmdArgs = self.ParseCmd(inputLine)
      if cmdArgs:
        self.ExecCmd(cmdArgs)
      inputLine = self.mScriptFile.readline()
    self.mScriptFile.close()
  
  #--
  def GetUserInput(self, prompt):
    """ Get User Input. """
    try:
      inputLine = utils.user_input(prompt, fin=self.mOpts['fin'],
          fout=self.mOpts['fout'])
      return inputLine
    except KeyboardInterrupt:
      self.ExitHandler(0, 'Received Keyboard Interurrupt')
    except  EOFError:
      self.ExitHandler(0, 'Received EOF')
    except IOError, msg:
      self.ExitHandler(0, msg)
  
  #--
  def ExitHandler(self, signal, eventmsg):
    """ SIGINT, and other bad tidings, exit handler """
    self.PError('Shell', self.mOpts['shName'], eventmsg)
    self.Print(self.CmdQuit())


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':

  import Tkinter as tk

  #--
  def main():
    """ Shell Unit Test Main """
    root = tk.Tk()
    shwin = gterm.GuiWinShell(root, Shell, shPS1='utshell$ ')
    root.mainloop()

  # run unit test
  main()
