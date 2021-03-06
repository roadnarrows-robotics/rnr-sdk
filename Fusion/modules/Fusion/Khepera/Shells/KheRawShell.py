#! /usr/bin/env python
################################################################################
#
# KheRawShell.py
#

""" Khepera II Raw Command-Line Shell Module

Khepera II raw serial command-line shell base class module.

The KheRawShell is GUI auto-capable if a GuiTerm is attached.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.24

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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

try:
  import readline   # optional module
except ImportError:
  pass

import Fusion.Utils.Shell as Shell
import Fusion.Utils.Tools as utils

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiWinShell as gterm

import Fusion.Khepera.Cmd.KheSerial as KheSerial


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: KheRawShell
#-------------------------------------------------------------------------------
class KheRawShell(Shell.Shell):
  """ Khepera Raw Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Raw Shell.

        Parameters:
          args    - arguments (not used)
            robotCmds   - robot commands object. Default: KheSerial()
            baudrate    - command option serial port baudrate. Default: 9600
            port        - command option serial port device name.
                          Default: None
            closeonexit - do [not] close serial connection on exit.
                          Default: True
            **shopts    - Shell core options.
    """
    # initialize shell base object
    Shell.Shell.__init__(self, args, **kwargs)

    # status data initialization
    self.mPortIsOpen    = False             # track port state 

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # add to options
    self.mOpts['baudrate']    = 9600
    self.mOpts['port']        = None
    self.mOpts['robotCmds']   = None
    self.mOpts['closeonexit'] = True

    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'Khepera Raw',
      'shVersion': '1.1',
      'shPS1': 'kheraw$ ',
      'shPS2': 'kheraw> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.items():
      if key not in kwargs or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = KheSerial.KheSerial()
        else:
          kwargs[key] = value

    # set all options from arguments
    Shell.Shell.Config(self, **kwargs)

    # makes referencing a little easier
    self.mRobotCmds = self.mOpts['robotCmds']


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # The Command Dictionary (See Shell.py)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def InitCmdDict(self):
    """ Initialize Command Dictionary Override.

        Return Value:
          None
    """
    # set shell base object's commands
    Shell.Shell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      # open command
      'open': {
        'help': {
          'desc': "Open serial connection to the Khepera robot.",
          'usage': "open <port> [<baudrate>]",
          'args': {
            '<port>': "Port name (device).",
            '<baudrate>': "Supported baudrate. Default: 9600."
          },
          'rvals': "port <port> opened at <baudrate> baud"
        },
        'parse': [
          { 'refmt': Shell.ReStart + Shell.ReArgStr + Shell.ReArgPosInt +
                     Shell.ReEnd,
            'cvt':   [self.CvtStr, self.CvtInt]
          },
          { 'refmt': Shell.ReStart + Shell.ReArgStr + Shell.ReEnd,
            'cvt':   [self.CvtStr]
          }
        ],
        'prereq': [],
        'exec':   self.CmdOpen
      },

      # close command
      'close': {
        'help': {
          'desc': "Close connection to the Khepera robot.",
          'usage': "close",
          'rvals': 'connection closed'
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [],
        'exec':   self.CmdClose
      },


      # rawcmd command
      'rawcmd': {
        'help': {
          'desc': "Send raw command to the serial connection.",
          'usage': [
            'rawcmd <cmdstr>',
          ],
          'args': {
            '<cmdstr>': "Raw command string.",
          },
          'rvals': [
            "Response string."
          ],
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgStr + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdRawCmd
      },

      # flush command
      'flush': {
        'help':{
          'desc': "Flush serial connection input and output, "
                  "discarding all data in buffers.",
          'usage': "flush",
          'rvals': "None"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdFlush
      },

      # err command
      'err': {
        'help':{
          'desc': "Print last serial connection error.",
          'rvals': "Last error string."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [],
        'exec':   self.CmdErr
      },

      # debug command
      'debug': {
        'help': {
          'desc': "Set serial connection debugging level.",
          'usage': "debug <level>",
          'args': {
            '<level>': 'Debugging level 0=off, 1-5'
          },
          'rvals': 'debugging level <level>'
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [],
        'exec':   self.CmdDebug
      }
    }
  
    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdOpen(self, port, baudrate=9600):
    """ Open Serial Port Shell Command. """
    try:
      self.mRobotCmds.Open(port, baudrate=baudrate)
    except IOError:
      return 'err', self.mRobotCmds.GetErrStr()
    self.mPortIsOpen = True
    return 'ok', "port %s opened at %d baud" % (port, baudrate)

  #--
  def CmdClose(self):
    """ Close Serial Port Shell Command. """
    self.mRobotCmds.Close()
    self.mPortIsOpen = False
    return 'ok', 'connection closed'
  
  #--
  def CmdRawCmd(self, cmd):
    """ Raw Khepera Shell Command. """
    rsp = self.mRobotCmds.SendCmd(cmd)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp
  
  #--
  def CmdFlush(self):
    """ Flush Serial Input Shell Command. """
    self.mRobotCmds.FlushInput()
    self.mRobotCmds.FlushOutput()
    return 'ok', ''
  
  #--
  def CmdErr(self):
    """ Return Last Khepera Error Shell Command. """
    return 'ok', self.mRobotCmds.GetErrStr()
  
  #--
  def CmdDebug(self, dl):
    """ Enable/Disable Serial Port Debugging Shell Command. """
    self.mRobotCmds.SetDebugLevel(dl)
    return 'ok', 'debugging level %d' % dl
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parameter Converters 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CvtTrue(self, s):
    """ Blindly Convert Argument into True. """
    return 'ok', True
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Prerequisites 
  #   All prerequisite checkers must return either True (ok) or False.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PreReqOpen(self, cmd):
    """ 'Serial Port Must Be Open' Prerequisite. """
    b = self.mRobotCmds.IsOpen()
    if not b:
      if self.mPortIsOpen:
        self.Print('Port closed\n')
      self.RspErr("command requires port to be open: '%s'" % (cmd))
    else:
      if not self.mPortIsOpen:
        self.Print("Port '%s' opened.\n" % (self.mRobotCmds.GetPort()))
    self.mPortIsOpen = b
    return self.mPortIsOpen


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Response Post-Processors
  #   All response post-processors must print any formatted data to fout.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Callback Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def OnRunInit(self):
    """ On Run Initialization Callback.
    
        Auto-connect to robot if the port is specified.
    
        Return Value:
          None
    """
    if not self.mRobotCmds:
      self.mRobotCmds = KheSerial.KheSerial()

    if self.mOpts['port'] and  not self.mRobotCmds.IsOpen():
      try:
        # establish communication with robot, change as needed
        self.mRobotCmds.Open(self.mOpts['port'],
            baudrate=self.mOpts['baudrate'])
      except IOError as err:
        PrintUsageErr("%s" % err)
        return 2

  #--
  def OnRunStart(self):
    """ On Run Start Callback.

        Print robot connect status.
    
        Return Value:
          None
    """
    if self.mRobotCmds.IsOpen():
      self.Print("Port '%s' open at %d baud.\n" % \
          (self.mRobotCmds.GetPort(), self.mRobotCmds.GetBaudRate()))
      self.mPortIsOpen = True

  #--
  def OnQuit(self):
    """ On Quit Callback.

        Close connection.

        Return Value:
          None
    """
    if self.mOpts['closeonexit']:
      self.mRobotCmds.Close()


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

import sys
import getopt

_Argv0   = __file__
_ShName  = 'kheraw'

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
  print("%s Command Shell." % (_ShName))
  print("""Options and arguments:
  -p, --port <port>          : Open on this serial port.
  -b, --baudrate <baudrate>  : Open with this baudrate.
  -s, --script <file>        : Read commands from file.

  -h, --help                 : Display this help and exit.
  """)
  
#--
def main(shclass=None, argv=None, **kwargs):
  """ Raw Main. """
  global _Argv0, _ShName

  if not shclass:
    shclass = KheRawShell

  if argv is None:
    argv = sys.argv

  _Argv0 = kwargs.get('argv0', __file__)
  _ShName = kwargs.get('shName', 'Khepera Raw')

  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hs:p:b:",
                                ['help', 'script=', 'port=', 'baudrate='])
    except getopt.error as msg:
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
        except ValueError as msg:
          raise Usage(msg)
  except Usage as err:
    PrintUsageErr(err.msg)
    return 2

  sh = shclass(**kwargs)

  return sh.Run()

#--
if __name__ == "__main__":
  sys.exit( main(shclass=KheRawShell, argv=None) )
