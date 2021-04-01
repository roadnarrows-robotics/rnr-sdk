#! /usr/bin/env python

################################################################################
#
# DemoFusion.py
#

"""Fusion Demo

The Fusion Demo demonstrates a command-line interface to the Fusion Reactor
application.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.01.05

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
import getopt
import Fusion.Core.Reactor as Reactor

_Argv0   = ''

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
  """ Print Fusion Command-Line Usage Message """
  print("usage: %s [options]..." % (_Argv0))
  print("""Options and arguments:
  -i, --ini <filename>       : additional ini configuration file. Default: None
      --debuglevel <num>     : debug level 0=off, 1 - 5. Default: 0
      --debugfile <filename> : debug output filename. Default: stdout

  -h, --help                 : Display this help and exit.

Environment variables:
FUSION                       : points to the Fusion package base directory
FUSIONSTARTUP                : a site/user standard ini configuration file
  """)
  
#--
def main(argv=None, **kwargs):
  """ Fusion Main. """
  global _Argv0

  print('main')
  if argv is None:
    argv = sys.argv

  print('args', repr(argv))
  _Argv0 = argv[0]
  #if 'argv0' in kwargs:
  #  _Argv0 = kwargs['argv0']
  #else:
  #  _Argv0   = __file__

  # Reactor defaults
  kwargs = {'vRobot': None, 'vBrain': None, 'iniFileName': None,
            'debuglevel': 0, 'debugfout': None}
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hi:",
                                ['help', 'ini=', 'debuglevel=', 'debugfile='])
    except getopt.error as msg:
      raise Usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        PrintUsage()
        return 0
      elif opt in ('-i', '--ini'):
        kwargs['iniFileName'] = optarg
      elif opt in ('--debuglevel'):
        try:
          kwargs['debuglevel'] = int(optarg)
        except ValueError as msg:
          raise Usage(msg)
      elif opt in ('--debugfile'):
        try:
          fout = open(optarg, 'w')
        except IOError as msg:
          raise Usage(msg)
        kwargs['debugfout'] = fout
  except Usage as err:
    PrintUsageErr(err.msg)
    return 2

  reactor = Reactor.Reactor(**kwargs)
  reactor.mGuiRoot.mainloop()
  return 0


# run fusion
if __name__ == "__main__":
  sys.exit( main(argv=None) )
