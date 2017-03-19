###############################################################################
#
# Package:  NetMsgs
#
# File:     dumpxmldb
#

## \file 
##
## $LastChangedDate: 2010-07-31 08:48:56 -0600 (Sat, 31 Jul 2010) $
## $Rev: 521 $
##
## \brief Python test script to dump the post-parsed xml database.
##
## \par Usage:
## netmsgsgen [OPTIONS] --xml=<em>xmlfile</em>
## 
## netmsgsgen --version
## netmsgsgen --help
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2010-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##

#
###############################################################################

import sys
import os
import getopt

# fix up path to point to pre-installed package area
pkgroot   = '/prj/pkg/netmsgs'
sys.path  = [pkgroot+'/nmPython/tests', pkgroot+'/nmPython/modules'] + sys.path

import NetMsgs.NetMsgsXmlParser as nmXmlParser

## invoked command name
_Argv0    = __file__

## command name
_CmdName  = 'dumpxmldb.py'

## command version
_CmdVer   = '1.0.0'

#--
class Usage(Exception):
  """ Command-Line Options Usage Exception Class. """
  def __init__(self, msg):
    self.msg = msg
##

#--
def PrintUsageErr(emsg):
  """ Print Error Usage Message. """
  if emsg:
    print "%s: %s" % (_Argv0, emsg)
  else:
    print "%s: error" % (_Argv0)
  print "Try '%s --help' for more information." % (_Argv0)
##

#--
def PrintUsage():
  """ Print Command-Line Print Usage Message. """
  print """
usage: %s [OPTIONS] ---xml=<xmlfile>

       %s [OPTIONS] --version
       %s [OPTIONS] --help
"""  % (_Argv0, _Argv0, _Argv0)
  print """Options and arguments:
  -d, --debug=<level>       : Debug level. One of: 0 1 2
                              Default: 0  (off)

  -x, --xml=<xml>           : Input NetMsgs XML file path name.

  -v, --version             : Display version strings and exit.
  -h, --help                : Display this help and exit.
  """
##

#--
def GetOptions(argv=None, **kwargs):
  """ Get Main Options

      Parameters:
        argv    - Argument list. If not None, the overrides command-line
                  arguments.
        kwargs  - Keyword argument list.  

      Return:
        Command arguments dictionary.
  """
  global _Argv0, _CmdName, _CmdVer

  # command-line arguments
  if argv is None:
    argv = sys.argv

  _Argv0  = kwargs.get('argv0', __file__)
  cmdargs = {}

  # defaults
  cmdargs['debug']      = kwargs.get('debug', 0)
  cmdargs['xmlfile']    = kwargs.get('xmlfile', None)

  # parse command-line options
  try:
    try:
      opts, args = getopt.getopt(argv[1:], "?hvd:x:",
          [ 'help', 'version', 'debug=', 'xml=', ''])
    except getopt.error, msg:
      raise Usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        PrintUsage()
        sys.exit(0)
      elif opt in ('-v', '--version'):
        print "%s %s" %(_Argv0, _CmdVer)
        sys.exit(0)
      elif opt in ('-d', '--debug'):
        try:
          cmdargs['debug'] = int(optarg)
        except ValueError:
          raise Usage("%s %s: not an integer value." % (opt, optarg))
      elif opt in ('-x', '--xml'):
        cmdargs['xmlfile'] = optarg
  except Usage, err:
    PrintUsageErr(err.msg)
    sys.exit(2)

  # validate and set command-line non-option arguments
  if not cmdargs['xmlfile']:
    PrintUsageErr("No input xml file specified.")
    sys.exit(2)

  return cmdargs
##

#--
def main(argv=None, **kwargs):
  """ Main.
        
      Parameters:
        argv    - Optional argument list to override command-line arguments.
        kwargs  - Optional keyword argument list.
  """
  global _Argv0, _CmdName, _CmdVer
  
  kwargs = GetOptions(argv, **kwargs)

  xml = nmXmlParser.NetMsgsXmlParser( filename=kwargs['xmlfile'],
                                      debug=kwargs['debug'] )

  try:
    xml.Parse()
  except nmXmlParser.NetMsgsXmlParserError as inst:
    print "Error: %s" % (inst)
    return 8

  xml.PrettyPrintDB()

  return 0
##

## Execute script.
if __name__ == '__main__':
  sys.exit( main(argv=None) )
