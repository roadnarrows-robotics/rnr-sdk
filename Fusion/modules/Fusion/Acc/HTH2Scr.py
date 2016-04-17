################################################################################
#
# HTH2Scr.py
#

""" Heart To Heart Conversion Module

Converts Heart To Heart comma separated values file to Fusion KHR-1
Shell script.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.01.15

Copyright (C) 2007.  RoadNarrows LLC.
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

# Default Home Position - change as necessary
HomePos = [
    5,   0,  90, 225, 225,  90, 175, 180,  90, 225, 225, 225,
   88, 115, 115,  90,  92, 225,  92,  65,  65,  90,  88, 225
]

# Servo Speed in seconds/degree
SpeedTbl = [
    0.0028,   # speed 0 (fastest)
    0.0030,   # speed 1
    0.0032,   # speed 2
    0.0034,   # speed 3
    0.0036,   # speed 4
    0.0038,   # speed 5
    0.0086,   # speed 6
    0.0178    # speed 7 (slowest)
]

def WaitTimeHeuristic(speed, prevChanList, nextChanList):
  """ Heuristically determine wait time between position changes. """
  n = 0
  maxDelta = 10
  while n < len(nextChanList):
    delta = abs(nextChanList[n]-prevChanList[n])
    if delta > maxDelta:
      maxDelta = delta
    n += 1
  wtime = maxDelta * SpeedTbl[speed]
  return wtime

#--
def CvtHomeRelPosToAbs(relChanList):
  """ Convert "relative to home" position to absolute position. """
  absChanList = []
  n = 0
  while n < len(relChanList):
    relpos = int(relChanList[n])
    if relpos != 225:
      absChanList += [(relpos+HomePos[n])]
    else:
      absChanList += [HomePos[n]]
    n += 1
  return absChanList

#--
def PrintHeader(ofp, hthFile, scrFile, desc):
  """ Print script file comment header block. """
  ofp.write('#' * 120 + '\n')
  ofp.write('# %s (converted from %s)\n' % \
    (os.path.basename(scrFile), os.path.basename(hthFile)))
  if desc:
    ofp.write('#\n')
    ofp.write('# %s\n' % desc)
  ofp.write('#\n')
  ofp.write('# RoadNarrows LLC\n')
  ofp.write('#' * 120 + '\n\n')

#--
def ParseCsvLine(line):
  """ Parse and convert one line of HeartToHeart CSV File. """
  args = line.split(',')
  speed = int(args[2])
  chanList = CvtHomeRelPosToAbs(args[3:])
  return 0, speed, chanList

#--
def Cvt2PosSetScript(ifp, ofp):
  """ Convert HTH file to position setting script. """
  oldChanList = HomePos
  line = ifp.readline()
  while line:
    rc, speed, chanList = ParseCsvLine(line)
    if rc != 0:
      return rc
    ofp.write('pos set speed %d channels' % speed)
    for pos in chanList:
      ofp.write(' %d' % pos)
    ofp.write('\n')
    wtime = WaitTimeHeuristic(speed, oldChanList, chanList)
    ofp.write('wait %4.2f\n' % wtime)
    oldChanList = chanList
    line = ifp.readline()
  ofp.close()
  return 0

#--
def Cvt2PgmMotionScript(ifp, ofp, motion):
  """ Convert HTH file to programming motion script. """
  ofp.write('# motion %d data\n' % (motion))
  line = ifp.readline()
  posnum = 0
  while line:
    rc, speed, chanList = ParseCsvLine(line)
    if rc != 0:
      return rc
    ofp.write('motion %d set pos %d speed %d ch' % (motion, posnum, speed))
    for pos in chanList:
      ofp.write(' %d' % pos)
    ofp.write('\n')
    line = ifp.readline()
    posnum += 1
  ofp.write('\n# number of motion %d positions\n' % (motion))
  ofp.write('motion %d set count %d\n' % (motion, posnum))
  ofp.close()
  return 0

#--
def HTH2Scr(hthFile, scrFile, desc, motion):
  """ Convert HeartToHeart comma separated values file to Fusion script
      file.

      Parameters:
        hthFile   - HeartToHeart input file name. If None, then stdin is
                    read for input.
        scrFile   - Fusion KHR-1 script output file name. If None, then
                    script file is written to stdout.
        desc      - Short description of motion.
        motion    - Program motion <motion> if not None rather than 
                    position setting script.

      Return Value:
        0   = success
        >0  = conversion error
  """
  if hthFile:
    ifp = file(hthFile, 'r')
  else:
    hthFile = '<stdin>'
    ifp = sys.stdin
  if scrFile:
    ofp = file(scrFile, 'w')
  else:
    scrFile = '<stdout>'
    ofp = sys.stdout

  PrintHeader(ofp, hthFile, scrFile, desc)

  # echo out motion intentions
  if desc:
    ofp.write('echo %s\n\n' % desc)

  # get pass hth header
  line = ifp.readline()

  # convert
  if motion is None:
    return Cvt2PosSetScript(ifp, ofp)
  else:
    return Cvt2PgmMotionScript(ifp, ofp, motion)

#--
if __name__ == '__main__':
  import getopt
   
  _Argv0   = __file__
   
  #--
  class Usage(Exception):
    """ Command-Line Options Usage Exception Class. """
    def __init__(self, msg):
      self.msg = msg
   
  #--
  def PrintUsageErr(emsg):
    """ Print Error Usage Message. """
    if emsg:
      print "%s: %s" % (_Argv0, emsg)
    else:
      print "%s: error" % (_Argv0)
    print "Try '%s --help' for more information." % (_Argv0)
   
  #--
  def PrintUsage():
    """ Print Command-Line Usage Message """
    print "usage: %s [options]..." % (_Argv0)
    print """Options and arguments:
    -c, --csv <file>     : Heart2Heart comma separated values file to convert.
    -s, --script <file>  : Fusion KHR1 converted output script file.
    -d, --desc <string>  : Description string.
    -m, --motion <num>   : Motion number. If specified, output motion 
                           programming script rather than position setting
                           script.
  
    -h, --help           : Display this help and exit.
    """
     
  #--
  def main():
    """ Main. """
    global _Argv0
   
    argv = sys.argv

    h2hFile = None
    scrFile = None
    desc    = None
    motion  = None
   
    try:
      try:
        opts, args = getopt.getopt(argv[1:], "?hs:c:d:m:",
                        ['help', 'script=', 'csv=', 'desc=', 'motion='])
      except getopt.error, msg:
        raise Usage(msg)
      for opt, optarg in opts:
        if opt in ('-h', '--help', '-?'):
          PrintUsage()
          return 0
        elif opt in ('-c', '--csv'):
          hthFile = optarg
        elif opt in ('-s', '--script'):
          scrFile = optarg
        elif opt in ('-d', '--desc'):
          desc = optarg
        elif opt in ('-m', '--motion'):
          motion = int(optarg)
    except Usage, err:
      PrintUsageErr(err.msg)
      return 2
   
    return HTH2Scr(hthFile, scrFile, desc, motion)
   
  # run command
  sys.exit(main())
