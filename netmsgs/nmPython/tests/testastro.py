#
# File: testastro.py
#
# Test python generation and run-time operation from the AstroMsgs.xml
# specification.
#
# Copyright:
#   (C) 2010.  RoadNarrows LLC.
#   (http://www.roadnarrows.com)
#   All Rights Reserved
#

import sys
import os


pkgroot   = '/prj/pkg/netmsgs'
pymodule  = 'AstroMsgs'
pyfile    = "%s/nmPython/tests/%s.py" % (pkgroot, pymodule)
xmlfile   = "%s/examples/%s.xml" % (pkgroot, pymodule)
gencmd    = "%s/nmPython/scripts/netmsgsgen" % (pkgroot)
sys.path  = [pkgroot+'/nmPython/tests', pkgroot+'/nmPython/modules'] + sys.path
cmd       = "%s --lang=python --xml=%s %s" % (gencmd, xmlfile, pyfile)

rc = os.system(cmd)
if rc != 0:
  sys.exit(rc)

import NetMsgs.NetMsgsBase as nmBase
from AstroMsgs import *

nm = AstroNetMsgs(trace=True)

tvals = nm.MakeMsgTemplate(AstroMsgId.CmdUniverse)

tvals['Truth'] = False
tvals['Aura'] = 'j'
tvals['Gluon'] = 5
tvals['Electron'] = -5
tvals['Planet'] = 505
tvals['StellarObj'] = -505
tvals['StarSystem'] = 500050
tvals['Galaxy'] = -99
tvals['GalaxyGroup'] = 2300400
tvals['Filament'] = -2300400
tvals['HiddenDim'] = 0xfacec01f
tvals['ParallelUniv'] = 0xdeadd00d
tvals['VisUnivLY'] = 13.7e6
tvals['FullUnivLY'] = 25.0e6
tvals['TheUnamed'] = "Chesire Cat"
tvals['Math']['One'] = 1
tvals['Math']['Two'] = 2
tvals['Math']['DotDotDot'] = 0xffffffff
tvals['Physics'] = [100, 1010, 101001]
tvals['StarType'] = 'G'
tvals['StarInfo']['Color'] = "redorange"
tvals['StarInfo']['MassSun'] = 2.24245
tvals['StarInfo']['Name'] = "RedMeat"
tvals['StarInfo']['TempK'] = 3500
tvals['StarInfo']['Type'] = 'M'

nmBase.PrettyPrintAssignExpr('tvals', tvals)

try:
  buf = nm.nmPackMsg(AstroMsgId.CmdUniverse, tvals)
except nmBase.NetMsgsError, inst:
  print >>sys.stderr, "Error:", inst
  sys.exit(8)

rvals = {}

try:
  n = nm.nmUnpackMsg(AstroMsgId.CmdUniverse, buf, rvals)
except nmBase.NetMsgsError, inst:
  print >>sys.stderr, "Error:", inst
  sys.exit(8)

print >>sys.stderr
nmBase.PrettyPrintAssignExpr('rvals', rvals)

def lrcmp(L, R):
  eq = True
  for k in L.iterkeys():
    if type(L[k]) == dict:
      if not lrcmp(L[k], R[k]):
        print >>sys.stderr, "\nL[%s] != R[%s]" % (k, k)
        eq = False
    elif L[k] != R[k]:
      print >>sys.stderr, "\nL[%s]=%s !=\nR[%s]=%s" % (k, L[k], k, R[k])
      eq = False
  return eq

print >>sys.stderr, """
Comparing pre-packed(L) and post-unpacked(R) values. All should be equal except
for small rounding errros that occur for Floating-Point Numbers.
"""
if lrcmp(tvals, rvals):
  print >>sys.stderr, "\ntvals == rvals"
else:
  print >>sys.stderr, "\ntvals != rvals"

sys.exit(0)
