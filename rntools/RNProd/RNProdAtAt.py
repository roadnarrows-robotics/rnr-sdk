###############################################################################
#
# RNProdAtAt.py
#

""" RoadNarrows Product Make Tools - Process '@@' identifiers in template files.

The RNProdAtAt module provides functions to scan and process '@indentifier@'
with the respective value(s).

Author: Robin D. Knight
Email:  robin.knight@roadnarrows.com
URL:    http://www.roadnarrows.com

Copyright (C) 2009.  RoadNarrows LLC.
All Rights Reserved

$LastChangedDate: 2009-09-04 12:09:46 -0600 (Fri, 04 Sep 2009) $
$Rev: 118 $
"""

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

import os
import time
import shutil
import re

_reAtAt = re.compile('@([a-zA-Z_]\w*)@|@([a-zA-Z_]\w*):(.+)@')
_Now    = time.localtime()

AtAtBuiltIns = {
  'THIS_YEAR':    repr(_Now.tm_year),
  'THIS_DATE':    "%d.%02d.%02d" % (_Now.tm_year, _Now.tm_mon, _Now.tm_mday),
  'THIS_TIME':    "%02d:%02d:%02d" % (_Now.tm_hour, _Now.tm_min, _Now.tm_sec),
}

#--
def AtAtIdentifier(match):
  if match.lastindex == 1:
    return (match.group(1), None)
  if match.lastindex == 3:
    return (match.group(2), match.group(3))
  else:
    return (None, None)

#--
def AtAtWriteValAtomic(fp, val, fmt):
  if fmt:
    fp.write(fmt % (val))
  elif type(val) == str:
    fp.write(val)
  else:
    fp.write(repr(val))

#--
def AtAtWriteValList(fp, valList, fmt):
  for val in valList:
    AtAtWriteValAtomic(fp, val, fmt)
    if not fmt:
      fp.write(' ')

#--
def AtAtWriteVal(fp, id, valDict, fmt=None):
  val = valDict[id]
  if val is None:
    pass
  elif callable(val):
    val(fp, id, valDict, fmt)
  elif type(val) == list:
    AtAtWriteValList(fp, val, fmt)
  else:
    AtAtWriteValAtomic(fp, val, fmt)

#--
def AtAtReplace(fileNameIn, userDict, bReplace=True):
  fileNameTmp = fileNameIn + '.tmp'
  fpSrc = open(fileNameIn, 'r')
  fpTmp = open(fileNameTmp, 'w')
  lineNum = 1
  line = fpSrc.readline()
  while line:
    #print "[%d] %s" % (lineNum, line),
    m = 0
    for match in _reAtAt.finditer(line):
      #print match.group(0), match.start(), match.end()
      fpTmp.write(line[m:match.start()])
      id,fmt = AtAtIdentifier(match)
      if id in userDict:
        AtAtWriteVal(fpTmp, id, userDict, fmt)
      elif id in AtAtBuiltIns:
        AtAtWriteVal(fpTmp, id, AtAtBuiltIns, fmt)
      else:
        AtAtWarning(fileNameIn, lineNum, match.start(), id, 
            "unknown identifier")
        fpTmp.write(match.group(0))
      m = match.end()
    if m < len(line):
      fpTmp.write(line[m:])
    lineNum += 1
    line = fpSrc.readline()
  fpSrc.close()
  fpTmp.close()
  if bReplace:
    os.rename(fileNameTmp, fileNameIn)

#--
def AtAtWarning(fileName, lineNum, colNum, *args):
  """ Print AtAt warning.

      Parameters:
        *args   - List of warning message arguments.
  """
  wmsg = "Warning: %s[%d,%d]" % (fileName, lineNum, colNum)
  for a in args:
    wmsg += ": %s" %(a)
  print wmsg


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  import RNProdParam as Param

  def pkg_info_iter(fp, id, prodDict, fmt):
    _icnt = 0
    for _ivar in prodDict['pkg_info']:
      s = eval(fmt)
      fp.write(s)
      _icnt += 1

  def pkg_list_iter(fp, id, prodDict, fmt):
    _icnt = 0
    for _ivar in prodDict['pkg_list']:
      s = eval(fmt)
      fp.write(s)
      _icnt += 1

  def main():
    """ Unit Test Main """
    testdict = {
        'prod_name':    'foo',
        'prod_fqname':  'foo-1.2.3',
        'pkg_list':     ['/prj/pkg/librnr', '/prj/pkg/RoboSight'],
        'pkg_info':     [ { 'package': 'librnr',
                            'synopsis': 'RNR Library 1',
                            'seealso': 'source documentation',
                            'description':
'The librnr library provides a handy set of tools to quickly\n' + \
'develop very cool rn applications.'
                          },
                          { 'package': 'RoboSight',
                            'synopsis': 'RNIS cools stuff',
                            'description': 'blah, blah\nblah'
                          },
                        ],
        'pkg_info_iter': pkg_info_iter,
        'pkg_list_iter': pkg_list_iter,

    }
    fileOrig  = Param.RNProdFqDirTemplates+ '/' + Param.RNProdTemplateReadMe
    fileCopy  = 'tests/atattest.txt'
    shutil.copyfile(fileOrig, fileCopy)
    AtAtReplace(fileCopy, testdict, bReplace=False)

  # run unit test
  main()
