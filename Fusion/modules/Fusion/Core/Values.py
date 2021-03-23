################################################################################
#
# Values.py
#

""" Core values shared by Fusion Applications

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.01.03

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

import os
import stat
import sys
import shutil

from Fusion.VERSION import FusionVersion
import Fusion.Utils.Tools as utils

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Static Values
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

if os.name == 'posix':
  FusionPlatform = 'Posix'
elif os.name == 'nt':
  FusionPlatform = 'Windows'
else:
  raise RuntimeError("Fusion is only defined for 'posix' and 'nt' platforms")

# The Python Package
FusionPkgName = 'Fusion'

# Environment Variables
FusionEnvVarFusion      = '$FUSION'
FusionEnvVarStartup     = '$FUSIONSTARTUP'

# Fusion Base Files
FusionIniBaseName = 'fusion.ini'
if FusionPlatform == 'Posix':
  FusionUserIniBaseName   = '.fusion'
else:
  FusionUserIniBaseName   = 'fusion.ini'

# Fusion Subdirectories
FusionSubDirImages  = 'Gui' + os.sep + 'Images'


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Run-Time Determinations
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

# Environment Variable Values
FusionEnvValFusion    = None
FusionEnvValStartup   = None

# Fusion Package Absolute Directory Names
FusionPkgDir          = None
FusionImageDir        = None

# Fusion Fully Qualified File Names
FusionIniFileName     = None
FusionUserIniFileName = None


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GUI Child Window Core Keyword Options
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

FusionCWinKeyOnDestroy    = 'ondestroy'   # inform parent on-destroy callback
FusionCWinKeyWinId        = 'winid'       # unique child window id string
FusionCWinKeyGeometry     = 'geometry'    # ending geometry 'wxh+x+y'
FusionCWinKeyTitle        = 'title'       # title of window
FusionCWinKeyCreateServer = 'create_server' # create window request server
FusionCWinKeyQSize        = 'qsize'       # window server queue size


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

#--
def _getEnvVars():
  """ Get environment variable values. """
  global FusionEnvValFusion
  global FusionEnvValStartup
  FusionEnvValFusion  = os.getenv(FusionEnvVarFusion[1:])
  FusionEnvValStartup = os.getenv(FusionEnvVarStartup[1:])

#--
def _getFusionPkgDir():
  """ Return verified Fusion package base directory. """
  # try getting package directory from package attributes
  pkgdir = utils.getpkgdir(FusionPkgName)
  # failed, so try environmental variable
  if not pkgdir:
    pkgdir = FusionEnvVal
  # failed again to get package directory name
  if not pkgdir:
    print(
      'Values error: cannot find %s package base directory' % FusionPkgName,
      file=sys.stderr)
    print('Try setting environment variable %s' % FusionEnvVar, file=sys.stderr)
    return None
  # test directory name
  try:
    mode = os.stat(pkgdir)[stat.ST_MODE]
  except OSError as msg:
    print('Values error: ', msg, file=sys.stderr)
    print(
    'Environment variable %s must point to the %s package base directory' % \
        (FusionEnvVar, FusionPkgName),
        file=sys.stderr)
    return None
  if stat.S_ISDIR(mode):
    return pkgdir
  else:
    print('Values error: %s: not a directory' % pkgdir, file=sys.stderr)
    print(
    'Environment variable %s must point to the %s package base directory' % \
        (FusionEnvVar, FusionPkgName),
        file=sys.stderr)
    return None

#--
def _getFusionPkgSubDir(subdir):
  """ Return verified Fusion package subdirectory. """
  if not FusionPkgDir or not subdir:
    return
  pkgsubdir = os.path.join(FusionPkgDir, subdir)
  try:
    mode = os.stat(pkgsubdir)[stat.ST_MODE]
  except OSError as msg:
    print('Values error: ', msg, file=sys.stderr)
    print(
    '%s package scrogged? Subdirectory %s should exist under %s' % \
        (FusionPkgName, subdir, FusionPkgDir),
        file=sys.stderr)
    return None
  if stat.S_ISDIR(mode):
    return pkgsubdir
  else:
    print('Values error: %s: not a directory' % subdir, file=sys.stderr)
    print(
    '%s package scrogged? Subdirectory %s should exist under %s' % \
        (FusionPkgName, subdir, FusionPkgDir),
        file=sys.stderr)
    return None

#--
def _setFusionIni():
  """ Set Fusion Ini file for specific platform. """
  try:
    os.stat(FusionIniFileName)
    return
  except (OSError, IOError):
    pass
  if FusionPlatform == 'Posix':
    ext = '.posix'
  else:
    ext = '.win'
  try:
    shutil.copy2(FusionIniFileName+ext, FusionIniFileName)
  except IOError as msg:
    print("Cannot set Fusion ini file: %s" % msg, file=sys.stderr)

#--
def _listValues():
  varList = [
    'FusionPkgName',
    'FusionVersion',
    'FusionPlatform',
    'FusionEnvVarFusion',
    'FusionEnvVarStartup',
    'FusionIniBaseName',
    'FusionUserIniBaseName',
    'FusionSubDirImages',
    'FusionEnvValFusion',
    'FusionEnvValStartup',
    'FusionPkgDir',
    'FusionImageDir',
    'FusionIniFileName',
    'FusionUserIniFileName',
  ]

  envList = [
    FusionEnvVarFusion,
    FusionEnvVarStartup,
    '$HOME'
  ]

  vout = []
  for var in varList:
    vout.append('%s=%s' % (var, repr(eval(var))))

  eout = []
  for var in envList:
    eout.append('%s=%s' % (var, os.getenv(var[1:])))

  return vout, eout


_Once = 0

#--
def InitOnce():
  """ One time, run-time determinations. """
  global _Once
  global FusionEnvValFusion, FusionPkgDir, FusionEnvVarFusion, FusionImageDir
  global FusionIniFileName, FusionPlatform, FusionUserIniFileName 
  if _Once:
    return
  _Once = 1
  _getEnvVars()
  FusionPkgDir    = _getFusionPkgDir()
  if not FusionEnvValFusion:
    FusionEnvValFusion = FusionPkgDir
    #os.putenv(FusionEnvVarFusion[1:], FusionEnvValFusion)   # doesn't work
    os.environ[FusionEnvVarFusion[1:]] = FusionEnvValFusion  # but this does
  FusionImageDir  = _getFusionPkgSubDir(FusionSubDirImages)
  FusionIniFileName = FusionPkgDir + os.sep + FusionIniBaseName
  if FusionPlatform == 'Posix':
    home = os.getenv('HOME')
    FusionUserIniFileName = home + os.sep + FusionUserIniBaseName
  else:
    homedrive = os.getenv('HOMEDRIVE')
    homepath  = os.getenv('HOMEPATH')
    home = homedrive + homepath
    FusionUserIniFileName = home + os.sep + FusionUserIniBaseName
    #os.putenv('HOME', home)  # this doesn't work
    os.environ['HOME'] = home # but this does
  _setFusionIni()



#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  nvList, evList = _listValues()
  print('Fusion Variables')
  for v in nvList:
    print(v)
  print('Environment Variables')
  for v in evList:
    print(v)
