#!/usr/bin/env python

""" 
Laelaps python modules setup.py script.
"""

## \file 
##
## $LastChangedDate: 2016-03-18 12:47:57 -0600 (Fri, 18 Mar 2016) $
## $Rev: 4362 $
##
## \brief Laelaps Python Setup Script.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##
# @EulaBegin@
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
# @EulaEnd@

import os
import sys
from distutils.core import setup, Extension

## RN Package Root Directory (not python package)
pkgroot = '../..'

#
## Package pydoc additional information (required by rnmake utilities)
#
PyDocInfo = {
  'org_initials':   'RNR',
  'index_template': pkgroot+"/docs/pydoc.html.tpl",
  'images_dir':     pkgroot+"/docs/images",
  'images': {
    'ORG_LOGO': 'Logo.png',
    'FAVICON':  'favicon.png',
  },
}

#
## Package Information (required by setup and rnmake utilities)
#
PkgInfo = {
  'name':           'Laelaps',
  'version':        '1.0.0',
  'description':    'RoadNarrows Laelaps Python Package',
  'long_description':"""
The Laelaps python package provides modules and plug-in extension for the
Laelaps mobile robotic platform.
""",
  'author':       'Robin Knight',
  'author_email': 'robin.knight@roadnarrows.com',
  'maintainer':   'RoadNarrows LLC',
  'url':          'http://www.roadnarrows.com/',
  'platforms':    "any",
  'license':"""
This is free python modules and binary extensions software; see the source for
copying conditions. There is NO warranty; not even for MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.
Copyright (C) 2011-2013 RoadNarrows LLC
""",
  'packages':     ['Laelaps', 'Laelaps.images'],
  'package_dir':  {'Laelaps':'modules/Laelaps',
                   'Laelaps.images':'modules/Laelaps/images'},
  'package_data': {'Laelaps':['_ImuMspMsgs.*', '_RoboClawMsgs.*',
                              '_WatchDogMsgs.*'],
                    'Laelaps.images':['*.png', '*.jpg']},
  'scripts':      [],
}

#
## List of XML files
#
xmlfiles = [ ]


## run
if __name__ == "__main__":
  setup(**PkgInfo)
