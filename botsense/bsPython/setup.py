#!/usr/bin/env python3

""" 
BotSense setup.py script.
"""

## \file 
##
## $LastChangedDate: 2010-09-25 09:06:47 -0600 (Sat, 25 Sep 2010) $
## $Rev: 605 $
##
## \brief BotSense Setup Script.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2010-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##

import os
import sys
from distutils.core import setup, Extension

## RN Package Root Directory (not python package)
pkgroot = '..'

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
  'name':           'BotSense',
  'version':        '3.0.1',
  'description':    'RoadNarrows Robotics BotSense Python Package',
  'long_description':"""
The BotSense python package provides the core client modules plus the
standard proxied device plug-ins.
""",
  'author':       'Robin Knight',
  'author_email': 'robin.knight@roadnarrows.com',
  'maintainer':   'RoadNarrows LLC',
  'url':          'http://www.roadnarrows.com/',
  'platforms':    "any",
  'license':"""
This is free software; see the source for copying conditions. There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Copyright (C) 2010 RoadNarrows LLC
""",
  'packages':     ['BotSense'],
  'package_dir':  {'BotSense':'modules/BotSense'},
  'package_data': {'BotSense':['_BotSenseCore.*', '_bs*.*']},
  'scripts':      [],
}

#
## List of XML files
#
xmlfiles = [
  pkgroot+'/libbotsense/bsProxyMsgs.xml',
	pkgroot+'/bsModules/bsI2C/bsI2CMsgs.xml',
	pkgroot+'/bsModules/bsNull/bsNullMsgs.xml',
	pkgroot+'/bsModules/bsSerial/bsSerialMsgs.xml',
]


## run
if __name__ == "__main__":
  setup(**PkgInfo)
