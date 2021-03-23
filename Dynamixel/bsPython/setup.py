#!/usr/bin/env python3

""" 
BotSense setup.py script.
"""

## \file 
##
## $LastChangedDate: 2011-06-22 08:41:22 -0600 (Wed, 22 Jun 2011) $
## $Rev: 1061 $
##
## \brief BotSense Dynamixel Setup Script.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2011-2017. RoadNarrows LLC.\n
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
  'name':           'BotSense.Dynamixel',
  'version':        '1.0.0',
  'description':    'RoadNarrows Robotics BotSense Python Package Add-On',
  'long_description':"""
The BotSense Khepera III python package provides the BotSense plug-in
extensions for the BotSense python package.
""",
  'author':       'Robin Knight',
  'author_email': 'robin.knight@roadnarrows.com',
  'maintainer':   'RoadNarrows LLC',
  'url':          'http://www.roadnarrows.com/',
  'platforms':    "any",
  'license':"""
This is free software; see the source for copying conditions. There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Copyright (C) 2011 RoadNarrows LLC
""",
  'packages':     ['BotSense.Dynamixel'],
  'package_dir':  {'BotSense.Dynamixel':'modules/BotSense/Dynamixel'},
  'package_data': {'BotSense.Dynamixel':['_bs*.*']},
  'scripts':      [],
}

#
## List of XML files
#
xmlfiles = [
	pkgroot+'/bsModules/bsDyna/bsDynaMsgs.xml',
]


## run
if __name__ == "__main__":
  setup(**PkgInfo)
