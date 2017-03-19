#!/usr/bin/env python

""" 
Hekateros python modules setup.py script.
"""

## \file 
##
## $LastChangedDate: 2016-03-18 12:24:39 -0600 (Fri, 18 Mar 2016) $
## $Rev: 4360 $
##
## \brief Hekateros Python Setup Script.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2011-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##
# @EulaBegin@
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
  'name':           'Hekateros',
  'version':        '1.0.0',
  'description':    'RoadNarrows Robotics Hekateros Python Package',
  'long_description':"""
The Hekateros python package provides modules and plug-in
extensions for the Hekateros robotic manipulator.
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
  'packages':     ['Hekateros'],
  'package_dir':  {'Hekateros':'modules/Hekateros',
                   'Hekateros.images':'modules/Hekateros/images'},
  'package_data': {'Hekateros':['_bs*.*'],
                   'Laelaps.images':['*.png', '*.jpg']},
  'scripts':      [],
}

#
## List of XML files
#
xmlfiles = [
	pkgroot+'/bsModules/bsHek/bsHekMsgs.xml',
]


## run
if __name__ == "__main__":
  setup(**PkgInfo)
