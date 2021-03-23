#!/usr/bin/env python3

""" 
NetMsgs standard setup.py script.
"""

## \file 
##
## $LastChangedDate: 2010-07-31 08:48:56 -0600 (Sat, 31 Jul 2010) $
## $Rev: 521 $
##
## \brief NetMsgs Setup Script.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2009-2017. RoadNarrows LLC.\n
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
  'name':           'NetMsgs',
  'version':        '1.2.2',
  'description':    'RoadNarrows NetMsgs Python Package',
  'long_description':"""
The NetMsgs python package provides the set of tools to generate
language-specific packing and unpacking routines, plus run-time message
processing routines. The generated output files are specified from RoadNarrows
NetMsgs XML input files.

Supported Output Language Generation:
o C <name>.h and <name>.c
o Python <name>.py
""",
  'author':       'Robin Knight',
  'author_email': 'robin.knight@roadnarrows.com',
  'maintainer':   'RoadNarrows LLC',
  'url':          'http://www.roadnarrows.com/',
  'platforms':    "any",
  'license':"""
This is free software; see the source for copying conditions. There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Copyright (C) 2009-2018 RoadNarrows LLC
""",
  'packages':     ['NetMsgs'],
  'package_dir':  {'NetMsgs':'modules/NetMsgs'},
  'package_data': {'NetMsgs':['_NetMsgsCore.*']},
  'scripts':      ['scripts/netmsgsgen'],
}

## run
if __name__ == "__main__":
  setup(**PkgInfo)
