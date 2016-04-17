#!/usr/bin/env python

from distutils.core import setup
from sys import version_info

# RN Package Root Directory (not python package)
pkgroot = '.'

# source locations relative setup.py
imgdir = 'modules/Fusion/Gui/Images/'

# destination locations relative to prefix
sitepkg = 'lib/python'+repr(version_info[0])+'.'+repr(version_info[1])+'/site-packages/'

#
# Package pydoc additional information (required by rnmake utilities)
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
# Package Information (required by setup and rnmake utilities)
#
PkgInfo = {
  'name':           'Fusion',
  'version':        '0.15.0',
  'description':    'RoadNarrows Fusion Python Package',
  'long_description':"""
Fusion is a RoadNarrows-developed robotics demonstration application.
""",
  'author':       'Robin Knight',
  'author_email': 'robin.knight@roadnarrows.com',
  'maintainer':   'RoadNarrows LLC',
  'url':          'http://www.roadnarrows.com/',
  'platforms':    "any",
  'license':"""
This is free software; see the source for copying conditions. There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
Copyright (C) 2005-2010 RoadNarrows LLC
""",
  'packages':   [
    'Fusion',
    'Fusion.Acc',
    'Fusion.Core',
    'Fusion.Gui',
    'Fusion.Hemisson',
    'Fusion.Hemisson.Brains',
    'Fusion.Hemisson.Cmd',
    'Fusion.Hemisson.Gui',
    'Fusion.Hemisson.Robots',
    'Fusion.Hemisson.Shells',
    'Fusion.Hemisson.Utils',
    'Fusion.Khepera',
    'Fusion.Khepera.Brains',
    'Fusion.Khepera.Cmd',
    'Fusion.Khepera.Gui',
    'Fusion.Khepera.Robots',
    'Fusion.Khepera.Shells',
    'Fusion.Khepera.Utils',
    'Fusion.KHR1',
    'Fusion.KHR1.Brains',
    'Fusion.KHR1.Cmd',
    'Fusion.KHR1.Gui',
    'Fusion.KHR1.Robots',
    'Fusion.KHR1.Shells',
    'Fusion.KHR1.Utils',
    'Fusion.KHR2',
    'Fusion.KHR2.Brains',
    'Fusion.KHR2.Cmd',
    'Fusion.KHR2.Gui',
    'Fusion.KHR2.Robots',
    'Fusion.KHR2.Shells',
    'Fusion.KHR2.Utils',
    'Fusion.Utils',
  ],
  'package_dir':{'Fusion':'modules/Fusion'},
  'scripts':    ['scripts/Fusion', 'scripts/Fusion.bat'],
  'data_files': [(sitepkg+'Fusion/Gui/Images', [
    imgdir+'BlockyDisabled.gif',        imgdir+'FusionBat0.gif',
    imgdir+'FusionNav0.gif',            imgdir+'RalfWalking3.gif',
    imgdir+'BlockyErrored.gif',         imgdir+'FusionBat1.gif',
    imgdir+'FusionNav1.gif',            imgdir+'RalfWalkingMan.gif',
    imgdir+'Blocky_Error.gif',          imgdir+'FusionBat2.gif',
    imgdir+'FusionNav2.gif',            imgdir+'RNRHemisson.gif',
    imgdir+'Blocky_Info.gif',           imgdir+'FusionBat3.gif',
    imgdir+'FusionNav3.gif',            imgdir+'RNRKhepera.gif',
    imgdir+'BlockyNotLoaded.gif',       imgdir+'FusionBatMan.gif',
    imgdir+'FusionNav4.gif',            imgdir+'RNRKHR-1.gif',
    imgdir+'BlockyStopped.gif',         imgdir+'FusionCamera0.gif',
    imgdir+'FusionNavMan.gif',          imgdir+'RNRLogo.gif',
    imgdir+'BlockyWalk0.gif',           imgdir+'FusionCamera1.gif',
    imgdir+'Robot.gif',                 imgdir+'BlockyWalk1.gif',
    imgdir+'FusionCamera2.gif',         imgdir+'SerConn.gif',
    imgdir+'BlockyWalk2.gif',           imgdir+'FusionCameraMan.gif',
    imgdir+'Load.gif',                  imgdir+'SerDisc.gif',
    imgdir+'BlockyWalk3.gif',           imgdir+'FusionMap0.gif',
    imgdir+'Pause.gif',                 imgdir+'Step.gif',
    imgdir+'Blocky_Warning.gif',        imgdir+'FusionMap1.gif',
    imgdir+'Play.gif',                  imgdir+'Stop.gif',
    imgdir+'Brain.gif',                 imgdir+'FusionMap2.gif',
    imgdir+'RalfWalking0.gif',          imgdir+'Eject.gif',
    imgdir+'FusionMap3.gif',            imgdir+'RalfWalking1.gif',
    imgdir+'EStop.gif',                 imgdir+'FusionMapMan.gif',
    imgdir+'RalfWalking2.gif',
  ]),
  ]
}

if __name__ == "__main__":
  setup(**PkgInfo)
