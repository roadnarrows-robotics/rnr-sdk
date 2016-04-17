################################################################################
#
# __plugins__.py
#

""" Fusion Plugin List for vRobots and vBrains.

Lists of Fusion plugin types for vRobots and vBrains. For each
'RobotPluginPath' and 'BrainPluginPath' directory compenent specified
in the 'ini' configuration file(s), there should exist a __plugins__.py
file to ease Fusion attaching.

For vRobots, add a path to RobotPluginPath in one of the ini files
for Fusion to automatically search that path. Likewise for vBrains,
add a path to BrainPluginPath.

vRobots Plugin Type.
  Each vRobots[] entry has the following fields:
    filename        - python file name relative to this directory
    entrypoint      - vRobot class entry point (symbol)
    description     - description of vRobot

vBrains Plugin Type.
  Each vBrains[] entry has the following fields:
    filename        - python file name relative to this directory
    entrypoint      - vBrain class entry point (symbol)
    description     - description of vBrain

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.07

Copyright (C) 2006-2008.  RoadNarrows LLC.
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

# vRobots Plugin Type
vRobots = [
  ['Robots/vHemisson.py', 'vHemisson', 
   'Hemisson robot plus any attached modules.']
]

# vBrains Plugin Type
vBrains = [
  ['Brains/vBrainOpticFlow.py', 'vBrainOpticFlow', 
   'Optical flow brain with linear camera.'],
  ['Brains/vBrainDemoPhobia.py', 'vBrainDemoPhobia', 
   'Demophobic behavior utilizing the USS and TTS modules.'],
  ['Brains/vBrainyBerg.py', 'vBrainyBerg', 
   'Braitenberg behaviors utilizing the linear camera and TTS modules.'],
]
