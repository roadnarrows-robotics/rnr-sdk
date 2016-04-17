################################################################################
#
# __init__.py
#

""" Fusion Core Subpackage

This package provides the control and 'glue' for the brains and robots,
all integrated into a GUI.

Public Modules:
  Reactor         - Robotic Fusion Reactor Module. (the main man)
  Mirror          - Fusion Reactor Event Dispather
  Gluon           - The "Glue" Module between Fusion, vRobot, and 
                    vBrain Objects.
  vBrain          - Virtual Brain virtual base class defines the 
                    required brain interface.
  vBrainThreaded  - Virtual Brain threaded virtual base class defines the 
                    typical brain interface.
  vRobot          - Virtual Robot virtual base class defines the required 
                    physical/simulated robot interface.
  vRobotThreaded  - Virtual Robot virtual base class defines the typical 
                    robot interface.
  FusionIniDD     - Fusion 'ini' definition dictionary.
  Values          - Fusion package and application core values.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.04

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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

__all__ = [
  'Reactor',
  'Gluon',
  'vBrain',
  'vBrainThreaded',
  'vRobot',
  'vRobotThreaded',
  'FusionIniDD',
  'Values'
]
