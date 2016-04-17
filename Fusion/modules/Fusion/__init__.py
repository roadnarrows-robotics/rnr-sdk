################################################################################
#
# __init__.py
#

""" Fusion Package

Fusion is a RoadNarrows LLC robotics demonstration package.

Public Subpackages:
  Core      - Fusion core subpackage including the Reactor.
  Utils     - Fusion general purpose utilities subpackage.
  Gui       - Fusion Graphical User Interface windows, dialogs,
              and utilities.
  Hemisson  - K-Team Hemisson robot subpackage.
  Khepera   - K-Team KheperaII robot subpackage.
  KHR1      - Kondo KHR-1 humanoid robot subpackage.
  Templates - User development templates.
  Demos     - Simple Fusion demos.

Author:   Robin D. Knight
Email:    robin.knight@roadnarrowsrobotics.com
URL:      http://www.roadnarrowsrobotics.com
Created:  2005.12.04
Modified: 2007.01.08

Copyright (C) 2004, 2005, 2006, 2007.  RoadNarrows LLC.
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

from Fusion.VERSION import FusionVersion

__version__ = FusionVersion
__author__  = 'Robin D. Knight'
__date__    = '2007.01.08'

__all__ = [
  'Core',
  'Gui',
  'Hemisson',
  'Khepera',
  'KHR1',
  'KHR2',
  'Utils',
  'Templates',
  'Demos'
]
