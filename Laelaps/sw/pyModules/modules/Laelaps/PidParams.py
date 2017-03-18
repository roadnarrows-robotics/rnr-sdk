###############################################################################
#
# Module:   Laelaps.PidParams.py
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     PidParams.py
#
## \file 
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $
## $Rev: 4293 $
##
## \brief PID parameters classes.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright
##   \h_copy 2015-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
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
#
###############################################################################

import sys
import os
import time
import math

PidKDiv = float(0x010000)
PidKMin = 0.0
PidKMax = float(2**31)

# ------------------------------------------------------------------------------
# Class VelPidParams
# ------------------------------------------------------------------------------

class VelPidParams:
  def __init__(self):
    self.m_Kp       = 0.0
    self.m_Ki       = 0.0
    self.m_Kd       = 0.0
    self.m_maxQpps  = 0
    self.m_maxAccel = 0.0

  def setFromRawCtlrVals(self, Kp, Ki, Kd, maxQpps):
    self.m_Kp       = Kp/PidKDiv
    self.m_Ki       = Ki/PidKDiv
    self.m_Kd       = Kd/PidKDiv
    self.m_maxQpps  = maxQpps
