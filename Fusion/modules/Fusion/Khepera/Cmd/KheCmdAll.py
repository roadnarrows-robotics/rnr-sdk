################################################################################
#
# KheCmdAll.py
#

""" All Khepera II Commands Module

Command/response interface for all Khepera II commands.
Command sets supported:
  Khepera Base set

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.31

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

import Fusion.Khepera.Cmd.KheSerial as KheSerial
import Fusion.Khepera.Cmd.KheCmdBase as KheCmdBase

#-------------------------------------------------------------------------------
# Public Interface
#-------------------------------------------------------------------------------

#
# Khepera All Commands Class
#
class KheCmdAll(KheSerial.KheSerial, 
                KheCmdBase.KheCmdBase):
  """ Khepera All Command and Response Class
  """

  def __init__(self, port=None, baudrate=9600, dbgobj=None):
    """ Initialize a Khepera serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the Khepera
        serial port object will be in the closed state.

        Parameters:
          port      - serial port (default: no port)
          baudrate  - baudrate (default: 9600)
          dbgobj    - PyDebug object. None will create the object.
    """
    KheSerial.KheSerial.__init__(self, port, baudrate, dbgobj)

