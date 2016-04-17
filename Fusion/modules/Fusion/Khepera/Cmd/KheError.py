################################################################################
#
# KheError.py
#

""" Khepera Error Handling Module

Khepera error class and data.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.18

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

import re

#-------------------------------------------------------------------------------
# Public Interface
#-------------------------------------------------------------------------------

#
# Data
#

#
# Khepera Error Class
#
class KheError:
  """ Khepera Error Class
  """

  def __init__(self):
    """ Initialize Khepera error object.
    """
    self.mErrStr = ''
    self._mReKheErrRsp = re.compile('.*<(E[0-9A-F][0-9A-F])(: )*(.*)>');

  def GetErrStr(self):
    """ Get the current error message.

        Return Value:
          Error string
    """
    return self.mErrStr

  def ClearErrStr(self):
    """ Clear any current error message.

        Return Value:
          None
    """
    self.mErrStr = ''

  def SetErrGeneral(self, errMsg):
    """ Set general error message.

       Parameters:
         errMsg - error message string
        
        Return Value:
          None
    """
    self.mErrStr = errMsg
    return None

  def SetErrBadRsp(self, errRsp, errMsg=None):
    """ Set bad response error message.

       Parameters:
         errrsp - errored response string
        
        Return Value:
          None
    """
    self.mErrStr = "Bad response: "
    if errMsg: self.mErrStr += errMsg + ": "
    self.mErrStr += repr(errRsp) 
    return None


  def SetErrNoRsp(self):
    """ Set no response error message.

        Return Value:
          None
    """
    self.mErrStr = 'No response'
    return None

  def SetErrRspTimeout(self):
    """ Set response timeout error message.

        Return Value:
          None
    """
    self.mErrStr = 'Response timed out'
    return None

  def SetErrBadParam(self, paramName, paramErr, paramVal=None):
    """ Set bad input parameter value error message.

       Parameters:
         paramName  - parameter name
         errval - invalid value
        
        Return Value:
          None
    """
    self.mErrStr = "Bad input parameter '%s': %s" % (paramName, paramErr)
    if paramVal:
      self.mErrStr += ': ' + repr(paramVal)
    return None
