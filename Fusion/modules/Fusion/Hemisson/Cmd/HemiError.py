################################################################################
#
# HemiError.py
#

""" Hemisson Error Handling Module

Hemisson error class and data.

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

# Error Code Dictionary
HemiErrDict = {
  "E01": "Unknown command",
  "E02": "Invalid command syntax",
  "E03": "Hemisson buffer overflow",
  "E11": "Invalid module one-character identifier",
  "E12": "Module is not connected",
  "E13": "Module support is not enabled in this RN HemiOS version",
  "E14": "Module is busy",
  "E15": "Invalid command length",
  "E16": "Command element is out-of-range",
  "E17": "Unknown module command"
}

#-------------------------------------------------------------------------------
# CLASS: HemiError
#-------------------------------------------------------------------------------
class HemiError:
  """ Hemisson Error Class
  """

  #--
  def __init__(self):
    """ Initialize Hemisson error object.
    """
    self.mErrStr = ''
    self._mReHemiErrRsp = re.compile('.*<(E[0-9A-F][0-9A-F])(: )*(.*)>');

  #--
  def GetErrStr(self):
    """ Get the current error message.

        Return Value:
          Error string
    """
    return self.mErrStr

  #--
  def ClearErrStr(self):
    """ Clear error string.
      
        Return Value:
          None
    """
    self.mErrStr = ''

  #--
  def GetHemissonErr(self, rsp):
    """ Parse potential Hemisson error message from response.

       Parameters:
         rsp - response string
        
        Return Value:
          If error message found, return parsed error string, else
          return empty string.
    """
    hemierr = ''
    if rsp:
      match = self._mReHemiErrRsp.match(rsp)
      if match:
        ec   = match.group(1)
        elem = match.group(3)
        hemierr = "Hemisson error: "
        if ec in HemiErrDict:
          hemierr += HemiErrDict[ec]
        else:
          hemierr += ec + ": unknown error code"
        if elem:
          hemierr += ": " + elem
    return hemierr

  #--
  def SetErrGeneral(self, errmsg):
    """ Set general error message.

       Parameters:
         errmsg - error message string
        
        Return Value:
          None
    """
    self.mErrStr = errmsg
    return None

  #--
  def SetErrBadRsp(self, errrsp):
    """ Set bad response error message.

       Parameters:
         errrsp - errored response string
        
        Return Value:
          None
    """
    if errrsp:
      errstr = self.GetHemissonErr(errrsp)
      if errstr:
        self.mErrStr = errstr
      else:
        self.mErrStr = 'Bad response: ' + repr(errrsp) 
    else:
      self.SetErrNoRsp()
    return None


  #--
  def SetErrNoRsp(self):
    """ Set no response error message.

        Return Value:
          None
    """
    self.mErrStr = 'No response'
    return None

  #--
  def SetErrRspTimeout(self):
    """ Set response timeout error message.

        Return Value:
          None
    """
    self.mErrStr = 'Response timed out'
    return None

  #--
  def SetErrBadParam(self, param, errval):
    """ Set bad input parameter value error message.

       Parameters:
         param  - parameter name
         errval - invalid value
        
        Return Value:
          None
    """
    self.mErrStr = 'Bad input parameter: ' + param + '=' + repr(errval)
    return None
