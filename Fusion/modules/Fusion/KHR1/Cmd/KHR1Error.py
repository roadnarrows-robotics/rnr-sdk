################################################################################
#
# KondoError.py
#

""" Kondo KHR-1 Error Handling Module

Kondo KHR-1 error class and data.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.24

Copyright (C) 2005.  RoadNarrows LLC.
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

#-------------------------------------------------------------------------------
# Gloabal Data
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
# CLASS: KHR1Error
#-------------------------------------------------------------------------------
class KHR1Error:
  """ Kondo KHR-1 Error Class
  """

  #--
  def __init__(self):
    """ Initialize Kondo KHR-1 error object.
    """
    self.mErrStr = ''

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
  def SetErrGeneral(self, errMsg, errRsp=None):
    """ Set general error message.

        Parameters:
          errMsg - error message string
        
        Return Value:
          None
    """
    self.mErrStr = errMsg
    if errRsp != None:
      self.mErrStr += self._DataToString(errRsp)
    return None

  #--
  def SetErrBadRsp(self, errMsg, errRsp):
    """ Set bad response error message.

        Parameters:
          errRsp - errored response string
        
        Return Value:
          None
    """
    if errMsg:
      self.mErrStr = errMsg + ": "
    else:
      self.mErrStr = 'Bad response: '
    self.mErrStr += self._DataToString(errRsp)
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
  def SetErrBadChkSum(self, goodChkSum, errRsp):
    """ Set bad response checksum error message.

        Return Value:
          None
    """
    self.mErrStr = 'Bad checksum - received 0x%02x, expected 0x%02x: rsp: ' \
                      % (errRsp[-1], goodChkSum)
    self.mErrStr += self._DataToString(errRsp)
    return None

  #--
  def SetErrBadAck(self, goodAck, errRsp):
    """ Set bad acknowledgement error message.

        Return Value:
          None
    """
    self.mErrStr = 'Bad ACK - received 0x%02x, expected 0x%02x: rsp: ' \
                      % (errRsp[-1], goodAck)
    self.mErrStr += self._DataToString(errRsp)
    return None

  #--
  def SetErrRcbSync(self, errRsp):
    """ Set RCB-1 synchronization error message.

        Return Value:
          None
    """
    self.mErrStr = 'RCB-1 sync error - %s' % (errRsp)
    return None

  #--
  def SetErrBadParam(self, paramName, paramErr):
    """ Set bad input parameter value error message.

        Parameters:
          paramName - parameter name
          paramErr  - parameter error
        
        Return Value:
          None
    """
    self.mErrStr = "Bad input parameter '" + paramName + "': " + paramErr
    return None


  #
  # Private Interface
  #

  #--
  def _DataToString(self, binData):
    """ Convert binary byte list to readable string.

        Parameters:
          binData  - byte list of binary data
        
        Return Value:
          None
    """
    if not binData: return "*empty*"
    s = ''
    sep = ''
    for byte in binData:
      s += "%s0x%02x " % (sep, byte)
      sep = ' '
    return s

