###############################################################################
#
# Package:  BotSense
#
# File: BotSenseError.py
#

""" 
BotSense Error Classes and Supporting Routines
"""

## \file 
## \package BotSense.BotSenseError
##
## $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
## $Rev: 581 $
##
## \brief BotSense Error Classes and Routines
##
## \sa
## \htmlonly
##  <a href="../pydoc/BotSense.BotSenseError.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2010-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##

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
###############################################################################

import BotSenseCore as bsCore

#--
def StrError(ecode):
  """ Get the error string describing the BotSense error code.
      
      The absolute value of the error code is taken prior retrieving the
      string. An unknown or out-of-range error code will be mapped to
      BS_ECODE_BADEC.
    
      Parameters:
        ecode     - BotSense error code.

      Return:
        The appropriate error code string.
  """
  sErr = bsCore.bsStrError(ecode)
  if not sErr:
    sErr = 'Error'
  return sErr
##

#--
def ChkType(obj, T):
  """ Check of oject is an instance of type T.

      Raises a BotSenseError exception if false.

      Parameters:
        obj   - Object instance.
        T     - Required type.
  """
  if not isinstance(obj, T):
    if hasattr(T, '__name__'):
      name = T.__name__
    elif hasattr(T, '__class__.__name__'):
      name = T.__class__.__name__
    else:
      name = repr(T)
    raise BotSenseError(bsCore.BS_ECODE_BAD_VAL,
        "Object type is not the '%s' required type." % (name))
##

#--
def ChkReturnIsOk(rc, emsg=None):
  """ Check if return is ok (BsOk).

      Raises a BotSenseError exception if false.

      Parameters:
        rc    - BotSense return code.
        emsg  - Optional error message string.
  """
  if rc != bsCore.BS_OK:
    raise BotSenseError(rc, emsg)
##

#--
def ChkReturnIsNonNeg(rc, emsg=None):
  """ Check if return is non-negative.

      Raises a BotSenseError exception if false.

      Parameters:
        rc    - BotSense return code.
        emsg  - Optional error message string.
  """
  if rc < 0:
    raise BotSenseError(rc, emsg)
##


#------------------------------------------------------------------------------
# CLASS: BotSenseError
#------------------------------------------------------------------------------
class BotSenseError(Exception):
  """ BotSense Exception Class. """

  #--
  def __init__(self, ecode, emsg=None):
    """ Raise exception.

        Parameters:
          ecode  - BotSense error code.
          msg    - Exception message string.
    """
    ## \h_botsense error code
    self.ecode = ecode

    ## error message
    self.emsg = emsg
  ##

  #--
  def __str__(self):
    """ String representation. """
    s = "%s(ecode=%d)" % (StrError(self.ecode), self.ecode)
    if self.emsg:
      s += ": %s" % (self.emsg)
    return s
  ##
##
