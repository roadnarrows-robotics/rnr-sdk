###############################################################################
#
# Package:  NetMsgs
#
# File: NetMsgsLibFlat.py
#

""" 
NetMsgs Run-Time Library Packing and Unpacking Flat Encoding Module.
"""

## \file 
## \package NetMsgs.NetMsgsLibFlat
##
## $LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
## $Rev: 550 $
##
## \brief NetMsgs Run-Time Library Packing and Unpacking Flat Encoding Module.
##
## The NetMsgsLibFlat module defines the NetMsgsStreamBuf derived run-time
## class. The NetMsgsFlat class provides all of the functionality to pack,
## unpack, and trace messages encoded as flat fixed-length fields with no
## message or field headers.
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsLibFlat.html">PyDoc Generated Documentation</a>
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

import sys
import warnings

import NetMsgs.NetMsgsBase as nmBase
from   NetMsgs.NetMsgsLib import *
import NetMsgs.NetMsgsLibStreamBuf as nmStream

## space over
space = lambda indent: "%*s" % (indent, '')

#-----------------------------------------------------------------------------
# CLASS: NetMsgs
#-----------------------------------------------------------------------------

class NetMsgsFlat(nmStream.NetMsgsStreamBuf):
  """ RoadNarrows Flat Fixed-Field encode Net Messages Class.

      Flat message encoding for fixed-sized, flat messages with no message or
      field header information.
  """

  #--
  def __init__(self, msgdefset, **kwargs):
    """ Initialize NetMsgsFlat instance.

        Parameters:
          msgdefset - Set of message definitions.
          kwargs    - Optional keyword arguments. See NetMsgsStreamBuf.
    """
    kwargs['encoding'] = 'flat'
    nmStream.NetMsgsStreamBuf.__init__(self, msgdefset, **kwargs)
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Packing Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmPackFieldHdr(self, fielddef, val, stateId):
    """ Pack field header. 

        No header is packed.

        Parameters:
          fielddef  - Field definition.
          val       - Field value(s).
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.StateFieldSet(stateId, fhdr={'fhdr_size': 0})
    return ''
  ##

  #--
  def nmPackMsgHdr(self, msgid, msgdef, stateId):
    """ Pack message header. 

        No header is packed.

        Parameters:
          msgid     - Message identifier.
          msgdef    - Message definition.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.StateSet(stateId, msghdr={'msghdr_size': 0})
    return ''
  ##

  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Unpacking Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmUnpackFieldHdr(self, buf, offset, stateId):
    """ Unpack field header. 

        No header is unpacked.

        Parameters:
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fhdr = self.StateFieldSet(stateId, fhdr={'fhdr_size': 0})
    return offset
  ##

  #--
  def nmUnpackMsgHdr(self, msgid, msgdef, buf, offset, fvals, stateId):
    """ Unpack message header. 

        No header is unpacked.

        Parameters:
          msgid     - Message identifier.
          msgdef    - Message definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    self.StateSet(stateId, msghdr={'msghdr_size': 0})
    return offset
  ##

##

