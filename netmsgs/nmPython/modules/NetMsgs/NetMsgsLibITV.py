###############################################################################
#
# Package:  NetMsgs
#
# File: NetMsgsLibITV.py
#

""" 
NetMsgs Run-Time Library Packing and Unpacking ITV Module.
"""

## \file 
## \package NetMsgs.NetMsgsLibITV
##
## $LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
## $Rev: 550 $
##
## \brief NetMsgs Run-Time Library Packing and Unpacking ITV Module.
##
## The NetMsgsLibITV module defines the derived NetMsgsStreamBuf derived
## run-time ITV class. The NetMsgsITV class provides all of the functionality
## to pack, unpack, and trace messages encoded in the Identifier-Type-Value
## message format.
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsLibITV.html">PyDoc Generated Documentation</a>
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

import NetMsgs.NetMsgsBase as nmBase
from   NetMsgs.NetMsgsLib import *
import NetMsgs.NetMsgsLibStreamBuf as nmStream



#-----------------------------------------------------------------------------
# CLASS: NetMsgsITV
#-----------------------------------------------------------------------------

class NetMsgsITV(nmStream.NetMsgsStreamBuf):
  """ RoadNarrows Identifier-Type-Value encoded Net Messages Class.

      ITV message encoding provides field headers to identify and process
      fields. Fields may be in any order. Unknown field are ignored.
  """

  #--
  def __init__(self, msgdefset, **kwargs):
    """ Initialize NetMsgsITV instance.

        Parameters:
          msgdefset - Set of message definitions.
          kwargs    - Optional keyword arguments. See NetMsgsStreamBuf.
    """
    kwargs['encoding'] = 'itv'
    nmStream.NetMsgsStreamBuf.__init__(self, msgdefset, **kwargs)
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Derived State Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def StateNew(self, msgid, op, **kwargs):
    """ Create a new packing/unpacking operational state.

        The operational state contains message information plus a stack of
        current field processing states. Each field state has header,
        control, run-time determined values, and tracing parameters.

        Parameters:
          msgid   - Message id.
          op      - Operation string. One of: 'pack', 'unpack'.
          kwargs  - Optional initial state overrides and implementation
                    specifics.

        Return:
          State id which is a key into the specific state.
    """
    stateId = nmStream.NetMsgsStreamBuf.StateNew(self, msgid, op, **kwargs)
    self.StateFieldSet(stateId, 'fctl', has_hdr=True)
    return stateId
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Derived Packing Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmPackFieldHdr(self, fielddef, val, stateId):
    """ Pack ITV field header. 

        Parameters:
          fielddef  - Field definition.
          val       - Field value(s).
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    fhdr    = {}
    fstate  = self.StateFieldGet(stateId)
    fid     = fstate['fid']
    ftype   = fstate['ftype']
    self.ChkReqFValType(fielddef, val, stateId=stateId)
    if not fstate['fctl']['has_hdr']:   # don't pack header (vector item)
      count = None
      buf = ''
    elif ftype == 's':    # string compound field type
      count = len(val)
      buf  = PackU8(fid, endian=self.mEndian)
      buf += PackU8(ftype, endian=self.mEndian)
      buf += PackU8(count, endian=self.mEndian)
      fhdr['fid']   = fid
      fhdr['ftype'] = ftype
      fhdr['count'] = count
    elif ftype == '{':    # struct compound field type
      msgdef  = fielddef['msgdef']
      count   = msgdef['max_count']
      buf  = PackU8(fid, endian=self.mEndian)
      buf += PackU8(ftype, endian=self.mEndian)
      buf += PackU8(count, endian=self.mEndian)
      fhdr['fid']   = fid
      fhdr['ftype'] = ftype
      fhdr['count'] = count
    elif ftype == '[':    # vector compound field type
      vdef  = fielddef['vdef']
      vtype = vdef['ftype']
      count = len(val)
      buf  = PackU8(fid, endian=self.mEndian)
      buf += PackU8(ftype, endian=self.mEndian)
      buf += PackU8(count, endian=self.mEndian)
      buf += PackU8(vtype, endian=self.mEndian)
      fhdr['fid']   = fid
      fhdr['ftype'] = ftype
      fhdr['count'] = count
      fhdr['vtype'] = vtype
    else:                 # simple type
      count = None
      buf  = PackU8(fid, endian=self.mEndian)
      buf += PackU8(ftype, endian=self.mEndian)
      fhdr['fid']   = fid
      fhdr['ftype'] = ftype
    fhdr['fhdr_size'] = len(buf)
    self.StateFieldSet(stateId, fhdr=fhdr)
    return buf
  ##

  #--
  def nmPackString(self, fielddef, val, stateId):
    """ Pack variable length string field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.ChkReqFValType(fielddef, val)
    count     = len(val)
    max_count = fielddef.get('max_count', nmBase.NMStringMaxCount)
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgStringRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=count)
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackString(val, count)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackVector(self, fielddef, vallist, stateId):
    """ Pack variable vector field.

        Parameters:
          fielddef  - Field definition.
          vallist   - Vector (list) of vector item values.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.ChkReqFValType(fielddef, vallist)
    vdef      = fielddef['vdef']
    vname     = vdef['name']
    vtype     = vdef['ftype']
    count     = len(vallist)
    max_count = fielddef['max_count']
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgVectorRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=count)
    buf = self.nmPackFieldHdr(fielddef, vallist, stateId)
    if self.mTrace: self.TraceField(fielddef, vallist, buf, stateId)
    setfunc   = self.mFuncMap[vtype][self.SETFUNC]
    packfunc  = self.mFuncMap[vtype][self.PACKFUNC]
    i   = 0
    self.StateFieldPush(stateId, fname=vname, fid=0, ftype=vtype)
    if vtype in nmBase.NMFTypeCodeSimple:
      self.StateFieldSet(stateId, 'fctl', has_hdr=False)
    for val in vallist:
      self.StateFieldSet(stateId, i=i)
      self.StateFieldSet(stateId, fid=i+1)
      val  = setfunc(vdef, val, stateId)
      buf += packfunc(vdef, val, stateId)
      i   += 1
    self.StateFieldPop(stateId)
    return buf
  ##

  #--
  def nmPackMsgHdr(self, msgid, msgdef, stateId):
    """ Pack message header. 

        Parameters:
          msgid     - Message identifier.
          msgdef    - Message definition.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    count = msgdef['max_count']
    buf = PackU16(msgid, endian=self.mEndian)
    buf += PackU8(count, endian=self.mEndian)
    msghdr = {'msghdr_size':len(buf), 'msgid':msgid, 'count':count}
    self.StateSet(stateId, msghdr=msghdr)
    return buf
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Unpacking Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmUnpackFieldHdr(self, buf, offset, stateId):
    """ Unpack ITV field header. 

        Parameters:
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fhdr    = {}
    fstate  = self.StateFieldGet(stateId)
    start   = offset
    if self.StateFieldGet(stateId, 'fctl', 'has_hdr'):
      # common header part
      fid, offset   = UnpackU8(buf, offset, endian=self.mEndian)
      ftype, offset = UnpackU8(buf, offset, endian=self.mEndian)
      ftype = "%c" % ftype
      fhdr['fid'] = fid
      fhdr['ftype'] = ftype
      if ftype == 's':      # string compound field type
        fhdr['count'], offset = UnpackU8(buf, offset, endian=self.mEndian)
      elif ftype == '{':    # struct compound field type
        fhdr['count'], offset = UnpackU8(buf, offset, endian=self.mEndian)
      elif ftype == '[':    # vector compound field type
        fhdr['count'], offset = UnpackU8(buf, offset, endian=self.mEndian)
        vtype, offset = UnpackU8(buf, offset, endian=self.mEndian)
        fhdr['vtype'] = "%c" % vtype
    fhdr['fhdr_size'] = offset - start
    self.StateFieldSet(stateId, fhdr=fhdr)
    return offset
  ##

  #--
  def nmUnpackPad(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack variable number of pad bytes from the buffer.

        No field values are set.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    count = 0
    while (ord(buf[offset]) == nmBase.NMPadFVal) and (offset < len(buf)):
      offset += 1
      count += 1
    self.StateFieldSet(stateId, count=count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, nmBase.NMPadFVal, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackString(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack variable lenght string field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname     = fielddef['name']
    count     = self.StateFieldGet(stateId, 'fhdr', 'count')
    max_count = fielddef['max_count']
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgStringRange, count, max_count),
                 stateId=stateId)
    val, offset = UnpackString(buf, count, offset=offset)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, count=count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackStruct(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack structure field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname     = fielddef['name']
    msgdef    = fielddef['msgdef']
    max_count = msgdef['max_count']
    fhdr      = self.StateFieldGet(stateId, 'fhdr')
    count     = fhdr['count']
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgStructRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = {}
      val = fvals[fname]
    else:
      val = {}
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return self.nmUnpackStream(msgdef, buf, offset, val, stateId)
  ##

  #--
  def nmUnpackVector(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack variable vector field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname     = fielddef['name']
    vdef      = fielddef['vdef']
    vname     = vdef['name']
    vtype     = vdef['ftype']
    max_count = fielddef['max_count']
    fhdr      = self.StateFieldGet(stateId, 'fhdr')
    count     = fhdr['count']
    if vtype != fhdr['vtype']:
      self.Error("%s" % fielddef['name'], EMsgBadMsg,
           "unpacked vtype=%c != expected vtype=%c" % (fhdr['vtype'], vtype),
           stateId=stateId)
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgVectorRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = []
      val = fvals[fname]
    else:
      val = []
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    if vtype in nmBase.NMFTypeCodeSimple:
      self.StateFieldSet(stateId, 'fctl', has_hdr=False)
    else:
      self.StateFieldSet(stateId, 'fctl', has_hdr=True)
    unpackfunc = self.mFuncMap[vtype][self.UNPACKFUNC]
    vval  = {}
    i     = 0
    self.StateFieldPush(stateId, fname=vname, fid=0, ftype=vtype)
    while i < count:
      self.StateFieldSet(stateId, 'trace', trace_start=offset)
      self.StateFieldSet(stateId, i=i)
      offset = self.nmUnpackFieldHdr(buf, offset, stateId)
      offset = unpackfunc(vdef, buf, offset, vval, stateId)
      val += [vval[vname]]
      i   += 1
      vval = {}
    self.StateFieldPop(stateId)
    return offset
  ##

  #--
  def nmUnpackStream(self, msgdef, buf, offset, fvals, stateId):
    """ Unpack a field stream from the buffer.

        Parameters:
          msgdef    - Message definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    msgname   = msgdef['name']
    fdeflist  = msgdef['fielddef']
    max_count = msgdef['max_count']
    count     = self.StateFieldGet(stateId, 'count')
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgStructRange, count, max_count),
                 stateId=stateId)
    i   = 0
    while i < count:
      self.StateFieldPush(stateId)
      self.StateFieldSet(stateId, 'trace', trace_start=offset)
      if ord(buf[offset]) == nmBase.NMPadFVal:
        fid = 0
        ftype = nmBase.NMFCode('pad')
        fielddef = {'name':'pad', 'fid':0, 'ftype':ftype}
      else:
        offset  = self.nmUnpackFieldHdr(buf, offset, stateId)
        fid     = self.StateFieldGet(stateId, 'fhdr', 'fid')
        ftype   = self.StateFieldGet(stateId, 'fhdr', 'ftype')
        fielddef = self.FindFId(msgdef, fid)
      if fielddef:
        fname = fielddef['name']
        fid   = fielddef['fid']
        if ftype != fielddef['ftype']:
          self.Error("%s" % fname,
                 "%s received '%s' != expected '%s'" % \
                     (EMsgBadMsg, ftype, fielddef['ftype']),
                 stateId=stateId)
        self.StateFieldSet(stateId, fname=fname, fid=fid, ftype=ftype)
        unpackfunc = self.mFuncMap[ftype][self.UNPACKFUNC]
        offset = unpackfunc(fielddef, buf, offset, fvals, stateId)
      else:
        self.Warning("fid=%u" % (fid), "Field not defined - ignoring")
        self.StateFieldSet(stateId, 'fctl', noexec=True)
        unpackfunc = self.mFuncMap[ftype][self.UNPACKFUNC]
        offset = unpackfunc(fielddef, buf, offset, fvals, stateId)
      self.StateFieldPop(stateId)
      i += 1
    return offset
  ##

  #--
  def nmUnpackMsgHdr(self, msgid, msgdef, buf, offset, fvals, stateId):
    """ Unpack ITV message header. 

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
    msghdr = {}
    start = offset
    msghdr['msgid'], offset = UnpackU16(buf, offset, endian=self.mEndian)
    msghdr['count'], offset = UnpackU8(buf, offset, endian=self.mEndian)
    msghdr['msghdr_size'] = offset - start
    if msgid != msghdr['msgid']:
      self.Error("%s" % msgdef['name'], EMsgBadMsg,
           "unpacked msgid=%u != expected msgid=%u" % (msghdr['msgid'], msgid),
           stateId=stateId)
    count     = msghdr['count']
    max_count = msgdef['max_count']
    if count > max_count:
      self.Error("%s" % msgdef['name'],
                 "%s %u > %u" % (EMsgStructRange, count, max_count),
                 stateId=stateId)
    self.StateSet(stateId, msghdr=msghdr)
    self.StateFieldSet(stateId, count=count)
    return offset
  ##
##
