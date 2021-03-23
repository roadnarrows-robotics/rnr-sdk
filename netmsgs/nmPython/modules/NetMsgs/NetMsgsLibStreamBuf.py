###############################################################################
#
# Package:  NetMsgs
#
# File: NetMsgsLibStreamBuf.py
#

""" 
NetMsgs Run-Time Library Packing and Unpacking Stream Buffer Module.
"""

## \file 
## \package NetMsgs.NetMsgsLibStreamBuf
##
## $LastChangedDate: 2012-12-11 14:28:44 -0700 (Tue, 11 Dec 2012) $
## $Rev: 2574 $
##
## \brief NetMsgs Run-Time Library Packing and Unpacking Stream Buffer Module.
##
## The NetMsgsLibStreamBuf module defines the base NetMsgsStreamBuf run-time
## class. The NetMsgsStreamBuf class provides all serialized stream buffer
## packing and de-serialized stream buffer unpacking virtual functionality.
## Packed buffers are transportable across a network of heterogenous nodes.
##
## The NetMsgsStreamBuf class assumes a (generated) python data structures of
## the form define by the NetMsgsGenPy module given an NetMsgs XML input
## specification.
##
## The particular message encodings are defined in derived classes such as
## NetMsgsFlat and NetMsgsITV. Other encoded message classes should derive
## from the NetMsgsStreamBuf class.
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
import itertools

import NetMsgsBase as nmBase
from   NetMsgsLib import *

## space over
space = lambda indent: "%*s" % (indent, '')

#-----------------------------------------------------------------------------
# CLASS: NetMsgs
#-----------------------------------------------------------------------------
class NetMsgsStreamBuf:
  """ RoadNarrows Net Messages Stream Buffer Virtual Base Class.

      NetMsgsStreamBuf supports serialized packing and de-serialized
      unpacking of a registered set of message definitions.

      Packing and unpacking operations are multi-thread safe.
  """

  #--
  def __init__(self, msgdefset, **kwargs):
    """ Initialize NetMsgsStreamBuf instance.

        Parameters:
          msgdefset - Set of message definitions.
          kwargs    - Optional keyword arguments.
                        msgsetname=str  - Name. Default: "MessageSet"
                        encoding=val    - Message stream encoding.
                                          Default: unknown
                        endian=val      - Byte ordering. Default: big
                        nowarn=bool     - Disable warnings. Default: False.
                        trace=bool      - Trace packing/unpacking.
                                          Default: False
    """
    ## index to pack function
    self.PACKFUNC   = 0

    ## index to unpack function
    self.UNPACKFUNC = 1

    ## index to set function
    self.SETFUNC    = 2

    ## Message definition set.
    self.mMsgDefSet = msgdefset

    # optimize message definition set
    self.Optimize()

    ## message definition set string name
    self.mMsgDefSetName = kwargs.get('msgsetname', "MessageSet")

    ## message encoding
    self.mEncoding = kwargs.get('encoding', 'unknown')

    ## byte ordering
    self.AttrSetByteOrder(kwargs.get('endian', 'big'))

    ## do [not] disable warnings
    self.mNoWarn = kwargs.get('nowarn', False)

    ## disable python runtime warnings
    if self.mNoWarn:
      warnings.simplefilter('ignore', RuntimeWarning)

    ## tracing
    self.mTrace = kwargs.get('trace', False)

    ## atomic state counter
    self.mStateId = itertools.count()

    ## pack/unpack state
    self.mState = {}

    ## Field Code to Function Mappings
    self.mFuncMap = {
        nmBase.NMFCode('pad'):
          [self.nmPackPad,      self.nmUnpackPad,     self.nmIdent],  
        nmBase.NMFCode('char'):
          [self.nmPackChar,     self.nmUnpackChar,    self.nmIdent],  
        nmBase.NMFCode('u8'):
          [self.nmPackU8,       self.nmUnpackU8,      self.nmSetInt],  
        nmBase.NMFCode('s8'):
          [self.nmPackS8,       self.nmUnpackS8,      self.nmSetInt],  
        nmBase.NMFCode('bool'):
          [self.nmPackBool,     self.nmUnpackBool,    self.nmIdent],  
        nmBase.NMFCode('u16'):
          [self.nmPackU16,      self.nmUnpackU16,     self.nmSetInt],  
        nmBase.NMFCode('s16'):
          [self.nmPackS16,      self.nmUnpackS16,     self.nmSetInt],  
        nmBase.NMFCode('u32'):
          [self.nmPackU32,      self.nmUnpackU32,     self.nmSetInt],  
        nmBase.NMFCode('s32'):
          [self.nmPackS32,      self.nmUnpackS32,     self.nmSetInt],  
        nmBase.NMFCode('u64'):
          [self.nmPackU64,      self.nmUnpackU64,     self.nmSetInt],  
        nmBase.NMFCode('s64'):
          [self.nmPackS64,      self.nmUnpackS64,     self.nmSetInt],  
        nmBase.NMFCode('f32'):
          [self.nmPackF32,      self.nmUnpackF32,     self.nmSetFPN],  
        nmBase.NMFCode('f64'):
          [self.nmPackF64,      self.nmUnpackF64,     self.nmSetFPN],  
        nmBase.NMFCode('p32'):
          [self.nmPackP32,      self.nmUnpackP32,     self.nmIdent],  
        nmBase.NMFCode('p64'):
          [self.nmPackP64,      self.nmUnpackP64,     self.nmIdent],  
        nmBase.NMFCode('string'):
          [self.nmPackString,   self.nmUnpackString,  self.nmSetStr],  
        nmBase.NMFCode('struct'):
          [self.nmPackStruct,   self.nmUnpackStruct,  self.nmIdent],  
        nmBase.NMFCode('vector'):
          [self.nmPackVector,   self.nmUnpackVector,  self.nmIdent],  
    }
  ##

  #--
  def Optimize(self):
    """ Optimize message definition set for packing and unpacking. """
    for msgid, msgdef in self.mMsgDefSet.iteritems():
      self.OptimizeMsgDef(msgid, msgdef)
  ##

  #--
  def OptimizeMsgDef(self, msgid, msgdef):
    """ Optimize message definition for packing and unpacking.

        Parameters:
          msgid   - Message id.
          msgdef  - Message definition.
    """
    if type(msgid) != int:
      self.Error("msgid=%s" % repr(msgid), EMsgBadId)
    if not msgdef.has_key('name'):
      msgdef['name'] = "Msg%s" % repr(msgid)
    msgdef['msgid'] = msgid
    fdeflist = self.GetDefReqVal(msgdef, 'fielddef',
          epreface="MsgDef %s" % (msgdef['name']))
    max_count = 0
    for fielddef in fdeflist:
      self.OptimizeFieldDef(fielddef)
      if fielddef['disposition'] == 'active':
        max_count += 1
    msgdef['max_count'] = max_count
  ##

  #--
  def OptimizeFieldDef(self, fielddef):
    """ Optimize field definition for packing and unpacking.

        Parameters:
          fielddef  - Field definition.
    """
    fname = self.GetDefReqVal(fielddef, 'name', epreface="FieldDef")
    fid   = self.GetDefReqVal(fielddef, 'fid', epreface="FieldDef")
    ftype = self.GetDefReqVal(fielddef, 'ftype', epreface="FieldDef")
    if not fielddef.has_key('disposition'):
      fielddef['disposition'] = 'active'
    if fielddef['disposition'] == 'deprecated':
      return
    if ftype == nmBase.NMFCode('pad'):        # pad
      if not fielddef.has_key('count'):
        fielddef['count'] = nmBase.NMPadDftCount
    elif ftype == nmBase.NMFCode('string'):   # string
      if not fielddef.has_key('max_count'):
        fielddef['max_count'] = nmBase.NMStringMaxCount
    elif ftype == nmBase.NMFCode('vector'):   # vector
      if not fielddef.has_key('max_count'):
        fielddef['max_count'] = nmBase.NMVectorMaxCount
      vdef  = self.GetDefReqVal(fielddef, 'vdef', epreface="FieldDef")
      if not vdef.has_key('name'):
        vdef['name'] = "%s_item" % fname
      vdef['fid'] = 0     # field id not used, but required when packing
      self.OptimizeFieldDef(vdef)
    elif ftype == nmBase.NMFCode('struct'):   # struct
      msgdef  = self.GetDefReqVal(fielddef, 'msgdef', epreface="FieldDef")
      if not msgdef.has_key('name'):
        msgdef['name'] = "%s_struct" % fname
      msgdef['msgid'] = 0 # field id not used, but required when packing
      self.OptimizeMsgDef(msgdef['msgid'], msgdef)
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Attribute Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def AttrAddMsgDef(self, msgid, msgdef):
    """ Add/Overwrite message definition to set.

        Parameters:
          msgid   - Unique message identifier number.
          msgdef  - Message definition.
    """
    self.mMsgDefSet[msgid] = msgdef
    self.OptimizeMsgDef(msgdef)
  ##

  #--
  def AttrDelMsgDef(self, msgid):
    """ Delete message definition from set.

        Parameters:
          msgid   - Unique message identifier number.
    """
    try:
      del self.mMsgDefSet[msgid]
    except KeyError:
      self.Error(repr(msgid), EMsgNoMsgDef)
  ##

  #--
  def AttrSetByteOrder(self, endian):
    """ Set message packing and unpack field byte order.

        Parameters:
          endian  - Byte order. One of: big little native
    """
    if endian in nmBase.NMEndian:
      ## Byte order
      self.mEndian = endian
    else:
      self.Error("endian=%s" % (endian), EMsgEndian)
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual State Functions
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
    # get next state id (action is atomic)
    stateId = self.mStateId.next()
    d = {
      'msgid':        msgid,  # message id
      'op':           op,     # pack/unpack
      'field_count':  0,      # number of fields in message
      'msghdr': {},           # message header info
      'mctl':   {},           # message control
      'fstack': [             # field stack
        { 'fpar':   None,       # field parent    
          'fctl':   {'has_hdr':False, 'noexec': False}, # field control
          'fname':  None,       # field name
          'fid':    0,          # field id
          'ftype':  None,       # field type
          'fhdr':   {},         # field header info
          'trace':  {'trace_depth':0, 'trace_start':0, 'trace_end':None},
                                # trace state
        }
      ], 
    }

    # overrides and implementation specifics
    for k,v in kwargs.iteritems():
      d[k] = v

    # add new state (thread safe)
    self.mState[stateId] = d

    return stateId
  ##

  #--
  def StateDelete(self, stateId):
    """ Delete operational state.

        Parameters:
          stateId   - State id.
    """
    if stateId is not None:
      del self.mState[stateId]
  ##

  #--
  def StateGet(self, stateId, *keypath):
    """ Get state value at key path.

        Parameters:
          keypath   - Optional list of key arguments.
    """
    v = self.mState[stateId]
    for key in keypath:
      v = v[key]
    return v
    return self.mState[stateId][key]
  ##

  #--
  def StateSet(self, stateId, *keypath, **kwargs):
    """ Set a list of state values at key path.

        Parameters:
          keypath   - Optional list of key arguments.
          kwargs    - List of key=value arguments.
    """
    d = self.mState[stateId]
    for key in keypath:
      d = d[key]
    for k,v in kwargs.iteritems():
      d[k] = v
  ##

  #--
  def StateFieldPush(self, stateId, **kwargs):
    """ Push new field state onto stack.

        Parameters:
          stateId - State id.
          kwargs  - Optional initial field state overrides and
                    implementation specifics.
    """
    parent  = self.mState[stateId]['fstack'][-1]

    # defaults and required
    d           = {}
    d['fpar']   = parent['ftype']
    d['fctl']   = parent['fctl'].copy()
    d['fname']  = None,
    d['fid']    = 0
    d['ftype']  = None
    d['fhdr']   = {}
    d['trace']  = { 'trace_depth':  parent['trace']['trace_depth']+1,
                    'trace_start':  0,
                    'trace_end':    None
                  }

    # overrides and implementation specifics
    for k,v in kwargs.iteritems():
      d[k] = v

    # push
    self.mState[stateId]['fstack'] += [d]
  ##

  #--
  def StateFieldPop(self, stateId):
    """ Pop field state stack - deleting top state.

        Parameters:
          stateId   - State id.
    """
    self.mState[stateId]['fstack'].pop()
  ##

  #--
  def StateFieldGet(self, stateId, *keypath):
    """ Get top of field stack state value at key path.

        Parameters:
          keypath   - Optional list of key arguments.
    """
    v = self.mState[stateId]['fstack'][-1]
    for key in keypath:
      v = v[key]
    return v
  ##

  #--
  def StateFieldSet(self, stateId, *keypath, **kwargs):
    """ Set a list of top of field stack state values at key path.

        Parameters:
          keypath   - Optional list of key arguments.
          kwargs    - List of key=value arguments.
    """
    d = self.mState[stateId]['fstack'][-1]
    for key in keypath:
      d = d[key]
    for k,v in kwargs.iteritems():
      d[k] = v
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Packing Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmPackFieldHdr(self, fielddef, val, stateId):
    """ Pack field header. 

        No header is packed in base function.

        Parameters:
          fielddef  - Field definition.
          val       - Field value(s).
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.StateFieldSet(stateId, fhdr={})
    return ''
  ##

  #--
  def nmPackPad(self, fielddef, val, stateId):
    """ Pack pad byte(s). 

        Count pad bytes are packed.

        Parameters:
          fielddef  - Field definition.
          val       - Field value(s) - ignored.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    count = fielddef['count']
    buf   = ''
    i     = 0
    while i < count:
      buf += PackU8(0)
      i   += 1
    self.StateFieldSet(stateId, count=count)
    if self.mTrace:
      self.TraceField(fielddef, 0, buf, stateId)
    return buf
  ##

  #--
  def nmPackBool(self, fielddef, val, stateId):
    """ Pack boolean field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackBool(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackChar(self, fielddef, val, stateId):
    """ Pack one character field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackChar(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackU8(self, fielddef, val, stateId):
    """ Pack 8-bit unsigned integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackU8(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackS8(self, fielddef, val, stateId):
    """ Pack 8-bit signed integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackS8(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackU16(self, fielddef, val, stateId):
    """ Pack 16-bit unsigned integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackU16(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackS16(self, fielddef, val, stateId):
    """ Pack 16-bit signed integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackS16(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackU32(self, fielddef, val, stateId):
    """ Pack 32-bit unsigned integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackU32(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackS32(self, fielddef, val, stateId):
    """ Pack 32-bit signed integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackS32(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackU64(self, fielddef, val, stateId):
    """ Pack 64-bit unsigned integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackU64(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackS64(self, fielddef, val, stateId):
    """ Pack 64-bit signed integer field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackS64(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackF32(self, fielddef, val, stateId):
    """ Pack 32-bit floating-point number field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackF32(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackF64(self, fielddef, val, stateId):
    """ Pack 64-bit floating-point number field.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackF64(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackP32(self, fielddef, val, stateId):
    """ Pack 32-bit pointer field.

        Pointer is always packed in natvie byte order.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackP32(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackP64(self, fielddef, val, stateId):
    """ Pack 64-bit pointer field.

        Pointer is always packed in natvie byte order.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackP64(val, endian=self.mEndian)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackString(self, fielddef, val, stateId):
    """ Pack variable length string into fixed-size field.

        Field length is fixed max_count bytes. If length of string is less
        than max_count, the value is right padded with zeros.

        Parameters:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.ChkReqFValType(fielddef, val, stateId=stateId)
    count     = len(val)
    max_count = fielddef['max_count']
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgStringRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=max_count)
    buf  = self.nmPackFieldHdr(fielddef, val, stateId)
    buf += PackString(val, max_count)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return buf
  ##

  #--
  def nmPackStruct(self, fielddef, fvals, stateId):
    """ Pack structure field.

        Parameters:
          fielddef  - Field definition.
          fvals     - Dictionary of associated field values.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.ChkReqFValType(fielddef, fvals, stateId=stateId)
    msgdef    = fielddef['msgdef']
    max_count = msgdef['max_count']
    self.StateFieldSet(stateId, count=max_count)
    buf = self.nmPackFieldHdr(fielddef, fvals, stateId)
    if self.mTrace: self.TraceField(fielddef, fvals, buf, stateId)
    buf += self.nmPackStream(msgdef, fvals, stateId) 
    return buf
  ##

  #--
  def nmPackVector(self, fielddef, vallist, stateId):
    """ Pack variable vector into fixed-size field.

        Vector length is fixed max_count bytes. If length is less than
        max_count, the value is right padded with vector items of zero value.

        Parameters:
          fielddef  - Field definition.
          vallist   - Vector (list) of vector item values.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.ChkReqFValType(fielddef, vallist, stateId=stateId)
    vdef      = fielddef['vdef']
    vname     = vdef['name']
    vtype     = vdef['ftype']
    count     = len(vallist)
    max_count = fielddef['max_count']
    if count > max_count:
      self.Error("%s" % fielddef['name'],
                 "%s %u > %u" % (EMsgVectorRange, count, max_count),
                 stateId=stateId)
    self.StateFieldSet(stateId, count=max_count)
    buf = self.nmPackFieldHdr(fielddef, vallist, stateId)
    if self.mTrace: self.TraceField(fielddef, vallist, buf, stateId)
    setfunc   = self.mFuncMap[vtype][self.SETFUNC]
    packfunc  = self.mFuncMap[vtype][self.PACKFUNC]
    i   = 0
    self.StateFieldPush(stateId, fname=vname, fid=0, ftype=vtype)
    for val in vallist:
      self.StateFieldSet(stateId, i=i)
      val  = setfunc(vdef, val, stateId)
      buf += packfunc(vdef, val, stateId)
      i   += 1
    zval = self.ZeroFType(vdef)
    while i < max_count:
      self.StateFieldSet(stateId, i=i)
      buf += packfunc(vdef, zval, stateId)
      i   += 1
    self.StateFieldPop(stateId)
    return buf
  ##

  #--
  def nmPackStream(self, msgdef, fvals, stateId):
    """ Pack a stream of fields.

        Parameters:
          msgdef    - Message definition.
          fvals     - Dictionary of associated field values.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    msgname   = msgdef['name']
    fdeflist  = msgdef['fielddef']
    count     = len(fdeflist)
    max_count = msgdef['max_count']
    self.StateFieldSet(stateId, count=max_count)
    buf = ''
    i   = 0
    while i < count:
      fielddef  = fdeflist[i]
      fname     = fielddef['name']
      fid       = fielddef['fid']
      ftype     = fielddef['ftype']
      if ftype == nmBase.NMFCode('pad'):    # pad field 
        val = 0
      else:
        val   = fvals.get(fname, None)
      self.StateFieldPush(stateId, fname=fname, fid=fid, ftype=ftype)
      val   = self.mFuncMap[ftype][self.SETFUNC](fielddef, val, stateId)
      buf  += self.mFuncMap[ftype][self.PACKFUNC](fielddef, val, stateId)
      self.StateFieldPop(stateId)
      i    += 1
    return buf
  ##

  #--
  def nmPackMsgHdr(self, msgid, msgdef, stateId):
    """ Pack message header. 

        No header is packed in base function.

        Parameters:
          msgid     - Message identifier.
          msgdef    - Message definition.
          stateId   - Packing state id.

        Return:
          Packed buffer.
    """
    self.StateSet(stateId, msghdr={})
    return ''
  ##

  #--
  def nmPackMsg(self, msgid, fvals):
    """ Pack a message.

        Parameters:
          msgid     - Message identifier.
          fvals     - Dictionary of associated field values.

        Return:
          Packed buffer.
    """
    try:
      msgdef = self.mMsgDefSet[msgid]
    except KeyError:
      self.Error("MsgDefSet", repr(msgid), EMsgNoMsgDef)
    stateId = self.StateNew(msgid, 'pack')
    buf  = self.nmPackMsgHdr(msgid, msgdef, stateId)
    max_count = msgdef['max_count']
    self.StateSet(stateId, field_count=max_count)
    if self.mTrace: self.TraceStart("Packing", msgid, msgdef, buf, stateId)
    buf += self.nmPackStream(msgdef, fvals, stateId)
    if self.mTrace: self.TraceBufBlock("Output", buf)
    self.StateDelete(stateId)
    return buf
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Unpacking Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmUnpackFieldHdr(self, buf, offset, stateId):
    """ Unpack field header. 

        No header is unpacked in base function.

        Parameters:
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fhdr = self.StateFieldSet(stateId, fhdr={})
    return offset
  ##

  #--
  def nmUnpackPad(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack fixed number of pad bytes from the buffer.

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
    count = fielddef['count']
    if len(buf[offset:]) >= count:
      offset += count
    else:
      nmBase.NetMsgsError("unpack pad", EMsgNoMem,
          "count=%u > buf_size=%u" % (count, len(buf[offset:])),
          stateId=stateId)
    self.StateFieldSet(stateId, count=count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, nmBase.NMPadFVal, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackBool(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack boolean field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackBool(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackChar(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack one character field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackChar(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackU8(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 8-bit unsigned integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackU8(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackS8(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 8-bit signed integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackS8(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackU16(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 16-bit unsigned integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackU16(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackS16(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 16-bit signed integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackS16(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackU32(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 32-bit unsigned integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackU32(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackS32(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 32-bit signed integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackS32(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackU64(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 64-bit unsigned integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackU64(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackS64(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 64-bit signed integer field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackS64(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackF32(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 32-bit floating-point number field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackF32(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackF64(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 64-bit floating-point number field from the buffer.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackF64(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackP32(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 32-bit pointer field from the buffer.

        Pointers are unpacked in native byte order.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackP32(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackP64(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack 64-bit pointer field from the buffer.

        Pointers are unpacked in native byte order.

        Parameters:
          fielddef  - Field definition.
          buf       - Buffer to unpack.
          offset    - Buffer offset where unpacking begins.
          fvals     - Dictionary to hold unpacked field values.
          stateId   - Unpacking state id.

        Return:
          New buffer offset.
    """
    fname = fielddef['name']
    val, offset = UnpackP64(buf, offset=offset, endian=self.mEndian)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    return offset
  ##

  #--
  def nmUnpackString(self, fielddef, buf, offset, fvals, stateId):
    """ Unpack fixed-size string field from the buffer.

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
    max_count = fielddef['max_count']
    val, offset = UnpackString(buf, max_count, offset=offset)
    val = val.rstrip('\x00')
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = val
    self.StateFieldSet(stateId, count=max_count)
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
    self.StateFieldSet(stateId, count=max_count)
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
    """ Unpack fixed-size vector field from the buffer.

        The unpacked vector length is a fixed max_count items.

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
    self.StateFieldSet(stateId, count=max_count)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if not self.StateFieldGet(stateId, 'fctl', 'noexec'):
      fvals[fname] = []
      val = fvals[fname]
    else:
      val = []
    if self.mTrace: self.TraceField(fielddef, val, buf, stateId)
    unpackfunc  = self.mFuncMap[vtype][self.UNPACKFUNC]
    vval  = {}
    i     = 0
    self.StateFieldPush(stateId, fname=vname, fid=0, ftype=vtype)
    while i < max_count:
      self.StateFieldSet(stateId, 'trace', trace_start=offset)
      self.StateFieldSet(stateId, i=i)
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
    count     = len(fdeflist)
    self.StateFieldSet(stateId, count=max_count)
    i   = 0
    while i < count:
      fielddef  = fdeflist[i]
      fname     = fielddef['name']
      fid       = fielddef['fid']
      ftype     = fielddef['ftype']
      unpackfunc = self.mFuncMap[ftype][self.UNPACKFUNC]

      self.StateFieldPush(stateId, fname=fname, fid=fid, ftype=ftype)
      self.StateFieldSet(stateId, 'trace', trace_start=offset)
      offset = self.nmUnpackFieldHdr(buf, offset, stateId)
      offset = unpackfunc(fielddef, buf, offset, fvals, stateId)
      self.StateFieldPop(stateId)

      i += 1
    return offset
  ##

  #--
  def nmUnpackMsgHdr(self, msgid, msgdef, buf, offset, fvals, stateId):
    """ Unpack message header. 

        No header is unpacked in base function.

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
    self.StateSet(stateId, msghdr={})
    return offset
  ##

  #--
  def nmUnpackMsg(self, msgid, buf, fvals):
    """ Unpack a message from the buffer.

        Parameters:
          msgid     - Message identifier.
          buf       - Buffer to unpack.
          fvals     - Dictionary to hold unpacked field values.

        Return:
          Number of bytes unpacked.
    """
    try:
      msgdef = self.mMsgDefSet[msgid]
    except KeyError:
      self.Error("MsgDefSet", repr(msgid), EMsgNoMsgDef)
    stateId = self.StateNew(msgid, 'unpack')
    offset  = 0
    offset = self.nmUnpackMsgHdr(msgid, msgdef, buf, offset, fvals, stateId)
    self.StateFieldSet(stateId, 'trace', trace_end=offset)
    if self.mTrace:
      self.TraceStart("Unpacking", msgid, msgdef, buf, stateId)
      self.TraceBufBlock("Input", buf, 0)
    offset = self.nmUnpackStream(msgdef, buf, offset, fvals, stateId)
    self.StateDelete(stateId)
    return offset
  ##


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Virtual Set Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def nmIdent(self, fielddef, val, stateId):
    """ Identity function.

        Parameter:
          fielddef  - Field definition.
          val       - Field value.
          stateId   - State id.

        Return:
          val
    """
    if val is None:
      self.Error("%s" % fielddef['name'], EMsgNoFVal, stateId=stateId)
    return val
  ##

  #--
  def nmSetInt(self, fielddef, val, stateId):
    """ Set [un]signed integer value and check any contraints.

        Parameter:
          fielddef  - Field definition.
          val       - Integer field value.
          stateId   - State id.

        Return:
          (Converted) field value.
    """
    val = fielddef.get('const', val)
    if val is None:
      self.Error("%s" % fielddef['name'], EMsgNoFVal, stateId=stateId)
    min = fielddef.get('min', val)
    max = fielddef.get('max', val)
    if (val < min) or (val > max):
      self.Error("%s=%s" % (fielddef['name'], repr(val)),
                 "%s %s <= number <= %s" % (EMsgIntRange, repr(min), repr(max)),
                 stateId=stateId)
    return val
  ##

  #--
  def nmSetFPN(self, fielddef, val, stateId):
    """ Set floating-point number value and check any contraints.

        Parameter:
          fielddef  - Field definition.
          val       - FPN field value.
          stateId   - State id.

        Return:
          (Converted) field value.
    """
    val = fielddef.get('const', val)
    if val is None:
      self.Error("%s" % fielddef['name'], EMsgNoFVal, stateId=stateId)
    min = fielddef.get('min', val)
    max = fielddef.get('max', val)
    if (val < min) or (val > max):
      self.Error("%s=%s" % (fielddef['name'], repr(val)),
                 "%s %s <= number <= %s" % (EMsgFPNRange, repr(min), repr(max)),
                 stateId=stateId)
    return val
  ##

  #--
  def nmSetStr(self, fielddef, val, stateId):
    """ Set string value and check any contraints.

        Parameter:
          fielddef  - Field definition.
          val       - String field value.
          stateId   - State id.

        Return:
          (Converted) field value.
    """
    val = fielddef.get('const', val)
    if val is None:
      self.Error("%s" % fielddef['name'], EMsgNoFVal, stateId=stateId)
    return val
  ##

  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Support Functions
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #--
  def MakeMsgTemplate(self, msgid):
    """ Make a message template from the given message definition.

        Parameters:
          msgid   - Message id.

        Return:
          Message template dictionary.
    """
    try:
      msgdef = self.mMsgDefSet[msgid]
    except KeyError:
      self.Error("MsgDefSet", repr(msgid), EMsgNoMsgDef)
    return self._MakeMsgTemplateR(msgdef['fielddef'])
  ##

  #--
  def _MakeMsgTemplateR(self, fdeflist):
    """ Recursively make a message template helper.

        Parameters:
          fdeflist  - Field definition list.

        Return:
          Message template dictionary.
    """
    template = {}
    for fielddef in fdeflist:
      fname = fielddef['name']
      ftype = fielddef['ftype']
      if ftype == nmBase.NMFCode('struct'):
        template[fname] = self._MakeMsgTemplateR(fielddef['msgdef']['fielddef'])
      elif ftype == nmBase.NMFCode('vector'):
        template[fname] = []
      elif ftype == nmBase.NMFCode('pad'):
        pass
      else:
        template[fname] = fielddef.get('const', None)
    return template
  ##

  #--
  def GetDefReqVal(self, d, k, epreface='', stateId=None):
    """ Get a NetMsgs message or field definition required keyword value.

        Raises NetMsgsError exception of failure.

        Parameters:
          d         - Definition dictionary.
          k         - Definition key.
          epreface  - Optional error preface string on error.
          stateId   - Optional state id.

        Return:
          Definition value.
    """
    try:
      return d[k]
    except KeyError as inst:
      name = d.get('name', '?name?')
      self.Error(epreface, name, inst.args[0], EMsgReqKeyword, stateId=stateId)
  ##

  #--
  def ChkReqFValType(self, fielddef, fval, stateId=None):
    """ Check field definition type against field value type.

        Parameters:
          fielddef  - Field definition.
          fval      - Field value.
          stateId   - State id.
    """
    ftype = fielddef['ftype']
    # boolean
    if ftype == nmBase.NMFCode('bool'): 
      if (type(fval) == bool) or (type(fval) != int):
        return
    # character
    elif ftype == nmBase.NMFCode('char'):
      if (type(fval) == str) and (len(fval) == 1):
        return
    # floating-point number
    elif ftype in [nmBase.NMFCode('f32'), nmBase.NMFCode('f64')]:
      if (type(fval) == float) or (type(fval) == int):
        return
    # string
    elif ftype == nmBase.NMFCode('string'):
      if type(fval) == str:
        return
    # structure
    elif ftype == nmBase.NMFCode('struct'):
      if type(fval) == dict:
        return
    # vector
    elif ftype == nmBase.NMFCode('vector'):
      if type(fval) == list:
        return
    # simple
    elif (type(fval) != str) and (type(fval) != dict) and (type(fval) != list):
      return
    # error
    self.Error("%s(fid=%u)" % (fielddef['name'], fielddef['fid']),
               "%s(%c)=%s" % (nmBase.NMFTypeCode2Xml[ftype], ftype, repr(fval)),
               EMsgFValType,
               stateId=stateId)
  ##

  #--
  def ZeroFType(self, fielddef):
    """ Return "zero" value assocated with field type.

        Parameters:
          fielddef  - Field definition.
    """
    ftype = fielddef['ftype']
    if fielddef['disposition'] == 'deprecated':
      return None
    elif    ftype == nmBase.NMFCode('bool'):
      return False
    elif  ftype == nmBase.NMFCode('char'):
      return '\x00'
    elif  ftype == nmBase.NMFCode('string'):
      return ''
    elif  ftype in [nmBase.NMFCode('f32'), nmBase.NMFCode('f64')]:
      return 0.0
    elif ftype == nmBase.NMFCode('vector'):
      return [ self.ZeroFType(fielddef['vdef']) ]
    elif ftype == nmBase.NMFCode('struct'):
      msgdef    = fielddef['msgdef']
      fdeflist  = msgdef['fielddef']
      d = {}
      for fielddef in fdeflist:
        zval = self.ZeroFType(fielddef)
        if zval is not None:
          d[fielddef['name']] = zval
      return d
    else:
      return 0
  ##

  #--
  def FindFId(self, msgdef, fid):
    """ Find field definition with the given identifier in the (sub)message
        definition.

        Parameters:
          msgdef    - Message definition.
          fid       - Unique field id within message definition.

       Return:
        Returns field definition on success, None on failure.
    """
    fdeflist = msgdef['fielddef']
    for fielddef in fdeflist:
      if fielddef['fid'] == fid:
        return fielddef
    return None
  ##

  #--
  def Warning(self, *args):
    """ Issue warning message.

        Parameters:
          *args   - List of warning message arguments.
    """
    if self.mNoWarn:
      return
    wmsg = 'Warning'
    for a in args:
      if a:
        wmsg += ": %s" % (a)
    print(wmsg, file=sys.stderr)
  ##

  #--
  def Error(self, *args, **kwargs):
    """ Raise NetMsgsError error.

        Parameters:
          *args   - List of error message arguments.
          kwargs  - Keyword arguments.
                      stateId - Optional state id.
    """
    emsg = ''
    for a in args:
      if a:
        if not emsg:
          emsg += "%s" % (a)
        else:
          emsg += ": %s" % (a)
    self.StateDelete(kwargs.get('stateId', None))
    raise nmBase.NetMsgsError(emsg)
  ##

  #--
  def TraceStart(self, preface, msgid, msgdef, buf, stateId, fp=sys.stderr):
    """ Start of a message trace.

        Parameters:
          preface   - Preface string.
          msgid     - Message id.
          msgdef    - Message definition.
          buf       - Packed buffer.
          stateId   - Packing/unpacking state id.
          fp        - Opened output file pointer.
                          
    """
    msgname     = msgdef['name']
    max_count   = msgdef['max_count']
    field_count = self.StateGet(stateId, 'field_count')
    text  = ["--- %s Message %s(msgid=%u):\n" % (preface, msgname, msgid)]
    text += ["encoding=%s" % (self.mEncoding)]
    text += ["endian=%s"  % (self.mEndian)]
    text += ["field_count=%s" % repr(field_count)]
    text += ["max_count=%s" % repr(max_count)]
    fp.write('\n')
    self.Trace2Col(text, buf, stateId)
  ##

  #--
  def TraceField(self, fielddef, fval, buf, stateId, fp=sys.stderr):
    """ Trace a field.

        Parameters:
          fielddef  - Field definition.
          fval      - Field value.
          buf       - Packed buffer.
          stateId   - Packing/unpacking state id.
          fp        - Opened output file pointer.
    """
    fname   = fielddef['name']
    fid     = fielddef['fid']
    ftype   = fielddef['ftype']
    fval    = repr(fval)
    fstate  = self.StateFieldGet(stateId)
    xtype   = nmBase.NMFTypeCode2Xml.get(ftype, ftype)

    # print field value, info, and buffer
    if fstate['fpar'] == nmBase.NMFCode('vector'):     # vector item
      text  = ["%s[%u] = %s\t" % (xtype, fstate['i'], fval)]
    elif ftype == nmBase.NMFCode('pad'):        # pad
      count = fstate['count']
      text  = ["%s:%s(fid=%u, count=%u) =\t" % (xtype, 'pad', fid, count)]
    elif ftype == nmBase.NMFCode('string'):        # string
      max_count = fielddef['max_count']
      count     = fstate['count']
      text  = ["%s:%s(fid=%u) = %s\t" % (xtype, fname, fid, fval)]
      text += ["count=%s" % repr(count)]
      text += ["max_count=%s" % repr(max_count)]
    elif ftype == nmBase.NMFCode('vector'):        # vector
      vdef      = fielddef['vdef']
      vtype     = vdef['ftype']
      max_count = fielddef['max_count']
      count     = fstate['count']
      text  = ["%s:%s(fid=%u) =\t" % (xtype, fname, fid)]
      text += ["vtype=%s" % (vtype)]
      text += ["count=%s" % repr(count)]
      text += ["max_count=%s" % repr(max_count)]
    elif ftype == nmBase.NMFCode('struct'):        # struct
      msgdef    = fielddef['msgdef']
      max_count = msgdef['max_count']
      count     = fstate['count']
      text  = ["%s:%s(fid=%u) =\t" % (xtype, fname, fid)]
      text += ["count=%s" % repr(count)]
      text += ["max_count=%s" % repr(max_count)]
    else:                     # simple
      text  = ["%s:%s(fid=%u) = %s\t" % (xtype, fname, fid, fval)]

    self.Trace2Col(text, buf, stateId)
  ##

  #--
  def Trace2Col(self, textList, buf, stateId, fp=sys.stderr):
    """ Trace text and buffer in a two-column format.

        Column 1 contains the formatted text output. Column 2 contains the
        formatted contents of the buffer.

        Parameters:
          textList  - List of raw text phrases. The phrase are join to fit
                      into column 1. Special phrase end characters:
                        '\\t' = Do not append other phrases to this phrase.
                        '\\n' = Same as above, but also keep column 2 blank.
          buf       - Packed buffer.
          stateId   - Packing/unpacking state id.
          fp        - Opened output file pointer.
    """
    trace   = self.StateFieldGet(stateId, 'trace')
    start   = trace['trace_start']
    end     = trace['trace_end']
    if end is None:
      end = len(buf[start:])
    indent  = 2 * (trace['trace_depth'])
    indentb = indent + 5  # left column subsequent lines indentation
    col2    = 40          # column 2 start position
    text1   = []          # formatted left column text lines
    text2   = []          # formatted right column text lines

    # format column 1
    line = ''
    for phrase in textList:
      if phrase.endswith('\n') or phrase.endswith('\t'):
        sp      = space(indent)
        text1  += [sp+phrase]
        indent  = indentb
      elif not line:
        sp      = space(indent)
        line    = sp + phrase
        phrase  = ''
      elif len(line) + 2 + len(phrase) < col2-2:
        line   += ', ' + phrase
        phrase  = ''
      else:
        text1  += [line]
        indent  = indentb
        sp      = space(indent)
        line    = sp + phrase
    if line:
      text1 += [line]

    # format column 2
    if end is None:
      end = len(buf[start:])  # buf end position + 1
    n = end - start           # number of bytes
    while n > 0:
      if n >= 8:
        m = 8
      else:
        m = n
      s = ''
      i = 0
      while i < m:
        c = buf[start+i]
        s += "0x%02x " % (ord(c))
        i += 1
      text2 += [s]
      start += m
      n     -= m
        
    # output two column text
    col = 0
    i   = 0
    for line in text1:
      if line.endswith('\n'):
        fp.write(line)
        col = 0
        continue
      elif line.endswith('\t'):
        fp.write(line[:-1])
        col = len(line) - 1
      else:
        fp.write(line)
        col = len(line)
      if i < len(text2):
        if col > col2-2:
          fp.write('\n')
          col = 0
        fp.write(space(col2-col))
        fp.write(text2[i])
        i += 1
      fp.write('\n')

    while i < len(text2):
        fp.write(space(col2))       # space over to column 2
        fp.write(text2[i])          # line
        fp.write('\n')
        i += 1
  ##

  #--
  def TraceBufBlock(self, preface, buf, start=0, end=None, fp=sys.stderr):
    """ Trace a block bytes in packed buffer.

        Parameters:
          preface   - Preface string.
          buf         - Packed buffer.
          start       - Start position in buffer.
          end         - End position + 1 in buffer.
          fp        - Opened output file pointer.
    """
    if end is None:
      end = len(buf[start:])  # buf end position + 1
    n = end - start
    fp.write("%s Buffer (%u bytes):\n" % (preface, n))
    nmBase.PrintBuf(buf[start:], count=n, nlfreq=16, fp=fp)
  ##

  #--
  def DebugState(self, stateId):
    """ Debug print current operational state.

        Parameters:
          stateId   - State id.
    """
    nmBase.PrettyPrintDict(self.StateGet(stateId), name="StateId=%u" % stateId)
  ##

##
