###############################################################################
#
# Package:  NetMsgs
#
# File: NetMsgsLib.py
#

""" 
NetMsgs Run-Time Library Packing and Unpacking Base Module
"""

## \file 
## \package NetMsgs.NetMsgsLib
##
## $LastChangedDate: 2010-08-04 15:07:55 -0600 (Wed, 04 Aug 2010) $
## $Rev: 550 $
##
## \brief NetMsgs Run-Time Library Packing and Unpacking Base Module
##
## The NetsMsgsLib module provides basic packing and unpacking functions plus
## some byte and bit utilities.
##
## \sa
## \htmlonly
##  <a href="../pydoc/NetMsgs.NetMsgsLib.html">PyDoc Generated Documentation</a>
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
import struct

import NetMsgsBase as nmBase

#
# Set filter for warnings.
# 
# DeprecationWarning:
#   Issued by strut module. Convert deprecation warning to error since it is
#   really an out-of-range error.
#
# RuntimeWarning:
#   NetMsgs run-time warnings are printed.
#
warnings.simplefilter('error', DeprecationWarning)
warnings.simplefilter('always', RuntimeWarning)


#-----------------------------------------------------------------------------
# Globals
#-----------------------------------------------------------------------------

#
# Error Messages
#

## bad integer identifier
EMsgBadId        = "Identifier not an integer"
## bad endian 
EMsgEndian       = "Invalid byte order value"
## buffer too small
EMsgNoMem        = "No space in buffer"
## bad value type
EMsgFValType     = "Invalid field value type"
## value not defined
EMsgNoFVal       = "Field value not defined"
## bad range
EMsgRange        = "Field value out-of-range"
## bad integer range
EMsgIntRange     = "Integer out-of-range"
## bad FPN range
EMsgFPNRange     = "Floating-point number out-of-range"
## bad string length
EMsgStringRange  = "String length out-of-range"
## bad struct count
EMsgStructRange  = "Message/Struct field count out-of-range"
## bad vector length
EMsgVectorRange  = "Vector length out-of-range"
## bad buffer
EMsgBufType      = "Invalid buffer type"
## no msg definition
EMsgNoMsgDef     = "Message definition not found"
## missing keyword
EMsgReqKeyword   = "Required definition keyword not found"
## index out of range
EMsgNoIndex      = "Required index out of range"
## bad message
EMsgBadMsg       = "Bad message format"


#-----------------------------------------------------------------------------
# Locals
#-----------------------------------------------------------------------------

## Message byte ordering type - struct module code equivalent
_EndianCode   = {'big':'>', 'little':'<', 'native':'='}

## Native endianess
_EndianNative = None 


#-----------------------------------------------------------------------------
# Utilities
#-----------------------------------------------------------------------------

#--
def GetNativeByteOrder():
  """ Get the native byte order.

      Return:
        'big' or 'little'
  """
  global _EndianNative
  if not _EndianNative:
    if PackU16(1, endian='native') == '\0x00\0x01':
      _EndianNative = 'big'
    else:
      _EndianNative = 'little'
  return _EndianNative
##


#-----------------------------------------------------------------------------
# Base Packing Functions
#-----------------------------------------------------------------------------

#--
def _PackException(ftype, val, endian, inst):
  """ Parse raised exception during packing, and re-raise as NetMsgError.

      Parameters:
        ftype     - Readable field type where exception occurred.
        val       - Field value.
        endian    - Byte order.
        inst      - Exception instance.
  """
  exception = type(inst)

  # endian is the only key that can cause this exception
  if exception == KeyError:
    raise nmBase.NetMsgsError("pack endian=%s: %s" % \
                              (repr(endian), EMsgEndian))

  # field value is the wrong type
  elif exception == TypeError:
    raise nmBase.NetMsgsError("pack %s=%s: %s" % \
                              (ftype, repr(val), EMsgFValType))

  # struct warning format: "'X' format requires m <= number <= M"
  elif exception == DeprecationWarning:
    emsg = inst.args[0]
    emsg = emsg.partition('format requires ')[2]
    emsg = EMsgRange + ' ' + emsg
    raise nmBase.NetMsgsError("pack %s=%s: %s" % (ftype, repr(val), emsg))

  # other exception
  else:
    raise nmBase.NetMsgsError("pack %s: %s" % (ftype, inst))
##

#--
def PackBool(val, endian='big'):
  """ Pack boolean into 1 byte.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if val:
    return PackU8(1, endian=endian)
  else:
    return PackU8(0, endian=endian)
##

#--
def PackChar(val, endian='big'):
  """ Pack one 8-bit ASCII character into 1 byte.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack('c', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('char', val, endian, inst)
##

#--
def PackU8(val, endian='big'):
  """ Pack an 8-bit unsigned integer into 1 byte.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if (type(val) == str) and len(val) == 1:
    val = ord(val)
  try:
    return struct.pack(_EndianCode[endian]+'B', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('u8', val, endian, inst)
##

#--
def PackS8(val, endian='big'):
  """ Pack an 8-bit signed integer into 1 byte.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if (type(val) == str) and len(val) == 1:
    val = ord(val)
  try:
    return struct.pack(_EndianCode[endian]+'b', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('s8', val, endian, inst)
##

#--
def PackU16(val, endian='big'):
  """ Pack a 16-bit unsigned integer into 2 bytes.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'H', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('u16', val, endian, inst)
##

#--
def PackS16(val, endian='big'):
  """ Pack a 16-bit signed integer into 2 bytes.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'h', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('s16', val, endian, inst)
##

#--
def PackU32(val, endian='big'):
  """ Pack a 32-bit unsigned integer into 4 bytes.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'I', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('u32', val, endian, inst)
##

#--
def PackS32(val, endian='big'):
  """ Pack a 32-bit signed integer into 4 bytes.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'i', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('s32', val, endian, inst)
##

#--
def PackU64(val, endian='big'):
  """ Pack a 64-bit unsigned integer into 8 bytes.

      If native architectures only support 4 byte long long's, then the 4 MSBs
      are padded with zero's.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if struct.calcsize('Q') == 8:
    try:
      return struct.pack(_EndianCode[endian]+'Q', val)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _PackException('u64', val, endian, inst)
  elif endian == 'big' or \
      (endian == 'native' and GetNativeByteOrder() == 'big'):
    return PackU32(0, endian='big') + PackU32(val, endian='big')
  else: # little
    return PackU32(val, endian='little') + PackU32(0, endian='little')
##

#--
def PackS64(val, endian='big'):
  """ Pack a 64-bit signed integer into 8 bytes.

      If native architectures only support 4 byte long long's, then the 4 MSBs
      are padded with zero's.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if struct.calcsize('q') == 8:
    try:
      return struct.pack(_EndianCode[endian]+'q', val)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _PackException('s64', val, endian, inst)
  if val & 0x80000000:
    msb = 0xffffffff      # negative value, extend sign
  else:
    msb = 0               # positive value
  if endian == 'big' or (endian == 'native' and GetNativeByteOrder() == 'big'):
    return PackS32(msb, endian='big') + PackS32(val, endian='big')
  else: # little
    return PackS32(val, endian='little') + PackS32(msb, endian='little')
##

#--
def PackF32(val, endian='big'):
  """ Pack a 32-bit floating-point number into 4 bytes.

      Is this guaranteed to be in IEEE 745 32-bit format?

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'f', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('f32', val, endian, inst)
##

#--
def PackF64(val, endian='big'):
  """ Pack a 64-bit floating-point number into 8 bytes.

      Is this guaranteed to be in IEEE 745 64-bit format?
      Will this pack into 8 bytes on 32-bit architectures?

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  try:
    return struct.pack(_EndianCode[endian]+'d', val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('f64', val, endian, inst)
##

#--
def PackP32(val, endian='big'):
  """ Pack a 32-bit floating-point number into 4 bytes.

      Note:
        Pointers are really foreign to python. However, a python application
        may have a linked c library that requires it. And NetMsgs supports it.

      Note:
        Pointers are always packed in native byte order.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if struct.calcsize('L') == 4:
    try:
      return struct.pack(_EndianCode['native']+'L', val)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _PackException('p32', val, endian, inst)
  else:
    msb = val >> 32
    lsb = val & 0xffffffff
    if msb != 0:
      warnings.warn("pack p32=%u: non-zero 4 MSB's truncated" % (val),
          RuntimeWarning)
    try:
      return struct.pack(_EndianCode['native']+'I', lsb)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _PackException('p32', val, endian, inst)
##

#--
def PackP64(val, endian='big'):
  """ Pack a 64-bit floating-point number into 8 bytes.

      Note:
        Pointers are really foreign to python. However, a python application
        may have a linked c library that requires it. And NetMsgs supports it.

      Note:
        Pointers are always packed in native byte order.

      Parameters:
        val       - Field value.
        endian    - Byte order.

      Return:
        Binary string.
  """
  if struct.calcsize('L') == 8:
    try:
      return struct.pack(_EndianCode['native']+'Q', val)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _PackException('p64', val, endian, inst)
  elif GetNativeByteOrder() == 'big':
    return PackU32(0, endian='big') + PackU32(val, endian='big')
  else: # little
    return PackU32(val, endian='little') + PackU32(0, endian='little')
##

#--
def PackString(val, count=None):
  """ Pack a string.

      The endianess is ignore.
      The terminating Null is not packed.

      Parameters:
        val       - Field value.
        count     - Number of bytes to pack. None == all.

      Return:
        Binary string.
  """
  try:
    n = len(val)
    if count is None:
      count = n
    elif count > n:
      val += (count-n) * '\x00'
  except (TypeError) as inst:
    _PackException('string', val, endian, inst)
  try:
    return struct.pack("%us" % (count), val)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _PackException('string', val, endian, inst)
##


#-----------------------------------------------------------------------------
# Base Unpacking Functions
#-----------------------------------------------------------------------------

#--
def _UnpackException(ftype, offset, endian, inst):
  """ Parse raised exception during unpacking, and re-raise as NetMsgError.

      Parameters:
        ftype     - Readable field type where exception occurred.
        offset    - Offset in buffer where error occurred.
        endian    - Byte order.
        inst      - Exception instance.
  """
  exception = type(inst)

  # endian is the only key that can cause this exception
  if exception == KeyError:
    raise nmBase.NetMsgsError("unpack endian=%s: %s" % \
                              (repr(endian), EMsgEndian))

  # field value is the wrong type
  elif exception == TypeError:
    raise nmBase.NetMsgsError("unpack %s: offset=%u: %s" % \
                              (ftype, offset, EMsgBufType))

  # struct error format: "unpack_from ... "
  elif exception == struct.error:
    emsg = inst.args[0]
    emsg = emsg.partition('unpack_from ')[2]
    emsg = 'Field ' + emsg
    raise nmBase.NetMsgsError("unpack %s: offset=%u: %s" % \
                              (ftype, offset, emsg))

  # other exception
  else:
    raise nmBase.NetMsgsError("unpack %s: offset=%u: %s" % \
                              (ftype, offset, inst))
##

#--
def UnpackBool(buf, offset=0, endian='big'):
  """ Unpack boolean into 1 byte.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  val, offset = UnpackU8(buf, offset=offset, endian=endian)
  if val:
    return True, offset
  else:
    return False, offset
##

#--
def UnpackChar(buf, offset=0, endian='big'):
  """ Unpack one 8-bit ASCII character into 1 byte.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'c', buf, offset)[0],
            offset+1)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('char', offset, endian, inst)
##

#--
def UnpackU8(buf, offset=0, endian='big'):
  """ Unpack an 8-bit unsigned integer into 1 byte.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'B', buf, offset)[0],
            offset+1)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('u8', offset, endian, inst)
##

#--
def UnpackS8(buf, offset=0, endian='big'):
  """ Unpack an 8-bit signed integer into 1 byte.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'b', buf, offset)[0],
            offset+1)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('s8', offset, endian, inst)
##

#--
def UnpackU16(buf, offset=0, endian='big'):
  """ Unpack a 16-bit unsigned integer into 2 bytes.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'H', buf, offset)[0],
            offset+2)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('u16', offset, endian, inst)
##

#--
def UnpackS16(buf, offset=0, endian='big'):
  """ Unpack a 16-bit signed integer into 2 bytes.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'h', buf, offset)[0],
            offset+2)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('s16', offset, endian, inst)
##

#--
def UnpackU32(buf, offset=0, endian='big'):
  """ Unpack a 32-bit unsigned integer into 4 bytes.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'I', buf, offset)[0],
            offset+4)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('u32', offset, endian, inst)
##

#--
def UnpackS32(buf, offset=0, endian='big'):
  """ Unpack a 32-bit signed integer into 4 bytes.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'i', buf, offset)[0],
            offset+4)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('s32', offset, endian, inst)
##

#--
def UnpackU64(buf, offset=0, endian='big'):
  """ Unpack a 64-bit unsigned integer into 8 bytes.

      If native architectures only support 4 byte long long's, then the 4 MSBs
      are padded with zero's.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  if struct.calcsize('Q') == 8:
    try:
      return (struct.unpack_from(_EndianCode[endian]+'Q', buf, offset)[0],
              offset+8)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _UnpackException('u64', offset, endian, inst)
  elif endian == 'big' or \
      (endian == 'native' and GetNativeByteOrder() == 'big'):
    msb, offset = UnpackU32(buf, offset=offset, endian='big')
    lsb, offset = UnpackU32(buf, offset=offset, endian='big')
  else: # little
    lsb, offset = UnpackU32(buf, offset=offset, endian='little')
    msb, offset = UnpackU32(buf, offset=offset, endian='little')
  if msb != 0:
    warnings.warn("unpack u64=0x%08x%08x: non-zero 4 MSB's truncated" % \
        (msb, lsb), RuntimeWarning)
  return (lsb, offset)
##

#--
def UnpackS64(buf, offset=0, endian='big'):
  """ Unpack a 64-bit signed integer into 8 bytes.

      If native architectures only support 4 byte long long's, then the 4 MSBs
      are padded with zero's.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  if struct.calcsize('q') == 8:
    try:
      return (struct.unpack_from(_EndianCode[endian]+'q', buf, offset)[0],
              offset+8)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _UnpackException('s64', offset, endian, inst)
  elif endian == 'big' or \
      (endian == 'native' and GetNativeByteOrder() == 'big'):
    msb, offset = UnpackU32(buf, offset=offset, endian='big')
    lsb, offset = UnpackU32(buf, offset=offset, endian='big')
  else: # little
    lsb, offset = UnpackU32(buf, offset=offset, endian='little')
    msb, offset = UnpackU32(buf, offset=offset, endian='little')
  if (msb != 0) and (msb != 0xffffffff): # test zero or sign extended msb
    warnings.warn("unpack s64=0x%08x%08x: non-zero 4 MSB's truncated" % \
        (msb, lsb), RuntimeWarning)
  return (lsb, offset)
##

#--
def UnpackF32(buf, offset=0, endian='big'):
  """ Unpack a 32-bit floating-point number into 4 bytes.

      Is this guaranteed to be in IEEE 745 32-bit format?

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'f', buf, offset)[0],
            offset+4)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('f32', offset, endian, inst)
##

#--
def UnpackF64(buf, offset=0, endian='big'):
  """ Unpack a 64-bit floating-point number into 8 bytes.

      Is this guaranteed to be in IEEE 745 64-bit format?
      Will this pack into 8 bytes on 32-bit architectures?

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode[endian]+'d', buf, offset)[0],
            offset+8)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('f64', offset, endian, inst)
##

#--
def UnpackP32(buf, offset=0, endian='big'):
  """ Unpack a 32-bit floating-point number into 4 bytes.

      Note:
        Pointers are really foreign to python. However, a python application
        may have a linked c library that requires it. And NetMsgs XML and
        run-time C library supports it.

      Note:
        Pointers are always unpacked in native byte order.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from(_EndianCode['native']+'I', buf, offset)[0],
            offset+4)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('p32', offset, endian, inst)
##

#--
def UnpackP64(buf, offset=0, endian='big'):
  """ Unpack a 64-bit floating-point number into 8 bytes.

      Note:
        Pointers are really foreign to python. However, a python application
        may have a linked c library that requires it. And NetMsgs supports it.

      Note:
        Pointers are always unpacked in native byte order.

      Parameters:
        buf       - Input packed buffer.
        offset    - Offset in buffer.
        endian    - Byte order.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  if struct.calcsize('L') == 8:
    try:
      return (struct.unpack_from(_EndianCode[endian]+'L', buf, offset)[0],
              offset+8)
    except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
      _UnpackException('u8', offset, endian, inst)
  elif (endian == 'native') and (GetNativeByteOrder() == 'big'):
    msb, offset = UnpackU32(buf, offset=offset, endian='big')
    lsb, offset = UnpackU32(buf, offset=offset, endian='big')
  else: # little
    lsb, offset = UnpackU32(buf, offset=offset, endian='little')
    msb, offset = UnpackU32(buf, offset=offset, endian='little')
  if msb != 0:
    warnings.warn("unpack p64=0x%08x%08x: non-zero 4 MSB's truncated" % \
        (msb, lsb), RuntimeWarning)
  return (lsb, offset)
##

#--
def UnpackString(buf, count, offset=0):
  """ Unpack a string.

      The endianess is ignore.
      The terminating Null is not packed.

      Parameters:
        buf       - Input packed buffer.
        count     - Number of bytes to unpack.
        offset    - Offset in buffer.

      Return:
        2-tuple of unpacked value, new buffer offset.
  """
  try:
    return (struct.unpack_from("%us" % (count), buf, offset)[0], offset+count)
  except (KeyError, TypeError, DeprecationWarning, struct.error) as inst:
    _UnpackException('string', offset, 'big', inst)
##
