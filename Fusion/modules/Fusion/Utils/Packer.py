################################################################################
#
# Packer.py
#

""" Byte stream packer/unpacker module.

Byte stream packer/unpacker class.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.11.11

Copyright (C) 2007, RoadNarrows LLC.
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
# CLASS: Packer
#-------------------------------------------------------------------------------
class Packer:
  """ Byte stream packer/unpacker class.
  """

  #--
  def __init__(self):
    """ Initialize packer instance.
    """
    pass

  #--
  def PackMsg(self, packfmt):
    """ Byte pack a message.

        Parameters:
          packfmt   - Packing format list with each entry a field 
                      format dictionary.  Field format keys:
                        'fval': field value or list of field values of ftype
                        'ftype':  field format type. One of:
                          'u8'    - unsigned 8 bits, big-endian
                          'u16'   - unsigned 16 bits, big-endian
                          'u32'   - unsigned 32 bits, big-endian
                          'char'  - single ascii character
                          'zstr'  - zero-terminated character string
            

        Return Value:
          List of packed bytes.
    """
    msg = []
    for ffmt in packfmt:
      fval  = ffmt['fval']
      ftype = ffmt.get('ftype', 'u8')
      if type(fval) == list:
        for val in fval:
          msg += self.PackField(val, ftype)
      else:
          msg += self.PackField(fval, ftype)
    return msg

  #--
  def PackField(self, fval, ftype):
    """ Byte pack field.

        Parameters:
          fval      - field value
          ftype     - field format type (see PackMsg())

        Return value:
          List of packed bytes.
    """
    if ftype == 'u8':
      return self.PackU8(fval)
    elif ftype == 'u16':
      return self.PackU16(fval)
    elif ftype == 'u32':
      return self.PackU32(fval)
    elif ftype == 'char':
      return self.PackChar(fval)
    elif ftype == 'zstr':
      return self.PackZStr(fval)
    else:
      raise TypeError(ftype)

  #--
  def PackU8(self, fval):
    """ Pack 8-bit unsigned integer into 1 byte.

        Parameters:
          fval      - field value

        Return value:
          List of packed bytes.
    """
    if type(fval) == str:
      fval = ord(fval)
    return [fval & 0xff]

  #--
  def PackU16(self, fval):
    """ Pack 16-bit unsigned integer into 2 bytes.

        Parameters:
          fval      - field value

        Return value:
          List of packed bytes.
    """
    msg  = []
    msg += [((fval >> 8) & 0xff)]
    msg += [(fval & 0xff)]
    return msg

  #--
  def PackU32(self, fval):
    """ Pack 32-bit unsigned integer into 4 bytes.

        Parameters:
          fval      - field value

        Return value:
          List of packed bytes.
    """
    msg  = []
    msg += [((fval >> 24) & 0xff)]
    msg += [((fval >> 16) & 0xff)]
    msg += [((fval >>  8) & 0xff)]
    msg += [(fval & 0xff)]
    return msg

  #--
  def PackChar(self, fval):
    """ Pack 8-bit ascii character into 1 byte.

        Parameters:
          fval      - field value

        Return value:
          List of packed bytes.
    """
    return ["%c" % fval]

  #--
  def PackZStr(self, fval):
    """ Pack zero-terminated string into len(fval) + 1 bytes.

        Parameters:
          fval      - field value

        Return value:
          List of packed bytes.
    """
    msg = []
    for c in fval:
      msg += [ord(c)]
    msg += [0]
    return msg

  #--
  def UnpackMsg(self, msg, unpackfmt, pos=0):
    """ Unpack a message into field value(s).

        Parameters:
          msg       - byte message (list or str).
          unpackfmt - Unpacking format list with each entry a field 
                      format dictionary.  Field format keys:
                        'fname':  field name.
                        'ftype':  field format type. One of:
                          'u8'    - unsigned 8 bits, big-endian
                          'u16'   - unsigned 16 bits, big-endian
                          'u32'   - unsigned 32 bits, big-endian
                          'char'  - single ascii character
                          'zstr'  - zero-terminated character string
                          DEFAULT: 'u8'
                        'fcount':   number of field instances of ftype.
                          DEFAULT: 1
          pos       - starting byte index position in message
            
        Return Value:
          On successful unpacking, dictionary of format:
            {'_rc':'ok', '_pos':<pos>, <fname>:<val> ...} where
              <pos> is the message index after the last unpacked field.
          On an unpacking error, dictionary of format:
            {'_rc':'error', '_pos':<pos>, '_error_msg:<emsg> ...} where
              <pos> is the index in the message where the error occurred
              <emsg> is the specific error message strring
              ... are zero or more unpacked <fname>:<val> prior to error.
    """
    unpackedVals = {}
    lenMsg = len(msg)
    for ffmt in unpackfmt:
      if pos >= lenMsg:
        self._error(unpackedVals, pos, "message too short to unpack all fields")
        return unpackedVals
      fname  = ffmt['fname']
      ftype  = ffmt.get('ftype', 'u8')
      fcount = ffmt.get('fcount', 1)
      i = 0
      while i < fcount:
        val = self.UnpackField(msg, ftype, pos)
        if val['_rc'] == 'error':
          self._error(unpackedVals, val['_pos'], val['_error_msg'])
          return unpackedVals
        elif fcount > 1:
          if i == 0:
            unpackedVals[fname] = []
          unpackedVals[fname] += [val['fval']]
        else:
          unpackedVals[fname] = val['fval']
        pos = val['_pos']
        i += 1
    unpackedVals['_rc']   = 'ok'
    unpackedVals['_pos']  = pos
    return unpackedVals

  #--
  def UnpackField(self, msg, ftype, pos=0):
    """ Unpack byte messages into field value.

        Parameters:
          msg       - byte message (list or str).
          ftype     - field format type (see UnpackMsg())
          pos       - starting byte index position in message

        Return value:
          On successful unpacking, dictionary of format:
            {'_rc':'ok', '_pos':<pos>, '_fval':<val>} where
              <pos> is the message index after the last unpacked field.
              <val> is the unpacked field value.
          On an unpacking error, dictionary of format:
            {'_rc':'error', '_pos':<pos>, '_error_msg:<emsg> ...} where
              <pos> is the index in the message where the error occurred
              <emsg> is the specific error message strring
              ... are zero or more unpacked <fname>:<val> prior to error.
    """
    if ftype == 'u8':
      return self.UnpackU8(msg, pos)
    elif ftype == 'u16':
      return self.UnpackU16(msg, pos)
    elif ftype == 'u32':
      return self.UnpackU32(msg, pos)
    elif ftype == 'char':
      return self.UnpackChar(msg, pos)
    elif ftype == 'zstr':
      return self.UnpackZStr(msg, pos)
    else:
      raise TypeError(ftype)

  #--
  def UnpackU8(self, msg, pos=0):
    """ Unpack 1 byte into 8-bit unsigned  integer.

        Parameters:
          msg       - byte message (list or str).
          pos       - starting byte index position in message

        Return value:
          See UnpackField().
    """
    unpackedVal = {}
    if pos < len(msg):
      unpackedVal['_rc']  = 'ok'
      unpackedVal['_pos'] = pos+1
      if type(msg[pos]) == str:
        unpackedVal['fval'] = ord(msg[pos])
      else:
        unpackedVal['fval'] = (msg[pos] & 0xff)
    else:
      self._error(unpackedVal, pos, "message too short to unpack u8")
    return unpackedVal

  #--
  def UnpackU16(self, msg, pos=0):
    """ Unpack 2 bytes into 16-bit unsigned  integer.

        Parameters:
          msg       - byte message (list or str).
          pos       - starting byte index position in message

        Return value:
          See UnpackField().
    """
    unpackedVal = {}
    if pos+1 < len(msg):
      fval = ((ord(msg[pos]) << 8) | (ord(msg[pos+1]) & 0xff)) & 0xffff;
      unpackedVal['_rc']  = 'ok'
      unpackedVal['_pos'] = pos+2
      unpackedVal['fval'] = fval
    else:
      self._error(unpackedVal, pos, "message too short to unpack u16")
    return unpackedVal

  #--
  def UnpackU32(self, msg, pos=0):
    """ Unpack 4 bytes into 32-bit unsigned  integer.

        Parameters:
          msg       - byte message (list or str).
          pos       - starting byte index position in message

        Return value:
          See UnpackField().
    """
    unpackedVal = {}
    if pos+3 < len(msg):
      fval  = (ord(msg[pos])   << 24)
      fval |= (ord(msg[pos+1]) << 16)
      fval |= (ord(msg[pos+2]) <<  8)
      fval |=  ord(msg[pos+3])
      unpackedVal['_rc']  = 'ok'
      unpackedVal['_pos'] = pos+4
      unpackedVal['fval'] = fval
    else:
      self._error(unpackedVal, pos, "message too short to unpack u32")
    return unpackedVal

  #--
  def UnpackChar(self, msg, pos=0):
    """ Unpack 1 byte into 8-bit ascii character (string of length 1).

        Parameters:
          msg       - byte message (list or str).
          pos       - starting byte index position in message

        Return value:
          See UnpackField().
    """
    unpackedVal = {}
    if pos < len(msg):
      unpackedVal['_rc']  = 'ok'
      unpackedVal['_pos'] = pos+1
      unpackedVal['fval'] = "%c" % msg[pos]
    else:
      self._error(unpackedVal, pos, "message too short to unpack u8")
    return unpackedVal

  #--
  def UnpackZStr(self, msg, pos=0):
    """ Unpack bytes in message to the zero byte value into string.

        Parameters:
          msg       - byte message (list or str).
          pos       - starting byte index position in message

        Return value:
          See UnpackField().
    """
    unpackedVal = {}
    if pos < len(msg):
      try:
        if type(msg) == str:
          i = msg.find('\0', pos)
          if i < 0:
            raise ValueError
        else:
          i = msg.index(0, pos)
      except:
        self._error(unpackedVal, pos, "null-terminated string not found")
        return unpackedVal
      fval = ''
      while pos < i:
        fval += "%c" % msg[pos]
        pos += 1
      unpackedVal['_rc']  = 'ok'
      unpackedVal['_pos'] = pos+1
      unpackedVal['fval'] = fval
    else:
      self._error(unpackedVal, pos, "message too short to unpack zstr")
    return unpackedVal

  #--
  def DumpBytes(self, msg):
    """ Dump message bytes to stdout.

        Parameters:
          msg       - byte message (list or str).
    """
    for byte in msg:
      if type(byte) == str:
        byte = ord(byte)
      if byte >= 0x20 and byte <= 0x7e:
        print("%c" % byte, end='')
      else:
        print("0x%02x" % byte, end='')
    print()

  #--
  def _error(self, unpackedVals, pos, error_msg):
    """ Set error message in unpacked values dictionary.

        Parameters:
          unpackedVals  - See UnpackMsg()
          pos           - Error byte position in message.
          error_msg     - Error message string.
    """
    unpackedVals['_rc']        = 'error'
    unpackedVals['_pos']       = pos
    unpackedVals['_error_msg'] = error_msg
    return unpackedVals
