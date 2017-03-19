#
# Package:  RoadNarrows Robotics I2C
#
# Module:   rnr.basetypes
#
# Link:     https://roadnarrows.com
#
# File:     basetypes.py
#

## \file
##
## $LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $  
## $Rev: 4278 $ 
## 
## \brief RoadNarrows Robotics I2C types module.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \copyright
##   \h_copy 2016-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
#
# @EulaBegin@
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
# @EulaEnd@
#

import rnr.i2ccore as i2ccore

#--
def sizeof(bsObj):
  """ Size of object in bytes. Size is contextual by object type. """
  return bsObj.__sizeof__()
##



#------------------------------------------------------------------------------
# CLASS: ByteBuf
#------------------------------------------------------------------------------
class ByteBuf:
  """ RNR Byte Buffer Class.
  
      The ByteBuf class provides a wrapper around the byte_t* SWIG object.
      The byte_t is typedef'ed in C as an unsigned char. Byte buffers are used
      in packing and unpacking buffers in function calls.

      Note: There does not seem to be any way to advance the pointer of the
            underlining byte_t* SWIG object, so a copy from/to a python byte
            list (buffer) is required.
  """

  #--
  def __init__(self, size=256):
    """ Initialize byte buffer instance.

        Parameters:
          size  - Number of bytes of the underlining fixed sized SWIG
                  object.
    """
    ## internal swig object
    self._swigObj = i2ccore.new_byteArray(size)

    ## internal swig object fixed buffer size
    self._size = size

    ## python byte string 
    self.buf = ''

  #--
  def __del__(self):
    """ Delete byte buffer instance. """
    i2ccore.delete_byteArray(self._swigObj)
  ##

  #--
  @staticmethod
  def Constructor(seq):
    """ Conversion constructor.

        Parameters:
          seq   - Sequence of values to convert. Sequence type can be an
                  integer [0,255], a string, or a list of elements of
                  type integer, string, list.

        Return:
          New initialized ByteBuf().
    """
    buf = ByteBuf.bytestring(seq)
    bytebuf = ByteBuf(size=len(buf))
    bytebuf.buf = buf
    return bytebuf

  #--
  @staticmethod
  def Clone(buf):
    """ Smart clone constructor.

        If buf is a ByteBuf instance, then simply return buf.
        Otherwise convert buf to a ByteBuf instance.

        The underlining swig object contents are not touched, so swig copies
        will need to be done prior to or after calling a byte_t* core routine.

        Parameters:
          buf   - Either a ByteBuf instance or a sequence of values to convert.
                  Sequence type can be an integer [0,255], a string, or a list
                  of elements of type integer, string, list.

        Return:
          Existing or new ByteBuf().
    """
    if isinstance(buf, ByteBuf):
      return buf
    else:
      return ByteBuf.Constructor(buf)
  ##

  #--
  def __getitem__(self, i):
    """ x.__getitem__(i)  <==> x[i] """
    return self.buf.__getitem__(i)
  ##

  #--
  def __getslice__(self, i, j):
    """ x.__getslice__(i, j)  <==> x[i:j] """
    return self.buf.__getslice__(i, j)
  ##

  #--
  def __iadd__(self, y):
    """ x.__iadd__(y)  <==> x+=y """
    z = ByteBuf.bytestring(y)
    self.buf.__iadd__(z)
    return self
  ##

  #--
  def __imul__(self, y):
    """ x.__imul__(y)  <==> x*=y """
    self.buf.__imul__(y)
    return self
  ##

  #--
  def __iter__(self):
    """ x.__iter__()  <==> iter(x) """
    return self.buf.__iter__()
  ##

  #--
  def __len__(self):
    """ x.__len__() <==> len(x)
        
        Number of bytes used in buffer. 
    """
    return len(self.buf)
  ##

  #--
  def __str__(self):
    """ x.__str__() <==> str(x) <==> print x """
    return self.buf.__str__()
  ##

  #--
  def __setitem__(self, i, y):
    """ x.__setitem__(i, y)  <==> x[i]=y """
    z = self.byte(y)
    self.buf.__setitem__(i, z)
    return self
  ##

  #--
  def __setslice__(self, i, j, y):
    """ x.__setslice__(i, j, y)  <==> x[i:j]=y """
    z = ByteBuf.bytestring(y)
    self.buf.__setslice__(i, j, z)
    return self
  ##

  #--
  def __sizeof__(self):
    """ x.__sizeof__() -- size of swig object byte buffer, in bytes. """
    return self._size
  ##

  #--
  def avail(self):
    """ Available bytes in fixed swig object buffer. """
    return self._size - len(self.buf)
  ##

  #--
  def copyFromSwigObj(self, n, i=0, j=0):
    """ Copy swig object buffer to python buffer.

        Parameters:
          n   - Number of bytes to copy.
          i   - Starting source index in swig object buffer.
          j   - Starting destination index in python buffer.
    """
    assert i+n <= self._size, "index out of range"
    l = len(self.buf)
    if l < j+n:
      self.buf += chr(0) * (j + n - l)
    s = ''
    for k in xrange(n):
      s += chr(i2ccore.byteArray_getitem(self._swigObj, i+k))
    self.buf = self.buf[:j] + s
  ##

  #--
  def copyToSwigObj(self, n, i=0, j=0):
    """ Copy python buffer to instance swig object buffer.

        Parameters:
          n   - Number of bytes to copy.
          i   - Starting source index in python buffer.
          j   - Starting destination index in swig object buffer.
    """
    assert i+n <= len(self.buf), "index out of range"
    assert j+n <= self._size, "index out of range"
    for k in xrange(n):
      i2ccore.byteArray_setitem(self._swigObj, j+k, ord(self.buf[i+k]))
  ##

  #--
  def getSwigObj(self):
    """ Return raw swig object. """
    return self._swigObj
  ##

  #--
  def getBuffer(self):
    """ Return python buffer. """
    return self.buf
  ##

  #--
  def sizeof(self):
    """ Size of fixed swig object byte buffer, in bytes. """
    return self.__sizeof__()
  ##

  #--
  @staticmethod
  def byte(val):
    """ Static method to convert a value into a byte.
    
        Parameters:
          val   - Value to convert. Value type can be an integer [0,255],
                  a string of length 1, or a list of length 1 of element
                  type of integer, string, list.

        Return:
          On success, returns converted byte value.
          On failure, a BotSenseError exception is raised.
    """
    if type(val) == int:
      assert val <= 255, "integer too big"
      return val
    elif type(val) == str:
      assert len(val) == 1, "string too long"
      return ord(val[0])
    elif type(val) == list:
      assert len(val) == 1, "too many list eleements"
      return ByteBuf.byte(val[0])
    else:
      assert 0, "unsupported type"
  ##

  #--
  @staticmethod
  def bytelist(seq):
    """ Static method to convert a value sequence into a byte list.

        Parameters:
          seq     - Sequence of values to convert. Sequence type can be an
                    integer [0,255], a string, or a list of elements of
                    type integer, string, list.

        Return:
          On success, returns converted byte value list.
          On failure, a BotSenseError exception is raised.
    """
    if seq is None:
      return []
    elif type(seq) == int:
      outbuf = [ ByteBuf.byte(seq) ]
      return outbuf
    elif type(seq) == str or type(seq) == list:
      outbuf = []
      for val in seq:
        outbuf += [ ByteBuf.byte(val) ]
      return outbuf
    else:
      assert 0, "unsupported type"
  ##

  #--
  @staticmethod
  def bytec(val):
    """ Static method to convert a value into a byte character string.
    
        Parameters:
          val   - Value to convert. Value type can be an integer [0,255],
                  a string of length 1, or a list of length 1 of element
                  type of integer, string, list.

        Return:
          On success, returns converted byte value.
          On failure, a BotSenseError exception is raised.
    """
    if type(val) == int:
      assert val <= 255, "integer too big"
      return chr(val)
    elif type(val) == str:
      assert len(val) == 1, "string too long"
      return val[0]
    elif type(val) == list:
      assert len(val) == 1, "too many list elements"
      return ByteBuf.bytec(val[0])
    else:
      assert 0, "unsupported type"
  ##

  #--
  @staticmethod
  def bytestring(seq):
    """ Static method to convert a value sequence into a byte string.

        Parameters:
          seq     - Sequence of values to convert. Sequence type can be an
                    integer [0,255], a string, or a list of elements of
                    type integer, string, list.

        Return:
          On success, returns converted byte value list.
          On failure, a BotSenseError exception is raised.
    """
    if seq is None:
      return ''
    elif type(seq) == int:
      return ByteBuf.bytec(seq)
    elif type(seq) == str or type(seq) == list:
      outbuf = ''
      for val in seq:
        outbuf += ByteBuf.bytec(val)
      return outbuf
    else:
      assert 0, "unsupported type"

