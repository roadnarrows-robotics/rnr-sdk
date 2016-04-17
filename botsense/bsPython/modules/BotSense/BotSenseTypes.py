###############################################################################
#
# Package:  BotSense
#
# File: BotSenseTypes.py
#

""" 
BotSense Type Classes.
"""

## \file 
## \package BotSense.BotSenseTypes
##
## $LastChangedDate: 2012-12-11 14:28:19 -0700 (Tue, 11 Dec 2012) $
## $Rev: 2573 $
##
## \brief BotSense Types 
##
## \sa
## \htmlonly
##  <a href="../pydoc/BotSense.BotSenseError.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2010.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
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
import BotSenseError as bsError

#--
def sizeof(bsObj):
  """ Size of BotSense object in bytes. Size is contextual by object type. """
  return bsObj.__sizeof__()
##


#------------------------------------------------------------------------------
# CLASS: BsClient
#------------------------------------------------------------------------------
class BsClient:
  """ BotSense Client Wrapper Class.
  
      The BsClient class provides a safe wrapper around the BsClient_T *
      opaque object.
  """

  #--
  def __init__(self, clientName='bsclient'):
    """ 
      Create and initialize BotSense client instance.

      Parameters:
        clientName  - Proxied client name string.
    """
    ## Pointer to opaque object
    self.pClient = None

    pClient = bsCore.bsClientNew(clientName)
    if not pClient:
      raise bsError.BotSenseError(BS_ECODE_NO_RSRC, "Failed to create client.")

    self.pClient = pClient
  ##

  #--
  def __del__(self):
    """ Delete BotSense client instance. """
    if self.pClient is not None:
      bsCore.bsClientDelete(self.pClient)
    self.pClient = None
  ##

  #--
  def __sizeof__(self):
    """ x.__sizeof__() -- size of swig object, in bytes. """
    if self.pClient is not None:
      return 1    # opaque size
    else:
      return 0    # opaque size
  ##

  #--
  def AttrGetName(self):
    """ Get the client's name.

        Return:
          On success, returns client string name.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    clientName = bsCore.bsClientAttrGetName(self.pClient)
    if not clientName:
      clientName = 'unknown'
    return clientName
  ##

  #--
  def AttrGetTraceState(self, hndVConn):
    """ Get a client's virtual connection trace state.

        Parameters:
          hndVConn  - Virtual connection handle.

        Return:
          On success, returns True (enabled) or False (disabled).
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    state = bsCore.bsClientAttrGetTraceState(self.pClient, hndVConn)
    if state:
      return True
    else:
      return False
  ##

  #--
  def AttrGetVConnCount(self):
    """ Get a client's virtual connection trace state.

        Return:
          On success, returns client's virtual connection count.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    cnt = bsCore.bsClientAttrGetVConnCount(self.pClient)
    return cnt
  ##

  #--
  def AttrHasVConn(self, hndVConn):
    """ Check if the client has a virtual connection associated with the
        given handle.

        Parameters:
          hndVConn  - Virtual connection handle.

        Return:
          On success, returns True or False.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    tf = bsCore.bsClientAttrHasVConn(self.pClient, hndVConn)
    if tf:
      return True
    else:
      return False
  ##

  #--
  def AttrGetDevName(self, hndVConn):
    """ Get the proxied device name associated with the given handle.

        Parameters:
          hndVConn  - Virtual connection handle.

        Return:
          On success, returns the device URI string. If no device is found,
          then '#nodev#' is returned.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    devName = bsCore.bsClientAttrGetDevName(self.pClient, hndVConn)
    if not devName:
      devName = '#nodev#'
    return devName
  ##

  #--
  def AttrGetModName(self, hndVConn):
    """ Get the proxied interface module name associated with the given
        handle.

        Parameters:
          hndVConn  - Virtual connection handle.

        Return:
          On success, returns the I/F module URI string. If no module is
          found, then '#nomod#' is returned.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    modName = bsCore.bsClientAttrGetModName(self.pClient, hndVConn)
    if not modName:
      modName = '#nomod#'
    return modName
  ##

  #--
  def AttrSetLogging(self, nLevel):
    """ Set the client's diagnostic logging threshold.

        Parameters:
          nLevel  - New logging threshold level.
    """
    self._ChkSelf()
    bsCore.bsClientAttrSetLogging(self.pClient, nLevel)
  ##

  #--
  def AttrGetConnState(self):
    """ Get the client's connection state.

        Return:
          Dictionary specifying connection state.
    """
    self._ChkSelf()
    connstate = bsCore.BsClientConnState_T()
    bsCore.bsClientAttrGetConnState(self.pClient, connstate)
    d = {}
    if connstate.m_bIsConnected:
      d['is_connected'] = True
    else:
      d['is_connected']   = False
    d['server_hostname']  = connstate.m_sServerHostName
    return d
  ##

  #--
  def AttrGetTimeouts(self):
    """ Get the client's request and response timeouts in seconds.
    """
    self._ChkSelf()
    reqTimeout = bsCore.new_uintp();
    rspTimeout = bsCore.new_uintp();
    bsCore.bsClientAttrGetTimeouts(self.pClient, reqTimeout, rspTimeout)
    ret = (bsCore.uintp_value(reqTimeout), bsCore.uintp_value(rspTimeout))
    bsCore.delete_uintp(reqTimeout);
    bsCore.delete_uintp(rspTimeout);
    return ret
  ##

  #--
  def AttrSetTimeouts(self, reqTimeout, rspTimeout):
    """ Set the client's request and response timeouts in seconds.

        Parameters:
          reqTimeout  - Request (write) timeout.
          rspTimeout  - Response (read) timeout.
    """
    self._ChkSelf()
    bsCore.bsClientAttrSetTimeouts(self.pClient, reqTimeout, rspTimeout)
  ##

  #--
  def GetMsgName(self, hndVConn, msgId):
    """ Get the message name string.

        For each (virtual connection, message id) 2-tuple, the message
        name can be determinied (provided the msgid is valid and an
        application provides the information).

        Parameters:
          hndVConn  - Virtual connection handle.
          msgId     - Message id.

        Return:
          On success, returns the message name string. If the name cannot
          be determined, then 'unknown' is returned.
          On parameter check failure, a BotSenseError exception is raised.
    """
    self._ChkSelf()
    return bsCore.bsClientGetMsgName(self.pClient, hndVConn, msgId)
  ##

  #--
  def Trans(self, hndVConn, reqMsgId, reqBuf, rspMsgId,
                  rspMaxSize=bsCore.BSPROXY_MSG_BODY_MAX):
    """ Execute a request - response transaction with the server.

        The request message header is automatically generated. The response
        message header is stripped off from the received response.

        Raises a BotSenseError exception on failure.
         
        Parameters:
          hndVConn    - Virtual connection handle.
          reqMsgId    - Virtual connection unique request message id.
          reqBuf      - Packed request message body.
          rspMsgId    - Virtual connection expected response message id.
          rspMaxSize  - Maximum expected response body size.

        Return:
          Packed response message body buffer.
    """
    self._ChkSelf()
    reqByteBuf = ByteBuf.Clone(reqBuf)
    # leave space in buffer to prepend request message header
    reqByteBuf.copyToSwigObj(len(reqByteBuf), i=0, j=bsCore.BSPROXY_MSG_HDR_LEN)
    rspByteBuf = ByteBuf(rspMaxSize)
    n = bsCore.bsClientTrans(self.pClient, hndVConn,
                          reqMsgId, reqByteBuf.getSwigObj(), len(reqByteBuf),
                          rspMsgId, rspByteBuf.getSwigObj(), sizeof(rspByteBuf))
    bsError.ChkReturnIsNonNeg(n,
      "Client %s: Transaction request failed." % (self.AttrGetName()))
    rspByteBuf.copyFromSwigObj(n)
    return rspByteBuf.buf
  ##

  #--
  def _ChkSelf(self):
    """ Check that the opaque 'real' client exists. """ 
    if self.pClient is None:
      raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "No client.")
  ##

  #--
  @staticmethod
  def ChkClient(obj):
    """ Static check if object is a valid BsClient object.
    
        Parameters:
            obj   - Object instance.
    """
    bsError.ChkType(obj, BsClient)
    obj._ChkSelf()
  ##
##


#------------------------------------------------------------------------------
# CLASS: ByteBuf
#------------------------------------------------------------------------------
class ByteBuf:
  """ BotSense Byte Buffer Class.
  
      The ByteBuf class provides a wrapper around the byte_t* SWIG object.
      The byte_t is typedef'ed in C as an unsigned char. Byte buffers are used
      in packing and unpacking messages and in communication between the
      client and server.

      Note: There does not seem to be any way to advance the pointer of the
            underlining byte_t* SWIG object, so a copy from/to a python byte
            list (buffer) is required.
  """

  #--
  def __init__(self, size=bsCore.BSPROXY_MSG_BODY_MAX):
    """ Initialize byte buffer instance.

        Parameters:
          size  - Number of bytes of the underlining fixed sized SWIG
                  object.
    """
    # add header length
    size += bsCore.BSPROXY_MSG_HDR_LEN

    ## internal swig object
    self._swigObj = bsCore.new_byteArray(size)

    ## internal swig object fixed buffer size
    self._size = size

    ## python byte string 
    self.buf = ''
  ##

  #--
  def __del__(self):
    """ Delete byte buffer instance. """
    bsCore.delete_byteArray(self._swigObj)
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
    if i+n > self._size:
      raise bsError.BotSenseError(bsCore.BS_ECODE_BUF_TOO_SMALL,
          "copyFromSwigObj: swig object buffer size=%d < %d bytes" % \
              (self._size, i+n))
    l = len(self.buf)
    if l < j+n:
      self.buf += chr(0) * (j + n - l)
    s = ''
    for k in xrange(n):
      s += chr(bsCore.byteArray_getitem(self._swigObj, i+k))
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
    if i+n > len(self.buf):
      raise bsError.BotSenseError(bsCore.BS_ECODE_BUF_TOO_SMALL,
          "copyToSwigObj: python buffer size=%d < %d bytes" % \
              (len(self.buf), i+n))
    elif j+n > self._size:
      raise bsError.BotSenseError(bsCore.BS_ECODE_BUF_TOO_SMALL,
          "copyToSwigObj: swig object buffer size=%d < %d bytes" % \
              (self._size, j+n))
    for k in xrange(n):
      bsCore.byteArray_setitem(self._swigObj, j+k, ord(self.buf[i+k]))
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
      if val > 255:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return val
    elif type(val) == str:
      if len(val) != 1:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return ord(val[0])
    elif type(val) == list:
      if len(val) != 1:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return ByteBuf.byte(val[0])
    else:
      raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
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
      raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
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
      if val > 255:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return chr(val)
    elif type(val) == str:
      if len(val) != 1:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return val[0]
    elif type(val) == list:
      if len(val) != 1:
        raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
      else:
        return ByteBuf.bytec(val[0])
    else:
      raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
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
      raise bsError.BotSenseError(bsCore.BS_ECODE_BAD_VAL, "%s" % str(val))
  ##
##


#------------------------------------------------------------------------------
# CLASS: BsProxyMsgHdr
#------------------------------------------------------------------------------
class BsProxyMsgHdr(bsCore.BsProxyMsgHdr_T):
  """ BotSense bsProxy Message Header Wrapper Class. """

  #--
  def __init__(self, tid=0, vconn=bsCore.BSPROXY_VCONN_SERVER, msgid=0,
                      bodylen=0):
    """ Initialize BotSense Proxy Message Header instance.

        Parameters:
          tid     - Transaction id.
          vconn   - Virtual connection handle.
          msgid   - Message id.
          bodylen - Message body length, in bytes.
    """
    bsCore.BsProxyMsgHdr_T.__init__(self)
    ## message header fixed magic number
    self.m_hdrMagic   = bsCore.BSPROXY_MSG_MAGIC
    ## message header transaction id
    self.m_hdrTid     = tid
    ## message header virtual connection handle
    self.m_hdrVConn   = vconn
    ## message header message id
    self.m_hdrMsgId   = msgid
    ## message header body length
    self.m_hdrBodyLen = bodylen

    ## class attributes
    self._attrlist = ['m_hdrMagic', 'm_hdrTid', 'm_hdrVConn', 'm_hdrMsgId',
                      'm_hdrBodyLen']
  ##

  #--
  def __sizeof__(self):
    """ x.__sizeof__() -- size of header, in bytes. """
    return bsCore.BSPROXY_MSG_HDR_LEN
  ##

  #--
  def __getitem__(self, y):
    """ x.__getitem__(y) <==> x[y] """
    return getattr(self, y)
  ##

  #--
  def __setitem__(self, y, v):
    """ x.__setitem__(y, v) <==> x[y]=v """
    if y in self._attrlist:
      setattr(self, y, v)
    else:
      raise AttributeError(y)
  ##

  #--
  def __iter__(self):
    """ x.__iter__() <==> iter(x) """
    return self._attrlist.__iter__()
  ##

  #--
  def __str__(self):
    """ x.__str__() <==> str(x) <==> print x """
    d = {}
    for attr in self._attrlist:
      d[attr] = getattr(self, attr)
    return str(d)
  ##

  #--
  def ListAttrs(self):
    """ Return list of class variable attribute names. """
    return self._attrlist
  ##

  #--
  def PackMsgHdr(self, buf):
    """ Pack BotSense bsProxy message header

        Raises a BotSenseError exception on failure.
         
        Parameters:
          buf [out] - BotSenseTypes.ByteBuf output buffer instance.

        Return:
          On success, returns the number of bytes packed.
          On failure, a BotSenseError exception is raised.
    """
    bsError.ChkType(buf, ByteBuf)
    n = bsCore.bsPackMsgHdr(self, buf.getSwigObj(), buf.sizeof())
    bsError.ChkReturnIsNonNeg(n, "Failed to pack message header.")
    buf.copyFromSwigObj(n)
    return n 
  ##

  #--
  def UnpackMsgHdr(self, buf):
    """ Unpack BotSense bsProxy message header

        Raises a BotSenseError exception on failure.

        Parameters:
          buf [in]  - BotSenseTypes.ByteBuf input buffer instance.

        Return:
          On success, returns the number of bytes unpacked.
          On failure, a BotSenseError exception is raised.
    """
    bsError.ChkType(buf, ByteBuf)
    buf.copyToSwigObj(bsCore.BSPROXY_MSG_HDR_LEN)
    n = bsCore.bsUnpackMsgHdr(buf.getSwigObj(), len(buf), self)
    bsError.ChkReturnIsNonNeg(n, "Failed to unpack message header.")
    return n 
  ##
##


#------------------------------------------------------------------------------
# CLASS: BsVConnInfo
#------------------------------------------------------------------------------
class BsVConnInfo(bsCore.BsVConnInfo_T):
  """ BotSense Virtual Connection Information Wrapper Class.
  """

  #--
  def __init__(self):
    """ Initialize Virtual Connection Info instance. """
    bsCore.BsVConnInfo_T.__init__(self)

    self._attrlist = ['m_vconn', 'm_rd', 'm_client', 'm_devuri', 'm_moduri',
                      'm_modver', 'm_moddate']
  ##

  #--
  def __sizeof__(self):
    """ x.__sizeof__() -- size of swig object, in bytes. """
    return 5 + len(self.m_client) + len(self.m_devuri) + len(self.m_moduri) + \
        len(self.m_modver) + len(self.m_moddate)
  ##

  #--
  def __getitem__(self, y):
    """ x.__getitem__(y) <==> x[y] """
    return getattr(self, y)
  ##

  #--
  def __setitem__(self, y, v):
    """ x.__setitem__(y, v) <==> x[y]=v """
    if y in self._attrlist:
      setattr(self, y, v)
    else:
      raise AttributeError(y)
  ##

  #--
  def __iter__(self):
    """ x.__iter__() <==> iter(x) """
    return self._attrlist.__iter__()
  ##

  #--
  def __str__(self):
    """ x.__str__() <==> str(x) <==> print x """
    d = {}
    for attr in self._attrlist:
      d[attr] = getattr(self, attr)
    return str(d)
  ##

  #--
  def ListAttrs(self):
    """ Return list of class variable attribute names. """
    return self._attrlist
  ##
##


#------------------------------------------------------------------------------
# CLASS: BsClientAppInfo
#------------------------------------------------------------------------------
class BsClientAppInfo(bsCore.BsClientAppInfo_T):
  """ BotSense Client Application Info and Callback Wrapper Class.

      When an application (library) establishes an application-specific
      virtual connection, this static information and callbacks are passed
      to the BotSense client library.

      The information and callbacks are optional. The BotSense library will
      ignore any member value that is set to None or if the class instance
      is None (everything is ignored).
  """

  #--
  def __init__(self):
    """ Initialize BotSense Proxy Message Header instance. """
    bsCore.BsClientAppInfo_T.__init__(self)

    self._attrlist = ['app_name', 'brief', 'version', 'date', 'maintainer',
                      'license', 'fnGetMsgName']
  ##

  #--
  def __sizeof__(self):
    """ x.__sizeof__() -- size of swig object, in bytes. """
    return 8 * len(self._attrlist)
  ##

  #--
  def __getitem__(self, y):
    """ x.__getitem__(y) <==> x[y] """
    return getattr(self, y)
  ##

  #--
  def __setitem__(self, y, v):
    """ x.__setitem__(y, v) <==> x[y]=v """
    if y in self._attrlist:
      setattr(self, y, v)
    else:
      raise AttributeError(y)
  ##

  #--
  def __iter__(self):
    """ x.__iter__() <==> iter(x) """
    return self._attrlist.__iter__()
  ##

  #--
  def __str__(self):
    """ x.__str__() <==> str(x) <==> print x """
    d = {}
    for attr in self._attrlist:
      d[attr] = getattr(self, attr)
    return str(d)
  ##

  #--
  def ListAttrs(self):
    """ Return list of class variable attribute names. """
    return self._attrlist
  ##
##
