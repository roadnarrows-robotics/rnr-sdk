################################################################################
#
# Channel.py
#

""" Abstract Communication Channel Module.

A generalized, abstract model of a communication channel between a Fusion
vRobot and the physical/simulated robot.

Functional groups of channel operations are:
  init              - initializing the channel
  open/close        - open/close the communication channel
  read/write/flush  - low-level channel read/write/flush operations 
  command/response  - [a]synchronous command/response sequences
  reports           - asynchronous robot reports (aka: triggers and
                      indicators)
  error             - error handling
  debugging         - debug tracing
  callbacks         - callbacks of asynchronous events to controlling entity

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.06.30

Copyright (C) 2006.  RoadNarrows LLC.
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

import threading as thread
import os
import errno

import Fusion.Utils.Enum as Enum

if __debug__: import Fusion.Utils.PyDebug as PyDebug

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# 
# Channel Error Codes
#
ChanErrCode = Enum.Enum(
    "OK "
    "NOTOPEN CONNLOST CMDTIMEOUT NORSP RSPTIMEOUT "
    "BADRSP BADCHKSUM BADCRC BADSEQNUM "
    "NOTIMPLEMENTED",
    -1, -1)

#
# Channel Error Code Message Strings
#
ChanErrStr = {
  ChanErrCode.OK:             "Success",

  ChanErrCode.NOTOPEN:        "Channel not open",
  ChanErrCode.CONNLOST:       "Connection lost",
  ChanErrCode.CMDTIMEOUT:     "Command timed out",
  ChanErrCode.NORSP:          "No response",
  ChanErrCode.RSPTIMEOUT:     "Response timed out",
  ChanErrCode.BADRSP:         "Bad response",
  ChanErrCode.BADCHKSUM:      "Bad check sum",
  ChanErrCode.BADCRC:         "Bad CRC",
  ChanErrCode.BADSEQNUM:      "Bad sequence number",

  ChanErrCode.NOTIMPLEMENTED: "Function not implemented"
}


#-------------------------------------------------------------------------------
# CLASS: ChannelError Exception
#-------------------------------------------------------------------------------
class ChannelError(Exception):
  """ Channel Error Exception Class.

      Attributes (in order):
        chanerr - channel error code
                    chanerr < 0:  channel specific error code
                    chanerr > 0:  system specific error code (errno)
        estr    - translated channel error code to standard message
                  string
        emsg    - context specific explanation of the error
  """

  #--
  def __init__(self, chanerr, emsg=None):
    """ Initialize (raise) exception.

        Parameters:
          chanerr - channel error code
                      chanerr < 0:  channel specific error code
                      chanerr > 0:  system specific error code (errno)
          emsg    - context specific explanation of the error
    """
    self.chanerr = chanerr
    if chanerr < 0:
      if ChanErrCode.has_enum(chanerr):
        self.ename = ChanErrCode.name(chanerr)
        self.estr = ChanErrStr[chanerr]
      else:
        self.ename = ''
        self.estr = "Unknown channel error %s" % repr(chanerr)
    else:
      self.estr = os.strerror(chanerr) # also maps unknown
      if errno.errorcode.has_key(chanerr):
        self.ename = errno.errorcode[chanerr]
      else:
        self.ename = repr(chanerr)
    if emsg:
      self.emsg = emsg
    else:
      self.emsg = ""
    Exception.__init__(self, self.chanerr, self.estr, self.emsg)

  #--
  def __str__(self):
    """ Return formated error string. """
    if self.emsg:
      return "[ChannelError: %s(%d)] %s: %s" \
          % (self.ename, self.chanerr, self.estr, self.emsg)
    else:
      return "[ChannelError: %s(%d)] %s" \
          % (self.ename, self.chanerr, self.estr)


#-------------------------------------------------------------------------------
# CLASS: Channel
#-------------------------------------------------------------------------------
class Channel:
  """ Abstract Communication Channel Class. """

  #--
  def __init__(self, name="Channel", mimeType="channel/abstract",
                     dbgobj=None, onError=None, onClose=None, onReport=None,
                     port=None, **kwPortArgs):
    """ Initialize a channel object. Depending on the derived class
        implementation, the communication channel may or may not be
        opened during initialization.

        Parameters:
          name          - channel name
          mimeType      - type of channel
          dbgobj        - PyDebug object. None will create a local
                          debug object.
          onError       - on error event callback
          onClose       - on close event callback
          onReport      - on report event callback
          port          - channel port (if specified, will automatically 
                          try to open)
          **kwPortArgs  - optional keyword=value port options
    """
    self.mChanName      = name                # name of channel
    self.mChanMimeType  = mimeType            # MIME type of channel
    self.mChanSema      = thread.Semaphore()  # channel access sequence control
    self.mChanErrCode   = ChanErrCode.OK      # last error code
    self.mChanErrMsg    = ""                  # last error message
    self.mCbOnError     = None                # on error callback
    self.mCbOnClose     = None                # on close callback
    self.mCbOnReport    = None                # on report callback

    # debugging
    if __debug__:
      if not dbgobj:
        self.mDbg = PyDebug.PyDebug(self.mChanName)
      else:
        self.mDbg = dbgobj

    # open port if specified
    if port:
      self.Open(port, kwPortArgs)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Channel Open/Close Operations
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def Open(self, port, **kwArgs):
    """ Open a channel on the given port.

        A ChannelError exception is thrown if the port cannot be opened.
        
        Parameters:
          port    - channel port (device)
          kwArgs  - argument list of keyword=value channel configuration

        Return Value:
          None.
    """
    if __debug__: self.mDbg.d5print("opened %s" % self.GetPortName())
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "Open()")

  #--
  def Close(self):
    """ Close the serial port.
      
        Return Value:
          None.
    """
    if __debug__: self.mDbg.d5print("closed %s" % self.GetPortName())
    self.OnClose()
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "Close()")

  #--
  def OnClose(self):
    """ On Close event. Optionally called rather than raising a ChannelError
        to report that the channel as been closed.
      
        Return Value:
          None.
    """
    if self.mCbClose:
      self.mCbOnClose(self.GetPortName())


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Channel Attribute Operations
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsOpen(self):
    """ Test if the channel is open.
      
        Return Value:
          True if channel is open, else False.
    """
    return False

  #--
  def GetPortName(self):
    """ Get port name. Overide if port is not printable as is.
      
        Return Value:
          Port name or ""
    """
    return ""


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Channel Error/Debug Operations
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GetErrInfo(self):
    """ Get last error information.
      
        Return Value:
          (channel error code, error message string)
    """
    return self.mChanErrCode, self.mChanErrMsg

  #--
  def ClearErrInfo(self):
    """ Get last error information.
      
        Return Value:
          None
    """
    self.mChanErrCode = ChanErrCode.OK
    self.mChanErrMsg  = ""

  #--
  def OnError(self, chanerr, emsg):
    """ On Error event. Optionally called rather than raising a ChannelError
        to save last error information and make callback to controlling
        entity.
      
        Parameters:
          chanerr   - ChanErrCode value
          emsg      - error message string

        Return Value:
          None
    """
    self.mChanErrCode = chanerr
    self.mChanErrMsg  = emsg
    if chanerr != ChanErrCode.OK and self.mCbOnError:
      self.mCbOnError(self.mChanErrCode, self.mChanErrMsg)

  #--
  def SetDebugLevel(self, debuglevel, debugfout=None):
    """ Set serial debugging level.

        Note: This call is should be made only if KHR1Serial owns the debug
              object. Otherwise the KHR1Serial containing object may get a
              surprise.

        Parameters:
          debuglevel  - none to all [0, 5]
          debugfout   - opened debug output file
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.On(debuglevel, debugfout)
    pass # keep this line


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Channel Low-Level I/O Operations
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def Write(self, data):
    """ Write out data to open channel.

        This function may or may not block depending on derived
        implementation.

        A ChannelError exception is thrown on error.
        
        Parameters:
          data    - data (string, list, etc.) to write
        
        Return Value:
          On success, the number of data elements written (zero indicates 
          either a timout occurred if non-blocking condition). The number 
          of data elements returned maybe less than size. On error, 
          <0 is returned.
    """
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "Write()")

  #--
  def Read(self, size):
    """ Read at most size data elements from opened channel.
      
        This function may or may not block depending on derived
        implementation.

        A ChannelError exception is thrown on error.

        Parameters:
          size    - number of data elements to read
        
        Return Value:
          The read data which may be less than he number of elements
          requested. This may happened if fewer bytes are available, or
          a timeout has occurred, or the channel connection lost. 
          None indicates no data was read.
    """
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "Read()")

  #--
  def FlushInput(self):
    """ Flush input buffers of existing data.

        Return Value:
          None.
    """
    pass

  #--
  def Flush(self):
    """ Flush output data buffers.

        Return Value:
          None.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Channel Command/Response/Report Operations
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SendCmd(self, cmd, *args, **kwArgs):
    """ Send a command and wait for the response (or a timeout to occur). 
      
        Parameters:
          cmd      - command message
          *args    - implementation specific arguments
          *kwArgs  - implementation specific keyword=value arguments

        Return Value:
          On success, return the response data.
          On error or timeout, an error is reported and None is returned.
    """
    if not self.IsOpen():
      self.OnError(self.ERROR.ENOTOPEN)
      return None
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "SendCmd()")

  #--
  def SendAsyncCmd(self, cmd, *args, **kwArgs):
    """ Send a command asynchronously (i.e. don't wait for the response).
      
        Parameters:
          cmd      - command message
          *args    - implementation specific arguments
          *kwArgs  - implementation specific keyword=value arguments

        Return Value:
          On success, True is returned.
          On error or timeout, an error is reported and False is returned.
    """
    if not self.IsOpen():
      self.OnError(self.ERROR.ENOTOPEN)
      return False
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "SendAsyncCmd()")

  #--
  def RcvRsp(self, *args, **kwArgs):
    """ Receive an expected response (i.e. the corresponding command
        has previously been sent).
      
        Parameters:
          *args    - implementation specific arguments
          *kwArgs  - implementation specific keyword=value arguments

        Return Value:
          On success, return the response data.
          On error or timeout, an error is reported and None is returned.
    """
    raise ChannelError(ChanErrCode.NOTIMPLEMENTED, "SendCmd()")

  #--
  def OnReport(self, data):
    """ Received an asynchronous report event.
      
        Parameters:
          data    - report data (string, list, etc.)

        Return Value:
          None
    """
    if self.mCbOnReport:
      self.mCbOnReport(data)
