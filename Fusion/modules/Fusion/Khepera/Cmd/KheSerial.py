################################################################################
#
# KheSerial.py
#

""" Khepera II Serial Port Module

Khepera II generic serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.31

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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
import serial as ser

import Fusion.Khepera.Cmd.KheError as error

if __debug__: import Fusion.Utils.PyDebug as PyDebug


#-------------------------------------------------------------------------------
# CLASS: KheSerial
#-------------------------------------------------------------------------------
class KheSerial:
  """ Khepera II Serial Port Class. """

  # supported Khepera baudrates
  mKheperaSupportedBaudrates = [9600, 19200, 38400, 57600, 115200]

  #--
  def __init__(self, port=None, baudrate=9600, dbgobj=None):
    """ Initialize a Khepera II serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the Khepera II
        serial port object will be in the closed state.

        Parameters:
          port      - serial port (default: no port)
          baudrate  - serial baudrate: (default: 9600)
          dbgobj    - PyDebug object. None will create the object.
    """
    self._mCmdSema  = thread.Semaphore()  # serialize command/response sequence
    self.mErr       = error.KheError()    # error handler

    # debugging
    if __debug__:
      if not dbgobj:
        self.mDbg = PyDebug.PyDebug('KheSerial')
      else:
        self.mDbg = dbgobj

    # open port if specified
    if port:
      self.Open(port, baudrate)
    else:
      self._mHndSer = ser.Serial()

  #--
  def Open(self, port, baudrate=9600, bytesize=8, parity='N', stopbits=1):
    """ Open a serial port to the Khepera. 

        An IOError exception is thrown if the port cannot be opened.
        
        Parameters:
          port      - serial port (device)
          baudrate  - serial baudrate: (default: 9600)
          other parameters are ignored

        Return Value:
          No return value.
    """
    # Validate that Khepera supports this baudrate
    try:
      self.mKheperaSupportedBaudrates.index(baudrate)
    except ValueError:
      self.mErr.SetErrGeneral("Khepera unsupported baudrate: %d" % (baudrate))
      raise IOError(self.mErr.GetErrStr())

    try:
      self._mHndSer = ser.Serial(port=port, baudrate=baudrate,
                                 bytesize=8, parity='N', stopbits=2,
                                 timeout=0.3, writeTimeout=2.0)
      if __debug__: self.mDbg.d5print("open %s %d-8-N-2:" % (port, baudrate))
      self.FlushInput()
    except (ser.SerialException, OSError) as err:
      self.mErr.SetErrGeneral(err.__str__())
      raise IOError(err)

  #--
  def Close(self):
    """ Close the serial port.
      
        Return Value:
          No return value
    """
    self._mHndSer.close()

  #--
  def IsOpen(self):
    """ Test if the serial port is open.
      
        Return Value:
          True if port is open, else False.
    """
    return self._mHndSer.isOpen()

  #--
  def GetPort(self):
    """ Get current serial port name.
      
        Return Value:
          Port name or None
    """
    return self._mHndSer.getPort()

  #--
  def GetBaudRate(self):
    """ Get current baudrate setting.
      
        Return Value:
          Baudrate.
    """
    return self._mHndSer.getBaudrate()

  #--
  def GetSupportedBaudRates(self):
    """ Get list of Khepera supported serial port baudrates.
      
        Return Value:
          List of supported baudrates.
    """
    return self.mKheperaSupportedBaudrates

  #--
  def GetSupportedByteSizes(self):
    """ Get list of Khepera supported serial port byte sizes.
      
        Return Value:
          List of supported byte sizes.
    """
    return [8]

  #--
  def GetSupportedParities(self):
    """ Get list of Khepera supported serial port parities.
      
        Return Value:
          List of supported parities.
    """
    return ['N']

  #--
  def GetSupportedStopBits(self):
    """ Get list of Khepera supported serial port stopbit sizes.
      
        Return Value:
          List of supported stobits.
    """
    return [2]

  #--
  def GetErrStr(self):
    """ Get last error string.
      
        Return Value:
          Error string
    """
    return self.mErr.GetErrStr()

  #--
  def ClearErrStr(self):
    """ Clear error string.
      
        Return Value:
          None
    """
    self.mErr.ClearErrStr()

  #--
  def FlushInput(self):
    """ Flush input buffers of existing data.

        Return Value:
          No return value
    """
    try:
      self._mHndSer.flushInput()
    except (ser.SerialException, OSError) as err:
      if __debug__:
        self.mDbg.d5print("Error: flush on port %s: %s" % \
                      (self.GetPort(), err.__str__()))
      return

    try:
      while self._mHndSer.readline():
        pass
    except (ser.SerialException, OSError) as err:
      if __debug__:
        self.mDbg.d5print("Error: read on port %s: %s" % \
                      (self.GetPort(), err.__str__()))
      pass

  #--
  def FlushOutput(self):
    """ Clear output buffers of existing data, discarding the data.

        Return Value:
          No return value
    """
    try:
      self._mHndSer.flushOutput()
    except (ser.SerialException, OSError) as err:
      if __debug__:
        self.mDbg.d5print("Error: flush output on port %s: %s" % \
                      (self.GetPort(), err.__str__()))
      return
 
  #--
  def SetDebugLevel(self, debuglevel, debugfout=None):
    """ Set serial debugging level.

        Note: This call is should be made only if KheSerial owns the debug
              object. Otherwise the KheSerial containing object may get a
              surprise.

        Parameters:
          debuglevel  - none to all [0, 5]
          debugfout   - opened debug output file
        
        Return Value:
          None
    """
    if __debug__: self.mDbg.On(debuglevel, debugfout)
    pass # keep this line

  #--
  def SendCmd(self, cmd):
    """ Send a Khepera serial command and wait for the response. The wait
        will be at most 1 second before a timeout occurs.
      
        Parameters:
          cmd - command string (no newline NL or carriage return CR 
                characters)

        Return Value:
          On success, return the Khepera response string. The trailing NL 
          and CR characters are stripped. Empty string ('') responses are
          valid.
          On error or timeout, None is returned.
    """
    if not self.IsOpen():
      self.mErr.SetErrGeneral("Port not opened")
      return None
    if not cmd:  # null command
      return ''
    self._mCmdSema.acquire()
    if self._WriteCmd(cmd):
      rsp = self._ReadRsp(cmd[0].lower())
    else:
      rsp = None
    self._mCmdSema.release()
    return rsp


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _WriteCmd(self, cmd):
    """ Write out command.

       Parameters:
         cmd - command string (no newline NL or carriage return CR characters)
        
        Return Value:
          True on success, False on failure.
    """
    try:
      self._mHndSer.write(cmd + '\r')
      if __debug__: self.mDbg.d5print("write:", repr(cmd + '\r'))
      self._mHndSer.flush()
      return True
    except (ser.SerialException, OSError) as err:
      self.mErr.SetErrGeneral('Bad write: ' + err.__str__())
      return False

  #--
  def _ReadRsp(self, rspKey):
    """ Read in response line.

       Parameters:
         rspKey - expected response starting character (key)
        
        Return Value:
          On success, return response (without CR-NL).
          On error or timeout, an error is set and None is returned.
    """
    maxTries = 3
    tries = 0
    mismatch = False
    rsp = ''
    while tries < maxTries:
      # read the (partial) respone line
      try:
        sIn = self._mHndSer.readline()
        if __debug__:
          self.mDbg.d5print("try=%d readline: %s" % (tries, repr(sIn)))
      except (ser.SerialException, OSError) as err:
        self.mErr.SetErrGeneral('Bad read: ' + err.__str__())
        return None

      # got a (partial) repsonse
      if sIn:
        rsp += sIn
        # end of a response
        if len(rsp) > 2 and rsp[-2:] == '\r\n':
          # got the correct response
          if rsp[0] == rspKey:
            mismatch = False
            break
          # command-response mismatch
          else:
            self.mErr.SetErrGeneral(
                "Command/response mismatch: expected '%c...' got: %s" \
                  % (rspKey, repr(rsp)))
            mismatch = True
            rsp = ''
      tries += 1

    if mismatch:  # error has already been set
      return None
    elif rsp == '':
      self.mErr.SetErrGeneral('No response')
      return None
    elif tries >= maxTries:
      self.mErr.SetErrGeneral('Partial response: %s' % (repr(rsp)))
      return None
    else:
      return rsp[:-2]   # strip off CR-NL


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  if __debug__:
    dbg = PyDebug.PyDebug('KheSerial', debuglevel=5)
  else:
    dbg = None
  ser = KheSerial(port='/dev/ttyS0', dbgobj=dbg)
  print("Enter Khepera command(s) ('quit' to quit)")
  while True:
    cmd = input('cmd> ')
    if cmd == 'quit':
      ser.Close()
      break
    rsp = ser.SendCmd(cmd)
    if rsp:
      print(rsp)
    else:
      print(ser.GetErrStr())
