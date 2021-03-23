################################################################################
#
# KondoSerial.py
#

""" Kondo RCB-1 Serial Port Module

Kondo Robot Control Board (RCB-1) generic serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.26

Copyright (C) 2005.  RoadNarrows LLC.
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
import time

import Fusion.KHR1.Cmd.KHR1Error as error

if __debug__: import Fusion.Utils.PyDebug as PyDebug


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# KHR-1 RCB-1 Acknoledgement
ACK = 0x06


#-------------------------------------------------------------------------------
# CLASS: KHR1Serial
#-------------------------------------------------------------------------------
class KHR1Serial:
  """ KHR-1 RCB-1 Serial Port Class. """

  # supported KHR-1 baudrates
  mKHR1SupportedBaudrates = [115200]

  #--
  def __init__(self, port=None, dbgobj=None):
    """ Initialize a Kondo KHR-1 RCB-1 serial port object. If a serial port
        is specified, then the port will be opened. Otherwise, the KHR-1
        RCB-1 serial port object will be in the closed state.

        Parameters:
          port      - serial port (defaul: no port)
          dbgobj    - PyDebug object. None will create the object.
    """
    self._mCmdSema  = thread.Semaphore()  # serialize command/response sequence
    self.mErr       = error.KHR1Error()   # error handler

    # debugging
    if __debug__:
      if not dbgobj:
        self.mDbg = PyDebug.PyDebug('KHR1Serial')
      else:
        self.mDbg = dbgobj

    # open port if specified
    if port:
      self.Open(port)
    else:
      self._mHndSer = ser.Serial()

  #--
  def Open(self, port, baudrate=115200, bytesize=8, parity='N', stopbits=1):
    """ Open a serial port to the Kondo KHR-1 RCB-1. 

        An IOError exception is thrown if the port cannot be opened.
        
        Parameters:
          port - serial port (device)
          other parameters are ignored

        Return Value:
          No return value.
    """
    try:
      self._mHndSer = ser.Serial(port=port, baudrate=115200,
                                 bytesize=8, parity='N', stopbits=1,
                                 timeout=0.25, writeTimeout=2.0)
    except (ser.SerialException, OSError) as err:
      self.mErr.SetErrGeneral(err.__str__())
      raise IOError(err)

    if __debug__: self.mDbg.d5print("open %s 115200-8-N-1:" % port)

    # The ICS-PC serial cable supplied by Kondo ties the robot's TX line to 
    # the host's DTR. The host must set the DTR to low or the host will not
    # be able to receive responses from KHR-1.
    # N.B.  The DTR pull done is 'translator' from the roobt's TTL levels
    # to RS232 levels. Host (e.g. embedded single board computers) may not
    # have access to DTR (3 wire i/f). This is a true Kondo kludge.
    self._mHndSer.setDTR(0) 

    # Flush UART input buffer of any residual data
    self.FlushInput()

  #--
  def Close(self):
    """ Close the serial port.
      
        Return Value:
          No return value
    """
    self._mHndSer.close()
    if __debug__: self.mDbg.d5print("closed %s" % self.GetPort())

  #--
  def IsOpen(self):
    """ Test if the serial port is open.
      
        Return Value:
          True if port is open, else False.
    """
    return self._mHndSer.isOpen()

  #--
  def GetPort(self):
    """ Get serial port name.
      
        Return Value:
          Port name or None
    """
    return self._mHndSer.getPort()

  #--
  def GetSupportedBaudRates(self):
    """ Get list of KHR-1 supported serial port baudrates.
      
        Return Value:
          List of supported baudrates.
    """
    return self.mKHR1SupportedBaudrates

  #--
  def GetSupportedByteSizes(self):
    """ Get list of KHR-1 supported serial port byte sizes.
      
        Return Value:
          List of supported byte sizes.
    """
    return [8]

  #--
  def GetSupportedParities(self):
    """ Get list of KHR-1 supported serial port parities.
      
        Return Value:
          List of supported parities.
    """
    return ['N']

  #--
  def GetSupportedStopBits(self):
    """ Get list of KHR-1 supported serial port stopbit sizes.
      
        Return Value:
          List of supported stobits.
    """
    return [1]

  #--
  def GetErrStr(self):
    """ Get last error string
      
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
    """ Flush input buffers of existing data, discarding the data.

        Return Value:
          No return value
    """
    try:
      self._mHndSer.flushInput()
    except (ser.SerialException, OSError) as err:
      if __debug__:
        self.mDbg.d5print("Error: flush input on port %s: %s" % \
                      (self.GetPort(), err.__str__()))
      return

    try:
      while self._mHndSer.read(16):
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

  #--
  def BinToHexStr(self, binData):
    """ Convert binary data to readable ascii hex string.

        Parameters:
          binData - string or list of binary data bytes
        
        Return Value:
          Ascii hex string of format: '0xhh 0xhh 0xhh ... 0xhh '
    """
    if type(binData) == list:
      f = lambda x: x
    else:
      f = lambda x: ord(x)
    s = ''
    for b in binData:
      s += "0x%02x " % f(b)
    return s

  #--
  def HexToBinList(self, sHex):
    """ Convert hex string to binary byte list.

        Parameters:
          sHex - string of format: '0xhh 0xhh ...'
        
        Return Value:
          Binary data list.
    """
    binData = []
    for sArg in sHex.split():
      if len(sArg) > 2 and (sArg[0:2] == '0x' or sArg[0:2] == '0X'):
        base = 16
      else:
        base = 10
      try:
        val = int(sArg, base)
      except (SyntaxError, NameError, TypeError, ValueError):
        return []
      binData += [val]
    return binData

  #--
  def GenChkSum(self, binData):
    """ Generate a 7-bit check sum over the binary data.

        Parameters:
          binData - List of string of binary data.

        Return Value:
          7-bit check sum
    """
    chksum = 0
    for byte in binData:
      chksum += byte
    chksum &= 0x7F
    return chksum

  #--
  def SendCmd(self, binCmd, rspLen, rspChk='ack'):
    """ Send a Kondo KHR-1 RCB-1 serial binary command and wait for the
        response or until a timeout occurs. 
      
        Parameters:
          binCmd          - Binary command byte list (no checksum)
          rspLen          - Expected total response length in bytes
          rspChk=<method> - Response check method. One Of:
                              'ack' - response contains an ACKnowledgement
                              'checksum' - response contains a checksum

        Return Value:
          On success, return the binary response byte list. Any ACK or
          checksum is stripped from the list.
          On error or timeout, an error is set and None is returned.
    """
    if not self.IsOpen():
      self.mErr.SetErrGeneral("Port not opened")
      return None
    if not binCmd:  # null command
      self.mErr.SetErrGeneral("No command specified")
      return None
    self._mCmdSema.acquire()
    maxTries  = 4
    tries     = 0
    while tries < maxTries:
      if __debug__:
        self.mDbg.d5print("SendCmd 0x%02x: try=%d" % (binCmd[0], tries))
      if self._WriteCmd(binCmd):
        rsp = self._ReadRsp(rspLen, rspChk)
      else:
        rsp = None
      if rsp is not None:
        break
      if __debug__:
        self.mDbg.d5print("SendCmd %s: error: %s" % \
            (self.BinToHexStr(binCmd[:1]), self.GetErrStr()))
      #self._WriteCmd('')   # there is no clear/reset command to RCB
      time.sleep(0.01)
      self.FlushInput()
      tries += 1
    self._mCmdSema.release()
    return rsp


  #-----------------------------------------------------------------------------
  # Private Interface
  #-----------------------------------------------------------------------------

  #--
  def _WriteCmd(self, binCmd):
    """ Write out binary command.

        Parameters:
          binCmd    - binary command byte list (no checksum)
        
        Return Value:
          True on success, False on failure.
    """
    chksum  = 0
    sOut    = ""
    for byte in binCmd:
      chksum += byte
      sOut += "%c" % (byte)
    chksum &= 0x7F
    sOut += "%c" % (chksum)
    try:
      self._mHndSer.write(sOut)
      if __debug__: self.mDbg.d5print("write: %s" % self.BinToHexStr(sOut))
      self._mHndSer.flush()
      return True
    except (ser.SerialException, OSError) as err:
      self.mErr.SetErrGeneral('Bad write: ' + err.__str__())
      return False

  #--
  def _ReadRsp(self, rspLen, rspChk):
    """ Read in binary response.

        Parameters:
          rspLen          - Expected total response length in bytes
          rspChk=<method> - Response check method. One Of:
                              'ack' - response contains an ACKnowledgement
                              'checksum' - response contains a checksum
        
        Return Value:
          On success, return the binary response byte list. Any ACK or
          checksum is stripped from the list.
          On error or timeout, an error is set and None is returned.
    """
    maxTries  = 2
    tries     = 0
    binRsp    = []
    size      = rspLen
    while tries < maxTries:
      try:
        # read the (partial) response line
        sIn = ''
        sIn = self._mHndSer.read(size)
        if __debug__:
          self.mDbg.d5print("try=%d read(%d): rsp: %s" % \
              (tries, rspLen, self.BinToHexStr(sIn)))
      except (ser.SerialException, OSError) as err:
        self.mErr.SetErrGeneral('Bad read: ' + err.__str__())
        return None

      # got a (partial) response
      if sIn:
        for byte in sIn:
          binRsp += [ord(byte)]
        size -= len(sIn)
        if size <= 0:
          break

      tries += 1

    # validate response
    if len(binRsp) == 0:
      self.mErr.SetErrNoRsp()
      return None
    elif size > 0:
      self.mErr.SetErrBadRsp(
        "Partial response: expected %d bytes, got %d" % (rspLen, rspLen-size),
        binRsp)
      return None
    elif size < 0: 
      self.mErr.SetErrBadRsp(
        "Response too long: expected %d bytes, got %d" % (rspLen, rspLen-size),
        binRsp)
      return None
    elif rspChk == 'checksum':
      chksum = self.GenChkSum(binRsp[:-1])
      if chksum == binRsp[rspLen-1]:
        return binRsp[:-1]
      else:
        self.mErr.SetErrBadChkSum(chksum, binRsp)
        return None
    elif rspChk == 'ack':
      if binRsp[-1] == ACK:
        return binRsp[:-1]
      else:
        self.mErr.SetErrBadAck(ACK, binRsp)
        return None
    else:
      return binRsp


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------
if __name__ == '__main__':
  if __debug__:
    dbg = PyDebug.PyDebug('KHR1Serial', debuglevel=5)
  else:
    dbg = None
  khr1ser = KHR1Serial(dbgobj=dbg)

  # the commands
  # number: [desc, cmd, rsplen, rspack]
  cmdlist = {
    '1': ['Get RCB-1 ID', [0xfe], 2, 'checksum'],
    '2': ['Play RCB-1 Motion 1', [0xef, 0x00, 0x01], 2, 'ack'],
    '3': ['Get RCB-1 ID 1 Servo Positions', [0xfc, 0x01], 14, 'checksum'],
    '4': ['Set RCB-1 ID 0 Servo Positions', [0xfd, 0x00, 0x03] + [45] * 12,
      2, 'ack'],
    '5': ['Set Current Position as Home Position', [0xfb, 0x00], 2, 'ack'],
  }

  print("Enter KHR-1 commands ('quit' to quit, 'help' for command list)")
  while True:
    ans = raw_input('cmd> ')
    if not ans:
      continue
    args = ans.split()
    if len(args) == 0:
      continue
    if 'quit'.find(args[0]) == 0:
      khr1ser.Close()
      break
    elif 'help'.find(args[0]) == 0:
      print('Commands')
      print('--------')
      print('open <port> - Open serial connection')
      print('flush - Flush input')
      print('close - close connection')
      print('help - Print this help')
      print('quit - Quit test')
      cmds = cmdlist.keys()
      cmds.sort()
      for k in cmds:
        print("%s - %s" % (k, cmdlist[k][0]))
    elif 'open'.find(args[0]) == 0:
      khr1ser.Open(args[1])
    elif 'flush'.find(args[0]) == 0:
      khr1ser.FlushInput()
    elif 'close'.find(args[0]) == 0:
      khr1ser.Close()
    else:
      gotcmd = False
      for t,v in cmdlist.iteritems():
        if t == args[0]:
          gotcmd = True
          khr1ser.ClearErrStr()
          rsp = khr1ser.SendCmd(v[1], v[2], v[3])
          if rsp:
            print(khr1ser.BinToHexStr(rsp))
          else:
            print(khr1ser.GetErrStr())
      if not gotcmd:
        print("Huh?")
