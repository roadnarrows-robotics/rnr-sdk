################################################################################
#
# HemiCmdTts.py
#

""" Hemisson Text-To-Speech Command Module

Hemisson Text-To-Speech serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.18

Copyright (C) 2005. 2006.  RoadNarrows LLC.
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

import time

import Fusion.Hemisson.Cmd.HemiSerial as HemiSerial
import Fusion.Hemisson.Cmd._hemiUtils as hutil


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

TtsSpeakerGainMin   =   7   # Quietest
TtsSpeakerGainMax   =   0   # Loudest
TtsSpeakerGainDft   =   0   # Default speaker gain

TtsCannedMsgNumMin  =   1   # Minimum canned message number
TtsCannedMsgNumMax  =  30   # Maximum canned message number

TtsVoicePitchMin    =   7   # Lowest pitch
TtsVoicePitchMax    =   0   # Highest pitch (micky mouse)
TtsVoicePitchDft    =   5   # Default (power-on) speaker gain

TtsVoiceRateMin     =   0   # Slowest voice rate
TtsVoiceRateMax     =   3   # Fastest voice rate
TtsVoiceRateDft     =   3   # Default (power-on) voice rate

TtsMsgLenMax        =  72   # Maximum message length 

TtsStateNotSpeaking =   0   # TTS processor is not speaking
TtsStateSpeaking    =   1   # TTS processor is speaking


#-------------------------------------------------------------------------------
# CLASS: HemiCmdTts
#-------------------------------------------------------------------------------
class HemiCmdTts(HemiSerial.HemiSerial):
  """ Hemisson Text-To-Speech Command and Response Class. """

  #--
  def __init__(self, port=None, dbgobj=None):
    """ Initialize a Hemisson serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the Hemisson
        serial port object will be in the closed state.

        Parameters:
          port      - serial port (default: no port)
          dbgobj    - PyDebug object. None will create the object.
    """
    HemiSerial.HemiSerial.__init__(self, port, dbgobj)

    self.Init()

  #--
  def Init(self):
    """ One time initialization during object instantiation. """
    self.mGain  = TtsSpeakerGainDft   
    self.mPitch = TtsVoicePitchDft   
    self.mRate  = TtsVoiceRateDft   


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Robot Serial Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def TtsCmdSpeakCannedMsg(self, msgnum):
    """ Speak a canned (pre-stored) TTS message ('C').
  
        Parameters:
          msgnum  - number of the pre-stored message [1,30]
  
        Return Value:
          Message number on success, None on bad response.
    """
    msgnum = hutil.cap(msgnum, TtsCannedMsgNumMin, TtsCannedMsgNumMax)
    rsp = self.SendCmd('A,T,C,' + "%0.2X" % msgnum)
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  #--
  def TtsCmdSetSpeakerGain(self, gain):
    """ Set TTS speaker gain (volume) ('G').
  
        Parameters:
          gain  - spaker gain [0,7]
                    0 = loudest
                    7 = quietest
  
        Return Value:
          Newly set gain value on success, None on bad response.
    """
    gain = hutil.cap(gain, TtsSpeakerGainMax, TtsSpeakerGainMin)
    rsp = self.SendCmd('A,T,G,' + "%d" % gain)
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    self.mGain = hutil.cvtInt(rsp[6:])
    return self.mGain
  
  #--
  def TtsCmdSetVoicePitch(self, pitch):
    """ Set TTS pitch of voice in text-to-speech conversion ('P').
  
        Parameters:
          pitch - voice pitch [0,7]
                    0 = highest pitch (micky mouse)
                    7 = lowest pitch
  
        Return Value:
          Newly set pitch value on success, None on bad response.
    """
    pitch = hutil.cap(pitch, TtsVoicePitchMax, TtsVoicePitchMin)
    rsp = self.SendCmd('A,T,P,' + "%d" % pitch)
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    self.mPitch = hutil.cvtInt(rsp[6:])
    return self.mPitch
  
  #--
  def TtsCmdQueryState(self):
    """ Query speaking state of Text-To-Speech processor ('Q').
  
        Return Value:
          TtsStateNotSpeaking - TTS module is not currently speaking
          TtsStateSpeaking    - TTS module is currently speaking
          None                - bad response
    """
    rsp = self.SendCmd('A,T,Q')
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtInt(rsp[6:])
  
  #--
  def TtsCmdSetVoiceRate(self, rate):
    """ Set TTS rate (speed) of voice in text-to-speech conversion ('R').
  
        Parameters:
          rate  - voice rate [0,3]
                    0 = slowest rate
                    3 = fastest rate
  
        Return Value:
          Newly set rate value on success, None on bad response.
    """
    rate = hutil.cap(rate, TtsVoiceRateMin, TtsVoiceRateMax)
    rsp = self.SendCmd('A,T,P,' + "%d" % rate)
    if rsp is None: return None
    if len(rsp) != 7: return self.mErr.SetErrBadRsp(rsp)
    self.mRate = hutil.cvtInt(rsp[6:])
    return self.mRate
  
  #--
  def TtsCmdGetSettings(self):
    """ Get TTS current settings.
  
        Return Value:
          Dictionary of current settings.
    """
    return {'gain': self.mGain, 'pitch': self.mPitch, 'rate': self.mRate}
 
  #--
  def TtsCmdSay(self, msg):
    """ Speak a TTS message ('S').
  
        Parameters:
          msg   - text message to speak. Length: 1 - 72 bytes
  
        Return Value:
          Message said on success, None on bad response.
    """
    if len(msg) == 0: return self.mErr.SetErrBadParam('msg', msg)
    if len(msg) > TtsMsgLenMax: msg = msg[0:TtsMsgLenMax]
    rsp = self.SendCmd('A,T,S,' + "%s" % msg)
    if rsp is None: return None
    return self._TtsParseRspMsg(msg, rsp)
   
  #--
  def TtsCmdSaySync(self, msg):
    """ Speak a TTS message and wait until TTS processor is finished.
        This command will wait at most appoximately 30 seconds ('S','Q').
  
        Parameters:
          msg   - text message to speak. Length: 1 - 72 bytes
  
        Return Value:
          Message said on success, None on timeout or bad response.
    """
    if len(msg) == 0: return self.mErr.SetErrBadParam('msg', msg)
    if len(msg) > TtsMsgLenMax: msg = msg[0:TtsMsgLenMax]
    rsp = self.TtsCmdSay(msg)
    if not rsp: return None

    # determine initial timeout: assume 180 words/minute
    wordcnt = len(msg.split())
    timeout = wordcnt * 0.33
    #print timeout

    queries = 0
    #tto = 0.0
    while queries < 43:
      if self.TtsCmdQueryState() == TtsStateNotSpeaking: return rsp
      time.sleep(timeout)
      #tto += timeout
      #print tto
      timeout = 0.5 # new timeout
      queries += 1

    return self.mErr.SetErrRspTimeout()

  #--
  def TtsCmdGiveASpeech(self, speech):
    """ Give a TTS speech. That is, orate until the fat lady sings.
        This command sits down (returns) only after the rousing speech
        is finished ('S', 'Q').
  
        Parameters:
          speech   - long-winded, unlimited text message to speak.
  
        Return Value:
          "applause" on success, None on timeout or bad response.
    """
    speech = speech.expandtabs(1)
    speech = speech.split()
    msg = ''

    for word in speech:
      if len(msg) + len(word) < TtsMsgLenMax:
        if msg: msg += ' '
        msg += word
      else:
        # print msg
        if not self.TtsCmdSaySync(msg):
          return None
        msg = word

    if msg:
      # print msg
      if not self.TtsCmdSaySync(msg):
        return None

    return "applause"

  #--
  def TtsCmdGetVersion(self):
    """ Get Hemisson TTS version ('V').
  
        Return Value:
          Version integer value on success, None on bad response.
    """
    rsp = self.SendCmd('A,T,V')
    if rsp is None: return None
    if len(rsp) != 8: return self.mErr.SetErrBadRsp(rsp)
    return hutil.cvtHH(rsp, 6)
  
  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  #--
  def _TtsParseRspMsg(self, msg, rsp):
    """ Return message said or None. """
    if len(rsp) < 7: return self.mErr.SetErrBadRsp(rsp)
    if len(rsp[6:]) <= len(msg): return rsp[6:] # input does not always match
                                                # output
    return self.mErr.SetErrBadRsp(rsp)
