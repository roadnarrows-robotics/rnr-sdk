################################################################################
#
# KHR1Cmd.py
#

""" Kondo KHR-1 Command Module

Kondo KHR-1 serial command/response interface.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.10.24

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

import re
import time

import Fusion.KHR1.Cmd.KHR1Serial as KHR1Serial


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Robot Control Board 1 (RCB-1) Operational Data
# Some factoids:
# - Each RCB-1 can control up to 12 servos (channels 1 - 12)
# - Up to 32 RCB-1's can be chained together.
# - Each RCB-1 can store 4 scenarios in banks 0 - 3
# - Each scenario can store up to 200 motion indices with each motion index
#   pointing to a stored motion.
# - Each RCB-1 can store up to 40 motions.
# - Each motion can store up to 100 positions.
# - Each position specifies the speed and end positions of all 12 servos.
#
# The KHR-1 has 2 RCB-1's, normally ID's 0 and 1
#

RCB1ServoVersionBlue  =    'blue'   # Servo Blue Version
RCB1ServoVersionRed   =    'red'    # Servo Red Version

RCB1NumOfBoardIds     =   32    # Number of Board IDs possible
RCB1BoardIdMin        =    0    # Minimum RCB-1 Board ID 
RCB1BoardIdMax        =   31    # Maximum RCB-1 Board ID 
RCB1BoardIdZero       =    0    # First KHR-1 RCB-1 Board ID (typical)
RCB1BoardIdOne        =    1    # Second KHR-1 RCB-1 Board ID (typical)

RCB1NumOfChannels     =   12    # Number of channels/RCB

RCB1ServoSpeedMin     =    7    # Minimum servo speed (unitless)
RCB1ServoSpeedMax     =    0    # Maximum servo speed (unitless)
RCB1ServoSpeedDft     =    4    # Default servo speed (unitless)

RCB1ServoPosMin       =    0    # Minimum servo position (degrees)
RCB1ServoPosMax       =  180    # Maximum servo position (degrees)
RCB1ServoPos90        =   90    # 90 degree servo position
RCB1ServoPosFree      = 0xDD    # Special servo position (Red version)
RCB1ServoPos1         = 0xDE    # Special servo position (Red version)
RCB1ServoPos2         = 0xDF    # Special servo position (Red version)
RCB1ServoPos3         = 0xE0    # Special servo position (Red version)
RCB1ServoPosLow       = 0xE1    # Special servo position (Red version)
RCB1ServoPosHigh      = 0xE2    # Special servo position (Red version)

RCB1ServoPosSpecMin   = RCB1ServoPosFree # Minimum special servo position
RCB1ServoPosSpecMax   = RCB1ServoPosHigh # Maximum special servo position

RCB1MotionNumMin      =    0    # Minimum motion number
RCB1MotionNumMax      =   39    # Maximum motion number
RCB1NumOfMotions      =   40    # Maximum number of motions

RCB1PosNumMin         =    0    # Minimum position number in motion
RCB1PosNumMax         =   99    # Maximum position number in motion
RCB1NumOfMotionPos    =  100    # Maximum number of motion positions

RCB1ScenarioMin       =    0    # Minimum scenario bank
RCB1ScenarioMax       =    3    # Maximum scenario bank
RCB1NumOfScenarios    =    4    # Maximum number of scenarios

RCB1MotionIdxMin      =    0    # Minimum scenario motion index
RCB1MotionIdxMax      =  199    # Maximum scenario motion index
RCB1NumOfScenarioMotions =  200 # Maximum number of scenario motions 

RCB1ServoTrimMin      =  -20    # Minimum servo trim (-20 degrees)
RCB1ServoTrimMax      =   20    # Maximum servo trim (+19 degrees)
RCB1ServoTrimZero     =   20    # Point zero servo trim (0 degrees)

RCB1SwBitSleep        = 0x01    # Software bit - sleep enabled
RCB1SwBitMotion       = 0x02    # Software bit - motion enabled

#
# RCB-1 Command IDs
#
RCB1CmdIdSetId1             = 0xFF    # set RCB-1 ID (first byte)
RCB1CmdIdSetId2             = 0x5A    # set RCB-1 ID (second byte)
RCB1CmdIdGetId              = 0xFE    # get RCB-1 ID
RCB1CmdIdSetPos             = 0xFD    # set servo positions
RCB1CmdIdGetPos             = 0xFC    # get servo positions
RCB1CmdIdSetCurPosAsHome    = 0xFB    # set current position as home position
RCB1CmdIdGetHome            = 0xFA    # get home position
RCB1CmdIdSetMotionPos       = 0xF9    # set motion position
RCB1CmdIdGetMotionPos       = 0xF8    # get motion position
RCB1CmdIdSetMotionCnt       = 0xF7    # set motion's position count 
RCB1CmdIdGetMotionCnt       = 0xF6    # get motion's position count
RCB1CmdIdSetScenarioMotion  = 0xF5    # set scenario motion
RCB1CmdIdGetScenarioMotion  = 0xF4    # get scenario motion
RCB1CmdIdSetScenarioCnt     = 0xF3    # set scenario motion count
RCB1CmdIdGetScenarioCnt     = 0xF2    # get scenario motion count
RCB1CmdIdSetSwBits          = 0xF1    # set software bits
RCB1CmdIdGetSwBits          = 0xF0    # get software bits
RCB1CmdIdPlayMotion         = 0xEF    # play a motion
RCB1CmdIdPlayScenario       = 0xEE    # play a scenario
RCB1CmdIdSetKeyCtl          = 0xED    # set remote key control
RCB1CmdIdGetKeyCtl          = 0xEC    # get remote key control
RCB1CmdIdSetServoTrim       = 0xE9    # set servo trim
RCB1CmdIdGetServoTrim       = 0xE8    # get servo trim

#
# RCB-1 Response Values
#
RCB1RspAck            = KHR1Serial.ACK    # Response acknoledgement

#
# RCB-1 Command-Response Set Information
#   key:  hex command id
#   value:
#     desc    - short command description
#     cmdlen  - length of command including checksum
#     rsplen  - length of response including checksum or ack
#     rspchk  - response validation method: 'ack' or 'checksum'
#
RCB1CmdInfo = {
  RCB1CmdIdSetId1: {
    'desc':"Set RCB-1 Board ID",
    'cmdlen':4, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetId: {
    'desc':"Get RCB-1 Board ID",
    'cmdlen':2, 'rsplen':2, 'rspchk':'checksum'
  },
  RCB1CmdIdSetPos: {
    'desc':"Set <boardId> servo positions",
    'cmdlen':16, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetPos: {
    'desc':"Get <boardId> servo positions",
    'cmdlen':3, 'rsplen':14, 'rspchk':'checksum'
  },
  RCB1CmdIdSetCurPosAsHome: {
    'desc':"Set current <boardId> servo positions as <boardId>'s home position",
    'cmdlen':3, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetHome: {
    'desc':"Get <boardId> home position",
    'cmdlen':3, 'rsplen':14, 'rspchk':'checksum'
  },
  RCB1CmdIdSetMotionPos: {
    'desc':"Set <boardId> motion position",
    'cmdlen':18, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetMotionPos: {
    'desc':"Get <boardId> motion position",
    'cmdlen':5, 'rsplen':15, 'rspchk':'checksum'
  },
  RCB1CmdIdSetMotionCnt: {
    'desc':"Set <boardId> motion's position count",
    'cmdlen':5, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetMotionCnt: {
    'desc':"Get <boardId> motion's position count",
    'cmdlen':4, 'rsplen':3, 'rspchk':'checksum'
  },
  RCB1CmdIdSetScenarioMotion: {
    'desc':"Set <boardId> scenario motion",
    'cmdlen':6, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetScenarioMotion: {
    'desc':"Get <boardId> scenario motion",
    'cmdlen':5, 'rsplen':3, 'rspchk':'checksum'
  },
  RCB1CmdIdSetScenarioCnt: {
    'desc':"Set <boardId> scenario's motion count",
    'cmdlen':5, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetScenarioCnt: {
    'desc':"Get <boardId> scenario's motion count",
    'cmdlen':4, 'rsplen':3, 'rspchk':'checksum'
  },
  RCB1CmdIdSetSwBits: {
    'desc':"Set <boardId> software switch bits",
    'cmdlen':4, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetSwBits: {
    'desc':"Get <boardId> software switch bits",
    'cmdlen':3, 'rsplen':3, 'rspchk':'checksum'
  },
  RCB1CmdIdPlayMotion: {
    'desc':"Play a <boardId> motion",
    'cmdlen':4, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdPlayScenario: {
    'desc':"Play a <boardId> scenario",
    'cmdlen':4, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdSetKeyCtl: {
    'desc':"Set <boardId> remote key control command",
    'cmdlen':7, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetKeyCtl: {
    'desc':"Get <boardId> remote key control command",
    'cmdlen':4, 'rsplen':5, 'rspchk':'checksum'
  },
  RCB1CmdIdSetServoTrim: {
    'desc':"Set <boardId> servo trim",
    'cmdlen':15, 'rsplen':2, 'rspchk':'ack'
  },
  RCB1CmdIdGetServoTrim: {
    'desc':"Get <boardId> servo trim",
    'cmdlen':3, 'rsplen':14, 'rspchk':'checksum'
  },
}

#
# KHR-1 Data
#
KHR1BoardIdListDft    = [RCB1BoardIdZero, RCB1BoardIdOne] # KHR-1 default IDs
KHR1ServoVersionDft   = RCB1ServoVersionBlue  # Default servo version
KHR1NumOfChannelsDft  = RCB1NumOfChannels * 2 # Default number of channels/KHR-1

#
# Factory Defaults
#
KHR1FacDftActiveServos = {                    # active servos
  # RCB-1 Board ID 0
  'lrotshoulder': 0,    # left shoulder rotation motion
  'lshoulder':    1,    # left shoulder flexion/extension motion
  'lelbow':       2,    # left elbow flexion/extension motion
  'neck':         5,    # neck motion (side to side)
  'rrotshoulder': 6,    # right shoulder rotation motion
  'rshoulder':    7,    # right shoulder flexion/extension motion
  'relbow':       8,    # right elbow flexion/extension motion

  # RCB-1 Board ID 1
  'labdhip':      12,   # left hip abduction/adduction motion (side to side)
  'lhip':         13,   # left hip flexion/extension motion (back and front)
  'lknee':        14,   # left knee flexion/extension motion 
  'lankle':       15,   # left ankle flexion/extension motion (back and front)
  'llatankle':    16,   # left lateral/medial motion (side to side roll)
  'rabdhip':      18,   # right hip abduction/adduction motion (side to side)
  'rhip':         19,   # right hip flexion/extension motion (back and front)
  'rknee':        20,   # right knee flexion/extension motion 
  'rankle':       21,   # left ankle flexion/extension motion (back and front)
  'rlatankle':    22,   # right lateral/medial motion (side to side roll)
}

KHR1FacDftHome = [                            # home values
   5,   0,  90,  0,  0, 90, 175, 180, 90,  0,  0, 0, 
  88, 115, 115, 90, 92,  0,  92,  65, 65, 90, 88, 0
]

KHR1FacDftTrim = [0] * KHR1NumOfChannelsDft   # trim values


#-------------------------------------------------------------------------------
# CLASS: KHR1CmdBase
#-------------------------------------------------------------------------------
class KHR1CmdBase(KHR1Serial.KHR1Serial):
  """ Kondo KHR-1 Base Command and Response Class.

      There are two sets of commands:
        KHR-1 centric commands named Cmd<what>().
        RCB-1 centric commands named RCB1Cmd<what>().

      The KHR-1 typically contains 2 RCB-1's to control up to 24 servos.
      The KHR-1 centric commands keep the two RCB-1's in sync by issuing
      commands to both boards.

      The RCB-1 centric commands send messages to a specific RCB-1 
      identified by its Board ID.

      The KHR-1 commands must be provided with a a list of RCB-1 Board ID's
      in which to control and synchronize the robot's behaviors. The default
      set of Board ID's KHR1BoardIdListDft([0,1]) is almost always correct.
      The Board ID list is kept sorted in ascending order.
      See AttrSetBoardIdList().

      Board IDs are integers from [RCB1BoardIdMin(0), RCB1BoardIdMax(31)].

      An RCB-1 ordered channel list contains RCB1NumOfChannels(12) channel
      positions, in order of channel number. Each channel position specifies
      a position for the connected servo in integer degrees in the range
      [RCB1ServoPosMin(0), RCB1ServoPosMax(180)]. For the 'red' version
      of the servos, additional special position values may be specified:
        RCB1ServoPosFree(0xDD) - free
        RCB1ServoPos1(0xDE)    - position 1
        RCB1ServoPos2(0xDF)    - position 2
        RCB1ServoPos3(0xE0)    - position 3
        RCB1ServoPosLow(0xE1)  - position low
        RCB1ServoPosHigh(0xE2) - position high

     A KHR-1 ordered channel list is a concatenation of the RCB-1 ordered
     channel lists in Board ID ascending order. Typically, with two RCB-1's,
     there are KHR1NumOfChannelsDft(24) total channels.

     An RCB-1 ordered channel trim list is a channel list with retricted
     range of motion in integer degrees from
     [RCB1ServoTrimMin(-20), RCB1ServoTrimMax(19)].

     A KHR-1 ordered channel trim list is a concatenation of the RCB-1
     ordered channel trim lists in Board ID ascending order.

     A Servo speed is a unitless value in the range of:
      [RCB1ServoSpeedMin(7), RCB1ServoSpeedMax(0)] with
        7 == slowest speed      
        0 == fastest speed.

     The Software Switch Bits are servo control bits:
      RCB1SwBitSleep(0x01)    - TBD if set, do not control the servos (free)
      RCB1SwBitMotion(0x02)   - TBD 
  """

  def __init__(self, port=None,
                    boardIdList=KHR1BoardIdListDft,
                    servoVersion=KHR1ServoVersionDft,
                    activeServos=KHR1FacDftActiveServos,
                    dbgobj=None):
    """ Initialize a KHR-1 serial port object. If a serial port is
        specified, then the port will be opened. Otherwise, the KHR-1
        serial port object will be in the closed state.

        Parameters:
          port          - Serial port (device).
          boardIdList   - List of RCB-1 Board ID's for the KHR-1.
          servoVersion  - Version of the Kondo servos for the KHR-1.
          activeServos  - Dictionary of active servos defined mnemonics.
          dbgobj        - PyDebug object. None will create the object.
    """
    KHR1Serial.KHR1Serial.__init__(self, port, dbgobj)

    self.Init(boardIdList, servoVersion, activeServos)

  #--
  def Init(self, boardIdList, servoVersion, activeServos):
    """ One time initialization during object instantiation.

        Parameters:
          boardIdList   - List of RCB-1 Board ID's for the KHR-1
          servoVersion  - Version of the Kondo servos for the KHR-1
          activeServos  - Dictionary of active servos defined mnemonics.

        Return Value:
          None
    """
    self.AttrSetBoardIdList(boardIdList)
    self.AttrSetServoVersion(servoVersion)
    self.AttrSetActiveServos(**activeServos)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Attribute Methods
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def AttrSetBoardIdList(self, boardIdList):
    """ Set the list of RCB-1 board ID's for this KHR-1 robot.

        Note: The RCB-1 Board Id's are not assigned at the board
              level.

        Parameters:
          boardIdList - new list of board ID's

        Return Value:
          Returns new board ID list, sorted in ascending order.
    """
    if len(boardIdList) == 0:
      raise ValueError("KHR-1 requires at least 1 RCB-1 Board ID")
    self.mBoardIdList = []
    for boardId in boardIdList:
      if boardId < RCB1BoardIdMin or boardId > RCB1BoardIdMax:
        raise ValueError("%d: RCB-1 Board ID out-of-range" % boardId)
      self.mBoardIdList += [boardId]
    self.mNumOfChannels = len(self.mBoardIdList) * RCB1NumOfChannels
    self.mBoardIdList.sort()
    return self.mBoardIdList

  #--
  def AttrGetBoardIdList(self):
    """ Get the list of RCB-1 board ID's for this KHR-1 robot.

        Return Value:
          Returns current board ID list.
    """
    return self.mBoardIdList

  #--
  def AttrHasNumOfBoards(self):
    """ Get the number of RCB-1 boards for this KHR-1 robot.

        Return Value:
          Number of boards
    """
    return len(self.mBoardIdList)

  #--
  def AttrHasNumOfChannels(self):
    """ Get the number of servo channels for this KHR-1 robot.

        Return Value:
          Number of channels
    """
    return self.mNumOfChannels

  #--
  def AttrSetServoVersion(self, servoVersion):
    """ Set the servo version for this KHR-1 robot.

        Parameters:
          servoVersion - Version of the robot's servos. One of:
                          RCB1ServoVersionBlue - basic
                          RCB1ServoVersionRed  - advanced

        Return Value:
          Returns new servo version.
    """
    if servoVersion in [RCB1ServoVersionBlue, RCB1ServoVersionRed]: 
      self.mServoVersion = servoVersion
    else:
      raise ValueError("%s: KHR-1 servo version not recognized" % servoVersion)
    return self.mServoVersion

  #--
  def AttrGetServoVersion(self):
    """ Get the servo version for this KHR-1 robot.

        Return Value:
          Returns current servo version.
    """
    return self.mServoVersion

  #--
  def AttrSetActiveServos(self, **activeServos):
    """ Define this KHR-1's active servos through channel number
        mnemonic assignment. An active servo is connected to one
        of the RCB-1's channels and is marked for control.

        Parameters:
          **activeServos  - Keyword arguments <mnem>=<chanNum> specifying
                            active KHR-1 servos.

        Return Value:
          Returns new active servo mnemonic assignment.
    """
    self.mActiveServos = activeServos.copy()
    return self.mActiveServos

  #--
  def AttrGetActiveServos(self):
    """ Get the mnemonic assignment dictionary of active servos
        for the KHR-1.

        Return Value:
          {<mnem_0>:<chanNum_0>, ... }
    """
    return self.mActiveServos

  #--
  def AttrHasNumOfActiveServos(self):
    """ Get the number of active servos for this KHR-1.

        Return Value:
          Number of active servos
    """
    return len(self.mActiveServos)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # KHR-1 Robot Centric Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdSetCurPos(self, speed, chanList):
    """ Set KHR-1 current servo positions (0xFD).

        Parameters:
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          chanList    - KHR-1 ordered channel list.

        Return Value:
          Returns KHR-1 ordered channel list of new positions on success. 
          Returns None on failure.
    """
    if not chanList or len(chanList) != self.mNumOfChannels:
      return self.mErr.SetErrBadParam('chanList', 'invalid number of channels')
    start, end = 0, RCB1NumOfChannels
    for boardId in self.mBoardIdList:
      if self.RCB1CmdSetCurPos(boardId, speed, chanList[start:end]) is None:
        return None
      start, end = end, end + RCB1NumOfChannels
    return chanList

  #--
  def CmdGetCurPos(self):
    """ Get KHR-1 current servo positions (0xFC).

        Return Value:
          Returns KHR-1 ordered channel list of current positions on
          success.  Returns None on failure.
    """
    chanList = []
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetCurPos(boardId)
      if rsp is None:
        return None
      chanList += rsp
    return chanList
 
  #--
  def CmdSetIncCurPos(self, speed, **kwargs):
    """ Incrementally set KHR-1 current servo positions (0xFC, 0xFD).
        Any servos not specified will remain at their respective
        current positions.

        Parameters:
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          **kwargs    - Keyword arguments <mnem>=<pos> specifying
                        new servo positions.

        Return Value:
          Returns KHR-1 ordered channel list of new positions on success. 
          Returns None on failure.
    """
    chanList = self.CmdGetCurPos()
    if not chanList:
      return None
    for chanMnem, chanNum in self.mActiveServos.items():
      if chanMnem in kwargs and chanNum < len(chanList):
         chanList[chanNum] = self._GroomChannel(kwargs[chanMnem])
    return self.CmdSetCurPos(speed, chanList)

  #--
  def CmdSetCurPosAsHomePos(self):
    """ Set KHR-1 current position as the Home position (0xFB).

        Return Value:
          Returns KHR-1 ordered channel list of the Home position on
          success. Returns None on failure.
    """
    for boardId in self.mBoardIdList:
      if self.RCB1CmdSetCurPosAsHomePos(boardId) is None:
        return None
    return self.CmdGetHomePos()

  #--
  def CmdGetHomePos(self):
    """ Get KHR-1 Home servo positions (0xFA).

        Return Value:
          Returns KHR-1 ordered channel list of the Home position on
          success. Returns None on failure.
    """
    chanList = []
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetHomePos(boardId)
      if rsp is None:
        return None
      chanList += rsp
    return chanList
 
  #--
  def CmdGoToHomePos(self):
    """ Go to KHR-1 current Home position (0xFA, 0xFD).

        Return Value:
          Returns KHR-1 ordered channel list of the Home position on
          success. Returns None on failure.
    """
    chanList = self.CmdGetHomePos()
    if chanList:
      return self.CmdSetCurPos(RCB1ServoSpeedDft, chanList)
    else:
      return None

  #--
  def CmdResetHomePos(self):
    """ Reset KHR-1 Home position to factory default (0xFD, 0xFB).

        Return Value:
          Returns KHR-1 ordered channel list of the Home position on
          success. Returns None on failure.
    """
    chanList = self.CmdSetCurPos(RCB1ServoSpeedDft, KHR1FacDftHome)
    if chanList:
      return self.CmdSetCurPosAsHomePos()
    else:
      return None

  #--
  def CmdSetMotionPos(self, motionNum, posNum, speed, chanList):
    """ Set a KHR-1 motion position's data (0xF9).

        Parameters:
          motionNum   - One of the 40 motions [0,39].
          posNum      - One of 100 postions within the motion [0,99].
          speed       - Servos speed, 0 = max, 7 = min
          chanList    - KHR-1 ordered channel list on end positions.

        Return Value:
          On success, returns
            {'motion_num':<motionNum>, 'pos_num':<posNum>, 'speed':<speed>,
             'channel_list':<chanList>}
          Returns None on failure.
    """
    if not chanList or len(chanList) != self.mNumOfChannels:
      return self.mErr.SetErrBadParam('chanList', 'invalid number of channels')
    start, end = 0, RCB1NumOfChannels
    for boardId in self.mBoardIdList:
      if self.RCB1CmdSetMotionPos(boardId, motionNum, posNum, speed,
                                  chanList[start:end]) is None:
        return None
      start, end = end, end + RCB1NumOfChannels
    return {'motion_num':motionNum, 'pos_num':posNum, 'speed':speed,
            'channel_list':chanList}

  #--
  def CmdGetMotionPos(self, motionNum, posNum):
    """ Get a KHR-1 motion position's data (0xF8).

        Parameters:
          motionNum   - One of the 40 motions [0,39].
          posNum      - One of 100 postions within the motion [0,99].

        Return Value:
          On success, returns
            {'motion_num':<motionNum>, 'pos_num':<posNum>, 'speed':<speed>,
             'channel_list':<chanList>}
          Returns None on failure.
    """
    rvals = {'motion_num':motionNum, 'pos_num':posNum}
    chanList = []
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetMotionPos(boardId, motionNum, posNum)
      if rsp is None:
        return None
      if 'speed' not in rvals:
        rvals['speed'] = rsp[0]
      elif rvals['speed'] != rsp[0]:
        self.mErr.SetErrRcbSync( "speeds differ at "
            "motion %d, position %d" % (motionNum, posNum))
        return None
      chanList += rsp[1]
    rvals['channel_list'] = chanList
    return rvals

  #--
  def CmdSetIncMotionPos(self, motionNum, posNum, speed, **kwargs):
    """ Incrementally set a KHR-1 motion data (0xF8, 0xF9).
        Any servos not specified will take on the previous motion's
        position respective value. Position zero must be fully
        specified.

        Parameters:
          motionNum   - One of the 40 motions [0,39].
          posNum      - One of 100 postions within the motion [0,99].
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          **kwargs    - Keyword arguments <mnem>=<pos> specifying
                        servo positions.

        Return Value:
          On success, returns
            {'motion_num':<motionNum>, 'pos_num':<posNum>, 'speed':<speed>,
             'channel_list':<chanList>}
          Returns None on failure.
    """
    if posNum > 0:
      motion = self.CmdGetMotionPos(motionNum, posNum-1)
      if not motion:
        return None
      chanList = motion['channel_list']
    else:
      chanList = [0] * self.mNumOfChannels
    n = 0
    for chanMnem, chanNum in self.mActiveServos.items():
      if chanMnem in kwargs and chanNum < len(chanList):
         chanList[chanNum] = self._GroomChannel(kwargs[chanMnem])
         n += 1
    if posNum == 0 and n < self.AttrHasNumOfActiveServos():
      self.SetErrGeneral("motion position 0 requires %d active servos: "
        "%d specified" % (self.AttrHasNumOfActiveServos(), n))
      return None
    return self.CmdSetMotionPos(motionNum, posNum, speed, chanList)

  #--
  def CmdSetMotionPosCnt(self, motionNum, posCnt):
    """ Set a KHR-1 motion's position count (number of positions) (0xF7).

        Parameters:
          motionNum   - One of the 40 motions [0,39].
          posCnt      - Position count for this motion [0,100]

        Return Value:
          Returns {'motion_num':<motionNum>, 'pos_cnt':<posCnt>}
          on success. Returns None on failure.
    """
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdSetMotionPosCnt(boardId, motionNum, posCnt)
      if rsp is None:
        return None
    return {'motion_num':motionNum, 'pos_cnt':posCnt}

  #--
  def CmdGetMotionPosCnt(self, motionNum):
    """ Get a KHR-1 motion's position count (number of postions) (0xF6).

        Parameters:
          motionNum   - One of the 40 motions [0,39].

        Return Value:
          Returns {'motion_num':<motionNum>, 'pos_cnt':<posCnt>}
          on success. Returns None on failure.
    """
    rvals = {'motion_num':motionNum}
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetMotionPosCnt(boardId, motionNum)
      if rsp is None:
        return None
      if 'pos_cnt' not in rvals:
        rvals['pos_cnt'] = rsp
      elif rvals['pos_cnt'] != rsp:
        self.mErr.SetErrRcbSync("position counts differ for motion %d" % \
            (motionNum))
        return None
    return rvals

  #--
  def CmdDumpMotion(self, motionNum):
    """ Dump all data for a KHR-1 motion (0xF6, 0xF8).

        Parameters:
          motionNum   - One of the 40 motions [0,39].

        Return Value:
          On success, returns:
            {'motion_num': motionNum, 
             'motion':[{'speed':<speed>, 'channel_list':<chanList>}, ...]}
          Returns None on failure.
    """
    rvals = {'motion_num':motionNum}
    rsp = self.CmdGetMotionPosCnt(motionNum)
    if rsp is None:
      return None
    posCnt = rsp['pos_cnt']
    posNum = 0
    motionData = []
    while posNum < posCnt:
      mo = self.CmdGetMotionPos(motionNum, posNum)
      if mo is None:  # dropped data
        motionData += [{'speed':'?', 'channel_list':'?'}]
      else:
        motionData += [{'speed':mo['speed'], 'channel_list':mo['channel_list']}]
      posNum += 1
    rvals['motion'] = motionData
    return rvals

  #--
  def CmdSetScenarioMotion(self, scenarioNum, motionIdx, motionNum):
    """ Set a KHR-1 scenario's motion index's data (0xF5).

        Parameters:
          scenarioNum - One of the 4 scenarios [0-3].
          motionIdx   - One of 200 motion indices within the scenario
                        [0,199].
          motionNum   - One of the 40 programmed motions [0,39].

        Return Value:
          On success, returns
            {'scenario_num':<scenarioNum>, 'motion_index':<motionIdx>,
             'motion_num':<motionNum>}
          Returns None on failure.
    """
    for boardId in self.mBoardIdList:
      if self.RCB1CmdSetScenarioMotion(boardId, scenarioNum, motionIdx,
                                       motionNum) is None:
        return None
    return {'scenario_num':scenarioNum, 'motion_index':motionIdx,
            'motion_num':motionNum}

  #--
  def CmdGetScenarioMotion(self, scenarioNum, motionIdx):
    """ Get a KHR-1 scenario's motion index's data (0xF4).

        Parameters:
          scenarioNum - One of the 4 scenarios [0-3].
          motionIdx   - One of 200 motion indices within the scenario                                   [0,199].

        Return Value:
          On success, returns
            {'scenario_num':<scenarioNum>, 'motion_index':<motionIdx>,
             'motion_num':<motionNum>}
          Returns None on failure.
    """
    rvals = {'scenario_num':scenarioNum, 'motion_index':motionIdx}
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetScenarioMotion(boardId, scenarioNum, motionIdx)
      if rsp is None:
        return None
      if 'motion_num' not in rvals:
        rvals['motion_num'] = rsp
      elif rvals['motion_num'] != rsp:
        self.mErr.SetErrRcbSync("motion numbers differ at "
            "scenario %d, index %d" % (scenarioNum, motionIdx))
        return None
    return rvals

  #--
  def CmdSetScenarioMotionCnt(self, scenarioNum, motionCnt):
    """ Set a KHR-1 scenario's motion count (number of motions) (0xF3).

        Parameters:
          scenarioNum - One of the 4 scenarios [0-3].
          motionCnt   - Motion count for this scenario [0,200]

        Return Value:
          On success, returns
            {'scenario_num':<scenarioNum>, 'motion_cnt':<motionCnt>}
          Returns None on failure.
    """
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdSetScenarioMotionCnt(boardId, scenarioNum, motionCnt)
      if rsp is None:
        return None
    return {'scenario_num':scenarioNum, 'motion_cnt':motionCnt}

  #--
  def CmdGetScenarioMotionCnt(self, scenarioNum):
    """ Get a KHR-1 scenario's motion count (number of motions) (0xF2).

        Parameters:
          scenarioNum - One of the 4 scenarios [0-3].

        Return Value:
          On success, returns
            {'scenario_num':<scenarioNum>, 'motion_cnt':<motionCnt>}
          Returns None on failure.
    """
    rvals = {'scenario_num':scenarioNum}
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetScenarioMotionCnt(boardId, scenarioNum)
      if rsp is None:
        return None
      if 'motion_cnt' not in rvals:
        rvals['motion_cnt'] = rsp
      elif rvals['motion_cnt'] != rsp:
        self.mErr.SetErrRcbSync("motion counts differ for scenario %d" % \
            (scenarioNum))
        return None
    return rvals

  #--
  def CmdDumpScenario(self, scenarioNum):
    """ Dump all data for a KHR-1 scenario (0xF2, 0xF4).

        Parameters:
          scenarioNum - One of the 4 scenarios [0-3].

        Return Value:
          On success, returns:
            {'scenario_num': scenarioNum, 'motion_num_list':<motionNumList>}
          Returns None on failure.
    """
    rvals = {'secnario_num':scenarioNum}
    rsp = self.CmdGetScenarioMotionCnt(scenarioNum)
    if rsp is None:
      return None
    motionCount = rsp['motion_cnt']
    motionIndex = 0
    motionNumList = []
    while motionIndex < motionCount:
      mo = self.CmdGetScenarioMotion(scenarioNum, motionIndex)
      if mo is None:  # dropped data
        motionNumList += ['?']
      else:
        motionNumList += [mo['motion_num']]
      motionIndex += 1
    rvals['motion_num_list'] = motionNumList
    return rvals

  #--
  def CmdSetSwBits(self, sleep=0, motion=1):
    """ Set the KHR-1 Software Switch Bits (0xF1).

        Parameters:
          sleep     - Do [not] sleep
          motion    - Do [not] enable motion

        Return Value:
          On success, returns
            {'sleep':<swbit>, 'motion':<swbit>}
              where <swbit> = 0 (off) or 1 (on)
          Returns None on failure.
    """
    swBits = 0
    if sleep:
      swBits |= RCB1SwBitSleep        
    if motion:
      swBits |= RCB1SwBitMotion
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdSetSwBits(boardId, swBits)
      if rsp is None:
        return None
    return {'sleep':sleep, 'motion':motion}

  #--
  def CmdGetSwBits(self):
    """ Get the KHR-1 Software Switch Bits (0xF0).

        Return Value:
          On success, returns
            {'sleep':<swbit>, 'motion':<swbit>}
              where <swbit> = 0 (off) or 1 (on)
          Returns None on failure.
    """
    swBits = None
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetSwBits(boardId)
      if rsp is None:
        return None
      if swBits is None:
        swBits = rsp
      elif swBits != rsp:
        self.mErr.SetErrRcbSync("software switch bits differ")
        return None
    rvals = {}
    if swBits & RCB1SwBitSleep:
      rvals['sleep'] = 1
    else:
      rvals['sleep'] = 0
    if swBits & RCB1SwBitMotion:
      rvals['motion'] = 1
    else:
      rvals['motion'] = 0
    return rvals

  #--
  def CmdPlayMotion(self, motionNum):
    """ Play a KHR-1 motion (0xEF).

        Parameters:
          motionNum   - One of the 40 motions [0,39].

        Return Value:
          Returns motion played on success, None on failure.
    """
    for boardId in self.mBoardIdList:
      if self.RCB1CmdPlayMotion(boardId, motionNum) is None:
        return None
    return motionNum

  #--
  def CmdPlayScenario(self, scenarioNum):
    """ Play a KHR-1 scenario (0xEE).

        Parameters:
          scenarioNum - scenario number

        Return Value:
          Returns scenario played on success, None on failure.
    """
    for boardId in self.mBoardIdList:
      if self.RCB1CmdPlayScenario(boardId, scenarioNum) is None:
        return None
    return scenarioNum

  #--
  def CmdSetTrim(self, chanTrimList):
    """ Set KHR-1 servo trim positions (0xE9).

        Parameters:
          chanTrimList  - KHR-1 ordered channel trim list. Each value
                          in degress is in [-20,19].

        Return Value:
          Returns KHR-1 ordered channel list of new trim positions on
          success.  Returns None on failure.
    """
    start, end = 0, RCB1NumOfChannels
    for boardId in self.mBoardIdList:
      if self.RCB1CmdSetTrim(boardId, chanTrimList[start:end]) is None:
        return None
      start, end = end, end + RCB1NumOfChannels
    return chanTrimList

  #--
  def CmdGetTrim(self):
    """ Get KHR-1 servo trim positions (0xE8).

        Return Value:
          Returns KHR-1 ordered channel trim list of current trim positions
          on success. Each trim position is in degrees from [-20,19].
          Returns None on failure.
    """
    chanTrimList = []
    for boardId in self.mBoardIdList:
      rsp = self.RCB1CmdGetTrim(boardId)
      if rsp is None:
        return None
      else:
        chanTrimList += rsp
    return chanTrimList


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Lower-Level RCB-1 Centric Commands
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def RCB1CmdSetBoardId(self, boardId):
    """ Set RCB-1 board ID (0xFF5A).

        Parameters:
          boardId     - RCB-1 board ID [0,31]

        Return Value:
          Returns new board ID on success, None on failure.
    """
    cmdId = RCB1CmdIdSetId1
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, RCB1CmdIdSetId2, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if rsp is not None:
      return rsp[0]
    else:
      return None

  #--
  def RCB1CmdGetBoardId(self):
    """ Get RCB-1 board ID (0xFE).

        Note: This command does not always return the expected value,
              returning randomly one of the two connected board ID's.

        Return Value:
          Returns current board ID on success, None on failure.
    """
    cmdId = RCB1CmdIdGetId
    cmdInfo = RCB1CmdInfo[cmdId]
    rsp = self.SendCmd([cmdId], cmdInfo['rsplen'], cmdInfo['rspchk'])
    if rsp is not None:
      return rsp[0]
    else:
      return None

  #--
  def RCB1CmdSetCurPos(self, boardId, speed, chanList):
    """ Set RCB-1 current servo positions (0xFD).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          chanList    - RCB-1 ordered channel list.

        Return Value:
          Returns RCB-1 ordered channel list of new positions on success.
          Returns None on failure.
    """
    cmdId = RCB1CmdIdSetPos
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    speed = self._cap(speed, RCB1ServoSpeedMax, RCB1ServoSpeedMin)
    if not chanList or len(chanList) != RCB1NumOfChannels:
      return self.mErr.SetErrBadParam('chanList', 'invalid number of channels')
    chanList = self._GroomChannelList(chanList)
    cmd = [cmdId, boardId, speed] + chanList
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return chanList
    else:
      return None

  #--
  def RCB1CmdGetCurPos(self, boardId):
    """ Get RCB-1 current servo positions (0xFC).

        Parameters:
          boardId     - RCB-1 board ID [0,31]

        Return Value:
          Returns RCB-1 ordered channel list of current servo positions
          on success. Returns None on failure.
    """
    cmdId = RCB1CmdIdGetPos
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1:]
    else:
      return None

  #--
  def RCB1CmdSetCurPosAsHomePos(self, boardId):
    """ Set RCB-1 current position as it's Home position (0xFB).

        Parameters:
          boardId     - RCB-1 board ID [0,31]

        Return Value:
          Returns the board ID on success, None on failure.
    """
    cmdId = RCB1CmdIdSetCurPosAsHome
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return boardId
    else:
      return None

  #--
  def RCB1CmdGetHomePos(self, boardId):
    """ Get RCB-1 Home servo positions (0xFA).

        Parameters:
          boardId     - RCB-1 board ID [0,31]

        Return Value:
          Returns RCB-1 ordered channel list of servo Home positions
          on success. Returns None on failure.
    """
    cmdId = RCB1CmdIdGetHome
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1:]
    else:
      return None

  #--
  def RCB1CmdSetMotionPos(self, boardId, motionNum, posNum, speed, chanList):
    """ Set an RCB-1 motion position's data (0xF9).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          motionNum   - One of the 40 motions. [0,39]
          posNum      - One of 100 postions within the motion [0,99].
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          chanList    - RCB-1 ordered channel list.

        Return Value:
          Returns posNum on success, None on failure.
    """
    cmdId = RCB1CmdIdSetMotionPos
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    posNum = self._cap(posNum, RCB1PosNumMin, RCB1PosNumMax)
    speed = self._cap(speed, RCB1ServoSpeedMax, RCB1ServoSpeedMin)
    if not chanList or len(chanList) != RCB1NumOfChannels:
      return self.mErr.SetErrBadParam('chanList', 'invalid number of channels')
    chanList = self._GroomChannelList(chanList)
    cmd = [cmdId, boardId, motionNum, posNum, speed] + chanList
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return posNum
    else:
      return None

  #--
  def RCB1CmdGetMotionPos(self, boardId, motionNum, posNum):
    """ Get an RCB-1 motion position's data (0xF8).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          motionNum   - One of the 40 motions. [0,39]
          posNum      - One of 100 postions within the motion [0,99].

        Return Value:
          Returns postion data (speed, chanList) at position number
          on success. Returns None on failure.
    """
    cmdId = RCB1CmdIdGetMotionPos
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    posNum = self._cap(posNum, RCB1PosNumMin, RCB1PosNumMax)
    cmd = [cmdId, boardId, motionNum, posNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return (rsp[1], rsp[2:])
    else:
      return None

  #--
  def RCB1CmdSetMotionPosCnt(self, boardId, motionNum, posCnt):
    """ Set an RCB-1 motion's position count (number of positions) (0xF7).

        Parameters:
          boardId     - RCB-1 board ID [0,31].
          motionNum   - One of the 40 motions [0,39].
          posCnt      - Position count for this motion [0,100]

        Return Value:
          Returns posCnt on success, None on failure.
    """
    cmdId = RCB1CmdIdSetMotionCnt
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    posCnt = self._cap(posCnt, RCB1PosNumMin, RCB1NumOfMotionPos)
    cmd = [cmdId, boardId, motionNum, posCnt]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return posCnt
    else:
      return None

  #--
  def RCB1CmdGetMotionPosCnt(self, boardId, motionNum):
    """ Get an RCB-1 motion's position count (number of positions) (0xF6).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          motionNum   - One of the 40 motions [0,39]

        Return Value:
          Returns position count [0,100] on success, None on failure.
    """
    cmdId = RCB1CmdIdGetMotionCnt
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    cmd = [cmdId, boardId, motionNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1]
    else:
      return None

  #--
  def RCB1CmdSetScenarioMotion(self, boardId, scenarioNum, motionIdx,
                                     motionNum):
    """ Set an RCB-1 scenario's motion index's data (0xF5).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          scenarioNum - One of the 4 scenarios [0-3].
          motionIdx   - One of 200 motion indices within the scenario
                        [0,199].
          motionNum   - One of the 40 programmed motions [0,39].

        Return Value:
          Returns motion index on success, None on failure.
    """
    cmdId = RCB1CmdIdSetScenarioMotion
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    scenarioNum = self._cap(scenarioNum, RCB1ScenarioMin, RCB1ScenarioMax)
    motionIdx = self._cap(motionIdx, RCB1MotionIdxMin, RCB1MotionIdxMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    cmd = [cmdId, boardId, scenarioNum, motionIdx, motionNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return motionIdx
    else:
      return None

  #--
  def RCB1CmdGetScenarioMotion(self, boardId, scenarioNum, motionIdx):
    """ Get an RCB-1 scenario's motion index's data (0xF4).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          scenarioNum - One of the 4 scenarios [0-3].
          motionIdx   - One of 200 motion indices within the scenario
                        [0,199].

        Return Value:
          Returns motion number at index on success, None on failure.
    """
    cmdId = RCB1CmdIdGetScenarioMotion
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    scenarioNum = self._cap(scenarioNum, RCB1ScenarioMin, RCB1ScenarioMax)
    motionIdx = self._cap(motionIdx, RCB1MotionIdxMin, RCB1MotionIdxMax)
    cmd = [cmdId, boardId, scenarioNum, motionIdx]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1]
    else:
      return None

  #--
  def RCB1CmdSetScenarioMotionCnt(self, boardId, scenarioNum, motionCnt):
    """ Set an RCB-1 scenario's motion count (number of motions) (0xF3).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          scenarioNum - One of the 4 scenarios [0-3].
          motionCnt   - Motion count for this motion [0,200]

        Return Value:
          Returns scenario's last index on success, None on failure.
    """
    cmdId = RCB1CmdIdSetScenarioCnt
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    scenarioNum = self._cap(scenarioNum, RCB1ScenarioMin, RCB1ScenarioMax)
    motionCnt = self._cap(motionCnt, RCB1MotionIdxMin, RCB1NumOfScenarioMotions)
    cmd = [cmdId, boardId, scenarioNum, motionCnt]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return motionCnt
    else:
      return None

  #--
  def RCB1CmdGetScenarioMotionCnt(self, boardId, scenarioNum):
    """ Get an RCB-1 scenario's motion count (number of motions) (0xF2).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          scenarioNum - One of the 4 scenarios [0-3].

        Return Value:
          Returns scenario's motion count [0,200] on success,
          None on failure.
    """
    cmdId = RCB1CmdIdGetScenarioCnt
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    scenarioNum = self._cap(scenarioNum, RCB1ScenarioMin, RCB1ScenarioMax)
    cmd = [cmdId, boardId, scenarioNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1]
    else:
      return None

  #--
  def RCB1CmdSetSwBits(self, boardId, swBits):
    """ Set the RCB-1 Software Switch Bits (0xF1).

        Parameters:
          boardId   - RCB-1 board ID [0,31]
          swBits    - Software switch bits

        Return Value:
          Returns byte of software switch bits settings on success. 
          Returns None on failure.
    """
    cmdId = RCB1CmdIdSetSwBits
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId, swBits]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return swBits
    else:
      return None

  #--
  def RCB1CmdGetSwBits(self, boardId):
    """ Get the RCB-1 Software Switch Bits (0xF0).

        Parameters:
          boardId   - RCB-1 board ID [0,31]

        Return Value:
          Returns byte of software switch bits settings on success. 
          Returns None on failure.
    """
    cmdId = RCB1CmdIdGetSwBits
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return rsp[1]
    else:
      return None

  #--
  def RCB1CmdPlayMotion(self, boardId, motionNum):
    """ Play an RCB-1 motion (0xEF).

        Parameters:
          boardId     - RCB-1 board ID [0,31]
          motionNum   - One of the 40 motions [0,39]

        Return Value:
          Returns motion played on success, None on failure.
    """
    cmdId = RCB1CmdIdPlayMotion
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    motionNum = self._cap(motionNum, RCB1MotionNumMin, RCB1MotionNumMax)
    cmd = [cmdId, boardId, motionNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return motionNum
    else:
      return None

  #--
  def RCB1CmdPlayScenario(self, boardId, scenarioNum):
    """ Play an RCB-1 scenario (0xEE).

        Parameters:
          boardId     - RCB-1 board ID [0,31].
          scenarioNum - One of the 4 scenarios [0-3].

        Return Value:
          Returns scenario played on success, None on failure.
    """
    cmdId = RCB1CmdIdPlayScenario
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    scenarioNum = self._cap(scenarioNum, RCB1ScenarioMin, RCB1ScenarioMax)
    cmd = [cmdId, boardId, scenarioNum]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return scenarioNum
    else:
      return None

  #--
  def RCB1CmdSetTrim(self, boardId, chanTrimList):
    """ Set RCB-1 servo trim positions (0xE9).

        Parameters:
          boardId       - RCB-1 board ID [0,31]
          chanTrimList  - RCB-1 ordered channel trim list. Each value
                          in degress is in [-20,19].

        Return Value:
          Returns RCB-1 ordered channel trim list of new trim positions on
          success.  Returns None on failure.
    """
    cmdId = RCB1CmdIdSetServoTrim
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    if not chanTrimList or len(chanTrimList) != RCB1NumOfChannels:
      return self.mErr.SetErrBadParam('chanTrimList',
                                      'invalid number of channels')
    chanCvtList = []
    for trim in chanTrimList:
      trim += RCB1ServoTrimZero
      trim = self._cap(trim, RCB1ServoTrimMin, RCB1ServoTrimMax)
      chanCvtList += [trim]
    cmd = [cmdId, boardId] + chanCvtList
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      return chanTrimList
    else:
      return None

  #--
  def RCB1CmdGetTrim(self, boardId):
    """ Get RCB-1 servo trim positions (0xE8).

        Parameters:
          boardId       - RCB-1 board ID [0,31]

        Return Value:
          Returns RCB-1 ordered channel trim list of current trim positions
          on success. Each trim position is in degrees from [-20,19].
          Returns None on failure.
    """
    cmdId = RCB1CmdIdGetServoTrim
    cmdInfo = RCB1CmdInfo[cmdId]
    boardId = self._cap(boardId, RCB1BoardIdMin, RCB1BoardIdMax)
    cmd = [cmdId, boardId]
    rsp = self.SendCmd(cmd, cmdInfo['rsplen'], cmdInfo['rspchk'])
    if self._ValidateBoardIdInRsp(rsp, boardId):
      chanTrimList = []
      for trim in rsp[1:]:
        chanTrimList += [trim-RCB1ServoTrimZero]
      return chanTrimList
    else:
      return None


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Private Interface
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _GroomChannelList(self, chanList):
    """ Groom channel list, forcing each position to be in valid range.

        Parameters:
          chanList      - list of channel positions

        Return Value:
          Returns groomed channel list.
    """
    groomedChanList = []
    for pos in chanList:
      groomedChanList += [self._GroomChannel(pos)]
    return groomedChanList

  #--
  def _GroomChannel(self, pos):
    """ Groom channel servo position, forcing position to be in valid range.

        Parameters:
          pos           - channel position

        Return Value:
          Returns groomed position.
    """
    if self.mServoVersion == RCB1ServoVersionRed and \
        pos >= RCB1ServoPosSpecMin and pos <= RCB1ServoPosSpecMax:
      return pos
    else:
      return self._cap(pos, RCB1ServoPosMin, RCB1ServoPosMax)

  #--
  def _ValidateBoardIdInRsp(self, rsp, boardId):
    """ Validate RCB-1 Board ID in response message.

        Parameters:
          rsp      - response message: [boardId, ...]
          boardId  - expected RCB-1 board ID

        Return Value:
          Returns True if valid board id, False otherwise.
    """
    if not rsp:
      return False
    elif rsp[0] != boardId:
      self.mErr.SetErrBadRsp( "Bad board ID - expected %d" % (boardId), rsp)
      return False
    else:
      return True

  #--
  def _cap(self, val, min, max):
    """ Cap value between minimum and maximum values.

        Parameters:
          val   - Value to cap.
          min   - The minimum the value can take on.
          max   - The maximum the value can take on.

        Return Value:
          Capped value.
    """
    if val < min: return min
    elif val > max: return max
    else: return val
