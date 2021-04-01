################################################################################
#
# StandTall.py
#

""" KHR-2 Stand Tall Module.

The Stand Tall module uses PIDs to keep the KHR-2 robot standing upright,
while lateral and varied forces are applied to the robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.11.10

Copyright (C) 2007.  RoadNarrows LLC.
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

import sys
import socket
import errno
import time
import math
import random
import threading as thread

import Fusion.KHR2.Cmd.BsProxyMsgDef as MsgDef
import Fusion.KHR2.Cmd.BsProxyClient as BsProxyClient
import Fusion.Acc.Pid as Pid

ServoNeck       =  0 
ServoLHipAbd    = 10
ServoLHip       = 11
ServoLAnkle     = 13
ServoLAnkleLat  = 14
ServoRHipAbd    = 16
ServoRHip       = 17
ServoRKnee      = 18
ServoRAnkle     = 19
ServoRAnkleLat  = 20

DirLHipAbd      = -1.0
DirRHipAbd      = -1.0
DirLAnkleLat    =  1.0
DirRAnkleLat    =  1.0

DirLHip         = -1.0
DirRHip         =  1.0
DirLAnkle       = -1.0
DirRAnkle       =  1.0

BpFootWidth       = 32.5        # mm from left to right sensor columns
BpFootHeight      = 86.0        # mm from top to bottom sensor rows
BpFootPtFulcrum   = (BpFootWidth/2.0, BpFootHeight/2.0)
                                # point fulcrum position at center of foot

# foot sensor index order
BpFootSensorUppL  = 0     # foot upper left
BpFootSensorUppR  = 1     # foot upper right
BpFootSensorMidL  = 2     # foot middle left
BpFootSensorMidR  = 3     # foot middle right
BpFootSensorLowL  = 4     # foot lower left
BpFootSensorLowR  = 5     # foot lower right
BpFootSensorToeL  = 6     # foot toe left
BpFootSensorToeR  = 7     # foot toe right

ComLeftX          = 0.0   # Center of Mass, left foot, x
ComLeftY          = 0.0   # Center of Mass, left foot, y
ComRightX         = 0.0   # Center of Mass, right foot, x
ComRightY         = 0.0   # Center of Mass, right foot, y

# concurrency semaphore
semaFoot = thread.Semaphore()
semaFoot.release()

#--
def centerofmass(sensors):
  """ Calculate a foot's center of masses of the sole.

        Parameters:
          sensors     - vector of sensor data
  """
  M     = 0.0   # total 'mass'
  x_com = 0.0   # x's center of mass
  y_com = 0.0   # y's center of mass

  # x center of mass, plus total mass
  sign = 1.0
  for i in range(0,6):
    x_com += sign * sensors[i] * BpFootPtFulcrum[0]
    M  += sensors[i]
    sign *= -1.0

  # y center of mass
  y_com = (sensors[BpFootSensorUppL] + sensors[BpFootSensorUppR] - \
           sensors[BpFootSensorLowL] - sensors[BpFootSensorLowR]) * \
           BpFootPtFulcrum[1]

  if M > 0.0:
    return (x_com/M, y_com/M)
  else:
    return (0.0, 0.0)

def baseposition():
  """ Configure robot into stable standing position """
  for servo in [ServoLAnkleLat, ServoRAnkleLat,
                ServoLAnkle, ServoLHip,
                ServoRAnkle, ServoRHip]:
    rsp = BsRcb3.CmdRCB3Move(servo, 50, 0.0)
    if not rsp:
      return
  BsRcb3.CmdRCB3Move(ServoLHipAbd, 50, DirLHipAbd*10.0)
  BsRcb3.CmdRCB3Move(ServoRHipAbd, 50, DirRHipAbd*10.0)

def CbComInputRoll():
  """ Roll PID input (setpoint, pv) callback """
  global ComLeftX, ComLeftY, ComRightX, ComRightY
  semaFoot.acquire()
  rsp = BsFoot.CmdBpFoot('bpfoot_left', 'getraw')
  if rsp:
    ComLeftX, ComLeftY = centerofmass(rsp['raw_data'])
  rsp = BsFoot.CmdBpFoot('bpfoot_right', 'getraw')
  if rsp:
    ComRightX, ComRightY = centerofmass(rsp['raw_data'])
  semaFoot.release()
  return 0.0, ComLeftX - ComRightX

def CbComOutputRoll(pos):
  """ Roll PID output (servo degrees) callback """
  pass
  #BsRcb3.CmdRCB3Move(ServoLAnkleLat, 50, DirLAnkleLat*pos)
  #BsRcb3.CmdRCB3Move(ServoLHipAbd, 50, DirLHipAbd*pos)
  #BsRcb3.CmdRCB3Move(ServoRAnkleLat, 50, DirRAnkleLat*pos)
  #BsRcb3.CmdRCB3Move(ServoRHipAbd, 50, DirRHipAbd*pos)

def CbComInputPitch():
  """ Pitch PID input (setpoint, pv) callback """
  return 0.0, ComLeftY + ComRightY

def CbComOutputPitch(pos):
  """ Pitch PID output (servo degrees) callback """
  BsRcb3.CmdRCB3Move(ServoLAnkle, 50, DirLAnkle*pos)
  BsRcb3.CmdRCB3Move(ServoLHip, 50, DirLHip*pos)
  BsRcb3.CmdRCB3Move(ServoRAnkle, 50, DirRAnkle*pos)
  BsRcb3.CmdRCB3Move(ServoRHip, 50, DirRHip*pos)


#
# Main
#
if __name__ == "__main__":

  # create botsense proxy clients
  BsFoot = BsProxyClient.BsProxyClient(bsproxy_addr='192.168.0.21')
  BsRcb3 = BsProxyClient.BsProxyClient(bsproxy_addr='127.0.0.1')

  # connect clients to respective proxy servers
  BsFoot.Connect()
  BsRcb3.Connect()

  # open RCB-3 proxied device
  BsRcb3.CmdDevOpen('rcb3', 0, '/dev/robot0')

  # wait to get RCB-3 version (communication is good)
  while True:
    version = BsRcb3.CmdRCB3Version()
    if version:
      print(version)
      break
    else:
      time.sleep(1)
 
  # open BrainPack left and right proxied devices
  BsFoot.CmdDevOpen('bpfoot_left', 0x10, '/dev/i2c/0')
  BsFoot.CmdDevOpen('bpfoot_right', 0x11, '/dev/i2c/0')

  # put robot into stable standing position
  baseposition()

  # Roll (side to side) PID
  pidRoll = Pid.Pid(CbComInputRoll, CbComOutputRoll, 
              0.0,                        # cv (degrees)
              0.25,                       # dt
              0.9, 0.5, 0.4,              # Kp, Ki, Kd
              Wp=1.0, Wd=1.0,             # error weights
              cv_min=-15.0, cv_max=15.0,  # cv min and max ranges (degrees)
              trace=True)

  # Pitch (front to back) PID
  pidPitch = Pid.Pid(CbComInputPitch, CbComOutputPitch,
              0.0,                        # cv (degrees)
              0.25,                       # dt
              0.2, 0.1, 0.2,              # Kp, Ki, Kd
              Wp=1.0, Wd=1.0,             # error weights
              cv_min=-20.0, cv_max=20.0,  # cv min and max ranges (degrees)
              trace=True)

  print("Press <enter> to terminate...")
  time.sleep(1)

  # start active control loops
  pidRoll.start()
  pidPitch.start()

  # wait for <enter>
  input()

  # stop active controls
  pidRoll.cancel()
  pidPitch.cancel()

  # put robot into stable standing position
  baseposition()

  # disconnect from proxy servers
  BsRcb3.Disconnect()
  BsFoot.Disconnect()
