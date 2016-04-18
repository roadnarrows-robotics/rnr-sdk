#
# Module:   Laelaps.Imu
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     Imu.py
#
## \file
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $  
## $Rev: 4293 $ 
## 
## \brief Laelaps Ineria Measurement Unit module.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.
##   (http://www.roadnarrows.com)
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

import os
import serial
import struct
import time
import math


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# MSP Protocol
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#
# Message layouts
#
MSP_PREAMBLE      ='$M'         # command/response preamble
MSP_DIR_TO        ='<'          # to IMU 
MSP_DIR_FROM      ='>'          # from IMU
MSP_CMD_PREAMBLE  = MSP_PREAMBLE + MSP_DIR_TO     # command preamble
MSP_RSP_PREAMBLE  = MSP_PREAMBLE + MSP_DIR_FROM   # response preamble

MSP_POS_SIZE    = 3     ## data size position in message
MSP_POS_CMD_ID  = 4     ## command id position in message
MSP_POS_DATA    = 5     ## start of data position in message

## command header length (preamble + size + cmdid)
MSP_CMD_HDR_LEN = len(MSP_CMD_PREAMBLE) + 2

## command minimum length (no data)
MSP_CMD_MIN_LEN = MSP_CMD_HDR_LEN + 1

## response header length (preamble + size + cmdid)
MSP_RSP_HDR_LEN = len(MSP_RSP_PREAMBLE) + 2 

## response minimum length (no data)
MSP_RSP_MIN_LEN = MSP_RSP_HDR_LEN + 1

#
#  Command Ids
#
MSP_IDENT     = 100       # get firmware and hardware identities
MSP_RAW_IMU   = 102       # get raw imu data
MSP_ATTITUDE  = 108       # get attitude (angx, angy, heading)

#
# Axes degrees of freedom
#
X     = 0       ## x axis index
Y     = 1       ## y axis index
Z     = 2       ## z axis index
ROLL  = 0       ## roll index
PITCH = 1       ## pitch index
YAW   = 2       ## yaw index
NUM_AXES  = 3   ## number of axes

# Conversion factors and units
G                       = 9.80665           ## standard gravity in m/s^2
DEG_TO_RAD              = math.pi / 180.0   ## math can save your life

MSP_ACCEL_RAW_TO_G        = 1.0 / 512.0   ## raw IMU acceleration conversion
MSP_GYRO_RAW_TO_DEG_PER_S = 1.0 / 4.096   ## raw IMU rotation conversion
MSP_ATTITUDE_RAW_TO_DEG   = 0.1           ## attitude conversion

UNITS_RAW       = 'raw'       ## raw sensor values
UNITS_G         = 'g'         ## acceleration in standard g's
UNITS_M_PER_S2  = 'm/s^2'     ## acceleration in meters/second^2
UNITS_DEG_PER_S = 'deg/s'     ## degrees/second
UNITS_RAD_PER_S = 'rad/s'     ## radians/second
UNITS_DEG       = 'degrees'   ## degrees
UNITS_RAD       = 'radians'   ## radians
UNITS_SI        = 'si'        ## international system of units
                              ##  (m, s, radians, m/s, m/s^2, radians/s)

# ------------------------------------------------------------------------------
# ImuException Class
# ------------------------------------------------------------------------------

##
## \brief IMU exception class.
##
class ImuException(Exception):
  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.message = msg

  def __repr__(self):
    return "ImuException(%s)" % (repr(self.message))

  def __str__(self):
    return self.message


#-------------------------------------------------------------------------------
# Imu Class
#-------------------------------------------------------------------------------

##
## \brief IMU class.
##
class Imu:

  #
  ## \brief Constructor.
  #
  def __init__(self):
    self.m_port         = None

  #
  ## \brief Open connection to IMU.
  ##
  ## \param device        Serial (symbolic) device name.
  ## \param baudrate      Serial baud rate.
  #
  def open(self, device, baudrate):
    #dev = os.path.join(os.path.dirname(device), os.readlink(device))
    try:
      devName = os.readlink(device)
      dirName = os.path.dirname(device)
      dev = os.path.join(dirName, devName)
    except OSError:
      dev = device
    try:
      self.m_port = serial.Serial(dev, baudrate=baudrate, timeout=0.5)
    except serial.SerialException as inst:
      raise ImuException(inst.message)
    self.m_port.flushInput()
    self.m_port.flushOutput()

  #
  ## \brief Close connection with IMU.
  #
  def close(self):
    self.m_port.close()

  #
  ## \brief Check if connecton is open.
  ##
  ## \return True or False
  #
  def isOpen(self):
    return self.m_port.isOpen()

  #
  ## \brief Send command.
  ##
  ## \param cmdId   Command id.
  ## \param cmdData Optional command data.
  #
  def sendCmd(self, cmdId, cmdData=""):
    lenData = len(cmdData)
    cmdBuf  = ""

    for b in MSP_CMD_PREAMBLE:
      cmdBuf += b

    cmdBuf += chr(lenData)
    chksum = lenData

    cmdBuf += chr(cmdId)
    chksum ^= cmdId

    for b in cmdData:
      cmdBuf += b
      chksum ^= b

    chksum &= 0xff
    cmdBuf += chr(chksum)

    self.m_port.write(cmdBuf)

  #
  ## \brief Receive response.
  ##
  ## \param cmdId   Command id associated with response.
  ## \param lenData Optional response data length.
  #
  def receiveRsp(self, cmdId, lenData=0):
    rspLen = MSP_RSP_HDR_LEN + lenData + 1
    rspBuf = self.m_port.read(rspLen)

    #print "rspBuf[%d] = '%s'" % (len(rspBuf), rspBuf.encode("hex"))

    msgLen = len(rspBuf)
    if msgLen < MSP_RSP_MIN_LEN:
      raise ImuException("Response too small: " \
          "Only %d bytes received." % (msgLen))

    fldSize = ord(rspBuf[MSP_POS_SIZE])
    if fldSize != lenData:
      raise ImuException("Response data length mismatch: " \
          "Received %d bytes, expected %d bytes." % (fldSize, lenData))

    fldCmdId = ord(rspBuf[MSP_POS_CMD_ID])
    if fldCmdId != cmdId:
      raise ImuException("Command Id mismatch: " \
          "Received %d, expected %d." % (fldCmdId, cmdId))

    chksum = lenData
    for b in rspBuf[MSP_POS_CMD_ID:-1]:
      chksum ^= ord(b)
    chksum &= 0xff

    fldChkSum = ord(rspBuf[-1])

    if chksum != fldChkSum:
      raise ImuException("Checksum mismatch: " \
          "Received 0x%02x, calculated 0x%02x." % (fldChkSum, chksum))

    if lenData > 0:
      return rspBuf[MSP_POS_DATA:-1]
    else:
      return ""

  #
  ## \brief Pack an unsigned 8-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packU8(self, val):
    return struct.pack('<B', val)

  #
  ## \brief Pack a signed 8-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packS8(self, val):
    return struct.pack('<b', val)

  #
  ## \brief Pack an unsigned 16-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packU16(self, val):
    return struct.pack('<H', val)

  #
  ## \brief Pack a signed 16-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packS16(self, val):
    return struct.pack('<h', val)

  #
  ## \brief Pack an unsigned 32-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packU32(self, val):
    return struct.pack('<I', val)

  #
  ## \brief Pack a signed 32-bit value.
  ##
  ## \param val   Value to pack.
  ##
  ## \return Packed string.
  #
  def packS32(self, val):
    return struct.pack('<i', val)

  #
  ## \brief Unpack an unsigned 8-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackU8(self, buf):
    return struct.unpack('<B', buf)[0]

  #
  ## \brief Unpack a signed 8-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackS8(self, buf):
    return struct.unpack('<b', buf)[0]

  #
  ## \brief Unpack an unsigned 16-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackU16(self, buf):
    return struct.unpack('<H', buf)[0]

  #
  ## \brief Unpack a signed 16-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackS16(self, buf):
    return struct.unpack('<h', buf)[0]

  #
  ## \brief Unpack an unsigned 32-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackU32(self, buf):
    return struct.unpack('<I', buf)[0]

  #
  ## \brief Unpack a signed 32-bit value from buffer.
  ##
  ## \param buf   String.
  ##
  ## \return Value.
  #
  def unpackS32(self, buf):
    return struct.unpack('<i', buf)[0]

  def readIdent(self):
    self.sendCmd(MSP_IDENT)

    rspData = self.receiveRsp(MSP_IDENT, 7)

    version     = self.unpackU8(rspData[0])
    multiType   = self.unpackU8(rspData[1])
    mspVersion  = self.unpackU8(rspData[2])
    caps        = self.unpackU32(rspData[3:])

    return {'version':version, 'multi_type':multiType,
            'msp_version':mspVersion, 'caps':caps}

  def readRawImu(self, accel_units='raw', gyro_units='raw', mag_units='raw'):
    accel = NUM_AXES * [0]   # raw accelerometer values
    gyro  = NUM_AXES * [0]   # raw gyroscope values
    mag   = NUM_AXES * [0]   # raw magnetometer values

    self.sendCmd(MSP_RAW_IMU)

    rspData = self.receiveRsp(MSP_RAW_IMU, 18)

    accel[X] = self.unpackS16(rspData[0:2])
    accel[Y] = self.unpackS16(rspData[2:4])
    accel[Z] = self.unpackS16(rspData[4:6])

    gyro[X] = self.unpackS16(rspData[6:8])
    gyro[Y] = self.unpackS16(rspData[8:10])
    gyro[Z] = self.unpackS16(rspData[10:12])

    mag[X] = self.unpackS16(rspData[12:14])
    mag[Y] = self.unpackS16(rspData[14:16])
    mag[Z] = self.unpackS16(rspData[16:18])

    if accel_units == UNITS_G:
      accel[X] *= MSP_ACCEL_RAW_TO_G
      accel[Y] *= MSP_ACCEL_RAW_TO_G
      accel[Z] *= MSP_ACCEL_RAW_TO_G
    elif accel_units in [UNITS_M_PER_S2, UNITS_SI]:
      accel[X] = accel[X] * MSP_ACCEL_RAW_TO_G * G
      accel[Y] = accel[Y] * MSP_ACCEL_RAW_TO_G * G
      accel[Z] = accel[Z] * MSP_ACCEL_RAW_TO_G * G
      
    if gyro_units == UNITS_DEG_PER_S:
      gyro[X] *= MSP_GYRO_RAW_TO_DEG_PER_S
      gyro[Y] *= MSP_GYRO_RAW_TO_DEG_PER_S
      gyro[Z] *= MSP_GYRO_RAW_TO_DEG_PER_S
    elif gyro_units in [UNITS_RAD_PER_S, UNITS_SI]:
      gyro[X] = gyro[X] * MSP_GYRO_RAW_TO_DEG_PER_S * DEG_TO_RAD
      gyro[Y] = gyro[Y] * MSP_GYRO_RAW_TO_DEG_PER_S * DEG_TO_RAD
      gyro[Z] = gyro[Z] * MSP_GYRO_RAW_TO_DEG_PER_S * DEG_TO_RAD
      
    return {'accel':accel, 'gyro':gyro, 'mag':mag}

  def readAttitude(self, units='raw'):
    self.sendCmd(MSP_ATTITUDE)
    #time.sleep(0.1)

    rspData = self.receiveRsp(MSP_ATTITUDE, 6)

    roll  = self.unpackS16(rspData[0:2])
    pitch = self.unpackS16(rspData[2:4])
    yaw   = self.unpackS16(rspData[4:6])

    if units == UNITS_DEG:
      roll  *= MSP_ATTITUDE_RAW_TO_DEG
      pitch *= MSP_ATTITUDE_RAW_TO_DEG
    elif units in [UNITS_RAD, UNITS_SI]:
      roll  = roll  * MSP_ATTITUDE_RAW_TO_DEG * DEG_TO_RAD
      pitch = pitch * MSP_ATTITUDE_RAW_TO_DEG * DEG_TO_RAD
      yaw   = yaw   * DEG_TO_RAD

    return (roll, pitch, yaw)


# ------------------------------------------------------------------------------
# UT main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  print "IMU Unit Test Example"
  print

  device    = "/dev/ttyUSB0"
  baud      = 115200

  print "Get raw IMU data:"

  imu = Imu()

  imu.open(device, baudrate=baud)

  ident = imu.readIdent()

  print "IMU Identity:"
  print "  Version:     %u" % (ident['version'])
  print "  Multi-Type:  %u" % (ident['multi_type'])
  print "  MSP Version: %u" % (ident['msp_version'])
  print "  Caps:        0x%04x" % (ident['caps'])

  while True:
    meas = imu.readRawImu()

    print "accel_raw(x,y,z) = %6d, %6d, %6d" % \
        (meas['accel'][X], meas['accel'][Y], meas['accel'][Z])
    print "gyro_raw(x,y,z)  = %6d, %6d, %6d" % \
        (meas['gyro'][X], meas['gyro'][Y], meas['gyro'][Z])

    meas = imu.readRawImu('g', 'deg/s')

    print "accel_g(x,y,z)   = %.3f, %.3f, %.3f" % \
        (meas['accel'][X], meas['accel'][Y], meas['accel'][Z])
    print "gyro_dps(x,y,z)  = %.3f, %.3f, %.3f" % \
        (meas['gyro'][X], meas['gyro'][Y], meas['gyro'][Z])

    rpy = imu.readAttitude('degrees')
    print "roll,pitch,yaw   = %.3f, %.3f, %.3f" % \
        (rpy[ROLL], rpy[PITCH], rpy[YAW])

    try:
      time.sleep(1.0)
    except KeyboardInterrupt:
      break
