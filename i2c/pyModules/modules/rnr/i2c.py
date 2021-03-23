#
# Package:  RoadNarrows Robotics I2C
#
# Module:   rnr.i2c
#
# Link:     https://roadnarrows.com
#
# File:     i2c.py
#

## \file
##
## $LastChangedDate: 2016-02-04 09:24:32 -0700 (Thu, 04 Feb 2016) $  
## $Rev: 4304 $ 
## 
## \brief RoadNarrows Robotics I2C module.
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

"""
RoadNarrows Robotics I2C.
"""

import os

import rnr.i2ccore as i2ccore
import rnr.basetypes as basetypes


# ------------------------------------------------------------------------------
# Fixed Data
# ------------------------------------------------------------------------------

#
# Seven bit addresses
#
I2C_ADDR_LOW    = 0x08      ## lowest address
I2C_ADDR_HIGH   = 0x77      ## higest address
I2C_ADDR_NONE   = 0xffff    ## no address


# ------------------------------------------------------------------------------
# Class I2CException
# ------------------------------------------------------------------------------

##
## \brief I2C exception class.
##
class I2CException(Exception):
  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.message = msg

  def __repr__(self):
    return "I2CException(%s)" % (repr(self.message))

  def __str__(self):
    return self.message


#-------------------------------------------------------------------------------
# Class i2c
#-------------------------------------------------------------------------------

##
## \brief I2C class.
##
class i2c:
  #
  ## \brief Class constructor.
  #
  def __init__(self):
    self.m_device  = ""
    self.m_fd      = -1
    self.m_addr    = I2C_ADDR_NONE

  #
  ## \brief Destructor.
  #
  def __del__(self):
    if self.m_fd >= 0:
      self.close()

  #
  ## \brief Open a I2C bus device.
  ##
  ## Raises I2CException on error.
  ##
  ## \param device  I2C device name (e.g. /dev/i2c-3).
  #
  def open(self, device):
    fd = i2ccore.i2ccore_open(device)
    if fd < 0:
      e = -fd
      raise I2CException("%s: open(): %s: (errno=%d)." % \
          (device, os.strerror(e), e))
    self.m_device = device
    self.m_fd = fd;

  #
  ## \brief Close an open I2C bus device.
  ##
  ## Raises I2CException on error.
  #
  def close(self):
    if self.m_fd >= 0:
      rc = i2ccore.i2ccore_close(self.m_fd)
      if rc < 0:
        e = -rc
        raise I2CException("%s: close(): %s: (errno=%d)." % \
            (self.m_device, os.strerror(e), e))
    self.m_device  = ""
    self.m_fd      = -1
    self.m_addr    = I2C_ADDR_NONE

  #
  ## \brief Test if I2C bus is open.
  ##
  ## \return True or False.
  #
  def is_open(self):
    if self.m_fd < 0:
      return False
    else:
      return True

  #
  ## \brief Read data from an attached device connected to the open I2C bus.
  ##
  ## Raises I2CException on error.
  ##
  ## \param addr    Attached device address.
  ## \param count   Number of bytes to read.
  ##
  ## \return List of bytes.
  #
  def read(self, addr, count):
    try:
      rbuf = basetypes.ByteBuf(count)
    except AssertionError as inst:
      raise I2CException("%s: read(0x%02x, ...): %s" % \
          (self.m_device, addr, inst.message))
    n = i2ccore.i2ccore_read(self.m_fd, self.m_addr, addr,
                  rbuf.getSwigObj(), basetypes.sizeof(rbuf))
    if n < 0:
      e = -n
      raise I2CException("%s: read(0x%02x, ...): %s: (errno=%d)." % \
          (self.m_device, addr, os.strerror(e), e))
    self.m_addr = addr
    try:
      rbuf.copyFromSwigObj(n)
    except AssertionError as inst:
      raise I2CException("%s: read(0x%02x, ...): %s" % \
          (self.m_device, addr, inst.message))
    return rbuf.buf

  #
  ## \brief Write data to an attached device connected to the open I2C bus.
  ##
  ## Raises I2CException on error.
  ##
  ## \param addr    Attached device address.
  ## \param buf     Buffer to write.
  #
  def write(self, addr, buf):
    try:
      wbuf = basetypes.ByteBuf.Clone(buf)
      wbuf.copyToSwigObj(len(wbuf))
    except AssertionError as inst:
      raise I2CException("%s: write(0x%02x, ...): %s" % \
          (self.m_device, addr, inst.message))
    n = i2ccore.i2ccore_write(self.m_fd, self.m_addr, addr,
                              wbuf.getSwigObj(), len(wbuf))
    if n < 0:
      e = -n
      raise I2CException("%s: write(0x%02x, ...): %s: (errno=%d)." % \
          (self.m_device, addr, os.strerror(e), e))
    self.m_addr = addr

  #
  ## \brief Transfer data to an attached device connected to the open I2C bus
  ## and receive response.
  ##
  ## \note Not all devices support optimize transfer. Use write_read() as an 
  ## alternate is these cases.
  ##
  ## Raises I2CException on error.
  ##
  ## \param addr    Attached device address.
  ## \param buf     Buffer to transfer.
  ## \param count   Number of bytes to read back.
  #
  def transfer(self, addr, buf, count):
    try:
      wbuf = basetypes.ByteBuf.Clone(buf)
      wbuf.copyToSwigObj(len(wbuf))
      rbuf = basetypes.ByteBuf(count)
    except AssertionError as inst:
      raise I2CException("%s: transfer(0x%02x, ...): %s" % \
          (self.m_device, addr, inst.message))
    rc = i2ccore.i2ccore_transfer(self.m_fd, self.m_addr, addr,
                              wbuf.getSwigObj(), len(wbuf),
                              rbuf.getSwigObj(), basetypes.sizeof(rbuf))
    if rc < 0:
      e = -rc
      raise I2CException("%s: transfer(0x%02x, ...): %s: (errno=%d)." % \
          (self.m_device, addr, os.strerror(e), e))
    self.m_addr = addr
    try:
      rbuf.copyFromSwigObj(n)
    except AssertionError as inst:
      raise I2CException("%s: read(0x%02x, ...): %s" % \
          (self.m_device, addr, inst.message))
    return rbuf.buf

  #
  ## \brief Write/read data to/from an attached device connected to the open
  ## I2C bus.
  ##
  ## Raises I2CException on error.
  ##
  ## \param addr    Attached device address.
  ## \param buf     Buffer to transfer.
  ## \param count   Number of bytes to read back.
  ##
  ## \return List of bytes.
  #
  def write_read(self, addr, buf, count):
    self.write(addr, buf)
    return self.read(addr, count)

  #
  ## \brief Test for the existence of a device with the given address.
  ##
  ## \param addr    Device address to check.
  ##
  ## \return True or False.
  #
  def check(self, addr):
    rc = i2ccore.i2ccore_check(self.m_fd, self.m_addr, addr)
    self.m_addr = addr
    if rc == 1:
      return True
    else:
      return False

  #
  ## \brief Scan I2C bus for all attached devices.
  ##
  ## \return List of addresses of found devices.
  #
  def scan(self):
    found = [] 
    for addr in range(I2C_ADDR_LOW, I2C_ADDR_HIGH+1):
      if self.check(addr):
        found += [addr]
    return found
