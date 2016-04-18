/******************************************************************************
 *
 * Package:   i2c
 *
 * File:      i2ccore.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief Core I2C python swig interface definitions file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 *   (C) 2016.  RoadNarrows LLC.
 *   (http://www.roadnarrows.com)
 *   All Rights Reserved
 */

/*
 * @EulaBegin@
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 *
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 *
 ******************************************************************************/

%module i2ccore
%{
#include "rnr/rnrconfig.h"
%}

%begin
%{
/*! \file
 *  \brief Swig generated core wrapper c file.
 */
%}

/* 
 * Required RNR C types
 */
typedef unsigned char byte_t;
typedef unsigned short ushort_t;
typedef unsigned int uint_t;
typedef unsigned long ulong_t;
typedef int bool_t;

%include "carrays.i"
%include "cpointer.i"


%array_functions(byte_t, byteArray);
%pointer_functions(uint_t, uintp);

%inline
%{
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/i2c.h"

#include "i2ccore.h"

/*!
 * \brief Open a I2C bus device.
 *
 * \param device  Device name.
 *
 * \return
 * On success, returns open file descriptor.
 * On failure, returns -errno.
 */
int i2ccore_open(const char *device)
{
  i2c_t i2c;
  int   rc;

  rc = i2c_open(&i2c, device);

  return rc < 0? -errno: i2c.fd;
}

/*!
 * \brief Close an open I2C bus device.
 *
 * \return
 * On success, returns 0.
 * On failure, returns -errno.
 */
int i2ccore_close(int fd)
{
  i2c_t i2c;
  
  i2c.fd = fd;

  i2c_close(&i2c);

  return OK;
}

/*!
 * \brief Read data from an attached device connected to the open I2C bus.
 *
 * \param fd        File descriptor.
 * \param cur_addr  I2C address of last I/O operation.
 * \param addr      I2C device address to read.
 * \param [out] buf Output buffer.
 * \param count     Number of byte to read.
 *
 * \return
 * On success, returns number of bytes read.
 * On failure, returns -errno.
 */
int i2ccore_read(int fd, unsigned short cur_addr, unsigned short addr,
                 byte_t buf[], unsigned int count)
{
  i2c_t i2c;
  int   n;

  i2c.fd    = fd;
  i2c.addr  = cur_addr;

  n = i2c_read(&i2c, addr, buf, count);

  return n < 0? -errno: n;
}

/*!
 * \brief Write data to an attached device connected to the open I2C bus.
 *
 * \param fd        File descriptor.
 * \param cur_addr  I2C address of last I/O operation.
 * \param addr      I2C device address to read.
 * \param [in] buf  Input buffer.
 * \param count     Number of byte to write.
 *
 * \return
 * On success, returns number of bytes written.
 * On failure, returns -errno.
 */
int i2ccore_write(int fd, unsigned short cur_addr, unsigned short addr,
                  byte_t buf[], unsigned int count)
{
  i2c_t i2c;
  int   n;

  i2c.fd    = fd;
  i2c.addr  = cur_addr;

  n = i2c_write(&i2c, addr, buf, count);

  return n < 0? -errno: n;
}

/*!
 * \brief Transfer data to an attached device connected to the open I2C bus
 * reead back.
 *
 * \param fd          File descriptor.
 * \param cur_addr    I2C address of last I/O operation.
 * \param addr        I2C device address to read.
 * \param [in] wbuf   Input buffer.
 * \param wcount      Number of byte to write.
 * \param [out] rbuf  Output buffer.
 * \param rcount      Number of byte to read.
 *
 * \return
 * On success, returns 0.
 * On failure, returns -errno.
 */
int i2ccore_transfer(int fd, unsigned short cur_addr, unsigned short addr,
                  byte_t wbuf[], unsigned int wcount,
                  byte_t rbuf[], unsigned int rcount)
{
  i2c_t i2c;
  int   rc;

  i2c.fd    = fd;
  i2c.addr  = cur_addr;

  rc = i2c_transfer(&i2c, addr, wbuf, wcount, rbuf, rcount);

  return rc < 0? -errno: rc;
}

/*!
 * \brief Test for the existence of a device with the given address.
 *
 * \param fd          File descriptor.
 * \param cur_addr    I2C address of last I/O operation.
 * \param addr        I2C device address to read.
 *
 * \return If device is found, returns 1, else returns 0.
 */
int i2ccore_check(int fd, unsigned short cur_addr, unsigned short addr)
{
  i2c_t i2c;

  i2c.fd    = fd;
  i2c.addr  = cur_addr;

  return i2c_exists(&i2c, addr);
}

%}

/*
 * Higher-level python interface to the core C library.
 */
%pythoncode
%{

"""
RoadNarrows Robotics i2ccore Python Inline Extensions and Wrappers.
"""

## \file 
## \package rnr.i2ccore
##
## \brief RoadNarrows Robotics Swigged Core Python Interface Module.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

%}
