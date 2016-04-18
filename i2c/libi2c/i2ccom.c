////////////////////////////////////////////////////////////////////////////////
//
// Package:   I2C
//
// Library:   libi2c
//
// File:      i2ccom.c
//
/*! \file
 *
 * $LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $
 * $Rev: 4278 $
 *
 * \brief Low-level I<sup>2</sup>C communication level implementation.
 *
 * This file has been modified from the original i2ccom.c source (see below).
 *
 * \todo Add ASCII line oriented input/output with cr-lf translation.
 * i2c_readline(), i2c_writeline().
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2007-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com) \n
 * All Rights Reserved
 *
 * <hr>
 * \par Original Source and Copyright:
 *  Copyright (C) 2004 K-TEAM SA
 *
 * \author   Yves Piguet (Calerga Sarl)
 * \author   Pierre Bureau (K-Team SA)
 * \author   Cedric Gaudin (K-Team SA)
 *
 * \par Original Header:
 * See "Original Source Header EULA" in source file.
 *
 * <hr>
 */
/*
 * @EulaBegin@
 * 
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
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/ioctl.h>

#include "rnr/rnrconfig.h"
#include "rnr/i2c-dev.h"
#include "rnr/i2c.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*! 
 * \brief Selects the I<sup>2</sup>C device on a given I<sup>2</sup>C Bus.
 *
 * \param [in,out] i2c  Pointer to I<sup>2</sup>C Bus handle.
 * \param addr          I<sup>2</sup>C device's 7/10-bit address.
 *
 * \return
 * Returns 0 on success. Else errno is set appropriately and -1 is returned.
 */
static int i2c_select_device(i2c_t *i2c, i2c_addr_t addr)
{
  int rc;

  // change device address only if necessary
  if( i2c->addr != addr )
  {
    if( (rc = ioctl(i2c->fd, I2C_SLAVE, addr)) < 0 )
    {
      return rc;
    }
    i2c->addr = addr;
  }
  return 0;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

int i2c_open(i2c_t *i2c, char const *device)
{
  // no device selected
  i2c->addr = (ushort_t)I2C_ADDR_NONE;      
    
  // set hte default I2C bus device
  if( device == NULL )
  {
    device = "/dev/i2c/0";
  }

  if( (i2c->fd = open(device, O_RDWR)) < 0)
  {
    return i2c->fd;
  }
  
  return 0;
}

void i2c_close(i2c_t *i2c)
{
  if( i2c->fd >= 0 )
  { 
    close(i2c->fd);
    i2c->fd   = -1;
    i2c->addr  = (ushort_t)I2C_ADDR_NONE;      
  }
}

int i2c_read(i2c_t *i2c, i2c_addr_t addr, byte_t *buf, uint_t len)
{
  int rc;

  if( (rc = i2c_select_device(i2c , addr)) != -1 )
  {
    rc = (int)read(i2c->fd, buf, (size_t)len);
  }

  return rc;
}

int i2c_write(i2c_t *i2c, i2c_addr_t addr, const byte_t *buf, uint_t len)
{
  int rc;

  if( (rc = i2c_select_device(i2c , addr)) != -1 )
  {
    rc = (int)write(i2c->fd , buf, (size_t)len);
  }
  
  return rc;
}

int i2c_transfer(i2c_t *i2c , i2c_addr_t addr, 
                const byte_t *write_buf , uint_t write_len,
                byte_t *read_buf , uint_t read_len)
{
  i2c_msg_t             msgs[2];
  i2c_rdwr_ioctl_data_t msgset;
  int                   rc;  
  
  // write message
  msgs[0].addr  = addr;
  msgs[0].flags = I2C_M_NOFLAGS;
  msgs[0].buf   = (char *)write_buf;
  msgs[0].len   = (short)write_len;
      
  // read message
  msgs[1].addr  = addr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].buf   = (char *)read_buf;
  msgs[1].len   = (short)read_len;
    
  msgset.msgs   = msgs;
  msgset.nmsgs  = 2;
  
  rc = ioctl(i2c->fd , I2C_RDWR, &msgset); 
 
  return rc;
}

int i2c_exists(i2c_t *i2c, i2c_addr_t addr)
{
  i2c_msg_t             msg;
  i2c_rdwr_ioctl_data_t msgset;
  int                   rc;
 
  msg.addr      = addr;
  msg.flags     = I2C_M_NOFLAGS;
  msg.buf       = NULL;
  msg.len       = 0;
    
  msgset.msgs   = &msg;
  msgset.nmsgs  = 1;

  rc = ioctl(i2c->fd, I2C_RDWR, &msgset);
  
  // answer from an I2C device
  return rc == 1? 1: 0;
}
     
int i2c_scan(i2c_t *i2c,
            int (*callback)(i2c_t *i2c, i2c_addr_t addr, void *context), 
            void * context)
{
  i2c_addr_t  addr;
  int         nFound = 0;
  int         rc;
 
  //
  // Scan the whole I<sup>2</sup>C Bus range.
  //
  for(addr=I2C_ADDR_DEV_LOW; addr<=I2C_ADDR_DEV_HIGH; ++addr)
  {
    if( i2c_exists(i2c, addr) )
    {
      ++nFound;

      if( callback != NULL )
      {
        if( (rc = callback(i2c, addr, context)) < 0 )
        {
          return rc;
        }
      }
    }
  }

  return nFound;
}
