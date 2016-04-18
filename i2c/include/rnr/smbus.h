////////////////////////////////////////////////////////////////////////////////
//
// Package:   I2C
//
// Library:   libi2c API
//
// File:      smbus.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-25 08:03:07 -0600 (Thu, 25 Mar 2010) $
 * $Rev: 309 $
 *
 * \brief System Management Bus (SMBus) over I<sup>2</sup>C communication
 * interface declarations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010  RoadNarrows LLC.
 * (http://www.roadnarrows.com) \n
 * All Rights Reserved
 *
 * <hr>
 * \par Original Source and Copyright:
 * See i2c-dev.h.
 *
 * <hr>
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _SMBBUS_H
#define _SMBBUS_H

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/errno.h>
#include <sys/ioctl.h>

#include "rnr/rnrconfig.h"
#include "rnr/i2c-dev.h"

C_DECLS_BEGIN

// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

extern int i2c_smbus_access(int fd, byte_t read_write, byte_t command, 
                      int size, i2c_smbus_data_t *data);

extern int i2c_smbus_write_quick(int fd, byte_t value);
  
extern int i2c_smbus_read_byte(int fd);

extern int i2c_smbus_write_byte(int fd, byte_t value);

extern int i2c_smbus_read_byte_data(int fd, byte_t command);

extern int i2c_smbus_write_byte_data(int fd, byte_t command, byte_t value);

extern int i2c_smbus_read_word_data(int fd, byte_t command);

extern int i2c_smbus_write_word_data(int fd, byte_t command, ushort_t value);

extern int i2c_smbus_process_call(int fd, byte_t command, ushort_t value);

extern int i2c_smbus_read_block_data(int fd, byte_t command, byte_t *values);

extern int i2c_smbus_write_block_data(int fd, byte_t command, byte_t length,
                                      const byte_t *values);

extern int i2c_smbus_read_i2c_block_data(int fd, byte_t command,
                                        byte_t *values);

extern int i2c_smbus_write_i2c_block_data(int fd, byte_t command, byte_t length,
                                  const byte_t *values);

extern int i2c_smbus_block_process_call(int fd, byte_t command, byte_t length,
                                        byte_t *values);

C_DECLS_END


#endif // _SMBBUS_H
