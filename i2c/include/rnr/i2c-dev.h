////////////////////////////////////////////////////////////////////////////////
//
// Package:   I2C
//
// Library:   libi2c API
//
// File:      i2c-dev.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $
 * $Rev: 4278 $
 *
 * \brief I<sup>2</sup>C character device interface.
 *
 * This file has been modified from the original source (see below).
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
 *  Copyright (C) 1995-97 Simon G. Vogl\n
 *  Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
 *
 * \par Original Author:
 * Simon G. Vogl\n
 * Frodo Looijaard 
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

#ifndef _I2C_DEV_H
#define _I2C_DEV_H

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

//
// The 16 I2C reserved addresses. Addresses are 7-bits, which are found in the
// upper 7-bits of the address byte.
//
#define I2C_ADDR_GEN_CALL     0x00    ///< general call/start byte  0000000 x
#define I2C_ADDR_CBUS         0x01    ///< CBUS format (obsolete)   0000001 x
#define I2C_ADDR_DIFF_BUS     0x02    ///< different bus            0000010 x
#define I2C_ADDR_FUTURE0      0x03    ///< future use block 0       0000011 x
#define I2C_ADDR_HIGH_SPEED_0 0x04    ///< high-speed master        0000100 x
#define I2C_ADDR_HIGH_SPEED_1 0x05    ///< high-speed master        0000101 x
#define I2C_ADDR_HIGH_SPEED_2 0x06    ///< high-speed master        0000110 x
#define I2C_ADDR_HIGH_SPEED_3 0x07    ///< high-speed master        0000111 x
#define I2C_ADDR_TEN_BIT_0    0x78    ///< 10-bit addressing        1111000 x
#define I2C_ADDR_TEN_BIT_1    0x79    ///< 10-bit addressing        1111001 x
#define I2C_ADDR_TEN_BIT_2    0x7a    ///< 10-bit addressing        1111010 x
#define I2C_ADDR_TEN_BIT_3    0x7b    ///< 10-bit addressing        1111011 x
#define I2C_ADDR_FUTURE1_0    0x7c    ///< future use block 1       1111100 x
#define I2C_ADDR_FUTURE1_1    0x7d    ///< future use block 1       1111101 x
#define I2C_ADDR_FUTURE1_2    0x7e    ///< future use block 1       1111110 x
#define I2C_ADDR_FUTURE1_3    0x7f    ///< future use block 1       1111111 x

//
// The 112 available I2C user device addresses.
//
#define I2C_ADDR_DEV_LOW      (I2C_ADDR_HIGH_SPEED_3 + 1)
                                      ///< first available device address
#define I2C_ADDR_DEV_HIGH     (I2C_ADDR_TEN_BIT_0 - 1)
                                      ///< last available device address
#define I2C_MAX_ADDRS         (I2C_ADDR_DEV_HIGH - I2C_ADDR_DEV_LOW + 1)     
                                      ///< maximum number of I2C addresses
                                      
//
// I2C message flags.
//
#define I2C_M_NOFLAGS         0         ///< no/clear flags
#define I2C_M_TEN             0x10      ///< ten-bit chip address
#define I2C_M_RD              0x01      ///< read bit
#define I2C_M_NOSTART         0x4000    ///< from original source - TBD 
#define I2C_M_REV_DIR_ADDR    0x2000    ///< from original source - TBD
#define I2C_M_IGNORE_NAK      0x1000    ///< from original source - TBD
#define I2C_M_NO_RD_ACK       0x0800    ///< from original source - TBD

/*!
 * \brief I<sup>2</sup>C Message Stucture
 *
 * The i2c_msg_t is used for pure I<sup>2</sup>Cc transactions for the
 * /dev interface.
 */
typedef struct i2c_msg_struct
{
  ushort_t    addr;     ///< 7/10-bit slave address 0xxx xxxx
  ushort_t    flags;    ///< flags
  short       len;      ///< message length (bytes)
  char       *buf;      ///< pointer to message data
} i2c_msg_t;

//
// Adapter Functionality Bits 
//
#define I2C_FUNC_I2C                      0x00000001  ///< low-level I2C
#define I2C_FUNC_10BIT_ADDR               0x0000000   ///< 10-bit address
#define I2C_FUNC_PROTOCOL_MANGLING        0x00000004 
                                            ///< I2C_M_{REV_DIR_ADDR,NOSTART,..}
#define I2C_FUNC_SMBUS_HWPEC_CALC         0x00000008  ///< SMBus 2.0
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL    0x00008000  ///< SMBus 2.0
#define I2C_FUNC_SMBUS_QUICK              0x00010000  ///< quick operation
#define I2C_FUNC_SMBUS_READ_BYTE          0x00020000  ///< read immediate byte
#define I2C_FUNC_SMBUS_WRITE_BYTE         0x00040000  ///< write immedieate byte
#define I2C_FUNC_SMBUS_READ_BYTE_DATA     0x00080000  ///< read data byte
#define I2C_FUNC_SMBUS_WRITE_BYTE_DATA    0x00100000  ///< wrtie data byte
#define I2C_FUNC_SMBUS_READ_WORD_DATA     0x00200000  ///< read data word
#define I2C_FUNC_SMBUS_WRITE_WORD_DATA    0x00400000  ///< write data word
#define I2C_FUNC_SMBUS_PROC_CALL          0x00800000  ///< w/r process word
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA    0x01000000  ///< read data blcok
#define I2C_FUNC_SMBUS_WRITE_BLOCK_DATA   0x02000000  ///< write data blcok
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK     0x04000000  ///< I2C-like block xfer
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK    0x08000000  ///< w/ 1-byte reg. addr.
#define I2C_FUNC_SMBUS_READ_I2C_BLOCK_2   0x10000000  ///< I2C-like block xfer
#define I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2  0x20000000  ///< w/ 2-byte reg. addr.
#define I2C_FUNC_SMBUS_BYTE \
  (I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE)
    ///< smbus immediate byte support
#define I2C_FUNC_SMBUS_BYTE_DATA \
  (I2C_FUNC_SMBUS_READ_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_BYTE_DATA)
    ///< smbus data byte support
#define I2C_FUNC_SMBUS_WORD_DATA \
  (I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA)
    ///< smbus data word support
#define I2C_FUNC_SMBUS_BLOCK_DATA \
  (I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_WRITE_BLOCK_DATA)
    ///< smbus data block support
#define I2C_FUNC_SMBUS_I2C_BLOCK \
  (I2C_FUNC_SMBUS_READ_I2C_BLOCK | I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)
    ///< smbus i2c data block 1-byte address support
#define I2C_FUNC_SMBUS_I2C_BLOCK_2 \
  (I2C_FUNC_SMBUS_READ_I2C_BLOCK_2 | I2C_FUNC_SMBUS_WRITE_I2C_BLOCK_2)
    ///< smbus i2c data block 2-byte address support

// 
// Data for SMBus Messages 
//
#define I2C_SMBUS_BLOCK_MAX       32  ///< As specified in SMBus standard 
#define I2C_SMBUS_I2C_BLOCK_MAX   32  ///< Not specified - use same structure

/*!
 * \brief I<sup>2</sup>C SMBus Data Stucture
 */
typedef union i2c_smbus_data_union
{
  byte_t    byte;                           ///< data byte     
  ushort_t  word;                           ///< data short word
  byte_t    block[I2C_SMBUS_BLOCK_MAX + 2];
                    ///< block[0] is used for length and one more for PEC
} i2c_smbus_data_t;

//
// SMBus ioctl access read or write operation markers (direction).
//
#define I2C_SMBUS_READ    1     ///< read
#define I2C_SMBUS_WRITE   0     ///< write

//
// SMBus ioctl transaction types (size parameter in the above functions) 
//
// Note: these no longer correspond to the (arbitrary) PIIX4 internal codes!
//
#define I2C_SMBUS_QUICK             0   ///< quick SMBus ioctl operation
#define I2C_SMBUS_BYTE              1   ///< immediate r/w byte operation
#define I2C_SMBUS_BYTE_DATA         2   ///< data byte r/w operation
#define I2C_SMBUS_WORD_DATA         3   ///< data word r/w operation
#define I2C_SMBUS_PROC_CALL         4   ///< issue word process call
#define I2C_SMBUS_BLOCK_DATA        5   ///< data block r/w operation
#define I2C_SMBUS_I2C_BLOCK_DATA    6   ///< i2c format block r/w/ operation
#define I2C_SMBUS_BLOCK_PROC_CALL   7   ///< SMBus 2.0: issue block process call


//
// I2C ioctl commands
//
// Note:  Additional calls are defined in the algorithm and hw dependent layers.
//        These can be listed here, or see the corresponding header files.
//

#define I2C_NOCMD       0       ///< no command
#define I2C_RETRIES     0x0701
  ///< number of times a device address should be polled when not acknowledging
#define I2C_TIMEOUT     0x0702  ///< set timeout - call with an int
#define I2C_SLAVE       0x0703  ///< change 7/10-bit slave address
#define I2C_SLAVE_FORCE 0x0706 
                ///< force change 7/10-bit slave address even if already taken
#define I2C_TENBIT      0x0704  ///< 0 for 7 bit addrs, != 0 for 10 bit
#define I2C_FUNCS       0x0705  ///< get the adapter functionality
#define I2C_RDWR        0x0707  ///< combined R/W transfer (one stop only)
#define I2C_PEC         0x0708  ///< != 0 for SMBus PEC
#define I2C_SMBUS       0x0720  ///< SMBus-level access


// Note: 10-bit addresses are NOT supported!

/*!
 * \brief I<sup>2</sup>C SMBus IOCTL Call Structure
 */
typedef struct i2c_smbus_ioctl_data_struct
{
  byte_t              read_write; ///< operation direction
  byte_t              command;    ///< ioctl command
  int                 size;       ///< data size
  i2c_smbus_data_t   *data;       ///< data
} i2c_smbus_ioctl_data_t;

/*!
 * \brief I<sup>2</sup>C SMBus IOCTL Multi-Message Structure
 */
typedef struct i2c_rdwr_ioctl_data_struct
{
  i2c_msg_t  *msgs;   ///< pointers to i2c_msgs
  int         nmsgs;  ///< number of i2c_msgs
} i2c_rdwr_ioctl_data_t;

C_DECLS_END


#endif // _I2C_DEV_H
