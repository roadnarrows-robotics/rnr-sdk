////////////////////////////////////////////////////////////////////////////////
//
// Package:   I2C
//
// Library:   libi2c API
//
// File:      i2c.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $
 * $Rev: 4278 $
 *
 * \brief Low-level I<sup>2</sup>C communication level.
 *
 * This file has been modified from the original i2ccom.h source (see below).
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2007-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com) \n
 * All Rights Reserved
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

#ifndef _I2C_H
#define _I2C_H

#include "rnr/rnrconfig.h"
#include "rnr/i2c-dev.h"

C_DECLS_BEGIN

/*! 
 * \brief I<sup>2</sup>C Device Address Type
 */
typedef ushort_t i2c_addr_t; 

#define I2C_ADDR_NONE    ((i2c_addr_t)(-1))   ///< no I2C address selected

/*!
 * \brief I<sup>2</sup>C Bus Handle Type
 */
typedef struct i2c_struct
{
  int         fd;     ///< opened file descriptor of the I2C bus device 
  i2c_addr_t  addr;   ///< address of the currently selected attached I2C device
} i2c_t;


// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

/*! 
 * \brief Open the host I<sup>2</sup>C Bus device.
 *
 * \param [out] i2c Pointer to I<sup>2</sup>C Bus handle.\n
 *                  Note: This descriptor must be allocated by the caller.
 * \param device    A null-terminated string of the device name. 
 *                  If NULL, then the default '/dev/i2c/0' is used.
 * \return
 * Returns 0 on success. Else errno is set appropriately and -1 is returned.
 */   
extern int  i2c_open(i2c_t *i2c , const char *device);

/*! 
 * \brief Closes an I<sup>2</sup>C Bus. 
 * 
 * \param [in,out] i2c  Pointer to I<sup>2</sup>C Bus handle.
 */
extern void i2c_close(i2c_t *i2c);

/*! 
 * \brief Read from an I<sup>2</sup>C device.
 *
 * The i2c_read() primitive reads data from a device at the given address on
 * the given I<sup>2</sup>C Bus.
 *
 * \param [in,out] i2c  Pointer to I<sup>2</sup>C Bus handle.
 * \param addr          I<sup>2</sup>C device's 7/10-bit address.
 * \param [out] buf     Pointer to the buffer that will receive the data bytes.
 * \param len           Size of the buffer in bytes.
 *
 * \return
 * On success, returns \h_ge 0 number of bytes read.\n
 * Else errno is set appropriately and -1 is returned.
 */
extern int  i2c_read(i2c_t *i2c, i2c_addr_t addr, byte_t *buf , uint_t len);

/*! 
 * \brief Write to an I<sup>2</sup>C device.
 *
 * The i2c_write() primitive writes data to a device at the given address
 * on the given I<sup>2</sup>C Bus.
 * 
 * \param [in,out] i2c  Pointer to I<sup>2</sup>C Bus handle.
 * \param addr          I<sup>2</sup>C device's 7/10-bit address.
 * \param buf           Pointer to the buffer that will be written.
 * \param len           Number of bytes to write.
 *
 * \return
 * On success, returns \h_ge 0 number of bytes written.\n
 * Else errno is set appropriately and -1 is returned.
 */
extern int  i2c_write(i2c_t *i2c, i2c_addr_t addr, const byte_t *buf,
                      uint_t len);

/*! 
 * \brief Perform a transfer with an I<sup>2</sup>C device.
 *
 * The i2c_transfer() primitive writes data to and then reads data from a
 * device at the given address on the given I<sup>2</sup>C Bus. It is optimize
 * to reduce start/stop sequences.
 *
 * \note Not all devices support the optimize tranfer.
 * Use i2c_write() / i2c_read() as an alternate is these cases.
 *
 * 
 * \param [in] i2c        Pointer to I<sup>2</sup>C Bus handle.
 * \param addr            I<sup>2</sup>C device's 7/10-bit address.
 * \param write_buf       Pointer to the buffer that contains the data 
 *                        to be written. 
 * \param write_len       Number of bytes to write. The data are in 
 *                        the write buffer.  
 * \param [out] read_buf  Pointer to the buffer that will receive the 
 *                        data. 
 * \param read_len        Size of the read buffer in bytes.
 *
 * \return
 * Returns \h_ge 0 on success.
 * Else errno is set appropriately and -1 is returned.
 */
extern int  i2c_transfer(i2c_t *i2c, i2c_addr_t addr, const byte_t *write_buf, 
			                  uint_t write_len, byte_t *read_buf, uint_t read_len);

/*! 
 * \brief Test the existance of a device at the given address on the 
 * given I<sup>2</sup>C Bus.
 * 
 * \param [in]  i2c  Pointer to I<sup>2</sup>C Bus handle.
 * \param addr       I<sup>2</sup>C device's 7/10-bit address.
 * 
 * \return
 * Returns 1 if the device is present. Else returns 0.
 */
extern int  i2c_exists(i2c_t *i2c, i2c_addr_t addr);

/*! 
 * \brief Scans the given I<sup>2</sup>C Bus to find all connected devices.
 * 
 * For each device found, the provided callback function is called.
 *
 * \param [in] i2c  Pointer to I<sup>2</sup>C Bus handle.
 * \param callback  A callback function called when a device is found.
 *                  If this callback function returns a value \<0 the 
 *                  bus scan stops immedialety and the value is 
 *                  used as return value by i2c_scan().
 * \param context   A user defined value or a pointer passed to the callback 
 *                  function.
 *
 * \return
 * Returns the number of deviced found on success. Else errno is set
 * appropriately and -1 is returned.
 */
extern int  i2c_scan(i2c_t *i2c,
                    int (*callback)(i2c_t *i2c, i2c_addr_t addr, void *context),
		                void *context);

C_DECLS_END


#endif // _I2C_H
