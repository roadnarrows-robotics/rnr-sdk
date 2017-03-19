////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      i2ccore.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-28 14:19:12 -0700 (Thu, 28 Jan 2016) $
 * $Rev: 4278 $
 *
 * \brief I2C python C interface header.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _I2CCORE_H
#define _I2CCORE_H


#include "rnr/rnrconfig.h"

extern int i2ccore_open(const char *device); 

extern int i2ccore_close(int fd);

extern int i2ccore_read(int fd,
                        unsigned short cur_addr,
                        unsigned short addr,
                        byte_t buf[],
                        unsigned int len);

extern int i2ccore_write(int fd,
                         unsigned short cur_addr,
                         unsigned short addr,
                         byte_t buf[],
                         unsigned int len);

int i2ccore_transfer(int fd, unsigned short cur_addr, unsigned short addr,
                  byte_t wbuf[], unsigned int wcount,
                  byte_t rbuf[], unsigned int rcount);

int i2ccore_check(int fd, unsigned short cur_addr, unsigned short addr);

#endif // _I2CCORE_H
