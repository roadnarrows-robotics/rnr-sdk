////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      checksum.h
//
/*! \file
 *
 * \brief Checksum algorithms.
 *
 * $LastChangedDate: $
 * $Rev: $
 *
 * \author Daniel Packard (daniel@roadnarrows.com) 
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 20010-2013.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _CHECKSUM_H
#define _CHECKSUM_H

#include <sys/types.h>

#include "rnr/rnrconfig.h"

#define CRC32_POLY_NORM   0x04c11db7    ///< 32-bit crc in normal (MSB) form

C_DECLS_BEGIN

/*!
 * \brief Computes the modular 8-bit checksum over buffer.
 *
 * The two's complitment is not taken.
 *
 * \param buf Buffer to checksum.
 * \param len Number of bytes in buf.
 *
 * \return 8-bit sum of bytes, discarding overflow.
 */
extern u8_t generate_checksum8(byte_t buf[], size_t len);

/*!
 * \brief Computes the modular 16-bit checksum over buffer.
 *
 * The two's complitment is not taken.
 *
 * \param buf Buffer to checksum.
 * \param len Number of bytes in buf.
 *
 * \return 16-bit sum of bytes, discarding overflow.
 */
extern u16_t generate_checksum16(byte_t buf[], size_t len);

/*!
 * \brief Computes the modular 32-bit checksum over buffer.
 *
 * The two's complitment is not taken.
 *
 * \param buf Buffer to checksum
 * \param len Number of bytes in buf.
 *
 * \return 32-bit sum of bytes, discarding overflow.
 */
extern u32_t generate_checksum32(byte_t buf[], size_t len);

/*!
 * 
 * \brief Computes the 32-bit cyclic redundance check over buffer.
 *
 * CRC polynomial: 0x04C11DB7 (normal representation).
 *
 * This CRC-32 is used by Ethernet, MPEG-2, gzip, gzip2, and others.
 *
 * \par This CRC can be specified as:
 * \termblock
 * \term width  \termdata 32 \endterm
 * \term poly   \termdata 0x04c11db7 \endterm
 * \term init   \termdata 0xffffffff \endterm
 * \term refin  \termdata false \endterm
 * \term refout \termdata false \endterm
 * \term xorout \termdata true \endterm
 * \endtermblock
 *
 * \param buf Buffer to checksum
 * \param len Number of bytes in buf.
 *
 * \return 32-bit CRC 
 */
extern u32_t generate_crc32(byte_t buf[], size_t len);

C_DECLS_END

#endif // _CHECKSUM_H
