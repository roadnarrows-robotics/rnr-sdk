////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Checksum algorithms.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/checksum.h}
 *
 * \author Daniel Packard (daniel@roadnarrows.com) 
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2010-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_CHECKSUM_H
#define _RNR_CHECKSUM_H

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

#endif // _RNR_CHECKSUM_H
