////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Simple character manipulations.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/char.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_CHAR_H
#define _RNR_CHAR_H

#include <stdio.h>

C_DECLS_BEGIN

/*!
 * \brief Convert hex nibble to ascii equivalent.
 * \param h Hex nibble [0,15].
 */
static inline char hexnibbletoa(int h)
{
  return (char)(h<10? '0'+h: 'a'+(h & 0x0f));
}

extern char *NewPrettyBuf(byte_t buf[], size_t len);

extern int PrettyPrintBuf(FILE *fp, byte_t buf[], size_t len);

C_DECLS_END

#endif // _RNR_CHAR_H
