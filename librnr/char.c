////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      char.h
//
/*! \file
 *
 * \brief Simple character manipulations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/char.h"

/*!
 * 
 * \brief Allocates a string buffer and copies a 'prettified' readable 
 * version of the contents of the given (binary) buffer.
 *
 * \param buf Buffer to prettify
 * \param len Number of bytes in buf.
 *
 * \return
 *  Allocated, null-terminated, prettified string. Use delete() to deallocate.
 */
char *NewPrettyBuf(byte_t buf[], size_t len)
{
  char  *sPretty = NEWSTR(len*4);
  char  *s;
  int   i, c;

  for(i=0, s=sPretty; i<len; ++i)
  {
    c = (int)buf[i];
    if( c == ' ' )
    {
      *s++ = ' ';
    }
    else if( isspace(c) )
    {
      *s++ = '\\';
      switch(c)
      {
        case '\f':
          *s++ = 'f';
          break;
        case '\n':
          *s++ = 'n';
          break;
        case '\r':
          *s++ = 'r';
          break;
        case '\t':
          *s++ = 't';
          break;
        case '\v':
          *s++ = 'v';
          break;
        default:
          *s++ = hexnibbletoa( (c>>4) & 0x0f );
          *s++ = hexnibbletoa( c & 0x0f );
          break;
      }
    }
    else if( isprint(c) )
    {
      *s++ = (char)c;
    }
    else
    {
      *s++ = '\\';
      *s++ = 'x';
      *s++ = hexnibbletoa( (c>>4) & 0x0f );
      *s++ = hexnibbletoa( c & 0x0f );
    }
  }
  *s = 0;
  return sPretty;
}

/*!
 * 
 * \brief Print out 'prettified' readable version of the contents of the 
 * given (binary) buffer.
 *
 * \param fp  Output file pointer.
 * \param buf Buffer to prettify.
 * \param len Number of bytes in buf.
 *
 * \return
 *  Number of characters printed.
 */
int PrettyPrintBuf(FILE *fp, byte_t buf[], size_t len)
{
  char   *sPretty;
  int     n;

  sPretty = NewPrettyBuf(buf, len);

  n = fprintf(fp, "%s", sPretty);

  delete(sPretty);

  return n;
}
