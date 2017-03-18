////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_hash.c
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:30:34 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1577 $
 *
 * \brief Example of using the librnr URI utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/uri.h"

/*!
 * \brief Make a safe string.
 *
 * \param s String to safe.
 */
#define SAFESTR(s)  ((s) != NULL? s: "")

/*!
 * \brief Strip leading and trailing white space.
 *
 * An 0 is insterted to strip the trailing white space.
 *
 * \param [out] s   String to strip.
 *
 * \return Returns pointer to start of non-white space token.
 */
static char *strip(char *s)
{
  size_t  n;
  char   *t;

  while( *s && isspace((int)*s))
  {
    s++;
  }

  if( *s == '\0' )
  {
    return s;
  }

  n = strlen(s);

  t = s + n - 1;
  
  while( (t > s) && isspace((int)*t))
  {
    t--;
  }

  t++;

  *t = '\0';

  return s;
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  char    buf[256];
  char   *sCmd;
  Uri_T  *pUri;
  char   *sUri;

  printf("Uniform Resource Indentifier Example.\n");
  printf("  (enter 'quit' to quit programe)\n\n");

  // process user commands
  for(;;)
  {
    printf("enter uri> ");

    // read user input
    if( !fgets(buf, (int)sizeof(buf), stdin) )
    {
      break;
    }

    sCmd = strip(buf);

    // null command
    if( (*sCmd == '\0') || (*sCmd == '\n') )
    {
      continue;
    }

    else if( !strcmp(sCmd, "quit") )
    {
      break;
    }

    else
    {
      pUri = UriParseNew(sCmd);
      printf("UriParseNew() ->\n");
      printf("{\n");
      printf("  m_sScheme   = \"%s\"\n", SAFESTR(pUri->m_sScheme));
      printf("  m_sUserInfo = \"%s\"\n", SAFESTR(pUri->m_sUserInfo));
      printf("  m_sHostName = \"%s\"\n", SAFESTR(pUri->m_sHostName));
      printf("  m_nPortNum  = %d\n",     pUri->m_nPortNum);
      printf("  m_sPath     = \"%s\"\n", SAFESTR(pUri->m_sPath));
      printf("  m_sQuery    = \"%s\"\n", SAFESTR(pUri->m_sQuery));
      printf("}\n");

      sUri = UriStrNew(pUri);
      printf("UriStrNew() -> \"%s\"\n", SAFESTR(sUri));
      delete(sUri);

      UriDelete(pUri);
      printf("UriDelete() -> deleted\n");
    }
  }

  return 0;
}
