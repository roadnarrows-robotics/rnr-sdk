////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_util.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief Dynamixel shell utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows LLC.
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _DYNASHELL_UTIL_H
#define _DYNASHELL_UTIL_H

#include <stdio.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaChain.h"

using namespace std;


// ----------------------------------------------------------------------------
// Macros and Types
// ----------------------------------------------------------------------------

/*!
 * \brief Allocate a new duplicated string convenience macro.
 *
 * \param s The null-terminated string to duplicate.
 *
 * \return Returns pointer to allocated string if s is not NULL and the
 * length of s \h_gt 0.\n Otherwise returns NULL.
 */
#define NEWSTR(s) newstr(s)

/*!
 * \brief Delete an allocated string convenience macro.
 *
 * The value s is set to NULL after deletion.
 *
 * \param s Pointer to the string to delete.
 */
#define DELSTR(s) do { if( s != NULL ) { delstr(s); s = NULL; } } while(0)

/*!
 * \brief Delete an allocated object convenience macro.
 *
 * The value p is set to NULL after deletion.
 *
 * \param p Pointer to the object to delete.
 */
#define DELOBJ(p) do { if( p != NULL ) { delete p; p = NULL; } } while(0)


// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

/*!
 * \brief Allocate a new duplicated string utility.
 *
 * \param s String to duplicate.
 *
 * \return Returns pointer to allocated string if s is not NULL and the
 * length of s \h_gt 0.\n Otherwise returns NULL.
 */
inline char *newstr(const char *s)
{
  char  *t;
  if( (s != NULL) && (*s != 0) )
  {
    t = new char[strlen(s)+1];
    strcpy(t, s);
  }
  else
  {
    t = NULL;
  }
  return t;
}

/*!
 * \brief A safer string delete utility.
 *
 * \param s String to delete.
 */
inline void delstr(const char *s)
{
  if( s != NULL )
  {
    delete[] (char *)s;
  }
}

#endif // _DYNASHELL_UTIL_H
