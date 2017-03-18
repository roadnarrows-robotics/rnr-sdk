////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      path.h
//
/*! \file
 *
 * $LastChangedDate: 2010-04-16 09:47:22 -0600 (Fri, 16 Apr 2010) $
 * $Rev: 327 $
 *
 * \brief General file name and file path utility declarations.
 *
 * Boths posix (the world) and Microsoft's file and path syntax are
 * supported.
 *
 * \par Supported Search Path Backus-Naur Form:
 * \verbatim
 * search_path ::=
 *    path
 *  | path path_sep search_path
 *
 * path ::=
 *    root rel_path
 *  | rel_path
 *
 * root :: =
 *      std_dir_sep
 *    | win_dir_sep
 *    | win_drive win_dir_sep
 *
 * rel_path ::=
 *      fname
 *    | fname dir_sep rel_path
 *
 * fname ::=
 *    '.'
 *  | '.' '.'
 *  | '~'
 *  | '~' OSLEGAL_USERNAME
 *  | '$' indentifier
 *  | '$' '{' indentifier '}'
 *  | OSLEGAL_FILENAME
 *
 * path_sep ::= std_path_sep | win_path_sep
 *
 * std_path_sep ::= ':'
 * win_path_sep ::= ';'
 *
 * dir_sep ::= std_dir_sep | win_dir_sep
 *
 * std_dir_sep ::= '/' 
 * win_dir_sep ::= '\\'
 *
 * win_drive ::= a-zA-Z ':'
 * \endverbatim
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
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

#ifndef _PATH_H
#define _PATH_H

#include <ctype.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * Search Path Iterator Structure
 */
typedef struct path_iter  PathIter_T;

/*!
 * \brief Check if the given path is an absolute path.
 *
 * \param sPath   Path name.
 *
 * \return Returnes true or false
 */
INLINE_IN_H bool_t PathIsAbsolute(const char *sPath)
{
  if( sPath == NULL )
  {
    return false;
  }
  else if( *sPath == DIR_SEP_CHAR )
  {
    return true;
  }
#if defined(__windows__)
  else if( isalpha(*sPath) && (*(sPath+1) == ':') &&
            (*(sPath+2) == DIR_SEP_CHAR) )
  {
    return true;
  }
#endif // __windwos__
  else
  {
    return false;
  }
}

extern char *NewExpandTilde(const char *sTildeExpr);

extern char *NewSearchPathExpanded(const char *sSearchPath);

extern char *NewSearchPathCanonicalized(const char *sSearchPath);

extern PathIter_T *SearchPathIterNew(const char *sSearchPath);

extern void SearchPathIterDelete(PathIter_T *pIter);

extern char *SearchPathIterFirst(PathIter_T *pIter, const char *sSearchFile);

extern char *SearchPathIterNext(PathIter_T *pIter);

extern char *NewNormPath(const char *sPath);

extern char *NewRealPath(const char *sPath);

extern char *NewJoinedPath(const char *sPath1, const char *sPath2);

C_DECLS_END


#endif // _PATH_H
