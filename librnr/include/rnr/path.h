////////////////////////////////////////////////////////////////////////////////
/*! \file
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
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/path.h}
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

#ifndef _RNR_PATH_H
#define _RNR_PATH_H

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


#endif // _RNR_PATH_H
