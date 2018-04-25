////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      path.c
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:30:34 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1577 $
 *
 * \brief General file name and file path utilities.
 *
 * Boths posix (the world) and Microsoft's file and path syntax are
 * supported.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
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

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <pwd.h>

#include "rnr/rnrconfig.h"
#include "rnr/dliststr.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/path.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * Legal identifier character
 */
#define is1stidentifier(c) (isapha((int)c) || ((c) == '_'))  ///< first id char
#define isidentifier(c)    (isalnum((int)c) || ((c) == '_')) ///< id char

/*!
 * Search Pather Iterator Structure
 */
struct path_iter
{
  DListStr_T      *m_pSearchPaths;    ///< dlist of search paths
  char            *m_sFileCanonical;  ///< search canonical file 
  size_t           m_nFileLen;        ///< length of canonical file
  char            *m_sFilePath;       ///< concatenated file path (output)
  DListStrIter_T   m_iterPaths;       ///< dlist iterator over paths
  bool_t           m_bIsEos;          ///< is [not] end of search flag
};

/*!
 * \brief Build a dlist of canonical search paths
 *
 * \param sSearchPath   Search path of form: path [path_sep path...]
 *
 * \return Returns new dlist of path components.
 */
static DListStr_T *NewSearchPathDList(const char *sSearchPath)
{
  DListStr_T  *pSearchPaths;  // list of search paths
  char        *sExpPath;      // expanded path
  char        *sPath;         // search path component
  char        *sSave = NULL;  // strtok() save pointer

  // new dlist
  if( (pSearchPaths = DListStrNewDft()) == NULL )
  {
    LOGERROR("DListStrNew() failed");
    return NULL;
  }

  // no search paths
  if( (sSearchPath == NULL) || (sSearchPath[0] == 0) )
  {
    return pSearchPaths;
  }

  // canonicalize expanded search path
  if( (sExpPath = NewSearchPathCanonicalized(sSearchPath)) == NULL )
  {
    return pSearchPaths;
  }

  // parse search path and append compenents to dlist
  for(sPath=strtok_r(sExpPath, PATH_SEP_STR, &sSave); 
      sPath!=NULL; 
      sPath=strtok_r(NULL, PATH_SEP_STR, &sSave))
  {
    if( DListStrAppend(pSearchPaths, sPath) == NULL )
    {
      LOGERROR("DListStrAppend() failed");
      break;
    }
  }

  delete(sExpPath);

  LOGDIAG4("%s(%s)", LOGFUNCNAME, sSearchPath);
  if( LOGABLE(LOG_LEVEL_DIAG4) )
  {
    DListStrPrint(pSearchPaths, DListStrDataPrint, LOG_GET_LOGFP());
  }

  return pSearchPaths;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Expand tilde expression.
 *
 * \param sTildeExpr    '~' [user]
 *
 * \return 
 *  Return allocated expanded tilde expression path on success.\n
 *  If the expression has invalid syntax or the user does not exist, then
 *  NULL is returned.
 */
char *NewExpandTilde(const char *sTildeExpr)
{
  char          *sUser;
  struct passwd *pPassEntry;
  uid_t          uid;
  char          *sExpanded = NULL;

  // expression must start with a tilde
  if( (sTildeExpr == NULL) || (*sTildeExpr != '~') )
  {
    return NULL;
  }

  // user name
  sUser = (char *)sTildeExpr + 1;

  // expand this user's home
  if( (*sUser == 0) || (*sUser == DIR_SEP_CHAR) || (*sUser == PATH_SEP_CHAR) )
  {
    if( (sExpanded = getenv("HOME")) == NULL )
    {
      uid = getuid();
      if( (pPassEntry = getpwuid(uid)) != NULL )
      {
        sExpanded = pPassEntry->pw_dir;
      }
    }
  }
  
  // expand other user's home
  else
  {
    if( (pPassEntry = getpwnam(sUser)) != NULL )
    {
      sExpanded = pPassEntry->pw_dir;
    }
  }

  if( sExpanded != NULL )
  {
    return new_strdup(sExpanded);
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Expands search path.
 *
 *  Expands any environment variables and tilde expressions in search path
 *  while building new search path.
 *
 * The path does not have to exist.
 *
 *  Variable are specified as:\n
 *    $\<identifier\> or ${\<identifier\>} where \<identifier\> is a legal shell
 *    variable name.
 *
 *  Tilde expressions are specified as:\n
 *    ~ or ~user
 *
 * \param sSearchPath   Search path of form: path [path_sep path...]
 *
 * \return 
 *  Return allocated expanded search path on success, NULL if search path is
 *  is not specified. If the search path is too long, a truncated version
 *  will be returned.
 */
char *NewSearchPathExpanded(const char *sSearchPath)
{
  char  *sExpPath;                    // expanded path
  const char  *s, *u;                 // working source char pointers
  char  *t;                           // working target char pointers
  char  *sName, *sVal;                // environment name=value strings
  size_t nExpLen = 0;                 // current length of expanded path
                                      //  (sans null character)
  size_t nMaxLen = MAX_SEARCH_PATH-1; // maximum length
  size_t nNameLen;                    // name length

  if( (sSearchPath == NULL) || (*sSearchPath == 0) )
  {
    return NULL;
  }

  sExpPath = NEWSTR(nMaxLen);

  for(s=sSearchPath, t=sExpPath; *s && nExpLen<nMaxLen; )
  {
    switch(*s)
    {
      case '$':
        ++s;
        if( *s == '{' )
        {
          ++s;
        }
        for(u=s; isidentifier(*s); ++s);
        sName = NULL;
        nNameLen = (size_t)(s - u);
        if( nNameLen > 0 )
        {
          sName = new_strndup(u, nNameLen);
          if( (sName != NULL) && ((sVal = getenv(sName)) != NULL) )
          {
            for(; *sVal && nExpLen<nMaxLen; ++nExpLen)
            {
              *t++ = *sVal++;
            }
          }
          delete(sName);
        }
        if( *s == '}' )
        {
          ++s;
        }
        break;
      case '~':
        for(u=s; (*s != DIR_SEP_CHAR) && (*s != 0); ++s);
        sName = NULL;
        nNameLen = (size_t)(s - u);
        if( nNameLen > 0 )
        {
          sName = new_strndup(u, nNameLen);
          if( (sName != NULL) && ((sVal = NewExpandTilde(sName)) != NULL) )
          {
            for(u=sVal; *u && nExpLen<nMaxLen; ++nExpLen)
            {
              *t++ = *u++;
            }
            delete(sVal);
          }
          delete(sName);
        }
        break;
      default:
        *t++ = *s++;
        ++nExpLen;
        break;
    }
  }
  *t = 0;

  if( nExpLen >= nMaxLen )
  {
    LOGERROR("Expanded search path truncated");
  }

  LOGDIAG4("'%s'=%s('%s')", sExpPath, LOGFUNCNAME, sSearchPath);

  return sExpPath;
}

/*!
 * \brief Expands and canonicalizes a search path.
 *
 * The path does not have to exist.
 *
 * A canonical search path has: \n
 * \li any beginning or ending path separators stipped
 * \li any contiguous sequence of path separators replaced with a single 
 *     separator
 * \li any contiguous sequence of directory separators replaced with a single
 *     separator
 * \li any terminating directory separator stripped from each path component
 *
 * \param sSearchPath   Search path of form: path [path_sep path...]
 *
 * \return 
 *  Return allocated canonical search path on success, NULL if search path is
 *  is not specified. If the search path is too long, a truncated version
 *  will be returned.
 */
char *NewSearchPathCanonicalized(const char *sSearchPath)
{
  char   *sExpPath;     // expanded search path
  char   *sPath;        // expanded path
  char   *sNorm;        // normalized path
  char   *sSep;         // path separator
  char   *sNew;         // canonical search path
  char   *sSave = NULL; // strtok() save pointer
  char   *s;            // working string
  size_t  k, m, n, len; // working sizes

  // expand any environment variables and '~''s listed in search path
  if( (sExpPath = NewSearchPathExpanded(sSearchPath)) == NULL )
  {
    return NULL;
  }

  len   = strlen(sExpPath);
  sNew  = NEWSTR(len);
  n     = 0;
  sSep  = "";
  k     = 0;

  for(sPath=strtok_r(sExpPath, PATH_SEP_STR, &sSave); 
      sPath!=NULL; 
      sPath=strtok_r(NULL, PATH_SEP_STR, &sSave))
  {
    sNorm = NewNormPath(sPath);
    m     = strlen(sNorm);

    // grow new search path
    if( n+k+m >= len )
    {
      len = (n + k + m) * 2;
      s = NEWSTR(len);
      strcpy(s, sNew);
      delete(sNew);
      sNew = s;
    }

    sprintf(sNew+n, "%s%s", sSep, sNorm);
    n += k + m;

    if( *sSep == 0 )
    {
      sSep  = PATH_SEP_STR;
      k     = strlen(sSep);
    }
  }

  delete(sExpPath);

  return sNew;
}

/*!
 * \brief Allocates and initilized a new search path iterator.
 *
 * This iterator can be reused multiple times with different search files by
 * calling SearchPathIterFirst(). SearchPathIterDelete() must be called when
 * finished with the iterator to clean up any allocated memory.
 *
 * \param sSearchPath   Search path of form: path [path_sep path...]
 *
 * \return
 *  Returns pointer to search path iterator on success, NULL on error
 */
PathIter_T *SearchPathIterNew(const char *sSearchPath)
{
  PathIter_T  *pIter = NEW(PathIter_T);

  if( (pIter->m_pSearchPaths = NewSearchPathDList(sSearchPath)) == NULL )
  {
    LOGERROR("NewSearchPathDlist() failed");
    SearchPathIterDelete(pIter);
    return NULL;
  }

  pIter->m_sFileCanonical = NULL;
  pIter->m_nFileLen       = 0;
  pIter->m_sFilePath      = NULL;
  pIter->m_bIsEos         = false;

  return pIter;
}

/*!
 * \brief Deletes search path iterator
 *
 * \param pIter Search path iterator (invalid after return from this call).
 */
void SearchPathIterDelete(PathIter_T *pIter)
{
  if( pIter == NULL )
  {
    return;
  }

  if( pIter->m_pSearchPaths != NULL )
  {
    DListStrDelete(pIter->m_pSearchPaths);
  }

  delete(pIter->m_sFileCanonical);
  delete(pIter->m_sFilePath);
  
  delete(pIter);
}

/*!
 * \brief Get the first concatenated file path.
 *
 * Initialize search path iterator from given search file and returns first
 * concatenated file path.
 *
 * \param pIter         Path iterator.
 * \param sSearchFile   Search path.
 *
 * \return
 *  Returns first file path on success, NULL if at end of search or on error.
 */
char *SearchPathIterFirst(PathIter_T *pIter, const char *sSearchFile)
{
  const char      *sPath;   // search path component
  size_t           nLen;    // string length

  // logistics to [re]set iterator
  delete(pIter->m_sFileCanonical);  // clear any old canonical serach file 
  pIter->m_sFileCanonical = NULL;
  delete(pIter->m_sFilePath);       // clear any old constructed file path
  pIter->m_sFilePath      = NULL;
  pIter->m_nFileLen       = 0;      // no file, no length
  pIter->m_bIsEos         = true;   // assume end-of-search condition until
                                    //   proven otherwise

  // no serach file specified
  if( (sSearchFile == NULL) || (*sSearchFile == 0) )
  {
    LOGERROR("%s(): no file specified", LOGFUNCNAME);
    return NULL;
  }

  // cannot canonicalize search file
  else if( (pIter->m_sFileCanonical = NewSearchPathCanonicalized(sSearchFile)) 
                                        == NULL )
  {
    LOGERROR("NewSearchPathCanonicalized() failed");
    return NULL;
  }

  // set canonical file name length - used to speed iteration
  pIter->m_nFileLen = strlen(pIter->m_sFileCanonical);

  // no search path - just return canonical file
  if( DListStrCount(pIter->m_pSearchPaths) == 0 )
  {
    LOGDIAG4("'%s'=%s()", pIter->m_sFileCanonical, LOGFUNCNAME);

    return pIter->m_sFileCanonical;
  }

  // construct first file path
  else if( (sPath = DListStrIterDataFirst(pIter->m_pSearchPaths,
                                          &(pIter->m_iterPaths))) != NULL )
  {
    pIter->m_bIsEos = false;

    nLen  = strlen(sPath) + strlen(DIR_SEP_STR) + pIter->m_nFileLen;

    pIter->m_sFilePath = NEWSTR(nLen);

    sprintf(pIter->m_sFilePath, "%s%s%s", 
      sPath, DIR_SEP_STR, pIter->m_sFileCanonical);

    LOGDIAG4("'%s'=%s()", pIter->m_sFilePath, LOGFUNCNAME);

    return pIter->m_sFilePath;
  }

  // shouldn't have gotten here - so terminate search
  else
  {
    return NULL;
  }
}

/*!
 * \brief Construct next file path from search path and search file name.
 *
 * \param pIter Path iterator.
 *
 * \return 
 *  Returns next constructed file path on success, NULL if at end of search 
 *  or on error
 */
char *SearchPathIterNext(PathIter_T *pIter)
{
  const char      *sPath;   // search path component
  size_t           nLen;    // string length

  // end of search
  if( pIter->m_bIsEos )
  {
    return NULL;
  }

  // get the next search path component
  else if( (sPath = DListStrIterDataNext(&(pIter->m_iterPaths))) != NULL )
  {
    nLen  = strlen(sPath) + strlen(DIR_SEP_STR) + pIter->m_nFileLen;

    delete(pIter->m_sFilePath);         // clear any old constructed file path
    pIter->m_sFilePath = NEWSTR(nLen);  // allocate new file path
    
    sprintf(pIter->m_sFilePath, "%s%s%s", 
      sPath, DIR_SEP_STR, pIter->m_sFileCanonical);

    LOGDIAG4("'%s'=%s()", pIter->m_sFilePath, LOGFUNCNAME);

    return pIter->m_sFilePath;
  }

  // end of search
  else
  {
    pIter->m_bIsEos = true;
    return NULL;
  }
}

/*!
 * \brief Normalize path.
 * 
 * Normalize path, eliminating double slashes and simplifying dot "." current
 * and ".." parent directory references.
 *
 * The path does not have to exist.
 *
 * \return 
 *  Returns allocated normalized path.
 */
char *NewNormPath(const char *sPath)
{
  char *sTopDir;
  char *sNew;
  int   i, j, k;
  int   slashes;

  // Degenerative empty string case.
  if( *sPath == 0 )
  {
    return new_strdup(".");
  }

  // Normalization results in a smaller length string, except in the
  // degenerative case above.
  sNew = NEWSTR(strlen(sPath));

  // The (implicit) top directory is either the root or current directory.
  if( PathIsAbsolute(sPath) )
  {
    sTopDir = DIR_SEP_STR;
  }
  else
  {
    sTopDir = ".";
  }

  //
  // Normalize the path.
  //
  for(i=0, j=0; sPath[i]!=0; )
  {
    switch( sPath[i] )
    {
      // 
      // Potential current dot or parent dotdot directory path component.
      //
      case '.':
        // "filename." or "filename.."
        if( (j > 0) && (sNew[j-1] != DIR_SEP_CHAR) )
        {
          sNew[j++] = sPath[i++];
        }

        // "../[path]" or ".."
        else if( (sPath[i+1] == '.') && 
                ((sPath[i+2] == DIR_SEP_CHAR) || (sPath[i+2] == 0)) )
        {
          // 
          // Try to backup to parent directory.
          // example 1: "path1/pdir/cdir/../path2" -> "pathl/pdir/path2"
          // example 2: "pdir/cdir/../path2"       -> "pdir/path2"
          // example 3: "cdir/../path2"            -> "path2"
          // example 4: "/cdir/../path2"           -> "/path2"
          // example 5: " cdir/../../path2"        -> "../path2"
          // example 6: "/cdir/../../path2"        -> "/path2"
          // example 7: "dir/../cdir/.."           -> "."
          //
          for(k=j-1, slashes=0; k>=0 && slashes<2; --k)
          {
            if( sNew[k] == DIR_SEP_CHAR )
            {
              ++slashes;
            }
          }
          ++k;

          //                   i                             i
          //             k     j                         k   j
          // "[path/]pdir/cdir/..[/path]" or "[path/]pdir/../..[/path]"
          //
          if( slashes == 2 )
          {
            // if current directory is "..", then "../.."
            if( (sNew[k+1] == '.') &&
                (sNew[k+2] == '.') &&
                (sNew[k+3] == DIR_SEP_CHAR) )
            {
              sNew[j++] = '.';
              sNew[j++] = '.';
            }
            // "[path]/pdir"
            else
            {
              j = k;
              sNew[j] = 0;
            }
          }

          //   i                   i
          //  kj              k    j
          // "/..[/path]" or "cdir/..[/path]"
          //
          else if( slashes == 1 )
          {
            // relative
            if( sNew[0] != DIR_SEP_CHAR )
            {
              j = 0;
              sNew[j++] = '.';
              sNew[j] = 0;
            }
            // absolute
            else
            {
              j = 0;
              sNew[j++] = DIR_SEP_CHAR;
              sNew[j] = 0;
            }
          }

          //  i
          //  j
          //  k
          // "..[/path]"
          //
          else  // slashes == 0
          {
            j = 0;
            sNew[j++] = '.';
            sNew[j++] = '.';
            sNew[j] = 0;
          }

          i += 2;
        }

        // current dot directory "./[path]" or "."
        else if( (sPath[i+1] == DIR_SEP_CHAR) || (sPath[i+1] == 0) )
        {
          i += sPath[i+1] == DIR_SEP_CHAR? 2: 1;
        }

        // dot filename ".filename"
        else
        {
          sNew[j++] = sPath[i++];
        }
        break;

      //
      // Directory component.
      //
      case DIR_SEP_CHAR:
        // directory separator that's not superfluous "[path1]/path2"
        if( (sPath[i+1] != DIR_SEP_CHAR) && (sPath[i+1] != 0) )
        {
          // "/[path]"
          if( j == 0 )
          {
            sNew[j++] = sPath[i++];
          }
          // "//path}
          else if( sNew[j-1] == DIR_SEP_CHAR )
          {
            ++i;
          }
          // "[path1/]./[path2]"
          else if((sNew[j-1] == '.') && 
                  ((j == 1) || (sNew[j-2] == DIR_SEP_CHAR)) )
          {
            --j;
            ++i;
          }
          // "[path1]/path2"
          else
          {
            sNew[j++] = sPath[i++];
          }
        }
        // ignore extra or ending separator "//" or "/"
        else
        {
          ++i;
        }
        break;

      // 
      // Non-special file character.
      //
      default:
        sNew[j++] = sPath[i++];
        break;
    }
  }

  sNew[j] = 0;

  if( *sNew == 0 )
  {
    strcpy(sNew, sTopDir);
  }

  return sNew;
}

/*!
 * \brief Expands and canonicalizes a real path.
 *
 * The expanded path must exist.
 *
 * \param sPath   Path file name.
 *
 * \return 
 *  Return allocated canonical path on success, NULL if path doest not exist.
 */
char *NewRealPath(const char *sPath)
{
  char  *s, *t;

  s = NewSearchPathCanonicalized(sPath);
  t = realpath(s, NULL);
  delete(s);
  return t;
}

/*!
 * \brief Join two file paths.
 *
 * The paths are expanded and canonicalized. The paths do not have to exist.
 * If the second, appended path is an absolute path, then the joined path is
 * the second path.
 *
 * \param  sPath1   Front absolute/relative path.
 * \param  sPath2   Appended, back end path.
 *
 * \return
 * On success, an allocated, canonical joined path name is returned.\n
 * On failure, NULL is returned.
 */
char *NewJoinedPath(const char *sPath1, const char *sPath2)
{
  char   *p1 = NULL;
  char   *p2 = NULL;
  char   *pj = NULL;

  // canonicalize path 2
  if( (p2 = NewSearchPathCanonicalized(sPath2)) == NULL )
  {
    LOGERROR("NewSearchPathCanonicalized(\"%s\") failed.", sPath2);
  }

  // if the second path is an absolute path, cannot join
  else if( PathIsAbsolute(p2) )
  {
    return p2;
  }

  // canonicalize path 1
  else if( (p1 = NewSearchPathCanonicalized(sPath1)) == NULL )
  {
    LOGERROR("NewSearchPathCanonicalized(\"%s\") failed.", sPath1);
  }

  // join
  else
  {
    pj = NEWSTR(strlen(p1)+strlen(DIR_SEP_STR)+strlen(p2));
    sprintf(pj, "%s%s%s", p1, DIR_SEP_STR, p2);
  }

  delete(p1);
  delete(p2);

  return pj;
}
