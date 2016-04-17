////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      uri.h
//
/*! \file
 *
 * $LastChangedDate: 2012-02-24 11:19:26 -0700 (Fri, 24 Feb 2012) $
 * $Rev: 1848 $
 *
 * \brief Uniform Resource Identifier (URI) parsing utilities declarations.
 *
 * \par URI Components:
 * scheme://userinfo\@hostname:port/path?query
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows LLC.
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

#ifndef _URI_H
#define _URI_H

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

#define URI_SEP_SCHEME            "://" ///< scheme separator
#define URI_SEP_SCHEME_LEN        3     ///< scheme separator length
#define URI_SEP_USER_PASSWORD     ":"   ///< user info password separator
#define URI_SEP_USER_PASSWORD_LEN 1     ///< user info password separator length
#define URI_SEP_USER_INFO         "@"   ///< user info separator
#define URI_SEP_USER_INFO_LEN     1     ///< user info separator length
#define URI_SEP_PORT              ":"   ///< port number separator
#define URI_SEP_PORT_LEN          1     ///< port number separator length
#define URI_SEP_PATH              "/"   ///< absolute path separator and start
#define URI_SEP_PATH_LEN          1     ///< absolute path separator length
#define URI_SEP_QUERY             "?"   ///< query separator
#define URI_SEP_QUERY_LEN         1     ///< query separator length

#define URI_PORT_MAX_LEN          5     ///< port max string length (2 bytes)

#define URI_SCHEME_FILE           "file"        ///< file scheme
#define URI_LOCAL_HOST            "localhost"   ///< default local hostname
#define URI_PORT_NONE             0             ///< no port number


/*!
 * URI Component Parts Structure.
 */
typedef struct uri_struct_t
{
  char *m_sScheme;      ///< scheme
  char *m_sUserInfo;    ///< user info
  char *m_sHostName;    ///< host name (domain or address)
  int   m_nPortNum;     ///< port number
  char *m_sPath;        ///< absolute file path
  char *m_sQuery;       ///< query
} Uri_T;


extern Uri_T *UriParseNew(const char *sUri);

extern char *UriParseHostNew(const char  *sHost,
                             char       **pHostName,
                             int         *pPortNum);

extern void UriSetScheme(Uri_T *pUri, const char *sScheme);

extern void UriSetHostName(Uri_T *pUri, const char *sHostName);

extern void UriSetPortNum(Uri_T *pUri, int nPortNum);

extern void UriSetPath(Uri_T *pUri, const char *sPath);

extern char *UriStrNew(const Uri_T *pUri);

extern void UriDelete(Uri_T *pUri);


C_DECLS_END

#endif // _URI_H
