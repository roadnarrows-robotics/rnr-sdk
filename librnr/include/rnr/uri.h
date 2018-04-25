////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Uniform Resource Identifier (URI) parsing utilities declarations.
 *
 * URI Components: \c scheme://userinfo\@hostname:port/path?query
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/uri.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2011-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_URI_H
#define _RNR_URI_H

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

#endif // _RNR_URI_H
