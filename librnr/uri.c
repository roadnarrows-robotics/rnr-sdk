////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      uri.c
//
/*! \file
 *
 * $LastChangedDate: 2012-02-24 11:19:26 -0700 (Fri, 24 Feb 2012) $
 * $Rev: 1848 $
 *
 * \brief Uniform Resource Identifier (URI) parsing utiliies.
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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/uri.h"

/*!
 * \brief Parse a URI string.
 *
 * Any URI string components not found are set to NULL. If no port is specified,
 * the port number is set to \ref URI_PORT_NONE (0).
 *
 * The Uri_T structure is allocated. The calling application owns the 
 * structure. See \ref UriDelete().
 *
 * \par Format:
 * scheme://userinfo\@hostname:port/path/to/something?query&field=val...
 *
 * \param sUri    URI string.
 *
 * \return Allocated and assigned Uri_T *.
 */
Uri_T *UriParseNew(const char *sUri)
{
  Uri_T  *pUri = NEW(Uri_T);      // new uri structure, initialized to 0 (null)
  char   *s    = (char *)sUri;    // working uri string head pointer
  char   *t;                      // working uri string tail pointer
  size_t  n;                      // size of uri string component

  // scheme://
  if( (t = strstr(s, URI_SEP_SCHEME)) != NULL )
  {
    n = (size_t)(t - s);

    if( n > 0 )
    {
      pUri->m_sScheme = new_strndup(s, n);
    }

    s = t + URI_SEP_SCHEME_LEN;
  }

  if( *s == 0 )
  {
    return pUri;
  }

  // userinfo@
  if( (t = strstr(s, URI_SEP_USER_INFO)) != NULL )
  {
    n = (size_t)(t - s);

    if( n > 0 )
    {
      pUri->m_sUserInfo = new_strndup(s, n);
    }

    s = t + URI_SEP_USER_INFO_LEN;
  }

  if( *s == 0 )
  {
    return pUri;
  }

  // hostname:port 
  s = UriParseHostNew(s, &(pUri->m_sHostName), &(pUri->m_nPortNum));

  if( *s == 0 )
  {
    return pUri;
  }

  // /path?query
  if( !strncmp(s, URI_SEP_PATH, (size_t)URI_SEP_PATH_LEN) )
  {
    if( *s == 0 )
    {
      return pUri;
    }

    if( (t = strstr(s, URI_SEP_QUERY)) != NULL )
    {
      n = (size_t)(t - s);

      if( n > 0 )
      {
        pUri->m_sPath = new_strndup(s, n);
      }

      s = t + URI_SEP_QUERY_LEN;

      if( *s != 0 )
      {
        pUri->m_sQuery = new_strdup(s);
      }
    }
    else
    {
      pUri->m_sPath = new_strdup(s);
    }
  }

  // ?query
  if( !strncmp(s, URI_SEP_QUERY, (size_t)URI_SEP_QUERY_LEN) )
  {
    s += URI_SEP_QUERY_LEN;

    if( *s != 0 )
    {
      pUri->m_sQuery = new_strdup(s);
    }
  }

  return pUri;
}

/*!
 * \brief Parse the host string.
 *
 * If no hostname is specified, NULL is set. If no port number is specified,
 * URI_PORT_NONE is set.
 *
 * \par Format:
 * hostname:port...
 *
 * \param sHost             Host string.
 * \param [out] pHostName   Pointer to the allocated parsed host name string.
 * \param [out] pPortNum    Pointer to the parsed port number.
 *
 * \return
 * Returns pointer to the first character after the host substring. If sHost
 * is NULL, NULL is return. If now URI components follow the host string,
 * will point to the end terminator of sHost.
 */
char *UriParseHostNew(const char *sHost, char **pHostName, int *pPortNum)
{
  char   *s;    // working host string head pointer
  char   *t;    // working host string tail pointer
  size_t  n;    // size of host string component

  *pHostName  = NULL;
  *pPortNum   = URI_PORT_NONE;

  if( sHost == NULL )
  {
    return NULL;
  }

  if( ((t = strstr(sHost, URI_SEP_PATH)) == NULL) && 
      ((t = strstr(sHost, URI_SEP_QUERY)) == NULL) )
  {
    t = (char *)sHost + strlen(sHost);
  }

  if( (s = strstr(sHost, URI_SEP_PORT)) == NULL )
  {
    s = t;
  }

  n = (size_t)(s - sHost);

  if( n > 0 )
  {
    *pHostName = new_strndup(sHost, n);
  }

  if( s+1 < t )
  {
    *pPortNum = (int)strtol(s+1, NULL, 10);
  }
  
  return t;
}

/*!
 * \brief Set the scheme.
 *
 * \param pUri    URI scheme.
 * \param sScheme New scheme.
 */
void UriSetScheme(Uri_T *pUri, const char *sScheme)
{
  if( pUri == NULL )
  {
    return;
  }

  if( pUri->m_sScheme != NULL )
  {
    delete(pUri->m_sScheme);
    pUri->m_sScheme = NULL;
  }

  if( (sScheme != NULL) && (*sScheme != 0) )
  {
    pUri->m_sScheme = new_strdup(sScheme);
  }
}

/*!
 * \brief Set the hostname.
 *
 * \param pUri        URI scheme.
 * \param sHostName   New hostname.
 */
void UriSetHostName(Uri_T *pUri, const char *sHostName)
{
  if( pUri == NULL )
  {
    return;
  }

  if( pUri->m_sHostName != NULL )
  {
    delete(pUri->m_sHostName);
    pUri->m_sHostName = NULL;
  }

  if( (sHostName != NULL) && (*sHostName != 0) )
  {
    pUri->m_sHostName = new_strdup(sHostName);
  }
}

/*!
 * \brief Set the port number.
 *
 * \param pUri    URI scheme.
 * \param nPortNum New scheme.
 */
void UriSetPortNum(Uri_T *pUri, int nPortNum)
{
  if( pUri == NULL )
  {
    return;
  }

  pUri->m_nPortNum = nPortNum >= 0? nPortNum: URI_PORT_NONE;
}

/*!
 * \brief Set the file path.
 *
 * \param pUri    URI scheme.
 * \param sPath New scheme.
 */
void UriSetPath(Uri_T *pUri, const char *sPath)
{
  if( pUri == NULL )
  {
    return;
  }

  if( pUri->m_sPath != NULL )
  {
    delete(pUri->m_sPath);
    pUri->m_sPath = NULL;
  }

  if( (sPath != NULL) && (*sPath != 0) )
  {
    pUri->m_sPath = new_strdup(sPath);
  }
}

/*!
 * \brief Construct a new URI string from the given URI components.
 *
 * \param pUri  Pointer to URI structure.
 *
 * \return Allocated string.
 */
char *UriStrNew(const Uri_T *pUri)
{
  char   *sUri;   // uri string
  Uri_T   uri = {"", "", "", 0, "", ""};
  Uri_T   sep = {"", "", "", 0, "", ""};
  char    bufPort[URI_SEP_PORT_LEN+URI_PORT_MAX_LEN+1] = {0, };
  size_t  n = 0;

  if( pUri == NULL )
  {
    return NULL;
  }

  if( pUri->m_sScheme != NULL )
  {
    n += strlen(pUri->m_sScheme) + (size_t)URI_SEP_SCHEME_LEN;
    uri.m_sScheme = pUri->m_sScheme;
    sep.m_sScheme = URI_SEP_SCHEME;
  }

  if( pUri->m_sUserInfo != NULL )
  {
    n += strlen(pUri->m_sUserInfo) + (size_t)URI_SEP_USER_INFO_LEN;
    uri.m_sUserInfo = pUri->m_sUserInfo;
    sep.m_sUserInfo = URI_SEP_USER_INFO;
  }

  if( pUri->m_sHostName != NULL )
  {
    n += strlen(pUri->m_sHostName);
    uri.m_sHostName = pUri->m_sHostName;

    if( pUri->m_nPortNum != URI_PORT_NONE )
    {
      n += (size_t)URI_SEP_PORT_LEN + (size_t)URI_PORT_MAX_LEN;
      snprintf(bufPort, sizeof(bufPort), "%s%d",
          URI_SEP_PORT, pUri->m_nPortNum);
      bufPort[sizeof(bufPort)-1] = 0;
    }
  }

  if( pUri->m_sPath != NULL )
  {
    n += strlen(pUri->m_sPath);
    uri.m_sPath = pUri->m_sPath;
    if( strncmp(uri.m_sPath, URI_SEP_PATH, URI_SEP_PATH_LEN) )
    {
      n++;
      sep.m_sPath = URI_SEP_PATH;
    }
  }

  if( pUri->m_sQuery != NULL )
  {
    n += URI_SEP_QUERY_LEN + strlen(pUri->m_sQuery);
    uri.m_sQuery = pUri->m_sQuery;
    sep.m_sQuery = URI_SEP_QUERY;
  }

  sUri = NEWSTR(n);

  sprintf(sUri, "%s%s%s%s%s%s%s%s%s%s",
      uri.m_sScheme, sep.m_sScheme,
      uri.m_sUserInfo, sep.m_sUserInfo,
      uri.m_sHostName, bufPort,
      sep.m_sPath, uri.m_sPath,
      sep.m_sQuery, uri.m_sQuery);

  return sUri;
}

/*!
 * \brief Delete the URI compenent structure.
 *
 * \param pUri    Pointer to allocated Uri_T.
 */
void UriDelete(Uri_T *pUri)
{
  if( pUri == NULL )
  {
    return;
  }

  if( pUri->m_sScheme != NULL )
  {
    delete(pUri->m_sScheme);
  }

  if( pUri->m_sUserInfo != NULL )
  {
    delete(pUri->m_sUserInfo);
  }

  if( pUri->m_sHostName != NULL )
  {
    delete(pUri->m_sHostName);
  }

  if( pUri->m_sPath != NULL )
  {
    delete(pUri->m_sPath);
  }

  if( pUri->m_sQuery != NULL )
  {
    delete(pUri->m_sQuery);
  }

  delete(pUri);
}
