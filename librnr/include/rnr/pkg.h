////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief RoadNarrows Robotics standard package information.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/pkg.h}
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

#ifndef _RNR_PKG_H
#define _RNR_PKG_H

C_DECLS_BEGIN

/*!
 * RNR Package Information Structure
 */
typedef struct
{
  const char *m_sPkgName;           ///< package name string
  const char *m_sPkgVersion;        ///< package dotted version string
  const char *m_sPkgTimeStamp;      ///< package build date
  const char *m_sPkgDate;           ///< package extended creation date string
  const char *m_sPkgFullName;       ///< package full name string (incl version)
  const char *m_sPkgAuthors;        ///< package author(s) string
  const char *m_sPkgOwners;         ///< package owner(s) string
  const char *m_sPkgDisclaimer;     ///< package disclaimer string
} PkgInfo_T;

C_DECLS_END


#endif // _RNR_PKG_H
