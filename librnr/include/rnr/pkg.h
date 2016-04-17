////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      pkg.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief RoadNarrows Robotics standard package information.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2005-2010.  RoadNarrows LLC.
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

#ifndef _PKG_H
#define _PKG_H

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


#endif // _PKG_H
