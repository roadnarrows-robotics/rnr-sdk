////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// File:      enConf.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-29 14:23:33 -0700 (Fri, 29 Jan 2016) $
 * $Rev: 4287 $
 *
 * \brief Target specific configuration.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _EU_CONF_H
#define _EU_CONF_H

#if defined(ARCH_linaro) || defined(ARCH_overo)
#undef  EU_HAS_PCL      ///< these old targets do not have PCL support
#undef EU_HAS_OPENNI    ///< system does not have OpenNI
#undef EU_HAS_GST_1_0   ///< system has older gstreamer-0.10
#else
#define EU_HAS_PCL      ///< default targets do have PCL support
#define EU_HAS_OPENNI   ///< system has OpenNI
#define EU_HAS_GST_1_0  ///< system has gstreamer-1.0
#endif // ARCH_linaro || ARCH_overo


//
// Eudoxus Directories, files, and environment variable names.
//

//#ifndef EU_HAS_NI

// 
// Install prefix
//
#ifndef EU_INSTALL_PREFIX
#define EU_INSTALL_PREFIX  "/usr/local"   ///< on-target product
#endif // EU_INSTALL_PREFIX

//
// Applications images directory.
//
#ifndef EU_IMAGE_DIR
#define EU_IMAGE_DIR  EU_INSTALL_PREFIX "/share/Eudoxus/images"
#endif

//
// Applications icon directory.
//
#ifndef EU_ICON_DIR
#define EU_ICON_DIR   EU_INSTALL_PREFIX "/share/Eudoxus/images/icons"
#endif

//
// Application configuration etc directory.
//
#ifndef EU_ETC_DIR
#define EU_ETC_DIR    EU_INSTALL_PREFIX "/etc/eudoxus"
#endif

//#endif // !EU_HAS_NI

//
// OpenNI modules file basename.
//
#ifndef EU_NI_MODULES_FILE_BASENAME
#define EU_NI_MODULES_FILE_BASENAME   "modules.xml"
#endif

//
// OpenNI licenses file basename.
//
#ifndef EU_NI_LICENSES_FILE_BASENAME
#define EU_NI_LICENSES_FILE_BASENAME  "licenses.xml"
#endif

//
// OpenNI configuration file basename.
//
#ifndef EU_NI_CONFIG_FILE_BASENAME
#define EU_NI_CONFIG_FILE_BASENAME    "openni-config.xml"
#endif

//
// OpenNI user openni configuraton fully qualified file name.
//
#ifndef EU_NI_USER_CONFIG_FILE
#define EU_NI_USER_CONFIG_FILE  "~/" EU_NI_CONFIG_FILE_BASENAME
#endif

//
// OpenNI system openni configuraton fully qualified file name.
//
#ifndef EU_NI_SYS_CONFIG_FILE
#define EU_NI_SYS_CONFIG_FILE  "/usr/local/etc" "/" EU_NI_CONFIG_FILE_BASENAME
#endif

//
// Eudoxus OpenNI configuraton file name environment variable.
//
#define EU_ENV_HOME             "HOME"
#define EU_ENV_NI_CONFIG_FILE   "EU_NI_CONFIG_FILE"

#endif // _EU_CONF_H
