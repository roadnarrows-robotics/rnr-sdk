////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      config.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Configuration parser declarations.
 *
 * The librnr config module implements a basic configuration file parser
 * and database that provides a structure similar to what you would find on
 * Microsoft Windows INI files.
 *
 * This file has been modified from the original source (see below).
 *
 * \par Syntax:
 * The configuration file consists of a series of sections, headed by a
 * "[section]" header and followed by "key: value" or "key=value" entries.\n 
 * \n
 * Comments start with the character '#'. The configuration parser wlll ignore
 * the '#' and all trailing characters to the end of the current line.
 *
 * \sa 
 * \ref example_config under "Related Pages" for an example usage of
 * configuration.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * <hr>
 * \par Original Source and Copyright:
 *
 * \par Original Author:
 * Jon Travis (jtravis@p00p.org)
 *
 * \par Original Copyright:
 * (C) 1992-2002
 *
 * \par Original Header:
 * See "Original Source Header EULA" in source file.
 *
 * <hr>
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
// Original Source Header and Copyright:
//
//  camserv - An internet streaming picture application
//
//  Copyright (C) 1999-2002  Jon Travis (jtravis@p00p.org)
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdio.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * Forward declaration of types, hiding implementation.
 */
typedef struct config_t         Config_T;       ///< configuration database type
typedef struct config_section_t ConfigSection_T;  ///< configuration section tpe
typedef struct config_iter_t    ConfigIter_T;   ///< configuration iterator type


// ...........................................................................
// Configuration Database Prototypes
// ...........................................................................

extern Config_T *ConfigDbNew(const char *sMainName);

extern void ConfigDbDelete(Config_T *pConfig);

extern Config_T *ConfigDbRead(const char *sFileName);

extern Config_T *ConfigDbReadFp(FILE *fp, const char *sFileName);

extern const char *ConfigDbGetName(Config_T *pConfig);

extern void ConfigDbSetName(Config_T *pConfig, const char *sMainName);

extern void ConfigDbPrint(Config_T *pConfig, FILE *fp);


// ...........................................................................
// Configuration Section Prototypes.
// ...........................................................................

extern ConfigSection_T *ConfigSectionNew(Config_T *pConfig,
                                         const char *sSectionName);

extern int ConfigSectionDelete(Config_T *pConfig,
                               ConfigSection_T *sSectionName);

extern ConfigSection_T *ConfigSectionGet(Config_T *pConfig,
                                         const char *sSectionName);

extern void ConfigSectionPrint(ConfigSection_T *pSect, FILE *fp);


// ...........................................................................
// Configuration Get key=value Entry Prototypes and Inlines 
// ...........................................................................

extern const char *ConfigGetStr(Config_T *pConfig, const char *sSectionName, 
                                const char *sKey);

extern int ConfigGetLong(Config_T *pConfig, const char *sSectionName,
                         const char *sKey, long *pVal);

extern int ConfigGetDouble(Config_T *pConfig, const char *sSectionName,
                           const char *sKey, double *pVal);

/*!
 * \brief Get and convert an int value from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param pVal          Converted value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
static inline int ConfigGetInt(Config_T *pConfig, const char *sSectionName,
                               const char *sKey, int *pVal)
{
  return ConfigGetLong(pConfig, sSectionName, sKey, (long *)pVal);
}

/*!
 * \brief Get and convert an unsigned long value from the configuration
 * database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param pVal          Converted value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
static inline int ConfigGetULong(Config_T *pConfig, const char *sSectionName,
                                 const char *sKey, unsigned long *pVal)
{
  return ConfigGetLong(pConfig, sSectionName, sKey, (long *)pVal);
}

/*!
 * \brief Get a value from the configuration database or default.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param sValDft       Default value.
 *
 * \return 
 * Returns value found in database on success.\n
 * Returns default value if no value found in database or on error.
 */
static inline const char *ConfigGetStrDft(Config_T *pConfig,
                                          const char *sSectionName,
                                          const char *sKey,
                                          const char *sValDft)
{
  const char *sVal;

  return (sVal = ConfigGetStr(pConfig, sSectionName, sKey)) != NULL?
                  sVal: sValDft;
}

// ...........................................................................
// Configuration Set key=value Entry Prototypes and Inlines
// ...........................................................................

extern int ConfigSetStr(Config_T *pConfig, const char *sSectionName, 
                        const char *sKey, const char *sVal);

extern int ConfigSetLong(Config_T *pConfig, const char *sSectionName, 
                         const char *sKey, long lVal);

extern int ConfigSetULong(Config_T *pConfig, const char *sSectionName, 
                          const char *sKey, unsigned long ulVal);

extern int ConfigSetDouble(Config_T *pConfig, const char *sSectionName, 
                           const char *sKey, double fVal);

/*!
 * \brief Set or update an int value in the configuration database.
 *
 * The long value is converted to a string prior to adding to the database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param iVal          Section key int value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
static inline int ConfigSetInt(Config_T *pConfig, const char *sSectionName,
                               const char *sKey, int iVal)
{
  return ConfigSetLong(pConfig, sSectionName, sKey, (long)iVal);
}


// ...........................................................................
// Configuration Delete Entry Functions
// ...........................................................................

extern int ConfigDelete(Config_T *pConfig, const char *sSectionName,
                        const char *sKey);


// ...........................................................................
// Configuration Iterator Functions
// ...........................................................................

extern ConfigIter_T *ConfigDbIterNew(Config_T *pConfig);

extern ConfigIter_T *ConfigSectionIterNew(Config_T *pConfig,
                                          const char *sSectionName);

extern void ConfigIterDelete(ConfigIter_T *pIter);

extern const char *ConfigIterNext(ConfigIter_T *pIter);

C_DECLS_END


#endif // _CONFIG_H
