////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Configuration parser declarations.
 *
 * The librnr config module implements a basic configuration file parser
 * and database that provides a structure similar to what you would find on
 * Microsoft Windows INI files.
 *
 * This file has been modified from the original source
 * (\ref config_h_original_src "see below").
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
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/config.h}
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
 *
 * <hr>
 * \anchor config_h_original_src
 * \par Original Source Comment Block
 *
 * \par Original Author
 * Jon Travis (jtravis@p00p.org)
 *
 * \par Original Copyright
 * (C) 1992-2002
 *
 * \par Original Header
 * \verbatim
 * camserv - An internet streaming picture application
 * 
 * Copyright (C) 1999-2002  Jon Travis (jtravis@p00p.org)
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 * \endverbatim
 *
 * <hr>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_CONFIG_H
#define _RNR_CONFIG_H

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


#endif // _RNR_CONFIG_H
