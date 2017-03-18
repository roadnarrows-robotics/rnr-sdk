////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      config.c
//
/*! \file
 *
 * $LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
 * $Rev: 3823 $
 *
 * \brief Configuration parser definitions.
 *
 * This file has been modified from the original source (see below).
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
//
// Original Source Header EULA:
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/hash.h"
#include "rnr/config.h"

// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------

/*!
 * Configuration limits.
 */
#define HASH_MIN_SECTIONS   (hashcount_t)8  ///< min. num. of database sections
#define HASH_MAX_SECTIONS   HASHCOUNT_T_MAX ///< max. num. of database sections
#define HASH_MIN_ENTRIES    (hashcount_t)8  ///< min. num. of section entries
#define HASH_MAX_ENTRIES    HASHCOUNT_T_MAX ///< max. num. of section entries

/*!
 * \brief Maximum input line size.
 */
#define HASH_INPUT_BUF_SIZE 1024

/*!
 * Configuration section structure type.
 */
struct config_section_t
{
  char    *m_sSectionName;  ///< section name
  hash_t  *m_hashSection;   ///< section's key=value entries hash table
};

/*!
 * Configuration database structure type.
 */
struct config_t 
{
  char    *m_sMainName;     ///< configuration name
  hash_t  *m_hashMain;      ///< main hash table of sections
};

/*!
 * Configuration iterator type.
 */
struct config_iter_t
{
  hscan_t   m_scanHash;     ///< hash table scanner (iterator)
};

/*!
 * Internal counter to make configuration names unique.
 */
static int  ConfigMainCounter = 0;

/*!
 * \brief Add or update a key-value pair in the given section.  
 *
 * \param pSect   Configuration section to update.
 * \param sKey    Section key to add or update.
 * \param sVal    Key value to add
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
static int ConfigSectionAddPair(ConfigSection_T *pSect,
                                const char *sKey, 
                                const char *sVal)
{
  hnode_t *pHashNode;
  char    *sKeyDup;
  char    *sValDup;

  // Lookup key in section. If it exists, then replace with new value
  if( (pHashNode = hash_lookup(pSect->m_hashSection, sKey)) != NULL )
  {
    sValDup = new_strdup(sVal);

    // delete old value
    delete(hnode_get(pHashNode));

    // replacing with new value
    hnode_put(pHashNode, sValDup);
  }

  // New section key
  else
  {
    sKeyDup = new_strdup(sKey);
    sValDup = new_strdup(sVal);

    if( !hash_insert(pSect->m_hashSection, sKeyDup, sValDup) )
    {
      delete(sKeyDup);
      delete(sValDup);
      return RC_ERROR;
    }
  }

  return OK;
}

/*!
 * \brief Delete section entry hash callback.
 *
 * When a key-value entry or section is deleted, the hash will make a callback
 * to this function to delete the hash key and data.
 *
 * \param pKey    Section key to delete.
 * \param pData   Key value to delete.
 */
static void ConfigSectionHashDeleteCb(void *pKey, void *pData)
{
  delete(pKey);   // section hash key (char *)
  delete(pData);  // section hash data (char *)
}

/*!
 * \brief Delete section hash callback.
 *
 * When a section or configuration datatbase is deleted, the hash will make a
 * callback to this function to delete the hash key and data (section).
 *
 * \param pKey    Section key to main hash.
 * \param pData   Key data to delete (ConfigSection_T *).
 */
static void ConfigMainHashDeleteCb(void *pKey, void *pData)
{
  ConfigSection_T *pSect;

  // main hash key points to section name, so don't delete here

  pSect = (ConfigSection_T *)pData;         // main hash data
  hash_table_destroy(pSect->m_hashSection); // section hash table
  delete(pSect->m_sSectionName);            // section name
  delete(pSect);                            // section
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

// ...........................................................................
// Configuration Database Functions
// ...........................................................................

/*!
 * \brief Create and initialize a new empty configuration database.
 *
 * \param sMainName Name of configuration database. If Null, then a name
 *                  will be auto-generated.
 *
 * \return 
 * Returns new configuration database on success.\n
 * Returns NULL on failure.
 */
Config_T *ConfigDbNew(const char *sMainName)
{
  Config_T *pConfig;
  char      buf[32];

  LOGDIAG4CALL(_TSTR(sMainName));

  // new main configuration structure
  pConfig = NEW(Config_T);

  // new empty main hash table of sections
  pConfig->m_hashMain = hash_table_create(true,
                                       HASH_MIN_SECTIONS,
                                       HASH_MAX_SECTIONS,
                                       NULL,
                                       NULL,
                                       ConfigMainHashDeleteCb);

  if( pConfig->m_hashMain == NULL )
  {
    LOGERROR("failed to create new configuration main hash table");
    delete(pConfig);
    return NULL;
  }

  // create internal name
  if( sMainName == NULL )
  {
    snprintf(buf, sizeof(buf), "__config.%d", ConfigMainCounter++);
    buf[sizeof(buf)-1] = 0;
    pConfig->m_sMainName = new_strdup(buf);
  }

  // use given name
  else
  {
    pConfig->m_sMainName = new_strdup(sMainName);
  }

  LOGDIAG3(_TPTR(pConfig));

  return pConfig;
}

/*!
 * \brief Delete the configuration database.
 *
 * All sections, section key=value entries, plus the database itself are
 * deleted.
 *
 * \param pConfig Configuration database.
 */
void ConfigDbDelete(Config_T *pConfig)
{
  LOGDIAG4CALL(_TSTR(pConfig));

  if( pConfig == NULL )
  {
    return;
  }

  // delete all sections and section entries
  hash_table_destroy(pConfig->m_hashMain);

  // now delete the structure
  delete(pConfig->m_sMainName);
  delete(pConfig);
}

/*!
 * \brief Create a new configuration database from a configuration file.
 *
 * The file syntax must conform to the simplified 'INI' format supported
 * by librnr config.
 *
 * The file is closed upon exiting this function.
 *
 * \param sFileName File path name of configuration file.
 *
 * \return 
 * Returns populated configuration database on success.\n
 * Returns NULL on failure.
 */
Config_T *ConfigDbRead(const char *sFileName)
{
  FILE      *fp;
  Config_T  *pConfig;

  LOGDIAG4CALL(_TSTR(sFileName));

  CHKPTR(sFileName, NULL);

  if( (fp = fopen(sFileName, "r")) == NULL )
  {
    LOGSYSERROR("%s", sFileName);
    return NULL;
  }
  else
  {
    pConfig = ConfigDbReadFp(fp, sFileName);
    fclose(fp);
    LOGDIAG4(_TPTR(pConfig));
    return pConfig;
  }
}

/*!
 * \brief Create a new configuration database from a opened configuration file.
 *
 * The file syntax must conform to the simplified 'INI' format supported
 * by librnr config.
 *
 * The file is remains open upon exiting this function.
 *
 * \param fp        Input file pointer.
 * \param sFileName Input file name. NULL is okay.
 *
 * \return 
 * Returns populated configuration database on success.\n
 * Returns NULL on failure.
 */
Config_T *ConfigDbReadFp(FILE *fp, const char *sFileName)
{
  Config_T        *pConfig;
  ConfigSection_T *pCurSect;
  char             bufGet[HASH_INPUT_BUF_SIZE];
  char             bufKey[HASH_INPUT_BUF_SIZE];
  char             bufSep[HASH_INPUT_BUF_SIZE];
  char             bufVal[HASH_INPUT_BUF_SIZE];
  char            *s, *t; 
  int              nLineNum;
  int              i;

  LOGDIAG4CALL(_TPTR(fp), _TSTR(sFileName));

  CHKPTR(fp, NULL);

  // New configuration
  if( (pConfig = ConfigDbNew(sFileName)) == NULL )
  {
    return NULL;
  }

  pCurSect = NULL;
  nLineNum = 0;

  while( fgets(bufGet, (int)sizeof(bufGet), fp) != NULL )
  {
    nLineNum++;

    // eat white-space
    for(i=0; isspace((int)bufGet[i]); ++i); 

    // comment or white space
    if( (bufGet[i] == 0) || (bufGet[i] == '#') || (bufGet[0] == '\n') ) 
    {
      continue; // empty or comment line
    }

    // 
    // Begin a section
    //
    if( bufGet[i] == '[' )
    {
      if( (t = strrchr(bufGet+i, ']' )) == NULL )
      {
        LOGERROR("Config %s: Line %d: malformed section statement: %d",
            pConfig->m_sMainName, nLineNum );
        continue; // recoverable error
      }
      s = &bufGet[i+1];
      *t = '\0';

      if( (pCurSect = ConfigSectionNew(pConfig, s)) == NULL )
      {
        LOGERROR("Config %s: [%s]: Line %d: failed adding section",
            pConfig->m_sMainName, s, nLineNum);
        ConfigDbDelete(pConfig);
        return NULL;  // non-recoverable error
      }
      continue;
    }

    //
    // Orphaned Key-Value Pair
    //
    if( pCurSect == NULL )
    {
      LOGERROR("Config %s: Line %d: not in a section",
          pConfig->m_sMainName, nLineNum );
      continue; // recoverable error
    }

    //
    // Key-Value Pair
    //
    bufKey[0] = bufSep[0] = bufVal[0] = 0;
    if( sscanf(bufGet+i, "%[^:= \t]%[:= \t]%[^\n#]", bufKey, bufSep, bufVal)
                                                                      != 3)
    {
      LOGERROR("Config %s: [%s]: Line %d: malformed input", 
          pConfig->m_sMainName, pCurSect->m_sSectionName, nLineNum );
      continue; // recoverable error
    }

    // strip any trailing white-space
    i = (int)strlen(bufVal) - 1;
    while( (i >= 0) && isspace((int)bufVal[i]) )
    {
      --i;
    }
    if( i < 0 )
    {
      LOGERROR("Config %s: [%s]: Line %d: malformed input", 
          pConfig->m_sMainName, pCurSect->m_sSectionName, nLineNum );
      continue; // recoverable error
    }
    else
    {
      bufVal[i+1] = 0;
    }

    if( ConfigSectionAddPair(pCurSect, bufKey, bufVal) == RC_ERROR )
    {
      LOGERROR("Config %s: [%s]: Line %d: failed to add key=value enttry", 
          pConfig->m_sMainName, pCurSect->m_sSectionName, nLineNum );
      ConfigDbDelete(pConfig);
      return NULL; // non-recoverable error
    }
  }

  LOGDIAG4(_TPTR(pConfig));

  return pConfig;
}

/*!
 * \brief Get the current name assigned to the configuration database.
 *
 * \param pConfig Configuration database.
 *
 * \return Returns current name.
 */
const char *ConfigDbGetName(Config_T *pConfig)
{
  CHKPTR(pConfig, NULL);
  return pConfig->m_sMainName;
}

/*!
 * \brief Assign a new name for the configuration database.
 *
 * \param pConfig   Configuration database.
 * \param sMainName New database name.
 */
void ConfigDbSetName(Config_T *pConfig, const char *sMainName)
{
  CHKPTR(pConfig);
  CHKPTR(sMainName);
  delete(pConfig->m_sMainName);
  pConfig->m_sMainName = new_strdup(sMainName);
}

/*!
 * \brief Print configuration database to the output file stream.
 *
 * The output format is valid for input.
 *
 * \param pConfig Configuration database.
 * \param fp      Output file pointer.
 */
void ConfigDbPrint(Config_T *pConfig, FILE *fp)
{
  hscan_t          scanHash;
  hnode_t         *pHashNode;
  ConfigSection_T *pSect;

  fprintf(fp, "# Configuration %s\n", pConfig->m_sMainName);
  hash_scan_begin(&scanHash, pConfig->m_hashMain);
  while((pHashNode = hash_scan_next(&scanHash)) != NULL )
  {
    pSect = (ConfigSection_T *)hnode_get(pHashNode);
    ConfigSectionPrint(pSect, fp);
    fprintf(fp, "\n");
  }
}


// ...........................................................................
// Configuration Section Functions
// ...........................................................................

/*!
 * \brief Add a new empty section to the configuration database.
 *
 * The section cannot already exist.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 *
 * \return 
 * Returns configuration section on success.\n
 * NULL on failure.
 */
ConfigSection_T *ConfigSectionNew(Config_T *pConfig, const char *sSectionName)
{
  ConfigSection_T *pSect;

  LOGDIAG4CALL(_TPTR(pConfig), _TSTR(sSectionName));

  CHKPTR(pConfig, NULL);
  CHKPTR(sSectionName, NULL);

  // make sure section does not already exist
  if( hash_lookup(pConfig->m_hashMain, sSectionName) != NULL )
  {
    LOGERROR("%s: [%s]: section multiply defined",
        pConfig->m_sMainName, sSectionName);
    return NULL;
  }
    
  // new section structure 
  pSect = NEW(ConfigSection_T);

  // new section hash table of key=value entries
  pSect->m_hashSection = hash_table_create(true,
                                       HASH_MIN_ENTRIES,
                                       HASH_MAX_ENTRIES,
                                       NULL,
                                       NULL,
                                       ConfigSectionHashDeleteCb);

  if( pSect->m_hashSection == NULL )
  {
    LOGERROR("%s: [%s]: failed to create new section hash table",
        pConfig->m_sMainName, sSectionName);
    delete(pSect);
    return NULL;
  }

  // section name
  pSect->m_sSectionName = new_strdup(sSectionName);

  // insert section into main hash table
  hash_insert(pConfig->m_hashMain, (char *)pSect->m_sSectionName, pSect);

  return pSect;
}

/*!
 * \brief Delete a section from the configuration database.
 *
 * \param pConfig Configuration database.
 * \param pSect   Configuration section to delete.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigSectionDelete(Config_T *pConfig, ConfigSection_T *pSect)
{
  LOGDIAG4CALL(_TPTR(pConfig), _TPTR(pSect));

  if( pSect == NULL )
  {
    return OK;
  }

  // make sure section exist
  if( hash_lookup(pConfig->m_hashMain, pSect->m_sSectionName) == NULL )
  {
    LOGERROR("%s: [%s]: section not define",
        pConfig->m_sMainName, pSect->m_sSectionName);
    return RC_ERROR;
  }
    
  // delete all section entries
  else
  {
    hash_delete(pConfig->m_hashMain, pSect->m_sSectionName);
    return OK;
  }
}

/*!
 * \brief Get a section from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 *
 * \return
 * Returns configuration section on success.\n
 * NULL on failure.
 */
ConfigSection_T *ConfigSectionGet(Config_T *pConfig, const char *sSectionName)
{
  hnode_t *pHashNode;

  if( (pHashNode = hash_lookup(pConfig->m_hashMain, sSectionName)) != NULL )
  {
    return (ConfigSection_T *)hnode_get(pHashNode);
  }
  return NULL;
}

/*!
 * \brief Print one section to the output file stream.
 *
 * The output format is valid for input.
 *
 * \param pSect   Configuration section.
 * \param fp      Output file pointer.
 */
void ConfigSectionPrint(ConfigSection_T *pSect, FILE *fp)
{
  hscan_t          scanHash;
  hnode_t         *pHashNode;

  fprintf(fp, "[%s]\n", pSect->m_sSectionName);
  hash_scan_begin(&scanHash, pSect->m_hashSection);
  while( (pHashNode = hash_scan_next(&scanHash)) != NULL )
  {
    fprintf(fp, "%s=%s\n",
        (char *)hnode_getkey(pHashNode), (char *)hnode_get(pHashNode));
  }
}


// ...........................................................................
// Configuration Get key=value Entry Functions
// ...........................................................................

/*!
 * \brief Get a string value from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 *
 * \return Returns string value one success, NULL on failure.
 */
const char *ConfigGetStr(Config_T *pConfig, const char *sSectionName, 
                         const char *sKey)
{
  ConfigSection_T *pSect;
  hnode_t         *pHashNode;

  if( (pSect = ConfigSectionGet(pConfig, sSectionName)) == NULL )
  {
    return NULL;
  }

  else if( (pHashNode = hash_lookup(pSect->m_hashSection, sKey)) == NULL )
  {
    return NULL;
  }
  
  else
  {
    return hnode_get(pHashNode);
  }
}

/*!
 * \brief Get and convert a long value from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param pVal          Converted value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigGetLong(Config_T *pConfig, const char *sSectionName,
                                     const char *sKey, long *pVal)
{
  const char  *sVal;
  long         lVal;
  char        *sEnd;

  if( (sVal = ConfigGetStr(pConfig, sSectionName, sKey)) == NULL )
  {
    return RC_ERROR;
  }
  else if( (sVal == NULL) || (*sVal == 0) )
  {
    LOGERROR("Config %s: [%s]: value is null",
          pConfig->m_sMainName, sSectionName);
    return RC_ERROR;
  }
  else
  { 
    lVal = strtol(sVal, &sEnd, 0);

    if( *sEnd != 0 )
    {
      LOGERROR("Config %s: [%s]: %s=%s: not an integer",
          pConfig->m_sMainName, sSectionName, sKey, sVal);
      return RC_ERROR;
    }
    else
    {
      *pVal = lVal;
      return OK;
    }
  }
}

/*!
 * \brief Get and convert a double value from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param pVal          Converted value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigGetDouble(Config_T *pConfig, const char *sSectionName,
                    const char *sKey, double *pVal)
{
  const char  *sVal;
  double       fVal;
  char        *sEnd;

  if( (sVal = ConfigGetStr(pConfig, sSectionName, sKey)) == NULL )
  {
    return RC_ERROR;
  }
  else if( (sVal == NULL) || (*sVal == 0) )
  {
    LOGERROR("Config %s: [%s]: value is null",
          pConfig->m_sMainName, sSectionName);
    return RC_ERROR;
  }
  else
  { 
    fVal = strtod(sVal, &sEnd);

    if( *sEnd != 0 )
    {
      LOGERROR("Config %s: [%s]: %s=%s: not a float",
          pConfig->m_sMainName, sSectionName, sKey, sVal);
      return RC_ERROR;
    }
    else
    {
      *pVal = fVal;
      return OK;
    }
  }
}


// ...........................................................................
// Configuration Set key=value Entry Functions
// ...........................................................................

/*!
 * \brief Set or update a string value in the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param sVal          Section key value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigSetStr(Config_T *pConfig, const char *sSectionName, 
                 const char *sKey, const char *sVal)
{
  ConfigSection_T *pSect;

  LOGDIAG4CALL(_TPTR(pConfig), _TSTR(sSectionName), _TSTR(sKey), _TSTR(sVal));

  /* Can't add to an undefined section! */
  if( (pSect = ConfigSectionGet(pConfig, sSectionName)) == NULL )
  {
    LOGDIAG4("%s: [%s]: section not defined",
        pConfig->m_sMainName, sSectionName);
    return RC_ERROR;
  }
  else
  {
    return ConfigSectionAddPair(pSect, sKey, sVal);
  }
}

/*!
 * \brief Set or update a long value in the configuration database.
 *
 * The long value is converted to a string prior to adding to the database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param lVal          Section key long value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigSetLong(Config_T *pConfig, const char *sSectionName, 
                  const char *sKey, long lVal)
{
  char buf[64];

  sprintf(buf, "%ld", lVal);
  return ConfigSetStr(pConfig, sSectionName, sKey, buf);
}

/*!
 * \brief Set or update an unsigned long value in the configuration database.
 *
 * The long value is converted to a string prior to adding to the database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param ulVal         Section key unsigned long value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigSetULong(Config_T *pConfig, const char *sSectionName, 
                   const char *sKey, unsigned long ulVal)
{
  char buf[64];

  sprintf(buf, "%lu", ulVal);
  return ConfigSetStr(pConfig, sSectionName, sKey, buf);
}

/*!
 * \brief Set or update a double value in the configuration database.
 *
 * The double value is converted to a string prior to adding to the database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 * \param fVal          Section key double value.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigSetDouble(Config_T *pConfig, const char *sSectionName, 
                    const char *sKey, double fVal)
{
  char buf[64];

  sprintf(buf, "%.16f", fVal);
  return ConfigSetStr(pConfig, sSectionName, sKey, buf);
}


// ...........................................................................
// Configuration Delete Entry Functions
// ...........................................................................

/*!
 * \brief Delete entry from the configuration database.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 * \param sKey          Section key.
 *
 * \return Returns OK(0) on success, RC_ERROR(-1) on failure.
 */
int ConfigDelete(Config_T *pConfig, const char *sSectionName, const char *sKey)
{
  ConfigSection_T *pSect;

  /* Can't add to an undefined section! */
  if( (pSect = ConfigSectionGet(pConfig, sSectionName)) == NULL )
  {
    LOGDIAG4("%s: [%s]: section not defined",
        pConfig->m_sMainName, sSectionName);
    return RC_ERROR;
  }
  else if( !hash_delete(pSect->m_hashSection, (char *)sKey) )
  {
    LOGDIAG4("%s: [%s]: %s could not delete",
        pConfig->m_sMainName, sSectionName, sKey);
    return RC_ERROR;
  }
  else
  {
    return OK;
  }
}


// ...........................................................................
// Configuration Iterator Functions
// ...........................................................................

/*!
 * \brief Create a new configuration database iterator.
 *
 * The iterator will iterator over section names.
 *
 * \param pConfig Configuration database.
 *
 * \return New iterator.
 */
ConfigIter_T *ConfigDbIterNew(Config_T *pConfig)
{
  ConfigIter_T    *pIter;

  pIter = NEW(ConfigIter_T);
  hash_scan_begin(&pIter->m_scanHash, pConfig->m_hashMain);

  return pIter;
}

/*!
 * \brief Create a new configuration section iterator.
 *
 * The iterator will iterator over section keys.
 *
 * \param pConfig       Configuration database.
 * \param sSectionName  Configuration section name.
 *
 * \return New iterator.
 */
ConfigIter_T *ConfigSectionIterNew(Config_T *pConfig, const char *sSectionName)
{
  ConfigSection_T *pSect;
  ConfigIter_T    *pIter;

  if( (pSect = ConfigSectionGet(pConfig, sSectionName)) == NULL )
  {
    return NULL;
  }
  else
  {
    pIter = NEW(ConfigIter_T);
    hash_scan_begin(&pIter->m_scanHash, pSect->m_hashSection);
    return pIter;
  }
}

/*!
 * \brief Delete a configuration iterator.
 *
 * \param pIter Configuration iterator.
 */
void ConfigIterDelete(ConfigIter_T *pIter)
{
  delete(pIter);
}

/*!
 * \brief Get the next value in iteration.
 *
 * After an iterator is created, the first call will return the first vale.
 * Subsequent calls return the next values.
 *
 * If iterator was created to iterate over the section names, then the values
 * returned are section names. If the iterator was created to iterate over
 * section keys, then the values returned are keys.
 *
 * \param pIter Configuration iterator.
 *
 * \return Returns next value on success, NULL at end of iteration.
 */
const char *ConfigIterNext(ConfigIter_T *pIter)
{
  hnode_t *pHashNode;

  if( pIter == NULL )
  {
    return NULL;
  }
  else if( (pHashNode = hash_scan_next(&pIter->m_scanHash)) == NULL )
  {
    return NULL;
  }
  else
  {
    return hnode_getkey(pHashNode);
  }
}
