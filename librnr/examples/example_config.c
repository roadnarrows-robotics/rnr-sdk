////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_config.c
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:30:34 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1577 $
 *
 * \brief Example of using the librnr config module.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2007-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/config.h"

/*!
 * \brief command execution function type
 */
typedef void (*CmdExec_T)(int, char *[]);

/*!
 * \brief command structure type
 */
typedef struct
{
  const char  *m_sCmdName;        ///< command name
  CmdExec_T    m_fnCmdExec;       ///< command execution function
  const char  *m_sCmdHelpArgs;    ///< command help arguments
  const char  *m_sCmdHelpDesc;    ///< command help description
} Cmd_T;

static Config_T *TheConfig = NULL;    ///< the configuration db

/*!
 * \brief Check argument count against minimum required.
 *
 * \param argc    Argument count.
 * \param min     Required minimum count.
 */
#define CHKARGS(argc, min)  \
  { if((argc-1) < min) \
    { \
      fprintf(stderr, "Error: %d arguments required\n", min); \
      return; \
    } \
  }

/*!
 * \brief Check that the configuration db exists.
 */
#define CHKCFG() \
  { \
    if(TheConfig == NULL) \
    { \
      fprintf(stderr, "Error: configuration db does not exist\n"); \
      return; \
    } \
  }

/*!
 * \brief Check that the configuration db does not exist.
 */
#define CHKNOCFG() \
  { \
    if(TheConfig != NULL) \
    { \
      fprintf(stderr, "Error: configuration %s exists - delete first\n", \
        ConfigDbGetName(TheConfig)); \
      return; \
    } \
  }

// forward declaration
static void CmdHelp(int, char *[]);

/*!
 * \brief Quit this example program.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdQuit(int argc, char *argv[])
{
  exit(0);
}

/*!
 * \brief Set diagnotics logging level.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdLog(int argc, char *argv[])
{
  int   n;

  CHKARGS(argc, 1);

  n = atoi(argv[1]);

  LOG_SET_THRESHOLD(n);
}

/*!
 * \brief Delete configuration db.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdDbDelete(int argc, char *argv[])
{
  CHKCFG();

  ConfigDbDelete(TheConfig);
  TheConfig = NULL;
}

/*!
 * \brief Create new, empty configuration db.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdDbNew(int argc, char *argv[])
{
  CHKNOCFG();

  TheConfig = ConfigDbNew(NULL);
}

/*!
 * \brief Print out entire configuration db.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdDbPrint(int argc, char *argv[])
{
  CHKCFG();

  ConfigDbPrint(TheConfig, stdout);
}

/*!
 * \brief Create new configuration db from file.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdDbRead(int argc, char *argv[])
{
  CHKARGS(argc, 1);
  CHKNOCFG();

  TheConfig = ConfigDbRead(argv[1]);
}

/*!
 * \brief Delete configuration key.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdDelete(int argc, char *argv[])
{
  CHKARGS(argc, 2);
  CHKCFG();

  if( ConfigDelete(TheConfig, argv[1], argv[2]) != OK )
  {
    printf("Error: failed to delete\n");
  }
  else
  {
    printf("%s: [%s]: %s: deleted\n", ConfigDbGetName(TheConfig),
      argv[1], argv[2]);
  }
}

/*!
 * \brief Find configuration key=values matching substring.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdFind(int argc, char *argv[])
{
  char          *sPat;
  size_t         lenPat;
  ConfigIter_T  *pIterSects;
  ConfigIter_T  *pIterKeys;
  const char    *sSectionName;
  const char    *sKey;
  const char    *sVal;

  CHKARGS(argc, 1);
  CHKCFG();

  sPat    = argv[1];
  lenPat  = strlen(sPat);

  pIterSects = ConfigDbIterNew(TheConfig);

  while( (sSectionName = ConfigIterNext(pIterSects)) != NULL )
  {
    pIterKeys = ConfigSectionIterNew(TheConfig, sSectionName);

    while( (sKey = ConfigIterNext(pIterKeys)) != NULL )
    {
      if( !strncmp(sPat, sKey, lenPat) )
      {
        sVal = ConfigGetStr(TheConfig, sSectionName, sKey);
        printf("%s: [%s]: %s=%s\n",
          ConfigDbGetName(TheConfig), sSectionName, sKey, sVal);
      }
    }

    ConfigIterDelete(pIterKeys);
  }

  ConfigIterDelete(pIterSects);
}

/*!
 * \brief Get configuration value string converting to double.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdGetd(int argc, char *argv[])
{
  double  fVal;

  CHKARGS(argc, 2);
  CHKCFG();

  if( ConfigGetDouble(TheConfig, argv[1], argv[2], &fVal) != OK )
  {
    printf("Error: failed to get/convert\n");
  }
  else
  {
    printf("%s: [%s]: %s=%f\n", ConfigDbGetName(TheConfig),
      argv[1], argv[2], fVal);
  }
}

/*!
 * \brief Get configuration value string converting to int.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdGeti(int argc, char *argv[])
{
  int  iVal;

  CHKARGS(argc, 2);
  CHKCFG();

  if( ConfigGetInt(TheConfig, argv[1], argv[2], &iVal) != OK )
  {
    printf("Error: failed to get/convert\n");
  }
  else
  {
    printf("%s: [%s]: %s=%d\n", ConfigDbGetName(TheConfig),
      argv[1], argv[2], iVal);
  }
}

/*!
 * \brief Get configuration value string.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdGets(int argc, char *argv[])
{
  const char  *sVal;

  CHKARGS(argc, 2);
  CHKCFG();

  if( (sVal = ConfigGetStr(TheConfig, argv[1], argv[2])) == NULL )
  {
    printf("Error: failed to get\n");
  }
  else
  {
    printf("%s: [%s]: %s=%s\n", ConfigDbGetName(TheConfig),
      argv[1], argv[2], sVal);
  }
}

/*!
 * \brief Delete entier configuration db section.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdSectDelete(int argc, char *argv[])
{
  ConfigSection_T *pSect;

  CHKARGS(argc, 1);
  CHKCFG();

  if( (pSect = ConfigSectionGet(TheConfig, argv[1])) == NULL )
  {
    printf("Error: failed to delete section\n");
  }
  else
  {
    ConfigSectionDelete(TheConfig, pSect);
    printf("%s: [%s] deleted\n", ConfigDbGetName(TheConfig), argv[1]);
  }
}

/*!
 * \brief Create new, empty configuration db section.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdSectNew(int argc, char *argv[])
{
  CHKARGS(argc, 1);
  CHKCFG();

  if( ConfigSectionNew(TheConfig, argv[1]) == NULL )
  {
    printf("Error: failed to add section\n");
  }
  else
  {
    printf("%s: [%s] added\n", ConfigDbGetName(TheConfig), argv[1]);
  }
}

/*!
 * \brief Set or create key=\<double\> entry in section.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdSetd(int argc, char *argv[])
{
  double  fVal;

  CHKARGS(argc, 3);
  CHKCFG();

  fVal = atof(argv[3]);

  if( ConfigSetDouble(TheConfig, argv[1], argv[2], fVal) != OK )
  {
    printf("Error: failed to set\n");
  }
  else
  {
    printf("%s: [%s]: %s=%f\n", ConfigDbGetName(TheConfig),
        argv[1], argv[2], fVal);
  }
}

/*!
 * \brief Set or create key=\<int\> entry in section.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdSeti(int argc, char *argv[])
{
  int  iVal;

  CHKARGS(argc, 3);
  CHKCFG();

  iVal = atoi(argv[3]);

  if( ConfigSetInt(TheConfig, argv[1], argv[2], iVal) != OK )
  {
    printf("Error: failed to set\n");
  }
  else
  {
    printf("%s: [%s]: %s=%d\n", ConfigDbGetName(TheConfig),
        argv[1], argv[2], iVal);
  }
}

/*!
 * \brief Set or create key=\<string\> entry in section.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdSets(int argc, char *argv[])
{
  CHKARGS(argc, 3);
  CHKCFG();

  if( ConfigSetStr(TheConfig, argv[1], argv[2], argv[3]) != OK )
  {
    printf("Error: failed to set\n");
  }
  else
  {
    printf("%s: [%s]: %s=%s\n", ConfigDbGetName(TheConfig),
      argv[1], argv[2], argv[3]);
  }
}

/*!
 * \brief The command set.
 */
static Cmd_T CmdTbl[] =
{
  { "dbdelete", CmdDbDelete, "", 
    "delete existing configuration db"
  },
  { "dbnew", CmdDbNew, "",
    "create new, empty configuration db"
  },
  { "dbprint", CmdDbPrint, "",
    "print out configuration db"
  },
  { "dbread", CmdDbRead, "<filename>",
    "read configuration db from file"
  },

  { "delete", CmdDelete, "<section> <key>",
    "delete entry from section"
  },

  { "find", CmdFind, "<pattern>",
    "find all entries that partially match pattern"
  },

  { "getd", CmdGetd, "<section> <key>",
    "get key=<double> in section"
  },
  { "geti", CmdGeti, "<section> <key>",
    "get key=<int> in section"
  },
  { "gets", CmdGets, "<section> <key>",
    "get key=<string> in section"
  },

  { "help", CmdHelp, "",
    "print list of commands"
  },
  { "log", CmdLog, "<level>",
    "set logging level [0-4]"
  },
  { "quit", CmdQuit, "",
    "quit program"
  },

  { "sectdelete", CmdSectDelete, "<section>",
    "delete section in configuration db"
  },
  { "sectnew", CmdSectNew, "<section>",
    "create new, empty section in configuration db"
  },

  { "setd", CmdSetd, "<section> <key> <double>",
    "set key=<double> in section"
  },
  { "seti", CmdSeti, "<section> <key> <int>",
    "set key=<int> in section"
  },
  { "sets", CmdSets, "<section> <key> <string>",
    "set key=<string> in section"
  },

  {NULL, }
};

/*!
 * \brief Print command help.
 *
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
static void CmdHelp(int argc, char *argv[])
{
  Cmd_T *pCmd;
  int    n;

  for(pCmd=CmdTbl; pCmd->m_sCmdName!=NULL; pCmd++)
  {
    n = printf("%s %s", pCmd->m_sCmdName, pCmd->m_sCmdHelpArgs);
    printf("%*s - %s\n", 30-n, "", pCmd->m_sCmdHelpDesc);
  }
}

/*!
 * \brief Tokenize input.
 *
 * \param string      Input string
 * \param [out] tokv  Array of tokens.
 * \param tokmax      Maximum number of tokens.
 *
 * \return Number of tokens.
 */
static int Tokenize(char *string, char *tokv[], size_t tokmax)
{
  int       tokc = 0;

  while( tokc < tokmax )
  {
    while(*string && isspace((int)*string))
    {
      string++;
    }
    if(!*string)
    {
      break;
    }
    tokv[tokc] = string;
    while(*string && !isspace((int)*string))
    {
      string++;
    }
    tokc++;
    if( *string == 0 )
    {
      break;
    }
    *string++ = 0;
  }

  return tokc;
}

/*!
 * \brief Match command.
 *
 * \param sPattern  (Partial) command string.
 * \param tblCmds   Table of commands.
 */
static Cmd_T *MatchCmd(const char *sPattern, Cmd_T tblCmds[])
{
  Cmd_T         *p;
  Cmd_T         *pCmd = NULL;
  size_t         n;
  bool_t         bIsUnique = true;
  int            cnt;
  
  n = strlen(sPattern);

  // find command
  for(p=tblCmds; p->m_sCmdName!=NULL; p++)
  {
    if( !strncmp(sPattern, p->m_sCmdName, n) )
    {
      pCmd = p;
      break;
    }
  }

  // no matching command found
  if( pCmd == NULL )
  {
    fprintf(stderr, "Error: %s: Command not found\n", sPattern);
    return NULL;
  }

  // verify uniqueness
  for(p=tblCmds, cnt=0; p->m_sCmdName!=NULL; p++)
  {
    if( (p != pCmd) && !strncmp(sPattern, p->m_sCmdName, n) )
    {
      cnt++;

      // first non-unique
      if( bIsUnique )
      {
        fprintf(stderr, "Error: %s: Command not unique: Matches %s %s ",
            sPattern, pCmd->m_sCmdName, p->m_sCmdName);
        bIsUnique = false;
      }
      // subsequent non-unique
      else if( cnt < 3 )
      {
        fprintf(stderr, "%s ", p->m_sCmdName);
      }
      // too many to print
      else
      {
        fprintf(stderr, "...");
        break;
      }
    }
  }

  if( !bIsUnique )
  {
    fprintf(stderr, "\n");
    return NULL;
  }

  return pCmd;
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  char      input[256];
  int       tokc;
  char     *tokv[16];
  Cmd_T    *pCmd;

  printf("  Configuration Example\n");
  printf("(enter 'help' for list of commands; "
         "partial command matching supported)\n\n");

  // process user commands
  for(;;)
  {
    printf("config> ");

    // read user input
    if( !fgets(input, (int)sizeof(input), stdin) )
    {
      break;
    }

    // tokenize input
    tokc = Tokenize(input, tokv, arraysize(tokv));

    if( tokc <= 0 )
    {
      continue;
    }

    // find command
    pCmd = MatchCmd(tokv[0], CmdTbl);

    if( pCmd != NULL )
    {
      // execute command
      pCmd->m_fnCmdExec(tokc, tokv);
    }
  }

  return 0;
}
