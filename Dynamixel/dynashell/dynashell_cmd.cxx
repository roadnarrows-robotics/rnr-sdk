////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_cmd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell Command Base and Derived Core Classes.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_util.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaShellCmd Class
// -----------------------------------------------------------------------------

/*!
 * \brief Print command help with the description aligned at the given
 * indentation.
 *
 * \param indent  Column indentation.
 * \param width   Column width.
 */
void DynaShellCmd::PrintHelp(int indent, int width)
{
  int           col;        // current column position

  // print brief
  PrintDelim(width, '-');
  col = printf("%s - ", m_sPubName);
  PrintBlock(col, indent, width, m_sCmdHelpBrief);

  // print synopses
  if( (m_sCmdHelpArgs != NULL) && (*m_sCmdHelpArgs != 0) )
  {
    printf("\n");
    PrintSynopses(indent, width);
  }


  // print description
  if( (m_sCmdHelpDesc != NULL) && (*m_sCmdHelpDesc != 0) )
  {
    printf("\n");
    PrintBlock(0, indent, width, m_sCmdHelpDesc);
  }
}

/*!
 * \brief Print synsopses.
 *
 * \param indent  Column indentation.
 * \param width   Column width.
 */
void DynaShellCmd::PrintSynopses(int indent, int width)
{
  int           col;        // current column position
  char          buf[256];   // working buffer
  const char   *s, *t;      // working pointers
  size_t        len;        // working length
  bool          bFirst;     // first synopsis
  bool          bLast ;     // last synopsis

  if( (m_sCmdHelpArgs != NULL) && (*m_sCmdHelpArgs != 0) )
  {
    for(s=m_sCmdHelpArgs, bFirst=true, bLast=false; !bLast; )
    {
      t = strchr(s, '\n');

      if( t != NULL )
      {
        len = (size_t)(t-s);
      }
      else
      {
        len = strlen(s);
        bLast = true;
      }
      if( len >= sizeof(buf) )
      {
        len = sizeof(buf)-1;
      }

      strncpy(buf, s, len);
      buf[len] = 0;

      if( bFirst )
      {
        col = printf("%*sUsage: %s ", indent, "", m_sPubName);
        bFirst = false;
      }
      else
      {
        col = printf("%*s       %s ", indent, "", m_sPubName);
      }

      PrintBlock(col, col, width, buf);

      if( t != NULL )
      {
        s = t + 1;
      }
    }
  }
}

/*!
 * \brief Print a block of indented text of width.
 *
 * \param col     Current column position.
 * \param indent  Column indentation from 0.
 * \param width   Column width from 0;
 * \param sText   Text to print.
 */
void DynaShellCmd::PrintBlock(int col, int indent, int width, const char *sText)
{
  int         n;      // word length
  const char *s, *t;  // working pointers

  if( col >= width )
  {
    printf("\n");
    col = 0;
  }

  for(s=sText; *s != 0; )
  {
    // find end of word
    t = eow(s);

    // end of string
    if( t == NULL )
    {
      n = (int)strlen(s);
      t = s + n;
    }

    // got a word
    else
    {
      n = (int)(t - s);
    }

    // will exceed line limit, go to next line
    if( col+n >= width )
    {
      printf("\n");
      col = printf("%*s", indent, "");
    }

    // print help word
    if( n > 0 )
    {
      col += printf("%.*s", n, s);
    }
    //col += printf("|");

    switch(*t)
    {
      case ' ':
      case '\t':
      case '\v':
        col += printf(" ");
        s = ++t;
        break;
      case '\n':
      case '\r':
        printf("\n");
        col = printf("%*s", indent, "");
        s = ++t;
        break;
      case 0:
      default:
        s = t;
    }
  }

  printf("\n");
}

void DynaShellCmd::PrintDelim(int width, const char cDelim)
{
  while( width-- > 0 )
  {
    printf("%c", cDelim);
  }
  printf("\n");
}

/*!
 * \brief Find end of word.
 *
 * A word is define as a set of non-whitespace characters delimitedd by white
 * space.
 *
 * \param s   Null-terminated string to search.
 *
 * \return Returns a pointer to the first trailing white space character or
 * NULL if no white space found.
 */
char *DynaShellCmd::eow(const char *s)
{
  char  *t;

  if( (s == NULL) || (*s == 0) )
  {
    return NULL;
  }

  for(t=(char *)s; *t; ++t)
  {
    if( isspace(*t) )
    {
      break;
    }
  }

  return *t? t: NULL;
}


// -----------------------------------------------------------------------------
// DynaShellCmdChainIn Class
// -----------------------------------------------------------------------------

/*!
 * \brief Execute 'read-like' command on servos.
 *
 * \param shell   Dynamixel shell.
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
void DynaShellCmdChainIn::Exec(DynaShell &shell, int argc, char *argv[])
{
  int         nServoId;
  int         iter;
  DynaServo  *pServo;

  TRY( ChkArgCnt(shell, argc) );

  TRY( ChkComm(shell) );
  TRY( ChkChainNotEmpty(shell) );

  if( !strcmp(argv[0], "chain") )
  {
    TRY( ChkArgCntEQ(shell, argc, 1) );

    if( m_bMasterOpOnly )
    {
      for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNextMaster(&iter))
      {
        pServo = shell.m_pDynaChain->GetServo(nServoId);
        doExec(shell, pServo);
      }
    }
    else
    {
      for(nServoId = shell.m_pDynaChain->IterStart(&iter);
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter))
      {
        pServo = shell.m_pDynaChain->GetServo(nServoId);
        doExec(shell, pServo);
      }
    }
  }

  else
  {
    for(iter=0; iter<argc; ++iter)
    {
      TRY( ToInt(shell, argv[iter], &nServoId) );
      TRY( ChkChainHasServo(shell, nServoId) );
      TRY( !m_bMasterOpOnly || ChkChainIsMasterServo(shell, nServoId) );

      pServo = shell.m_pDynaChain->GetServo(nServoId);
      doExec(shell, pServo);
    }
  }
}

/*!
 * \brief Command tab completion generator.
 *
 * Completes <servo_id> | chain [servo_id [servo_id ...]]
 *
 * \param shell     Dynamixel shell.
 * \param sText     Partial text string to complete.
 * \param uTextLen  Length of text.
 * \param nState    Generator state. If FIRST, then initialize any statics.
 * \param sContext  Generator context (i.e. canonical command path).
 *
 * \return
 * If a first/next match is made, return allocated completed match.\n
 * Otherwise return NULL.
 */
char *DynaShellCmdChainIn::TabCompletion(DynaShell   &shell,
                                         const char  *sText,
                                         size_t       uTextLen,
                                         int          nState,
                                         const char  *sContext)
{
  char  buf[16];

  if( (shell.m_pDynaChain == NULL) ||
      (shell.m_pDynaChain->GetNumberInChain() == 0) )
  {
    return NULL;
  }

  if( m_bMasterOpOnly && shell.m_pDynaChain->GetNumberOfMastersInChain() == 0 )
  {
    return NULL;
  }

  //
  // New command to complete - initialize.
  //
  if( nState == ReadLine::FIRST )
  {
    if( m_bMasterOpOnly )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStartMaster(&m_nTabIter);
    }
    else
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
    }

    if( ReadLine::wc(sContext) == ReadLine::wc(m_sPubName) )
    {
      m_bTabIncChain = true;
    }
    else
    {
      m_bTabIncChain = false;
    }
  }

  while( m_nTabServoId != DYNA_ID_NONE )
  {
    snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
    buf[sizeof(buf)-1] = 0;

    if( m_bMasterOpOnly )
    {
      m_nTabServoId = shell.m_pDynaChain->IterNextMaster(&m_nTabIter);
    }
    else
    {
      m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
    }

    if( !strncmp(buf, sText, uTextLen) )
    {
      return ReadLine::dupstr(buf);
    }
  }

  if( m_bTabIncChain )
  {
    m_bTabIncChain = false;
    if( !strncmp("chain", sText, uTextLen) )
    {
      return ReadLine::dupstr("chain");
    }
  }

  // no more matches
  return NULL;
}


// -----------------------------------------------------------------------------
// DynaShellCmdChainOut Class
// -----------------------------------------------------------------------------

/*!
 * \brief Execute 'write-like' command on servos.
 *
 * \param shell   Dynamixel shell.
 * \param argc    Command argument count.
 * \param argv    Array of arguments.
 */
void DynaShellCmdChainOut::Exec(DynaShell &shell, int argc, char *argv[])
{
  uint_t      uNumPairs;
  ExecTup_T   tup[DYNA_ID_NUMOF];
  int         nServoId;
  int         nVal;
  int         iter;
  int         cnt;

  TRY( ChkArgCnt(shell, argc) );

  if( (argc % 2) != 0 )
  {
    shell.Error("Unmatched servo_id,value pairs.");
    return;
  }

  uNumPairs = (uint_t)(argc / 2);

  if( uNumPairs > DYNA_ID_NUMOF )
  {
    shell.Error("%u: Too many servo_id,value pairs.", uNumPairs);
    return;
  }

  TRY( ChkComm(shell) );
  TRY( ChkChainNotEmpty(shell) );

  // full chain of servos
  if( !strcmp(argv[0], "chain") )
  {
    TRY( ChkArgCntEQ(shell, argc, 2) );
    TRY( ArgToInt(shell, argv[1], &nVal) );

    if( m_bMasterOpOnly )
    {
      for(nServoId = shell.m_pDynaChain->IterStartMaster(&iter), cnt=0;
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNextMaster(&iter), ++cnt)
      {
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nVal     = nVal;
      }
    }
    else
    {
      for(nServoId = shell.m_pDynaChain->IterStart(&iter), cnt=0;
          nServoId != DYNA_ID_NONE;
          nServoId = shell.m_pDynaChain->IterNext(&iter), ++cnt)
      {
        tup[cnt].m_nServoId = nServoId;
        tup[cnt].m_nVal     = nVal;
      }
    }

    uNumPairs = (uint_t)cnt;
  }

  // list of servos
  else
  {
    for(iter=0, cnt=0; iter<argc; iter+=2, ++cnt)
    {
      TRY( ToInt(shell, argv[iter], &nServoId) );
      TRY( ArgToInt(shell, argv[iter+1], &nVal) );
      TRY( ChkChainHasServo(shell, nServoId) );
      TRY( !m_bMasterOpOnly || ChkChainIsMasterServo(shell, nServoId) );

      tup[cnt].m_nServoId = nServoId;
      tup[cnt].m_nVal     = nVal;
    } 
  }

  doExec(shell, tup, uNumPairs);
}

/*!
 * \brief Command tab completion generator.
 *
 * Completes <servo_id> | chain NULL [servo_id NULL [servo_id NULL ...]]
 *
 * \param shell     Dynamixel shell.
 * \param sText     Partial text string to complete.
 * \param uTextLen  Length of text.
 * \param nState    Generator state. If FIRST, then initialize any statics.
 * \param sContext  Generator context (i.e. canonical command path).
 *
 * \return
 * If a first/next match is made, return allocated completed match.\n
 * Otherwise return NULL.
 */
char *DynaShellCmdChainOut::TabCompletion(DynaShell   &shell,
                                          const char  *sText,
                                          size_t       uTextLen,
                                          int          nState,
                                          const char  *sContext)
{
  char  buf[16];
  int   nArgNum;

  if( (shell.m_pDynaChain == NULL) ||
      (shell.m_pDynaChain->GetNumberInChain() == 0) )
  {
    return NULL;
  }

  else if( m_bMasterOpOnly && 
          (shell.m_pDynaChain->GetNumberOfMastersInChain() == 0) )
  {
    return NULL;
  }

  // argument number of already (expanded) arguments
  nArgNum = ReadLine::wc(sContext) - ReadLine::wc(m_sPubName);

  // every odd argument is the servo value, which cannot be expanded
  if( nArgNum & 0x01 )
  {
    return NULL;
  }

  //
  // New command to complete - initialize.
  //
  if( nState == ReadLine::FIRST )
  {
    if( m_bMasterOpOnly )
    {
      m_nTabServoId = shell.m_pDynaChain->IterStartMaster(&m_nTabIter);
    }
    else
    {
      m_nTabServoId = shell.m_pDynaChain->IterStart(&m_nTabIter);
    }

    if( nArgNum == 0 )
    {
      m_bTabIncChain = true;
    }
    else
    {
      m_bTabIncChain = false;
    }
  }

  while( m_nTabServoId != DYNA_ID_NONE )
  {
    snprintf(buf, sizeof(buf), "%d", m_nTabServoId);
    buf[sizeof(buf)-1] = 0;

    if( m_bMasterOpOnly )
    {
      m_nTabServoId = shell.m_pDynaChain->IterNextMaster(&m_nTabIter);
    }
    else
    {
      m_nTabServoId = shell.m_pDynaChain->IterNext(&m_nTabIter);
    }

    if( !strncmp(buf, sText, uTextLen) )
    {
      return ReadLine::dupstr(buf);
    }
  }

  if( m_bTabIncChain )
  {
    m_bTabIncChain = false;
    if( !strncmp("chain", sText, uTextLen) )
    {
      return ReadLine::dupstr("chain");
    }
  }

  // no more matches
  return NULL;
}
