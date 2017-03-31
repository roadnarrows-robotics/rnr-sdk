////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      laelaps_utils.cxx
//
/*! \file
 *
 * \brief Diagnostic utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/color.h"

#include "laelaps_diag.h"

using namespace std;

//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

//
// Tag colors
//
#define COLOR_PASS  ANSI_CSI ANSI_SGR_FG_COLOR_GREEN 
#define COLOR_WARN  ANSI_CSI ANSI_SGR_FG_COLOR_YELLOW 
#define COLOR_FAIL  ANSI_CSI ANSI_SGR_FG_COLOR_RED 
#define COLOR_YN    ANSI_CSI ANSI_SGR_FG_COLOR_BLUE 
#define COLOR_FATAL ANSI_CSI ANSI_SGR_FG_COLOR_LIGHT_RED 
#define COLOR_OFF   ANSI_COLOR_RESET

//
// Separators
//
static const char *DiagSep = \
"-----------------------------------------------------------------------------";

static const char *SubHdrSep = \
". . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .";

static const char *SubSumTag = "  ...";
static const char *TotSumTag = "...";

static int getch()
{
  struct termios old = {0};
  char           c = 0;

  if( tcgetattr(0, &old) < 0 )
  {
    return EOF;
  }

  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;

  if( tcsetattr(0, TCSANOW, &old) < 0 )
  {
    return EOF;
  }

  if( read(0, &c, 1) < 0 )
  {
    return EOF;
  }

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;

  if( tcsetattr(0, TCSADRAIN, &old) < 0 )
  {
  }

  return c;
}

//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

int kbhit()
{
  int   c;

  while( (c = getch()) == 0 );

  return c;
}

//
// Diagnostic tags
//
const int MaxTagSize = 32;

//
// Diagnostic Tags
//
char PassTag[MaxTagSize];
char WarnTag[MaxTagSize];
char FailTag[MaxTagSize];
char WaitTag[MaxTagSize];
char YNTag[MaxTagSize];
char FatalTag[MaxTagSize];

void setTags(bool bColor)
{
  if( bColor )
  {
    sprintf(PassTag,  "[" COLOR_PASS "PASS" COLOR_OFF "] ");
    sprintf(WarnTag,  "[" COLOR_WARN "WARN" COLOR_OFF "] ");
    sprintf(FailTag,  "[" COLOR_FAIL "FAIL" COLOR_OFF "] ");
    sprintf(WaitTag,  "       ");
    sprintf(YNTag,    "[" COLOR_YN "y/n" COLOR_OFF "]  ");
    sprintf(FatalTag, "[" COLOR_FATAL "FATAL" COLOR_OFF"] ");
  }
  else
  {
    sprintf(PassTag,  "[PASS] ");
    sprintf(WarnTag,  "[WARN] ");
    sprintf(FailTag,  "[FAIL] ");
    sprintf(WaitTag,  "       ");
    sprintf(YNTag,    "[y/n]  ");
    sprintf(FatalTag, "[FATAL]");
  }
}

void printHdr(string strDiag)
{
  printf("%s\n", DiagSep);
  printf("%s\n", strDiag.c_str());
  printf("%s\n\n", DiagSep);
}

void printSubHdr(string strName)
{
  //printf("%s\n", SubHdrSep);
  printf("+ + %s + +\n", strName.c_str());
}

void printTestResult(const char *sTag, const char *sFmt, ...)
{
  va_list ap;

  va_start(ap, sFmt);
  printf("%s ", sTag);
  vprintf(sFmt, ap);
  printf("\n");
  va_end(ap);
  fflush(stdout);
}

void printSubTotals(DiagStats &stats)
{
  printf("%s %d/%d passed.\n\n", SubSumTag, stats.passCnt, stats.testCnt);
}

void printTotals(DiagStats &stats)
{
  printf("%s %d/%d diagnostics passed.\n\n",
      TotSumTag, stats.passCnt, stats.testCnt);
}

void printGrandTotals(DiagStats &stats)
{
  printf("%s\n", DiagSep);
  printf("%s\n", DiagSep);
  printf("Gran Total: %d/%d diagnostics passed.\n",
      stats.passCnt, stats.testCnt);
  printf("%s\n", DiagSep);
  printf("%s\n", DiagSep);
}

/*!
 * \}
 */
