////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      laelaps_diag.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Diagnotics header file.
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

#ifndef _LAELAPS_DIAG_H
#define _LAELAPS_DIAG_H

#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeDesc.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeWd.h"


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// External Data
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Shared interfaces used by diagnostics
//
extern laelaps::LaeDesc   RobotDesc;
extern laelaps::LaeI2C    I2CBus;
extern laelaps::LaeI2CMux I2CMux;
extern laelaps::LaeWd     WatchDog;

//
// Diagnostic Tags
//
extern char PassTag[];
extern char WarnTag[];
extern char FailTag[];
extern char WaitTag[];
extern char YNTag[];
extern char FatalTag[];

/*!
 * \brief Simple diagnostics statistics class.
 */
class DiagStats
{
public:
  int   passCnt;    // diagnostic test pass count
  int   testCnt;    // diagnostic test count
  bool  fatal;      // diagnoistics can [not] continue

  /*!
   * \brief Constructor.
   */
  DiagStats()
  {
    zero();
  }

  /*!
   * \brief Destructor.
   */
  ~DiagStats()
  {
  }

  /*!
   * \brief Zero statistics.
   */
  void zero()
  {
    passCnt = 0;
    testCnt = 0;
    fatal   = false;
  }

  /*!
   * \brief Compound assignment operator.
   *
   * A compound assignment (does not need to be a member, but often is, to
   * modify any private members). The addition of rhs to *this takes place
   * here.
   *
   * \param rhs   Right hand side object.
   *
   * \return *this.
   */
  DiagStats &operator+=(const DiagStats &rhs)
  {
    passCnt += rhs.passCnt;
    testCnt += rhs.testCnt;
    fatal    = rhs.fatal;
    return *this;
  }
 
  /*!
   * \brief Addition operator.
   *
   * Friends defined inside class body are inline and are hidden from
   * non-ADL lookup (Argument-Dependent Lookup). Passing lhs by value helps
   * optimize chained a+b+c. Otherwise, both parameters may be const references.
   *
   * \param lhs   Left hand side lvalue.
   * \param rhs   Right hand side object.
   *
   * \return *this.
   */
  friend DiagStats operator+(DiagStats        lhs,
                             const DiagStats& rhs)
  {
     lhs += rhs; // reuse compound assignment
     return lhs;
  }
};


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Top-Level Diagnostics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern DiagStats runCpuDiagnostics();

extern DiagStats runProductDiagnostics();

extern DiagStats runMotorsDiagnostics(bool bTestMotion);

extern DiagStats runToFDiagnostics();

extern DiagStats runImuDiagnostics();

extern DiagStats runWatchDogDiagnostics();

extern DiagStats runCamDiagnostics();

extern DiagStats runBatteryDiagnostics(bool bAnyKey);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern void setTags(bool bColor);

extern void printHdr(std::string strDiag);

extern void printSubHdr(std::string strName);

extern void printTestResult(const char *sTag, const char *sFmt, ...);

extern void printSubTotals(DiagStats &stats);

extern void printTotals(DiagStats &stats);

extern void printGrandTotals(DiagStats &stats);

extern int kbhit();

#endif // _LAELAPS_DIAG_H

