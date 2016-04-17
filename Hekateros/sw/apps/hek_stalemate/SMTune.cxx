////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMTune.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-01-21 10:11:12 -0700 (Sat, 21 Jan 2012) $
 * $Rev: 1700 $
 *
 * \brief StaleMate tunaables.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
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
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <stdarg.h>
#include <libgen.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// chess tuning parameters
double  TuneDiffThresholdRed    = 45.0;      ///< red chan. diff threshold
double  TuneBoardThresholdRed   = 110000.0; ///< red chan. board threshld
double  TuneDiffThresholdBlue   = 45.0;      ///< blue chan. diff threshld
double  TuneBoardThresholdBlue  = 100000.0; ///< blue chan. board threshld
int     TuneFramesWithChangeCnt = 180;      ///< numof conseq. diff frames
double  TuneSquareThresholdRed  = 10000.0;  ///< red square diff threshld
double  TuneSquareThresholdBlue = 10000.0;  ///< blue square diff threshld
double  TuneEpsilon             = 15.0;     ///< max dist to goal planning
double  TuneChessSquareDim      = 59.0;     ///< chess board square size
double  TuneChessDEBaseX        = 350.0;    ///< chess board DE line dist
double  TuneChessDEBaseY        = -5.0;      ///< chess board DE line dist
const char *TuneChessEngine     = "/usr/games/gnuchess";
                                            ///< default chess engine
const char *TuneOpenRavePython  = "/usr/bin/openrave.py";
                                            ///< default openrave interface

