////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      StaleMateTune.h
//
/*! \file
 *
 * $LastChangedDate: 2012-06-05 15:17:26 -0600 (Tue, 05 Jun 2012) $
 * $Rev: 2028 $
 *
 * \brief Tuned parameters and constants.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _STALEMATETUNE_H
#define _STALEMATETUNE_H

#include "opencv/cv.h"
#include "opencv/highgui.h"

// .............................................................................
// FEATURES AND TESTS
// .............................................................................

// support languages
#define STALEMATE_LANG_EN   0   ///< English (default)
#define STALEMATE_LANG_ES   1   ///< Spanish
#define STALEMATE_LANG_FR   2   ///< French
#define STALEMATE_LANG_DE   3   ///< German
#define STALEMATE_LANG_AV   99  ///< Avaric :-)

#define STALEMATE_LANG      STALEMATE_LANG_EN    ///< this build's language

// .............................................................................
// TUNING
// .............................................................................

// time durations, timeouts, etc
const int       T_MsgDft  = 2000;   ///< default status message show time (ms)
const int       T_MsgInf  = 0;      ///< status message infinite show time

// chess tuning parameters
extern double  TuneDiffThresholdRed;
extern double  TuneBoardThresholdRed;
extern double  TuneDiffThresholdBlue;
extern double  TuneBoardThresholdBlue;
extern int     TuneFramesWithChangeCnt;
extern double  TuneSquareThresholdRed;
extern double  TuneSquareThresholdBlue;
extern double  TuneEpsilon;
extern double  TuneChessSquareDim;
extern double  TuneChessDEBaseX;
extern double  TuneChessDEBaseY;
extern const char *TuneChessEngine;
extern const char *TuneOpenRavePython;


// .............................................................................
// GUI COLORS AND STYLES
// .............................................................................

// widget look and feel
const char* const GuiTitleStrColor  = "#aa0000";            ///< title color
const char* const GuiTitleStrFont   = "Arial 20";           ///< title font
const char* const GuiLabelStrColor  = "#fed700";            ///< label color
const char* const GuiLabelStrFont   = "Arial 15";           ///< label font
const char* const GuiLabel2StrColor = "#fed700";            ///< label2 color
const char* const GuiLabel2StrFont  = "Arial Bold 12";      ///< label2 font
const char* const GuiLabel3StrColor = "#fed700";            ///< label3 color
const char* const GuiLabel3StrFont  = "Arial 10";           ///< label3 font


// .............................................................................
// MESSAGES
// .............................................................................

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// spanish
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#if STALEMATE_LANG == STALEMATE_LANG_ES

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// french
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#elif STALEMATE_LANG == STALEMATE_LANG_FR

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// german
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#elif STALEMATE_LANG == STALEMATE_LANG_DE

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// avaric
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#elif STALEMATE_LANG == STALEMATE_LANG_AV

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// english is the default
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#else

/*! working status */
const char* const MsgWorking = "processing...";

/*! initial prompt for the user to take an image */
const char* const MsgTakeAPic = "Take an image...";

const char* const MsgTakeAPicRedo = "Retake an image...";

#endif // STALEMATE_LANG_EN

#endif // _STALEMATETUNE_H
