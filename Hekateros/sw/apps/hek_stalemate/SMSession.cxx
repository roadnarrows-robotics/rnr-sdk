////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      WMSession.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-18 10:01:05 -0600 (Mon, 18 Jun 2012) $
 * $Rev: 2056 $
 *
 * \brief Workflow data class.
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

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;



//------------------------------------------------------------------------------
// StaleMateSession Class
//------------------------------------------------------------------------------

/*!
 * \brief RNMP Workflow session data constructor.
 *
 * \note Keep light weight to be globally declared.
 */
StaleMateSession::StaleMateSession()
{
  m_eCurState               = StaleMateStateStart;
  m_ePrevState              = StaleMateStateStart;

  m_gui.pWin                = NULL;
  m_gui.pMenu               = NULL;
  m_gui.wIconicChessBoard   = NULL;
  m_gui.wBlackLabel         = NULL;
  m_gui.wWhiteLabel         = NULL;
  m_gui.wBufHistory         = NULL;
  m_gui.wChainState         = NULL;

  m_vid.sVidDevName         = NULL;
  m_vid.uVidDevMinor        = 0;
  m_vid.sizeVideo.width     = 0;
  m_vid.sizeVideo.height    = 0;
  m_vid.pVidCapture         = NULL;
  m_vid.pImgFrame           = NULL;
  m_vid.pVidToGuiTrans      = NULL;
  m_vid.pImgDisplay0        = NULL;
  m_vid.uImgIndex0          = -1;
  m_vid.pImgDisplay1        = NULL;
  m_vid.uImgIndex1          = -1;

  m_hek.bUseArm             = true;
  m_hek.bIsSafe             = false;
  m_hek.sHekDevName         = NULL;
  m_hek.nHekBaudRate        = 1000000;
  m_hek.bUseOpenRave        = false;
  m_hek.pDynaComm           = NULL;
  m_hek.pDynaChain          = NULL;
  m_hek.pDynaBgThread       = NULL;
  // m_hek.motionPlanner.initRobotHek90();
  m_hek.motionPlanner.initRobotHek91();
  m_hek.bHomeDefined        = false;
  m_hek.bParkDefined        = false;
  m_hek.bAtHomePos          = false;
  m_hek.bIsParked           = false;

  m_calib.bCalibrated       = false;
  m_calib.ptDEBottom.x      = TuneChessDEBaseX;
  m_calib.ptDEBottom.y      = TuneChessDEBaseY;
  m_calib.pImgEmptyRed      = NULL;
  m_calib.pImgEmptyBlue     = NULL;

  m_game.bUseChessEngine    = true;
  m_game.nChessBoardDim     = 8;
  m_game.bHekHasWhite       = false;
  m_game.eGameState         = ChessGameStateNoGame;
  m_game.pImgPrevRed        = NULL;
  m_game.fDiffThRed         = TuneDiffThresholdRed;
  m_game.fBoardThRed        = TuneBoardThresholdRed;
  m_game.pImgPrevBlue       = NULL;
  m_game.fDiffThRed         = TuneDiffThresholdBlue;
  m_game.fBoardThBlue       = TuneBoardThresholdBlue;
  m_game.uSpikedFrameCnt    = 0;
}

StaleMateSession::~StaleMateSession()
{
  if( m_hek.pDynaBgThread != NULL )
  {
    m_hek.pDynaBgThread->Stop();
    m_hek.pDynaBgThread->UnregisterAgent();
    delete m_hek.pDynaBgThread;
  }

  if( m_hek.pDynaChain != NULL )
  {
    m_hek.pDynaChain->EStop();
    delete m_hek.pDynaChain;
  }

  if( m_hek.pDynaComm != NULL )
  {
      delete m_hek.pDynaComm;
  }

  if( m_hek.sHekDevName != NULL )
  {
    delete[] m_hek.sHekDevName;
  }

  if( m_vid.sVidDevName != NULL )
  {
    delete[] m_vid.sVidDevName;
  }

  StaleMateLiveFeedCaptureStop(*this);

  if( m_vid.pImgDisplay0 != NULL )
  {
    cvReleaseImage(&m_vid.pImgDisplay0);
  }

  if( m_vid.pImgDisplay1 != NULL )
  {
    cvReleaseImage(&m_vid.pImgDisplay1);
  }

  if( m_vid.pVidToGuiTrans != NULL )
  {
    delete m_vid.pVidToGuiTrans;
  }

  if( m_calib.pImgEmptyRed != NULL )
  {
    cvReleaseImage(&m_calib.pImgEmptyRed);
  }
  if( m_calib.pImgEmptyBlue != NULL )
  {
    cvReleaseImage(&m_calib.pImgEmptyBlue);
  }

  if( m_game.pImgPrevRed != NULL )
  {
    cvReleaseImage(&m_game.pImgPrevRed);
  }
  if( m_game.pImgPrevBlue != NULL )
  {
    cvReleaseImage(&m_game.pImgPrevBlue);
  }


  if( m_gui.pMenu != NULL )
  {
    m_gui.pMenu->Unbind();
  }

  if( m_gui.pWin != NULL )
  {
    m_gui.pWin->WorkspaceRemoveAll();
    delete m_gui.pWin;
  }
}

void StaleMateSession::SetVideoDevice(const char *sVidDevName,
                                      uint_t      uVidDevMinor)
{
  m_vid.sVidDevName   = newstr(sVidDevName);
  m_vid.uVidDevMinor  = uVidDevMinor;
}

void StaleMateSession::SetHekDevice(const char *sHekDevName, int nBaudRate)
{
  m_hek.sHekDevName   = newstr(sHekDevName);
  m_hek.nHekBaudRate  = nBaudRate;
}
