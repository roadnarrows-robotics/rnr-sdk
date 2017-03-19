////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMVision.cxx
//
/*! \file
 *
 * $LastChangedDate: 2011-08-17 10:38:51 -0600 (Wed, 17 Aug 2011) $
 * $Rev: 1255 $
 *
 * \brief Chess vision routines.
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
// Private Interface 
// ---------------------------------------------------------------------------

// 640/8 * 480/8 * 255 = max square difference
static double VisMaxSqDiff = 1224000.0; 

static int VisMarkGameState(StaleMateSession &session)
{
  IplImage   *pImg;
  IplImage   *pImgGray;
  IplImage   *pImgDiff;
  double      fMin;
  CvPoint     ptMin;
  double      fMax;
  CvPoint     ptMax;
  int         rc;

  //
  // Grab and save image of current board state.
  //
  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  if( session.m_game.pImgPrevRed != NULL )
  {
    cvReleaseImage(&session.m_game.pImgPrevRed);
  }

  session.m_game.pImgPrevRed = StaleMateCreateRgbChannel(pImg, CHANNEL_RED); 

  cvSetImageROI(session.m_game.pImgPrevRed, session.m_calib.rectRoiBoard);

  if( session.m_game.pImgPrevBlue != NULL )
  {
    cvReleaseImage(&session.m_game.pImgPrevBlue);
  }

  session.m_game.pImgPrevBlue = StaleMateCreateRgbChannel(pImg, CHANNEL_BLUE);

  cvSetImageROI(session.m_game.pImgPrevBlue, session.m_calib.rectRoiBoard);

  cvReleaseImage(&pImg);

  //
  // Auto-calibrate difference of video frames that have no game state changes.
  //

  // wait for few video frames to pass
  session.m_gui.pWin->WaitKey(250);

  // get another frame
  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  // difference image
  pImgDiff = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  cvSetImageROI(pImgDiff, session.m_calib.rectRoiBoard);

  //
  // Blue
  //
  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_BLUE); 

  cvSetImageROI(pImgGray, session.m_calib.rectRoiBoard);

  cvAbsDiff(pImgGray, session.m_game.pImgPrevBlue, pImgDiff);

  cvMinMaxLoc(pImgDiff, &fMin, &fMax, &ptMin, &ptMax, NULL);

  //session.m_game.fDiffThBlue = fMax + TuneDiffThresholdBlue;
  session.m_game.fDiffThBlue = TuneDiffThresholdBlue;

  session.m_game.fBoardThBlue = TuneBoardThresholdBlue;

  cvReleaseImage(&pImgGray);

  //
  // Red
  //
  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_RED); 

  cvSetImageROI(pImgGray, session.m_calib.rectRoiBoard);

  cvAbsDiff(pImgGray, session.m_game.pImgPrevRed, pImgDiff);

  cvMinMaxLoc(pImgDiff, &fMin, &fMax, &ptMin, &ptMax, NULL);

//  cerr << "DBG: Mark fMaxRed=" << fMax << endl;

  // session.m_game.fDiffThRed = fMax + TuneDiffThresholdRed;
  session.m_game.fDiffThRed = TuneDiffThresholdRed;

  session.m_game.fBoardThRed = TuneBoardThresholdRed;

  //
  // Other data
  //
  session.m_game.uSpikedFrameCnt = 0;

  //
  // Reset ROI's
  //
  cvResetImageROI(session.m_game.pImgPrevRed);
  cvResetImageROI(session.m_game.pImgPrevBlue);

  //
  // Release
  //
  cvReleaseImage(&pImgDiff);
  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

static int VisWatchForBoardStateChange(StaleMateSession &session)
{
  IplImage     *pImg;
  IplImage     *pImgGray;
  IplImage     *pImgDiff;
  CvScalar      sum;
  int           rc;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  cvSetImageROI(session.m_game.pImgPrevRed, session.m_calib.rectRoiBoard);

  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_RED); 
  cvSetImageROI(pImgGray, session.m_calib.rectRoiBoard);

  pImgDiff = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  cvZero(pImgDiff);
  cvSetImageROI(pImgDiff, session.m_calib.rectRoiBoard);

  cvAbsDiff(pImgGray, session.m_game.pImgPrevRed, pImgDiff);

double      fMin;
CvPoint     ptMin;
double      fMax;
CvPoint     ptMax;
cvMinMaxLoc(pImgDiff, &fMin, &fMax, &ptMin, &ptMax, NULL);

//  cerr << "DBG: Watch fMaxRed=" << fMax << endl;

  cvThreshold(pImgDiff, pImgDiff, session.m_game.fDiffThRed,
                  0.0, CV_THRESH_TOZERO);

  sum = cvSum(pImgDiff);

  //cerr << "DBG: sum=" << sum.val[0] << endl;

  if( sum.val[0] > session.m_game.fBoardThRed )
  {
    session.m_game.uSpikedFrameCnt++;
  }
  else
  {
    session.m_game.uSpikedFrameCnt = 0;
  }

  cvResetImageROI(session.m_game.pImgPrevRed);
  cvResetImageROI(pImgDiff);

  StaleMateIoIShow(session, pImgDiff, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImgDiff);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

static int VisFindBestCandiateSquares(StaleMateSession &session,
                                      CandidateSq_T     candidate[],
                                      int               nCount)
{
  IplImage *pImg;
  IplImage *pImgMask;
  IplImage *pImgRed;
  IplImage *pImgRedDiff;
  IplImage *pImgBlue;
  IplImage *pImgBlueDiff;
  IplImage *pImgMaskDiff;
  CvScalar  colorMask = CV_RGB(255, 255, 255);
  CvScalar  sum;
  double    fSumRed, fSumBlue, fNorm;
  int       i, j, k, m;
  int       rc;

  if( !session.m_calib.bCalibrated )
  {
    LOGERROR("Not calibrated - cannot do mask test.");
    return 0;
  }

  else if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return 0;
  }

  pImgMask  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  pImgRed   = StaleMateCreateRgbChannel(pImg, CHANNEL_RED);
  pImgBlue  = StaleMateCreateRgbChannel(pImg, CHANNEL_BLUE);

  pImgRedDiff   = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  pImgBlueDiff  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  pImgMaskDiff  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 

  cvZero(pImgRedDiff);
  cvZero(pImgBlueDiff);
  cvZero(pImgMaskDiff);

  cvAbsDiff(pImgRed, session.m_game.pImgPrevRed, pImgRedDiff);
  cvAbsDiff(pImgBlue, session.m_game.pImgPrevBlue, pImgBlueDiff);

  cvThreshold(pImgRedDiff, pImgRedDiff, session.m_game.fDiffThRed,
                  0.0, CV_THRESH_TOZERO);
  cvThreshold(pImgBlueDiff, pImgBlueDiff, session.m_game.fDiffThBlue,
                  0.0, CV_THRESH_TOZERO);

  memset(candidate, 0, sizeof(CandidateSq_T)*(size_t)nCount);

  for(i=0; i<session.m_game.nChessBoardDim; ++i)
  {
    for(j=0; j<session.m_game.nChessBoardDim; ++j)
    {
      // make mask
      cvZero(pImgMask);
      cvFillConvexPoly(pImgMask, session.m_calib.ptBoard[i][j], 4, colorMask);

      //cerr << "DGG: square mask " << i << "," << j << endl;
      //StaleMateIoIShow(session, pImgMask, session.m_vid.uImgIndex1);
      //session.m_gui.pWin->WaitKey(1000);

      cvAnd(pImgRedDiff, pImgMask, pImgMaskDiff);
      sum = cvSum(pImgMaskDiff);

      fSumRed = sum.val[0] > TuneSquareThresholdRed? (double)sum.val[0]: 0.0;

      fNorm = fSumRed / VisMaxSqDiff;
      if( fNorm > 1.0 )
      {
        fNorm = 1.0;
      }

      // insert in descending order
      if( fSumRed > TuneSquareThresholdRed )
      {
        for(k=0; k<nCount; ++k)
        {
          if( fNorm > candidate[k].m_fNorm )
          {
            for(m=nCount-1; m>k; --m)
            {
              candidate[m] = candidate[m-1];
            }
            candidate[k].m_nRow     = i;
            candidate[k].m_nCol     = j;
            candidate[k].m_nChannel = CHANNEL_RED;
            candidate[k].m_fNorm    = fNorm;
            break;
          }
        }
      }

      cvAnd(pImgBlueDiff, pImgMask, pImgMaskDiff);
      sum = cvSum(pImgMaskDiff);
    
      fSumBlue = sum.val[0] > TuneSquareThresholdBlue? (double)sum.val[0]: 0.0;

      fNorm = fSumBlue / VisMaxSqDiff;
      if( fNorm > 1.0 )
      {
        fNorm = 1.0;
      }

      // insert in descending order
      if( fSumBlue > TuneSquareThresholdBlue )
      {
        // red was inserted, replace if blue is better
        if( k < nCount )
        {
          if( fNorm > candidate[k].m_fNorm )
          {
            candidate[k].m_nChannel = CHANNEL_BLUE;
            candidate[k].m_fNorm    = fNorm;
          }
        }

        // insert in descending order
        else
        {
          for(k=0; k<nCount; ++k)
          {
            if( fNorm > candidate[k].m_fNorm )
            {
              for(m=nCount-1; m>k; --m)
              {
                candidate[m] = candidate[m-1];
              }
            }
            candidate[k].m_nRow     = i;
            candidate[k].m_nCol     = j;
            candidate[k].m_nChannel = CHANNEL_BLUE;
            candidate[k].m_fNorm    = fNorm;
            break;
          }
        }
      }
    }
  }

  cvReleaseImage(&pImgMask);
  cvReleaseImage(&pImgRed);
  cvReleaseImage(&pImgRedDiff);
  cvReleaseImage(&pImgBlue);
  cvReleaseImage(&pImgBlueDiff);
  cvReleaseImage(&pImgMaskDiff);
  cvReleaseImage(&pImg);

  for(k=0, m=0; k<nCount; ++k)
  {
    if( candidate[k].m_fNorm > 0.0 )
    {
      ++m;
    }
  }

  return m;
}

static void VisShowCandidates(StaleMateSession &session,
                             CandidateSq_T      candidate[],
                             int                nCount)
{
  IplImage *pImg;
  CvScalar  colorPoly = CV_RGB(0, 192, 64);
  CvScalar  colorText = CV_RGB(0, 192, 64);
  CvFont    font;
  char      bufText[32];
  CvPoint  *pPolyLines[1];
  int       nPolyPts[1] = {4};
  int       i, j, k;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return;
  }

  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.80, 0.80, 0, 1);

  for(k=0; k<nCount; ++k)
  {
    i = candidate[k].m_nRow;
    j = candidate[k].m_nCol;

    sprintf(bufText, "%s", StaleMateChessBoardCoordStr(i, j));
    cvPutText(pImg, bufText, session.m_calib.ptBoard[i][j][3],
          &font, colorText);
    pPolyLines[0] = session.m_calib.ptBoard[i][j];
    cvPolyLine(pImg, pPolyLines, nPolyPts, 1, 2, colorPoly, 5, 8);
  }

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImg);
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

int StaleMateVisionRecordBoardState(StaleMateSession &session)
{
  int   rc;

  rc = VisMarkGameState(session);

  return rc;
}

bool StaleMateVisionWatchForOpponentsMove(StaleMateSession &session)
{
  VisWatchForBoardStateChange(session);

  if( session.m_game.uSpikedFrameCnt < TuneFramesWithChangeCnt )
  {
    return false;
  }

  else
  {
    return true;
  }
}

int StaleMateVisionFindBestCandidates(StaleMateSession &session,
                                      CandidateSq_T     candidate[],
                                      int               nCount)
{
  int   n;

  n = VisFindBestCandiateSquares(session, candidate, nCount);

  VisShowCandidates(session, candidate, n);

  // wait for slow humans to see the candidates
  if( n > 1 )
  {
    session.m_gui.pWin->WaitKey(500);
  }

  return n;
}

double StaleMateVisionCheckEmptySq(StaleMateSession &session,
                                   int               nRow,
                                   int               nCol)
{
  IplImage *pImg;
  IplImage *pImgMask;
  IplImage *pImgRed;
  IplImage *pImgRedDiff;
  IplImage *pImgMaskDiff;
  CvScalar  colorMask = CV_RGB(255, 255, 255);
  CvScalar  sum;
  double    fSumRed, fSumBlue, fNorm;
  int       i, j, k, m;
  int       rc;

  if( !session.m_calib.bCalibrated )
  {
    LOGERROR("Not calibrated - cannot do mask test.");
    return 0.0;
  }

  else if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return 0.0;
  }

  pImgMask  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  pImgRed   = StaleMateCreateRgbChannel(pImg, CHANNEL_RED);

  //cvSetImageROI(pImgMask, session.m_calib.rectRoiBoard);
  //cvSetImageROI(pImgRed, session.m_calib.rectRoiBoard);

  pImgRedDiff   = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
  pImgMaskDiff  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 

  //cvSetImageROI(pImgRedDiff, session.m_calib.rectRoiBoard);
  //cvSetImageROI(pImgMaskDiff, session.m_calib.rectRoiBoard);

  cvZero(pImgRedDiff);
  cvZero(pImgMaskDiff);

  cvAbsDiff(pImgRed, session.m_calib.pImgEmptyRed, pImgRedDiff);

  cvThreshold(pImgRedDiff, pImgRedDiff, session.m_game.fDiffThRed,
                  0.0, CV_THRESH_TOZERO);

  // make mask
  cvZero(pImgMask);
  cvFillConvexPoly(pImgMask, session.m_calib.ptBoard[nRow][nCol], 4, colorMask);

  cvAnd(pImgRedDiff, pImgMask, pImgMaskDiff);
  sum = cvSum(pImgMaskDiff);

  fSumRed = sum.val[0] > TuneSquareThresholdRed? (double)sum.val[0]: 0.0;

  fNorm = fSumRed / VisMaxSqDiff;

  if( fNorm > 1.0 )
  {
    fNorm = 1.0;
  }

  cvReleaseImage(&pImgMask);
  cvReleaseImage(&pImgRed);
  cvReleaseImage(&pImgRedDiff);
  cvReleaseImage(&pImgMaskDiff);
  cvReleaseImage(&pImg);

  return fNorm;
}

void StaleMateVisionDrawSq(StaleMateSession &session,
                           IplImage         *pImg,
                           int               nRow,
                           int               nCol,
                           const char       *sText,
                           CvScalar          color)
{
  CvFont    font;
  CvPoint   ptText;
  CvPoint  *pPolyLines[1];
  int       nPolyPts[1] = {4};

  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.80, 0.80, 0, 1);
  ptText.x = session.m_calib.ptBoard[nRow][nCol][3].x + 4;
  ptText.y = session.m_calib.ptBoard[nRow][nCol][3].y - 8;
  cvPutText(pImg, sText, ptText, &font, color);
  pPolyLines[0] = session.m_calib.ptBoard[nRow][nCol];
  cvPolyLine(pImg, pPolyLines, nPolyPts, 1, 2, color, 5, 8);
}
