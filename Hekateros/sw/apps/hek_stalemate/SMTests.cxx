////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMTests.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief Test routines.
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
// Public Interface 
// ---------------------------------------------------------------------------

void StaleMateDrawLines(CvSeq *pLines, IplImage *pImg)
{
  float  *line;
  float   rho;
  float   theta;
  CvPoint pt1, pt2;
  double  a, b;
  int i;

  for(i=0; i<pLines->total; ++i)
  {
    line  = (float*)cvGetSeqElem(pLines, i);
    rho   = line[0];
    theta = line[1];
    a     = cos(theta);
    b     = sin(theta);

    if( fabs(a) < 0.001 )
    {
      pt1.x = pt2.x = cvRound(rho);
      pt1.y = 0;
      pt2.y = pImg->height;
    }
    else if( fabs(b) < 0.001 )
    {
      pt1.y = pt2.y = cvRound(rho);
      pt1.x = 0;
      pt2.x = pImg->width;
    }
    else
    {
      pt1.x = 0;
      pt1.y = cvRound(rho/b);
      pt2.x = cvRound(rho/a);
      pt2.y = 0;
    }
    cvLine(pImg, pt1, pt2, CV_RGB(255,0,0), 1, 8);
  }
}

int StaleMateTestHoughLines(StaleMateSession &session)
{
  IplImage     *pImg;
  IplImage     *pImgGray;
  CvMemStorage *pStorage;
  CvSeq        *pLines;
  int           nMaxDilations = 0;
  int           i;
  int           rc;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_GRAY); 
  pStorage = cvCreateMemStorage(0);

  cvCanny(pImgGray, pImgGray, 50, 200, 3);

  for(i=0; i<nMaxDilations; ++i)
  {
    cvDilate(pImgGray, pImgGray, NULL, 1);
  }

  //cvThreshold(pImgGray, pImgGray, 10, 255, CV_THRESH_BINARY);
  
  //cvErode(pImgGray, pImgGray, NULL, 1);

  //cvAdaptiveThreshold(pImgGray, pImgGray, 255, 
  //                      CV_ADAPTIVE_THRESH_MEAN_C,
  //                      CV_THRESH_BINARY, 
  //                      5);

  pLines = cvHoughLines2(pImgGray, pStorage,
                //CV_HOUGH_PROBABILISTIC,
                CV_HOUGH_STANDARD,
                1, CV_PI/360, 150, 0, 0);
                //1, CV_PI/180, 150, 0, 0);
  
  cvCvtColor(pImgGray, pImg, CV_GRAY2RGB);

  StaleMateDrawLines(pLines, pImg);

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

  cvReleaseMemStorage(&pStorage);
  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

int StaleMateTestRgbChannel(StaleMateSession &session, int nRgbChannel)
{
  IplImage *pImg;
  IplImage *pImgGray;
  int       rc;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgGray = StaleMateCreateRgbChannel(pImg, nRgbChannel);

  StaleMateIoIShow(session, pImgGray, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

int StaleMateTestHsvChannel(StaleMateSession &session, int nHsvChannel)
{
  IplImage *pImg;
  IplImage *pImgHsv;
  IplImage *pImgGray;
  int       rc;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgHsv  = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 3); 

  cvCvtColor(pImg, pImgHsv, CV_BGR2HSV);
  
  pImgGray = StaleMateCreateHsvChannel(pImgHsv, nHsvChannel);

  StaleMateIoIShow(session, pImgGray, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgHsv);
  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

int StaleMateTestChessBoardCalib(StaleMateSession &session)
{
  IplImage     *pImg;
  IplImage     *pImgGray;
  CvSize        sizeChessBoard = cvSize(5, 5);  // chessboard is 7x7
  CvPoint2D32f  bufCorners[100];
  int           nCornerCnt;
  int           rc;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_GRAY); 

  rc = cvFindChessboardCorners(pImgGray, sizeChessBoard, bufCorners,
                      &nCornerCnt, CV_CALIB_CB_ADAPTIVE_THRESH);

  cvCvtColor(pImgGray, pImg, CV_GRAY2RGB);

  cvDrawChessboardCorners(pImg, sizeChessBoard, bufCorners,
                          nCornerCnt, rc);

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

int StaleMateTestGrayScaleDifference(StaleMateSession &session,
                                     int              nRgbChannel)
{
  static IplImage     *pImgPrev = NULL;

  IplImage     *pImg;
  IplImage     *pImgGray;
  IplImage     *pImgDiff;
  int           rc;
  CvScalar      s;

  if( pImgPrev == NULL )
  {
    pImgPrev = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 
    cvZero(pImgPrev);
    return HEK_OK;
  }

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgGray = StaleMateCreateRgbChannel(pImg, nRgbChannel);
  pImgDiff = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 

  cvAbsDiff(pImgGray, pImgPrev, pImgDiff);

  StaleMateIoIShow(session, pImgDiff, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgPrev);
  cvReleaseImage(&pImgDiff);
  cvReleaseImage(&pImg);

  pImgPrev = pImgGray;

  return HEK_OK;
}

int StaleMateTestMask(StaleMateSession &session, int row, int col)
{
  IplImage *pImg;
  IplImage *pImgMask;
  IplImage *pImgGray;
  CvScalar  colorMask = CV_RGB(255, 255, 255);
  int       rc;

  if( !session.m_calib.bCalibrated )
  {
    LOGERROR("Not calibrated - cannot do mask test.");
    return -HEK_ECODE_INTERNAL;
  }

  else if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return -HEK_ECODE_VIDEO;
  }

  pImgMask = cvCreateImage(session.m_vid.sizeVideo, IPL_DEPTH_8U, 1); 

  cvZero(pImgMask);

  cvFillConvexPoly(pImgMask, session.m_calib.ptBoard[row][col], 4, colorMask);

  pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_RED);

  cvAnd(pImgGray, pImgMask, pImgGray);

  StaleMateIoIShow(session, pImgGray, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgMask);
  cvReleaseImage(&pImgGray);
  cvReleaseImage(&pImg);

  return HEK_OK;
}

int StaleMateTestPlayChess(StaleMateSession &session,
                           bool              bUseArm,
                           bool              bUseVision,
                           bool              bUseGnuChess)
{
  StaleMateChessNewGame(session);
  StaleMateUciNewGame(session);

  // move the white queen's knight
  session.m_gui.pWin->WaitKey(500);
  StaleMateChessMovePiece(session, 7, 1, 5, 2);

  // move the black king's pawn
  session.m_gui.pWin->WaitKey(500);
  StaleMateChessMovePiece(session, 1, 4, 2, 4);

  // move the white queen's knight
  session.m_gui.pWin->WaitKey(500);
  StaleMateChessMovePiece(session, 5, 2, 3, 1);

  // move the black kings rook's pawn
  session.m_gui.pWin->WaitKey(500);
  StaleMateChessMovePiece(session, 1, 7, 3, 7);

  // move the white queen's knight
  session.m_gui.pWin->WaitKey(500);
  StaleMateChessMovePiece(session, 3, 1, 1, 2);
}

int StaleMateTestMove(StaleMateSession &session)
{
  static char           bufAlgSq[4] = {0, };
  static double         fAlgZ       = 50.0;
  static CvPoint3D32f   ptCart      = {200.0, 0.0, 50.0};
  static DynaPosTuple_T tupGoal[SM_HEK_NSERVOS_BASE] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_SHOULDER_R,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0}
  };
  DynaPosTuple_T tupCurPos[SM_HEK_NSERVOS_BASE] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_SHOULDER_R,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0}
  };
  ChessSquare_T         sq;
  CvPoint3D32f          ptGoal;

  StaleMateGuiDlgTestMove(session, bufAlgSq, &fAlgZ, &ptCart);

  // algebraic take priority over cartesion
  if( (bufAlgSq[0] != 0) && (session.m_calib.bCalibrated) )
  {
    sq = VAlgSqToVRowCol(session, bufAlgSq);
    if( (sq.m_nRow < 0) || (sq.m_nRow >= session.m_game.nChessBoardDim) ||
        (sq.m_nCol < 0) || (sq.m_nCol >= session.m_game.nChessBoardDim) )
    {
      LOGERROR("Test Move: '%s': invalid algebraic chess square.", bufAlgSq);
      return HEK_ECODE_BAD_VAL;
    }
    ptGoal.x = session.m_calib.ptHekDist[sq.m_nRow][sq.m_nCol].x;
    ptGoal.y = session.m_calib.ptHekDist[sq.m_nRow][sq.m_nCol].y;
    ptGoal.z = fAlgZ;
  }

  // cartesion
  else
  {
    ptGoal = ptCart;
  }

  LOGDIAG2("Test Move: goal=(%.1f,%.1f,%.1f).",
      ptGoal.x, ptGoal.y, ptGoal.z);

  // DANIEL make sure the goal is acheivable
  // goal boundary checks.
  // ...
  
  // plan move
  session.m_hek.motionPlanner.ConstrainedPlanner(ptGoal, tupGoal,
                                              SM_HEK_NSERVOS_BASE);

  // get current position of some sanity checks
  StaleMateHekGetCurPos(session, tupCurPos, SM_HEK_NSERVOS_BASE);

  // DANIEL make sure the motion is valid and safe, especially servos 2 & 3
  // ...

  // move
  StaleMateHekSyncMove(session, tupGoal, SM_HEK_NSERVOS_BASE);

  return HEK_OK;
}

bool StaleMateTestUci(StaleMateSession &session)
{
  enum TestUciState_T
  {
    TestUciStateStart,
    TestUciStateThink,
    TestUciStateMove,
    TestUciStateEnd
  };

  static TestUciState_T   TestUciState = TestUciStateStart; // state
  static ChessPieceColor  TestUciSide  = WhitePiece;        // side

  char  bufAlgSqFrom[4];
  char  bufAlgSqTo[4];
  char  bufMsg[128];

  if( TestUciState == TestUciStateEnd )
  {
    TestUciState = TestUciStateStart;
  }

  switch( TestUciState )
  {
    case TestUciStateStart:
      StaleMateUciSelfPlay(session, true);
      StaleMateChessNewGame(session);
      session.m_gui.pWin->ShowStatus("Testing UCI Interface");
      TestUciSide  = WhitePiece;
      TestUciState = TestUciStateMove;
      break;
    case TestUciStateThink:
      StaleMateUciSelfPlay(session, false);
      TestUciState = TestUciStateMove;
      break;
    case TestUciStateMove:
      if( StaleMateUciGetEnginesMove(session, bufAlgSqFrom, bufAlgSqTo) )
      {
        switch( bufAlgSqFrom[0] )
        {
          case UciCodeCheckMate:
            sprintf(bufMsg, "Check Mate! %s mates %s", 
                (TestUciSide==WhitePiece? "Black": "White"),
                (TestUciSide==WhitePiece? "White": "Black"));
            StaleMateChessHistAdd(session, bufMsg);
            session.m_gui.pWin->ShowStatus(bufMsg);
            TestUciState = TestUciStateEnd;
            break;
          case UciCodeResign:
            sprintf(bufMsg, "%s resigns!",
                (TestUciSide==WhitePiece? "Black": "White"));
            StaleMateChessHistAdd(session, bufMsg);
            session.m_gui.pWin->ShowStatus(bufMsg);
            TestUciState = TestUciStateEnd;
            break;
          case UciCodeDraw:
            sprintf(bufMsg, "%s", "Game is a draw!");
            StaleMateChessHistAdd(session, bufMsg);
            session.m_gui.pWin->ShowStatus(bufMsg);
            TestUciState = TestUciStateEnd;
            break;
          default:
            if( StaleMateChessIsCastleMove(session, bufAlgSqFrom, bufAlgSqTo) )
            {
              StaleMateChessMovePiece(session, bufAlgSqFrom, bufAlgSqTo);
              StaleMateChessCastleRook(session, bufAlgSqFrom, bufAlgSqTo);
              StaleMateChessMovePiece(session, bufAlgSqFrom, bufAlgSqTo);
              StaleMateChessCastled(session, TestUciSide);
            }
            else
            {
              StaleMateChessMovePiece(session, bufAlgSqFrom, bufAlgSqTo);
            }
            TestUciState = TestUciStateThink;
        }
        
        TestUciSide = (ChessPieceColor)(((int)TestUciSide + 1) & 0x01);
      }
      break;
    case TestUciStateEnd:
      break;
  }

  return TestUciState < TestUciStateEnd? true: false;
}
