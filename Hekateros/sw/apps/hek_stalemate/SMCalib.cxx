////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMCalib.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-05 15:17:26 -0600 (Tue, 05 Jun 2012) $
 * $Rev: 2028 $
 *
 * \brief StaleMate calibration.
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
using namespace cv;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

static bool SMCalibBoardSquares(StaleMateSession &session, int nChessBoardDim)
{
  IplImage     *pImg;
  IplImage     *pImgGray = NULL;
  int           nVertDim = nChessBoardDim - 1;
  int           nVertTot = nVertDim * nVertDim;
  CvSize        sizeChessBoard = cvSize(nVertDim, nVertDim);
  CvPoint2D32f  ptCorners[nVertTot];
  int           nCornerCnt;
  CvPoint       ptImg[nVertDim][nVertDim];
  Rot           eRot;
  int           i, j, k, m, n;
  int           w1, w2, w3, h1, h2, h3;
  double        dx1, dy1, dx2, dy2;
  int           delta, ddelta;
  double        x, y;
  double        x11, y11, x12, y12, x21, y21, x22, y22;
  CvScalar      colorPoly = CV_RGB(192, 0, 255);
  CvScalar      colorText = CV_RGB(0, 192, 64);
  CvFont        font;
  char          bufText[32];
  CvPoint      *pPolyLines[nChessBoardDim];
  int           nPolyPts[nChessBoardDim];
  int           rc = 0;

  // - - - -
  // Run chess calibration
  // - - - -
  while( (rc == 0) && 
      (session.m_gui.pMenu->GetCurrentAction() == StaleMateActionCalib) )
  {
    pImg = StaleMateVideoCreateSnapShot(session);

    if( pImg == NULL )
    {
      break;
    }

    if( pImgGray != NULL )
    {
      cvReleaseImage(&pImgGray);
    }

    pImgGray = StaleMateCreateRgbChannel(pImg, CHANNEL_GRAY); 

    rc = cvFindChessboardCorners(pImgGray, sizeChessBoard, ptCorners,
                      &nCornerCnt, CV_CALIB_CB_ADAPTIVE_THRESH);

    cvCvtColor(pImgGray, pImg, CV_GRAY2RGB);

    //cvDrawChessboardCorners(pImg, sizeChessBoard, ptCorners,
    //                      nCornerCnt, rc);

    StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

    cvReleaseImage(&pImg);

    session.m_gui.pWin->WaitKey(5);
  }

  // - - - -
  // Calibration aborted or failed.
  // - - - -
  if( rc == 0 )
  {
    if( pImgGray != NULL )
    {
      cvReleaseImage(&pImgGray);
    }
    LOGERROR("Failed to calibrate board squares.");
    return false;
  }

  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.50, 0.50, 0, 1);

  // - - - -
  // Determine how the corners are rotated clockwise relative to the chess board
  // 1h position ([0][0] square).
  // - - - -
  m = nVertDim + 1; // catty-corner index
  if( (ptCorners[0].x < ptCorners[m].x) &&
      (ptCorners[0].y < ptCorners[m].y) )
  {
    eRot = Rot0;
    cerr << "DBG: corners are not rotated." << endl;
  }
  else if( (ptCorners[0].x > ptCorners[m].x) &&
           (ptCorners[0].y < ptCorners[m].y))
  {
    eRot = Rot90;
    cerr << "DBG: corners are 90 degrees cw rotated." << endl;
  }
  else if( (ptCorners[0].x > ptCorners[m].x) &&
           (ptCorners[0].y > ptCorners[m].y))
  {
    eRot = Rot180;
    cerr << "DBG: corners are 180 degrees cw rotated." << endl;
  }
  else if( (ptCorners[0].x < ptCorners[m].x) &&
           (ptCorners[0].y > ptCorners[m].y))
  {
    eRot = Rot270;
    cerr << "DBG: corners are 270 degrees cw rotated." << endl;
  }
  else
  {
    eRot = Rot0;
    cerr << "DBG: cannot determine corners rotation - assuming 0 degrees."
          << endl;
  }

  // - - - -
  // Convert corners to image points normalizing to 0 degree rotation.
  // - - - -
  for(i=0; i<nVertTot; ++i)
  {
    switch( eRot )
    {
      default:
      case Rot0:
        m = i / nVertDim;
        n = i % nVertDim;
        break;
      case Rot90:
        m = i % nVertDim;
        n = (nVertTot - i - 1) / nVertDim;
        break;
      case Rot180:
        m = (nVertTot - i - 1) / nVertDim;
        n = (nVertTot - i - 1) % nVertDim;
        break;
      case Rot270:
        m = (nVertTot - i - 1) % nVertDim;
        n = i / nVertDim;
        break;
    }

    ptImg[m][n].x = cvRound(ptCorners[i].x);
    ptImg[m][n].y = cvRound(ptCorners[i].y);

    sprintf(bufText, "%d", m*nVertDim+n);
    cvPutText(pImg, bufText, ptImg[m][n], &font, colorText);
  }

  cerr << "DBG: corners[]={";
  for(m=0; m<nVertDim; ++m)
  {
    cerr << endl;
    for(n=0; n<nVertDim; ++n)
    {
      cerr << "(" << ptImg[m][n].x << "," << ptImg[m][n].y << ") ";
    }
  }
  cerr << "}" << endl;

  memset(session.m_calib.ptBoard, 0, sizeof(session.m_calib.ptBoard));

  // - - - -
  // Central Board Area
  // - - - -
  for(i=1, m=0; i<nVertDim; ++i, ++m)
  {
    for(j=1, n=0; j<nVertDim; ++j, ++n)
    {
      session.m_calib.ptBoard[i][j][0].x = ptImg[m][n].x;
      session.m_calib.ptBoard[i][j][0].y = ptImg[m][n].y;

      session.m_calib.ptBoard[i][j][1].x = ptImg[m][n+1].x;
      session.m_calib.ptBoard[i][j][1].y = ptImg[m][n+1].y;

      session.m_calib.ptBoard[i][j][2].x = ptImg[m+1][n+1].x;
      session.m_calib.ptBoard[i][j][2].y = ptImg[m+1][n+1].y;
      
      session.m_calib.ptBoard[i][j][3].x = ptImg[m+1][n].x;
      session.m_calib.ptBoard[i][j][3].y = ptImg[m+1][n].y;
    }
  }

  // - - - -
  // Outer Left Column
  // - - - -
  for(i=1, j=0, k=1, m=2, n=3; i<nChessBoardDim-1; ++i)
  {
    // shared points
    session.m_calib.ptBoard[i][j][1].x = session.m_calib.ptBoard[i][k][0].x;
    session.m_calib.ptBoard[i][j][1].y = session.m_calib.ptBoard[i][k][0].y;
    session.m_calib.ptBoard[i][j][2].x = session.m_calib.ptBoard[i][k][3].x;
    session.m_calib.ptBoard[i][j][2].y = session.m_calib.ptBoard[i][k][3].y;

    // find the top widths of the 3 right neighbors
    w1 = iabs(session.m_calib.ptBoard[i][k][1].x -
              session.m_calib.ptBoard[i][k][0].x);
    w2 = iabs(session.m_calib.ptBoard[i][m][1].x -
              session.m_calib.ptBoard[i][m][0].x);
    w3 = iabs(session.m_calib.ptBoard[i][n][1].x -
              session.m_calib.ptBoard[i][n][0].x);

    // calculate the delta widths and the delta deltas
    delta   = w1 - w2;
    ddelta  = 0; //delta - (w2 - w3);

    // estimate top left x position
    x = (double)(session.m_calib.ptBoard[i][j][1].x - (w1 + delta + ddelta));

    //
    //  y = (y2 - y1)/(x2 - x1) * (x - x1) + y1
    //
  
    // delta x and y of the top points of the right neighbor
    dx1 = (double)(session.m_calib.ptBoard[i][k][1].x -
              session.m_calib.ptBoard[i][k][0].x);
    dy1 = (double)(session.m_calib.ptBoard[i][k][1].y -
              session.m_calib.ptBoard[i][k][0].y);

    // calculate top left y, given the x, by the two-point line formula:
    y = dy1/dx1 * (double)(x - session.m_calib.ptBoard[i][k][0].x) +
                (double)(session.m_calib.ptBoard[i][k][0].y);

    // set top-left point
    session.m_calib.ptBoard[i][j][0].x = cvRound(x);
    session.m_calib.ptBoard[i][j][0].y = cvRound(y);

    // find the bottom widths of the 3 right neighbors
    w1 = iabs(session.m_calib.ptBoard[i][k][3].x -
              session.m_calib.ptBoard[i][k][2].x);
    w2 = iabs(session.m_calib.ptBoard[i][m][3].x -
              session.m_calib.ptBoard[i][m][2].x);
    w3 = iabs(session.m_calib.ptBoard[i][n][3].x -
              session.m_calib.ptBoard[i][n][2].x);

    // calculate the delta widths and the delta deltas
    delta   = w1 - w2;
    ddelta  = 0; //delta - (w2 - w3);

    // estimate bottom left x position
    x = (double)(session.m_calib.ptBoard[i][j][2].x - (w1 + delta + ddelta));

    //
    //  y = (y2 - y1)/(x2 - x1) * (x - x1) + y1
    //
  
    // delta x and y of the bottom points of the right neighbor
    dx1 = (double)(session.m_calib.ptBoard[i][k][3].x -
              session.m_calib.ptBoard[i][k][2].x);
    dy1 = (double)(session.m_calib.ptBoard[i][k][3].y -
              session.m_calib.ptBoard[i][k][2].y);

    // calculate bottom left y, given the x, by the two-point line formula:
    y = dy1/dx1 * (double)(x - session.m_calib.ptBoard[i][k][2].x) +
                (double)(session.m_calib.ptBoard[i][k][2].y);

    // set bottom-left point
    session.m_calib.ptBoard[i][j][3].x = cvRound(x);
    session.m_calib.ptBoard[i][j][3].y = cvRound(y);
  }

  // - - - -
  // Outer Right Column
  // - - - -
  for(i=1, j=nChessBoardDim-1, k=j-1, m=j-2, n=j-3; i<nChessBoardDim-1; ++i)
  {
    // shared points
    session.m_calib.ptBoard[i][j][0].x = session.m_calib.ptBoard[i][k][1].x;
    session.m_calib.ptBoard[i][j][0].y = session.m_calib.ptBoard[i][k][1].y;
    session.m_calib.ptBoard[i][j][3].x = session.m_calib.ptBoard[i][k][2].x;
    session.m_calib.ptBoard[i][j][3].y = session.m_calib.ptBoard[i][k][2].y;

    // find the top widths of the 3 left neighbors
    w1 = iabs(session.m_calib.ptBoard[i][k][1].x -
              session.m_calib.ptBoard[i][k][0].x);
    w2 = iabs(session.m_calib.ptBoard[i][m][1].x -
              session.m_calib.ptBoard[i][m][0].x);
    w3 = iabs(session.m_calib.ptBoard[i][n][1].x -
              session.m_calib.ptBoard[i][n][0].x);

    // calculate the delta widths and the delta deltas
    delta   = w1 - w2;
    ddelta  = 0; //delta - (w2 - w3);

    // estimate top right x position
    x = (double)(session.m_calib.ptBoard[i][j][0].x + (w1 + delta + ddelta));

    //
    //  y = (y2 - y1)/(x2 - x1) * (x - x1) + y1
    //
  
    // delta x and y of the top points of the left neighbor
    dx1 = (double)(session.m_calib.ptBoard[i][k][0].x -
              session.m_calib.ptBoard[i][k][1].x);
    dy1 = (double)(session.m_calib.ptBoard[i][k][0].y -
              session.m_calib.ptBoard[i][k][1].y);

    // calculate top left y, given the x, by the two-point line formula:
    y = dy1/dx1 * (double)(x - session.m_calib.ptBoard[i][k][1].x) +
                (double)(session.m_calib.ptBoard[i][k][1].y);

    // set top-right point
    session.m_calib.ptBoard[i][j][1].x = cvRound(x);
    session.m_calib.ptBoard[i][j][1].y = cvRound(y);

    // find the bottom widths of the 3 left neighbors
    w1 = iabs(session.m_calib.ptBoard[i][k][3].x -
              session.m_calib.ptBoard[i][k][2].x);
    w2 = iabs(session.m_calib.ptBoard[i][m][3].x -
              session.m_calib.ptBoard[i][m][2].x);
    w3 = iabs(session.m_calib.ptBoard[i][n][3].x -
              session.m_calib.ptBoard[i][n][2].x);

    // calculate the delta widths and the delta deltas
    delta   = w1 - w2;
    ddelta  = 0; //delta - (w2 - w3);

    // estimate bottom right x position
    x = (double)(session.m_calib.ptBoard[i][j][3].x + (w1 + delta + ddelta));

    //
    //  y = (y2 - y1)/(x2 - x1) * (x - x1) + y1
    //
  
    // delta x and y of the bottom points of the left neighbor
    dx1 = (double)(session.m_calib.ptBoard[i][k][3].x -
              session.m_calib.ptBoard[i][k][2].x);
    dy1 = (double)(session.m_calib.ptBoard[i][k][3].y -
              session.m_calib.ptBoard[i][k][2].y);

    // calculate bottom left y, given the x, by the two-point line formula:
    y = dy1/dx1 * (double)(x - session.m_calib.ptBoard[i][k][2].x) +
                (double)(session.m_calib.ptBoard[i][k][2].y);

    // set bottom-right point
    session.m_calib.ptBoard[i][j][2].x = cvRound(x);
    session.m_calib.ptBoard[i][j][2].y = cvRound(y);
  }

  // - - - -
  // Outer Top Row
  // - - - -
  for(j=1, i=0, k=1, m=2, n=3; j<nChessBoardDim-1; ++j)
  {
    // shared points
    session.m_calib.ptBoard[i][j][2].x = session.m_calib.ptBoard[k][j][1].x;
    session.m_calib.ptBoard[i][j][2].y = session.m_calib.ptBoard[k][j][1].y;
    session.m_calib.ptBoard[i][j][3].x = session.m_calib.ptBoard[k][j][0].x;
    session.m_calib.ptBoard[i][j][3].y = session.m_calib.ptBoard[k][j][0].y;

    // find the left heights of the 3 bottom neighbors
    h1 = iabs(session.m_calib.ptBoard[k][j][3].y -
              session.m_calib.ptBoard[k][j][0].y);
    h2 = iabs(session.m_calib.ptBoard[m][j][3].y -
              session.m_calib.ptBoard[m][j][0].y);
    h3 = iabs(session.m_calib.ptBoard[n][j][3].y -
              session.m_calib.ptBoard[n][j][0].y);

    // calculate the delta heights and the delta deltas
    delta   = h1 - h2;
    ddelta  = 0; //delta - (h2 - h3);

    // estimate top left y position
    y = (double)(session.m_calib.ptBoard[i][j][3].y - (h1 + delta + ddelta));

    //
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    //

    // delta x and y of the left points of bottom neighbor
    dx1 = (double)(session.m_calib.ptBoard[k][j][3].x -
              session.m_calib.ptBoard[k][j][0].x);
    dy1 = (double)(session.m_calib.ptBoard[k][j][3].y -
              session.m_calib.ptBoard[k][j][0].y);

    // calculate top left x, given the y, by the two-point line formula:
    x = dx1/dy1 * (double)(y - session.m_calib.ptBoard[k][j][0].y) +
                (double)(session.m_calib.ptBoard[k][j][0].x);

    // set top-left point
    session.m_calib.ptBoard[i][j][0].x = cvRound(x);
    session.m_calib.ptBoard[i][j][0].y = cvRound(y);

    // find the right heights of the 3 bottom neighbors
    h1 = iabs(session.m_calib.ptBoard[k][j][2].y -
              session.m_calib.ptBoard[k][j][1].y);
    h2 = iabs(session.m_calib.ptBoard[m][j][2].y -
              session.m_calib.ptBoard[m][j][1].y);
    h3 = iabs(session.m_calib.ptBoard[n][j][2].y -
              session.m_calib.ptBoard[n][j][1].y);

    // calculate the delta heights and the delta deltas
    delta   = h1 - h2;
    ddelta  = 0; //delta - (h2 - h3);

    // estimate top right y position
    y = (double)(session.m_calib.ptBoard[i][j][2].y - (h1 + delta + ddelta));

    //
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    //

    // delta x and y of the right points of bottom neighbor
    dx1 = (double)(session.m_calib.ptBoard[k][j][2].x -
              session.m_calib.ptBoard[k][j][1].x);
    dy1 = (double)(session.m_calib.ptBoard[k][j][2].y -
              session.m_calib.ptBoard[k][j][1].y);

    // calculate top right x, given the y, by the two-point line formula:
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    x = dx1/dy1 * (double)(y - session.m_calib.ptBoard[k][j][1].y) +
                (double)(session.m_calib.ptBoard[k][j][1].x);

    // set top-right point
    session.m_calib.ptBoard[i][j][1].x = cvRound(x);
    session.m_calib.ptBoard[i][j][1].y = cvRound(y);
  }

  // - - - -
  // Outer Bottom Row
  // - - - -
  for(j=1, i=nChessBoardDim-1, k=i-1, m=i-2, n=i-3; j<nChessBoardDim-1; ++j)
  {
    // shared points
    session.m_calib.ptBoard[i][j][0].x = session.m_calib.ptBoard[k][j][3].x;
    session.m_calib.ptBoard[i][j][0].y = session.m_calib.ptBoard[k][j][3].y;
    session.m_calib.ptBoard[i][j][1].x = session.m_calib.ptBoard[k][j][2].x;
    session.m_calib.ptBoard[i][j][1].y = session.m_calib.ptBoard[k][j][2].y;

    // find the left heights of the 3 top neighbors
    h1 = iabs(session.m_calib.ptBoard[k][j][3].y -
              session.m_calib.ptBoard[k][j][0].y);
    h2 = iabs(session.m_calib.ptBoard[m][j][3].y -
              session.m_calib.ptBoard[m][j][0].y);
    h3 = iabs(session.m_calib.ptBoard[n][j][3].y -
              session.m_calib.ptBoard[n][j][0].y);

    // calculate the delta heights and the delta deltas
    delta   = h1 - h2;
    ddelta  = 0; //delta - (h2 - h3);

    // estimate bottom left y position
    y = (double)(session.m_calib.ptBoard[i][j][0].y + (h1 + delta + ddelta));

    //
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    //

    // delta x and y of the left points of top neighbor
    dx1 = (double)(session.m_calib.ptBoard[k][j][3].x -
              session.m_calib.ptBoard[k][j][0].x);
    dy1 = (double)(session.m_calib.ptBoard[k][j][3].y -
              session.m_calib.ptBoard[k][j][0].y);

    // calculate bottom left x, given the y, by the two-point line formula:
    x = dx1/dy1 * (double)(y - session.m_calib.ptBoard[k][j][0].y) +
                (double)(session.m_calib.ptBoard[k][j][0].x);

    // set top-left point
    session.m_calib.ptBoard[i][j][3].x = cvRound(x);
    session.m_calib.ptBoard[i][j][3].y = cvRound(y);

    // find the right heights of the 3 top neighbors
    h1 = iabs(session.m_calib.ptBoard[k][j][2].y -
              session.m_calib.ptBoard[k][j][1].y);
    h2 = iabs(session.m_calib.ptBoard[m][j][2].y -
              session.m_calib.ptBoard[m][j][1].y);
    h3 = iabs(session.m_calib.ptBoard[n][j][2].y -
              session.m_calib.ptBoard[n][j][1].y);

    // calculate the delta heights and the delta deltas
    delta   = h1 - h2;
    ddelta  = 0; //delta - (h2 - h3);

    // estimate bottom right y position
    y = (double)(session.m_calib.ptBoard[i][j][1].y + (h1 + delta + ddelta));

    //
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    //

    // delta x and y of the right points of top neighbor
    dx1 = (double)(session.m_calib.ptBoard[k][j][2].x -
              session.m_calib.ptBoard[k][j][1].x);
    dy1 = (double)(session.m_calib.ptBoard[k][j][2].y -
              session.m_calib.ptBoard[k][j][1].y);

    // calculate bottom right x, given the y, by the two-point line formula:
    //  x = (x2 - x1)/(y2 - y1) * (y - y1) + x1
    x = dx1/dy1 * (double)(y - session.m_calib.ptBoard[k][j][1].y) +
                (double)(session.m_calib.ptBoard[k][j][1].x);

    // set top-right point
    session.m_calib.ptBoard[i][j][2].x = cvRound(x);
    session.m_calib.ptBoard[i][j][2].y = cvRound(y);
  }
  
  // - - - -
  // Top Left Corner
  // - - - -
  i = 0; j = 0; m = 1; n = 1;

  // shared points
  session.m_calib.ptBoard[i][j][1].x = session.m_calib.ptBoard[i][n][0].x;
  session.m_calib.ptBoard[i][j][1].y = session.m_calib.ptBoard[i][n][0].y;
  session.m_calib.ptBoard[i][j][2].x = session.m_calib.ptBoard[i][n][3].x;
  session.m_calib.ptBoard[i][j][2].y = session.m_calib.ptBoard[i][n][3].y;
  session.m_calib.ptBoard[i][j][3].x = session.m_calib.ptBoard[m][j][0].x;
  session.m_calib.ptBoard[i][j][3].y = session.m_calib.ptBoard[m][j][0].y;
  
  // line 1
  x11 = (double)session.m_calib.ptBoard[i][n][0].x;
  y11 = (double)session.m_calib.ptBoard[i][n][0].y;
  x12 = (double)session.m_calib.ptBoard[i][n][1].x;
  y12 = (double)session.m_calib.ptBoard[i][n][1].y;
  dx1 = x12 - x11;
  dy1 = y12 - y11;

  // line 2
  x21 = (double)session.m_calib.ptBoard[m][j][0].x;
  y21 = (double)session.m_calib.ptBoard[m][j][0].y;
  x22 = (double)session.m_calib.ptBoard[m][j][3].x;
  y22 = (double)session.m_calib.ptBoard[m][j][3].y;
  dx2 = x22 - x21;
  dy2 = y22 - y21;

  // calculate intersection
  x = (y21 - y11 + dy1/dx1 * x11 - dy2/dx2 * x21) / (dy1/dx1 - dy2/dx2);
  y = dy1/dx1 * (x - x11) + y11;

  // set top-left point
  session.m_calib.ptBoard[i][j][0].x = cvRound(x);
  session.m_calib.ptBoard[i][j][0].y = cvRound(y);

  // - - - -
  // Top Right Corner
  // - - - -
  i = 0; j = nChessBoardDim-1; m = 1; n = j - 1;

  // shared points
  session.m_calib.ptBoard[i][j][0].x = session.m_calib.ptBoard[i][n][1].x;
  session.m_calib.ptBoard[i][j][0].y = session.m_calib.ptBoard[i][n][1].y;
  session.m_calib.ptBoard[i][j][2].x = session.m_calib.ptBoard[m][j][1].x;
  session.m_calib.ptBoard[i][j][2].y = session.m_calib.ptBoard[m][j][1].y;
  session.m_calib.ptBoard[i][j][3].x = session.m_calib.ptBoard[m][j][0].x;
  session.m_calib.ptBoard[i][j][3].y = session.m_calib.ptBoard[m][j][0].y;
  
  // line 1
  x11 = (double)session.m_calib.ptBoard[i][n][0].x;
  y11 = (double)session.m_calib.ptBoard[i][n][0].y;
  x12 = (double)session.m_calib.ptBoard[i][n][1].x;
  y12 = (double)session.m_calib.ptBoard[i][n][1].y;
  dx1 = x12 - x11;
  dy1 = y12 - y11;

  // line 2
  x21 = (double)session.m_calib.ptBoard[m][j][1].x;
  y21 = (double)session.m_calib.ptBoard[m][j][1].y;
  x22 = (double)session.m_calib.ptBoard[m][j][2].x;
  y22 = (double)session.m_calib.ptBoard[m][j][2].y;
  dx2 = x22 - x21;
  dy2 = y22 - y21;

  // calculate intersection
  x = (y21 - y11 + dy1/dx1 * x11 - dy2/dx2 * x21) / (dy1/dx1 - dy2/dx2);
  y = dy1/dx1 * (x - x11) + y11;

  // set top-left point
  session.m_calib.ptBoard[i][j][1].x = cvRound(x);
  session.m_calib.ptBoard[i][j][1].y = cvRound(y);
  
  // - - - -
  // Bottom Right Corner
  // - - - -
  i = nChessBoardDim-1; j = nChessBoardDim-1; m = i - 1; n = j - 1;

  // shared points
  session.m_calib.ptBoard[i][j][0].x = session.m_calib.ptBoard[i][n][1].x;
  session.m_calib.ptBoard[i][j][0].y = session.m_calib.ptBoard[i][n][1].y;
  session.m_calib.ptBoard[i][j][1].x = session.m_calib.ptBoard[m][j][2].x;
  session.m_calib.ptBoard[i][j][1].y = session.m_calib.ptBoard[m][j][2].y;
  session.m_calib.ptBoard[i][j][3].x = session.m_calib.ptBoard[i][n][2].x;
  session.m_calib.ptBoard[i][j][3].y = session.m_calib.ptBoard[i][n][2].y;
  
  // line 1
  x11 = (double)session.m_calib.ptBoard[i][n][2].x;
  y11 = (double)session.m_calib.ptBoard[i][n][2].y;
  x12 = (double)session.m_calib.ptBoard[i][n][3].x;
  y12 = (double)session.m_calib.ptBoard[i][n][3].y;
  dx1 = x12 - x11;
  dy1 = y12 - y11;

  // line 2
  x21 = (double)session.m_calib.ptBoard[m][j][1].x;
  y21 = (double)session.m_calib.ptBoard[m][j][1].y;
  x22 = (double)session.m_calib.ptBoard[m][j][2].x;
  y22 = (double)session.m_calib.ptBoard[m][j][2].y;
  dx2 = x22 - x21;
  dy2 = y22 - y21;

  // calculate intersection
  x = (y21 - y11 + dy1/dx1 * x11 - dy2/dx2 * x21) / (dy1/dx1 - dy2/dx2);
  y = dy1/dx1 * (x - x11) + y11;

  // set top-left point
  session.m_calib.ptBoard[i][j][2].x = cvRound(x);
  session.m_calib.ptBoard[i][j][2].y = cvRound(y);

  // - - - -
  // Bottom Left Corner
  // - - - -
  i = nChessBoardDim-1; j = 0; m = i - 1; n = 1;

  // shared points
  session.m_calib.ptBoard[i][j][0].x = session.m_calib.ptBoard[m][j][3].x;
  session.m_calib.ptBoard[i][j][0].y = session.m_calib.ptBoard[m][j][3].y;
  session.m_calib.ptBoard[i][j][1].x = session.m_calib.ptBoard[m][j][2].x;
  session.m_calib.ptBoard[i][j][1].y = session.m_calib.ptBoard[m][j][2].y;
  session.m_calib.ptBoard[i][j][2].x = session.m_calib.ptBoard[i][n][3].x;
  session.m_calib.ptBoard[i][j][2].y = session.m_calib.ptBoard[i][n][3].y;
  
  // line 1
  x11 = (double)session.m_calib.ptBoard[i][n][2].x;
  y11 = (double)session.m_calib.ptBoard[i][n][2].y;
  x12 = (double)session.m_calib.ptBoard[i][n][3].x;
  y12 = (double)session.m_calib.ptBoard[i][n][3].y;
  dx1 = x12 - x11;
  dy1 = y12 - y11;

  // line 2
  x21 = (double)session.m_calib.ptBoard[m][j][0].x;
  y21 = (double)session.m_calib.ptBoard[m][j][0].y;
  x22 = (double)session.m_calib.ptBoard[m][j][3].x;
  y22 = (double)session.m_calib.ptBoard[m][j][3].y;
  dx2 = x22 - x21;
  dy2 = y22 - y21;

  // calculate intersection
  x = (y21 - y11 + dy1/dx1 * x11 - dy2/dx2 * x21) / (dy1/dx1 - dy2/dx2);
  y = dy1/dx1 * (x - x11) + y11;

  // set top-left point
  session.m_calib.ptBoard[i][j][3].x = cvRound(x);
  session.m_calib.ptBoard[i][j][3].y = cvRound(y);
  

  // - - - -
  // Draw calibration
  // - - - -
  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.75, 0.75, 0, 1);

  for(i=0; i<nChessBoardDim; ++i)
  {
    for(j=0, n=0; j<nChessBoardDim; ++j)
    {
      if( (session.m_calib.ptBoard[i][j][0].x == 0) &&
          (session.m_calib.ptBoard[i][j][1].y == 0) )
      {
        continue;
      }

      cvPutText(pImg, StaleMateChessBoardCoordStr(i, j),
          session.m_calib.ptBoard[i][j][3], &font, colorText);

      pPolyLines[n] = session.m_calib.ptBoard[i][j];
      nPolyPts[n]   = 4;
      n++;
    }
    if( n > 0 )
    {
      cvPolyLine(pImg, pPolyLines, nPolyPts, n, 2, colorPoly,
          1, 8);
    }
  }

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImgGray);

  return true;
}

static bool SMCalibVideo(StaleMateSession &session)
{
  IplImage  *pImg;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return false;
  }

  if( session.m_calib.pImgEmptyRed != NULL )
  {
    cvReleaseImage(&session.m_calib.pImgEmptyRed);
  }
  session.m_calib.pImgEmptyRed = StaleMateCreateRgbChannel(pImg, CHANNEL_RED);

  if( session.m_calib.pImgEmptyBlue != NULL )
  {
    cvReleaseImage(&session.m_calib.pImgEmptyBlue);
  }
  session.m_calib.pImgEmptyBlue = StaleMateCreateRgbChannel(pImg, CHANNEL_BLUE);

  cvReleaseImage(&pImg);

  return true;
}

static bool SMCalibRoi(StaleMateSession &session, int nChessBoardDim)
{
  int   k;
  int   w1, w2;
  int   h1, h2;

  k = nChessBoardDim - 1;

  if( session.m_calib.ptBoard[0][0][0].x < session.m_calib.ptBoard[k][0][3].x )
  {
    session.m_calib.rectRoiBoard.x = session.m_calib.ptBoard[0][0][0].x;
  }
  else
  {
    session.m_calib.rectRoiBoard.x = session.m_calib.ptBoard[k][0][3].x;
  }

  if( session.m_calib.ptBoard[0][0][0].y < session.m_calib.ptBoard[0][k][1].y )
  {
    session.m_calib.rectRoiBoard.y = session.m_calib.ptBoard[0][0][0].y;
  }
  else
  {
    session.m_calib.rectRoiBoard.y = session.m_calib.ptBoard[0][k][1].y;
  }

  w1 = session.m_calib.ptBoard[0][k][1].x - session.m_calib.ptBoard[0][0][0].x;
  w2 = session.m_calib.ptBoard[k][k][2].x - session.m_calib.ptBoard[k][0][3].x;

  session.m_calib.rectRoiBoard.width = w1 >= w2? w1: w2;

  h1 = session.m_calib.ptBoard[k][0][3].y - session.m_calib.ptBoard[0][0][0].y;
  h2 = session.m_calib.ptBoard[k][k][2].y - session.m_calib.ptBoard[0][k][1].y;

  session.m_calib.rectRoiBoard.height = h1 >= h2? h1: h2;

  return true;
}

// positive x is forward, positive y is left
static bool SMCalibDist(StaleMateSession &session,
                        int               nChessBoardDim,
                        CvPoint2D32f     &ptDEBottom)
{
  CvPoint2D32f  ptOrig;
  double        x, y;
  int           i, j;

  // origin is the center of upper left corner chess square
  ptOrig.x = ptDEBottom.x + (double)nChessBoardDim * TuneChessSquareDim -  
                          TuneChessSquareDim / 2.0;
  ptOrig.y = ptDEBottom.y + (double)(nChessBoardDim)/2.0 * TuneChessSquareDim -
                          TuneChessSquareDim / 2.0;

  for(i=0, x = ptOrig.x; i<nChessBoardDim; ++i, x -= TuneChessSquareDim)
  {
    for(j=0, y = ptOrig.y; j<nChessBoardDim; ++j, y -= TuneChessSquareDim)
    {
      session.m_calib.ptHekDist[i][j].x = x;
      session.m_calib.ptHekDist[i][j].y = y;
    }
  }

  session.m_calib.ptDEBottom = ptDEBottom;

  return true;
}

static void SMCalibShowAnnotated(StaleMateSession &session)
{
  int         nDim = session.m_game.nChessBoardDim;
  IplImage   *pImg;
  CvScalar    colorText = CV_RGB(0, 64, 64);
  CvScalar    colorPoly = CV_RGB(64, 32, 255);
  CvScalar    colorRoi = CV_RGB(0, 32, 64);
  CvFont      font;
  char        bufText[32];
  CvPoint    *pPolyLines[nDim];
  int         nPolyPts[nDim];
  CvPoint     pt1, pt2;
  int         i, j, k;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return;
  }

  cvInitFont(&font, CV_FONT_HERSHEY_DUPLEX, 0.75, 0.75, 0, 1);

  pt1.x = 5;
  pt1.y = 25;
  cvPutText(pImg, "Calibration", pt1, &font, colorText);

  for(i=0; i<nDim; ++i)
  {
    for(j=0, k=0; j<nDim; ++j)
    {
      pt1 = session.m_calib.ptBoard[i][j][3];
      pt1.x += 3;
      pt1.y -= 3;
      cvPutText(pImg, StaleMateChessBoardCoordStr(i, j), pt1, &font, colorText);
      pPolyLines[k] = session.m_calib.ptBoard[i][j];
      nPolyPts[k]   = 4;
      k++;
    }
    cvPolyLine(pImg, pPolyLines, nPolyPts, nDim, 2, colorPoly, 2, 8);
  }

  pt1.x = session.m_calib.rectRoiBoard.x;
  pt1.y = session.m_calib.rectRoiBoard.y;
  pt2.x = pt1.x + session.m_calib.rectRoiBoard.width - 1;
  pt2.y = pt1.y + session.m_calib.rectRoiBoard.height - 1;
  cvRectangle(pImg, pt1, pt2, colorRoi, 1);

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);

  cvReleaseImage(&pImg);
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

void StaleMateCalib(StaleMateSession &session)
{
  session.m_calib.bCalibrated = false;

  if( !SMCalibBoardSquares(session, session.m_game.nChessBoardDim) )
  {
    return;
  }

  if( !SMCalibVideo(session) )
  {
    return;
  }

  if( !SMCalibRoi(session, session.m_game.nChessBoardDim) )
  {
    return;
  }

  if( !SMCalibDist(session, session.m_game.nChessBoardDim, 
                            session.m_calib.ptDEBottom) )
  {
    return;
  }
  
  SMCalibShowAnnotated(session);

  LOGDIAG1("Calibration complete.");

  session.m_calib.bCalibrated = true;
}
