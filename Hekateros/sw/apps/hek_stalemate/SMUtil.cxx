////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMUtil.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief StaleMate utilities.
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

void StaleMateIoIShow(StaleMateSession &session,
                      IplImage         *pIoI,
                      uint_t            uImgIndex)
{
  IplImage  **ppImg;

  // transform image of interest into gui image
  if( pIoI == NULL )
  {
    return;
  }

  if( uImgIndex == session.m_vid.uImgIndex0 )
  {
    ppImg = &session.m_vid.pImgDisplay0;
  }
  else if( uImgIndex == session.m_vid.uImgIndex1 )
  {
    ppImg = &session.m_vid.pImgDisplay1;
  }
  else
  {
    LOGERROR("Unknow image index %u.", uImgIndex);
    return;
  }

  // clear display image
  //cvZero(*ppImg);

  //
  // Image of Interest must be size of target display image.
  //
  if( (pIoI->width != (*ppImg)->width) || (pIoI->height != (*ppImg)->height) )
  {
    LOGERROR("cvSize(IoI)=(%d,%d) != cvSize(ImgDisplay)=(%d,%d).",
      pIoI->width, pIoI->height,
      (*ppImg)->width, (*ppImg)->height);
    return;
  }

  //
  // Image properties are different, adjust display image to new properties.
  //
  if( (pIoI->nChannels != (*ppImg)->nChannels) ||
      (pIoI->depth != (*ppImg)->depth) )
  {
    cvReleaseImage(ppImg);
    *ppImg = cvCloneImage(pIoI);
  }

  session.m_vid.pVidToGuiTrans->Trans(pIoI, *ppImg);

  // show image
  session.m_gui.pWin->ShowCvImage(*ppImg, uImgIndex);
}

IplImage *StaleMateCreateRgbChannel(IplImage *pImg, int nRgbChannel)
{
  IplImage *pImgGray  = cvCreateImage(cvGetSize(pImg), IPL_DEPTH_8U, 1); 

  switch(nRgbChannel)
  {
    case CHANNEL_RED:
      cvSplit(pImg, NULL, NULL, pImgGray, NULL);
      break;
    case CHANNEL_GREEN:
      cvSplit(pImg, NULL, pImgGray, NULL, NULL);
      break;
    case CHANNEL_BLUE:
      cvSplit(pImg, pImgGray, NULL, NULL, NULL);
      break;
    case CHANNEL_GRAY:
    default:
      cvCvtColor(pImg, pImgGray, CV_RGB2GRAY);
      break;
  }

  return pImgGray;
}

IplImage *StaleMateCreateHsvChannel(IplImage *pImg, int nHsvChannel)
{
  IplImage *pImgGray  = cvCreateImage(cvGetSize(pImg), IPL_DEPTH_8U, 1); 

  switch(nHsvChannel)
  {
    case CHANNEL_HUE:
      cvSplit(pImg, pImgGray, NULL, NULL, NULL);
      break;
    case CHANNEL_SATURATION:
      cvSplit(pImg, NULL, pImgGray, NULL, NULL);
      break;
    case CHANNEL_VALUE:
      cvSplit(pImg, NULL, NULL, pImgGray, NULL);
      break;
    case CHANNEL_GRAY:
    default:
      cvZero(pImgGray);
      break;
  }

  return pImgGray;
}

void StaleMateReadCfg(StaleMateSession &session)
{
  enum record_t
  {
    RecPark, RecHome, RecMotPlanner,
    RecGripOpen, RecGripNarrow, RecGripGrab, RecGripClose
  };

  static const char *sFile = "/prj/pkg/Hekateros/share/stalemate.cfg";

  FILE     *fp;
  char      line[256];
  char     *s;
  int       n;
  int       order;
  int       rc = HEK_OK;

  if( (fp = fopen(sFile, "r")) == NULL )
  {
    LOGDIAG1("%s: No file.", sFile);
    return;
  }

  order = RecPark;

  while( ((s = fgets(line, sizeof(line), fp)) != NULL) && (rc == HEK_OK) )
  {
    if( (*s == '#') || isspace((int)*s) )
    {
      continue;
    }

    switch( order )
    {
      case RecPark:
        n = sscanf(s, "%d %d %d %d %d %d",
                  &HekKeyPosPark[0].m_nPos, &HekKeyPosPark[1].m_nPos,
                  &HekKeyPosPark[2].m_nPos, &HekKeyPosPark[3].m_nPos,
                  &HekKeyPosPark[4].m_nPos, &HekKeyPosPark[5].m_nPos);
        if( n != SM_HEK_NSERVOS_TOTAL_M )
        {
          LOGERROR("%s: Park position: Only %d servo values read.", sFile, n);
          rc = -HEK_ECODE_BAD_VAL;
        }
        else
        {
          session.m_hek.bParkDefined = true;
        }
        break;
      case RecHome:
        n = sscanf(s, "%d %d %d %d %d %d",
                  &HekKeyPosHome[0].m_nPos, &HekKeyPosHome[1].m_nPos,
                  &HekKeyPosHome[2].m_nPos, &HekKeyPosHome[3].m_nPos,
                  &HekKeyPosHome[4].m_nPos, &HekKeyPosHome[5].m_nPos);
        if( n != SM_HEK_NSERVOS_TOTAL_M )
        {
          LOGERROR("%s: Home position: Only %d servo values read.", sFile, n);
          rc = -HEK_ECODE_BAD_VAL;
        }
        else
        {
          session.m_hek.bHomeDefined = true;
        }
        break;
      case RecMotPlanner:
        n = sscanf(s, "%d %d %d %d %d",
                &HekKeyPosMotPlanner[0].m_nPos, &HekKeyPosMotPlanner[1].m_nPos,
                &HekKeyPosMotPlanner[2].m_nPos, &HekKeyPosMotPlanner[3].m_nPos,
                &HekKeyPosMotPlanner[4].m_nPos);
        if( n != SM_HEK_NSERVOS_BASE_M )
        {
          LOGERROR("%s: Motion planner position: Only %d servo values read.",
              sFile, n);
          rc = -HEK_ECODE_BAD_VAL;
        }
        else
        {
          session.m_hek.motionPlanner.setOffsets(HekKeyPosMotPlanner, n);
        }
        break;
      case RecGripOpen:
        if( (n = sscanf(s, "%d", &HekKeyPosGripperOpen.m_nPos)) != 1 )
        {
          LOGERROR("%s: Gripper open position: Cannot read value.", sFile);
          rc = -HEK_ECODE_BAD_VAL;
        }
        break;
      case RecGripNarrow:
        if( (n = sscanf(s, "%d", &HekKeyPosGripperNarrow.m_nPos)) != 1 )
        {
          LOGERROR("%s: Gripper narrow position: Cannot read value.", sFile);
          rc = -HEK_ECODE_BAD_VAL;
        }
        break;
      case RecGripGrab:
        if( (n = sscanf(s, "%d", &HekKeyPosGripperGrab.m_nPos)) != 1 )
        {
          LOGERROR("%s: Gripper grab position: Cannot read value.", sFile);
          rc = -HEK_ECODE_BAD_VAL;
        }
        break;
      case RecGripClose:
        if( (n = sscanf(s, "%d", &HekKeyPosGripperClose.m_nPos)) != 1 )
        {
          LOGERROR("%s: Gripper close position: Cannot read value.", sFile);
          rc = -HEK_ECODE_BAD_VAL;
        }
        break;
      default:
        break;
    }
    ++order;
  }

  if( rc != HEK_OK )
  {
    LOGERROR("%s: Bad format - aborted parse.");
  }
  else
  {
    LOGDIAG1("%s: StaleMate configuration successfully read.", sFile);
  }

  fclose(fp);
}
