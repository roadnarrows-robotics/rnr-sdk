////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      State.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-02-18 12:42:09 -0700 (Mon, 18 Feb 2013) $
 * $Rev: 2691 $
 *
 * \brief State base class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS, EMPLOYEES or
// CONTRACTORS OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO
// ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 */
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdarg.h>
#include <libgen.h>

#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "WM/WM.h"
#include "WM/WMCamera.h"
#include "WM/WMWin.h"
#include "WM/WMLookFeel.h"
#include "WM/WMSession.h"
#include "WM/WMState.h"


using namespace std;
using namespace WM;
using namespace WMWin;


//------------------------------------------------------------------------------
// WMState Class
//------------------------------------------------------------------------------

/*!
 * \brief Default intialization constructor.
 */
WMState::WMState(WMSession &session, int nInitAction)
{
  m_pWin            = session.m_pWin;
  m_pMenu           = NULL;
  m_nInitAction     = nInitAction;
  m_nCurAction      = WMUIActionNone;
  m_bHardBttnEnable = true;
  m_bFatal          = false;
  m_bModified       = false;
}

/*!
 * \brief Default destructor.
 */
WMState::~WMState()
{
  destroyInterface();
}

 /*!
  * \brief Get the next input action into this state.
  *
  * Actions: Soft button push.
  *
  * The retrieved next action becomes the current action.
  *
  * \param session Session data.
  *
  * \return Next action.
  */
int WMState::getNextAction(WMSession &session, int nTimeoutMs)
{
  session.m_pWin->WaitEvent(nTimeoutMs);

  // user action
  if( m_pMenu != NULL )
  {
    m_nCurAction = m_pMenu->GetCurrentAction();
    m_pMenu->SetCurrentAction(WMUIActionNone);
  }
  else
  {
    m_nCurAction = WMUIActionNone;
  }
  return m_nCurAction;
}

void WMState::buildInterface(WMSession &session)
{
  // one-time initialization of button menu
  initButtonMenu();

  // bind menu to window and set initial action
  if( m_pMenu != NULL )
  {
    m_pMenu->Bind(session.m_pWin);
    m_nCurAction = m_pMenu->SetCurrentAction(m_nInitAction);
  }

  // page identifier
  session.m_pWin->ShowPageId(session.GetPageNum());

  // enable/disable/toggle menu button states
  setMenuStates(session);

  // set local state context in session
  session.SetContext(this);
}

void WMState::destroyInterface()
{
  if( m_pMenu != NULL )
  {
    m_pMenu->Unbind();
    m_pMenu = NULL;
  }

  // remove all widgets from workspace
  m_pWin->WorkspaceRemoveAll();
}

/*!
 * \brief Test if hard button is pushed.
 *
 * \param session Session data.
 *
 * \return Returns true or false.
 */
bool WMState::isHardButtonPushed(WMSession &session)
{
#ifdef WZCSB
  uint_t key = session.m_pWin->GetLastKeyPress() & 0x00ff;
  LOGDIAG4("Key 0x%x pressed. Camera key=0x%x.", key, WMHardButtonCtlKey);
  return key == WMHardButtonCtlKey ? true: false;
#elif defined ARCH_overo
  return WMHardButtonCtlGetState();
#else
  uint_t key = session.m_pWin->GetLastKeyPress() & 0x00ff;
  LOGDIAG4("Key 0x%x pressed. Camera key=0x%x.", key, WMHardButtonCtlKey);
  return key == (uint_t)'c' ? true: false;    // click
#endif // ARCH_OVERO
}


//------------------------------------------------------------------------------
// WMStateImage Class
//------------------------------------------------------------------------------

/*!
 * \brief Default intialization constructor.
 */
WMStateImage::WMStateImage(WMSession    &session,
                           const CvSize &sizeDisplay,
                           const CvSize &sizeIoI,
                           int          nInitAction) :
    WMState(session, nInitAction)
{
  m_pImgIoI         = NULL;
  m_pImgShadow      = NULL;
  m_pImgMarked      = NULL;
  m_pImgDisplay     = NULL;
  m_sizeDisplay     = sizeDisplay;
  m_sizeIoI         = sizeIoI;
  m_pTransIoI       = NULL;
  m_pTransShadow    = NULL;
  m_bIoICanZoomIn   = true;
  m_bIoICanZoomOut  = false;
  m_bIoIIsPanning   = false;
  m_bShadowChanged  = false;
}

/*!
 * \brief Default destructor.
 */
WMStateImage::~WMStateImage()
{
  destroyInterface();
}

/*!
 * \brief Get the next input action into this state.
 *
 * Actions: Soft button push.
 * Actions: Mouse-over-image click and drag events.
 *
 * The retrieved next action becomes the current action.
 *
 * \param session Session data.
 *
 * \return Next action.
 */
int WMStateImage::getNextAction(WMSession &session, int nTimeoutMs)
{
  session.m_pWin->WaitEvent(nTimeoutMs);

  if( m_pMenu == NULL )
  {
    m_nCurAction = WMUIActionNone;
  }
  else
  {
    if( (m_nCurAction = m_pMenu->GetCurrentAction()) == WMUIActionNone )
    {
      m_nCurAction = m_mouse.GetCurrentAction();
    }
    m_pMenu->SetCurrentAction(WMUIActionNone);
    m_mouse.SetCurrentAction(WMUIActionNone);
  }
  return m_nCurAction;
}
 
void WMStateImage::buildInterface(WMSession &session)
{
  // build base
  WMState::buildInterface(session);

  // workspace is an opencv image
  session.m_pWin->WorkspaceSetAsCvImage();

  // bind mouse to window workspace
  m_mouse.Bind(session.m_pWin);

  // no current image of interest
  m_pImgIoI = NULL;

  // shadow (low-res) image
  m_pImgShadow = cvCreateImage(m_sizeDisplay, IPL_DEPTH_8U, 3);

  // image displayed to user
  m_pImgDisplay = cvCreateImage(m_sizeDisplay, IPL_DEPTH_8U, 3);

  // clear display image
  cvSet(m_pImgDisplay, GuiRgbImageBg);

  // show blank image
  session.m_pWin->ShowCvImage(m_pImgDisplay);

  // no marked up image
  m_pImgMarked = NULL;

  // image of interest default region of interest
  m_rectIoIRoI    = cvRect(0, 0, m_sizeIoI.width, m_sizeIoI.height);
  m_bShadowChanged = true;

  // displayed image of interest window transformation object
  m_pTransIoI = new WMWinIoI(m_sizeIoI, m_sizeDisplay,
                             WMImgTransRot, WMImgTransAlign, WMImgTransCrop);

  // displayed shadow window transformation object
  m_pTransShadow = new WMWinIoI(m_sizeDisplay, m_sizeDisplay,
                             WMImgTransRot, WMImgTransAlign, WMImgTransCrop);
}

void WMStateImage::destroyInterface()
{
  m_mouse.Unbind();

  if( m_pImgShadow != NULL )
  {
    cvReleaseImage(&m_pImgShadow);    // shadow (low-res) IoI
    m_pImgShadow = NULL;
  }

  if( m_pImgDisplay != NULL )
  {
    cvReleaseImage(&m_pImgDisplay);   // release displayed image
    m_pImgDisplay = NULL;
  }

  if( m_pTransIoI != NULL )
  {
    delete m_pTransIoI;
    m_pTransIoI = NULL;
  }

  if( m_pTransShadow != NULL )
  {
    delete m_pTransShadow;
    m_pTransShadow = NULL;
  }
}

/*!
 * \brief Reset image view to defaults.
 *
 * All zoom and pan settings are set to normal.
 *
 * \param session   Session data.
 */
void WMStateImage::resetView(WMSession &session)
{
  // image of interest default region of interest
  m_rectIoIRoI = cvRect(0, 0, m_sizeIoI.width, m_sizeIoI.height);
  m_bShadowChanged = true;

  // edit logic
  m_bIoICanZoomIn   = true;
  m_bIoICanZoomOut  = false;
  m_bIoIIsPanning   = false;

  // image tranformation parameters
  m_pTransIoI->SetTransParams(m_sizeIoI,
                              m_rectIoIRoI,
                              m_sizeDisplay,
                              WMImgTransRot,
                              WMImgTransAlign,
                              WMImgTransCrop);
}

/*!
 *  \brief Show pan region of interest 
 */
void WMStateImage::markupPanImg(WMSession &session)
{
  CvRect shadowROI = mapRect(m_rectIoIRoI);
  CvScalar brighter = cvScalar(80);

  cvSetImageROI(m_pImgMarked, shadowROI);
  cvAddS(m_pImgMarked, brighter, m_pImgMarked);
  cvResetImageROI(m_pImgMarked);
}

/*!
 * \brief Show transformed marked up image to display.
 *
 * Calls virtual function markupImg() to perform task 
 * specific markups.
 *
 * \param session       Session data.
 */
void WMStateImage::showMarkupImg(WMSession &session)
{
  // if region of interest changed, copy new shadow image
  if( m_bShadowChanged )
  {
    LOGDIAG3("Updating shadow image.");
    if(m_bIoIIsPanning)
    {
      cvResize(m_pImgIoI, m_pImgShadow);
    } 
    else
    {
      cvSetImageROI(m_pImgIoI, m_rectIoIRoI);
      cvResize(m_pImgIoI, m_pImgShadow);
      cvResetImageROI(m_pImgIoI);
    }

    m_bShadowChanged = false;
  }

  cvCopy(m_pImgShadow, m_pImgDisplay);
  m_pImgMarked = m_pImgDisplay;

  // mark up
  markupImg(session);

  if (m_bIoIIsPanning)
  {
    markupPanImg(session);
  }

  // show
  // DHP
  // showImg(session, m_pImgMarked);
  session.m_pWin->ShowCvImage(m_pImgDisplay);
}

/*!
 * \brief Show transformed image on display.
 *
 * \param session       Session data.
 * \param pImg          Image.
 */
void WMStateImage::showImg(WMSession &session, IplImage *pImg)
{
  // clear display image
  cvSet(m_pImgDisplay, GuiRgbImageBg);

  // transform and place captured frame on display image
  if( pImg != NULL )
  {
    // m_pTransIoI->WMWinIoITrans(pImg, m_pImgDisplay);
    LOGDIAG4("Resolution of displayed image: %d, %d", 
                                 pImg->width, pImg->height);
    m_pTransShadow->WMWinIoITrans(pImg, m_pImgDisplay);
  }

  // show image
  session.m_pWin->ShowCvImage(m_pImgDisplay);
}

/*
 * \brief Translate image points from high-res to shadow image
 *
 * Transformation utility function.
 *
 * \param CvPoint& ptHiRes
 */
CvPoint WMStateImage::mapPt(CvPoint& ptHiRes)
{
  double fScale;
  CvPoint ptShadow;   ///< point in shadow image coordinates

  // subtract rectIoIRoI corner to get coords in rectIoIRoI 
  ptShadow.x = ptHiRes.x - m_rectIoIRoI.x;
  ptShadow.y = ptHiRes.y - m_rectIoIRoI.y;

  // find scale from rect coords to shadow res
  fScale = (1.0 * (double)m_sizeDisplay.width) / (double)m_rectIoIRoI.width;

  ptShadow.x = (int)(fScale*ptShadow.x);
  ptShadow.y = (int)(fScale*ptShadow.y);
  
  return ptShadow;
}

/*
 * \brief Translate image points from high-res to panning shadow image
 *
 * Transformation utility function.
 *
 * \param CvPoint& ptHiRes
 */
CvPoint WMStateImage::mapPtPan(CvPoint& ptHiRes)
{
  double fScale ;
  CvPoint ptShadow;   ///< point in shadow image coordinates

  // find scale from rect coords to shadow res
  fScale = (1.0 * m_sizeDisplay.width) / m_sizeIoI.width;

  ptShadow.x = (int)(fScale*ptHiRes.x);
  ptShadow.y = (int)(fScale*ptHiRes.y);
  
  return ptShadow;
}

/*
 * \brief Translate CvRect from high-res to shadow image
 *
 * Transformation utility function.
 *
 * \param CvRect& rectHiRes
 */
CvRect WMStateImage::mapRect(CvRect& rectHiRes)
{
  double fScale;
  CvRect rectShadow;   ///< point in shadow image coordinates

  fScale = (1.0 * m_sizeDisplay.width) / m_sizeIoI.width;

  rectShadow.x      = (int)(rectHiRes.x*fScale);
  rectShadow.y      = (int)(rectHiRes.y*fScale);
  rectShadow.width  = (int)(rectHiRes.width*fScale);
  rectShadow.height = (int)(rectHiRes.height*fScale);

  return rectShadow;
}

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// State Trasition Functions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Zoom in to the current marked up image.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateImage::transZoomIn(WMSession &session)
{
  CvSize        sizeZoom;     ///< working zoom size
  int           x, y;         ///< working coordinates

  if( !m_bIoICanZoomIn )
  {
    return;
  }

  sizeZoom = ar43width((int)(m_rectIoIRoI.width * 0.9));

  if( (sizeZoom.width <= m_sizeDisplay.width) &&
      (sizeZoom.height <= m_sizeDisplay.height) )
  {
    sizeZoom = ar43width(m_sizeDisplay);
    m_bIoICanZoomIn = false;
  }

  // x = (m_sizeIoI.width - sizeZoom.width) / 2;
  // y = (m_sizeIoI.height - sizeZoom.height) / 2;

  x = m_rectIoIRoI.x + (m_rectIoIRoI.width / 2) - (sizeZoom.width /2);
  y = m_rectIoIRoI.y + (m_rectIoIRoI.height/ 2) - (sizeZoom.height/2);

  m_rectIoIRoI = cvRect(x, y, sizeZoom.width, sizeZoom.height);
  m_bShadowChanged = true;

  if ( !m_bIoIIsPanning )
  {
  m_pTransIoI->SetTransParams(m_sizeIoI,
                              m_rectIoIRoI,
                              m_sizeDisplay,
                              WMImgTransRot,
                              WMImgTransAlign,
                              WMImgTransCrop);
  }

  showMarkupImg(session);

  m_bIoICanZoomOut = true;

  setModifiedState(true);
}

/*!
 * \brief Zoom out of the current marked up image.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateImage::transZoomOut(WMSession &session)
{
  CvSize        sizeZoom;     ///< working zoom size
  int           x, y;         ///< working coordinates

  if( !m_bIoICanZoomOut )
  {
    return;
  }

  sizeZoom = ar43width((int)(m_rectIoIRoI.width * 1.1));

  if( (sizeZoom.width >= m_sizeIoI.width) ||
      (sizeZoom.height >= m_sizeIoI.height) )
  {
    sizeZoom = m_sizeIoI;
    m_bIoICanZoomOut = false;
  }

  // RoI remains centered at current location
  x = m_rectIoIRoI.x + (m_rectIoIRoI.width / 2) - (sizeZoom.width /2);
  y = m_rectIoIRoI.y + (m_rectIoIRoI.height/ 2) - (sizeZoom.height/2);
  
  // UNLESS RoI goes outside bounds of image
  if( x + sizeZoom.width  > m_sizeIoI.width ) 
  {
    x = (m_sizeIoI.width - sizeZoom.width);
  } 
  else if ( x < 0 ) 
  {
    x = 0;
  } 

  if( y + sizeZoom.height  > m_sizeIoI.height ) 
  {
    y = (m_sizeIoI.height - sizeZoom.height);
  } 
  else if ( y < 0 ) 
  {
    y = 0;
  } 

  m_rectIoIRoI = cvRect(x, y, sizeZoom.width, sizeZoom.height);
  m_bShadowChanged = true;

  if ( !m_bIoIIsPanning ) 
  {
  m_pTransIoI->SetTransParams(m_sizeIoI,
                              m_rectIoIRoI,
                              m_sizeDisplay,
                              WMImgTransRot,
                              WMImgTransAlign,
                              WMImgTransCrop);
  }

  showMarkupImg(session);

  m_bIoICanZoomIn = true;

  setModifiedState(true);
}

/*!
 * \brief Pan marked up image up/down and left/right.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateImage::transPan(WMSession &session)
{
  CvPoint         ptRaw;        // this raw point position
  CvPoint         ptImg;        // this transformed point position
  int             nAction;      // current mouse action
  int             x, y;         // working coordinates
  CvRect          shadowRoI;    // RoI in shadow image

  ptRaw = m_mouse.GetMousePoint();

  // out of scope
  if( (ptRaw.x < 0) || (ptRaw.x >= m_sizeDisplay.width) ||
      (ptRaw.y < 0) || (ptRaw.y >= m_sizeDisplay.height) )
  {
    //DHP: log this
    LOGDIAG4("Mouse pt out of scope for panning.");

    //DHP: show panning image for first pass through
    showMarkupImg(session);
    return;
  }

  ptImg = m_pTransIoI->WMWinIoIMapPoint(ptRaw);
  // ptImg = m_pTransShadow->WMWinIoIMapPoint(ptRaw);

  //cerr << "DBG: (" << ptRaw.x << "," << ptRaw.y << ") "
  //  << "==> (" << ptImg.x << "," << ptImg.y << ")" << endl;

  nAction = getCurAction();

  if( nAction == WMUIActionDragStart )
  {
    LOGDIAG4("Drag Start");
    m_ptPanRaw = ptRaw;
    m_ptPan    = ptImg;
    return;
  }
  else if( nAction == WMUIActionDragEnd )
  {
    LOGDIAG4("Drag End.");
    return;
  }

  // Ignore hover _near_ same spot
  if(( abs(m_ptPanRaw.x - ptRaw.x) < 10 ) && (abs(m_ptPanRaw.y - ptRaw.y) < 10))
  {
    return;
  }

  x = m_rectIoIRoI.x - m_ptPan.x + ptImg.x;
  y = m_rectIoIRoI.y - m_ptPan.y + ptImg.y;

  if( x < 0 )
  {
    x = 0;
  }
  else if( x + m_rectIoIRoI.width > m_sizeIoI.width )
  {
    x = m_sizeIoI.width - m_rectIoIRoI.width;
  }

  if( y < 0 )
  {
    y = 0;
  }
  else if( y + m_rectIoIRoI.height > m_sizeIoI.height )
  {
    y = m_sizeIoI.height - m_rectIoIRoI.height;
  }

  m_rectIoIRoI.x = x;
  m_rectIoIRoI.y = y;

  // while panning, set temporary RoI to entire high-res image
  CvRect tmpRoI = cvRect(0,0, m_sizeIoI.width, m_sizeIoI.height);
  m_pTransIoI->SetTransParams(m_sizeIoI,
                              tmpRoI,
                              m_sizeDisplay,
                              WMImgTransRot,
                              WMImgTransAlign,
                              WMImgTransCrop);

  m_ptPanRaw = ptRaw;
  m_ptPan    = ptImg;

  showMarkupImg(session);

  setModifiedState(true);
}


//------------------------------------------------------------------------------
// WMStateCvCamera Class
//------------------------------------------------------------------------------

/*!
 * \brief Default intialization constructor.
 */
WMStateCvCamera::WMStateCvCamera(WMSession       &session,
                                 const CvSize    &sizeDisplay,
                                 const CameraRes eVideoRes,
                                 const CameraRes eImageRes,
                                 int             nInitAction) :
    WMState(session, nInitAction),
    m_camera(session.m_sVidDevName, session.m_nVideoIndex, eVideoRes, eImageRes)
{
  m_pImgMarked      = NULL;
  m_pImgDisplay     = NULL;
  m_sizeDisplay     = sizeDisplay;
  m_eVideoRes       = eVideoRes;
  m_pTransIoI       = NULL;
  m_bHardBttnEnable = true;
  m_bPaused         = false;
}

/*!
 * \brief Default destructor.
 */
WMStateCvCamera::~WMStateCvCamera()
{
  destroyInterface();
}

/*!
 * \brief Get the next input action into this state.
 *
 * Actions: Soft button push.
 * Actions: Hard button push.
 *
 * The retrieved next action becomes the current action.
 *
 * \param session Session data.
 *
 * \return Next action.
 */
int WMStateCvCamera::getNextAction(WMSession &session, int nTimeoutMs)
{
  session.m_pWin->WaitEvent(nTimeoutMs);

  if( m_pMenu == NULL )
  {
    m_nCurAction = WMUIActionNone;
  }
  else
  {
    if( (m_nCurAction = m_pMenu->GetCurrentAction()) == WMUIActionNone )
    {
      if( m_bHardBttnEnable && isHardButtonPushed(session) )
      {
        m_nCurAction = WMUIActionHardButtonPush;
      }
    }
    m_pMenu->SetCurrentAction(WMUIActionNone);
  }
  return m_nCurAction;
}
 
void WMStateCvCamera::buildInterface(WMSession &session)
{
  // build base
  WMState::buildInterface(session);

  // workspace is an opencv image
  session.m_pWin->WorkspaceSetAsCvImage();

  // image displayed to user
  m_pImgDisplay = cvCreateImage(m_sizeDisplay, IPL_DEPTH_8U, 3);

  // clear display image
  cvSet(m_pImgDisplay, GuiRgbImageBg);

  // show blank image
  session.m_pWin->ShowCvImage(m_pImgDisplay);

  // no marked up image
  m_pImgMarked = NULL;

  // start video capture
  m_camera.startCamera(m_eVideoRes);

  if( m_camera.isCameraRunning() )
  {
    m_pTransIoI = new WMWinIoI(m_camera.getVideoResolution(),
                               m_sizeDisplay,
                               WMImgTransRot,
                               WMImgTransAlign,
                               WMImgTransCrop);
  }

  else
  {
    m_pTransIoI = NULL;
    setFatal();
  }
}

void WMStateCvCamera::destroyInterface()
{
  if( m_pImgMarked != NULL )
  {
    cvReleaseImage(&m_pImgMarked);  // laser marked image
    m_pImgMarked = NULL;
  }

  if( m_pImgDisplay != NULL )
  {
    cvReleaseImage(&m_pImgDisplay);   // release displayed image
    m_pImgDisplay = NULL;
  }

  if( m_pTransIoI != NULL )
  {
    delete m_pTransIoI;
    m_pTransIoI = NULL;
  }
}

/*!
 * \brief Clone current video frame.
 *
 * \param session   Session data.
 */
void WMStateCvCamera::cloneFrame(WMSession &session)
{
  IplImage  *pImgFrame;

  // released old marked up image
  if( m_pImgMarked != NULL )
  {
    cvReleaseImage(&m_pImgMarked);
    m_pImgMarked = NULL;
  }

  // clone calibration image
  if( (pImgFrame = m_camera.getCurFrame()) != NULL )
  {
    m_pImgMarked = cvCloneImage(pImgFrame);
  }
}

/*!
 * \brief Show transformed marked up video frame clone to display.
 *
 * Calls virtual function markupImg() to perform task specific markups.
 *
 * \param session       Session data.
 */
void WMStateCvCamera::showMarkupImg(WMSession &session)
{
  // clone current calibration image
  cloneFrame(session);

  // mark up
  markupImg(session);

  // show
  showImg(session, m_pImgMarked);
}

/*!
 * \brief Show transformed image on display.
 *
 * \param session       Session data.
 * \param pImg          Image.
 */
void WMStateCvCamera::showImg(WMSession &session, IplImage *pImg)
{
  // clear display image
  cvSet(m_pImgDisplay, GuiRgbImageBg);

  // transform and place captured frame on display image
  if( pImg != NULL )
  {
    m_pTransIoI->WMWinIoITrans(pImg, m_pImgDisplay);
  }

  // show image
  session.m_pWin->ShowCvImage(m_pImgDisplay);
}

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// State Trasition Functions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Show video frame to display.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateCvCamera::transRunVideo(WMSession &session)
{
  IplImage    *pImgFrame;

  if( (pImgFrame = m_camera.grabFrame()) == NULL )
  {
    session.PushFatalState("Failed to grab video frame.");
  }
  else
  {
    showImg(session, pImgFrame);
  }
}

/*!
 * \brief Show video frame with markup overlay to display.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateCvCamera::transRunMarkupVideo(WMSession &session)
{
  IplImage    *pImgFrame;

  if( (pImgFrame = m_camera.grabFrame()) == NULL )
  {
    session.PushFatalState("Failed to grab video frame.");
  }
  else
  {
    showMarkupImg(session);
  }
}

/*!
 * \brief Stop video.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateCvCamera::transStopVideo(WMSession &session)
{
  m_camera.stopCamera();
}


//------------------------------------------------------------------------------
// WMStateGstCamera Class
//------------------------------------------------------------------------------

/*!
 * \brief Default intialization constructor.
 */
WMStateGstCamera::WMStateGstCamera(WMSession       &session,
                                   const CvSize    &sizeDisplay,
                                   const CameraRes eVideoRes,
                                   const CameraRes eImageRes,
                                   int             nInitAction) :
    WMState(session, nInitAction),
    m_camera(session.m_sVidDevName, session.m_nVideoIndex, eVideoRes, eImageRes)
{
  m_sizeVidWin      = sizeDisplay;
  m_bHardBttnEnable = true;
  m_bPaused         = false;
}

/*!
 * \brief Default destructor.
 */
WMStateGstCamera::~WMStateGstCamera()
{
  destroyInterface();
}

/*!
 * \brief Get the next input action into this state.
 *
 * Actions: Soft button push.
 * Actions: Hard button push.
 *
 * The retrieved next action becomes the current action.
 *
 * \param session Session data.
 *
 * \return Next action.
 */
int WMStateGstCamera::getNextAction(WMSession &session, int nTimeoutMs)
{
  session.m_pWin->WaitEvent(nTimeoutMs);

  if( m_pMenu == NULL )
  {
    m_nCurAction = WMUIActionNone;
  }
  else
  {
    if( (m_nCurAction = m_pMenu->GetCurrentAction()) == WMUIActionNone )
    {
      if( m_bHardBttnEnable && isHardButtonPushed(session) )
      {
        m_nCurAction = WMUIActionHardButtonPush;
      }
    }
    m_pMenu->SetCurrentAction(WMUIActionNone);
  }
  return m_nCurAction;
}
 
void WMStateGstCamera::buildInterface(WMSession &session)
{
  // build base
  WMState::buildInterface(session);

  // workspace is an opencv image
  session.m_pWin->WorkspaceSetAsGstWin(m_sizeVidWin);

  // set camera's X-window view window id
  m_camera.setXid(session.m_pWin->GetGstXid());

  // start video capture
  m_camera.startCamera();
}

void WMStateGstCamera::destroyInterface()
{
}

/*!
 * \brief Clone current video frame.
 *
 * \todo Clone GST frame.
 *
 * \param session   Session data.
 */
void WMStateGstCamera::cloneFrame(WMSession &session)
{
}

/*!
 * \brief Show transformed marked up video frame clone to display.
 *
 * Calls virtual function markupImg() to perform task specific markups.
 *
 * \param session       Session data.
 */
void WMStateGstCamera::showMarkupImg(WMSession &session)
{
  // mark up and show
  markupImg(session);
}

/*!
 * \brief Show transformed image on display.
 *
 * \param session       Session data.
 * \param pImg          Image.
 */
void WMStateGstCamera::showImg(WMSession &session, IplImage *pImg)
{
}

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// State Trasition Functions
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Show video frame to display.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateGstCamera::transRunVideo(WMSession &session)
{
  // nothing to do: its all done with gobject callbacks
}

/*!
 * \brief Show video frame with markup overlay to display.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateGstCamera::transRunMarkupVideo(WMSession &session)
{
}

/*!
 * \brief Stop video.
 *
 * State transition function.
 *
 * \param session   Session data.
 */
void WMStateGstCamera::transStopVideo(WMSession &session)
{
  m_camera.stopCamera();
}
