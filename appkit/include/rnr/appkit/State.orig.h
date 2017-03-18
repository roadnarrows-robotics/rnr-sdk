////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      State.h
//
/*! \file
 *
 * $LastChangedDate: 2013-03-24 08:20:22 -0600 (Sun, 24 Mar 2013) $
 * $Rev: 2782 $
 *
 * \brief Workflow state class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
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
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_STATE_H
#define _RNR_STATE_H

#include <vector>

#include "rnr/rnrconfig.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/Camera.h"
#include "rnr/win.h"
#include "rnr/Session.h"

namespace rnr
{
  // class StateMach
  // addState(State)
  // addStates(State, ...)
  // setStartState(State)
  // state enums start, terminate, this
  // getCurStateId()
  // getPrevStateId()
  // start()

  //----------------------------------------------------------------------------
  // State Class
  //----------------------------------------------------------------------------
  
  // data: ref, name, value, transition table
  // receiveEvent()
  // newstate = dispatchEvent()
  // fnGuard
  // fnTrans
  // fnTransDefault
  // getStateId()
  // getRefId()
  // getStateName()
  // getCurEvent()
  // queueNextEvent()
  
  /*!
   * Workflow Simplified State Machine Class
   */
  class State
  {
  public:
    State(Session &session, int nInitAction=UIActionNone);
  
    virtual ~State();
  
    virtual int getNextAction(Session &session, int nTimeoutMs);
  
    /*!
     * \brief Set the next input action to process.
     *
     * Subsequent call to getNextAction() will retrieve this action.
     *
     * \return The to-be-retrieved next action.
     */
    virtual int setNextAction(int nAction)
    {
      if( m_pMenu != NULL )
      {
        m_pMenu->SetCurrentAction(nAction);
      }
    }
  
    /*!
     * \brief Get the currently retrieved input action.
     *
     * \return Current action.
     */
    int getCurAction()
    {
      return m_nCurAction;
    }
  
    /*!
     * \brief Test if state has fatal condition.
     *
     * \return Returns true or false.
     */
    bool hasFatal()
    {
      return m_bFatal;
    }
  
    void setFatal()
    {
      m_bFatal = true;
    }
  
    bool isModified()
    {
      return m_bModified;
    }
  
    void setModifiedState(bool bModified)
    {
      m_bModified = bModified;
    }

    /*!
     * \brief Enable/disable masking of hard button input.
     *
     * \param bEnDis Enable (true) or disable (false) reading hard button input.
     */
    void maskHardButton(bool bEnDis)
    {
      m_bHardBttnEnable = bEnDis;
    }
  
    virtual bool isHardButtonPushed(Session &session);
  
    /*!
     * \brief Prompt user, given the state
     *
     * \param session   Session data.
     */
    virtual void prompt(Session &session) { }
  
    virtual void setMenuStates(Session &session) { }
  
    //.........................................................................
    // State Transition Functions
    //.........................................................................
    virtual void transQuit(Session &session)
    {
      session.PushState(Session::StateIdEnd);
    }
  
    virtual void transPrev(Session &session)
    {
      session.PopState();
    }
  
  protected:
    Window *m_pWin;             ///< bound window
    Menu   *m_pMenu;            ///< button menu
    int       m_nInitAction;      ///< initial state input action
    int       m_nCurAction;       ///< current state input action
    bool      m_bFatal;           ///< fatal error
    bool      m_bModified;        ///< state [not] modified
    bool      m_bHardBttnEnable;  ///< enable/disable hard button input
    
    virtual void buildInterface(Session &session);
    virtual void destroyInterface();
    virtual void initButtonMenu() { }
  };
  
  
  //----------------------------------------------------------------------------
  // StateImage Class
  //----------------------------------------------------------------------------
  
  /*!
   * Image Manipulation Workflow State Data Class
   */
  class StateImage : public State
  {
  public:
    Mouse     m_mouse;          ///< mouse instance
    IplImage   *m_pImgIoI;        ///< pointer to current image of interest
    IplImage   *m_pImgShadow;     ///< pointer to "shadow" (low-res) IoI
    IplImage   *m_pImgMarked;     ///< pointer to marked up image
    IplImage   *m_pImgDisplay;    ///< pointer to displayed image
    CvSize      m_sizeDisplay;    ///< size of display region
    CvSize      m_sizeIoI;        ///< image of interest size
    WinIoI   *m_pTransIoI;      ///< displayed ioi transformation
    WinIoI   *m_pTransShadow;   ///< displayed shadow transformation
    CvRect      m_rectIoIRoI;     ///< image of interest region of interest
    CvPoint     m_ptPanRaw;       ///< last pan raw point position
    CvPoint     m_ptPan;          ///< last transformed pan point position
    bool        m_bIoICanZoomIn;  ///< image of interest can [not] zoom in
    bool        m_bIoICanZoomOut; ///< image of interest can [not] zoom out
    bool        m_bIoIIsPanning;  ///< image of interest panning [not] active
    bool        m_bShadowChanged; ///< shadow image does [not] need to update
  
    StateImage(Session    &session,
                 const CvSize &sizeDisplay,
                 const CvSize &sizeIoI,
                 int          nInitAction=UIActionNone);
  
    virtual ~StateImage();
  
    virtual int getNextAction(Session &session, int nTimeoutMs);
  
    virtual void resetView(Session &session);
  
    void setIoI(IplImage *pImgIoI)
    {
      m_pImgIoI = pImgIoI;
    }
  
    void markupPanImg(Session &session);
  
    virtual void markupImg(Session &session) { }
    virtual void showMarkupImg(Session &session);
    virtual void showImg(Session &session, IplImage *pImg);
    virtual CvPoint mapPt(CvPoint& ptHiRes);
    virtual CvPoint mapPtPan(CvPoint& ptHiRes);
    virtual CvRect mapRect(CvRect& rectHiRes);
  
    //.........................................................................
    // State Transition Functions
    //.........................................................................
    virtual void transZoomIn(Session &session);
    virtual void transZoomOut(Session &session);
    virtual void transPan(Session &session);
  
  protected:
    virtual void buildInterface(Session &session);
    virtual void destroyInterface();
  };
  
  
  //----------------------------------------------------------------------------
  // StateCvCamera Class
  //----------------------------------------------------------------------------
  
  /*!
   * OpenCV Video Camera Control Workflow State Data Class
   */
  class StateCvCamera : public State
  {
  public:
    CvCamera  m_camera;           ///< video camera instance
    IplImage   *m_pImgMarked;       ///< pointer to marked up image
    IplImage   *m_pImgDisplay;      ///< pointer to displayed image
    CvSize      m_sizeDisplay;      ///< size of display region
    CameraRes   m_eVideoRes;        ///< video resolution
    WinIoI   *m_pTransIoI;        ///< displayed ioi transformation
    bool        m_bPaused;          ///< capturing images is [not] paused
  
    StateCvCamera(Session       &session,
                    const CvSize    &sizeDisplay,
                    const CameraRes eVideoRes,
                    const CameraRes eImageRes,
                    int             nInitAction=UIActionNone);
  
    virtual ~StateCvCamera();
  
    virtual int getNextAction(Session &session, int nTimeoutMs);
  
    /*!
     * \brief Test if state is paused.
     *
     * \return Returns true or false.
     */
    bool isPaused()
    {
      return m_bPaused;
    }
  
    void setPauseState(bool bPaused)
    {
      m_bPaused = bPaused;
    }
  
    virtual void cloneFrame(Session &session);
    virtual void markupImg(Session &session) { }
    virtual void showMarkupImg(Session &session);
    virtual void showImg(Session &session, IplImage *pImg);
  
    //.........................................................................
    // State Transition Functions
    //.........................................................................
    virtual void transRunVideo(Session &session);
    virtual void transRunMarkupVideo(Session &session);
    virtual void transStopVideo(Session &session);
  
  protected:
    virtual void buildInterface(Session &session);
    virtual void destroyInterface();
  };
  
  
  //----------------------------------------------------------------------------
  // StateGstCamera Class
  //----------------------------------------------------------------------------
  
  /*!
   * OpenCV Video Camera Control Workflow State Data Class
   */
  class StateGstCamera : public State
  {
  public:
    GstCamera m_camera;           ///< video camera instance
    bool        m_bPaused;          ///< capturing images is [not] paused
    CvSize      m_sizeVidWin;       ///< size of video window
  
    StateGstCamera(Session       &session,
                     const CvSize    &sizeDisplay,
                     const CameraRes eVideoRes,
                     const CameraRes eImageRes,
                     int             nInitAction=UIActionNone);
  
    virtual ~StateGstCamera();

    virtual int getNextAction(Session &session, int nTimeoutMs);
 
    /*!
     * \brief Test if state is paused.
     *
     * \return Returns true or false.
     */
    bool isPaused()
    {
      return m_bPaused;
    }
  
    void setPauseState(bool bPaused)
    {
      m_bPaused = bPaused;
    }
  
    virtual void cloneFrame(Session &session);
    virtual void markupImg(Session &session) { }
    virtual void showMarkupImg(Session &session);
    virtual void showImg(Session &session, IplImage *pImg);
  
    //.........................................................................
    // State Transition Functions
    //.........................................................................
    virtual void transRunVideo(Session &session);
    virtual void transRunMarkupVideo(Session &session);
    virtual void transStopVideo(Session &session);
  
  protected:
    virtual void buildInterface(Session &session);
    virtual void destroyInterface();
  };

} // namespace rnr


#endif // _RNR_STATE_H
