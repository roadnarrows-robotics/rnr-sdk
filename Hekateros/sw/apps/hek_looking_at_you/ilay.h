////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   ImLookingAtYou
//
// File:      ilay.h
//
/*! \file
 *
 * $LastChangedDate: 2011-07-20 11:17:03 -0600 (Wed, 20 Jul 2011) $
 * $Rev: 1145 $
 *
 * \brief Hekateros follow the face declarations.
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

#ifndef _ILAY_H
#define _ILAY_H

#include <time.h>
#include <string.h>

#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"
#include "rnr/rnrWinOpenCv.h"
#include "rnr/rnrWinMenu.h"
#include "rnr/rnrWinIoI.h"

#include "Hekateros/Hekateros.h"

using namespace rnrWin;

/*!
 * Configuration
 */
const char* const IlayWinName = "Hekateros I'm Looking At You";
                                        ///< name (and id) of main window

/*!
 * Video device major number.
 */
#ifdef STALEMATE_VID_DEV_MAJOR
const int IlayVidDevMajor = STALEMATE_VID_DEV_MAJOR;
#else
const int IlayVidDevMajor = 81;
#endif

//
// Window Sizes and Transformations
//
#if defined(ARCH_overo) && STALEMATE_DISPLAY==LGLCD43

#include "Hekateros/hw/lglcd43.h"

const rnrWinRot   IlayImgTransRot    = rnrWinRot0;         ///< rotation
const rnrWinAlign IlayImgTransAlign  = rnrWinAlignCenter;  ///< alignment
const bool        IlayImgTransCrop   = false;              ///< cropping
const int         IlayWinWidth       = LgLcd43DisplayWidth;  ///< width
const int         IlayWinHeight      = LgLcd43DisplayHeight; ///< height

#else // full size monitor 

const rnrWinRot   IlayImgTransRot    = rnrWinRot0;         ///< rotation
const rnrWinAlign IlayImgTransAlign  = rnrWinAlignCenter;  ///< alignment
const bool        IlayImgTransCrop   = false;              ///< cropping
const int         IlayWinWidth       = 1440;               ///< width
const int         IlayWinHeight      = 900;                ///< height

#endif // ARCH_overo ...

const int   IlayMaxButtonsPerMenu = 10;    ///< max number of buttons/menu


/*!
 * Ilay States
 */
enum IlayState
{
  IlayStateStart,          ///< (re)start
  IlayStateRun,            ///< run chess application
  IlayStateNewGame,        ///< start new game
  IlayStateMakeMove,       ///< start new game
  // TODO more
  IlayStateEnd             ///< end (exit application)
};

/*!
 * Menu Actions
 */
enum IlayAction
{
  IlayActionQuit = rnrWinUIActionNumOf,  ///< quit operation
  IlayActionIdle,                        ///< run in idle mode
  IlayActionNewGame,                     ///< start new game
  IlayActionMakeMove,                    ///< make a move
};


//------------------------------------------------------------------------------
// IlaySession Class
//------------------------------------------------------------------------------

/*!
 * Ilay Session State and Data Class
 */
class IlaySession
{
public:
  IlaySession();

  ~IlaySession();

  void SetVideoDevice(const char *sVidDevName, uint_t uVidDevMinor);

  IlayState GetCurState()
  {
    return m_eCurState;
  }

  void SetCurState(IlayState eNewState)
  {
    if( eNewState != m_eCurState )
    {
      m_ePrevState  = m_eCurState;
      m_eCurState   = eNewState;
    } 
  }

  IlayState GetPrevState()
  {
    return m_ePrevState;
  }

  IlayState  m_eCurState;      ///< application current state
  IlayState  m_ePrevState;     ///< application previous state
  rnrWindow      *m_pWin;           ///< application main gui window
  rnrWinMenu     *m_pMenu;          ///< application main menu         
  char           *m_sVidDevName;    ///< video device name
  int             m_uVidDevMinor;   ///< video device minor number (index)
  CvSize          m_sizeVideo;      ///< WxH size of video
  CvCapture      *m_pVidCapture;    ///< video capture handle
  rnrWinIoI      *m_pVidToGuiTrans; ///< video to gui image transform operator
  IplImage       *m_pGuiVidImg;     ///< captured video frame gui transformed
};

/*!
 * Action Function Type.
 */
typedef int (*IlayActionFunc_T)(IlaySession &);


//------------------------------------------------------------------------------
// Inline utilities
//------------------------------------------------------------------------------

/*!
 * \brief Allocate new duplicated string.
 *
 * \param s String to duplicate.
 *
 * \return Returns pointer to allocated string if s is not NULL and the
 * length of s \h_gt 0.\n Otherwise returns NULL.
 */
inline char *newstr(const char *s)
{
  char  *t;
  if( (s != NULL) && (*s != 0) )
  {
    t = new char[strlen(s)+1];
    strcpy(t, s);
  }
  else
  {
    t = NULL;
  }
  return t;
}

/*!
 * \brief Delete allocated string.
 *
 * \param s String to delete.
 *
 * \return Returns NULL.
 */
inline char *delstr(const char *s)
{
  if( s != NULL )
  {
    delete[] s;
  }
  return NULL;
}


//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------

extern int IlayMasterControl(IlaySession &session);

extern void IlayIoIShow(IlaySession &session, IplImage *pIoI);

extern int IlayVideoCaptureRestart(IlaySession &session);

extern void IlayVideoCaptureStop(IlaySession &session);

extern int IlayVideoCaptureFrame(IlaySession   &session,
                                      IplImage         **ppImgFrame);

extern int IlayVideoTakeImg(IlaySession   &session,
                                 IplImage         **ppImgSnapShot);



#endif // _ILAY_H
