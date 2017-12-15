////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_win
//
// File:      SessionWin.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief SessionWin derived class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_SESSION_WIN_H
#define _RNR_SESSION_WIN_H

#include <stdio.h>

#include <string>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/State.h"
#include "rnr/appkit/StateMach.h"
#include "rnr/appkit/Session.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  //----------------------------------------------------------------------------
  // SessionWin Class
  //----------------------------------------------------------------------------

  /*!
   * Workflow Session Class
   *
   * Session state persist throughout the livetime of an application. A series
   * of workflow states, each with its on finite state machine, is supportedc.
   */
  class SessionWin : public Session
  {
  public:
    /*!
     * \brief Video source types.
     */
    enum VideoSrcType
    {
      VideoSrcTypeDevice = 0,   ///< source direct from video device
      VideoSrcTypeUdp,          ///< source from UDP stream
      VideoSrcTypeFile,         ///< source from file
      VideoSrcTypeNumOf         ///< number of video source types
    };

    static const VideoSrcType VideoSrcTypeDft = VideoSrcTypeDevice;
    static const char * const VideoDevNameDft;
    static const int          VideoIndexDft   = 0;
    static const int          VideoPortDft    = 4000;

    /*!
     * \brief Default initialization contructor.
     *
     * \param nSessionId        Session id.
     * \param strSessionName    Session name.
     * \param pWin              Pointer to created window.
     */
    SessionWin(int                nSessionId = 0,
               const std::string &strSessionName = "Session",
               Win               *pWin = NULL) :
        Session(nSessionId, strSessionName),
        m_pWin(pWin),
        m_eVideoSrcType(VideoSrcTypeDft),
        m_strVideoDevName(VideoDevNameDft),
        m_nVideoIndex(VideoIndexDft),
        m_nVideoPort(VideoPortDft)
    {
    }
  
    /*!
     * \brief Destructor.
     */
    ~SessionWin()
    {
      if( m_pWin != NULL )
      {
        delete m_pWin;
      }
    }
  
    /*!
     * \brief Set session's window.
     *
     * The session own the window object and will delete it window replacement
     * or on session object deletion.
     *
     * \param pWin    Pointer to window object.
     */
    void setWin(Win *pWin)
    {
      if( m_pWin != NULL )
      {
        delete m_pWin;
      }
      m_pWin = pWin;
    }

    /*!
     * \brief Get the session window object.
     *
     * \return Pointer to the window object.
     */
    Win *getWin()
    {
      return m_pWin;
    }

    /*!
     * \brief Get the session window object.
     *
     * \return Reference to the window object. Cannot be null.
     */
    Win &win()
    {
      return *m_pWin;
    }

    /*!
     * \brief Get the video source type.
     *
     * \return Return one of \ref VideoSrcType.
     */
    VideoSrcType getVideoSrcType()
    {
      return m_eVideoSrcType;
    }
  
    /*!
     * \brief Set the video source type.
     *
     * \param eVideoSrcType   One of \ref VideoSrcType.
     */
    void setVideoSrcType(VideoSrcType eVideoSrcType)
    {
      m_eVideoSrcType = eVideoSrcType;
    }
  
    /*!
     * \brief Set the video device name.
     *
     * The video index (device minor number) is determined and also set.
     *
     * \param strVideoDevName   Video device name.
     */
    int setVideoDevice(const std::string &strVideoDevName);

    /*!
     * \brief Get the video device name.
     *
     * \return String.
     */
    std::string getVideoDevName()
    {
      return m_strVideoDevName;
    }
  
    /*!
     * \brief Get the video device index.
     *
     * \return Integer index.
     */
    int getVideoIndex()
    {
      return m_nVideoIndex;
    }
  
    /*!
     * \brief Get the video index associated with the device.
     *
     * \param strVideoDevName Video camera device name.
     *
     * \return Returns video index on success, \ref RC_ERROR(-1) on failure.
     */
    int getVideoIndex(const std::string &strVideoDevName);

    /*!
     * \brief Get the video stream port.
     *
     * \return UDP port.
     */
    int getVideoPort()
    {
      return m_nVideoPort;
    }
  
    /*!
     * \brief Set the video stream port.
     *
     * \param nVideoPort  UDP port.
     */
    void setVideoPort(int nVideoPort)
    {
      m_nVideoPort = nVideoPort;
    }

  protected:
    Win          *m_pWin;             ///< application main gui window
    VideoSrcType  m_eVideoSrcType;    ///< video source type
    std::string   m_strVideoDevName;  ///< video device name
    int           m_nVideoIndex;      ///< video index (device minor number)
    int           m_nVideoPort;       ///< video stream UDP/IP port
  };

} // namespace rnr


#endif // _RNR_SESSION_WIN_H
