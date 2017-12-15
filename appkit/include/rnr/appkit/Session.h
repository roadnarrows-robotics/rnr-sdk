////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Session.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Session base class.
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

#ifndef _RNR_SESSION_H
#define _RNR_SESSION_H

#include <stdio.h>

#include <string>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/State.h"
#include "rnr/appkit/StateMach.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  //----------------------------------------------------------------------------
  // Session Class
  //----------------------------------------------------------------------------

  /*!
   * \brief Session Class
   *
   * Session state data persist throughout the lifetime of an application or
   * user session. Applications may support multiple sessions simultaneously.
   */
  class Session
  {
  public:
    /*!
     * \brief Default initialization contructor.
     *
     * \param nSessionId        Session id.
     * \param strSessionName    Session name.
     */
    Session(int nSessionId = 0, const std::string &strSessionName = "Session") :
        m_nSessionId(nSessionId), m_strSessionName(strSessionName),
        m_sm(nSessionId, strSessionName)
    {
    }
  
    /*!
     * \brief Destructor.
     */
    ~Session()
    {
    }
  
    /*!
     * \brief Session's state machine.
     *
     * \return State machine reference.
     */
    StateMach &sm()
    {
      return m_sm;
    }

    /*!
     * \brief Get session id.
     *
     * \return Session id.
     */
    int getSessionId() const
    {
      return m_nSessionId;
    }

    /*!
     * \brief Get session name.
     *
     * \return String.
     */
    std::string getSessionName() const
    {
      return m_strSessionName;
    }

    /*!
     * \brief Set state relevant context.
     *
     * \param pContext  State specific context data.
     */
    void setContext(void *pContext)
    {
      m_pContext = pContext;
    }
  
    /*!
     * \brief Get state relevant context.
     *
     * \return State specific context data.
     */
    void *getContext()
    {
      return m_pContext;
    }
  
    /*!
     * \brief Set session error.
     *
     * \param ecode   Application specific error code.
     * \param sFmt    Format string.
     * \param ...     Formatted variable arguments.
     */
    void setError(int ecode, const char *sFmt, ...);
  
    /*!
     * \brief Set session fatal error.
     *
     * \param ecode   Application specific error code.
     * \param sFmt    Format string.
     * \param ...     Formatted variable arguments.
     */
    void setFatal(int ecode, const char *sFmt, ...);
  
    /*!
     * \brief Get the last error code.
     *
     * \return Application specific error code.
     */
    int getErrorCode()
    {
      return m_ecode;
    }

    /*!
     * \brief Get the last error message.
     *
     * \return Error message.
     */
    std::string getErrorMsg()
    {
      return m_bufErrorMsg;
    }
  
    /*!
     * \brief Test if session is in fatal condition.
     *
     * \return Returns true or false.
     */
    bool isFatal()
    {
      return m_bHasFatal;
    }
  
  protected:
    int         m_nSessionId;       ///< session id
    std::string m_strSessionName;   ///< session name
    StateMach   m_sm;               ///< session state machine
    void       *m_pContext;         ///< state specific data
    int         m_ecode;            ///< last error code
    char        m_bufErrorMsg[256]; ///< error message
    bool        m_bHasFatal;        ///< does [not] have fatal condition
  };

} // namespace rnr


#endif // _RNR_SESSION_H
