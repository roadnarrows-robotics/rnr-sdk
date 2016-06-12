////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      WinMenu.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-08 08:22:06 -0600 (Wed, 08 May 2013) $
 * $Rev: 2920 $
 *
 * \brief RoadNarrows Robotics base window button menu interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#ifndef _RNR_WIN_BUTTON_MENU_H
#define _RNR_WIN_BUTTON_MENU_H

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/Win.h"

//
// RoadNarrows Robotics Windowing Interface
//
namespace rnr
{
#if defined(ARCH_overo)
  const char* const WinIconDirDft = "/usr/local/share/appkit/images/icons";
#else
  const char* const WinIconDirDft = "/prj/share/appkit/images/icons";
#endif // ARCH_overo
  
  /*!
   * \brief Window button types.
   */
  enum WinButtonType
  {
    WinButtonTypeImage,   ///< button has image(s)
    WinButtonTypeLabel    ///< button has text label(s)
  };

  //...........................................................................
  // Class WinButton
  //...........................................................................

  /*!
   * \brief Window button base class.
   */
  class WinButton
  {
  public:

    /*!
     * \brief WinButton initialization constructor.
     *
     * \param eType             Button type.
     * \param nEvent            Event associated with button press (and id).
     * \param eInitState        Button widget initial state.
     *                          See \ref rnrwin_widgetstate.
     * \param eAlign            Button alignment. See \ref rnmpwin_align.
     * \param strAltText        Button alternative text string.
     * \param strTagNormal      Button visual tag for normal state.
     *                          Either icon path or text string label.
     * \param strToolTipNormal  Button tooltip for normal state.
     * \param strTagActive      Button visual tag for active state.
     *                          Either icon path or text string label.
     *                          Empty for no tag.
     * \param strToolTipActive  Button tooltip for active state.
     */
    WinButton(WinButtonType      eType,
              int                nEvent,
              WidgetState        eInitState,
              AlignOp            eAlign,
              const std::string &strAltText,
              const std::string &strTagNormal,
              const std::string &strToolTipNormal  = "",
              const std::string &strTagActive      = "",
              const std::string &strToolTipActive  = "") :
        m_eType(eType),
        m_nEvent(nEvent),
        m_eInitState(eInitState),
        m_eState(eInitState),
        m_eAlign(eAlign),
        m_strAltText(strAltText),
        m_strTagNormal(strTagNormal),
        m_strToolTipNormal(strToolTipNormal),
        m_strTagActive(strTagActive),
        m_strToolTipActive(strToolTipActive)
    {
    }

    /*!
     * \brief WinButton copy constructor.
     *
     * \param src   Source window button.
     */
    WinButton(const WinButton &src)
    {
      copy(&src);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~WinButton() { }

    /*!
     * \brief Get the normal icon image widget.
     *
     * \return Returns (possibly NULL) pointer to image widget
     * (gui toolkit specific).
     */
    virtual void *getNormalImageWidget()
    {
      return NULL;
    }

    /*!
     * \brief Get the active icon image widget.
     *
     * \return Returns (possibly NULL) pointer to image widget
     * (gui toolkit specific).
     */
    virtual void *getActiveImageWidget()
    {
      return NULL;
    }

    /*!
     * \brief Get the icon image widget associated with the current button
     * state.
     *
     * \return 
     * Returns pointer to image widget (gui toolkit specific) if found.\n
     * Otherwise returns NULL.
     */
    void *getCurrentImageWidget();

    /*!
     * \brief Get the text label associated with the current button state.
     *
     * \return Returns (possibly empty) text string.
     */
    std::string getCurrentLabel();

    /*!
     * \brief Get the tooltip associated with the current button state.
     *
     * \return Returns (possibly empty) text string.
     */
    std::string getCurrentToolTip();

    friend class WinButtonMenu;

  protected:
    WinButtonType   m_eType;            ///< button type
    int             m_nEvent;           ///< button push "mouse click" event
    WidgetState     m_eInitState;       ///< initial button widget state
    WidgetState     m_eState;           ///< current button widget state
    AlignOp         m_eAlign;           ///< left or right alignment
    std::string     m_strAltText;       ///< alternate text for icon(s)
    std::string     m_strTagNormal;     ///< button icon for normal state
    std::string     m_strToolTipNormal; ///< button tooltip for normal state
    std::string     m_strTagActive;     ///< button icon for active state
    std::string     m_strToolTipActive; ///< button tooltip for active state

    /*!
     * \brief Deep copy of source to this.
     *
     * \param pSrc    Pointer to source button.
     */
    void copy(const WinButton *pSrc);
  };


  //...........................................................................
  // Class WinButtonMenu
  //...........................................................................
 
  /*!
   * \brief Window button menu base class.
   *
   * Buttons with icons or text are placed on the left or right side of the
   * application window. Each icon is associated with an application defined
   * event.
   */
  class WinButtonMenu
  {
  public:
    typedef std::map<int, WinButton*>  MapBttns_T;
                                              ///< map of added menu buttons

    /*!
     * \brief WinButtonMenu initialization constructor.
     *
     * \param strIconPath   Icon path of search directories.
     */
    WinButtonMenu(const std::string &strIconPath=WinIconDirDft)
    {
      m_pWin            = NULL;
      m_strIconPath     = strIconPath;
      m_nCurrentEvent   = UIEventNone;
      m_nPreviousEvent  = UIEventNone;
    }

    /*!
     * \brief WinButtonMenu destructor.
     */
    virtual ~WinButtonMenu();

    /*!
     * \brief Append directory to icon search path.
     *
     * \param strIconDir    Directory holding icons.
     */
    void appendIconDir(const std::string &strIconDir)
    {
      if( m_strIconPath.empty() )
      {
        m_strIconPath = strIconDir;
      }
      else
      {
        m_strIconPath = m_strIconPath + PATH_SEP_STR + strIconDir;
      }
    }

    /*!
     * \brief Prepend directory to icon search path.
     *
     * \param strIconDir    Directory holding icons.
     */
    void prependIconDir(const std::string &strIconDir)
    {
      if( m_strIconPath.empty() )
      {
        m_strIconPath = strIconDir;
      }
      else
      {
        m_strIconPath = strIconDir + PATH_SEP_STR + m_strIconPath;
      }
    }

    /*!
     * \brief Set icon search path.
     *
     * \param strIconPath   Icon path of search directories.
     */
    void setIconPath(const std::string &strIconPath)
    {
      m_strIconPath = strIconPath;
    }

    /*!
     * \brief Get icon search path.
     *
     * \return Icon path string.
     */
    std::string getIconPath()
    {
      return m_strIconPath;
    }

    /*!
     * \brief Add button with image to menu.
     *
     * \note Buttons are added to window only at bind() time.
     *
     * \param nEvent            Unique event (and id) associated with button
     *                          press.
     * \param eInitState        Button widget initial state.
     *                          See \ref rnrwin_widgetstate.
     * \param eAlign            Button alignment. See \ref rnmpwin_align.
     * \param strAltText        Button alternative text string.
     * \param strIconNormal     Button icon image file name for normal state.
     * \param strToolTipNormal  Button tooltip for normal state.
     * \param strIconActive     Button icon image file name for active state.
     *                          Empty for no icon.
     * \param strToolTipActive  Button tooltip for active state.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addImageButton(int                nEvent,
                                WidgetState        eInitState,
                                AlignOp            eAlign,
                                const std::string &strAltText,
                                const std::string &strIconNormal,
                                const std::string &strToolTipNormal  = "",
                                const std::string &strIconActive     = "",
                                const std::string &strToolTipActive  = "")
    {
      return false;
    }

    /*!
     * \brief Add button with text label to menu.
     *
     * \note Buttons are added to window only at bind() time.
     *
     * \param nEvent            Unique event (and id) associated with button
     *                          press.
     * \param eInitState        Button widget initial state.
     *                          See \ref rnrwin_widgetstate.
     * \param eAlign            Button alignment. See \ref rnmpwin_align.
     * \param strLabelNormal    Button text label for normal state.
     * \param strToolTipNormal  Button tooltip for normal state.
     * \param strLabelActive    Button text label for active state.
     *                          Empty for no label.
     * \param strToolTipActive  Button tooltip for active state.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addLabelButton(int                nEvent,
                                WidgetState        eInitState,
                                AlignOp            eAlign,
                                const std::string &strLabelNormal,
                                const std::string &strToolTipNormal  = "",
                                const std::string &strLabelActive    = "",
                                const std::string &strToolTipActive  = "")
    {
      return false;
    }

    /*!
     * \brief Bind the button menu to the given window.
     *
     * Any previously bound menu is unbound. This menu is reset to its initial
     * state. The menu buttons are added to the window and displayed.
     *
     * \param pWin  Pointer to binding window.
     */
    virtual void bind(Win *pWin);

    /*!
     * \brief Unbind the menu from the currently bound window.
     */
    virtual void unbind();

    /*!
     * \brief Get the current button menu event.
     *
     * \return Current event.
     */
    int getCurrentEvent()
    {
      return m_nCurrentEvent;
    }
  
    /*!
     * \brief Set the current button menu event.
     *
     * \param nNewEvent   New event unique identifier.
     *
     * \return New current event.
     */
    int setCurrentEvent(int nNewEvent)
    {
      if( nNewEvent != m_nCurrentEvent )
      {
        m_nPreviousEvent = m_nCurrentEvent;
      }
      m_nCurrentEvent = nNewEvent;
      return m_nCurrentEvent;
    }

    /*!
     * \brief Get the previous menu event.
     *
     * \return Current action.
     */
    int getPreviousEvent()
    {
      return m_nPreviousEvent;
    }
  
    /*!
     * \brief Get the current number of menu items;
     *
     * \return Number of items.
     */
    size_t getNumMenuButtons()
    {
      return m_mapButtons.size();
    }

    /*!
     * \brief Get the given button state.
     *
     * \param nEvent    Menu-wide unique event button identifier.
     *
     * \return Current button state. See \ref rnmpwin_widgetstate
     */
    WidgetState getButtonState(int nEvent)
    {
      WidgetState eState = WidgetStateNormal;

      if( m_mapButtons.find(nEvent) != m_mapButtons.end() )
      {
        eState = m_mapButtons[nEvent]->m_eState;
      }
      return eState;
    }

    /*!
     * \brief Set the given button state.
     *
     * \param nEvent    Menu-wide unique event button identifier.
     * \param eNewState New button state. See \ref rnmpwin_widgetstate
     */
    void setButtonState(int nEvent, WidgetState eNewState)
    {
      MapBttns_T::iterator  pos;
      
      if( (pos = m_mapButtons.find(nEvent)) != m_mapButtons.end() )
      {
        changeButtonState(pos->second, eNewState);
      }
    }

    /*!
     * \brief Set the states of the given buttons.
     *
     * \param strEvent  Event reason string (used for logging only).
     * \param ...       Sequence of event,state pairs. The sequence is
     *                  terminated by the UIEventNone special 'no' action
     *                  event.
     */
    void setButtonStateList(const std::string &strEvent, ...);

    /*!
     * \brief Reset all menu buttons states to their initial states.
     */
    void resetAllButtonStates();

  protected:
    Win            *m_pWin;               ///< menus are bound to this window
    std::string     m_strIconPath;        ///< icon directories search path
    int             m_nCurrentEvent;      ///< current (user) menu event 
    int             m_nPreviousEvent;     ///< previous (user) menu event
    MapBttns_T      m_mapButtons;         ///< menu button map

    /*!
     * \brief Make icon image file path.
     *
     * The built file path is check of existence.
     *
     * \param strIconFile Icon file absolute or relative file name.
     *
     * \return Returns string containing icon path. Empty if no file specified
     * or found.
     */
    std::string makeIconPath(const std::string &strIconFile);

    /*!
     * \brief Change the state and gui look of the given button.
     *
     * \param pButton   Pointer to menu button instance.
     * \param eNewState New button state. See \ref rnmpwin_widgetstate
     */
    void changeButtonState(WinButton *pButton, WidgetState eNewState);

    /*!
     * \brief Menu button press callback.
     *
     * \param nEvent    Button event (and button id).
     * \param user_data User-supplied data (this). 
     */
    static void onMenuButtonClick(int nEvent, void *user_data);
  };
    
} // namespace


#endif // _RNR_WIN_BUTTON_MENU_H
