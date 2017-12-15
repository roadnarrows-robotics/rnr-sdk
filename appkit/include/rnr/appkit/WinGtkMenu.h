////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_wingtk
//
// File:      WinGtkMenu.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics GTK derived window button menu interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#ifndef _RNR_WIN_GTK_BUTTON_MENU_H
#define _RNR_WIN_GTK_BUTTON_MENU_H

#include <string>

#include "rnr/rnrconfig.h"

#include <gtk/gtk.h>

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinGtk.h"
#include "rnr/appkit/WinMenu.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  //...........................................................................
  // Class WinGtkButton
  //...........................................................................

  /*!
   * \brief GTK window derived button class.
   */
  class WinGtkButton : public WinButton
  {
  public:

    /*!
     * \brief WinGtkButton initialization constructor.
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
    WinGtkButton(WinButtonType      eType,
                 int                nEvent,
                 WidgetState        eInitState,
                 AlignOp            eAlign,
                 const std::string &strAltText,
                 const std::string &strTagNormal,
                 const std::string &strToolTipNormal  = "",
                 const std::string &strTagActive      = "",
                 const std::string &strToolTipActive  = "");

    /*!
     * \brief WinGtkButton copy constructor.
     *
     * \param src   Source button item.
     */
    WinGtkButton(const WinGtkButton &src);

    /*!
     * \brief Destructor.
     */
    virtual ~WinGtkButton();

    /*!
     * \brief Get the normal icon image widget.
     *
     * \return Returns (possibly NULL) pointer to image widget
     * (gui toolkit specific).
     */
    virtual void *getNormalImageWidget()
    {
      return m_wImgNormal;
    }

    /*!
     * \brief Get the active icon image widget.
     *
     * \return Returns (possibly NULL) pointer to image widget
     * (gui toolkit specific).
     */
    virtual void *getActiveImageWidget()
    {
      return m_wImgActive;
    }

    friend class WinGtkButtonMenu;

  protected:
    GtkWidget        *m_wImgNormal;     ///< normal state menu button icon
    GtkWidget        *m_wImgActive;     ///< active state menu button icon
    
    /*!
     * \brief Load icon image from file.
     *
     * \note For best visualization, the icons should be 48x48 pixels.
     *
     * \param strIconPath   Icon file path.
     *
     * \return Returns pointer to GtkImage widget if icon successfully loaded,
     * else returns NULL.
     */
    GtkWidget *loadIcon(const std::string &strIconPath);
  };


  //...........................................................................
  // Class WinGtkButtonMenu
  //...........................................................................
 
  /*!
   * \brief GTK window derived button menu class.
   *
   * Buttons with icons or text are placed on the left or right side of the
   * application window. Each icon is associated with an application defined
   * event.
   */
  class WinGtkButtonMenu : public WinButtonMenu
  {
  public:

    /*!
     * \brief WinGtkButtonMenu initialization constructor.
     * 
     * \param strIconPath   Icon path of search directories.
     */
    WinGtkButtonMenu(const std::string &strIconPath=WinIconDirDft) :
      WinButtonMenu(strIconPath)
    {
    }

    /*!
     * \brief WinGtkButtonMenu destructor.
     */
    virtual ~WinGtkButtonMenu() { }

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
                                const std::string &strToolTipActive  = "");

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
                                const std::string &strToolTipActive  = "");

  protected:
  };
    
} // namespace


#endif // _RNR_WIN_GTK_BUTTON_MENU_H
