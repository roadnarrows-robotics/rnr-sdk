////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      Win.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics Win abstract base class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows
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

#ifndef _RNR_WIN_H
#define _RNR_WIN_H

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

#include "rnr/appkit/WinLookFeel.h"

//
// Windowing Declarations
//
namespace rnr
{
  /*!
   * \ingroup rnrwin_h
   * \defgroup rnrwin_align  Alignment Transformation Directives
   *
   * Window widget, image, or video alignment.
   * \{
   */
  enum AlignOp
  {
    AlignOpDefault,   ///< default alignment (left)
    AlignOpLeft,      ///< left alignment
    AlignOpCenter,    ///< center alignment
    AlignOpRight      ///< rigth alignment
  };
  /*! \} */

  /*!
   * \ingroup rnrwin_h
   * \defgroup rnrwin_rot  Rotation Transformation Directives
   *
   * Window widget, image, or video rotation.
   * \{
   */
  enum RotOp
  {
    RotOp0,           ///< 0\h_deg rotation (none)
    RotOp90,          ///< 90\h_get rotation
    RotOp180,         ///< 180\h_get rotation (flip verically)
    RotOp270          ///< 270\h_get rotation (-90\h_deg)
  };
  /*! \} */

  /*!
   * \ingroup rnrwin_h
   * \defgroup rnrwin_uievent User Interface Reserved Events
   *
   * Reserved events. User defined events must be after these values.
   * \{
   */
  enum UIEvent
  {
    UIEventNone,        ///< no action
    UIEventClick,       ///< mouse click 
    UIEventDragStart,   ///< start of mouse drag
    UIEventDragging,    ///< dragging mouse
    UIEventDragEnd,     ///< end of mouse drag
    UIEventNumOf        ///< number of reserved actions
  };
  /*! \} */

  /*!
   * \ingroup rnrwin_h
   * \defgroup rnrwin_widgetstate   GUI Widget State
   * \{
   */
  enum WidgetState
  {
    WidgetStateNormal,      ///< normal - enabled but not selected
    WidgetStateActive,      ///< selected
    WidgetStateDisabled,    ///< disabled
    WidgetStateNumOf        ///< number of states
  };
  /*! \} */

  /*!
   * \brief Mouse Callback Function Type.
   *
   * User-supplied callback function. The callback is called on mouse events
   * focused on an OpenCV image widget.
   *
   * Typical user-defined mouse events are select a region, drag, and draw.
   *
   * \param cv_event  OpenCv defined events. See CV_EVENT_*.
   * \param x         Mouse x pixel coordinate relative to upper left image
   *                  widget. 
   * \param y         Mouse y pixel coordinate relative to upper left image
   *                  widget. 
   * \param           Windowing system specific flags.
   * \param user_data User-supplied data. 
   */
  typedef void (*MouseCbFunc_T)(int   cv_event,
                                int   x,
                                int   y,
                                int   flags,
                                void *user_data);

  /*!
   * \brief Keyboard Callback Function Type.
   *
   * User-supplied callback function on keyboard events.
   *
   * \param code      Keyboard code.
   * \param state     Keyboard state.
   * \param user_data User-supplied data. 
   */
  typedef void (*KeyCbFunc_T)(int code, int state, void *user_data);

  /*!
   * \brief Menu button Callback Function Type.
   *
   * User-supplied callback function on menu button pushes.
   *
   * \param nButtnId  Button id.
   * \param user_data User-supplied data. 
   */
  typedef void (*BttnCbFunc_T)(int nBttnId, void *user_data);


  //...........................................................................
  // Class Win
  //...........................................................................

  /*!
   * \brief RNR Win window abstract base class.
   *
   * An instance of a derived Win class divides the screen into three vertical
   * regions above, and two regions below.
   *
   * \verbatim
   *        -----------------------------------------------
   *        |     |                                 |     |
   *        |     |                                 |     |
   *        |Left |                                 |Right|
   *        |Menu |           Workspace             | Menu|
   *        |Box  |                                 |  Box|
   *        |     |                                 |     |
   *        |     |                                 |     |
   *        -----------------------------------------------
   *        | ref |         status  message               |
   *        -----------------------------------------------
   * \endverbatim
   *
   * The left and right menu boxes hold on-screen buttons. The workspace can
   * hold a collection of application specific widgets, including OpenCV image
   * matrices and GStreamer video. Below, the reference box displays workflow
   * context, while the status line displays prompts, errors, and info messages.
   */
  class Win
  {
  public:
    static const int MaxCvImages          = 4; ///< max workspace images/video 
    static const int NumOfButtonMenus     = 2; ///< left and right menus

    /*!
     * \brief Windowing class initialization constructor.
     *
     * \param strWinName          Window name.
     * \param nWidth              Window width in pixels.
     * \param nHeight             Window height in pixels.
     * \param bDecorate           Do [not] decorate window with title, border,
     *                            etc. Windowing system specific.
     */
    Win(const std::string &strWinName,
        int               nWidth,
        int               nHeight,
        bool              bDecorate=true);

    /*!
     * \brief Destructor.
     */
    virtual ~Win() { }

    /*!
     * \brief Get GUI toolkit name.
     *
     * \return GUI toolkit identifier string.
     */
    virtual const char *getGuiToolkitName()
    {
      return "abstract";
    }

    /*!
     * \brief Set GUI look and feel value.
     *
     * A look and feel value can be a color, font, line spacing, etc. There is
     * a standard set which can be extended for application-specific needs.
     *
     * Standard set:
     * \termblock
     * \term \b key \termdata \b description \endterm
     * \term color_win_bg \termdata window background color \endterm
     * \term color_status_fg \termdata status bar foreground color \endterm
     * \term color_status_bg \termdata status bar background color \endterm
     * \term color_status_border \termdata status bar border color \endterm
     * \term color_button_bg \termdata button background color \endterm
     * \term color_text_fg \termdata default text foreground color \endterm
     * \term color_text_bg \termdata default text background color \endterm
     * \term color_image_bg \termdata default image background color \endterm
     * \term color_rn_black \termdata RoadNarrows black \endterm
     * \term color_rn_white \termdata RoadNarrows black \endterm
     * \term color_rn_red \termdata RoadNarrows black \endterm
     * \term color_rn_yellow \termdata RoadNarrows black \endterm
     * \term font_text \termdata default text font \endterm
     * \term font_status \termdata status bar font \endterm
     * \term font_large \termdata large font \endterm
     * \term font_medium \termdata medium font \endterm
     * \term font_small \termdata small font \endterm
     * \term font_tiny \termdata tiny font \endterm
     * \endtermblock
     *
     * \param strKey    Key.
     * \param strVal    Value.
     */
    virtual void setLookAndFeel(const std::string &strKey,
                                const std::string &strVal)
    {
      m_mapLookFeel[strKey] = strVal;
    }

    /*!
     * \brief Get the GUI look and feel value.
     *
     * \param strKey    Key.
     *
     * \return Value.
     */
    virtual std::string getLookAndFeel(const std::string &strKey)
    {
      std::string strVal;

      if( m_mapLookFeel.find(strKey) != m_mapLookFeel.end() )
      {
        strVal = m_mapLookFeel[strKey];
      }
      return strVal;
    }

    /*!
     * \brief Show/refresh all gui widgets in window.
     */
    virtual void show() = 0;
    
    /*!
     * \brief Show a OpenCv image on the workspace.
     *
     * \param img         Pointer to image.
     * \param uImgIndex   Image instance.
     */
    virtual void showCvImage(cv::Mat &img, uint_t uImgIndex=0) = 0;

    /*!
     * \brief Show page reference number.
     *
     * \param nPageRef   Page reference number.
     */
    virtual void showPageRef(int nPageRef) = 0;

    /*!
     * \brief Show page reference string.
     *
     * \param strPageRef    Page reference string.
     */
    virtual void showPageRef(const std::string &strPageRef) = 0;

    /*!
     * \brief Show status message.
     *
     * \param sFmt  Format string.
     * \param ...   Variable arguments.
     */
    virtual void showStatus(const char *sFmt, ...) = 0;

    /*!
     * \brief Clear status message.
     */
    virtual void clearStatus() = 0;

    /*!
     * \brief Wait for keypress or timeout.
     *
     * Window widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds. Set to 0 for no timeout.
     *
     * \return Returns code of last key pressed or -1 if timed out.
     */
    virtual int waitKey(int delay) = 0;

    /*!
     * \brief Wait for timeout or registered mouse event.
     *
     * Window widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds \h_gt 0.
     */
    virtual void  waitMouse(int delay) = 0;
    
    /*!
     * \brief Wait for timeout.
     *
     * Window widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds \h_gt 0.
     */
    virtual void  wait(int delay) = 0;
    
    /*!
     * \brief Add a button with an image to window button menu.
     *
     * If the image fails to load, the alternate text is display instead.
     *
     * \param nBttnId       Window unique button id.
     * \param eAlign        Menu item icon alignment. See \ref rnmpwin_align
     * \param strIconPath   Path to icon image file.
     * \param strAltText    Button alternate text.
     * \param strToolTip    Optional button tool tip text.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addImageButton(int                nBttnId,
                                AlignOp            eAlign,
                                const std::string &strIconPath,
                                const std::string &strAltText,
                                const std::string &strToolTip="") = 0;

     /*!
     * \brief Add a button with an image widget to window button menu.
     *
     * \param nBttnId       Window unique button id.
     * \param eAlign        Menu item icon alignment. See \ref rnmpwin_align
     * \param wBttnImg      Pointer to button image widget
     *                      (gui toolkit specific).
     * \param strAltText    Button alternate text.
     * \param strToolTip    Optional button tool tip text.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addImageButton(int                nBttnId,
                                AlignOp            eAlign,
                                void              *wBttnImg,
                                const std::string &strAltText,
                                const std::string &strToolTip="") = 0;

    /*!
     * \brief Add a button with a label to window button menu.
     *
     * \param nBttnId       Window unique button id.
     * \param eAlign        Menu item icon alignment. See \ref rnmpwin_align
     * \param strLabel      Button text label.
     * \param strToolTip    Optional button tool tip text.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addLabelButton(int                nBttnId,
                                AlignOp            eAlign,
                                const std::string &strLabel,
                                const std::string &strToolTip="") = 0;

    /*!
     * \brief Replace existing button image with new image.
     *
     * \param nBttnId       Unique button id.
     * \param strIconPath   Path to icon image file.
     * \param strToolTip    Optional button tool tip text. If empty, then the
     *                      current tooltip, if any, is used.
     *
     * \return Returns true if button image is replaced, false otherwise.
     */
    virtual bool replaceButtonImage(int                nBttnId,
                                    const std::string &strIconPath,
                                    const std::string &strToolTip="") = 0;

    /*!
     * \brief Replace existing button image with new GTK image widget.
     *
     * \param nBttnId       Unique button id.
     * \param wBttnImg      Pointer to new button image widget
     *                      (gui toolkit specific).
     * \param strToolTip    Optional button tool tip text. If empty, then the
     *                      current tooltip, if any, is used.
     *
     * \return Returns true if button image is replaced, false otherwise.
     */
    virtual bool replaceButtonImage(int                nBttnId,
                                    void              *wBttnImg,
                                    const std::string &strToolTip="") = 0;

    /*!
     * \brief Replace existing button label with new label.
     *
     * \param nBttnId       Unique button id.
     * \param strLabel      Button text label.
     * \param strToolTip    Optional button tooltip text. If empty, then the
     *                      current tooltip, if any, is used.
     *
     * \return Returns true if button label is replaced, false otherwise.
     */
    virtual bool replaceButtonLabel(int                nBttnId,
                                    const std::string &strLabel,
                                    const std::string &strToolTip="") = 0;

    /*!
     * \brief Remove a button from a menu.
     *
     * \param nBttnId   Unique button id.
     *
     * \return Returns true if button is removed, false otherwise.
     */
    virtual bool removeButton(int nBttnId) = 0;

    /*!
     * \brief Remove all buttons from all menus.
     */
    virtual void removeAllButtons() = 0;

    /*!
     * \brief Show a button's widget state.
     *
     * \param nBttnId     Unique button id.
     * \param eBttnState  New button state. See \ref rnmpwin_itemstate.
     *
     * \return Returns true if button state set, false otherwise.
     */
    virtual bool showButtonState(int nBttnId, WidgetState eBttnState) = 0;

    /*!
     * \brief Get the window's workspace size.
     *
     * \return Size of workspace.
     */
    cv::Size getWorkspaceSize()
    {
      return cv::Size(m_rectWorkspace.width, m_rectWorkspace.height);
    }

    /*!
     * \brief Initialize window's workspace to hold a native GUI toolkit
     * widget tree.
     *
     * All previous gui elements are erased.
     */
    virtual void initWorkspace() = 0;

    /*!
     * \brief Initialize the workspace as a display to show OpenCV images.
     *
     * This is a convinience function to use if only one image gui element is
     * contained in the workspace.
     *
     * Use \code showCvImage() \endcode to display an OpenCv image.
     *
     * All previous gui elements are erased.
     *
     * \return Index (handle) to CvImage widget instance.
     */ 
    virtual uint_t initWorkspaceAsCvImage() = 0;

    /*!
     * \brief Initialize the workspace as a window display to show GStreamer
     * video/images.
     *
     * GStreamer automatically displays video/images via the GTK callback
     * mechanism, so no show function is required.
     *
     * This is a convinience function to use if only one stream is
     * contained in the workspace.
     *
     * All previous gui elements are erased.
     */ 
    virtual void initWorkspaceAsGstWin(const cv::Size &sizeVidWin) = 0;

     /*!
     * \brief Add an OpenCV display widget to the container widget. 
     *
     * The container widget must be added to the window workspace by
     * the application to keep the widget control correct.
     *
     * \param wContainer  Pointer to the container widget.
     *                    (gui toolkit specific).
     *
     * \return On success, returns index (handle) to CvImage instance.\n
     * On failure, returns -1.
     */
    virtual uint_t addWorkspaceCvImageDisplay(void *wContainer) = 0;

    /*!
     * \brief Remove OpenCV display widget from workspace.
     *
     * \param uImgIndex   Image instance.
     */
    virtual void removeWorkspaceCvImageDisplay(uint_t uImgIndex) = 0;

    /*!
     * \brief Show/refresh all gui widgets in workspace.
     */
    virtual void showWorkspace() = 0;
    
    /*!
     * \brief Remove all widgets contained in the window's workspace container.
     */
    virtual void eraseWorkspace() = 0;

    /*!
     * \brief Get GStreamer X-Window id.
     *
     * \return Xid.
     */
    ulong_t getGstXid()
    {
      return m_uGstWinXid;
    }

    /*!
     * \brief Register application mouse event callback.
     *
     * \param funcCb      Mouse callback function.
     * \param param       User provided data given on callback.
     * \param uImgIndex   Image instance.
     */
    void registerCvImageMouseCallback(MouseCbFunc_T  funcCb,
                                      void          *param,
                                      uint_t         uImgIndex=0)
    {
      if( uImgIndex < MaxCvImages )
      {
        m_funcMouseCb[uImgIndex] = funcCb;
        m_dataMouseCb[uImgIndex] = param;
      }
    }

    /*!
     * \brief Unregister application mouse event callback.
     *
     * \param uImgIndex   Image instance.
     */
    void unregisterCvImageMouseCallback(uint_t uImgIndex=0)
    {
      if( uImgIndex < MaxCvImages )
      {
        m_funcMouseCb[uImgIndex] = NULL;
        m_dataMouseCb[uImgIndex] = NULL;
      }
    }

    /*!
     * \brief Register application keyboard press event callback.
     *
     * \param funcCb    Keyboard callback function.
     * \param param     User provided data given on callback.
     */
    void registerKeyboardCallback(KeyCbFunc_T funcCb, void *param)
    {
      m_funcKeyCb = funcCb;
      m_dataKeyCb = param;
    }

    /*!
     * \brief Unregister application keyboard press event callback.
     */
    void unregisterKeyboardCallback()
    {
      m_funcKeyCb = NULL;
      m_dataKeyCb = NULL;
    }

    /*!
     * \brief Register application button click event callback.
     *
     * \param funcCb    Button click callback function.
     * \param param     User provided data given on callback.
     */
    void registerButtonClickCallback(BttnCbFunc_T funcCb, void *param)
    {
      m_funcBttnCb = funcCb;
      m_dataBttnCb = param;
    }

    /*!
     * \brief Unregister application button click event callback.
     */
    void unregisterButtonClickCallback()
    {
      m_funcBttnCb = NULL;
      m_dataBttnCb = NULL;
    }

  protected:
    typedef std::map<std::string, std::string> MapLookFeel_T;

    std::string     m_strWinName;     ///< window name (and title)
    int             m_nWinWidth;      ///< window width
    int             m_nWinHeight;     ///< window height
    bool            m_bDecorate;      ///< do [not] decorate window
    CvRect          m_rectWorkspace;  ///< workspace bounding rectangle
    ulong_t         m_uGstWinXid;     ///< GstWin container X window id
    uint_t          m_uLastKey;       ///< last pressed keyboard code
    bool            m_bMouseEvent;    ///< was a mouse event (click, drag, etc)
    MouseCbFunc_T   m_funcMouseCb[MaxCvImages]; ///< registered mouse cb func
    void           *m_dataMouseCb[MaxCvImages]; ///< registered mouse cb data
    KeyCbFunc_T     m_funcKeyCb;      ///< registered keyboard callback func
    void           *m_dataKeyCb;      ///< registered keyboard callback data
    BttnCbFunc_T    m_funcBttnCb;     ///< registered menu button callback func
    void           *m_dataBttnCb;     ///< registered menu button callback data
    MapLookFeel_T   m_mapLookFeel;    ///< look and feel map

    /*!
     * \brief Set look and feel standard set defaults.
     */
    void setLookAndFeelDefaults();
  };
 
} // rnr namespace


#endif // _RNR_WIN_H
