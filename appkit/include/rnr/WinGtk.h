////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_wingtk
//
// File:      WinGtk.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics GTK derived WinGtk window class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2013.  RoadNarrows
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

#ifndef _RNR_WIN_GTK_H
#define _RNR_WIN_GTK_H

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <pango/pango.h>

#include "rnr/WinLookFeel.h"
#include "rnr/Win.h"

//
// Windowing Declarations
//
namespace rnr
{
  //...........................................................................
  // Class WinGtk
  //...........................................................................

  /*!
   * \brief WinGtk window derived class.
   *
   * The WinGtk class uses the GTK (GIMP Toolkit) libraries for creating the
   * graphical user interfaces.
   */
  class WinGtk : public Win
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \param strWinName          Window name.
     * \param nWidth              Window width in pixels.
     * \param nHeight             Window height in pixels.
     * \param bDecorate           Do [not] decorate window with title, border,
     *                            etc. Windowing system specific.
     */
     WinGtk(const std::string &strWinName,
            int                nWidth,
            int                nHeight,
            bool               bDecorate=true);

    /*!
     * \brief Destructor.
     */
    virtual ~WinGtk();

    /*!
     * \brief Get GUI toolkit name.
     *
     * \return GUI toolkit identifier string.
     */
    virtual const char *getGuiToolkitName()
    {
      return "gtk+";
    }

    /*!
     * \brief Set GUI look and feel value.
     *
     * \param strKey  Key.
     * \param strVal  Value.
     */
    virtual void setLookAndFeel(const std::string &strKey,
                                const std::string &strVal);

    /*!
     * \brief Get the GUI look and feel GDK color.
     *
     * \param strKey    Key.
     *
     * \return Pointer to GDK color value.
     */
    virtual GdkColor *getLookAndFeelGdkColor(const std::string &strKey)
    {
      if( m_mapColors.find(strKey) != m_mapColors.end() )
      {
        return &m_mapColors[strKey];
      }
      else
      {
        return &m_mapColors["color_rn_black"];
      }
    }

    /*!
     * \brief Get the GUI look and feel Pango font description.
     *
     * \param strKey    Key.
     *
     * \return Pointer to pango font description value.
     */
    virtual PangoFontDescription *getLookAndFeelFont(const std::string &strKey)
    {
      if( m_mapFonts.find(strKey) != m_mapFonts.end() )
      {
        return m_mapFonts[strKey];
      }
      else
      {
        return m_mapFonts["color_rn_black"];
      }
    }

    /*!
     * \brief Show/refresh all gui widgets in window.
     */
    virtual void show()
    {
      gtk_widget_show_all(m_wWorkspace);
      gtk_widget_show(m_wMain);
    }

    /*!
     * \brief Show a OpenCV image on the workspace.
     *
     * The workspace must contain a CvImage prior to calling this function.
     *
     * \param pImg        Pointer to image.
     * \param uImgIndex   Image instance.
     */
    virtual void showCvImage(cv::Mat &img, uint_t uImgIndex=0);

    /*!
     * \brief Show page reference number.
     *
     * \param nPageRef   Page reference number.
     */
    virtual void showPageRef(int nPageId);

    /*!
     * \brief Show page reference.
     *
     * \param strPageRef    Page reference string.
     */
    virtual void showPageRef(const std::string &strPageRef);

    /*!
     * \brief Show status message.
     *
     * \param sFmt  Format string.
     * \param ...   Variable arguments.
     */
    virtual void showStatus(const char *sFmt, ...);

    /*!
     * \brief Clear status message.
     */
    virtual void clearStatus();

    /*!
     * \brief Wait for keypress or timeout.
     *
     * GTK widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds. Set to 0 for no timeout.
     *
     * \return Returns code of last key pressed or -1 if timed out.
     */
    virtual int waitKey(int delay);

    /*!
     * \brief Wait for timeout or registered mouse event.
     *
     * Window widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds \h_gt 0.
     */
    virtual void  waitMouse(int delay);

    /*!
     * \brief Wait for timeout.
     *
     * GTK widgets can be updated during this wait.
     *
     * \param delay   Timeout delay in millseconds \h_gt 0.
     */
    virtual void wait(int delay);
    
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
                                const std::string &strToolTip="");

    /*!
     * \brief Add a button with a GTK image widget to window button menu.
     *
     * \param nBttnId       Window unique button id.
     * \param eAlign        Menu item icon alignment. See \ref rnmpwin_align
     * \param wBttnImg      Button image widget (GtkWidget *).
     * \param strAltText    Button alternate text.
     * \param strToolTip    Optional button tool tip text.
     *
     * \return Returns true on success, false otherwise.
     */
    virtual bool addImageButton(int                nBttnId,
                                AlignOp            eAlign,
                                void              *wBttnImg,
                                const std::string &strAltText,
                                const std::string &strToolTip="");

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
                                const std::string &strToolTip="");

    /*!
     * \brief Add a GTK button widget to window button menu.
     *
     * \param nBttnId       Unique button id.
     * \param eAlign        Menu item icon alignment. See \ref rnmpwin_align.
     * \param wBttn         Button widget.
     */
    virtual bool addButton(int nBttnId, AlignOp eAlign, GtkWidget *wBttn);

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
                                    const std::string &strToolTip="");


    /*!
     * \brief Replace existing button image with new GTK image widget.
     *
     * \param nBttnId       Unique button id.
     * \param wBttnImg      New button image widget (GtkWidget *).
     * \param strToolTip    Optional button tool tip text. If empty, then the
     *                      current tooltip, if any, is used.
     *
     * \return Returns true if button image is replaced, false otherwise.
     */
    virtual bool replaceButtonImage(int                nBttnId,
                                    void              *wBttnImg,
                                    const std::string &strToolTip="");

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
                                    const std::string &strToolTip="");


    /*!
     * \brief Remove a button from a menu.
     *
     * \param nBttnId   Unique button id.
     *
     * \return Returns true if button is removed, false otherwise.
     */
    virtual bool removeButton(int nBttnId);

    /*!
     * \brief Remove all buttons from all menus.
     */
    virtual void removeAllButtons();

    /*!
     * \brief Show a button's widget state.
     *
     * \param nBttnId     Unique button id.
     * \param eBttnState  New button state. See \ref rnmpwin_itemstate.
     *
     * \return Returns true if button state set, false otherwise.
     */
    virtual bool showButtonState(int nBttnId, WidgetState eBttnState);

    /*!
     * \brief Initialize window's workspace to hold a GTK toolkit
     * widget tree.
     *
     * All previous gui elements are erased.
     */
    virtual void initWorkspace()
    {
      initWorkspaceAsGtk();
    }

    /*!
     * \brief Initialize window's workspace to hold a GTK widget tree.
     *
     * All previous gui elements are erased.
     */
    virtual void initWorkspaceAsGtk();

    /*!
     * \brief Initialize the workspace as a display to show OpenCV images.
     *
     * This is a convinience function to use if only one image widget is
     * contained in the workspace.
     *
     * Alternately, for a mixed workspace of GTK widgets and up to
     * MaxCvImages(4) OpenCV images, the follow functions are available:
     * \code
     * initWorkspace() or initWorkSpaceAsGtk()
     * addWorkspaceCvImageDisplay()
     * \endcode
     *
     * Use \code showCvImage() \endcode to display an OpenCv image.
     *
     * All previous gui elements are erased.
     *
     * \return Index (handle) to CvImage widget instance.
     */ 
    virtual uint_t initWorkspaceAsCvImage();

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
    virtual void initWorkspaceAsGstWin(const cv::Size &sizeVidWin);

    /*!
     * \brief Add an OpenCV display widget to the container widget. 
     *
     * The container widget must be added to the window workspace by
     * the application to keep the widget control correct.
     *
     * \param wContainer  Pointer to (derived) GTK_BOX container widget.
     *
     * \return On success, returns index (handle) to CvImage instance.\n
     * On failure, returns -1.
     */
    virtual uint_t addWorkspaceCvImageDisplay(void *wContainer);

    /*!
     * \brief Remove OpenCV display widget from workspace.
     *
     * \param uImgIndex   Image instance.
     */
    virtual void removeWorkspaceCvImageDisplay(uint_t uImgIndex);

    /*!
     * \brief Show/refresh all gui widgets in workspace.
     */
    virtual void showWorkspace()
    {
      gtk_widget_show_all(m_wWorkspace);
      //gtk_widget_show(m_wMain);
    }

    /*!
     * \brief Remove all widgets contained in the window's workspace container.
     *
     * \note A widget is actually destroyed iff there are no references to
     *        the widget.
     */
    virtual void eraseWorkspace();

    /*!
     * \brief Get the top-level main widget.
     *
     * \return Pointer to widget.
     */
    GtkWidget *getMainWidget()
    {
      return m_wMain;
    }

    /*!
     * \brief Get the top-level main window.
     *
     * \return Pointer to window widget.
     */
    GtkWindow *getMainWindowWidget()
    {
      return m_wWindowMain;
    }

    /*!
     * \brief Get the window's workspace top vertical box container widget.
     *
     * \return Pointer to GtkVBox widget.
     */
    GtkWidget *getWorkspaceVBox()
    {
      return m_wWorkspace;
    }

    /*!
     * \brief Get the window's workspace extra vertical box container widget.
     *
     * This widget is between the worksapce vertical box and the status line
     * and is contained within the workspace. It is only visible if the
     * application adds gui elements to it.
     *
     * \return Pointer to GtkVBox widget.
     */
    GtkWidget *getExtraVBox()
    {
      return m_wExtraVBox;
    }

    /*!
     * \brief Get GStreamer X-Window widget.
     *
     * \return Pointer to GStreamer bound widget.
     */
    GtkWidget *getGstWin()
    {
      return m_wGstWin;
    }
 
  protected:
    /*!
     * Internal button information structure
     */
    struct  BttnInfo_T
    {
      int         m_nBttnId;          ///< unique button id
      GtkWidget  *m_wBttn;            ///< button widget
      WinGtk     *m_pWin;             ///< this
    };

    typedef std::map<int, BttnInfo_T> MapBttns_T;    ///< button map type
    typedef std::map<std::string, GdkColor>  MapGdkColor_T; ///< color map type
    typedef std::map<std::string, PangoFontDescription *>  MapPangoFont_T;
                                                ///< font map type
    // GUI Widgets
    GtkWidget      *m_wMain;          ///< main widget
    GtkWindow      *m_wWindowMain;    ///< main widnow widget
    GtkWidget      *m_wContainer;     ///< top container widget (hbox)
    GtkWidget      *m_wBttnMenu[NumOfButtonMenus];
                                      ///< left menu container widget
    GtkWidget      *m_wWorkspace;     ///< middle workspace container widget
    GtkWidget      *m_wExtraVBox;     ///< extra vertical box
    GtkWidget      *m_wIcon;          ///< workspace identifying icon
    GtkWidget      *m_wStatusFrame;   ///< holds page reference and status bar
    GtkWidget      *m_wPageRef;       ///< workflow page reference
    GtkWidget      *m_wStatusBar;     ///< status message bar

    // Image/Video Widgets and Data
    GtkWidget      *m_wCvImage[MaxCvImages];      ///< CvImage widgets
    GtkWidget      *m_wCvImageBox[MaxCvImages];   ///< CvImage box containers
    GtkWidget      *m_wGstWin;                    ///< GstWin container widget
    CvSize          m_sizeCvImage[MaxCvImages];   ///< size of shown CvImage
    CvPoint         m_ptCvImageOrig[MaxCvImages]; ///< origin of shown CvImage

    // Menu Button Info
    MapBttns_T      m_mapBttns[NumOfButtonMenus];
                                      ///< left/right map of menu buttons.

    // Look and Feel
    MapGdkColor_T   m_mapColors;      ///< look and feel color map
    MapPangoFont_T  m_mapFonts;       ///< look and feel font map

    /*!
     * \brief Get button menu index from alignment enum.
     *
     * \param eAlign  Alignment enumuration.
     *
     * \return Menu index.
     */
    inline int getBttnMenuIdx(AlignOp eAlign)
    {
      return eAlign == AlignOpRight? 1: 0;
    }

    /*!
     * \brief Find button.
     *
     * \param nBttnId   Unique button id.
     * \param pos       Position iterator. Set to end(), if no button found.
     *
     * \return Returns true if button found, false otherwise.
     */
    bool findBttn(int nBttnId, MapBttns_T::iterator &pos)
    {
      if( (pos = m_mapBttns[0].find(nBttnId)) == m_mapBttns[0].end() )
      {
        if( (pos = m_mapBttns[1].find(nBttnId)) == m_mapBttns[1].end() )
        {
          return false;
        }
      }
      return true;
    }

    /*!
     * \brief Convert Look and Feel defaults to native GTK/GDK values.
     */
    void convertLookAndFeelDefaults();
    
    /*!
     * \brief Configure OpenCV image widget.
     *
     * \param uImgIndex Image instance.
     */
    virtual void configureCvImage(uint_t uImgIndex);
    
    /*!
     * \brief Remove widget callback handler.
     *
     * \param w           Widget to be removed.
     * \param user_data   Any supplied callback data (parent container widget).
     */
    static void onRemoveWidget(GtkWidget *w, gpointer user_data);

    /*!
     * \brief Timeout expiry callback handler.
     *
     * The supplied user data is set to 1 (true).
     *
     * \param user_data   Pointer to user supplied expiry flag.
     *
     * \return Returns FALSE.
     */
    static gboolean onAlarm(gpointer user_data);

    /*!
     * \brief Keyboard press event callback handler.
     *
     * If registered, the application callback function will be called.
     *
     * \param w         Widget where keyboard event occurred.
     * \param event     Keyboard event.
     * \param user_data Supplied user data (this).
     *
     * \return Returns FALSE.
     */
    static gboolean onKeyPress(GtkWidget   *w,
                               GdkEventKey *event,
                               gpointer    *user_data);

    /*!
     * \brief Mouse event on OpenCV Image widget workspace callback handler.
     *
     * The workspace must be set as an CvImage workspace.
     *
     * If registered, the application callback function will be called.
     *
     * The pixel coordinates are contrained to any shown OpenCv image with the
     * origin at the image origin. Mouse events outside the image return
     * nopoint().
     *
     * \param w         CvImage widget where mouse event occurred.
     * \param event     Mouse event.
     * \param user_data Supplied user data (this).
     *
     * \return Returns FALSE.
     */
    static gboolean onMouseCvImage(GtkWidget   *w,
                                   GdkEventKey *event,
                                   gpointer    *user_data);

    /*!
     * \brief Button press event handler.
     *
     * If registered, the application callback function will be called.
     *
     * \param w           Button widget.
     * \param user_data   Button information
     */
    static void onButtonClick(GtkWidget *w, gpointer *user_data);

    /*!
     * \brief Realize GStreamer video window callback.
     *
     * Once the window has been realized, the X-Window id can be obtained. The
     * id is critical for rendering gst video and images to the gtk widget.
     *
     * \param w         Gtk draw widget where video will be overlaied.
     * \param user_data Supplied user data (this).
     */
    static void onRealizeGstWin(GtkWidget *w, gpointer user_data);
  };

} // rnr namespace


#endif // _RNR_WIN_GTK_H
