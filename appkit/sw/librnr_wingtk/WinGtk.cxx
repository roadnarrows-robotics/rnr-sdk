////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Roboitics Windowing Package
//
// Library:   librnr_wingtk
//
// File:      WinGtk.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-15 11:45:50 -0600 (Mon, 15 Jul 2013) $
 * $Rev: 3131 $
 *
 * \brief RoadNarrows GTK derived window class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows
 * (http://www.RoadNarrows.com)
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

#include <sys/types.h>
#include <stdarg.h>
#include <libgen.h>
#include <unistd.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/WinLookFeel.h"
#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinGtk.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <gtk/gtk.h>
#include <gdk/gdk.h>
#include <gdk/gdkkeysyms.h>
#include <gdk/gdkx.h>

using namespace std;
using namespace cv;
using namespace rnr;

//
// OpenCv public but unpublizied functions and types
//
typedef struct _CvImageWidget CvImageWidget;
extern GtkWidget *cvImageWidgetNew(int flags);
extern void cvImageWidgetSetImage(CvImageWidget *widget, const CvArr *arr);


//------------------------------------------------------------------------------
// Class WinGtk
//------------------------------------------------------------------------------

WinGtk::WinGtk(const string &strWinName,
               int           nWidth,
               int           nHeight,
               bool          bDecorate) :
    Win(strWinName, nWidth, nHeight, bDecorate)

{
  const int  nHBoxBorder       =  6;    // hbox outside border width
  const int  nMenuWidth        = 48;    // button menu width
  const int  nStatusBarHeight  = 20;    // status bar height
  const int  nWorkspaceBorder  =  4;    // workspace outside border width

  int         nWorkspaceWidth;
  int         nWorkspaceHeight = nHeight - nStatusBarHeight;
  GtkWidget  *wHBox;
  GtkWidget  *wVBox;
  GtkWidget  *w;
  int         i;

  nWorkspaceWidth  = nWidth - (nHBoxBorder + nMenuWidth + nWorkspaceBorder) * 2;
  nWorkspaceHeight = nHeight - nHBoxBorder - nStatusBarHeight 
                             - nWorkspaceBorder;

  // default colors & fonts
  convertLookAndFeelDefaults();

  // required
  gtk_init(0, NULL);

  // create a new gtk window
  m_wMain = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  m_wWindowMain = GTK_WINDOW(m_wMain);
  
  //
  // Configure window's look and feel
  //
  gtk_window_set_title(m_wWindowMain, m_strWinName.c_str());
  gtk_window_resize(m_wWindowMain, m_nWinWidth, m_nWinHeight);
  gtk_window_set_decorated(m_wWindowMain, bDecorate);
  gtk_widget_modify_bg(m_wMain, GTK_STATE_NORMAL, &m_mapColors["color_win_bg"]);
  //gtk_widget_modify_base(m_wMain, GTK_STATE_NORMAL,
  //    &m_mapColors["color_win_bg"]);
 
  //
  // Top-Level Container.
  //
  // Each window can only have one direct descendent widget. This is it: a 
  // vertical box that will contaion all child widgets.
  //
  m_wContainer = gtk_vbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(m_wMain), m_wContainer);

  //
  // The left-menu, workspace, right-menu horizontal box container
  //
  wHBox = gtk_hbox_new(FALSE, 0);
  gtk_container_set_border_width(GTK_CONTAINER(wHBox), nHBoxBorder);
  gtk_box_pack_start(GTK_BOX(m_wContainer), wHBox, FALSE, FALSE, 0);

  // 
  // Left menu vertical button box container
  //
  w = gtk_vbutton_box_new();
  gtk_button_box_set_layout(GTK_BUTTON_BOX(w), GTK_BUTTONBOX_START);
  g_object_set(G_OBJECT(w),
      "width-request", nMenuWidth,
      "height-request", nWorkspaceHeight,
      NULL);
  gtk_container_set_border_width(GTK_CONTAINER(w), 2);
  gtk_button_box_set_child_size(GTK_BUTTON_BOX(w), nMenuWidth, nMenuWidth);
  gtk_button_box_set_child_ipadding(GTK_BUTTON_BOX(w), 0, 0);
  gtk_box_set_spacing(GTK_BOX(w), 1);
  gtk_box_pack_start(GTK_BOX(wHBox), w, TRUE, TRUE, 0);
  m_wBttnMenu[getBttnMenuIdx(AlignOpLeft)] = w;

  //
  // Middle workspace vertical box container
  //
  m_wWorkspace = gtk_vbox_new(FALSE, 0);
  g_object_set(G_OBJECT(m_wWorkspace),
      "width-request", nWorkspaceWidth,
      "height-request", nWorkspaceHeight,
      NULL);
  gtk_widget_modify_bg(m_wWorkspace, GTK_STATE_NORMAL,
      &m_mapColors["color_win_bg"]);
  gtk_container_set_border_width(GTK_CONTAINER(m_wWorkspace), nWorkspaceBorder);
  gtk_box_pack_start(GTK_BOX(wHBox), m_wWorkspace, FALSE, FALSE, 0);
  m_rectWorkspace = cvRect(nMenuWidth, 0, nWorkspaceWidth, nWorkspaceHeight);
  for(i=0; i<MaxCvImages; ++i)
  {
    m_wCvImage[i]    = NULL;
    m_wCvImageBox[i] = NULL;
  }
  m_wGstWin     = NULL;

  // 
  // Right menu vertical button box container
  //
  w = gtk_vbutton_box_new();
  gtk_button_box_set_layout(GTK_BUTTON_BOX(w), GTK_BUTTONBOX_START);
  g_object_set(G_OBJECT(w),
      "width-request", nMenuWidth,
      "height-request", nWorkspaceHeight,
      NULL);
  gtk_button_box_set_child_size(GTK_BUTTON_BOX(w), nMenuWidth, nMenuWidth);
  gtk_button_box_set_child_ipadding(GTK_BUTTON_BOX(w), 0, 0);
  gtk_box_set_spacing(GTK_BOX(w), 1);
  gtk_box_pack_start(GTK_BOX(wHBox), w, TRUE, TRUE, 0);
  m_wBttnMenu[getBttnMenuIdx(AlignOpRight)] = w;

  //
  // Extra widgets vbox
  //
  m_wExtraVBox = gtk_vbox_new(FALSE, 0);
  g_object_set(G_OBJECT(m_wExtraVBox),
      "width-request", nWorkspaceWidth,
      NULL);
  gtk_box_pack_start(GTK_BOX(m_wContainer), m_wExtraVBox, FALSE, FALSE, 0);

  //
  // Bottom status.
  //
  m_wStatusFrame = gtk_frame_new(NULL);
  gtk_box_pack_start(GTK_BOX(m_wContainer), m_wStatusFrame, FALSE, FALSE, 0);
  gtk_widget_modify_bg(m_wStatusFrame, GTK_STATE_NORMAL,
      &m_mapColors["color_status_border"]);
  gtk_container_set_border_width(GTK_CONTAINER(m_wStatusFrame), 3);

  wHBox = gtk_hbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(m_wStatusFrame), wHBox);

  // page reference
  m_wPageRef = gtk_label_new("");
  gtk_widget_modify_fg(m_wPageRef, GTK_STATE_NORMAL,
      &m_mapColors["color_status_fg"]);
  gtk_widget_modify_bg(m_wPageRef, GTK_STATE_NORMAL,
      &m_mapColors["color_status_bg"]);
  g_object_set(G_OBJECT(m_wPageRef),
      "width-request",  nMenuWidth,
      "height-request", nStatusBarHeight,
      NULL);
  gtk_label_set_justify(GTK_LABEL(m_wPageRef), GTK_JUSTIFY_LEFT);
  gtk_box_pack_start(GTK_BOX(wHBox), m_wPageRef, FALSE, FALSE, 4);

  // status bar
  m_wStatusBar = gtk_label_new("");
  gtk_widget_modify_fg(m_wStatusBar, GTK_STATE_NORMAL,
      &m_mapColors["color_status_fg"]);
  gtk_widget_modify_bg(m_wStatusBar, GTK_STATE_NORMAL,
      &m_mapColors["color_status_bg"]);
  g_object_set(G_OBJECT(m_wStatusBar),
      "width-request", nWorkspaceWidth-nMenuWidth,
      "height-request", nStatusBarHeight,
      NULL);
  gtk_label_set_justify(GTK_LABEL(m_wStatusBar), GTK_JUSTIFY_LEFT);
  gtk_misc_set_alignment(GTK_MISC(m_wStatusBar), 0, 0.5);
  gtk_box_pack_start(GTK_BOX(wHBox), m_wStatusBar, FALSE, FALSE, 4);

  //
  // Install event handlers
  //
  gtk_signal_connect(GTK_OBJECT(m_wMain), "key-press-event",
                     GTK_SIGNAL_FUNC(WinGtk::onKeyPress), this);


  // show window
  gtk_widget_show_all(m_wMain);
}

WinGtk::~WinGtk()
{
  MapPangoFont_T::iterator  iter;

  gtk_widget_destroy(m_wMain);

  // delete all font decsriptors
  for(iter=m_mapFonts.begin(); iter!=m_mapFonts.end(); ++iter)
  {
    pango_font_description_free(iter->second);
  }
}

void WinGtk::setLookAndFeel(const std::string &strKey,
                            const std::string &strVal)
{
  GdkColor              color;
  PangoFontDescription *pFont;

  Win::setLookAndFeel(strKey, strVal);

  if( strKey.find("color") == 0 )
  {
    gdk_color_parse(strVal.c_str(), &color);
    m_mapColors[strKey] = color;

    if( strKey == "color_win_bg" )
    {
      gtk_widget_modify_bg(m_wMain, GTK_STATE_NORMAL, &color);
      gtk_widget_modify_base(m_wMain, GTK_STATE_NORMAL, &color);
      gtk_widget_modify_bg(m_wWorkspace, GTK_STATE_NORMAL, &color);
    }
    else if( strKey == "color_status_fg" )
    {
      gtk_widget_modify_fg(m_wPageRef, GTK_STATE_NORMAL, &color);
      gtk_widget_modify_fg(m_wStatusBar, GTK_STATE_NORMAL, &color);
    }
    else if( strKey == "color_status_bg" )
    {
    }
    else if( strKey == "color_status_border" )
    {
      gtk_widget_modify_bg(m_wStatusFrame, GTK_STATE_NORMAL, &color);
    }
    else if( strKey == "color_button_bg" )
    {
      for(size_t i=0; i<NumOfButtonMenus; ++i)
      {
        for(size_t j=0; j<m_mapBttns[i].size(); ++i)
        {
          gtk_widget_modify_bg(m_mapBttns[i][j].m_wBttn, GTK_STATE_NORMAL,
              &color);
          gtk_widget_modify_bg(m_mapBttns[i][j].m_wBttn, GTK_STATE_INSENSITIVE,
              &color);
        }
      }
    }
  }

  else if( strKey.find("font") == 0 )
  {
    pFont = pango_font_description_from_string(strVal.c_str());
    m_mapFonts[strKey] = pFont;
  }

  showWorkspace();
}

void WinGtk::showCvImage(Mat &img, uint_t uImgIndex)
{
  if( (uImgIndex < MaxCvImages) && (m_wCvImage[uImgIndex] != NULL) )
  {
    uint_t  i = uImgIndex;

    cvImageWidgetSetImage((CvImageWidget *)m_wCvImage[i], &img);
    m_sizeCvImage[i] = Size(img.cols, img.rows);
    m_ptCvImageOrig[i].x = (m_rectWorkspace.width - m_sizeCvImage[i].width) / 2;
    m_ptCvImageOrig[i].y = (m_rectWorkspace.height - m_sizeCvImage[i].height)/2;
    if( m_ptCvImageOrig[i].x < 0 )
    {
      m_ptCvImageOrig[i].x = 0;
    }
    if( m_ptCvImageOrig[i].y < 0 )
    {
      m_ptCvImageOrig[i].y = 0;
    }
    gtk_widget_show(m_wCvImage[i]);
  }
  else
  {
    LOGERROR("Workspace has no CvImage[%u] widget.", uImgIndex);
  }
}

void WinGtk::showPageRef(int nPageRef)
{
  char        buf[16];

  // format status text
  snprintf(buf, sizeof(buf), "%d", nPageRef);
  buf[sizeof(buf)-1] = 0;

  gtk_label_set_text(GTK_LABEL(m_wPageRef), buf);
  gtk_widget_show(m_wPageRef);
}

void WinGtk::showPageRef(const string &strPageRef)
{
  if( !strPageRef.empty() )
  {
    gtk_label_set_text(GTK_LABEL(m_wPageRef), strPageRef.c_str());
    gtk_widget_show(m_wPageRef);
  }
}

void WinGtk::showStatus(const char *sFmt, ...)
{
  char        buf[256];
  va_list     ap;  

  // format status text
  va_start(ap, sFmt);
  vsnprintf(buf, sizeof(buf), sFmt, ap);
  buf[sizeof(buf)-1] = 0;
  va_end(ap);

  gtk_label_set_text(GTK_LABEL(m_wStatusBar), buf);
  gtk_widget_show(m_wStatusBar);
}

/*!
 * \brief Clear status message.
 */
void WinGtk::clearStatus()
{
  gtk_label_set_text(GTK_LABEL(m_wStatusBar), "");
  gtk_widget_show(m_wStatusBar);
}

int WinGtk::waitKey(int delay)
{
  int     expired = 0;
  guint   timer = 0;
  int     key;

  // set alarm timer
  if( delay > 0 )
  {
    timer = g_timeout_add(delay, WinGtk::onAlarm, &expired);
  }

  // wait for keyboard press
  while( gtk_main_iteration_do(FALSE) &&
         (m_uLastKey == 0) &&
         !expired );

  // got a keyboard press
  if( !expired )
  {
    // cancel timer
    if( delay > 0 )
    {
      g_source_remove(timer);
    }
    key = (int)m_uLastKey;
    m_uLastKey = 0;
  }

  // expired without keyboard event
  else
  {
    key = -1;
  }

  return key;
}

void WinGtk::waitMouse(int delay)
{
  int expired = 0;
  guint timer = 0;

  // causes an infinite block
  if( delay <= 0 )
  {
    return;
  }

  timer = g_timeout_add(delay, WinGtk::onAlarm, &expired);

  m_bMouseEvent = false;

  while( gtk_main_iteration_do(FALSE) && !expired && !m_bMouseEvent );

  if( !expired )
  {
    m_bMouseEvent = false;
    g_source_remove(timer);
  }
}

void WinGtk::wait(int delay)
{
  int expired = 0;
  guint timer = 0;

  // causes an infinite block
  if( delay <= 0 )
  {
    return;
  }

  timer = g_timeout_add(delay, WinGtk::onAlarm, &expired);

  while( gtk_main_iteration_do(FALSE) && !expired );
}

bool WinGtk::addImageButton(int           nBttnId,
                            AlignOp       eAlign,
                            const string &strIconPath,
                            const string &strAltText,
                            const string &strToolTip)
{
  GtkWidget  *wBttnImg = NULL;
  GtkWidget  *wBttn;

  if( !strIconPath.empty() && (access(strIconPath.c_str(), F_OK|R_OK) == 0) )
  {
    wBttnImg = gtk_image_new_from_file(strIconPath.c_str());
  }

  if( wBttnImg == NULL )
  {
    return addLabelButton(nBttnId, eAlign, strAltText, strToolTip);
  }

  wBttn = gtk_button_new();

  gtk_widget_modify_bg(wBttn, GTK_STATE_NORMAL,
      &m_mapColors["color_button_bg"]);
  gtk_widget_modify_bg(wBttn, GTK_STATE_INSENSITIVE,
      &m_mapColors["color_button_bg"]);
  gtk_button_set_image(GTK_BUTTON(wBttn), wBttnImg);
  gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

  return addButton(nBttnId, eAlign, wBttn);
}

bool WinGtk::addImageButton(int           nBttnId,
                            AlignOp       eAlign,
                            void         *wBttnImg,
                            const string &strAltText,
                            const string &strToolTip)
{
  GtkWidget     *wBttn;

  if( wBttnImg != NULL )
  {
    wBttn = gtk_button_new();
    gtk_widget_modify_bg(wBttn, GTK_STATE_NORMAL,
        &m_mapColors["color_button_bg"]);
    gtk_widget_modify_bg(wBttn, GTK_STATE_INSENSITIVE,
        &m_mapColors["color_button_bg"]);
    gtk_button_set_image(GTK_BUTTON(wBttn), (GtkWidget *)wBttnImg);
    gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

    return addButton(nBttnId, eAlign, wBttn);
  }
  else
  {
    return addLabelButton(nBttnId, eAlign, strAltText, strToolTip);
  }
}

bool WinGtk::addLabelButton(int           nBttnId,
                            AlignOp       eAlign,
                            const string &strLabel,
                            const string &strToolTip)
{
  GtkWidget  *wBttn;

  if( !strLabel.empty() )
  {
    wBttn = gtk_button_new_with_label(strLabel.c_str());
  }
  else
  {
    char buf[16];
    snprintf(buf, sizeof(buf), "Bttn %d", nBttnId);
    buf[sizeof(buf)-1] = 0;
    wBttn = gtk_button_new_with_label(buf);
  }

  gtk_widget_modify_bg(wBttn, GTK_STATE_NORMAL,
      &m_mapColors["color_button_bg"]);
  gtk_widget_modify_bg(wBttn, GTK_STATE_INSENSITIVE,
      &m_mapColors["color_button_bg"]);
  gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

  return addButton(nBttnId, eAlign, wBttn);
}

bool WinGtk::addButton(int nBttnId, AlignOp eAlign, GtkWidget *wBttn)
{
  BttnInfo_T  info;
  int         n = getBttnMenuIdx(eAlign);

  info.m_nBttnId  = nBttnId;
  info.m_wBttn    = wBttn;
  info.m_pWin     = this;

  // add to button map
  m_mapBttns[n][nBttnId] = info;

  // bind button press event
  gtk_signal_connect(GTK_OBJECT(wBttn), "clicked",
                      GTK_SIGNAL_FUNC(onButtonClick),
                      &m_mapBttns[n][nBttnId]);

  gtk_box_pack_start(GTK_BOX(m_wBttnMenu[n]), wBttn, TRUE, TRUE, 0);

  // RDK TODO Need to center image with 1 px border to fit 5 buttons/side
  //GtkBorder bttnborder = {1,1,1,1};
  //gfloat    x, y;
  //gtk_button_set_image_position(GTK_BUTTON(wBttn), GTK_POS_RIGHT);
  gtk_button_set_alignment(GTK_BUTTON(wBttn), 0.5, 0.5);
  //g_object_set(G_OBJECT(wBttn),
  //             image-position", GTK_POS_RIGHT,
  //             "inner-border", &bttnborder,
  //             "image-spacing", 0,
  //             NULL);
  //gtk_widget_set_size_request(wBttn, 50, 50);

  // show
  gtk_widget_show(wBttn);
  gtk_widget_show(m_wMain);
  gtk_widget_queue_draw( GTK_WIDGET(wBttn) );

  return true;
}

bool WinGtk::replaceButtonImage(int           nBttnId,
                                const string &strIconPath,
                                const string &strToolTip)
{
  MapBttns_T::iterator  pos;
  GtkWidget            *wBttnImg = NULL;
  GtkWidget            *wBttn;

  if( !findBttn(nBttnId, pos) )
  {
    return false;
  }

  if( !strIconPath.empty() && (access(strIconPath.c_str(), F_OK|R_OK) == 0) )
  {
    wBttnImg = gtk_image_new_from_file(strIconPath.c_str());
  }

  if( wBttnImg == NULL )
  {
    return false;
  }

  wBttn = pos->second.m_wBttn;

  gtk_button_set_image(GTK_BUTTON(wBttn), wBttnImg);

  gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

  // redraw
  gtk_widget_show(wBttn);
  gtk_widget_show(m_wMain);

  return true;
}

bool WinGtk::replaceButtonImage(int           nBttnId,
                                void         *wBttnImg,
                                const string &strToolTip)
{
  MapBttns_T::iterator  pos;
  GtkWidget            *wBttn;

  if( !findBttn(nBttnId, pos) )
  {
    return false;
  }

  wBttn = pos->second.m_wBttn;

  gtk_button_set_image(GTK_BUTTON(wBttn), (GtkWidget *)wBttnImg);

  gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

  // redraw
  gtk_widget_show(wBttn);
  gtk_widget_show(m_wMain);

  return true;
}

bool WinGtk::replaceButtonLabel(int                nBttnId,
                                const std::string &strLabel,
                                const std::string &strToolTip)
{
  MapBttns_T::iterator  pos;
  GtkWidget            *wBttn;

  if( !findBttn(nBttnId, pos) )
  {
    return false;
  }

  wBttn = pos->second.m_wBttn;

  gtk_button_set_label(GTK_BUTTON(wBttn), strLabel.c_str());

  gtk_widget_set_tooltip_text(wBttn,
                              strToolTip.empty()? NULL: strToolTip.c_str());

  // redraw
  gtk_widget_show(wBttn);
  gtk_widget_show(m_wMain);

  return true;
}

bool WinGtk::removeButton(int nBttnId)
{
  MapBttns_T::iterator  pos;
  int                   n;
  GtkWidget            *wBttn;

  if( (pos = m_mapBttns[0].find(nBttnId)) != m_mapBttns[0].end() )
  {
    n = 0;
  }
  else if( (pos = m_mapBttns[1].find(nBttnId)) != m_mapBttns[1].end() )
  {
    n = 1;
  }
  else
  {
    return false;
  }

  wBttn = pos->second.m_wBttn;

  // 
  // Replace any exiting button image with NULL or the image widget is hosed
  // for reuse.
  //
  gtk_button_set_image(GTK_BUTTON(wBttn), NULL);

  // remove widget from gui
  gtk_container_remove(GTK_CONTAINER(m_wBttnMenu[n]), wBttn);

  // remove widget from map
  m_mapBttns[n].erase(pos);

  // redraw
  gtk_widget_show(m_wMain);

  return true;
}

void WinGtk::removeAllButtons()
{
  MapBttns_T::iterator  iter;
  int                   n;
  GtkWidget            *wBttn;

  for(n=0; n<2; ++n)
  {
    for(iter=m_mapBttns[n].begin(); iter != m_mapBttns[n].end(); ++iter)
    {
      wBttn = iter->second.m_wBttn;

      // 
      // Replace any exiting button image with NULL or the image widget is hosed
      // for reuse.
      //
      gtk_button_set_image(GTK_BUTTON(wBttn), NULL);

      // remove widget from gui
      gtk_container_remove(GTK_CONTAINER(m_wBttnMenu[n]), wBttn);
    }
    m_mapBttns[n].clear();
  }

  // redraw
  gtk_widget_show(m_wMain);
}

bool WinGtk::showButtonState(int nBttnId, WidgetState eBttnState)
{
  MapBttns_T::iterator  pos;

  if( !findBttn(nBttnId, pos) )
  {
    return false;
  }

  switch( eBttnState )
  {
    case WidgetStateNormal:
      gtk_widget_set_sensitive(pos->second.m_wBttn, TRUE);
      gtk_widget_set_state(pos->second.m_wBttn, GTK_STATE_NORMAL);
      break;
    case WidgetStateActive:
      gtk_widget_set_sensitive(pos->second.m_wBttn, TRUE);
      gtk_widget_set_state(pos->second.m_wBttn, GTK_STATE_ACTIVE);
      break;
    case WidgetStateDisabled:
      gtk_widget_set_sensitive(pos->second.m_wBttn, FALSE);
      break;
    default:
      return false;
  }

  gtk_widget_show_all(pos->second.m_wBttn);

  return true;
}

void WinGtk::initWorkspaceAsGtk()
{
  // remove all workspace widgets - nothing else required
  eraseWorkspace();
}

uint_t WinGtk::initWorkspaceAsCvImage()
{
  GtkWidget        *wAlign;
  
  // remove any old workspace widgets
  eraseWorkspace();

  // new CvImage widget
  m_wCvImage[0] = cvImageWidgetNew(CV_WINDOW_AUTOSIZE);

  // CvImage container box widget
  m_wCvImageBox[0] = m_wWorkspace;

  // center-top alignment
  wAlign = gtk_alignment_new(0.5, 0.0, 0, 0);
  gtk_box_pack_start(GTK_BOX(m_wWorkspace), wAlign, TRUE, TRUE, 0);
  
  // CvImage container box widget
  m_wCvImageBox[0] = gtk_hbox_new(FALSE, 0);
  gtk_container_add(GTK_CONTAINER(wAlign), m_wCvImageBox[0]);
  
  // initial size of widget is the size of the workspace
  m_sizeCvImage[0] = cvSize(m_rectWorkspace.width, m_rectWorkspace.height);

  // initial origin
  m_ptCvImageOrig[0] = cvPoint(0, 0);

  // pack at the start of the container widget
  gtk_box_pack_start(GTK_BOX(m_wCvImageBox[0]), m_wCvImage[0], TRUE, TRUE, 0);

  // configure the CvImage widget
  configureCvImage(0);

  // force show
  gtk_widget_show_all(m_wWorkspace);

  return 0;
}

void WinGtk::initWorkspaceAsGstWin(const Size &sizeVidWin)
{
  // remove any old workspace widgets
  eraseWorkspace();

  //---
  // Video window
  //---
  m_wGstWin = gtk_drawing_area_new();
  g_object_set(G_OBJECT(m_wGstWin),
      "width-request",  sizeVidWin.width,
      "height-request", sizeVidWin.height,
      NULL);
  gtk_widget_modify_bg(m_wGstWin, GTK_STATE_NORMAL,
      &m_mapColors["color_win_bg"]);
  gtk_widget_modify_bg(m_wGstWin, GTK_STATE_ACTIVE,
      &m_mapColors["color_win_bg"]);
  gtk_widget_modify_bg(m_wGstWin, GTK_STATE_SELECTED,
      &m_mapColors["color_win_bg"]);
  g_signal_connect(m_wGstWin, "realize", G_CALLBACK(onRealizeGstWin), this);
  gtk_widget_set_double_buffered(m_wGstWin, FALSE);
  gtk_box_pack_start(GTK_BOX(m_wWorkspace), m_wGstWin, FALSE, FALSE, 0);

  //---
  // Show
  //---
  showWorkspace();

  //
  // Realize the window now so that the video window gets created and we can
  // obtain its XID before the pipeline is started up and the videosink
  // asks for the XID of the window to render onto.
  //
  gtk_widget_realize(m_wGstWin);

  if( m_uGstWinXid == 0 )
  {
    LOGERROR("Cannot determine xid.");
  }
}

uint_t WinGtk::addWorkspaceCvImageDisplay(void *wContainer)
{
  GtkWidget *wBox = (GtkWidget *)wContainer;
  uint_t     i;

  for(i=0; i<MaxCvImages; ++i)
  {
    if( m_wCvImage[i] == NULL )
    {
      break;
    }
  }

  if( i >= MaxCvImages )
  {
    LOGERROR("No more CvImage displays available.");
    return -1;
  }

  // new CvImage widget
  m_wCvImage[i] = cvImageWidgetNew(CV_WINDOW_AUTOSIZE);

  // CvImage container box widget
  m_wCvImageBox[i] = wBox;

  // initial size of widget is unknown
  m_sizeCvImage[i] = cvSize(0, 0);
  m_ptCvImageOrig[i] = cvPoint(0, 0);

  // pack at the start of the container widget
  gtk_box_pack_start(GTK_BOX(m_wCvImageBox[i]), m_wCvImage[i], TRUE, TRUE, 0);

  // configure the CvImage widget
  configureCvImage(i);

  return i;
}

void WinGtk::removeWorkspaceCvImageDisplay(uint_t uImgIndex)
{
  // remove (and destroy) old CvImage widget
  if( (uImgIndex < MaxCvImages) && (m_wCvImage[uImgIndex] != NULL) )
  {
    gtk_container_remove(GTK_CONTAINER(m_wCvImageBox[uImgIndex]),
                                      m_wCvImage[uImgIndex]);
    m_wCvImage[uImgIndex]     = NULL;
    m_wCvImageBox[uImgIndex]  = NULL;
  }
}

void WinGtk::eraseWorkspace()
{
  int   i;

  gtk_container_foreach(GTK_CONTAINER(m_wWorkspace),
                        WinGtk::onRemoveWidget,
                        m_wWorkspace);

  for(i=0; i<MaxCvImages; ++i)
  {
    m_wCvImage[i]     = NULL;
    m_wCvImageBox[i]  = NULL;
  }
}


//..............................................................................
// Protected Implementation
//..............................................................................

void WinGtk::convertLookAndFeelDefaults()
{
  GdkColor                  color;
  PangoFontDescription     *pFont;
  MapLookFeel_T::iterator   iter;

  for(iter=m_mapLookFeel.begin(); iter!=m_mapLookFeel.end(); ++iter)
  {
    if( iter->first.find("color") == 0 )
    {
      gdk_color_parse(iter->second.c_str(), &color);
      m_mapColors[iter->first] = color;
    }
    else if( iter->first.find("font") == 0 )
    {
      pFont = pango_font_description_from_string(iter->second.c_str());
      m_mapFonts[iter->first] = pFont;
    }
  }
}

void WinGtk::configureCvImage(uint_t uImgIndex)
{
  uint_t      i = uImgIndex;

  gtk_widget_modify_bg(m_wCvImage[i], GTK_STATE_NORMAL,
      &m_mapColors["color_image_bg"]);

  gtk_signal_connect(GTK_OBJECT(m_wCvImage[i]), "button-press-event",
                        GTK_SIGNAL_FUNC(WinGtk::onMouseCvImage), this);
  gtk_signal_connect(GTK_OBJECT(m_wCvImage[i]), "button-release-event",
                        GTK_SIGNAL_FUNC(WinGtk::onMouseCvImage), this);
  gtk_signal_connect(GTK_OBJECT(m_wCvImage[i]), "motion-notify-event",
                        GTK_SIGNAL_FUNC(WinGtk::onMouseCvImage), this);

  gtk_widget_show(m_wCvImage[i]);
  gtk_widget_show(m_wMain);
  gtk_widget_queue_draw(GTK_WIDGET(m_wCvImage[i]));
}

void WinGtk::onRemoveWidget(GtkWidget *w, gpointer user_data)
{
  GtkWidget *wContainer = (GtkWidget *)user_data;
  gtk_container_remove(GTK_CONTAINER(wContainer), w);
}

gboolean WinGtk::onAlarm(gpointer user_data)
{
  *(int *)user_data = 1;
  return FALSE;
}

gboolean WinGtk::onKeyPress(GtkWidget   *w,
                            GdkEventKey *event,
                            gpointer    *user_data)
{
  WinGtk *pWin = (WinGtk *)user_data;

  int code = 0;

  switch( event->keyval )
  {
    case GDK_Escape:
      code = 27;
      break;
    case GDK_Return:
    case GDK_Linefeed:
      code = '\n';
      break;
    case GDK_Tab:
      code = '\t';
      break;
    default:
      code = event->keyval;
      break;
  }

  pWin->m_uLastKey = (code & 0xffff) | (event->state << 16);

  if( pWin->m_funcKeyCb )
  {
    pWin->m_funcKeyCb(code, event->state, pWin->m_dataKeyCb);
  }

  return FALSE;
}

gboolean WinGtk::onMouseCvImage(GtkWidget   *w,
                                GdkEventKey *event,
                                gpointer    *user_data)
{
  WinGtk *pWin = (WinGtk *)user_data;

  CvPoint2D32f  pt32f = {-1., -1.};
  CvPoint       pt = {-1,-1};
  int           cv_event = -1;
  int           state = 0;
  uint_t        i;

  if( event->type == GDK_MOTION_NOTIFY )
  {
    GdkEventMotion* event_motion = (GdkEventMotion*)event;

    cv_event = CV_EVENT_MOUSEMOVE;
    pt32f.x = cvRound(event_motion->x);
    pt32f.y = cvRound(event_motion->y);
    state = event_motion->state;
  }

  else if( event->type == GDK_BUTTON_PRESS ||
           event->type == GDK_BUTTON_RELEASE ||
           event->type == GDK_2BUTTON_PRESS )
  {
    GdkEventButton* event_button = (GdkEventButton*)event;

    pt32f.x = cvRound(event_button->x);
    pt32f.y = cvRound(event_button->y);

    if( event_button->type == GDK_BUTTON_PRESS )
    {
      cv_event = event_button->button == 1 ? CV_EVENT_LBUTTONDOWN :
                 event_button->button == 2 ? CV_EVENT_MBUTTONDOWN :
                 event_button->button == 3 ? CV_EVENT_RBUTTONDOWN : 0;
    }
    else if( event_button->type == GDK_BUTTON_RELEASE )
    {
      cv_event = event_button->button == 1 ? CV_EVENT_LBUTTONUP :
                 event_button->button == 2 ? CV_EVENT_MBUTTONUP :
                 event_button->button == 3 ? CV_EVENT_RBUTTONUP : 0;
    }
    else if( event_button->type == GDK_2BUTTON_PRESS )
    {
      cv_event = event_button->button == 1 ? CV_EVENT_LBUTTONDBLCLK :
                 event_button->button == 2 ? CV_EVENT_MBUTTONDBLCLK :
                 event_button->button == 3 ? CV_EVENT_RBUTTONDBLCLK : 0;
    }

    state = event_button->state;
  }

  if( cv_event >= 0 )
  {
    pt = cvPointFrom32f(pt32f);

    //dbgpoint("mouse", pt);
    
    int flags = (state & GDK_SHIFT_MASK ? CV_EVENT_FLAG_SHIFTKEY : 0) |
        (state & GDK_CONTROL_MASK ? CV_EVENT_FLAG_CTRLKEY : 0) |
        (state & (GDK_MOD1_MASK|GDK_MOD2_MASK) ? CV_EVENT_FLAG_ALTKEY : 0) |
        (state & GDK_BUTTON1_MASK ? CV_EVENT_FLAG_LBUTTON : 0) |
        (state & GDK_BUTTON2_MASK ? CV_EVENT_FLAG_MBUTTON : 0) |
        (state & GDK_BUTTON3_MASK ? CV_EVENT_FLAG_RBUTTON : 0);

    for(i=0; i<MaxCvImages; ++i)
    {
      if( (pWin->m_wCvImage[i] == NULL) || (pWin->m_funcMouseCb[i] == NULL) )
      {
        continue;
      }

      //pt.x -= pWin->m_ptCvImageOrig[i].x;
      //pt.y -= pWin->m_ptCvImageOrig[i].y;
      
      if( (pt.x >= 0) && (pt.x < pWin->m_sizeCvImage[i].width) ||
          (pt.y >= 0) && (pt.y < pWin->m_sizeCvImage[i].height) )
      {
        pWin->m_bMouseEvent = true;
        pWin->m_funcMouseCb[i](cv_event, pt.x, pt.y, flags,
                                pWin->m_dataMouseCb[i]);
        break;
      }
    }
  }

  return FALSE;
}

void WinGtk::onButtonClick(GtkWidget *w, gpointer *user_data)
{
  WinGtk::BttnInfo_T *pInfo = (WinGtk::BttnInfo_T *)user_data;

  if( pInfo->m_pWin->m_funcBttnCb != NULL )
  {
    pInfo->m_pWin->m_bMouseEvent = true;
    pInfo->m_pWin->m_funcBttnCb(pInfo->m_nBttnId, pInfo->m_pWin->m_dataBttnCb);
  }
}

void WinGtk::onRealizeGstWin(GtkWidget *w, gpointer user_data)
{
  WinGtk *pWin = (WinGtk *)user_data;

#if GTK_CHECK_VERSION(2,18,0)
  //
  // I copied this code. Kept it for meta-pedagogical reasons.
  //
  // This is here just for pedagogical purposes, GDK_WINDOW_XID will call
  // it as well in newer Gtk versions.
  //
  if( !gdk_window_ensure_native(w->window) )
  {
    LOGERROR("Couldn't create native window needed for GstXOverlay!");
  }
#endif

#ifdef GDK_WINDOWING_X11
  pWin->m_uGstWinXid = GDK_WINDOW_XID(gtk_widget_get_window(pWin->m_wGstWin));
#endif
}


//.............................................................................
// Simple Functions
//.............................................................................

#if 0 // TODO

/*!
 * Create GTK window with minumum decorations.
 *
 * \param sWinName    Window name (and id).
 * \param width       Window width in pixels.
 * \param height      Window height in pixels.
 */
void rnr::WinCreate(const char *sWinName, int width, int height)
{
  // use openCV to create and size window
  cvNamedWindow(sWinName, 0);
  cvResizeWindow(sWinName, width, height);

  // now go to the native gtk space for fine tuning and operatiion

  GtkWidget *widgetCv  = (GtkWidget *)cvGetWindowHandle(sWinName);
  GtkWindow *windowCv  = GTK_WINDOW(gtk_widget_get_toplevel(widgetCv));
 
  // kill title bar, resize controls, exit control, menus, etc.
  gtk_window_set_decorated(windowCv, false);
}
#endif // TODO
