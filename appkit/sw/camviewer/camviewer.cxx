////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Program:   camviewer
//
// File:      camviewer.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief Camera viewer.
 *
 * Video source may be a video device, a UDP stream, or a file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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

#include <stdio.h>
#include <unistd.h>
#include <string>

#include "rnr/rnrconfig.h"

#include "rnr/appkit/WinLookFeel.h"
#include "rnr/appkit/WinGtk.h"
#include "rnr/appkit/WinGtkMenu.h"

using namespace rnr;

enum CamViewerEvents
{
  CamViewerEventSnapshot = UIEventNumOf,
  CamViewerEventZoomIn,
  CamViewerEventZoomOut,
  CamViewerEventSettings,
  CamViewerEventExit
};

void initMenu(WinGtkButtonMenu &menu)
{
  menu.addImageButton(CamViewerEventSnapshot, WidgetStateNormal, AlignOpLeft,
                      "video\ncamera",
                      "icon_camera.png", "Take a picture.",
                      "icon_video.png", "Start video.");
  menu.addImageButton(CamViewerEventZoomIn, WidgetStateDisabled, AlignOpLeft,
                      "+zoom",
                      "icon_zoom_in.png", "Zoom in.");
  menu.addImageButton(CamViewerEventZoomOut, WidgetStateDisabled, AlignOpLeft,
                      "-zoom",
                      "icon_zoom_out.png", "Zoom out.");
  menu.addImageButton(CamViewerEventSettings, WidgetStateNormal, AlignOpRight,
                      "config",
                      "icon_settings.png", "Configure settings.");
  menu.addImageButton(CamViewerEventExit, WidgetStateNormal, AlignOpRight,
                      "exit",
                      "icon_exit.png", "Exit.");
}

int main(int argc, char *argv[])
{
  GtkWidget  *wWorkspace;
  GtkWidget  *wLabel;

  WinGtk            win("camviewer", 640, 480, true);
  WinGtkButtonMenu  menu("/prj/pkg/appkit/share/images/icons");

  initMenu(menu);

  win.initWorkspace();

  menu.bind(&win);

  win.showStatus("Press any key to speed through or simply wait.");
  win.waitKey(1000);

  win.showPageRef("vid");
  win.waitKey(1000);

  win.showStatus("Green window background.");
  win.setLookAndFeel("color_win_bg", "#003300");
  win.waitKey(1000);

  win.showStatus("Default window background.");
  win.setLookAndFeel("color_win_bg", GuiStrColorWinBg);
  win.waitKey(1000);

  win.showStatus("Status bar red foreground.");
  win.setLookAndFeel("color_status_fg", GuiStrColorRNRed);
  win.waitKey(1000);

  win.showStatus("Yellow status bar border.");
  win.setLookAndFeel("color_status_border", GuiStrColorRNYellow);
  win.waitKey(1000);

  wWorkspace = win.getWorkspaceVBox();

  wLabel = gtk_label_new(win.getGuiToolkitName());
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL,
      win.getLookAndFeelGdkColor("color_rn_white"));
  gtk_widget_modify_font(wLabel, win.getLookAndFeelFont("font_large"));
  gtk_box_pack_start(GTK_BOX(wWorkspace), wLabel, FALSE, FALSE, 4);
  win.showWorkspace();
  win.showStatus("Title.");
  win.waitKey(1000);

  for(int i=5; i>0; --i)
  {
    win.showStatus("Self destruct in %ds", i);
    win.wait(1000);
  }

  return 0;
}
