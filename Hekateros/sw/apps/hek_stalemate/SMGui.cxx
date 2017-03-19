////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMGui.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief StaleMate Graphical User Interface routines.
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

#include <sys/types.h>
#include <stdarg.h>
#include <libgen.h>
#include <ctype.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// strip leading and trailing white space
static void strip(const char *src, char *dst)
{
  size_t      n = 0;
  const char *t;
  int         c;

  // no string
  if( src == NULL )
  {
    *dst = 0;
    return;
  }

  // skip leading space
  while( (c = *(src+n)) != 0 )
  {
    if( !isspace(c) )
    {
      break;
    }
    n++;
  }
  
  // all white space
  if( c == 0 )
  {
    *dst = 0;
    return;
  }

  t = src + n;

  // strip trailing white space
  for(n=strlen(src); n>0; --n)
  {
    c = src[n-1];
    if( !isspace(c) )
    {
      break;
    }
  }

  // copy
  strncpy(dst, t, n);
  *(dst + n) = 0;
}


// ---------------------------------------------------------------------------
// Public and Callback Interface 
// ---------------------------------------------------------------------------

void StaleMateGuiDlgNewGame(StaleMateSession &session)
{
  GtkWidget *wDlg;
  GtkWidget *wContentArea;
  GtkWidget *wFrame;
  GtkWidget *wBox;
  GtkWidget *wRadioWhite;
  GtkWidget *wRadioBlack;
  GdkColor   colorFrame;

  // create the widgets
  wDlg = gtk_dialog_new_with_buttons("New Chess Game",
              session.m_gui.pWin->GetMainWindowWidget(),
              (GtkDialogFlags)(GTK_DIALOG_MODAL|GTK_DIALOG_DESTROY_WITH_PARENT),
              GTK_STOCK_OK, GTK_RESPONSE_OK,
              NULL);

  wContentArea = gtk_dialog_get_content_area(GTK_DIALOG(wDlg));
   
  // colors and fonts
  gdk_color_parse(GuiStrColorRed, &colorFrame);

  //
  // add content
  //
  wFrame = gtk_aspect_frame_new(" Hekateros Color ", 0.0, 0.0, 0, TRUE);
  gtk_container_add(GTK_CONTAINER(wContentArea), wFrame);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);
  wBox = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  // radio buttons
  wRadioWhite = gtk_radio_button_new_with_label(NULL, "White");
  wRadioBlack = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioWhite),
                                      "Black");

  if( session.m_game.bHekHasWhite )
  {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(wRadioWhite), TRUE);
  }
  else
  {
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(wRadioBlack), TRUE);
  }

  // call back
  //g_signal_connect(wRadioWhite, "toggled",
  //                  G_CALLBACK(GuiDlgNewGameCb), &session);

  // pack 
  gtk_box_pack_start(GTK_BOX(wBox), wRadioWhite, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioBlack, TRUE, TRUE, 2);


  // show 
  gtk_widget_show_all(wDlg);

  // block run till  close
  switch( gtk_dialog_run(GTK_DIALOG(wDlg)) )
  {
    case GTK_RESPONSE_OK:
      if( gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(wRadioWhite)) )
      {
        session.m_game.bHekHasWhite = true;
      }
      else
      {
        session.m_game.bHekHasWhite = false;
      }
      break;
    default:
      break;
  }
  
  // destroy the dialog
  gtk_widget_destroy(wDlg);
}

void StaleMateGuiDlgTune(StaleMateSession &session)
{
  GtkWidget  *wDlg;
  GtkWidget  *wContentArea;
  GtkWidget  *wFrame;
  GtkWidget  *wVBox;
  GtkWidget  *wBox;
  GtkWidget  *wLabel;

  GtkWidget  *wVFC;
  GtkWidget  *wSquareSize;
  GtkWidget  *wBoardLocDEx;
  GtkWidget  *wBoardLocDEy;
  GtkWidget  *wBoardDiffRed;
  GtkWidget  *wBoardDiffBlue;
  GtkWidget  *wSqDiffRed;
  GtkWidget  *wSqDiffBlue;

  GdkColor    colorFrame;
  char        buf[32];
  const char *s;

  // create the widgets
  wDlg = gtk_dialog_new_with_buttons("StaleMate Tuning",
              session.m_gui.pWin->GetMainWindowWidget(),
              (GtkDialogFlags)(GTK_DIALOG_MODAL|GTK_DIALOG_DESTROY_WITH_PARENT),
              GTK_STOCK_OK, GTK_RESPONSE_OK,
              NULL);

  wContentArea = gtk_dialog_get_content_area(GTK_DIALOG(wDlg));
   
  // colors and fonts
  gdk_color_parse(GuiStrColorRed, &colorFrame);

  //
  // add content
  //
  wVBox = gtk_vbox_new(TRUE, 5);
  gtk_container_add(GTK_CONTAINER(wContentArea), wVBox);

  // ---
  // board location and size
  // ---
  wFrame = gtk_aspect_frame_new(" Board Location and Size",
      0.0, 0.0, 0, TRUE);
  gtk_box_pack_start(GTK_BOX(wVBox), wFrame, FALSE, FALSE, 0);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);
  wBox = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  wLabel = gtk_label_new(" square size (mm):");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wSquareSize = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wSquareSize, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wSquareSize), 6);
  gtk_widget_set_size_request(wSquareSize, 70, 25);
  sprintf(buf, "%.2lf", TuneChessSquareDim);
  gtk_entry_set_text(GTK_ENTRY(wSquareSize), buf);

  wLabel = gtk_label_new(" square de base distance (mm): x:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wBoardLocDEx = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wBoardLocDEx, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wBoardLocDEx), 6);
  gtk_widget_set_size_request(wBoardLocDEx, 70, 25);
  sprintf(buf, "%.2lf", TuneChessDEBaseX);
  gtk_entry_set_text(GTK_ENTRY(wBoardLocDEx), buf);

  wLabel = gtk_label_new(" y:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wBoardLocDEy = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wBoardLocDEy, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wBoardLocDEy), 6);
  gtk_widget_set_size_request(wBoardLocDEy, 70, 25);
  sprintf(buf, "%.2lf", TuneChessDEBaseY);
  gtk_entry_set_text(GTK_ENTRY(wBoardLocDEy), buf);

  // ---
  // detectiong board differences
  // ---
  wFrame = gtk_aspect_frame_new(" Board Difference Detection ",
      0.0, 0.0, 0, TRUE);
  gtk_box_pack_start(GTK_BOX(wVBox), wFrame, FALSE, FALSE, 0);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);
  wBox = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  // video frame diff count
  wLabel = gtk_label_new("video frame count:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wVFC = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wVFC, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wVFC), 4);
  gtk_widget_set_size_request(wVFC, 50, 25);
  sprintf(buf, "%d", TuneFramesWithChangeCnt);
  gtk_entry_set_text(GTK_ENTRY(wVFC), buf);

  wLabel = gtk_label_new(" red board threshold:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wBoardDiffRed = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wBoardDiffRed, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wBoardDiffRed), 10);
  gtk_widget_set_size_request(wBoardDiffRed, 110, 25);
  sprintf(buf, "%.1f", TuneBoardThresholdRed);
  gtk_entry_set_text(GTK_ENTRY(wBoardDiffRed), buf);

  wLabel = gtk_label_new(" blue board threshold:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wBoardDiffBlue = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wBoardDiffBlue, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wBoardDiffBlue), 10);
  gtk_widget_set_size_request(wBoardDiffBlue, 110, 25);
  sprintf(buf, "%.1lf", TuneBoardThresholdBlue);
  gtk_entry_set_text(GTK_ENTRY(wBoardDiffBlue), buf);


  // ---
  // detectiong square differences
  // ---
  wFrame = gtk_aspect_frame_new(" Square Candidate Detection ",
      0.0, 0.0, 0, TRUE);
  gtk_box_pack_start(GTK_BOX(wVBox), wFrame, FALSE, FALSE, 0);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);
  wBox = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  wLabel = gtk_label_new(" red square threshold:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wSqDiffRed = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wSqDiffRed, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wSqDiffRed), 10);
  gtk_widget_set_size_request(wSqDiffRed, 110, 25);
  sprintf(buf, "%.1lf", TuneSquareThresholdRed);
  gtk_entry_set_text(GTK_ENTRY(wSqDiffRed), buf);

  wLabel = gtk_label_new(" blue square threshold:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wSqDiffBlue = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wSqDiffBlue, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wSqDiffBlue), 10);
  gtk_widget_set_size_request(wSqDiffBlue, 110, 25);
  sprintf(buf, "%.1lf", TuneSquareThresholdBlue);
  gtk_entry_set_text(GTK_ENTRY(wSqDiffBlue), buf);


  // show 
  gtk_widget_show_all(wDlg);


  // block run till  close
  switch( gtk_dialog_run(GTK_DIALOG(wDlg)) )
  {
    case GTK_RESPONSE_OK:
      if( (s = gtk_entry_get_text(GTK_ENTRY(wSquareSize))) != NULL )
      {
        sscanf(s, "%lf", &TuneChessSquareDim);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wBoardLocDEx))) != NULL )
      {
        sscanf(s, "%lf", &TuneChessDEBaseX);
        session.m_calib.ptDEBottom.x = TuneChessDEBaseX;
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wBoardLocDEy))) != NULL )
      {
        sscanf(s, "%lf", &TuneChessDEBaseY);
        session.m_calib.ptDEBottom.y = TuneChessDEBaseY;
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wVFC))) != NULL )
      {
        sscanf(s, "%d", &TuneFramesWithChangeCnt);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wBoardDiffRed))) != NULL )
      {
        sscanf(s, "%lf", &TuneBoardThresholdRed);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wBoardDiffBlue))) != NULL )
      {
        sscanf(s, "%lf", &TuneBoardThresholdBlue);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wSqDiffRed))) != NULL )
      {
        sscanf(s, "%lf", &TuneSquareThresholdRed);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wSqDiffBlue))) != NULL )
      {
        sscanf(s, "%lf", &TuneSquareThresholdBlue);
      }
      break;
    default:
      break;
  }
  
  // destroy the dialog
  gtk_widget_destroy(wDlg);
}

void StaleMateGuiDlgSetPos(StaleMateSession &session)
{
  enum SetPosDlgRspId
  {
    SetPosDlgRspIdFreeze  = 1,
    SetPosDlgRspIdRelease,
    SetPosDlgRspIdRecord
  };

  GtkWidget *wDlg;
  GtkWidget *wContentArea;
  GtkWidget *wFrame;
  GtkWidget *wVBox;
  GtkWidget *wBox;
  GtkWidget *wRadioPosPark;
  GtkWidget *wRadioPosHome;
  GtkWidget *wRadioPosMotP;
  GtkWidget *wRadioPosGripOpen;
  GtkWidget *wRadioPosGripNarrow;
  GtkWidget *wRadioPosGripGrab;
  GdkColor   colorFrame;
  int        iRspId;
  int        nServoId;
  DynaServo *pServo;
  int        n;
  int        iter;

  // create the widgets
  wDlg = gtk_dialog_new_with_buttons("Set Key Hekateros Positions",
              session.m_gui.pWin->GetMainWindowWidget(),
              (GtkDialogFlags)(GTK_DIALOG_MODAL|GTK_DIALOG_DESTROY_WITH_PARENT),
              NULL);

  gtk_dialog_add_buttons(GTK_DIALOG(wDlg),
                      "Freeze",           SetPosDlgRspIdFreeze,
                      "Release",          SetPosDlgRspIdRelease,
                      "Record",           SetPosDlgRspIdRecord,
                      GTK_STOCK_CLOSE,    GTK_RESPONSE_CLOSE,
                      NULL);

  wContentArea = gtk_dialog_get_content_area(GTK_DIALOG(wDlg));
   
  // colors and fonts
  gdk_color_parse(GuiStrColorRed, &colorFrame);

  //
  // add content
  //
  wVBox = gtk_vbox_new(TRUE, 5);
  gtk_container_add(GTK_CONTAINER(wContentArea), wVBox);


  // ---
  // recording hekateros positions 
  // ---
  wFrame = gtk_aspect_frame_new(" Key Positions ",
                              0.0, 0.0, 0, TRUE);
  gtk_container_add(GTK_CONTAINER(wContentArea), wFrame);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);
  wBox = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  // radio buttons
  wRadioPosPark = gtk_radio_button_new_with_label(NULL, "Park");
  wRadioPosHome = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioPosPark),
                                      "Home");
  wRadioPosMotP = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioPosPark),
                                      "Motion-Planner");
  wRadioPosGripOpen = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioPosPark),
                                      "Gripper-Open");
  wRadioPosGripNarrow = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioPosPark),
                                      "Gripper-Narrow");
  wRadioPosGripGrab = gtk_radio_button_new_with_label_from_widget(
                                      GTK_RADIO_BUTTON (wRadioPosPark),
                                      "Gripper-Grab");

  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(wRadioPosPark), TRUE);

  // pack 
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosPark, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosHome, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosMotP, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosGripOpen, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosGripNarrow, TRUE, TRUE, 2);
  gtk_box_pack_start(GTK_BOX(wBox), wRadioPosGripGrab, TRUE, TRUE, 2);

  // show 
  gtk_widget_show_all(wDlg);

  // block run till close or exit
  while( (iRspId = gtk_dialog_run(GTK_DIALOG(wDlg))) > 0 )
  {
    switch( iRspId )
    {
      case SetPosDlgRspIdFreeze:
        StaleMateHekFreezeAll(session);
        break;
      case SetPosDlgRspIdRelease:
        StaleMateHekReleaseAll(session);
        break;
      case SetPosDlgRspIdRecord:
        if( gtk_toggle_button_get_active( GTK_TOGGLE_BUTTON(wRadioPosPark)) )
        {
          if( StaleMateHekIsSafe(session, true) )
          {
            StaleMateHekGetCurPos(session, HekKeyPosPark, SM_HEK_NSERVOS_TOTAL);
            session.m_hek.bParkDefined;
          }
          else
          {
            session.m_gui.pWin->ShowStatus(
                "Hekateros is not in a safe position.");
          }
        }
        else if( gtk_toggle_button_get_active(
                                      GTK_TOGGLE_BUTTON(wRadioPosHome)) )
        {
          if( StaleMateHekIsSafe(session, true) )
          {
            StaleMateHekGetCurPos(session, HekKeyPosHome, SM_HEK_NSERVOS_TOTAL);
            session.m_hek.bHomeDefined = true;
          }
          else
          {
            session.m_gui.pWin->ShowStatus(
                "Hekateros is not in a safe position.");
          }
        }
#if 0 // not supported here
        else if( gtk_toggle_button_get_active(
                                      GTK_TOGGLE_BUTTON(wRadioPosMotP)) )
        {
          StaleMateHekGetCurPos(session, HekKeyPosMotPlanner,
                                      SM_HEK_NSERVOS_BASE);
          session.m_hek.motionPlanner.setOffsets(HekKeyPosMotPlanner,
                                                SM_HEK_NSERVOS_BASE);           
        }
#endif // not supported here
        else if( gtk_toggle_button_get_active(
                                      GTK_TOGGLE_BUTTON(wRadioPosGripOpen)) )
        {
          StaleMateHekGetCurPos(session, &HekKeyPosGripperOpen, 1);
        }
        else if( gtk_toggle_button_get_active(
                                      GTK_TOGGLE_BUTTON(wRadioPosGripNarrow)) )
        {
          StaleMateHekGetCurPos(session, &HekKeyPosGripperNarrow, 1);
        }
        else if( gtk_toggle_button_get_active(
                                      GTK_TOGGLE_BUTTON(wRadioPosGripGrab)) )
        {
          StaleMateHekGetCurPos(session, &HekKeyPosGripperGrab, 1);
        }
      break;
    default:
      break;
    }
  }
  
  // destroy the dialog
  gtk_widget_destroy(wDlg);
}

void StaleMateGuiDlgTestMove(StaleMateSession &session,
                             char              bufAlgSq[], 
                             double           *pfAlgZ,
                             CvPoint3D32f     *pPtCart)
{
  GtkWidget  *wDlg;
  GtkWidget  *wContentArea;
  GtkWidget  *wLabel;
  GtkWidget  *wFrame;
  GtkWidget  *wVBox;
  GtkWidget  *wBox;
  GtkWidget  *wAlgSq, *wAlgZ;
  GtkWidget  *wX, *wY, *wZ;
  GdkColor    colorFrame;
  char        buf[32];
  const char *s;

  // create the widgets
  wDlg = gtk_dialog_new_with_buttons("Test Movement",
              session.m_gui.pWin->GetMainWindowWidget(),
              (GtkDialogFlags)(GTK_DIALOG_MODAL|GTK_DIALOG_DESTROY_WITH_PARENT),
              GTK_STOCK_OK, GTK_RESPONSE_OK,
              NULL);

  wContentArea = gtk_dialog_get_content_area(GTK_DIALOG(wDlg));
   
  // colors and fonts
  gdk_color_parse(GuiStrColorRed, &colorFrame);

  //
  // add content
  //
  wVBox = gtk_vbox_new(TRUE, 5);
  gtk_container_add(GTK_CONTAINER(wContentArea), wVBox);

  //
  // chess algebraic coordinates
  //
  wFrame = gtk_aspect_frame_new(" Algebraic Goal Position ", 0.0, 0.0, 0, TRUE);
  gtk_container_add(GTK_CONTAINER(wVBox), wFrame);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);

  wBox = gtk_hbox_new(TRUE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);

  wLabel = gtk_label_new("square:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wAlgSq = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wAlgSq, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wAlgSq), 2);
  gtk_widget_set_size_request(wAlgSq, 30, 25);

  if( (bufAlgSq != NULL) && (bufAlgSq[0] != 0) )
  {
    gtk_entry_set_text(GTK_ENTRY(wAlgSq), bufAlgSq);
  }

  wLabel = gtk_label_new("z:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wAlgZ = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wAlgZ, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wAlgZ), 6);
  gtk_widget_set_size_request(wAlgZ, 70, 25);
  sprintf(buf, "%6.1f", *pfAlgZ);
  gtk_entry_set_text(GTK_ENTRY(wAlgZ), buf);


  //
  // x,y,z coordinates
  //
  wFrame = gtk_aspect_frame_new(" Cartesian Goal Position ", 0.0, 0.0, 0, TRUE);
  gtk_container_add(GTK_CONTAINER(wVBox), wFrame);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_container_set_border_width(GTK_CONTAINER(wFrame), 10);

  wBox = gtk_hbox_new(TRUE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), wBox);
  
  wLabel = gtk_label_new("x:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wX = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wX, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wX), 6);
  gtk_widget_set_size_request(wX, 70, 25);
  sprintf(buf, "%6.1f", pPtCart->x);
  gtk_entry_set_text(GTK_ENTRY(wX), buf);

  wLabel = gtk_label_new("y:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wY = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wY, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wY), 6);
  gtk_widget_set_size_request(wY, 70, 25);
  sprintf(buf, "%6.1f", pPtCart->y);
  gtk_entry_set_text(GTK_ENTRY(wY), buf);

  wLabel = gtk_label_new("z:");
  gtk_box_pack_start(GTK_BOX(wBox), wLabel, FALSE, FALSE, 0);

  wZ = gtk_entry_new();
  gtk_box_pack_start(GTK_BOX(wBox), wZ, FALSE, FALSE, 0);
  gtk_entry_set_max_length(GTK_ENTRY(wZ), 6);
  gtk_widget_set_size_request(wZ, 70, 25);
  sprintf(buf, "%6.1f", pPtCart->z);
  gtk_entry_set_text(GTK_ENTRY(wZ), buf);


  // show 
  gtk_widget_show_all(wDlg);

  // block run till close
  switch( gtk_dialog_run(GTK_DIALOG(wDlg)) )
  {
    case GTK_RESPONSE_OK:
      bufAlgSq[0] = 0;
      if( (s = gtk_entry_get_text(GTK_ENTRY(wAlgSq))) != NULL )
      {
        sscanf(s, "%s", bufAlgSq);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wAlgZ))) != NULL )
      {
        sscanf(s, "%lf", pfAlgZ);
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wX))) != NULL )
      {
        sscanf(s, "%f", &(pPtCart->x));
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wY))) != NULL )
      {
        sscanf(s, "%f", &(pPtCart->y));
      }
      if( (s = gtk_entry_get_text(GTK_ENTRY(wZ))) != NULL )
      {
        sscanf(s, "%f", &(pPtCart->z));
      }
      break;
    default:
      break;
  }
  
  // destroy the dialog
  gtk_widget_destroy(wDlg);
}
