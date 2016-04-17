////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMMasterControl.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-18 10:01:05 -0600 (Mon, 18 Jun 2012) $
 * $Rev: 2056 $
 *
 * \brief Master control panel and functions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows 
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"
#include "rnr/rnrWinMenu.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

static const char *SMStateName[] =
{
  "Start StaleMate",  "Uncalibrated",   "No Game",
  "Playing Chess",    "Stepping",       "Paused",
  "End StaleMate"
};

static void SMMasterCtlSetMenuStates(StaleMateSession &session)
{
  MenuItemState   eMenuStateNewGame;  // menu item state
  MenuItemState   eMenuStateStep;     // menu item state
  MenuItemState   eMenuStateResume;   // menu item state
  MenuItemState   eMenuStateEndGame;  // menu item state

  switch( session.m_eCurState )
  {
    case StaleMateStateStart:
    case StaleMateStateUnCalib:
      eMenuStateNewGame   = MenuItemStateDisabled;
      eMenuStateStep      = MenuItemStateDisabled;
      eMenuStateResume    = MenuItemStateDisabled;
      eMenuStateEndGame   = MenuItemStateDisabled;
      break;
    case StaleMateStateNoGame:
      eMenuStateNewGame   = MenuItemStateNormal;
      eMenuStateStep      = MenuItemStateNormal;
      eMenuStateResume    = MenuItemStateDisabled;
      eMenuStateEndGame   = MenuItemStateDisabled;
      break;
    case StaleMateStateStepping:
      eMenuStateNewGame   = MenuItemStateDisabled;
      eMenuStateStep      = MenuItemStateDisabled;
      eMenuStateResume    = MenuItemStateDisabled;
      eMenuStateEndGame   = MenuItemStateNormal;
      break;
    case StaleMateStatePaused:
      eMenuStateNewGame   = MenuItemStateDisabled;
      eMenuStateStep      = MenuItemStateNormal;
      eMenuStateResume    = MenuItemStateNormal;
      eMenuStateEndGame   = MenuItemStateNormal;
      break;
    case StaleMateStatePlayingChess:
      eMenuStateNewGame   = MenuItemStateDisabled;
      eMenuStateStep      = MenuItemStateNormal;
      eMenuStateResume    = MenuItemStateDisabled;
      eMenuStateEndGame   = MenuItemStateNormal;
      break;
    case StaleMateStateEnd:
    default:
      eMenuStateNewGame   = MenuItemStateDisabled;
      eMenuStateStep      = MenuItemStateDisabled;
      eMenuStateResume    = MenuItemStateDisabled;
      eMenuStateEndGame   = MenuItemStateDisabled;
      break;
  }
 
  session.m_gui.pMenu->SetMenuItemStates("StaleMate",
            StaleMateActionNewGame,   eMenuStateNewGame,
            StaleMateActionStep,      eMenuStateStep,
            StaleMateActionResume,    eMenuStateResume,
            StaleMateActionEndGame,   eMenuStateEndGame,
            UIActionNone);
}

void SMMasterCtlSetState(StaleMateSession &session, StaleMateState eNewState)
{
  if( session.GetCurState() != eNewState )
  {
    session.SetCurState(eNewState);
    SMMasterCtlSetMenuStates(session);
    LOGDIAG1("StaleMate State: %s(%d).", SMStateName[eNewState], eNewState);
  }
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Execute Actions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Master Control Initialization Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

static int SMMasterCtlInitCamera(StaleMateSession &session)
{
  return HEK_OK;
}

static int SMMasterCtlInitVideo(StaleMateSession &session)
{
  IplImage *pImgFrame;  // pointer to captured video frame
  int       rc;         // return code

  // start video capture
  if( (rc = StaleMateLiveFeedCaptureStart(session)) != HEK_OK )
  {
    LOGERROR("StaleMateLiveFeedCaptureStart() failed");
    return -HEK_ECODE_VIDEO;
  } 

  // grab a frame into internal buffer
  if( (cvGrabFrame(session.m_vid.pVidCapture) > 0) && 
      ((pImgFrame=cvRetrieveFrame(session.m_vid.pVidCapture)) != NULL) )
  {
    session.m_vid.sizeVideo.width  = pImgFrame->width;
    session.m_vid.sizeVideo.height = pImgFrame->height;
    //don't release pImgFrame;
  }
  else 
  {
    LOGDIAG2("Cannot determine captured image size, using defaults"); 
    session.m_vid.sizeVideo.width  = 640;
    session.m_vid.sizeVideo.height = 480;
  }

  LOGDIAG2("Captured image size = ( %d, %d )", 
                session.m_vid.sizeVideo.width, session.m_vid.sizeVideo.height);

  return HEK_OK;
}

static void SMMasterCtlInitMenu(StaleMateSession &session)
{
  session.m_gui.pMenu = new Menu(StaleMateMaxButtonsPerMenu*2, HekIconDir);

  // Left Side Menus
  session.m_gui.pMenu->AddItem(StaleMateActionNewGame,
                          MenuItemStateNormal,
                          AlignLeft,
                          "new\ngame",
                          "icon_new_game.png");
  session.m_gui.pMenu->AddItem(StaleMateActionStep,
                          MenuItemStateNormal,
                          AlignLeft,
                          "step",
                          "icon_step.png");
  session.m_gui.pMenu->AddItem(StaleMateActionResume,
                          MenuItemStateNormal,
                          AlignLeft,
                          "resume",
                          "icon_resume.png");
  session.m_gui.pMenu->AddItem(StaleMateActionEndGame,
                          MenuItemStateNormal,
                          AlignLeft,
                          "end\ngame",
                          "icon_end.png");
  session.m_gui.pMenu->AddItem(StaleMateActionTestRgb,
                          MenuItemStateNormal,
                          AlignLeft,
                          "rgb",
                          "icon_rgb.png");
  session.m_gui.pMenu->AddItem(StaleMateActionTestHsv,
                          MenuItemStateNormal,
                          AlignLeft,
                          "hsv",
                          "icon_hsv.png");
  session.m_gui.pMenu->AddItem(StaleMateActionTestMotion,
                          MenuItemStateNormal,
                          AlignLeft,
                          "test\nmove",
                          "icon_test.png");
  session.m_gui.pMenu->AddItem(StaleMateActionTestUci,
                          MenuItemStateNormal,
                          AlignLeft,
                          "test\nuci",
                          "icon_uci.png");

  // Right Side Menus
  session.m_gui.pMenu->AddItem(StaleMateActionTune,
                          MenuItemStateNormal,
                          AlignRight,
                          "tuning",
                          "icon_tune.png");
  session.m_gui.pMenu->AddItem(StaleMateActionCalib,
                          MenuItemStateNormal,
                          AlignRight,
                          "calib",
                          "icon_calib.png");
  session.m_gui.pMenu->AddItem(StaleMateActionReadCfg,
                          MenuItemStateNormal,
                          AlignRight,
                          "read\ncfg",
                          "icon_cfg.png");
  session.m_gui.pMenu->AddItem(StaleMateActionSetPos,
                          MenuItemStateNormal,
                          AlignRight,
                          "set\npos",
                          "icon_setpos.png");
  session.m_gui.pMenu->AddItem(StaleMateActionSetSafe,
                          MenuItemStateNormal,
                          AlignRight,
                          "Is\nSafe",
                          "icon_safe.png");
  session.m_gui.pMenu->AddItem(StaleMateActionScan,
                          MenuItemStateNormal,
                          AlignRight,
                          "servo\nscan",
                          "icon_scan.png");
  session.m_gui.pMenu->AddItem(StaleMateActionFreeze,
                          MenuItemStateNormal,
                          AlignRight,
                          "freeze\npos",
                          "icon_freeze.png");
  session.m_gui.pMenu->AddItem(StaleMateActionPark,
                          MenuItemStateNormal,
                          AlignRight,
                          "park\npos",
                          "icon_park.png");
  session.m_gui.pMenu->AddItem(StaleMateActionHome,
                          MenuItemStateNormal,
                          AlignRight,
                          "home\npos",
                          "icon_home.png");
  session.m_gui.pMenu->AddItem(StaleMateActionEStop,
                          MenuItemStateNormal,
                          AlignRight,
                          "estop",
                          "icon_estop.png");
  session.m_gui.pMenu->AddItem(StaleMateActionQuit,
                          MenuItemStateNormal,
                          AlignRight,
                          "exit",
                          "icon_exit.png");

  // bind menu to window
  session.m_gui.pMenu->Bind(session.m_gui.pWin);
}

static void SMMasterCtlInitGui(StaleMateSession &session)
{
  GtkWidget *wVBox;       // workspace vertical box container
  CvSize     sizeWs;      // size of workspace
  CvSize     sizeTgt;     // size of target video images frames
  GtkWidget *wContainer;  // this workflow container
  GtkWidget *wValign;     // (vertical) alignment widget
  GtkWidget *wTbl;        // table widget
  GtkWidget *wFrame;      // working frame
  GtkWidget *wBoard;      // working chessboard widget
  GtkWidget *wBox1;       // working box widget 1
  GtkWidget *wBox2;       // working box widget 2
  GtkWidget *w;           // working widget
  GtkWidget *wLabel;      // working label widget
  GdkColor   color;       // working color
  GtkAttachOptions  optTbl = (GtkAttachOptions)(GTK_EXPAND | GTK_FILL);
                          // table attach options
                          
  // set the window workspace is gtk widget tree 
  session.m_gui.pWin->WorkspaceSetAsGtk();


  //---
  // Get window's workspace widget and size
  //---
  wVBox   = session.m_gui.pWin->WorkspaceGetVBox();
  sizeWs  = session.m_gui.pWin->WorkspaceGetSize();


  //---
  // Add a horizontal box to the workspace
  //---
  wContainer = gtk_hbox_new(FALSE, 20);
  gtk_container_add(GTK_CONTAINER(wVBox), wContainer);

  // left margin widget
  w = gtk_vbox_new(TRUE, 10);
  gtk_box_pack_start(GTK_BOX(wContainer), w, TRUE, TRUE, 0);

  // main table
  wValign = gtk_alignment_new(0, 0, 0, 0);
  gtk_box_pack_start(GTK_BOX(wContainer), wValign, TRUE, TRUE, 0);
  wTbl = gtk_table_new(4, 2, FALSE);  // 4x2 table
  gtk_container_add(GTK_CONTAINER(wValign), wTbl);

  // right margin widget
  w = gtk_vbox_new(TRUE, 10);
  gtk_box_pack_start(GTK_BOX(wContainer), w, TRUE, TRUE, 0);
  

  //---
  // Title label
  //---
  wLabel = gtk_label_new("Hekateros StaleMate");
  gdk_color_parse(GuiTitleStrColor, &color);
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_font(wLabel,
      pango_font_description_from_string(GuiTitleStrFont));

  // force size of label so table space full workspace
  g_object_set(G_OBJECT(wLabel),
                "width-request", sizeWs.width,
                 NULL);
  
  // attach title to table row 0, columns 0-1, with 5px vertical spacing
  gtk_table_attach(GTK_TABLE(wTbl), wLabel, 0, 2, 0, 1, optTbl, optTbl, 0, 5);
  gtk_widget_show(wLabel);


  // --
  // Add live annotated display
  // --
  wFrame = gtk_aspect_frame_new(NULL, 0.0, 0.0, 0, TRUE);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &color);
  w = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), w);
  gtk_table_attach(GTK_TABLE(wTbl), wFrame, 0, 1, 1, 2, optTbl, optTbl, 0, 5);

  // add CvImage to container box
  session.m_vid.uImgIndex0 = session.m_gui.pWin->WorkspacePackAtStartCvImage(w);


  // --
  // Add image processed display
  // --
  wFrame = gtk_aspect_frame_new(NULL, 0.0, 0.0, 0, TRUE);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &color);
  w = gtk_hbox_new(FALSE, 5);
  gtk_container_add(GTK_CONTAINER(wFrame), w);
  gtk_table_attach(GTK_TABLE(wTbl), wFrame, 1, 2, 1, 2, optTbl, optTbl, 0, 5);

  // add CvImage to container box
  session.m_vid.uImgIndex1 = session.m_gui.pWin->WorkspacePackAtStartCvImage(w);


  if( (sizeWs.width*2 < session.m_vid.sizeVideo.width) ||
      (sizeWs.height*2 < session.m_vid.sizeVideo.height) )
  {
    sizeTgt.width = sizeWs.width / 2;
    sizeTgt.height = sizeWs.height / 2;
  }
  else
  {
    sizeTgt = session.m_vid.sizeVideo;
  }

  // set image transformations
  session.m_vid.pVidToGuiTrans = new IoI(session.m_vid.sizeVideo,
                                         sizeTgt,
                                         StaleMateImgTransRot,
                                         StaleMateImgTransAlign,
                                         StaleMateImgTransCrop);

  // images displayed to user
  session.m_vid.pImgDisplay0 = cvCreateImage(sizeTgt, IPL_DEPTH_8U, 3);
  cvZero(session.m_vid.pImgDisplay0);
  session.m_vid.pImgDisplay1 = cvCreateImage(sizeTgt, IPL_DEPTH_8U, 3);
  cvZero(session.m_vid.pImgDisplay1);

  // --
  // Iconic Chess Board and Game History
  // --
  wBox1 = gtk_hbox_new(FALSE, 40);
  gtk_table_attach(GTK_TABLE(wTbl), wBox1, 0, 1, 2, 3,
      (GtkAttachOptions)(GTK_SHRINK), (GtkAttachOptions)(GTK_SHRINK),
      0, 5);

  wBox2 = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start_defaults(GTK_BOX(wBox1), wBox2);

  wLabel = gtk_label_new("Black");
  gtk_box_pack_start_defaults(GTK_BOX(wBox2), wLabel);
  gdk_color_parse(GuiLabel2StrColor, &color);
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_font(wLabel,
      pango_font_description_from_string(GuiLabel2StrFont));
  session.m_gui.wBlackLabel = wLabel;

  wBoard = gtk_table_new(8, 8, FALSE);  // 8x8 table
  gtk_box_pack_start_defaults(GTK_BOX(wBox2), wBoard);
  gtk_table_set_row_spacings(GTK_TABLE(wBoard), 0);
  gtk_table_set_col_spacings(GTK_TABLE(wBoard), 0);
  gtk_widget_set_size_request(wBoard, 256, 256);
  session.m_gui.wIconicChessBoard = wBoard;

  wLabel = gtk_label_new("White");
  gtk_box_pack_start_defaults(GTK_BOX(wBox2), wLabel);
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_font(wLabel,
      pango_font_description_from_string(GuiLabel2StrFont));
  session.m_gui.wWhiteLabel = wLabel;


  // --
  // Move History
  // --
  GtkWidget     *wView;
  GtkTextBuffer *buffer;
  GtkWidget     *wScrollWin;

  wBox2 = gtk_vbox_new(FALSE, 0);
  gtk_box_pack_start_defaults(GTK_BOX(wBox1), wBox2);

  wLabel = gtk_label_new("Move History");
  gtk_box_pack_start_defaults(GTK_BOX(wBox2), wLabel);
  gdk_color_parse(GuiLabel2StrColor, &color);
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_font(wLabel,
      pango_font_description_from_string(GuiLabel2StrFont));

  wFrame = gtk_aspect_frame_new(NULL, 0.0, 0.0, 0, TRUE);
  gtk_box_pack_start_defaults(GTK_BOX(wBox2), wFrame);
  gdk_color_parse(GuiStrColorRed, &color);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &color);

  wScrollWin = gtk_scrolled_window_new(NULL, NULL);
  gtk_container_add(GTK_CONTAINER(wFrame), wScrollWin);

  wView = gtk_text_view_new();
  gtk_container_add(GTK_CONTAINER(wScrollWin), wView);
  gdk_color_parse(GuiStrColorBlack, &color);
  gtk_widget_modify_base(wView, GTK_STATE_NORMAL, &color);
  gtk_widget_set_size_request(wView, 256, 240);
  gtk_text_view_set_editable(GTK_TEXT_VIEW(wView), FALSE);

  session.m_gui.wViewHistory = wView;
  session.m_gui.wBufHistory  = gtk_text_view_get_buffer(GTK_TEXT_VIEW (wView));

  // Change default font throughout the widget
  gtk_widget_modify_font(wView,
      pango_font_description_from_string(GuiLabel3StrFont));

  // Change default color throughout the widget
  gdk_color_parse(GuiLabel3StrColor, &color);
  gtk_widget_modify_text(wView, GTK_STATE_NORMAL, &color);


  // --
  // Hekateros Servo state
  // --
  w = StaleMateHekGuiInit(session);
  gtk_table_attach(GTK_TABLE(wTbl), w, 1, 2, 2, 3,
      (GtkAttachOptions)(GTK_SHRINK), (GtkAttachOptions)(GTK_SHRINK),
      0, 5);
  session.m_gui.wChainState = w;

  // --
  // Show
  // --
  session.m_gui.pWin->ShowCvImage(session.m_vid.pImgDisplay0,
                                  session.m_vid.uImgIndex0);
  session.m_gui.pWin->ShowCvImage(session.m_vid.pImgDisplay1,
                                  session.m_vid.uImgIndex1);

  gtk_widget_show(wContainer);
  gtk_widget_show_all(wVBox);
}

static int SMMasterCtlInit(StaleMateSession &session)
{
  int   rc;

  // initialize the camera
  if( (rc = SMMasterCtlInitCamera(session)) != HEK_OK )
  {
    LOGERROR("SMMasterCtlInitCamera()");
    return rc;
  }

  // initialize the video capture
  if( (rc = SMMasterCtlInitVideo(session)) != HEK_OK )
  {
    LOGERROR("SMMasterCtlInitVideo()");
    return rc;
  }

  // initialize the menus
  SMMasterCtlInitMenu(session);

  // initialize the gui
  SMMasterCtlInitGui(session);

  // initialize chess board
  StaleMateChessOneTimeInit(session);

  StaleMateHekInit(session);

  return HEK_OK;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

int StaleMateMasterControl(StaleMateSession &session)
{
  CvPoint2D32f  ptDEBottom;
  int           nAction;      // user action
  int           rc;           // return code

  int nRgbChannel = CHANNEL_RED;
  int nHsvChannel = CHANNEL_HUE;
  int nRow        = 0;
  int nCol        = 0;

  if( (rc = SMMasterCtlInit(session)) != HEK_OK )
  {
    LOGERROR("SMMasterCtlInit()");
    return rc;
  }

  StaleMateReadCfg(session);

  // set current state
  if( session.m_calib.bCalibrated )
  {
    SMMasterCtlSetState(session, StaleMateStateNoGame);
  }
  else
  {
    SMMasterCtlSetState(session, StaleMateStateUnCalib);
  }

  // set current action 
  nAction = StaleMateActionIdle;
  session.m_gui.pMenu->SetCurrentAction(nAction);

  // start video live feed thread
  StaleMateLiveFeedThreadStart(&session);

  // already
  session.m_gui.pWin->ShowStatus("Let's Rumble!");

  rc = HEK_OK;

  //
  // Run 
  //
  while( (session.GetCurState() != StaleMateStateEnd) && (rc == HEK_OK) )
  {
    StaleMateLiveFeedShow(session);
    
    StaleMateHekGuiShowChainState(session);

    // user action
    nAction = session.m_gui.pMenu->GetCurrentAction();

    switch( nAction )
    {
      case StaleMateActionQuit:         // quit application
        session.m_gui.pWin->ShowStatus("Quiting StaleMate");
        SMMasterCtlSetState(session, StaleMateStateEnd);
        StaleMateLiveFeedThreadStopJoin();
        break;

      case StaleMateActionIdle:       // unprocessed video, no arm movements
        break;

      case StaleMateActionTune:
        session.m_gui.pWin->ShowStatus("Tuning StaleMate");
        StaleMateGuiDlgTune(session);
        session.m_calib.bCalibrated = false;
        //SMMasterCtlSetState(session, StaleMateStateUnCalib);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        break;

      case StaleMateActionCalib:
        session.m_gui.pWin->ShowStatus("Calibrating the game.");
        StaleMateCalib(session);
        if( session.m_calib.bCalibrated )
        {
          SMMasterCtlSetState(session, StaleMateStateNoGame);
        }
        else
        {
          SMMasterCtlSetState(session, StaleMateStateUnCalib);
        }
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        break;

      case StaleMateActionReadCfg:
        session.m_gui.pWin->ShowStatus("Reading configuration.");
        StaleMateReadCfg(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        break;

      case StaleMateActionSetPos:
        session.m_gui.pWin->ShowStatus("Setting key Hekateros positions.");
        StaleMateGuiDlgSetPos(session);
        if( session.m_calib.bCalibrated )
        {
          SMMasterCtlSetState(session, StaleMateStateNoGame);
        }
        else
        {
          SMMasterCtlSetState(session, StaleMateStateUnCalib);
        }
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        break;

      case StaleMateActionNewGame:
      case StaleMateActionStep:
        // start a new game
        if( session.GetCurState() == StaleMateStateNoGame )
        {
          StaleMateGuiDlgNewGame(session);
          session.m_gui.pWin->ShowStatus("Starting a new game, Hekateros is %s",
            (session.m_game.bHekHasWhite? "white": "black"));
          StaleMateGameNew(session);
          session.m_game.eGameState = ChessGameStateStart;
          if( session.m_game.bHekHasWhite )
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionMakeMove);
          }
          else
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionWaitForMove);
          }
          if( nAction == StaleMateActionNewGame )
          {
            SMMasterCtlSetState(session, StaleMateStatePlayingChess);
          }
          else
          {
            SMMasterCtlSetState(session, StaleMateStateStepping);
          }
        }

        // paused
        else if( session.GetCurState() == StaleMateStatePaused )
        {
          // was hekateros move
          if( session.m_game.eGameState == ChessGameStateHeksMove )
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionWaitForMove);
          }
          /// was opponents move
          else if( session.m_game.eGameState == ChessGameStateOppsMove )
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionMakeMove);
          }
          SMMasterCtlSetState(session, StaleMateStateStepping);
        }

        break;

      case StaleMateActionResume:
        if( session.GetCurState() == StaleMateStatePaused )
        {
          SMMasterCtlSetState(session, StaleMateStatePlayingChess);

          // was hekateros move
          if( session.m_game.eGameState == ChessGameStateHeksMove )
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionWaitForMove);
          }
          /// was opponents move
          else if( session.m_game.eGameState == ChessGameStateOppsMove )
          {
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionMakeMove);
          }
        }
        break;

      case StaleMateActionEndGame:
        session.m_gui.pWin->ShowStatus("Game ended.");
        if( session.m_calib.bCalibrated )
        {
          SMMasterCtlSetState(session, StaleMateStateNoGame);
        }
        else
        {
          SMMasterCtlSetState(session, StaleMateStateUnCalib);
        }
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        break;

      case StaleMateActionMakeMove:
        switch( session.m_game.eGameState )
        {
          case ChessGameStateStart:
          case ChessGameStateOppsMove:
            session.m_game.eGameState = ChessGameStateHeksMove;
            session.m_gui.pWin->ShowStatus("Hekateros' move.");
            StaleMateGameHekaterosStart(session);
            break;
          case ChessGameStateHeksMove:
            session.m_game.eGameState = ChessGameStateHeksMove;
            if( StaleMateGameHekaterosMove(session) )
            {
              StaleMateGameUpdate(session);
              if( session.GetCurState() == StaleMateStateStepping )
              {
                session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
                SMMasterCtlSetState(session, StaleMateStatePaused);
              }
              else
              {
                session.m_gui.pMenu->SetCurrentAction(
                                                    StaleMateActionWaitForMove);
              }
            }
            break;
          case ChessGameStateCheckMate:
            StaleMateChessHistAdd(session, "Check Mate! Hekateros loses.");
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
          case ChessGameStateResign:
            StaleMateChessHistAdd(session, "Hekateros Resigns.");
            break;
          case ChessGameStateDraw:
            StaleMateChessHistAdd(session, "Game is a draw");
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
          default:
            LOGERROR("Bug: GameState=%d.", session.m_game.eGameState);
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
        }
        break;

      case StaleMateActionWaitForMove:
        switch( session.m_game.eGameState )
        {
          case ChessGameStateStart:
          case ChessGameStateHeksMove:
            session.m_game.eGameState = ChessGameStateOppsMove;
            StaleMateGameOpponentsStart(session);
            session.m_gui.pWin->ShowStatus("Opponent's move.");
            break;
          case ChessGameStateOppsMove:
            session.m_game.eGameState = ChessGameStateOppsMove;
            if( StaleMateGameOpponentsMove(session) )
            {
              if( session.GetCurState() == StaleMateStateStepping )
              {
                session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
                SMMasterCtlSetState(session, StaleMateStatePaused);
              }
              else
              {
                session.m_gui.pMenu->SetCurrentAction(StaleMateActionMakeMove);
              }
            }
            break;
          case ChessGameStateCheckMate:
            StaleMateChessHistAdd(session, "Check Mate! Hekateros wins.");
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
          case ChessGameStateResign:
            StaleMateChessHistAdd(session, "Opponent Resigns! Hekateros wins.");
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
          case ChessGameStateDraw:
            StaleMateChessHistAdd(session, "Game is a draw");
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
          default:
            LOGERROR("Bug: GameState=%d.", session.m_game.eGameState);
            session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
            break;
        }
        break;

      case StaleMateActionHome:
        session.m_gui.pWin->ShowStatus(
                              "Go to Home position. Any game play is paused.");
        StaleMateHekGotoHome(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        if( session.GetPrevState() == StaleMateStatePlayingChess )
        {
          SMMasterCtlSetState(session, StaleMateStatePaused);
        }
        break;

      case StaleMateActionFreeze:
        session.m_gui.pWin->ShowStatus(
            "The Hekateros is frozen at the current location. "
            "Any game play is paused.");
        StaleMateHekFreezeAll(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        if( session.GetPrevState() == StaleMateStatePlayingChess )
        {
          SMMasterCtlSetState(session, StaleMateStatePaused);
        }
        break;

      case StaleMateActionScan:
        session.m_gui.pWin->ShowStatus("Scanning for Hekateros servos...");
        session.m_gui.pWin->Wait(10);
        StaleMateHekScan(session);
        if( StaleMateHekIsSafe(session, true) )
        {
          StaleMateHekWriteSafeGoalSpeeds(session, 75);
        }
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        if( session.GetPrevState() == StaleMateStatePlayingChess )
        {
          SMMasterCtlSetState(session, StaleMateStatePaused);
        }
        break;

      case StaleMateActionPark:
        session.m_gui.pWin->ShowStatus("Park the Hekateros in a safe position. "
            "Any game play is terminated.");
        StaleMateHekParkIt(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        if( session.m_calib.bCalibrated )
        {
          SMMasterCtlSetState(session, StaleMateStateNoGame);
        }
        else
        {
          SMMasterCtlSetState(session, StaleMateStateUnCalib);
        }
        break;

      case StaleMateActionEStop:
        session.m_gui.pWin->ShowStatus("Hekateros Emergency Stop");
        StaleMateHekEStop(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionIdle);
        if( session.m_calib.bCalibrated )
        {
          SMMasterCtlSetState(session, StaleMateStateNoGame);
        }
        else
        {
          SMMasterCtlSetState(session, StaleMateStateUnCalib);
        }
        break;

      case StaleMateActionSetSafe:
        session.m_hek.bIsSafe = true;
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionNoOp);
        break;

      case StaleMateActionTestRgb:
        cerr << "Dbg: Test RGB Channel " << 
          (nRgbChannel == CHANNEL_BLUE? "blue":
              nRgbChannel == CHANNEL_GREEN? "green": "red")
          << "(" << nRgbChannel << ")" << endl;
        StaleMateTestRgbChannel(session, nRgbChannel);
        nRgbChannel = (nRgbChannel + 1) % 3;
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionNoOp);
        break;

      case StaleMateActionTestHsv:
        cerr << "Dbg: Test Hsv Channel " << 
          (nHsvChannel == CHANNEL_HUE? "hue":
              nHsvChannel == CHANNEL_SATURATION? "saturation": "value")
          << "(" << nHsvChannel << ")" << endl;
        StaleMateTestHsvChannel(session, nHsvChannel);
        nHsvChannel = (nHsvChannel + 1) % 3;
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionNoOp);
        break;

      case StaleMateActionTestMotion:
        StaleMateTestMove(session);
        session.m_gui.pMenu->SetCurrentAction(StaleMateActionNoOp);
        break;

      case StaleMateActionTestUci:
        if( !StaleMateTestUci(session) )
        {
          session.m_gui.pMenu->SetCurrentAction(StaleMateActionNoOp);
        }
        break;

      case UIActionNone:
      case StaleMateActionNoOp:
      default:
        break;
    }

    // wait for events
    session.m_gui.pWin->Wait(15);
  }

  //
  // clean up
  //
  StaleMateLiveFeedCaptureStop(session);

  return rc;
}
