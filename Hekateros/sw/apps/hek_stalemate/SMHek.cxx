////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMHek.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-18 10:01:05 -0600 (Mon, 18 Jun 2012) $
 * $Rev: 2056 $
 *
 * \brief StaleMate Hekateros routines.
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

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/EX.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

typedef struct
{
  int   m_nServoId;
  int   m_nPosMin;
  int   m_nPosMax;
} HekLimits_T;

// 0.9.1 arm safe limits
HekLimits_T HekPosLimits_0_9_1[SM_HEK_NSERVOS_TOTAL] =
{
  {HEK_SERVO_ID_BASE,         DYNA_POS_MIN_RAW+10,
                              DYNA_POS_MAX_RAW-10},
  {HEK_SERVO_ID_SHOULDER_L,   DYNA_EX106P_POS_MIN_RAW+10,
                              DYNA_EX106P_POS_MAX_RAW-10},
  {HEK_SERVO_ID_SHOULDER_R,   DYNA_EX106P_POS_MIN_RAW+10,
                              DYNA_EX106P_POS_MAX_RAW-10},
  {HEK_SERVO_ID_ELBOW,        DYNA_POS_MIN_RAW+2,
                              DYNA_POS_MAX_RAW-2},
  {HEK_SERVO_ID_WRIST_ROT,    DYNA_POS_MIN_RAW+10,
                              DYNA_POS_MAX_RAW-10},
  {HEK_SERVO_ID_WRIST_PITCH,  DYNA_POS_MIN_RAW+10,
                              DYNA_POS_MAX_RAW-10},
  {HEK_SERVO_ID_GRABOID,      DYNA_POS_MIN_RAW+10,
                              DYNA_POS_MAX_RAW-10}
};

static DynaPosTuple_T  HekTupBlank[SM_HEK_NSERVOS_TOTAL_M] =
{
  {HEK_SERVO_ID_BASE,         0},
  {HEK_SERVO_ID_SHOULDER_L,   0},
  {HEK_SERVO_ID_ELBOW,        0},
  {HEK_SERVO_ID_WRIST_ROT,    0},
  {HEK_SERVO_ID_WRIST_PITCH,  0},
  {HEK_SERVO_ID_GRABOID,      0}
};

DynaPosTuple_T  HekKeyPosPark[SM_HEK_NSERVOS_TOTAL_M] =
{
  {HEK_SERVO_ID_BASE,         715},
  {HEK_SERVO_ID_SHOULDER_L,   967},
  {HEK_SERVO_ID_ELBOW,        55},
  {HEK_SERVO_ID_WRIST_ROT,    848},
  {HEK_SERVO_ID_WRIST_PITCH,  248},
  {HEK_SERVO_ID_GRABOID,      440}
};

DynaPosTuple_T  HekKeyPosHome[SM_HEK_NSERVOS_TOTAL_M] =
{
  {HEK_SERVO_ID_BASE,         690},
  {HEK_SERVO_ID_SHOULDER_L,   807},
  {HEK_SERVO_ID_ELBOW,        229},
  {HEK_SERVO_ID_WRIST_ROT,    848},
  {HEK_SERVO_ID_WRIST_PITCH,  248},
  {HEK_SERVO_ID_GRABOID,      449}
};

DynaPosTuple_T  HekKeyPosMotPlanner[SM_HEK_NSERVOS_BASE_M] =
{
  {HEK_SERVO_ID_BASE,         690},
  {HEK_SERVO_ID_SHOULDER_L,   807},
  {HEK_SERVO_ID_ELBOW,        229},
  {HEK_SERVO_ID_WRIST_ROT,    848},
  {HEK_SERVO_ID_WRIST_PITCH,  248}
};

DynaPosTuple_T  HekKeyPosGripperOpen   = {HEK_SERVO_ID_GRABOID, 800};
DynaPosTuple_T  HekKeyPosGripperNarrow = {HEK_SERVO_ID_GRABOID, 810};
DynaPosTuple_T  HekKeyPosGripperGrab   = {HEK_SERVO_ID_GRABOID, 876};
DynaPosTuple_T  HekKeyPosGripperClose  = {HEK_SERVO_ID_GRABOID, 650};

typedef map<int,int>  GuiServoIdIndex;

static GuiServoIdIndex  GuiServoIdToWidgetIdx;
static GtkWidget       *GuiServoLabel[HEK_NSERVOS_TOT];
static GtkWidget       *GuiServoOdPos[HEK_NSERVOS_TOT];
static int              ValServoOdPos[HEK_NSERVOS_TOT];
static GtkWidget       *GuiServoOdDeg[HEK_NSERVOS_TOT];
static float            ValServoOdDeg[HEK_NSERVOS_TOT];
static GtkWidget       *GuiServoSpeed[HEK_NSERVOS_TOT];
static int              ValServoSpeed[HEK_NSERVOS_TOT];
static GtkWidget       *GuiServoLoad[HEK_NSERVOS_TOT];
static int              ValServoLoad[HEK_NSERVOS_TOT];
static GtkWidget       *GuiServoAlarms[HEK_NSERVOS_TOT];
static uint_t           ValServoAlarms[HEK_NSERVOS_TOT];


static void HekOrpSetLinks(StaleMateSession  &session,
                           DynaPosTuple_T     tupRaw[],
                           uint_t             uNumTups)
{
  DynaRealTuple_T  tupDeg[uNumTups];

  session.m_hek.motionPlanner.rawToDeg(tupRaw, tupDeg, (int)uNumTups); 
  StaleMateOrpDegToRadAndSend(session, tupDeg, (size_t)uNumTups);
}

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

GtkWidget *StaleMateHekGuiInit(StaleMateSession &session)
{
  GtkWidget  *wBox;       // working vertical box
  GtkWidget  *wFrame;     // frame widget
  GtkWidget  *wAlign;     // alignment widget
  GtkWidget  *wTbl;       // table widget
  GtkWidget  *wLabel;     // working label widget
  GdkColor    colorFrame; // working color
  GdkColor    colorText1; // working color
  GdkColor    colorText3; // working color
  PangoFontDescription  *fontText1;
  PangoFontDescription  *fontText3;
  GtkAttachOptions  optTbl = (GtkAttachOptions)(GTK_EXPAND | GTK_FILL);
  int         nServoId;
  DynaServo  *pServo;
  int         iter;
  int         row, col;
  int         i;

  gdk_color_parse(GuiStrColorRed, &colorFrame);
  gdk_color_parse(GuiStrColorLabelFg, &colorText1);
  gdk_color_parse(GuiStrColorLabel2Fg, &colorText3);

  fontText1 = pango_font_description_from_string(GuiLabel2StrFont);
  fontText3 = pango_font_description_from_string(GuiLabel3StrFont);

  wBox = gtk_vbox_new(FALSE, 0);

  wLabel = gtk_label_new("Hekateros State");
  gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText1);
  gtk_widget_modify_font(wLabel, fontText1);
  g_object_set(G_OBJECT(wLabel),
      "width-request", 640,
      NULL);
  gtk_box_pack_start_defaults(GTK_BOX(wBox), wLabel);

  wFrame = gtk_aspect_frame_new(NULL, 0.0, 0.0, 0, TRUE);
  gtk_widget_modify_bg(wFrame, GTK_STATE_NORMAL, &colorFrame);
  gtk_box_pack_start_defaults(GTK_BOX(wBox), wFrame);

  // top-center aligned servo table
  wAlign = gtk_alignment_new(0.5, 0, 0, 0);
  gtk_container_add(GTK_CONTAINER(wFrame), wAlign);

  wTbl = gtk_table_new(6, HEK_NSERVOS_TOT+1, FALSE);
  g_object_set(G_OBJECT(wTbl),
        "width-request",  640,
        "height-request", 256,
        NULL);
  gtk_container_add(GTK_CONTAINER(wAlign), wTbl);

  if( !session.m_hek.bUseArm  )
  {
    wLabel = gtk_label_new("No Arm");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText1);
    gtk_widget_modify_font(wLabel, fontText1);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                3, 4, 2, 3, optTbl, optTbl, 10, 5);
  }

  else
  {
    row = 0;
    col = 1;

    // servo labels
    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new(" ");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoLabel[i] = wLabel;
      gtk_widget_ref(GuiServoLabel[i]); // protect widget
    }

    row = 1;
    col = 0;

    // state labels
    wLabel = gtk_label_new("Odometer");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
    gtk_widget_modify_font(wLabel, fontText3);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                col, col+1, row, row+1, optTbl, optTbl, 10, 5);
    ++row;

    wLabel = gtk_label_new(" Degrees");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
    gtk_widget_modify_font(wLabel, fontText3);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                col, col+1, row, row+1, optTbl, optTbl, 10, 5);
    ++row;

    wLabel = gtk_label_new("   Speed");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
    gtk_widget_modify_font(wLabel, fontText3);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                col, col+1, row, row+1, optTbl, optTbl, 10, 5);
    ++row;

    wLabel = gtk_label_new("    Load");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
    gtk_widget_modify_font(wLabel, fontText3);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                col, col+1, row, row+1, optTbl, optTbl, 10, 5);
    ++row;

    wLabel = gtk_label_new("  Alarms");
    gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
    gtk_widget_modify_font(wLabel, fontText3);
    gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                                col, col+1, row, row+1, optTbl, optTbl, 10, 5);

    row = 1;
    col = 1;

    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new("0");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoOdPos[i] = wLabel;
      gtk_widget_ref(GuiServoOdPos[i]); // protect widget
      ValServoOdPos[i] = 0;
    }

    ++row;

    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new("0.0");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoOdDeg[i] = wLabel;
      gtk_widget_ref(GuiServoOdDeg[i]); // protect widget
      ValServoOdDeg[i] = 0.0;
    }

    ++row;

    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new("0");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoSpeed[i] = wLabel;
      gtk_widget_ref(GuiServoSpeed[i]); // protect widget
      ValServoSpeed[i] = 0;
    }

    ++row;

    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new("0");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoLoad[i] = wLabel;
      gtk_widget_ref(GuiServoLoad[i]); // protect widget
      ValServoLoad[i] = 0;
    }

    ++row;

    for(i=0; i<HEK_NSERVOS_TOT; ++i)
    {
      wLabel = gtk_label_new("OK");
      gtk_widget_modify_fg(wLabel, GTK_STATE_NORMAL, &colorText3);
      gtk_widget_modify_font(wLabel, fontText3);
      gtk_table_attach(GTK_TABLE(wTbl), wLabel, 
                            col+i, col+1+i, row, row+1, optTbl, optTbl, 10, 5);
      GuiServoAlarms[i] = wLabel;
      gtk_widget_ref(GuiServoAlarms[i]); // protect widget
      ValServoAlarms[i] = 0;
    }
  }

  gtk_widget_show_all(wFrame);

  return wBox;
}

void StaleMateHekGuiInitState(StaleMateSession &session)
{
  int         nServoId;
  int         iter;
  DynaServo  *pServo;
  int         i;
  char        buf[32];

  if( !session.m_hek.bUseArm  )
  {
    return;
  }

  for(nServoId=session.m_hek.pDynaChain->IterStart(&iter), i=0;
      (nServoId != DYNA_ID_NONE) && (i < HEK_NSERVOS_TOT);
      nServoId=session.m_hek.pDynaChain->IterNext(&iter), ++i)
  {
    pServo = session.m_hek.pDynaChain->GetServo(nServoId);

    GuiServoIdToWidgetIdx[nServoId] = i;

    sprintf(buf, "%d:%s", nServoId, pServo->GetModelName());
    gtk_label_set_text(GTK_LABEL(GuiServoLabel[i]), buf);

    StaleMateHekGuiShowServoState(session, nServoId);
  }
}

void StaleMateHekGuiShowChainState(StaleMateSession &session)
{
  GuiServoIdIndex::iterator iter;
  int                       nServoId;

  if( !session.m_hek.bUseArm  )
  {
    return;
  }

  for(iter  = GuiServoIdToWidgetIdx.begin();
      iter != GuiServoIdToWidgetIdx.end();
      ++iter)
  {
    nServoId = iter->first;

    if( nServoId != DYNA_ID_NONE )
    {
      StaleMateHekGuiShowServoState(session, nServoId);
    }
  }
}

void StaleMateHekGuiShowServoState(StaleMateSession &session, int nServoId)
{
  DynaServo                  *pServo;
  GuiServoIdIndex::iterator   iter;
  int                         i;
  char                        buf[64];
  int                         nOdPos;
  float                       fOdDeg;
  int                         nSpeed;
  int                         nLoad;
  uint_t                      uAlarms;
  int                         nChanges;

  if( !session.m_hek.bUseArm  )
  {
    return;
  }

  else if( (pServo = session.m_hek.pDynaChain->GetServo(nServoId)) == NULL )
  {
    return;
  }

  else if( GuiServoIdToWidgetIdx.find(nServoId) == GuiServoIdToWidgetIdx.end() )
  {
    return;
  }

  iter = GuiServoIdToWidgetIdx.find(nServoId);

  if( iter == GuiServoIdToWidgetIdx.end() )
  {
    return;
  }

  i = iter->second;

  nOdPos    = pServo->GetOdometer();
  fOdDeg    = 0.0; // RDK TODO
  nSpeed    = (pServo->GetCurSpeed() * 1000) / DYNA_SPEED_MAX_RAW;
  nLoad     = (pServo->GetCurLoad() * 1000) / DYNA_CUR_LOAD_MAG_MAX;
  uAlarms   = pServo->GetAlarms();
  nChanges  = 0;
  
  if( nOdPos != ValServoOdPos[i] )
  {
    sprintf(buf, "%d", nOdPos);
    gtk_label_set_text(GTK_LABEL(GuiServoOdPos[i]), buf);
    ValServoOdPos[i] = nOdPos;
    ++nChanges;
  }

  if( nOdPos != ValServoOdDeg[i] )
  {
    sprintf(buf, "%.1f", fOdDeg*180.0/PI);
    gtk_label_set_text(GTK_LABEL(GuiServoOdDeg[i]), buf);
    ValServoOdDeg[i] = fOdDeg;
    ++nChanges;
  }

  if( nSpeed != ValServoSpeed[i] )
  {
    sprintf(buf, "%d", nSpeed);
    gtk_label_set_text(GTK_LABEL(GuiServoSpeed[i]), buf);
    ValServoSpeed[i] = nSpeed;
    ++nChanges;
  }

  if( nLoad != ValServoLoad[i] )
  {
    sprintf(buf, "%d", nLoad);
    gtk_label_set_text(GTK_LABEL(GuiServoLoad[i]), buf);
    ValServoLoad[i] = nLoad;
    ++nChanges;
  }

  if( uAlarms != ValServoAlarms[i] )
  {
    DynaComm::GetAlarmsShortString(uAlarms, buf, sizeof(buf));
    gtk_label_set_text(GTK_LABEL(GuiServoAlarms[i]), buf);
    ValServoAlarms[i] = uAlarms;
    ++nChanges;
  }

  if( nChanges > 0 )
  {
    gtk_widget_show_all(session.m_gui.wChainState);
  }
}

void StaleMateHekGotoHome(StaleMateSession &session)
{
  int   rc;

  if( !session.m_hek.bHomeDefined )
  {
    session.m_gui.pWin->ShowStatus("Home position not defined.");
    return; 
  }

  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  session.m_hek.bAtHomePos = false;

  StaleMateHekSyncMove(session, HekKeyPosHome, arraysize(HekKeyPosHome));
  
  session.m_hek.bAtHomePos  = true;
  session.m_hek.bIsParked   = false;
}

void StaleMateHekParkIt(StaleMateSession &session)
{
  int                   rc;

  if( !session.m_hek.bParkDefined )
  {
    session.m_gui.pWin->ShowStatus("Park position not defined.");
    return; 
  }

  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  session.m_hek.bIsParked  = false;
  session.m_hek.bAtHomePos = false;

  StaleMateHekSyncMove(session, HekKeyPosPark, arraysize(HekKeyPosPark));

  session.m_hek.bIsParked = true;

  //while( StaleMateHekIsMoving(session) );

  //session.m_hek.pDynaChain->Release();
}

/*!
 * \brief Move Hekateros to the given (x,y,z) location.
 *
 * All locations are in real, phyisical coordinates.
 *
 * \param session   StaleMate session data.
 * \param ptGaol    Goal location.
 */
void StaleMateHekMoveTo(StaleMateSession &session, CvPoint3D32f &ptGoal)
{
  DynaPosTuple_T tupGoal[SM_HEK_NSERVOS_BASE_M] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0}
  };
  DynaPosTuple_T tupCurPos[SM_HEK_NSERVOS_BASE] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_SHOULDER_R,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0}
  };
  int   iL = 1, iR = 2;
  int   diffL, diffR;
  int   i;


  LOGDIAG2("Goal position: %f %f %f", ptGoal.x, ptGoal.y, ptGoal.z);

  // plan move
  session.m_hek.motionPlanner.ConstrainedPlanner(ptGoal, tupGoal,
                                              SM_HEK_NSERVOS_BASE_M);

  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  // get current position for some sanity checks
  StaleMateHekReadCurPos(session, tupCurPos, SM_HEK_NSERVOS_BASE);

  // validate
  for(i=0; i<SM_HEK_NSERVOS_BASE_M; ++i)
  {
    if( (tupGoal[i].m_nPos < session.m_hek.motionPlanner.m_servosMin[i] ) 
        || (tupGoal[i].m_nPos > session.m_hek.motionPlanner.m_servosMax[i] ) )
    {
      LOGERROR("Motion Planner: Servo %d: %d is out of safe range.",
          tupGoal[i].m_nServoId, tupGoal[i].m_nPos);
      return;
    }
  }

  // move
  StaleMateHekSyncMove(session, tupGoal, SM_HEK_NSERVOS_BASE_M);

  session.m_hek.bAtHomePos  = false;
  session.m_hek.bIsParked   = false;
}

/*!
 * \brief Move Hekateros to the given chessboard location at the given height.
 *
 * The virtual square coordinate is converted to the physical square row and
 * column.
 *
 * \param session   StaleMate session data.
 * \param sAlgSq    Virtual chess square location in algebraic notation.
 * \param fZ        Height above the board (mm).
 */
void StaleMateHekMoveTo(StaleMateSession &session,
                        const char       *sAlgSq,
                        double            fZ)
{
  ChessSquare_T sq;
  CvPoint3D32f  ptGoal;

  if( !session.m_hek.bUseArm ||
      !session.m_calib.bCalibrated ||
      !StaleMateHekIsSafe(session) )
  {
    return;
  }

  sq = VAlgSqToPRowCol(session, sAlgSq);

  if( (sq.m_nRow < 0) || (sq.m_nRow >= session.m_game.nChessBoardDim) ||
      (sq.m_nCol < 0) || (sq.m_nCol >= session.m_game.nChessBoardDim) )
  {
    LOGERROR("'%s', Unknown algabraic square.", sAlgSq);
    return;
  }

  ptGoal.x = session.m_calib.ptHekDist[sq.m_nRow][sq.m_nCol].x;
  ptGoal.y = session.m_calib.ptHekDist[sq.m_nRow][sq.m_nCol].y;
  ptGoal.z = fZ;

  StaleMateHekMoveTo(session, ptGoal);
}

void StaleMateHekGripperOpen(StaleMateSession &session)
{
  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  session.m_hek.pDynaChain->MoveTo(HEK_SERVO_ID_GRABOID,
                                  HekKeyPosGripperOpen.m_nPos); 

  //HekOrpSetLinks(session, &HekKeyPosGripperOpen, 1);
}

void StaleMateHekGripperNarrow(StaleMateSession &session)
{
  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  session.m_hek.pDynaChain->MoveTo(HEK_SERVO_ID_GRABOID,
                      HekKeyPosGripperNarrow.m_nPos); 

  //HekOrpSetLinks(session, &HekKeyPosGripperNarrow, 1);
}

void StaleMateHekGripperGrab(StaleMateSession &session)
{
  if( !session.m_hek.bUseArm || !StaleMateHekIsSafe(session) )
  {
    return;
  }

  session.m_hek.pDynaChain->MoveTo(HEK_SERVO_ID_GRABOID,
                      HekKeyPosGripperGrab.m_nPos); 

  //HekOrpSetLinks(session, &HekKeyPosGripperGrab, 1);
}

bool StaleMateHekIsStopped(StaleMateSession &session)
{
  int         nServoId;
  DynaServo  *pServo;
  int         iter;
  bool        bIsMoving;
  int         rc;

  if( !session.m_hek.bUseArm )
  {
    return true;
  }

  for(nServoId=session.m_hek.pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId=session.m_hek.pDynaChain->IterNext(&iter))
  {
    pServo = session.m_hek.pDynaChain->GetServo(nServoId);
    rc = pServo->ReadIsMoving(&bIsMoving);
    if( (rc == DYNA_OK) && bIsMoving )
    {
      return false;
    }
  }

  return true;
}

void StaleMateHekDisableOdometry(StaleMateSession &session)
{
  int         nServoId;
  int         iter;
  DynaServo  *pServo;

  if( !session.m_hek.bUseArm  )
  {
    return;
  }

  for(nServoId=session.m_hek.pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId=session.m_hek.pDynaChain->IterNext(&iter))
  {
    pServo = session.m_hek.pDynaChain->GetServo(nServoId);
    pServo->DisableOdometer();
  }
}

void StaleMateHekSyncMove(StaleMateSession &session,
                          DynaPosTuple_T    tup[],
                          uint_t            uNumTups)
{
  if( session.m_hek.bUseArm && StaleMateHekIsSafe(session) )
  {
    if( session.m_hek.pDynaChain->SyncMoveTo(tup, uNumTups) == DYNA_OK )
    {
      session.m_hek.bAtHomePos = false;
      session.m_hek.bIsParked  = false;

      //HekOrpSetLinks(session, tup, uNumTups);
    }
  }
}

void StaleMateHekWriteSafeGoalSpeeds(StaleMateSession &session,
                                     int               nGoalSpeed)
{
  int   i;

  DynaSpeedTuple_T  tup[SM_HEK_NSERVOS_TOTAL_M] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0},
    {HEK_SERVO_ID_GRABOID,      0}
  };

  for(i=0; i<SM_HEK_NSERVOS_TOTAL_M; ++i)
  {
    tup[i].m_nSpeed = nGoalSpeed;
  }

  if( session.m_hek.bUseArm && StaleMateHekIsSafe(session) )
  {
    session.m_hek.pDynaChain->SyncWriteGoalSpeed(tup, SM_HEK_NSERVOS_TOTAL_M);
  }
}

// read first nTups servo current odometer postions.
void StaleMateHekReadCurPos(StaleMateSession &session,
                            DynaPosTuple_T    tupCurPos[],
                            int               nTups)
{
  DynaServo  *pServo;
  int         i;

  if( !session.m_hek.bUseArm || (session.m_hek.pDynaChain == NULL) )
  {
    return;
  }

  for(i=0; i<nTups; ++i)
  {
    pServo = session.m_hek.pDynaChain->GetServo(tupCurPos[i].m_nServoId);
    
    if( pServo != NULL )
    {
      if( pServo->ReadCurPos(&tupCurPos[i].m_nPos) == DYNA_OK )
      {
        //HekOrpSetLinks(session, tupCurPos, nTups);
      }
    }
  }
}

// get first nTups servo current odometer positions.
void StaleMateHekGetCurPos(StaleMateSession &session,
                           DynaPosTuple_T    tupCurPos[],
                           int               nTups)
{
  DynaServo  *pServo;
  int         i;

  if( !session.m_hek.bUseArm || (session.m_hek.pDynaChain == NULL) )
  {
    return;
  }

  for(i=0; i<nTups; ++i)
  {
    pServo = session.m_hek.pDynaChain->GetServo(tupCurPos[i].m_nServoId);
    
    if( pServo != NULL )
    {
      tupCurPos[i].m_nPos = pServo->GetOdometer();
    }
  }
}

void StaleMateHekEStop(StaleMateSession &session)
{
  if( session.m_hek.bUseArm && (session.m_hek.pDynaChain != NULL) )
  {
    session.m_hek.pDynaChain->EStop();
  }
}

void StaleMateHekFreezeAll(StaleMateSession &session)
{
  if( session.m_hek.bUseArm && (session.m_hek.pDynaChain != NULL) )
  {
    session.m_hek.pDynaChain->Freeze();
  }
}

void StaleMateHekReleaseAll(StaleMateSession &session)
{
  if( session.m_hek.bUseArm || (session.m_hek.pDynaChain != NULL) )
  {
    session.m_hek.pDynaChain->Release();
  }
}

int StaleMateHekInit(StaleMateSession &session)
{
  int   nServoId;
  int   n;

  if( !session.m_hek.bUseArm )
  {
    return HEK_OK;
  }

  session.m_hek.pDynaComm = DynaComm::New(session.m_hek.sHekDevName,
                                          session.m_hek.nHekBaudRate);

  if( session.m_hek.pDynaComm == NULL )
  {
    session.m_gui.pWin->ShowStatus(
        "Failed to create dynamixel interface on '%s'@%d\n",
        session.m_hek.sHekDevName, session.m_hek.nHekBaudRate);
    session.m_hek.bUseArm = false;
    return -HEK_ECODE_DYNA;
  }

  session.m_hek.pDynaChain = new DynaChain(*session.m_hek.pDynaComm);

  session.m_hek.pDynaBgThread = new DynaBgThread();

  sleep(1); // RDK fix in dyna

  StaleMateHekScan(session);

  LOGDIAG1("%d servos added to dynamixel interface.", 
      session.m_hek.pDynaChain->GetNumberInChain());

  return HEK_OK;
}

void StaleMateHekScan(StaleMateSession &session)
{
  int         nServoId;
  DynaServo  *pServo;
  int         iter;
  int         n;
  char        buf[256];

  if( !session.m_hek.bUseArm || (session.m_hek.pDynaChain == NULL) )
  {
    session.m_gui.pWin->ShowStatus("No arm enabled.");
    return;
  }

  if( session.m_hek.pDynaBgThread->GetCurrentState() ==
                                            DynaBgThread::BgThreadStateRunning )
  {
    session.m_hek.pDynaBgThread->Stop();
  }

  session.m_hek.pDynaBgThread->UnregisterAgent();

  n = session.m_hek.pDynaChain->AddNewServosByScan();

  session.m_gui.pWin->ShowStatus("%d servos added to dynamixel interface.", n);

  if( session.m_hek.pDynaChain->HasServo(HEK_SERVO_ID_SHOULDER_L) &&
      session.m_hek.pDynaChain->HasServo(HEK_SERVO_ID_SHOULDER_R) )
  {
    if( !session.m_hek.pDynaChain->IsLinkedMaster(HEK_SERVO_ID_SHOULDER_L) )
    {
      session.m_hek.pDynaChain->LinkServos(HEK_SERVO_ID_SHOULDER_L,
                                         HEK_SERVO_ID_SHOULDER_R,
                                         true);
    }
  }
  else if( session.m_hek.pDynaChain->HasServo(HEK_SERVO_ID_SHOULDER_L) )
  {
    session.m_gui.pWin->ShowStatus(
                          "Warning: Right shoulder servo not detected: "
                          "Cannot operate safely!");
  }
  else if( session.m_hek.pDynaChain->HasServo(HEK_SERVO_ID_SHOULDER_R) )
  {
    session.m_gui.pWin->ShowStatus(
                          "Warning: Left shoulder servo not detected: "
                          "Cannot operate safely!");
  }

  StaleMateHekDisableOdometry(session);

  if( StaleMateHekIsSafe(session, true) )
  {
    StaleMateHekWriteSafeGoalSpeeds(session, 75);
  }

  session.m_hek.pDynaBgThread->RegisterChainAgent(session.m_hek.pDynaChain);

  session.m_hek.pDynaBgThread->Run();

  StaleMateHekGuiInitState(session);
}

bool StaleMateHekIsSafe(StaleMateSession &session, bool bReadPos)
{
  DynaPosTuple_T  tup[SM_HEK_NSERVOS_TOTAL] =
  {
    {HEK_SERVO_ID_BASE,         0},
    {HEK_SERVO_ID_SHOULDER_L,   0},
    {HEK_SERVO_ID_SHOULDER_R,   0},
    {HEK_SERVO_ID_ELBOW,        0},
    {HEK_SERVO_ID_WRIST_ROT,    0},
    {HEK_SERVO_ID_WRIST_PITCH,  0},
    {HEK_SERVO_ID_GRABOID,      0}
  };
  int   nServos;
  int   i, j;

  if( !session.m_hek.bUseArm )
  {
    return true;
  }
  else if( !session.m_hek.bIsSafe )
  {
    session.m_gui.pWin->ShowStatus("The \"Is Safe\" has not been pushed.");
    return false;
  }
  else if( session.m_hek.pDynaChain == NULL )
  {
    session.m_gui.pWin->ShowStatus("No chain.");
    return false;
  }
  else if( (nServos = session.m_hek.pDynaChain->GetNumberInChain())
                                                    != SM_HEK_NSERVOS_TOTAL )
  {
    session.m_gui.pWin->ShowStatus("No servos.");
    return false;
  }

  if( bReadPos )
  {
    StaleMateHekReadCurPos(session, tup, SM_HEK_NSERVOS_TOTAL);
  }
  else
  {
    StaleMateHekGetCurPos(session, tup, SM_HEK_NSERVOS_TOTAL);
  }

  for(i=0; i<SM_HEK_NSERVOS_TOTAL; ++i)
  {
    for(j=0; j<SM_HEK_NSERVOS_TOTAL; ++j)
    {
      if( tup[i].m_nServoId == HekPosLimits_0_9_1[j].m_nServoId )
      {
        if( (tup[i].m_nPos < HekPosLimits_0_9_1[j].m_nPosMin) ||
            (tup[i].m_nPos > HekPosLimits_0_9_1[j].m_nPosMax) )
        {
          session.m_gui.pWin->ShowStatus(
              "Servo %d: Current position %d out of safe limits.",
              tup[i].m_nServoId, tup[i].m_nPos);
          return false;
        }
      }
    }
  }

  return true;
}
