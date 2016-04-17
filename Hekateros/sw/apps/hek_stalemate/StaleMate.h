////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      StaleMate.h
//
/*! \file
 *
 * $LastChangedDate: 2012-06-18 10:01:05 -0600 (Mon, 18 Jun 2012) $
 * $Rev: 2056 $
 *
 * \brief Hekateros chess playing declarations.
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

#ifndef _STALEMATE_H
#define _STALEMATE_H

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <ctype.h>

#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"
#include "rnr/rnrWinOpenCv.h"
#include "rnr/rnrWinMenu.h"
#include "rnr/rnrWinIoI.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "Hekateros/Hekateros.h"

#include "StaleMateTune.h"
#include "SMBotPlanner.h"

using namespace rnrWin;

/*!
 * Configuration
 */
const char* const StaleMateWinName = "Hekateros StaleMate Chess";
                                        ///< name (and id) of main window

/*!
 * StaleMate uses the rotating base Hekateros with a Graboid end effector
 */
#define SM_HEK_DOF_BASE       HEK_DOF_BASE
#define SM_HEK_DOF_TOTAL      (HEK_DOF_BASE + HEK_DOF_EFFECTOR_GRABOID)
#define SM_HEK_NSERVOS_BASE   HEK_NSERVOS_BASE
#define SM_HEK_NSERVOS_BASE_M (HEK_NSERVOS_BASE-1)
#define SM_HEK_NSERVOS_TOTAL  (HEK_NSERVOS_BASE + HEK_NSERVOS_EFFECTOR_GRABOID)
#define SM_HEK_NSERVOS_TOTAL_M  \
  (SM_HEK_NSERVOS_BASE_M + HEK_NSERVOS_EFFECTOR_GRABOID)

/*!
 * Video device major number.
 */
#ifdef STALEMATE_VID_DEV_MAJOR
const int StaleMateVidDevMajor = STALEMATE_VID_DEV_MAJOR;
#else
const int StaleMateVidDevMajor = 81;
#endif

//
// Window Sizes and Transformations
//
#if defined(ARCH_overo) && STALEMATE_DISPLAY==LGLCD43

#include "Hekateros/hw/lglcd43.h"

const Rot         StaleMateImgTransRot    = Rot0;         ///< rotation
const Align       StaleMateImgTransAlign  = AlignCenter;  ///< alignment
const bool        StaleMateImgTransCrop   = false;              ///< cropping
const int         StaleMateWinWidth       = LgLcd43DisplayWidth;  ///< width
const int         StaleMateWinHeight      = LgLcd43DisplayHeight; ///< height

#else // full size monitor 

const Rot         StaleMateImgTransRot    = Rot0;         ///< rotation
const Align       StaleMateImgTransAlign  = AlignCenter;  ///< alignment
const bool        StaleMateImgTransCrop   = false;              ///< cropping
const int         StaleMateWinWidth       = 1440;               ///< width
const int         StaleMateWinHeight      = 900;                ///< height

#endif // ARCH_overo ...

const int   StaleMateMaxButtonsPerMenu = 15;    ///< max number of buttons/menu


/*!
 * StaleMate States
 */
enum StaleMateState
{
  StaleMateStateStart,          ///< start
  StaleMateStateUnCalib,        ///< uncalibrated game
  StaleMateStateNoGame,         ///< no game in process
  StaleMateStatePlayingChess,   ///< playing chess
  StaleMateStateStepping,       ///< one stepping through game
  StaleMateStatePaused,         ///< game paused
  StaleMateStateEnd             ///< end (exit application)
};

/*!
 * Menu Actions
 */
enum StaleMateAction
{
  StaleMateActionQuit = UIActionNumOf,  ///< quit operation

  StaleMateActionIdle,                        ///< run in idle mode
  StaleMateActionTune,                        ///< tune application
  StaleMateActionCalib,                       ///< calibration
  StaleMateActionSetPos,                      ///< set arm positions 
  StaleMateActionNewGame,                     ///< start new game
  StaleMateActionResume,                      ///< resume playing
  StaleMateActionStep  ,                      ///< step one move
  StaleMateActionEndGame,                     ///< end game
  StaleMateActionMakeMove,                    ///< make a move
  StaleMateActionWaitForMove,                 ///< their move - wait, watch 
  StaleMateActionPark,                        ///< run in idle mode
  StaleMateActionHome,                        ///< stop play, go to home pos.
  StaleMateActionFreeze,                      ///< freeze game
  StaleMateActionScan,                        ///< scan for servos
  StaleMateActionEStop,                       ///< emergency stop
  StaleMateActionTestRgb,                     ///< test rgb channels
  StaleMateActionTestHsv,                     ///< test hsv channels
  StaleMateActionTestMotion,                  ///< test motion move 
  StaleMateActionTestUci,                     ///< test UCI 
  StaleMateActionReadCfg,                     ///< read configuration
  StaleMateActionSetSafe,                     ///< "say" the arm is safe 

  StaleMateActionNoOp                         ///< do nothing
};

/*!
 * Chess Piece Type
 */
enum ChessPieceType
{
  WhiteKing,
  WhiteQueen,
  WhiteRook,
  WhiteBishop,
  WhiteKnight,
  WhitePawn,

  BlackKing,
  BlackQueen,
  BlackRook,
  BlackBishop,
  BlackKnight,
  BlackPawn,

  NoPiece,

  ChessPieceTypeNumOf
};

/*!
 * Chess Piece Color
 */
enum ChessPieceColor
{
  WhitePiece,
  BlackPiece,
  NoColor
};

/*!
 * Chess Square
 */
struct ChessSquare_T
{
  int m_nRow;
  int m_nCol;
};

/*!
 * Chess Game State
 */
enum ChessGameState
{
  ChessGameStateNoGame,
  ChessGameStateStart,
  ChessGameStateHeksMove,
  ChessGameStateOppsMove,
  ChessGameStateCheckMate,
  ChessGameStateResign,
  ChessGameStateDraw
};

/*!
 * Candidate Move Chess Square Type
 */
struct CandidateSq_T
{
  int     m_nRow;         // canditate square row
  int     m_nCol;         // canditate square column
  int     m_nChannel;     // channel with metric
  double  m_fNorm;        // normalized distance metric [0.0, 1.0]
};

/*!
 * Chess Engine Codes
 */
#define UCI_MOVE_BUF_SIZE 4         ///< maximum size of {from,to} move buffer

const char UciCodeDraw      = '=';      ///< game is a draw
const char UciCodeResign    = 'Q';      ///< player resigned
const char UciCodeCheckMate = '#';      ///< checkmate
const char UciCodeInvalid   = '?';      ///< invalid move
const char UciCodeError     = '!';      ///< bad or unexpected response


//------------------------------------------------------------------------------
// StaleMateSession Class
//------------------------------------------------------------------------------

/*!
 * StaleMate Session State and Data Class
 */
class StaleMateSession
{
public:
  StaleMateSession();

  ~StaleMateSession();

  void SetVideoDevice(const char *sVidDevName, uint_t uVidDevMinor);

  void SetHekDevice(const char *sHekDevName, int nBaudRate);
  
  StaleMateState GetCurState()
  {
    return m_eCurState;
  }

  void SetCurState(StaleMateState eNewState)
  {
    if( eNewState != m_eCurState )
    {
      m_ePrevState  = m_eCurState;
      m_eCurState   = eNewState;
    } 
  }

  StaleMateState GetPrevState()
  {
    return m_ePrevState;
  }

  StaleMateState  m_eCurState;        ///< application current state
  StaleMateState  m_ePrevState;       ///< application previous state

  struct
  {
    rnrWindow      *pWin;             ///< application main gui window
    Menu           *pMenu;            ///< application main menu         
    GtkWidget      *wIconicChessBoard; ///< 
    GtkWidget      *wBlackLabel;      ///<
    GtkWidget      *wWhiteLabel;      ///<
    GtkWidget      *wViewHistory;     ///<
    GtkTextBuffer  *wBufHistory;      ///<
    GtkWidget      *wChainState;      ///<
  } m_gui;


  struct
  {
    char           *sVidDevName;      ///< video device name
    int             uVidDevMinor;     ///< video device minor number (index)
    CvSize          sizeVideo;        ///< WxH size of video
    CvCapture      *pVidCapture;      ///< video capture handle
    IplImage       *pImgFrame;        ///< captured image frame
    IoI      *pVidToGuiTrans;   ///< video to gui image transform operator
    IplImage       *pImgDisplay0;     ///< live, annotated video
    uint_t          uImgIndex0;       ///< windows index for display image 0
    IplImage       *pImgDisplay1;     ///< vision/image processed image 
    uint_t          uImgIndex1;       ///< windows index for display image 1
  } m_vid;

  struct
  { 
    bool            bUseArm;          ///< no arm used
    bool            bIsSafe;          ///< arm is safe to move
    char           *sHekDevName;      ///< hekateros device name
    int             nHekBaudRate;     ///< hekateros baudrate
    bool            bUseOpenRave;
    DynaComm       *pDynaComm;        ///< dynamixel communication
    DynaChain      *pDynaChain;       ///< dynamixel chain
    DynaBgThread   *pDynaBgThread;    ///< dynamixel background thread
    SMBotPlanner    motionPlanner;
    bool            bHomeDefined;     ///< hekateros' home pos is [not] defined
    bool            bParkDefined;     ///< hekateros' park pos is [not] defined
    bool            bAtHomePos;       ///< hekateros is [not] at home
    bool            bIsParked;        ///< hekateros is [not] parked
  } m_hek;

  struct
  {
    bool            bCalibrated;      ///<
    CvPoint         ptBoard[8][8][4]; ///< pixel chess board position map
    CvRect          rectRoiBoard;     ///< 
    CvPoint2D32f    ptDEBottom;       ///< dialog specified D-E point
    CvPoint2D32f    ptHekDist[8][8];  ///< chess (x,y) distance in mm from arm
    IplImage       *pImgEmptyRed;     ///< 
    IplImage       *pImgEmptyBlue;    ///< 
  } m_calib;

  struct
  {
    bool            bUseChessEngine;  ///< 
    int             nChessBoardDim;   ///< normal 8x8 but can be smaller
    bool            bHekHasWhite;     ///< hekateros does [not] have white
    ChessGameState  eGameState;       ///<
    IplImage       *pImgPrevRed;      ///<
    double          fDiffThRed;       ///<
    double          fBoardThRed;      ///<
    IplImage       *pImgPrevBlue;     ///<
    double          fDiffThBlue;      ///<
    double          fBoardThBlue;     ///<
    uint_t          uSpikedFrameCnt;  ///<
    ChessSquare_T   sqHekFrom;        ///<
    ChessSquare_T   sqHekTo;          ///<
    ChessSquare_T   sqOppFrom;        ///<
    ChessSquare_T   sqOppTo;          ///<
  } m_game;
};

/*!
 * Action Function Type.
 */
typedef int (*StaleMateActionFunc_T)(StaleMateSession &);


//------------------------------------------------------------------------------
// External Data
//------------------------------------------------------------------------------

extern DynaPosTuple_T   HekKeyPosPark[];
extern DynaPosTuple_T   HekKeyPosHome[];
extern DynaPosTuple_T   HekKeyPosMotPlanner[];
extern DynaPosTuple_T   HekKeyPosGripperOpen;
extern DynaPosTuple_T   HekKeyPosGripperNarrow;
extern DynaPosTuple_T   HekKeyPosGripperGrab;
extern DynaPosTuple_T   HekKeyPosGripperClose;



//------------------------------------------------------------------------------
// Inline utilities
//------------------------------------------------------------------------------

/*!
 * \brief Allocate new duplicated string.
 *
 * \param s String to duplicate.
 *
 * \return Returns pointer to allocated string if s is not NULL and the
 * length of s \h_gt 0.\n Otherwise returns NULL.
 */
inline char *newstr(const char *s)
{
  char  *t;
  if( (s != NULL) && (*s != 0) )
  {
    t = new char[strlen(s)+1];
    strcpy(t, s);
  }
  else
  {
    t = NULL;
  }
  return t;
}

/*!
 * \brief Delete allocated string.
 *
 * \param s String to delete.
 *
 * \return Returns NULL.
 */
inline char *delstr(const char *s)
{
  if( s != NULL )
  {
    delete[] s;
  }
  return NULL;
}

/*!
 * \brief Map virtual algebriac square location to the virtual row and col.
 *
 * The virtual chess board always has 'white' at the bottom and 'black' at the
 * top. So "a1" --> 7,0  and "h8" --> 0,7.
 *
 * \param session   StaleMate session data.
 * \param sAlgSq    Virtual chess square location in algebraic notation.
 *
 * \return Virtual row,col location
 */
inline ChessSquare_T VAlgSqToVRowCol(StaleMateSession &session,
                                    const char       *sAlgSq)
{
  ChessSquare_T sq;
  sq.m_nCol = tolower(sAlgSq[0]) - (int)'a';
  sq.m_nRow = session.m_game.nChessBoardDim - 1 -
                                    (tolower(sAlgSq[1]) - (int)'1');
  return sq;
}

/*!
 * \brief Map virtual algebriac square location to the physical row and col.
 *
 * The virtual chess board always has 'white' at the bottom and 'black' at the
 * top. So "a1" --> 7,0  and "h8" --> 0,7. If Hekateros is playing 'black', then
 * the physical board is rotated 180\h_deg.
 *
 * \param session   StaleMate session data.
 * \param sAlgSq    Virtual chess square location in algebraic notation.
 *
 * \return Physical row,col location
 */
inline ChessSquare_T VAlgSqToPRowCol(StaleMateSession &session,
                                       const char       *sAlgSq)
{
  ChessSquare_T sq = VAlgSqToVRowCol(session, sAlgSq);

  if( !session.m_game.bHekHasWhite )
  {
    sq.m_nRow = session.m_game.nChessBoardDim - 1 - sq.m_nRow;
    sq.m_nCol = session.m_game.nChessBoardDim - 1 - sq.m_nCol;
  }
  return sq;
}

/*!
 * \brief Map physical row and col location to the virtual row and col.
 *
 * \param session   StaleMate session data.
 * \param sqP       Physical chess quare row,col location
 *
 * \return Virtual row,col location
 */
inline ChessSquare_T PRowColToVRowCol(StaleMateSession &session,
                                      ChessSquare_T    &sqP)
{
  ChessSquare_T sqV;

  if( session.m_game.bHekHasWhite )
  {
    sqV = sqP;
  }
  else
  {
    sqV.m_nRow = session.m_game.nChessBoardDim - 1 - sqP.m_nRow;
    sqV.m_nCol = session.m_game.nChessBoardDim - 1 - sqP.m_nCol;
  }
  return sqV;
}

/*!
 * \brief Map virtual row and col location to the physical row and col.
 *
 * \param session   StaleMate session data.
 * \param sq        Virtual chess quare row,col location
 *
 * \return Physical row,col location
 */
inline ChessSquare_T VRowColToPRowCol(StaleMateSession &session,
                                      ChessSquare_T    &sqV)
{
  ChessSquare_T sqP;

  if( session.m_game.bHekHasWhite )
  {
    sqP = sqV;
  }
  else
  {
    sqP.m_nRow = session.m_game.nChessBoardDim - 1 - sqV.m_nRow;
    sqP.m_nCol = session.m_game.nChessBoardDim - 1 - sqV.m_nCol;
  }
  return sqP;
}

//------------------------------------------------------------------------------
// Function Prototypes
//------------------------------------------------------------------------------

//
// master control
//
extern int StaleMateMasterControl(StaleMateSession &session);


//
// calibration
//
extern void StaleMateCalib(StaleMateSession &session);


//
// gui
//
extern void StaleMateGuiDlgNewGame(StaleMateSession &session);

extern void StaleMateGuiDlgTune(StaleMateSession &session);

extern void StaleMateGuiDlgSetPos(StaleMateSession &session);

extern void StaleMateGuiDlgTestMove(StaleMateSession &session,
                                    char              bufAlgSq[],
                                    double           *pfAlgZ,
                                    CvPoint3D32f     *pPtCart);


//
// game
//
extern int StaleMateGameNew(StaleMateSession &session);

extern int StaleMateGameUpdate(StaleMateSession &session);

extern void StaleMateGameHekaterosStart(StaleMateSession &session);

extern bool StaleMateGameHekaterosMove(StaleMateSession &session);

extern void StaleMateGameOpponentsStart(StaleMateSession &session);

extern bool StaleMateGameOpponentsMove(StaleMateSession &session);

//
// vision
//
extern int StaleMateVisionRecordBoardState(StaleMateSession &session);

extern bool StaleMateVisionWatchForOpponentsMove(StaleMateSession &session);

extern int StaleMateVisionFindBestCandidates(StaleMateSession &session,
                                             CandidateSq_T     candidate[],
                                             int               nCount);

extern double StaleMateVisionCheckEmptySq(StaleMateSession &session,
                                          int nRow, int nCol);

inline double StaleMateVisionCheckEmptySq(StaleMateSession &session,
                                          ChessSquare_T    &sq)
{
  return StaleMateVisionCheckEmptySq(session, sq.m_nRow, sq.m_nCol);
}

extern void StaleMateVisionDrawSq(StaleMateSession &session,
                                  IplImage         *pImg,
                                  int               nRow,
                                  int               nCol,
                                  const char       *sText,
                                  CvScalar          color);

inline void StaleMateVisionDrawSq(StaleMateSession &session,
                                  IplImage         *pImg,
                                  ChessSquare_T    &sq,
                                  const char       *sText,
                                  CvScalar          color)
{
  StaleMateVisionDrawSq(session, pImg, sq.m_nRow, sq.m_nCol, sText, color);
}


//
// live video feed
//
extern int StaleMateLiveFeedThreadStart(StaleMateSession *pSession);

extern void StaleMateLiveFeedThreadStop();

extern void StaleMateLiveFeedThreadJoin();

extern void StaleMateLiveFeedThreadStopJoin();

extern int StaleMateLiveFeedCaptureStart(StaleMateSession &session);

extern void StaleMateLiveFeedCaptureStop(StaleMateSession &session);

extern void StaleMateLiveFeedShow(StaleMateSession &session);

extern int StaleMateVideoCaptureFrame(StaleMateSession   &session,
                                      IplImage         **ppImgFrame);

extern IplImage *StaleMateVideoCreateSnapShot(StaleMateSession &session);


//
// hekateros
//
extern GtkWidget *StaleMateHekGuiInit(StaleMateSession &session);

extern void StaleMateHekGuiInitState(StaleMateSession &session);

extern void StaleMateHekGuiShowChainState(StaleMateSession &session);

extern void StaleMateHekGuiShowServoState(StaleMateSession &session,
                                          int               nServoId);

extern void StaleMateHekEStop(StaleMateSession &session);

extern void StaleMateHekFreezeAll(StaleMateSession &session);

extern void StaleMateHekReleaseAll(StaleMateSession &session);

extern void StaleMateHekGotoHome(StaleMateSession &session);

extern void StaleMateHekParkIt(StaleMateSession &session);

extern void StaleMateHekMoveTo(StaleMateSession &session,
                               CvPoint3D32f     &ptGoal);

extern void StaleMateHekMoveTo(StaleMateSession &session,
                               const char       *sAlgSq,
                               double            fZ);

extern void StaleMateHekGripperOpen(StaleMateSession &session);

extern void StaleMateHekGripperNarrow(StaleMateSession &session);

extern void StaleMateHekGripperGrab(StaleMateSession &session);

extern bool StaleMateHekIsStopped(StaleMateSession &session);

inline bool StaleMateHekIsMoving(StaleMateSession &session)
{
  return StaleMateHekIsStopped(session)? false: true;
}

extern void StaleMateHekDisableOdometry(StaleMateSession &session);

extern void StaleMateHekSyncMove(StaleMateSession &session,
                                 DynaPosTuple_T    tup[],
                                 uint_t            uNumTups);

extern void StaleMateHekGetCurPos(StaleMateSession &session,
                                  DynaPosTuple_T    tupCurPos[],
                                  int               nTups);

extern void StaleMateHekReadCurPos(StaleMateSession &session,
                                   DynaPosTuple_T    tupCurPos[],
                                   int               nTups);

extern void StaleMateHekWriteSafeGoalSpeeds(StaleMateSession &session,
                                            int               nGoalSpeed);

extern int StaleMateHekInit(StaleMateSession &session);

extern void StaleMateHekScan(StaleMateSession &session);

extern bool StaleMateHekIsSafe(StaleMateSession &session, bool bReadPos=false);

//
// iconic chess
//
extern void StaleMateChessOneTimeInit(StaleMateSession &session);

extern const char *StaleMateChessBoardCoordStr(int nRow, int nCol);

inline const char *StaleMateChessBoardCoordStr(ChessSquare_T sq)
{
  return StaleMateChessBoardCoordStr(sq.m_nRow, sq.m_nCol);
}

extern const char *StaleMateChessPieceLongName(ChessPieceType eChessPiece);

extern const char *StaleMateChessPieceAlgName(ChessPieceType eChessPiece);

extern void StaleMateChessNewGame(StaleMateSession &session);

extern void StaleMateChessNoGame(StaleMateSession &session);

extern bool StaleMateChessIsCastleMove(StaleMateSession &session,
                                       const char       *sAlgSqFrom,
                                       const char       *sAlgSqTo);

extern void StaleMateChessCastleRook(StaleMateSession &session,
                                     char             *sAlgSqFrom,
                                     char             *sAlgSqTo);

extern void StaleMateChessCastled(StaleMateSession &session,
                                  ChessPieceColor  eChessSide);

extern void StaleMateChessMovePiece(StaleMateSession &session,
                                    int nFromRow, int nFromCol,
                                    int nToRow,   int nToCol);

inline void StaleMateChessMovePiece(StaleMateSession &session,
                                    ChessSquare_T sqFrom,
                                    ChessSquare_T sqTo)
{
  StaleMateChessMovePiece(session, sqFrom.m_nRow, sqFrom.m_nCol,
                                   sqTo.m_nRow, sqTo.m_nCol);
}

inline void StaleMateChessMovePiece(StaleMateSession &session,
                                    const char       *sAlgSqFrom,
                                    const char       *sAlgSqTo)
{
  StaleMateChessMovePiece(session,
                          VAlgSqToVRowCol(session, sAlgSqFrom),
                          VAlgSqToVRowCol(session, sAlgSqTo));
}

extern void StaleMateChessRemovePiece(StaleMateSession &session,
                                      int nRow, int nCol);

inline void StaleMateChessRemovePiece(StaleMateSession &session,
                                      ChessSquare_T     sq)
{
  StaleMateChessRemovePiece(session, sq.m_nRow, sq.m_nCol);
}

inline void StaleMateChessRemovePiece(StaleMateSession &session,
                                      const char       *sAlgSq)
{
  StaleMateChessRemovePiece(session, VAlgSqToVRowCol(session, sAlgSq));
}

extern bool StaleMateChessIsEmptySquare(StaleMateSession &session,
                                        int nRow, int nCol);

inline bool StaleMateChessIsEmptySquare(StaleMateSession &session,
                                        ChessSquare_T     sq)
{
  return StaleMateChessIsEmptySquare(session, sq.m_nRow, sq.m_nCol);
}

inline bool StaleMateChessIsEmptySquare(StaleMateSession &session,
                                        const char       *sAlgSq)
{
  return StaleMateChessIsEmptySquare(session, VAlgSqToVRowCol(session, sAlgSq));
}

extern ChessPieceType StaleMateChessGetPieceType(StaleMateSession &session,
                                        int nRow, int nCol);

inline ChessPieceType StaleMateChessGetPieceType(StaleMateSession &session,
                                                 ChessSquare_T     sq)
{
  return StaleMateChessGetPieceType(session, sq.m_nRow, sq.m_nCol);
}

inline ChessPieceType StaleMateChessGetPieceType(StaleMateSession &session,
                                                 const char       *sAlgSq)
{
  return StaleMateChessGetPieceType(session, VAlgSqToVRowCol(session, sAlgSq));
}

extern ChessPieceColor StaleMateChessGetPieceColor(StaleMateSession &session,
                                                   int nRow, int nCol);

inline ChessPieceColor StaleMateChessGetPieceColor(StaleMateSession &session,
                                                   ChessSquare_T     sq)
{
  return StaleMateChessGetPieceColor(session, sq.m_nRow, sq.m_nCol);
}

inline ChessPieceColor StaleMateChessGetPieceColor(StaleMateSession &session,
                                                   const char       *sAlgSq)
{
  return StaleMateChessGetPieceColor(session, VAlgSqToVRowCol(session, sAlgSq));
}

extern void StaleMateChessHistClear(StaleMateSession &session);

extern void StaleMateChessHistAdd(StaleMateSession &session,
                                  const char       *sFmt,
                                  ...);


//
// universal chess interface
//
extern int StaleMateUciOneTimeInit(StaleMateSession &session,
                                  const char *sChessApp=TuneChessEngine);

extern void StaleMateUciKill(StaleMateSession &session);

extern int StaleMateUciTrans(StaleMateSession &session,
                             const char       *sReq,
                             char              bufRsp[],
                             size_t            sizeRspBuf);

extern void StaleMateUciCfg(StaleMateSession &session);

extern void StaleMateUciNewGame(StaleMateSession &session);

extern bool StaleMateUciGetEnginesMove(StaleMateSession &session,
                                       char              bufAlgSqFrom[],
                                       char              bufAlgSqTo[]);

extern int StaleMateUciPlayersMove(StaleMateSession &session,
                                   const char       *sAlgSqFrom,
                                   const char       *sAlgSqTo,
                                   char             *pUciCode);

extern void StaleMateUciSelfPlay(StaleMateSession &session, bool bIsNewGame);



//
// OpenRave Python interface
//
extern int StaleMateOrpOneTimeInit(StaleMateSession &session,
                                  const char *sOpenRaveApp=TuneOpenRavePython);

extern void StaleMateOrpCfg(StaleMateSession &session);

extern void StaleMateOrpKill(StaleMateSession &session);

extern int StaleMateOrpDegToRadAndSend(StaleMateSession &session,
                                       DynaRealTuple_T   posDeg[],
                                       size_t            nPos);


//
// utilities
//
extern void StaleMateIoIShow(StaleMateSession &session,
                             IplImage         *pIoI,
                             uint_t            uImgIndex);

extern IplImage *StaleMateCreateHsvChannel(IplImage *pImg, int nHsvChannel);

extern IplImage *StaleMateCreateRgbChannel(IplImage *pImg, int nRgbChannel);

extern void StaleMateReadCfg(StaleMateSession &session);


//
// test functions
//
extern int StaleMateTestHoughLines(StaleMateSession &session);

extern int StaleMateTestRgbChannel(StaleMateSession &session, int nRgbChannel);

extern int StaleMateTestHsvChannel(StaleMateSession &session, int nHsvChannel);

extern int StaleMateTestChessBoardCalib(StaleMateSession &session);

extern int StaleMateTestGrayScaleDifference(StaleMateSession &session,
                                            int              nRgbChannel);

extern int StaleMateTestMask(StaleMateSession &session, int nRow, int nCol);

extern int StaleMateTestMove(StaleMateSession &session);

extern int StaleMateTestPlayChess(StaleMateSession &session,
                                  bool              bUseArm,
                                  bool              bUseVision,
                                  bool              bUseGnuChess);

extern bool StaleMateTestUci(StaleMateSession &session);


#endif // _STALEMATE_H
