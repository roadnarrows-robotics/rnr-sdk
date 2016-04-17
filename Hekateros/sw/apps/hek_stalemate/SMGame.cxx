////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMGameState.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-18 10:01:05 -0600 (Mon, 18 Jun 2012) $
 * $Rev: 2056 $
 *
 * \brief Determine, track, and play the game.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows
 * (http://www.roadnarrows.com)
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

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * Optimal height above the board (z) to grap a particular chess piece.
 * Indexed ChessPieceType.
 */
static double GrabPieceZ[] =
{
  50.0, 50.0, 25.0, 25.0, 25.0, 10.0,
  50.0, 50.0, 25.0, 25.0, 25.0, 10.0,
  -5.0
};

enum HekChoreographState
{
  HekChoreoStateStart,
  HekChoreoStateGotoOppPiece,
  HekChoreoStateCenterOnOppPiece,
  HekChoreoStateNarrowGripperOverOppSq,
  HekChoreoStateDropDownOverOppPiece,
  HekChoreoStateGrabOppPiece,
  HekChoreoStateLiftOppPiece,
  HekChoreoStateGotoDeadPool,
  HekChoreoStateTossOppPiece,
  HekChoreoStateGotoFromSq,
  HekChoreoStateCenterOnHekPiece,
  HekChoreoStateNarrowGripperOverHekSq,
  HekChoreoStateDropDownOverHekPiece,
  HekChoreoStateGrabHekPiece,
  HekChoreoStateLiftHekPiece,
  HekChoreoStateGotoToSq,
  HekChoreoStateDropDownOverEmptySq,
  HekChoreoStateDropHekPiece,
  HekChoreoStateLiftUp,
  HekChoreoStateGoToHome,
  HekChoreoStateWait,
  HekChoreoStateEnd
};

struct HekChoreograph_T
{
  char                m_bufAlgSqFrom[4];    ///< 'from' virtual square 
  char                m_bufAlgSqTo[4];      ///< 'to' virtual square
  HekChoreographState m_eCurState;          ///< current choreograph state
  HekChoreographState m_eNextState;         ///< next choreograph state
  bool                m_bCastling;          ///< is [not] in costle move
};

static HekChoreograph_T HekChoreo;

static void SMGameHekFineTuneMoveOccupied(StaleMateSession &session,
                                          const char       *sAlgSq)
{
  // RDK TODO
}

static bool SMGameOppTryBestMove(StaleMateSession &session,
                                 CandidateSq_T     candidate[],
                                 int               nCandidates,
                                 int               iFromIdx,
                                 ChessSquare_T    *pSqFrom,
                                 ChessSquare_T    *pSqTo,
                                 char             *pUciCode)
{
  ChessPieceColor eHekColor;
  ChessPieceColor eOppColor;
  ChessPieceColor eSqColor;
  double          fNorm;
  ChessSquare_T   sqFrom;
  bool            bGotFrom;
  ChessSquare_T   sqTo;
  bool            bGotTo;
  int             i;
  int             rc;

  //
  // Find the best candidate square such that:
  //  o it was previously occuppied by an opponent's piece
  //  o now empty
  //
  eHekColor = session.m_game.bHekHasWhite? WhitePiece: BlackPiece;
  eOppColor = session.m_game.bHekHasWhite? BlackPiece: WhitePiece;

  bGotFrom = false;
  i        = iFromIdx;

  // physical location
  sqFrom.m_nRow = candidate[i].m_nRow;
  sqFrom.m_nCol = candidate[i].m_nCol;

  // map to virtual
  sqFrom = PRowColToVRowCol(session, sqFrom);

  // 'from' square was an empty
  if( StaleMateChessIsEmptySquare(session, sqFrom) )
  {
    //cerr << "DBG: sqaure[" << sqFrom.m_nRow << "][" << sqFrom.m_nCol
    //  << "] empty" << endl;
    return false;
  }

  // 'from' square is a different color
  else if( (eSqColor = StaleMateChessGetPieceColor(session, sqFrom))
                                                                  != eOppColor )
  {
    //cerr << "DBG: sqaure[" << sqFrom.m_nRow << "][" << sqFrom.m_nCol
    //  << "] color " << eSqColor << " != " << eOppColor << endl;
    return false;
  }

  // 'fram' square delta is to low
  else if( (fNorm = StaleMateVisionCheckEmptySq(session, sqFrom)) > 0.25 )
  {
    //cerr << "DBG: sqaure[" << sqFrom.m_nRow << "][" << sqFrom.m_nCol
    //  << "] empty norm " << fNorm << " > 0.25" << endl;
    return false;
  }

  else
  {
    bGotFrom = true;
  }

  //LOGDIAG2("Candiate opponent's 'from' move: %s",
  //    StaleMateChessBoardCoordStr(sqFrom));

  //
  // Find the best candidate destination square such that:
  //  o it was empty or occuppied by a hekateros' piece
  //  o now empty
  //
  for(i=0, bGotTo=false; (i<nCandidates) && !bGotTo; ++i)
  {
    // physical location
    sqTo.m_nRow = candidate[i].m_nRow;
    sqTo.m_nCol = candidate[i].m_nCol;

    // map to virtual
    sqTo = PRowColToVRowCol(session, sqTo);

    // same as 'from' square
    if( (sqTo.m_nRow == sqFrom.m_nRow) && (sqTo.m_nCol == sqTo.m_nRow) )
    {
      continue;
    }
    
    // took a piece
    else if( (StaleMateChessGetPieceColor(session, sqTo) == eHekColor) &&
              (candidate[i].m_nChannel == CHANNEL_BLUE) )
    {
      bGotTo = true;
    }

    // moved to an empty square
    else if( StaleMateChessIsEmptySquare(session, sqTo) )
    {
      bGotTo = true;
    }

    LOGDIAG2("Try opponent's move: %s%s",
                  StaleMateChessBoardCoordStr(sqFrom),
                  StaleMateChessBoardCoordStr(sqTo));

    // test if move is valid
    rc = StaleMateUciPlayersMove(session, StaleMateChessBoardCoordStr(sqFrom),
                                          StaleMateChessBoardCoordStr(sqTo),
                                          pUciCode);

    // invalid
    if( rc != HEK_OK )
    {
      bGotTo = false;
    }
  }

  if( !bGotTo )
  {
    return false;
  }
  
  *pSqFrom  = sqFrom;
  *pSqTo    = sqTo;

  return true;
}
                                 

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

int StaleMateGameNew(StaleMateSession &session)
{
  int   rc;

  StaleMateChessNewGame(session);
  StaleMateUciNewGame(session);
  rc = StaleMateVisionRecordBoardState(session);

  return rc;
}

int StaleMateGameUpdate(StaleMateSession &session)
{
  int   rc;

  session.m_gui.pWin->WaitKey(1000);
  rc = StaleMateVisionRecordBoardState(session);
  session.m_game.uSpikedFrameCnt = 0;

  return rc;
}

void StaleMateGameHekaterosStart(StaleMateSession &session)
{
  HekChoreo.m_bufAlgSqFrom[0] = 0;
  HekChoreo.m_bufAlgSqTo[0]   = 0;
  HekChoreo.m_eCurState       = HekChoreoStateStart;
  HekChoreo.m_eNextState      = HekChoreoStateStart;
  HekChoreo.m_bCastling       = false;
}

bool StaleMateGameHekaterosMove(StaleMateSession &session)
{
  static bool   bShowThinking = true;
  CvPoint3D32f  ptGoal;
  double        z;
  bool          bMoveIsDone = false;

  switch( HekChoreo.m_eCurState )
  {
    //
    // Start Hekateros move by retrieving the chess engine calculated move.
    //
    case HekChoreoStateStart:
      // get backend chess engine decision
      if( bShowThinking )
      {
        session.m_gui.pWin->ShowStatus("Hekateros is thinking");
        bShowThinking = false;
      }

      HekChoreo.m_bCastling = false;

      if( StaleMateUciGetEnginesMove(session, HekChoreo.m_bufAlgSqFrom,
                                              HekChoreo.m_bufAlgSqTo) )
      {
        LOGDIAG2("Choreagraphy: Make move: %s%s.",
                HekChoreo.m_bufAlgSqFrom, HekChoreo.m_bufAlgSqTo);

        bShowThinking = true;

        if( HekChoreo.m_bufAlgSqFrom[0] == UciCodeCheckMate )
        {
          session.m_game.eGameState = ChessGameStateCheckMate;
          HekChoreo.m_eCurState     = HekChoreoStateEnd;
        }

        else if( HekChoreo.m_bufAlgSqFrom[0] == UciCodeResign )
        {
          session.m_game.eGameState = ChessGameStateResign;
          HekChoreo.m_eCurState     = HekChoreoStateEnd;
        }

        else if( HekChoreo.m_bufAlgSqFrom[0] == UciCodeDraw )
        {
          session.m_game.eGameState = ChessGameStateDraw;
          HekChoreo.m_eCurState     = HekChoreoStateEnd;
        }

        else if( StaleMateChessIsCastleMove(session,
                                            HekChoreo.m_bufAlgSqFrom,
                                            HekChoreo.m_bufAlgSqTo) )
        {
          HekChoreo.m_bCastling = true;
          HekChoreo.m_eCurState = HekChoreoStateGotoFromSq;
        }

        // destination square is empty
        else if( StaleMateChessIsEmptySquare(session, HekChoreo.m_bufAlgSqTo) )
        {
          HekChoreo.m_eCurState = HekChoreoStateGotoFromSq;
        }

        // distination square is occupied by an opponent's piece
        else
        {
          HekChoreo.m_eCurState = HekChoreoStateGotoOppPiece;
        }
      }
      break;

    //
    // Need to remove an opponent's piece, so go to the destination square
    //
    case HekChoreoStateGotoOppPiece:
      LOGDIAG2("Choreagraphy: Go to opponent's square %s.",
              HekChoreo.m_bufAlgSqTo);
      session.m_gui.pWin->ShowStatus("Taking oppontent's piece at %s",
                            HekChoreo.m_bufAlgSqTo);
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, 250.0);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateCenterOnOppPiece;
      break;

    //
    // Now center of the opponent's piece
    //
    case HekChoreoStateCenterOnOppPiece:
      LOGDIAG2("Choreagraphy: Center on opponent's square.");
      SMGameHekFineTuneMoveOccupied(session, HekChoreo.m_bufAlgSqTo);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateNarrowGripperOverOppSq;
      break;

    //
    // Narrow the gripper so that it fits within the chess board square
    // boundry to reduce collisions with other pieces.
    //
    case HekChoreoStateNarrowGripperOverOppSq:
      LOGDIAG2("Choreagraphy: Narrow gripper.");
      StaleMateHekGripperNarrow(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateDropDownOverOppPiece;
      break;

    //
    // Now drop down over the opponents piece in preparation to grab the piece.
    //
    case HekChoreoStateDropDownOverOppPiece:
      LOGDIAG2("Choreagraphy: Drop down.");
      z = GrabPieceZ[
                  StaleMateChessGetPieceType(session,HekChoreo.m_bufAlgSqTo) ]; 
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, z);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateGrabOppPiece;
      break;

    //
    // Grab the opponent's piece.
    //
    case HekChoreoStateGrabOppPiece:
      LOGDIAG2("Choreagraphy: Grab opponent's piece.");
      StaleMateHekGripperGrab(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateLiftOppPiece;
      break;

    //
    // Lift the opponent's piece straight up.
    //
    case HekChoreoStateLiftOppPiece:
      LOGDIAG2("Choreagraphy: Lift up.");
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, 250.0);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateGotoDeadPool;
      break;

    //
    // Move to the 'dead pool' to the lower right of the chess board.
    //
    case HekChoreoStateGotoDeadPool:
      LOGDIAG2("Choreagraphy: Move to 'dead pool'.");
      ptGoal.x = 200.0;
      ptGoal.y = -350.0;
      ptGoal.z = 250.0;
      StaleMateHekMoveTo(session, ptGoal);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateTossOppPiece;
      break;

    //
    // Toss the opponent's piece and update the iconic chess state.
    //
    case HekChoreoStateTossOppPiece:
      LOGDIAG2("Choreagraphy: Toss opponent's dead man.");
      StaleMateHekGripperOpen(session);
      StaleMateChessRemovePiece(session, HekChoreo.m_bufAlgSqTo);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateGotoFromSq;
      break;

    //
    // Go to Hekateros' piece to be moved.
    //
    case HekChoreoStateGotoFromSq:
      LOGDIAG2("Choreagraphy: Go to Hekateros' 'from' square: %s.",
                  HekChoreo.m_bufAlgSqFrom);
      session.m_gui.pWin->ShowStatus("Moving Hek's piece from %s to %s",
                            HekChoreo.m_bufAlgSqFrom, HekChoreo.m_bufAlgSqTo);
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqFrom, 250.0);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateCenterOnHekPiece;
      break;

    //
    // Now center of the Hekateros' piece.
    //
    case HekChoreoStateCenterOnHekPiece:
      LOGDIAG2("Choreagraphy: Center on Hekateros' square.");
      SMGameHekFineTuneMoveOccupied(session, HekChoreo.m_bufAlgSqFrom);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateNarrowGripperOverHekSq;
      break;

    //
    // Narrow the gripper so that it fits within the chess board square
    // boundry to reduce collisions with other pieces.
    //
    case HekChoreoStateNarrowGripperOverHekSq:
      LOGDIAG2("Choreagraphy: Narrow gripper.");
      StaleMateHekGripperNarrow(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateDropDownOverHekPiece;
      break;

    //
    // Now drop down over the Hekateros piece in preparation to grab the piece.
    //
    case HekChoreoStateDropDownOverHekPiece:
      LOGDIAG2("Choreagraphy: Drop down.");
      z = GrabPieceZ[
                StaleMateChessGetPieceType(session,HekChoreo.m_bufAlgSqFrom) ]; 
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqFrom, z);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateGrabHekPiece;
      break;

    //
    // Grab the Hekateros' piece.
    //
    case HekChoreoStateGrabHekPiece:
      LOGDIAG2("Choreagraphy: Grab Hekateros' piece.");
      StaleMateHekGripperGrab(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateLiftHekPiece;
      break;

    //
    // Lift the Hekateros' piece straight up.
    //
    case HekChoreoStateLiftHekPiece:
      LOGDIAG2("Choreagraphy: Lift up.");
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqFrom, 250.0);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateGotoToSq;
      break;

    //
    // Go to Hekateros' destination empty chess board square. 
    //
    case HekChoreoStateGotoToSq:
      LOGDIAG2("Choreagraphy: Go to Hekateros' 'to' square: %s.",
                  HekChoreo.m_bufAlgSqTo);
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, 250.0);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateDropDownOverEmptySq;
      break;

    //
    // Drop down over the empty square.
    //
    case HekChoreoStateDropDownOverEmptySq:
      LOGDIAG2("Choreagraphy: Drop down.");
      z = GrabPieceZ[
                StaleMateChessGetPieceType(session,HekChoreo.m_bufAlgSqFrom)]; 
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, z+5 );
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateDropHekPiece;
      break;

    //
    // Carefully drop the Hekateros' piece.
    //
    case HekChoreoStateDropHekPiece:
      LOGDIAG2("Choreagraphy: Drop piece.");
      StaleMateHekGripperNarrow(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateLiftUp;
      break;

    //
    // Lift the Hekateros arm straight up.
    //
    case HekChoreoStateLiftUp:
      LOGDIAG2("Choreagraphy: Lift up.");
      StaleMateHekMoveTo(session, HekChoreo.m_bufAlgSqTo, 250.0);
      StaleMateChessMovePiece(session, HekChoreo.m_bufAlgSqFrom,
                                       HekChoreo.m_bufAlgSqTo);
      if( HekChoreo.m_bCastling )
      {
        StaleMateChessCastleRook(session,
                                 HekChoreo.m_bufAlgSqFrom,
                                 HekChoreo.m_bufAlgSqTo);
        StaleMateChessCastled(session,
                    session.m_game.bHekHasWhite? WhitePiece: BlackPiece);
        HekChoreo.m_eCurState  = HekChoreoStateWait;
        HekChoreo.m_eNextState = HekChoreoStateGotoFromSq;
        HekChoreo.m_bCastling  = false;
      }
      else
      {
        HekChoreo.m_eCurState  = HekChoreoStateWait;
        HekChoreo.m_eNextState = HekChoreoStateGoToHome;
      }
      break;

    //
    // Put Hekateros back at home position.
    //
    case HekChoreoStateGoToHome:
      LOGDIAG2("Choreagraphy: Goto Home.");
      StaleMateHekGotoHome(session);
      HekChoreo.m_eCurState  = HekChoreoStateWait;
      HekChoreo.m_eNextState = HekChoreoStateEnd;
      break;

    //
    // Wait for Hekateros to stop moving, the transition to the next state.
    //
    case HekChoreoStateWait:
      if( StaleMateHekIsStopped(session) )
      {
          HekChoreo.m_eCurState = HekChoreo.m_eNextState;
      }
      break;

    //
    // End of Hekateros move.
    //
    case HekChoreoStateEnd:
    default:
      LOGDIAG2("Choreagraphy: End.");
      bMoveIsDone = true;
      break;
  }

  return bMoveIsDone;
}

void StaleMateGameOpponentsStart(StaleMateSession &session)
{
  session.m_game.uSpikedFrameCnt = 0;
}

bool StaleMateGameOpponentsMove(StaleMateSession &session)
{
  int             nMaxCandidates = 6;
  CandidateSq_T   candidate[nMaxCandidates];
  int             nCandidates;
  bool            bGotMove;
  ChessSquare_T   sqFrom;
  ChessSquare_T   sqTo;
  char            cUciCode;
  CvScalar        colorFrom = CV_RGB(0, 192, 64);
  CvScalar        colorTo = CV_RGB(0, 64, 192);
  IplImage       *pImg;
  int             i;

  if( !StaleMateVisionWatchForOpponentsMove(session) )
  {
    return false;
  }

  nCandidates = StaleMateVisionFindBestCandidates(session, candidate,
                                                          nMaxCandidates);

  cerr << "DBG: number of candidates = " << nCandidates << endl;

  if( nCandidates < 2 )
  {
    session.m_game.uSpikedFrameCnt = 0;
    return false;
  }

  for(i=0; i<nCandidates; ++i)
  {
    cerr << "DBG: Candidate " << i
      << " = (" << candidate[i].m_nRow
      << "," << candidate[i].m_nCol
      << "," << candidate[i].m_nChannel
      << "," << candidate[i].m_fNorm
      << ")" << endl;
  }

  // try all of the (from,to) candidate combinations
  for(i=0, bGotMove=false; (i<nCandidates) && !bGotMove; ++i)
  {
    bGotMove = SMGameOppTryBestMove(session, candidate, nCandidates, i,
                                    &sqFrom, &sqTo, &cUciCode);
  }

  if( !bGotMove )
  {
    session.m_game.uSpikedFrameCnt = 0;
    return false;
  }

  //
  // Show move
  //
  if( (pImg = StaleMateVideoCreateSnapShot(session)) != NULL )
  {
    ChessSquare_T sqFromP = VRowColToPRowCol(session, sqFrom);
    ChessSquare_T sqToP   = VRowColToPRowCol(session, sqTo);
    StaleMateVisionDrawSq(session, pImg, sqFromP, "From", colorFrom);
    StaleMateVisionDrawSq(session, pImg, sqToP, "To", colorTo);
    StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex1);
    cvReleaseImage(&pImg);
  }

  switch( cUciCode )
  {
    case UciCodeCheckMate:
      session.m_game.eGameState = ChessGameStateCheckMate;
      break;
    case UciCodeResign:
      session.m_game.eGameState = ChessGameStateResign;
      break;
    case UciCodeDraw:
      session.m_game.eGameState = ChessGameStateDraw;
      break;
    default:
      // Update iconic chess state
      StaleMateChessMovePiece(session, sqFrom, sqTo);
      break;
  }

  return true;
}
