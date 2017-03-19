////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMUtil.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief StaleMate iconic chess interface.
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

#define TRY_SQUARE(n, r, c, rc) \
  do \
  { \
    if( (r < 0) || (r >= n) || (c < 0) || (c >= n) ) \
    { \
      LOGERROR("Chess coordinate (%d,%d) out-of-range.", r, c); \
      return rc; \
    } \
  } while(0)


//
// Algebraic chess coordinates (white perspective with A1 at white's bottom
// left corner).
// )
static const char *BoardCoordStr[8][8] =
{
  {"a8", "b8", "c8", "d8", "e8", "f8", "g8", "h8"},
  {"a7", "b7", "c7", "d7", "e7", "f7", "g7", "h7"},
  {"a6", "b6", "c6", "d6", "e6", "f6", "g6", "h6"},
  {"a5", "b5", "c5", "d5", "e5", "f5", "g5", "h5"},
  {"a4", "b4", "c4", "d4", "e4", "f4", "g4", "h4"},
  {"a3", "b3", "c3", "d3", "e3", "f3", "g3", "h3"},
  {"a2", "b2", "c2", "d2", "e2", "f2", "g2", "h2"},
  {"a1", "b1", "c1", "d1", "e1", "f1", "g1", "h1"}
};

struct ChessPiece_T
{
  const char *m_sLongName;
  const char *m_sAlgName;
  const char *m_sFileName;
  const int   m_nNumInstances;
  GtkWidget  *m_wImg[8];
};

static ChessPiece_T ChessPieceTbl[ChessPieceTypeNumOf] =
{
  {"White King",    "K",  "whiteking32.png",    1, NULL},
  {"White Queen",   "Q",  "whitequeen32.png",   2, NULL},
  {"White Rook",    "R",  "whiterook32.png",    2, NULL},
  {"White Bishop",  "B",  "whitebishop32.png",  2, NULL},
  {"White Knight",  "N",  "whiteknight32.png",  2, NULL},
  {"White Pawn",    "",   "whitepawn32.png",    8, NULL},

  {"Black King",    "K",  "blackking32.png",    1, NULL},
  {"Black Queen",   "Q",  "blackqueen32.png",   2, NULL},
  {"Black Rook",    "R",  "blackrook32.png",    2, NULL},
  {"Black Bishop",  "B",  "blackbishop32.png",  2, NULL},
  {"Black Knight",  "N",  "blackknight32.png",  2, NULL},
  {"Black Pawn",    "",   "blackpawn32.png",    8, NULL},

  {"no piece",      "",   "nopiece32.png",      0, NULL}
};

// 64 "no piece" widgets
static GtkWidget *NoPieceWidget[64];

struct ChessBoardState_T
{
  ChessPieceType    m_ePiece;
  ChessPieceColor   m_eColor;
  int               m_nInstance;
  GtkWidget        *m_wContainer;
};

static ChessBoardState_T ChessBoard[8][8];

static bool ChessWhiteCastled;
static bool ChessBlackCastled;

// RDK TODO and checks
static inline GtkWidget *ChessGuiGetPieceWidget(ChessPieceType eChessPiece,
                                                int            nInstance)
{
  if( eChessPiece == NoPiece )
  {
    return NoPieceWidget[nInstance];
  }
  else
  {
    return ChessPieceTbl[eChessPiece].m_wImg[nInstance];
  }
}

static inline int ChessGuiNoPieceInstance(int nRow, int nCol)
{
  return nRow * 8 + nCol;
}

static void ChessGuiReplacePiece(StaleMateSession &session,
                                 int nRow,        int nCol,
                                 ChessPieceType   eChessPiece,
                                 ChessPieceColor  eColor,
                                 int              nInstance)
{
  ChessBoardState_T *pSquare;

  pSquare = &ChessBoard[nRow][nCol];

  gtk_container_remove(GTK_CONTAINER(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(pSquare->m_ePiece, pSquare->m_nInstance));

  pSquare->m_ePiece     = eChessPiece;
  pSquare->m_eColor     = eColor;
  pSquare->m_nInstance  = nInstance;

  gtk_box_pack_start(GTK_BOX(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(pSquare->m_ePiece, pSquare->m_nInstance),
            TRUE, TRUE, 0);

  gtk_widget_show_all(pSquare->m_wContainer);
}

static void ChessGuiFillBoard(StaleMateSession &session)
{
  GtkWidget         *wBoard = session.m_gui.wIconicChessBoard;
  ChessBoardState_T *pSquare;
  int                nRow, nCol;

  for(nRow=0; nRow<8; ++nRow)
  {
    for(nCol=0; nCol<8; ++nCol)
    {
      pSquare = &ChessBoard[nRow][nCol];

      gtk_container_remove(GTK_CONTAINER(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(pSquare->m_ePiece, pSquare->m_nInstance));

      pSquare->m_ePiece     = NoPiece;
      pSquare->m_eColor     = NoColor;
      pSquare->m_nInstance  = ChessGuiNoPieceInstance(nRow, nCol);

      gtk_box_pack_start(GTK_BOX(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(NoPiece, pSquare->m_nInstance),
            TRUE, TRUE, 0);
    }
  }

  gtk_widget_show_all(wBoard);
}

static void ChessGuiClearBoard(StaleMateSession &session)
{
  GtkWidget         *wBoard = session.m_gui.wIconicChessBoard;
  ChessBoardState_T *pSquare;
  int                nRow, nCol;

  for(nRow=0; nRow<8; ++nRow)
  {
    for(nCol=0; nCol<8; ++nCol)
    {
      pSquare = &ChessBoard[nRow][nCol];

      gtk_container_remove(GTK_CONTAINER(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(pSquare->m_ePiece, pSquare->m_nInstance));

      pSquare->m_ePiece     = NoPiece;
      pSquare->m_eColor     = NoColor;
      pSquare->m_nInstance  = ChessGuiNoPieceInstance(nRow, nCol);

      gtk_box_pack_start(GTK_BOX(pSquare->m_wContainer),
            ChessGuiGetPieceWidget(NoPiece, pSquare->m_nInstance),
            TRUE, TRUE, 0);
    }
  }

  gtk_widget_show_all(wBoard);
}

static void ChessGuiDrawBoardLabel(StaleMateSession &session)
{
  if( session.m_game.bHekHasWhite )
  {
    gtk_label_set_text(GTK_LABEL(session.m_gui.wBlackLabel), "Black: Opponent");
    gtk_label_set_text(GTK_LABEL(session.m_gui.wWhiteLabel),
                                                          "White: Hekateros");
  }
  else
  {
    gtk_label_set_text(GTK_LABEL(session.m_gui.wBlackLabel),
                                                          "Black: Hekateros");
    gtk_label_set_text(GTK_LABEL(session.m_gui.wWhiteLabel), "White: Opponent");
  }
}

static void ChessGuiClearBoardLabel(StaleMateSession &session)
{
  gtk_label_set_text(GTK_LABEL(session.m_gui.wBlackLabel), "Black");
  gtk_label_set_text(GTK_LABEL(session.m_gui.wWhiteLabel), "White");
}

static void ChessInitEmptyBoard(StaleMateSession &session)
{
  GtkAttachOptions  optBoard = (GtkAttachOptions)(GTK_EXPAND); // | GTK_FILL);

  GtkWidget  *wBoard = session.m_gui.wIconicChessBoard;
  GdkColor    colorRed;
  GdkColor    colorWhite;
  GtkWidget  *wEventBox;
  GtkWidget  *wBox;
  int         nRow, nCol, k;

  gdk_color_parse(GuiStrColorRed, &colorRed);
  gdk_color_parse(GuiStrColorWhite, &colorWhite);

  for(nRow=0; nRow<8; ++nRow)
  {
    for(nCol=0; nCol<8; ++nCol)
    {
      wEventBox = gtk_event_box_new();
      gtk_event_box_set_above_child(GTK_EVENT_BOX(wEventBox), TRUE);
      gtk_table_attach(GTK_TABLE(wBoard), wEventBox,
                      nCol, nCol+1, nRow, nRow+1, optBoard, optBoard, 0, 0);

      wBox = gtk_hbox_new(FALSE, 0);
      gtk_container_add(GTK_CONTAINER(wEventBox), wBox);

      gtk_widget_set_size_request(wBox, 32, 32);

      k = (nRow + nCol) % 2;
      if( k == 0 )
      {
        gtk_widget_modify_bg(wEventBox, GTK_STATE_NORMAL, &colorWhite);
      }
      else
      {
        gtk_widget_modify_bg(wEventBox, GTK_STATE_NORMAL, &colorRed);
      }

      ChessBoard[nRow][nCol].m_ePiece     = NoPiece;
      ChessBoard[nRow][nCol].m_eColor     = NoColor;
      ChessBoard[nRow][nCol].m_wContainer = wBox;
      ChessBoard[nRow][nCol].m_nInstance  = ChessGuiNoPieceInstance(nRow, nCol);

      gtk_box_pack_start(GTK_BOX(ChessBoard[nRow][nCol].m_wContainer),
            ChessGuiGetPieceWidget(NoPiece, ChessBoard[nRow][nCol].m_nInstance),
            TRUE, TRUE, 0);
      ++k;
    }
  }

  gtk_widget_show_all(wBoard);
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

void StaleMateChessOneTimeInit(StaleMateSession &session)
{
  char        iconPath[MAX_PATH];
  GtkWidget  *wImg;
  int         i, j;

  //
  // Real Pieces
  //
  for(i=0; i<ChessPieceTypeNumOf; ++i)
  {
    snprintf(iconPath, sizeof(iconPath), "%s/chess/%s",
        HekImgDir, ChessPieceTbl[i].m_sFileName);
    iconPath[sizeof(iconPath)-1] = 0;

    for(j=0; j<ChessPieceTbl[i].m_nNumInstances; ++j)
    {
      ChessPieceTbl[i].m_wImg[j] = gtk_image_new_from_file(iconPath);
      gtk_widget_ref(ChessPieceTbl[i].m_wImg[j]); // protect widget
    }
  }

  //
  // No Piece Pieces
  //
  snprintf(iconPath, sizeof(iconPath), "%s/chess/%s",
        HekImgDir, ChessPieceTbl[NoPiece].m_sFileName);
    iconPath[sizeof(iconPath)-1] = 0;

  for(i=0; i<64; ++i)
  {
    NoPieceWidget[i] = gtk_image_new_from_file(iconPath);
    gtk_widget_ref(NoPieceWidget[i]); // protect widget
  }

  ChessInitEmptyBoard(session);
}

const char *StaleMateChessBoardCoordStr(int nRow, int nCol)
{
  TRY_SQUARE(8, nRow, nCol, NULL);
  return BoardCoordStr[nRow][nCol];
}

const char *StaleMateChessPieceLongName(ChessPieceType eChessPiece)
{
  if( (eChessPiece >= 0) && (eChessPiece < ChessPieceTypeNumOf) )
  {
    return ChessPieceTbl[eChessPiece].m_sLongName;
  }
  return "unknown";
}

const char *StaleMateChessPieceAlgName(ChessPieceType eChessPiece)
{
  if( (eChessPiece >= 0) && (eChessPiece < ChessPieceTypeNumOf) )
  {
    return ChessPieceTbl[eChessPiece].m_sAlgName;
  }
  return "unknown";
}

void StaleMateChessNewGame(StaleMateSession &session)
{
  int   nCol;

  //
  // Clear board
  //
  ChessGuiClearBoard(session);

  //
  // Black
  //
  ChessGuiReplacePiece(session, 0, 0, BlackRook,    BlackPiece, 0);
  ChessGuiReplacePiece(session, 0, 1, BlackKnight,  BlackPiece, 0);
  ChessGuiReplacePiece(session, 0, 2, BlackBishop,  BlackPiece, 0);
  ChessGuiReplacePiece(session, 0, 3, BlackQueen,   BlackPiece, 0);
  ChessGuiReplacePiece(session, 0, 4, BlackKing,    BlackPiece, 0);
  ChessGuiReplacePiece(session, 0, 5, BlackBishop,  BlackPiece, 1);
  ChessGuiReplacePiece(session, 0, 6, BlackKnight,  BlackPiece, 1);
  ChessGuiReplacePiece(session, 0, 7, BlackRook,    BlackPiece, 1);

  for(nCol=0; nCol<8; ++nCol)
  {
    ChessGuiReplacePiece(session, 1, nCol, BlackPawn, BlackPiece, nCol);
  }

  //
  // White
  //
  ChessGuiReplacePiece(session, 7, 0, WhiteRook,    WhitePiece, 0);
  ChessGuiReplacePiece(session, 7, 1, WhiteKnight,  WhitePiece, 0);
  ChessGuiReplacePiece(session, 7, 2, WhiteBishop,  WhitePiece, 0);
  ChessGuiReplacePiece(session, 7, 3, WhiteQueen,   WhitePiece, 0);
  ChessGuiReplacePiece(session, 7, 4, WhiteKing,    WhitePiece, 0);
  ChessGuiReplacePiece(session, 7, 5, WhiteBishop,  WhitePiece, 1);
  ChessGuiReplacePiece(session, 7, 6, WhiteKnight,  WhitePiece, 1);
  ChessGuiReplacePiece(session, 7, 7, WhiteRook,    WhitePiece, 1);

  for(nCol=0; nCol<8; ++nCol)
  {
    ChessGuiReplacePiece(session, 6, nCol, WhitePawn, WhitePiece, nCol);
  }

  ChessGuiDrawBoardLabel(session);

  ChessWhiteCastled = false;
  ChessBlackCastled = false;

  StaleMateChessHistClear(session);
  StaleMateChessHistAdd(session, "  New Game - Hekateros is %s\n",
              (session.m_game.bHekHasWhite? "White": "Black"));
}

void StaleMateChessNoGame(StaleMateSession &session)
{
  StaleMateChessNewGame(session);
  ChessGuiClearBoardLabel(session);
}

bool StaleMateChessIsCastleMove(StaleMateSession &session,
                                const char       *sAlgSqFrom,
                                const char       *sAlgSqTo)
{
  ChessPieceType  ePiece;

  ePiece = StaleMateChessGetPieceType(session, sAlgSqFrom);

  if( ePiece == WhiteKing )
  {
    if( ChessWhiteCastled )
    {
      return false;
    }
    else if( strncmp(sAlgSqFrom, "e1", 2) )
    {
      return false;
    }
    else if( strncmp(sAlgSqTo, "c1", 2) && strncmp(sAlgSqTo, "g1", 2) )
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else if( ePiece == BlackKing )
  {
    if( ChessBlackCastled )
    {
      return false;
    }
    else if( strncmp(sAlgSqFrom, "e8", 2) )
    {
      return false;
    }
    else if( strncmp(sAlgSqTo, "c8", 2) && strncmp(sAlgSqTo, "g8", 2) )
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
}

void StaleMateChessCastleRook(StaleMateSession &session,
                              char             *sAlgSqFrom,
                              char             *sAlgSqTo)
{
  if( !strncmp(sAlgSqTo, "c1", 2) )
  {
    strcpy(sAlgSqFrom, "a1");
    strcpy(sAlgSqTo, "d1");
  }
  else if( !strncmp(sAlgSqTo, "g1", 2) )
  {
    strcpy(sAlgSqFrom, "h1");
    strcpy(sAlgSqTo, "f1");
  }
  else if( !strncmp(sAlgSqTo, "c8", 2) )
  {
    strcpy(sAlgSqFrom, "a8");
    strcpy(sAlgSqTo, "d8");
  }
  else if( !strncmp(sAlgSqTo, "g8", 2) )
  {
    strcpy(sAlgSqFrom, "h8");
    strcpy(sAlgSqTo, "f8");
  }
}

void StaleMateChessCastled(StaleMateSession &session,
                           ChessPieceColor  eChessSide)
{
  char    bufText[64];

  if( eChessSide == WhitePiece )
  {
    ChessWhiteCastled = true;
    sprintf(bufText, "%s: Castled\n", StaleMateChessPieceLongName(WhiteKing));
  }
  else if( eChessSide == BlackPiece )
  {
    ChessBlackCastled = true;
    sprintf(bufText, "%s: Castled\n", StaleMateChessPieceLongName(BlackKing));
  }
  else
  {
    sprintf(bufText, "%s", "???\n");
  }
  StaleMateChessHistAdd(session, bufText);
}

void StaleMateChessMovePiece(StaleMateSession &session,
                             int nFromRow, int nFromCol,
                             int nToRow,   int nToCol)
{
  ChessBoardState_T  *pFromS, *pToS;
  ChessPieceType      eFromPiece;
  ChessPieceColor     eFromColor;
  int                 nFromInstance;
  const char         *sAlgFromSq, *sAlgToSq;
  const char         *sAlgPieceMove,  *sAlgPieceTaken;
  const char         *sNamePieceMove, *sNamePieceTaken;
  char                bufText[256];

  LOGDIAG3("Iconic Chess Move: (%d,%d) -> (%d,%d).",
      nFromRow, nFromCol, nToRow, nToCol);

  TRY_SQUARE(session.m_game.nChessBoardDim, nFromRow, nFromCol, );
  TRY_SQUARE(session.m_game.nChessBoardDim, nToRow, nToCol, );

  pFromS = &ChessBoard[nFromRow][nFromCol];
  pToS   = &ChessBoard[nToRow][nToCol];

  // save 'from' data
  eFromPiece    = pFromS->m_ePiece;
  eFromColor    = pFromS->m_eColor;
  nFromInstance = pFromS->m_nInstance;

  // algebraic form strings
  sAlgFromSq = StaleMateChessBoardCoordStr(nFromRow, nFromCol);
  sAlgToSq   = StaleMateChessBoardCoordStr(nToRow, nToCol);

  if( eFromPiece == NoPiece )
  {
    LOGERROR("No piece found at %s.", sAlgFromSq);
    return;
  }

  sAlgPieceMove   = StaleMateChessPieceAlgName(eFromPiece);
  sNamePieceMove  = StaleMateChessPieceLongName(eFromPiece);

  if( pToS->m_ePiece == NoPiece )
  {
    sAlgPieceTaken == "";
    sprintf(bufText, "%s-%s: %s%s  %s\n",
        sAlgFromSq, sAlgToSq, sAlgPieceMove, sAlgToSq, sNamePieceMove); 
  }
  else
  {
    sAlgPieceTaken  = StaleMateChessPieceAlgName(pToS->m_ePiece);
    sNamePieceTaken = StaleMateChessPieceLongName(pToS->m_ePiece);
    sprintf(bufText, "%s-%s: %sx%s  %s x %s\n",
        sAlgFromSq, sAlgToSq, sAlgPieceMove, sAlgToSq,
        sNamePieceMove, sNamePieceTaken); 
  }

  ChessGuiReplacePiece(session, nFromRow, nFromCol, NoPiece, NoColor,
                      ChessGuiNoPieceInstance(nFromRow, nFromCol));

  ChessGuiReplacePiece(session, nToRow, nToCol, eFromPiece, eFromColor,
                              nFromInstance);

  StaleMateChessHistAdd(session, bufText);
}

void StaleMateChessRemovePiece(StaleMateSession &session, int nRow, int nCol)
{
  LOGDIAG3("Iconic Chess Remove: (%d,%d).", nRow, nCol);

  TRY_SQUARE(session.m_game.nChessBoardDim, nRow, nCol, );

  ChessGuiReplacePiece(session, nRow, nCol, NoPiece, NoColor,
                      ChessGuiNoPieceInstance(nRow, nCol));
}

bool StaleMateChessIsEmptySquare(StaleMateSession &session, int nRow, int nCol)
{
  TRY_SQUARE(session.m_game.nChessBoardDim, nRow, nCol, false);

  return ChessBoard[nRow][nCol].m_ePiece == NoPiece? true: false;
}

ChessPieceType StaleMateChessGetPieceType(StaleMateSession &session,
                                          int nRow, int nCol)
{
  TRY_SQUARE(session.m_game.nChessBoardDim, nRow, nCol, NoPiece);

  return ChessBoard[nRow][nCol].m_ePiece;
}

ChessPieceColor StaleMateChessGetPieceColor(StaleMateSession &session,
                                            int nRow, int nCol)
{
  TRY_SQUARE(session.m_game.nChessBoardDim, nRow, nCol, NoColor);

  return ChessBoard[nRow][nCol].m_eColor;
}

void StaleMateChessHistClear(StaleMateSession &session)
{
  gtk_text_buffer_set_text(session.m_gui.wBufHistory, "", -1);
  gtk_widget_show(session.m_gui.wViewHistory);
  session.m_gui.pWin->Wait(1);
}

void StaleMateChessHistAdd(StaleMateSession &session, const char *sFmt, ...)
{
  char        buf[80];
  va_list     ap;  
  GtkTextIter iter;

  // format history entry text
  va_start(ap, sFmt);
  vsnprintf(buf, sizeof(buf), sFmt, ap);
  buf[sizeof(buf)-1] = 0;
  va_end(ap);

  gtk_text_buffer_get_end_iter(session.m_gui.wBufHistory, &iter);
  gtk_text_buffer_insert(session.m_gui.wBufHistory, &iter, buf, -1);
  gtk_text_view_scroll_to_iter(GTK_TEXT_VIEW(session.m_gui.wViewHistory),
                    &iter, 0.0, FALSE, 0, 0);
  gtk_widget_show(session.m_gui.wViewHistory);
  session.m_gui.pWin->Wait(1);
}
