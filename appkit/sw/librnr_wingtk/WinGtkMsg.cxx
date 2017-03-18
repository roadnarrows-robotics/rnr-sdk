////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_wingtk
//
// File:      WinGtkMsg.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief GTK dialog and status windowing functions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#include <sys/types.h>
#include <stdarg.h>
#include <libgen.h>
#include <stdlib.h>
#include <unistd.h>

#undef IT_TIMER   ///< use gtk timer instead

#ifdef IT_TIMER 
#include <signal.h>
#include <time.h>
#endif // IT_TIMER 

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv2/core/core.hpp"
#include "opencv/highgui/highgui.hpp"

#include <gtk/gtk.h>

#include "rnr/appkit/Win.h"


using namespace std;
using namespace rnrWin;


// ----------------------------------------------------------------------------
//  Private Interface
// ----------------------------------------------------------------------------

static GtkWidget   *SBWidget      = NULL;   ///< status bar gtk widget
static bool         SBIsVisible   = false;  ///< status bar is [not] visible
static GtkWindow   *SBWindow      = NULL;   ///< status bar gtk window
static GtkWidget   *SBLabelWidget = NULL;   ///< status bar label gtk widget
static CvRect       SBWinGeom     = {0, };  ///< status bar window geometry

#ifdef IT_TIMER 
static timer_t      SBTimerId     = NULL;   ///< status bar timer

#define SB_CLOCKID  CLOCK_REALTIME  ///< status bar clock id 
#define SB_SIG      SIGRTMIN        ///< status bar timer signal

/*!
 * \brief Status Bar timer handler.
 *
 * \param sig   Received signal.
 * \param si    Signal information.
 * \param uc    User data
 */
static void SBTimerHandler(int sig, siginfo_t *si, void *uc)
{
  StatusClear();
}

/*!
 * \brief Create Status Bar timer.
 */
static void SBTimerCreate()
{
  struct sigevent   sev;
  long long         freq_nanosecs;
  sigset_t          mask;
  struct sigaction  sa;

  // establish handler for timer signal
  sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = SBTimerHandler;
  sigemptyset(&sa.sa_mask);
  if( sigaction(SB_SIG, &sa, NULL) == -1 )
  {
    LOGSYSERROR("Cannot set Status Bar signal action: sigaction()");
  }

  // create the timer
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SB_SIG;
  sev.sigev_value.sival_ptr = &SBTimerId;
  if( timer_create(SB_CLOCKID, &sev, &SBTimerId) == -1 )
  {
    LOGSYSERROR("Cannot create Status Bar timer: timer_create()");
  }

  // unblock signal
  if( sigprocmask(SIG_UNBLOCK, &mask, NULL) == -1 )
  {
    LOGSYSERROR("Cannot unblock Status Bar signal: sigprocmask()");
  }
}

/*!
 * \brief Start the Status Bar timer.
 *
 * \param uMSec   Timeout in milliseconds.
 */
static void SBTimerStart(uint_t uMSec)
{
  struct itimerspec its;

  if( SBTimerId != NULL )
  {
    // first timeout
    its.it_value.tv_sec = uMSec / 1000;
    its.it_value.tv_nsec = (uMSec % 1000) * 1000000;

    // no repeat interval timer 
    its.it_interval.tv_sec = 0; //its.it_value.tv_sec;
    its.it_interval.tv_nsec = 0; //its.it_value.tv_nsec;

    if( timer_settime(SBTimerId, 0, &its, NULL) == -1 )
    {
      LOGSYSERROR("Cannot set Status Bar timer: timer_settime()");
    }
  }
}

/*!
 * \brief Stop the Status Bar timer.
 */
static void SBTimerStop()
{
  struct itimerspec its;

  if( SBTimerId != NULL )
  {
    // disarm
    its.it_value.tv_sec = 0;
    its.it_value.tv_nsec = 0;

    // no repeat interval timer 
    its.it_interval.tv_sec = 0; //its.it_value.tv_sec;
    its.it_interval.tv_nsec = 0; //its.it_value.tv_nsec;

    if( timer_settime(SBTimerId, 0, &its, NULL) == -1 )
    {
      LOGSYSERROR("Cannot disarm Status Bar timer: timer_settime()");
    }
  }
}

#else // GTK timeout method

guint SBTimerId = 0;  ///< statue bar timer

/*
 * \brief Status bar timer timeout handler.
 *
 * Hides the status bar.
 *
 * \param user_data   Application specific data
 *
 * \return FALSE
 */
static gboolean SBTimerAlarm(gpointer user_data)
{
  SBTimerId = 0;
  StatusClear();
  return FALSE;
}

/*!
 * \brief Start the Status Bar timer.
 *
 * \param uMSec   Timeout in milliseconds.
 */
static void SBTimerStart(uint_t uMSec)
{
  SBTimerId = g_timeout_add(uMSec, SBTimerAlarm, NULL);
}

/*!
 * \brief Stop the Status Bar timer.
 */
static void SBTimerStop()
{
  if( SBTimerId > 0 )
  {
    g_source_remove(SBTimerId);
    SBTimerId = 0;
  }
}

#endif // IT_TIMER 

/*!
 * \brief Capture frame event for status bar.
 *
 * \note Cannot seem to get this to work.
 *
 * \param window    GTK window.
 * \param event     The event (e.g. mouse click)
 * \param data      Application data.
 */
void SBFrameEvent(GtkWindow *window, GdkEvent *event, gpointer data)
{
  char *s = (char *)data;
  //cerr << "dbg: " << event << ", " << s << endl;
}

/*!
 * \brief Window state change event handler.
 *
 * \param w         Widget where keyboard event occurred.
 * \param event     State change event.
 * \param user_data Supplied user data.
 *
 * \return Returns FALSE.
 */
gboolean SBOnWindowState(GtkWidget   *w,
                         GdkEventKey *event,
                         gpointer    *user_data)
{
  //cerr << "dbg: " << event << endl;
}

/*!
 * \brief Create status bar.
 *
 * \param pWin      Application window.
 */
static void CreateStatusBar(rnrWindow *pWin)
{
  static int  h_status = 20;  // status window height

  GtkWindow  *wWindow;    // gtk window
  GdkColor    color;      // background color
  int         x_root;     // parent screen x position
  int         y_root;     // parent screen y position
  int         w_root;     // parent width
  int         h_root;     // parent height

  // get parent position and size
  wWindow = pWin->GetMainWindowWidget();
  gtk_window_get_position(wWindow, &x_root, &y_root);
  gtk_window_get_size(wWindow, &w_root, &h_root);

  // background color
  gdk_color_parse("#ffffcf", &color);

  // status bar window
  SBWidget          = gtk_window_new(GTK_WINDOW_POPUP);
  SBWindow          = GTK_WINDOW(SBWidget);
  SBWinGeom.x       = x_root;
  SBWinGeom.y       = y_root + h_root - h_status;
  SBWinGeom.width   = w_root;
  SBWinGeom.height  = h_status;

  gtk_window_set_keep_above(SBWindow, true);
  gtk_window_resize(SBWindow, SBWinGeom.width, SBWinGeom.height);
  gtk_window_move(SBWindow, SBWinGeom.x, SBWinGeom.y);
  //gtk_window_set_resizable(SBWindow, false); // cannot use
  gtk_window_set_decorated(SBWindow, false);
  gtk_widget_modify_bg(SBWidget, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_base(SBWidget, GTK_STATE_NORMAL, &color);

  // status bar label widget
  SBLabelWidget = gtk_label_new(NULL);
  gtk_container_add(GTK_CONTAINER(SBWindow), SBLabelWidget);

  //g_signal_connect(G_OBJECT(SBLabelWidget), "frame-event", 
  //    G_CALLBACK(SBFrameEvent), (void *)"frame-event");
 
  gtk_signal_connect(GTK_OBJECT(SBWidget), "window-state-event",
                        GTK_SIGNAL_FUNC(SBOnWindowState), NULL);

  gtk_widget_add_events(SBWidget, GDK_EXPOSURE_MASK |
                                  GDK_BUTTON_RELEASE_MASK |
                                  GDK_BUTTON_PRESS_MASK |
                                  GDK_POINTER_MOTION_MASK);
}


// ----------------------------------------------------------------------------
//  Public Interface
// ----------------------------------------------------------------------------

/*!
 * \brief Error dialog box.
 *
 * \param sWinName  Application window name (and id).
 * \param sMsg      Dialog message.
 */
void rnrWin::MsgBox(const char *sWinName, const char *sMsg)
{
  GtkWidget  *widget;
  GtkWindow  *window;
  GtkWidget  *wDialog;
  GtkStyle    style;
  GdkColor    color;

  gdk_color_parse("#ffffcf", &color);

  widget = (GtkWidget *)cvGetWindowHandle(sWinName);
  window = GTK_WINDOW(gtk_widget_get_toplevel(widget));

  wDialog = gtk_message_dialog_new(
              window,
              GTK_DIALOG_DESTROY_WITH_PARENT,
              GTK_MESSAGE_ERROR,
              GTK_BUTTONS_OK,
              "%s", sMsg);

  gtk_widget_modify_bg(wDialog, GTK_STATE_NORMAL, &color);
  gtk_widget_modify_base(wDialog, GTK_STATE_NORMAL, &color);

  gtk_dialog_run( GTK_DIALOG(wDialog) );
  gtk_widget_destroy(wDialog);
}

/*!
 * \brief Show status message.
 *
 * \param pWin      Application window.
 * \param uMSec     If zero, the message requires a mouse click to clear. 
 *                  Others, the status auto-clears in the give milliseconds.
 * \param sFmt      Format string.
 * \param ...       Variable arguments to format string.
 */
void rnrWin::StatusShow(rnrWindow   *pWin,
                        uint_t      uMSec,
                        const char *sFmt,
                        ...)
{
  static int  h_status = 20;  // status window height

  char        buf[256];   // working buffer
  va_list     ap;         // variable argument pointer

  // create status bar
  if( SBWidget == NULL )
  {
    CreateStatusBar(pWin);
  }
 
#ifdef IT_TIMER 
  // create status bar timer
  if( SBTimerId == NULL )
  {
    SBTimerCreate();
  }
#endif // IT_TIMER 

  // stop any timer
  SBTimerStop();

  // format message
  va_start(ap, sFmt);
  vsnprintf(buf, sizeof(buf), sFmt, ap);
  va_end(ap);
  buf[sizeof(buf)-1] = 0;

  //cerr << "dbg: " << uMSec << " " << buf << endl;

  // set text
  gtk_label_set_text(GTK_LABEL(SBLabelWidget), buf);

  // reshow window
  gtk_window_move(SBWindow, SBWinGeom.x, SBWinGeom.y);
  gtk_window_resize(SBWindow, SBWinGeom.width, SBWinGeom.height);
  gtk_widget_show(SBLabelWidget);
  gtk_widget_show(SBWidget);
	gtk_widget_queue_draw( GTK_WIDGET(SBLabelWidget) );
  SBIsVisible = true;
  pWin->WaitKey(100); // give time to draw

  if( uMSec > 0 )
  {
    SBTimerStart(uMSec);
  }
}

/*!
 * \brief Clear status message.
 */
void rnrWin::StatusClear()
{
  SBTimerStop();

  if( (SBWidget != NULL) && SBIsVisible )
  {
    gtk_widget_hide(SBWidget);
    SBIsVisible = false;
  }
}
