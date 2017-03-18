////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Unit Test: Test Camera Class.
//  
/*! \file
 *  
 * $LastChangedDate: 2013-07-13 14:13:21 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3127 $
 *
 *  \ingroup appkit_ut
 *  
 *  \brief Unit test for librnr_appkit Camera classes.
 *
 *  \author Robin Knight (robin.knight@roadnarrows.com)
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

#include <stdio.h>
#include <ctype.h>

#include <iostream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/CameraCv.h"
#include "rnr/CameraGst.h"

#include "gtest/gtest.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <gdk/gdkx.h>

using namespace ::std;
using namespace ::rnr;
using namespace ::cv;

#define KEY_L_SHIFT     0xe1       ///< left shift key code
#define KEY_R_SHIFT     0xe2       ///< right shift key code
#define KEY_L_CTRL      0xe3       ///< left ctrl key code
#define KEY_R_CTRL      0xe4       ///< right ctrl key code
#define KEY_L_ALT       0xe9       ///< left alt key code
#define KEY_R_ALT       0xea       ///< right alt key code


/*!
 *  \ingroup appkit_ut
 *  \defgroup appkit_ut_camera Camera Unit Tests
 *  \brief Fine-grained testing of Camera classes.
 *  \{
 */

static const char *TestMenu = 
  "\nTest Menu\n"
  "  'h'     print help\n"
  "  'q'     quit with success\n"
  "  's'     start video\n"
  "  'p'     stop video\n"
  "  'c'     click snapshot\n"
  "  '+/-'   increase/decrease video resolution\n"
  "  '>/<'   increase/decrease image resolution\n";

/*!
 * \brief Increment resolution to the next comman resolution.
 *
 * \param resCur    Current resolution.
 *
 * \param Returns next higher resolution.
 */
static CamRes incRes(const CamRes &resCur)
{
  CamRes resNew;

  switch( resCur.width )
  {
    case 320:
      resNew = CamResVGA;
      break;
    case 640:
      resNew = CamRes1024x768;
      break;
    case 1024:
      resNew = CamRes1440x1080;
      break;
    case 1440:
      resNew = CamRes1600x1200;
      break;
    case 1600:
      resNew = CamRes2048x1536;
      break;
    case 2048:
      resNew = CamRes2592x1944;
      break;
    default:
      resNew = resCur;
      break;
  }

  return resNew;
}

/*!
 * \brief Decrement resolution to the previous comman resolution.
 *
 * \param resCur    Current resolution.
 *
 * \param Returns previous lower resolution.
 */
static CamRes decRes(const CamRes &resCur)
{
  CamRes resNew;

  switch( resCur.width )
  {
    case 2592:
      resNew = CamRes2048x1536;
      break;
    case 2048:
      resNew = CamRes1600x1200;
      break;
    case 1600:
      resNew = CamRes1440x1080;
      break;
    case 1440:
      resNew = CamRes1024x768;
      break;
    case 1024:
      resNew = CamResVGA;
      break;
    case 640:
      resNew = CamResQVGA;
      break;
    default:
      resNew = resCur;
      break;
  }

  return resNew;
}


// .............................................................................
// OpenCv Camera Test
// .............................................................................

/*!
 *  \brief Test OpenCv CameraCv class.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testCameraCv()
{
  string  strVideoWinName("ut-CameraCv - Video");
  string  strImageWinName("ut-CameraCv - Image");
  CamRes  resVideo = CamResVGA;
  CamRes  resImage = CamResVGA;
  Mat     frame;
  Mat     snapshot;
  bool    bRun = true;
  int     c;
  int     rc = 0;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  CameraCv  cam;

  if( cam.isFatal() )
  {
    return -1;
  }

  printf("%s", TestMenu);

  namedWindow(strVideoWinName);
  
  while( bRun )
  {
    c = waitKey(10);

    // strip off shift bit
    if( c != -1 )
    {
      c &= 0x00ff;
    }

    switch( c )
    {
      case 'h':
        printf("%s", TestMenu);
        break;

      case 'q':
        rc = 0;
        bRun = false;
        break;

      case 's':
        if( cam.startVideo(resVideo) < 0 )
        {
          rc = -1;
          bRun = false;
        }
        break;

      case 'p':
        if( cam.stopVideo() < 0 )
        {
          rc = -1;
          bRun = false;
        }
        break;

      case 'c':
        if( cam.clickImage(snapshot, resImage) == OK )
        {
          namedWindow(strImageWinName);
          imshow(strImageWinName, snapshot);
        }
        break;

      case '+':
        resVideo = incRes(resVideo);
        printf("%dx%d video resolution.\n", resVideo.width, resVideo.height);
        if( cam.isCameraRunning() )
        {
          cam.startVideo(resVideo);
        }
        break;

      case '-':
        resVideo = decRes(resVideo);
        printf("%dx%d video resolution.\n", resVideo.width, resVideo.height);
        if( cam.isCameraRunning() )
        {
          cam.startVideo(resVideo);
        }
        break;

      case '>':
        resImage = incRes(resImage);
        printf("%dx%d image resolution.\n", resImage.width, resImage.height);
        break;

      case '<':
        resImage = decRes(resImage);
        printf("%dx%d image resolution.\n", resImage.width, resImage.height);
        break;

      case -1:
        if( cam.isCameraRunning() && !cam.isTakingAnImage() )
        {
          if( cam.grabFrame(frame) == OK )
          {
            imshow(strVideoWinName, frame);
          }
        }
        break;

      case KEY_L_SHIFT:
      case KEY_R_SHIFT:
      case KEY_L_CTRL:
      case KEY_R_CTRL:
      case KEY_L_ALT:
      case KEY_R_ALT:
        break;

      default:
        printf("0x%02x huh?\n", c);
        break;
    }
  }

  destroyWindow(strVideoWinName);
  destroyWindow(strImageWinName);

  LOG_SET_THRESHOLD(LOG_LEVEL_OFF);

  return rc;
}


// .............................................................................
// GStreamer Camera Test
// .............................................................................

static gulong GstWinXid   = 0;
static uint_t GtkLastKey  = 0;

/*!
 * \brief Realize GStreamer video window callback.
 *
 * Once the window has been realized, the X-Window id can be obtained. The id
 * is critical for renders gst video and images to the gtk widget.
 *
 * \param w         Gtk draw widget where video will be overlaied.
 * \param user_data Supplied user data (this).
 */
void GtkOnRealizeGstWin(GtkWidget *w, gpointer user_data)
{
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
  GstWinXid = GDK_WINDOW_XID(gtk_widget_get_window(w));
#endif
}

/*!
 * \brief Timeout expiry callback.
 *
 * The supplied user data is set to 1 (true).
 *
 * \param user_data   Pointer to user supplied expiry flag.
 *
 * \return Returns FALSE.
 */
gboolean GtkAlarm(gpointer user_data)
{
  *(int *)user_data = 1;
  return FALSE;
}

/*!
 * \brief Wait for keypress or timeout.
 *
 * GTK widgets can be updated during this wait.
 *
 * \param delay   Timeout delay in millseconds. Set to 0 for no timeout.
 *
 * \return Returns code of last key pressed or -1 if timedout.
 */
int GtkWaitKey(int delay)
{
  int expired = 0;
  guint timer = 0;
  uint_t   key;

  if( delay > 0 )
  {
    timer = g_timeout_add(delay, GtkAlarm, &expired);
  }

  while( gtk_main_iteration_do(FALSE) &&
         (GtkLastKey == 0) &&
         !expired );

  if( delay > 0 && !expired )
  {
    g_source_remove(timer);
  }

  key = GtkLastKey;

  GtkLastKey = 0;

  return key;
}

/*!
 * \brief Keyboard press event handler.
 *
 * If registered, the application callback function will be called.
 *
 * \param w         Widget where keyboard event occurred.
 * \param event     Keyboard event.
 * \param user_data Supplied user data (this).
 *
 * \return Returns FALSE.
 */
gboolean GtkOnKeyPress(GtkWidget   *w,
                       GdkEventKey *event,
                       gpointer    *user_data)
{
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
  }

  GtkLastKey = (code & 0xffff) | (event->state << 16);

  return FALSE;
}


/*!
 *  \brief Test GStreamer CameraGst class.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testCameraGst()
{
  static const char *TestMenuAddOn = "  't'     text overlay\n";

  string      strVideoWinName("ut-CameraGst - Video");
  string      strImageWinName("ut-CameraGst - Image");
  CamRes      resVideo = CamResVGA;
  CamRes      resImage = CamResVGA;
  Mat         frame;
  Mat         snapshot;

  GtkWidget  *wMain;
  GtkWindow  *wWin;
  GtkWidget  *wGst;
  gint        nIdRealize;
  gint        nIdKeyPress;

  bool        bRun = true;
  bool        bEnterText = false;
  int         c;
  char        bufText[256];
  int         n;
  int         rc = 0;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  CameraGst   cam;

  if( cam.isFatal() )
  {
    return -1;
  }

  printf("%s", TestMenu);
  printf("%s", TestMenuAddOn);

  gtk_init(0, NULL);

  //
  // Create a new gtk window
  //
  wMain = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  wWin  = GTK_WINDOW(wMain);
  
  //
  // Configure window's look and feel
  //
  gtk_window_set_title(wWin, strVideoWinName.c_str());
  gtk_window_resize(wWin, 640, 480);
  gtk_window_set_decorated(wWin, TRUE);
  
  //---
  // Video widget
  //---
  wGst = gtk_drawing_area_new();
  g_object_set(G_OBJECT(wGst),
      "width-request",  640,
      "height-request", 480,
      NULL);
  nIdRealize = g_signal_connect(wGst, "realize",
                  G_CALLBACK(GtkOnRealizeGstWin), NULL);
  gtk_widget_set_double_buffered(wGst, FALSE);
  gtk_container_add(GTK_CONTAINER(wMain), wGst);

  //
  // Event handlers
  //
  nIdKeyPress = gtk_signal_connect(GTK_OBJECT(wMain), "key-press-event",
                            GTK_SIGNAL_FUNC(GtkOnKeyPress), NULL);

  gtk_widget_show_all(wMain);

  cam.setXid(GstWinXid);

  while( bRun )
  {
    c = GtkWaitKey(30);

    // strip off shift bit
    if( c != -1 )
    {
      c &= 0x00ff;
    }

    if( bEnterText )
    {
      switch( c )
      {
        case '\n':
        case '\r':
          printf("\n");
          fflush(stdout);
          bufText[n] = 0;
          if( n > 0 )
          {
            cam.setTextOverlay(bufText);
          }
          else
          {
            cam.clearTextOverlay();
          }
          bEnterText = false;
          break;
        default:
          if( isprint(c) )
          {
            bufText[n++] = c;
            printf("%c", c);
            fflush(stdout);
          }
          break;
      }
      continue;
    }

    switch( c )
    {
      case 'h':
        printf("%s", TestMenu);
        printf("%s", TestMenuAddOn);
        break;

      case 'q':
        rc = 0;
        bRun = false;
        break;

      case 's':
        if( cam.startVideo(resVideo) < 0 )
        {
          rc = -1;
          bRun = false;
        }
        break;

      case 'p':
        if( cam.stopVideo() < 0 )
        {
          rc = -1;
          bRun = false;
        }
        break;

      case 'c':
        if( cam.clickImage(snapshot, resImage) == OK )
        {
          namedWindow(strImageWinName);
          imshow(strImageWinName, snapshot);
        }
        break;

      case '+':
        resVideo = incRes(resVideo);
        printf("%dx%d video resolution.\n", resVideo.width, resVideo.height);
        if( cam.isCameraRunning() )
        {
          cam.startVideo(resVideo);
        }
        break;

      case '-':
        resVideo = decRes(resVideo);
        printf("%dx%d video resolution.\n", resVideo.width, resVideo.height);
        if( cam.isCameraRunning() )
        {
          cam.startVideo(resVideo);
        }
        break;

      case '>':
        resImage = incRes(resImage);
        printf("%dx%d image resolution.\n", resImage.width, resImage.height);
        break;

      case '<':
        resImage = decRes(resImage);
        printf("%dx%d image resolution.\n", resImage.width, resImage.height);
        break;

      case 't':
        bEnterText = true;
        n = 0;
        printf("Text: ");
        fflush(stdout);
        break;

      case 0:
        break;

      case KEY_L_SHIFT:
      case KEY_R_SHIFT:
      case KEY_L_CTRL:
      case KEY_R_CTRL:
      case KEY_L_ALT:
      case KEY_R_ALT:
        break;

      default:
        printf("0x%02x huh?\n", c);
        break;
    }
  }

  cam.stopVideo();

  //gtk_signal_disconnect(GTK_OBJECT(wMain), nIdRealize);
  gtk_signal_disconnect(GTK_OBJECT(wMain), nIdKeyPress);
  gtk_widget_destroy(wMain);
  destroyWindow(strImageWinName);

  gtk_main_iteration_do(FALSE);

  while( gtk_events_pending() )
  {
    gtk_main_iteration_do(FALSE);
  }

  LOG_SET_THRESHOLD(LOG_LEVEL_OFF);

  return rc;
}


#ifndef JENKINS

/*!
 * \brief Test OpenCv camera class.
 *
 * \par The Test:
 * Construct CameraCv object.\n
 * run video.\n
 * take image.\n
 * change resolution.\n
 * Destroy CameraCv object.
 */
TEST(Camera, CameraCv)
{
  EXPECT_TRUE( testCameraCv() == 0 );
}

/*!
 * \brief Test OpenGst camera class.
 *
 * \par The Test:
 * Construct CameraGst object.\n
 * run video.\n
 * take image.\n
 * change resolution.\n
 * overlay text.\n
 * Destroy CameraGst object.
 */
TEST(Camera, CameraGst)
{
  EXPECT_TRUE( testCameraGst() == 0 );
}

#endif // JENKINS


/*!
 *  \}
 */
