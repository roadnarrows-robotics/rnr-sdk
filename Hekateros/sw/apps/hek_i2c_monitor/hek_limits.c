////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   hekLimits
//
// File:      hekLimits.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-20 17:11:12 -0600 (Mon, 20 May 2013) $
 * $Rev: 2983 $
 *
 * \brief  Hekateros library for monitoring limit switches.
 *
 * \author Brent Wilkins (brent@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef __HEKLIMITS_H__
#define __HEKLIMITS_H__

/* ----- Include Files ------------------------------------------------------ */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>      /* fork, etc. */
#include <syslog.h>
#include <sys/types.h>   /* mask etc.  */
#include <sys/stat.h>
#include <sys/select.h>
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c-dev.h"
#include "rnr/i2c.h"
#include "gpio-event-drv.h"

/* ----- Global Definitions ------------------------------------------------- */

#define INT_GPIO_NUM  147
#define DEBOUNCE_TIME 2

#define DEVNAME "/dev/i2c-3"  // Bus 3 is for user stuff on Overo
#define ADDR 0x20             // Address of 16 port IO expander
#define NBYTES 2              // Number of bytes to read

/*******************************************************************************
*
*  main
*
*******************************************************************************/

int main( int argc, char **argv )
{
  /* Process ID and Session ID BHW */
  pid_t pid, sid;
  pid = fork();

  /* Fork off of the parent process */
  if (pid < 0) {
    exit(EXIT_FAILURE);
  }
  if (pid > 0) {
    exit(EXIT_SUCCESS);
  }
  /* Change the file mode mask */
  umask(0);

  /* Open any logs here */
  openlog("hekLimits", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1);
  syslog(LOG_NOTICE, "daemon started by User %d", getuid ());

  /* Create a new SID for the child process */
  sid = setsid();
  if (sid < 0) {
    syslog(LOG_ERR, "Failed to get new session ID");
    exit(EXIT_FAILURE);
  }

  /* Close the standard file descriptors */
  close(STDIN_FILENO);
  close(STDOUT_FILENO);
  close(STDERR_FILENO);

  /* Start of non-daemon code */
  FILE   *fs;
  i2c_t   pI2C;  
  int   stat;
  byte_t* buf = (byte_t*) malloc(NBYTES);  

  /* Open gpio-event-module character device
   * This next if block is a 'quote' from DH's code */
  if (( fs = fopen( "/dev/gpio-event", "r" )) == NULL )
  {
    syslog(LOG_ERR, "Check to make sure gpio_event_drv has been loaded. Unable to open /dev/gpio-event");
    exit(EXIT_FAILURE);
  }

  ioctl( fileno( fs ), GPIO_EVENT_IOCTL_SET_READ_MODE, 1 );

  /* Open I2C bus to read from expander IC */
  if( (stat = i2c_open(&pI2C, DEVNAME)) < 0 )
  {
    syslog(LOG_ERR, "%s: Failed to open device: %d %s\n", DEVNAME, stat,
        strerror(errno));
    exit(EXIT_FAILURE);
  }

  /* Setup an EventMonitor struct to pass */
  GPIO_EventMonitor_t monitor;
  monitor.gpio = INT_GPIO_NUM;
  monitor.onOff = 1;  /* 1 = start monitoring */
  monitor.edgeType = GPIO_EventFallingEdge;
  monitor.debounceMilliSec = DEBOUNCE_TIME;

  if ( ioctl( fileno( fs ), GPIO_EVENT_IOCTL_MONITOR_GPIO, &monitor ) != 0 )
  {
    syslog(LOG_ERR, "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
    perror( "ioctl GPIO_EVENT_IOCTL_MONITOR_GPIO failed" );
  }

  while ( 1 )
  {
    ssize_t numBytes;
    char    argStr[ 60 ];

    argStr[ 0 ] = '\0';

    fd_set  readSet;
    struct  timeval tv;
    int     rc;

    syslog( LOG_NOTICE, "Waiting for data " );
    printf( "Waiting for data " );
    fflush( stdout );

    while ( 1 )
    {
      FD_ZERO( &readSet );
      FD_SET( fileno( fs ), &readSet );

      tv.tv_sec = 1;
      tv.tv_usec = 0;
    
      rc = select( fileno( fs ) + 1, &readSet, NULL, NULL, &tv );
    
      if ( rc == -1 )
      {
        syslog( LOG_ERR, "select on /dev/gpio-event failed" );
        perror( "select failed" );
      }
      else
      if ( rc > 0 )
      {
        /* Data is available */
        break;
      }
      else
      {
        /* debug */
        printf( "." );
        fflush( stdout );
      }
    } /* End inner while(1) */
    printf( "\n" );

    GPIO_Event_t    gpioEvent;

    if (( numBytes = fread( &gpioEvent, 1, sizeof( gpioEvent ), fs )) == sizeof( gpioEvent ))
    {
      snprintf( argStr, sizeof( argStr ), "%2d %c %ld.%06ld",
                gpioEvent.gpio,
                (( gpioEvent.edgeType == GPIO_EventRisingEdge ) ? 'R' : 'F' ),
                gpioEvent.time.tv_sec,
                gpioEvent.time.tv_usec );

      /* Read from address on i2c device */
      if( (stat = i2c_read(&pI2C, ADDR, buf, NBYTES)) < 0 )
      {
        fprintf(stderr,
                "Couldn't read device at address 0x%x: %d %s\n",
                ADDR, stat, strerror(errno));
        return 3; /* ToDo: return something
                     meaningful */
      }

      syslog(LOG_NOTICE, "Read: %d %d\n", buf[0], buf[1]);
    }
    else
    {
      if ( numBytes > 0 )
      {
        fprintf( stderr, "Read unexpected number of bytes: %d, expecting %d\n",
        numBytes, sizeof( gpioEvent ));
      }
    }
  } /* End first while(1) */

  free ( buf );
  fclose ( fs );


} /* end main */

#endif /* __HEKLIMITS_H__ */
