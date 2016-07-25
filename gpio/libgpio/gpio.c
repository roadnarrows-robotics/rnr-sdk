////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Library:   libgpio
//
// File:      gpio.c
//
/*! \file
 *
 * $LastChangedDate: 2015-04-09 15:03:13 -0600 (Thu, 09 Apr 2015) $
 * $Rev: 3916 $
 * 
 * \brief GPIO library definitions using the /sys/class/gpio interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 */
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/gpio.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

static const char *GpioRoot   = "/sys/class/gpio";

/*!
 * \brief Read GPIO current direction.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return
 * On success GPIO_DIR_IN(0) or GPIO_DIR_OUT(1) is returned.\n
 * Otherwise RC_ERROR(-1) is returned.
 */
static int gpioReadDirection(int gpio)
{
  int     fd;
  char    buf[MAX_PATH]; 
  ssize_t n;

  sprintf(buf, "%s/gpio%d/direction", GpioRoot, gpio);

  if( (fd = open(buf, O_RDONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  if( (n = read(fd, buf, sizeof(buf))) < 0 )
  {
    LOGSYSERROR("read(%d, %p, %zu)", fd, buf, sizeof(buf));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  buf[n] = 0;

  if( !strncmp(buf, GPIO_DIR_IN_STR, strlen(GPIO_DIR_IN_STR)) )
  {
    return GPIO_DIR_IN;
  }
  else if( !strncmp(buf, GPIO_DIR_OUT_STR, strlen(GPIO_DIR_OUT_STR)) )
  {
    return GPIO_DIR_OUT;
  }
  else
  {
    LOGERROR("Unknown direction value: \"%s\".", buf);
    return RC_ERROR;
  }
}

/*!
 * \brief Read GPIO current edge trigger.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return
 * On success GPIO_EDGE_NONE(0), GPIO_EDGE_RISING(1),
 * GPIO_EDGE_FALLING(2), or GPIO_EDGE_BOTH(3) is returned.\n
 * Otherwise RC_ERROR(-1) is returned.
 */
static int gpioReadEdge(int gpio)
{
  int     fd;
  char    buf[MAX_PATH]; 
  ssize_t n;

  sprintf(buf, "%s/gpio%d/edge", GpioRoot, gpio);

  if( (fd = open(buf, O_RDONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  if( (n = read(fd, buf, sizeof(buf))) < 0 )
  {
    LOGSYSERROR("read(%d, %p, %zu)", fd, buf, sizeof(buf));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  buf[n] = 0;

  if( !strncmp(buf, GPIO_EDGE_NONE_STR, strlen(GPIO_EDGE_NONE_STR)) )
  {
    return GPIO_EDGE_NONE;
  }
  else if( !strncmp(buf, GPIO_EDGE_RISING_STR, strlen(GPIO_EDGE_RISING_STR)) )
  {
    return GPIO_EDGE_RISING;
  }
  else if( !strncmp(buf, GPIO_EDGE_FALLING_STR, strlen(GPIO_EDGE_FALLING_STR)) )
  {
    return GPIO_EDGE_FALLING;
  }
  else if( !strncmp(buf, GPIO_EDGE_BOTH_STR, strlen(GPIO_EDGE_BOTH_STR)) )
  {
    return GPIO_EDGE_BOTH;
  }
  else
  {
    LOGERROR("Unknown edge value: \"%s\".", buf);
    return RC_ERROR;
  }
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

int gpioExport(int gpio)
{
  int   fd;
  char  buf[MAX_PATH]; 

  sprintf(buf, "%s/export", GpioRoot);

  if( (fd = open(buf, O_WRONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  sprintf(buf, "%d", gpio); 

  if( write(fd, buf, strlen(buf)) < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %zu)", fd, buf, strlen(buf));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  LOGDIAG3("Exported GPIO %d interface.", gpio);

  return OK;
}

int gpioUnexport(int gpio)
{
  int   fd;
  char  buf[MAX_PATH]; 

  sprintf(buf, "%s/unexport", GpioRoot);

  if( (fd = open(buf, O_WRONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  sprintf(buf, "%d", gpio); 

  if( write(fd, buf, strlen(buf)) < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %zu)", fd, buf, strlen(buf));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  LOGDIAG3("Unexported GPIO %d interface.", gpio);

  return OK;
}

int gpioSetDirection(int gpio, int dir)
{
  int         fd;
  char        buf[MAX_PATH]; 
  const char *sDir;

  switch( dir )
  {
    case GPIO_DIR_IN:
      sDir = GPIO_DIR_IN_STR;
      break;
    case GPIO_DIR_OUT:
      sDir = GPIO_DIR_OUT_STR;
      break;
    default:
      LOGERROR("GPIO %d: \"%s\": Invalid direction. Must be one of: %d %d.",
        gpio, dir, GPIO_DIR_IN, GPIO_DIR_OUT);
      return RC_ERROR;
  }

  sprintf(buf, "%s/gpio%d/direction", GpioRoot, gpio);

  if( (fd = open(buf, O_WRONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  if( write(fd, sDir, strlen(sDir)) < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %zu)", fd, sDir, strlen(sDir));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  LOGDIAG3("GPIO %d direction set to %s.", gpio, sDir);

  return OK;
}

int gpioSetEdge(int gpio, int edge)
{
  int         fd;
  char        buf[MAX_PATH]; 
  const char *sEdge;

  switch( edge )
  {
    case GPIO_EDGE_NONE:
      sEdge = GPIO_EDGE_NONE_STR;
      break;
    case GPIO_EDGE_RISING:
      sEdge = GPIO_EDGE_RISING_STR;
      break;
    case GPIO_EDGE_FALLING:
      sEdge = GPIO_EDGE_FALLING_STR;
      break;
    case GPIO_EDGE_BOTH:
      sEdge = GPIO_EDGE_BOTH_STR;
      break;
    default:
      LOGERROR("GPIO %d: \"%s\": Invalid edge. Must be in range [%d-%d].",
        gpio, edge, GPIO_EDGE_NONE, GPIO_EDGE_BOTH);
      return RC_ERROR;
  }

  sprintf(buf, "%s/gpio%d/edge", GpioRoot, gpio);

  if( (fd = open(buf, O_WRONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  if( write(fd, sEdge, strlen(sEdge)) < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %zu)", fd, sEdge, strlen(sEdge));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  LOGDIAG3("GPIO %d edge set to %s.", gpio, sEdge);

  return OK;
}

int gpioSetPull(int gpio, int pull)
{
  LOGERROR("Sysfs GPIO pull operation not supported.");
  return RC_ERROR;
}

int gpioProbe(int gpio, gpio_info_t *p)
{
  int   v;
  int   rc = OK;

  p->gpio     = gpio;
  p->pin      = -1; // gpioExportedToPin(gpio);
  p->dir      = GPIO_DIR_IN;
  p->edge     = GPIO_EDGE_NONE;
  p->pull     = GPIO_PULL_DS;
  p->value    = 0;

  if( (v = gpioReadDirection(gpio)) < 0 )
  {
    rc = RC_ERROR;
  }
  else
  {
    p->dir = v;
  }

  if( (v = gpioReadEdge(gpio)) < 0 )
  {
    rc = RC_ERROR;
  }
  else
  {
    p->edge = v;
  }

  if( (v = gpioRead(gpio)) < 0 )
  {
    rc = RC_ERROR;
  }
  else
  {
    p->value = v;
  }

  return rc;
}

int gpioRead(int gpio)
{
  int   fd;
  char  buf[MAX_PATH]; 
  char  c;

  sprintf(buf, "%s/gpio%d/value", GpioRoot, gpio);

  if( (fd = open(buf, O_RDONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_RDONLY)", buf);
    return RC_ERROR;
  }

  if( read(fd, &c, 1) < 0 )
  {
    LOGSYSERROR("read(%s, ...)", fd);
    close(fd);
    return RC_ERROR;
  }

  LOGDIAG3("Read GPIO %d value %c.", gpio, c);

  return c == '0'? 0: 1;
}

int gpioWrite(int gpio, int value)
{
  int   fd;
  char  buf[MAX_PATH]; 
  char  c;

  sprintf(buf, "%s/gpio%d/value", GpioRoot, gpio);

  if( (fd = open(buf, O_WRONLY)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_WRONLY)", buf);
    return RC_ERROR;
  }

  c = value == 0? '0': '1';

  if( write(fd, &c, 1) < 0 )
  {
    LOGSYSERROR("write(%s, \"%c\", 1)", fd, c);
    close(fd);
    return RC_ERROR;
  }

  LOGDIAG3("Wrote GPIO %d value %c.", gpio, c);

  return OK;
}

int gpioNotify(int fd, double timeout)
{
  fd_set          efds;
  struct timeval  tv;
  int             rc;

  FD_ZERO(&efds);
  FD_SET(fd, &efds);

  // no timeout
  if( timeout <= 0.0 )
  {
    rc = select(fd+1, NULL, NULL, &efds, NULL);
  }
  else
  {
    tv.tv_sec  = (long)timeout;
    tv.tv_usec = (long)(timeout - (double)tv.tv_sec) * 1000000;
    if( tv.tv_usec > 1000000 )
    {
      ++tv.tv_sec;
      tv.tv_usec = 0;
    }
    rc = select(fd+1, NULL, NULL, &efds, &tv);
  }

  // change
  if( rc > 0 )
  {
    LOGDIAG3("GPIO value changed.");
    rc = gpioQuickRead(fd);
  }

  // timeout
  else if( rc == 0 )
  {
    LOGDIAG3("GPIO watch timedout.");
    rc = gpioQuickRead(fd);
  }

  // error
  else
  {
    LOGSYSERROR("select(%d, ...)", fd);
  }

  return rc;
}

int gpioOpen(int gpio)
{
  int   fd;
  char  buf[MAX_PATH]; 

  sprintf(buf, "%s/gpio%d/value", GpioRoot, gpio);

  if( (fd = open(buf, O_RDWR)) < 0 )
  {
    LOGSYSERROR("open(\"%s\", O_RDWR)", buf);
    return RC_ERROR;
  }

  LOGDIAG3("Opened GPIO %d.", gpio);

  return fd;
}

int gpioClose(int fd)
{
  if( fd >= 0 )
  {
    close(fd);
  }

  return OK;
}

int gpioQuickRead(int fd)
{
  char  c;

  // go to top of 'file'
  lseek(fd, 0, SEEK_SET);

  if( read(fd, &c, 1) < 0 )
  {
    LOGSYSERROR("read(%s, ...)", fd);
    return RC_ERROR;
  }

  LOGDIAG3("Read GPIO value %c.", c);

  return c == '0'? 0: 1;
}

int gpioQuickWrite(int fd, int value)
{
  char    c;

  // go to top of 'file'
  lseek(fd, 0, SEEK_SET);

  c = value == 0? '0': '1';

  if( write(fd, &c, 1) < 0 )
  {
    LOGSYSERROR("write(%s, \"%c\", 1)", fd, c);
    return RC_ERROR;
  }

  LOGDIAG3("Wrote GPIO value %c.", c);

  return OK;
}

int gpioBitBang(int           fd,
                byte_t        pattern[],
                size_t        bitCount,
                unsigned int  usecIbd)
{
  size_t  byteCount = (bitCount + 7) / 8;
  size_t  i, j, k;
  int     mask, value;

  for(i=0, k=0; i<byteCount; ++i)
  {
    for(j=0, mask=0x80; j<8 && k<bitCount; ++j, ++k)
    {
      if( usecIbd > 0 )
      {
        usleep(usecIbd);
      }
      value = pattern[i] & mask? 1: 0;
      mask >>= 1;
      if( gpioQuickWrite(fd, value) < 0 )
      {
        return RC_ERROR;
      }
    }
  }

  return OK;
}

void gpioMakeDirname(int gpio, char buf[], size_t size)
{
  snprintf(buf, size, "%s/gpio%d", GpioRoot, gpio);
  buf[size-1] = 0;
}
