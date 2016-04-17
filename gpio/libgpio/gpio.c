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
 * (C) 2015.  RoadNarrows LLC.
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
    LOGERROR("Unknown direction read: \"%s\".", buf);
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
  int   fd;
  char  buf[MAX_PATH]; 

  if( (dir != GPIO_DIR_IN) && (dir != GPIO_DIR_OUT) )
  {
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

  sprintf(buf, "%s", (dir == GPIO_DIR_IN? GPIO_DIR_IN_STR: GPIO_DIR_OUT_STR));

  if( write(fd, buf, strlen(buf)) < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %zu)", fd, buf, strlen(buf));
    close(fd);
    return RC_ERROR;
  }

  close(fd);

  LOGDIAG3("GPIO %d direction set to %s.", gpio, buf);

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
  p->pin      = gpioExportedToPin(gpio);
  p->base     = 0;
  p->channel  = 0;
  p->bit      = 0;
  p->dir      = GPIO_DIR_IN;
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
