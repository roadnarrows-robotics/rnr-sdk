////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Module:    i2c-monitor
//
// File:      i2c-monitor.c
//
/*! \file
 *
 * $LastChangedDate: 2012-05-11 16:07:49 -0600 (Fri, 11 May 2012) $
 * $Rev: 1975 $
 *
 * \brief RoadNarrows i2c monitor for limit switches
 *
 * \author Robin Knight  (robin.knight@roadnarrows.com)
 * \author Brent Wilkins (brent@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, COMEDTEC, OR ANY MEMBERS/EMPLOYEES/CONTRACTORS
// OF COMEDTEC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  COMEDTEC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c-dev.h"
#include "rnr/i2c.h"

#define DEVNAME "/dev/i2c-3"  // Bus 3 is for user stuff on Overo
#define ADDR 0x20             // Address of 16 port IO expander
#define NBYTES 2              // Number of bytes to read

int main(int argc, char *argv[])
{
  if (argc != 4) {
    // 147 F 1336383892.210327
    printf("Usage: %s GPIO# Edge Timestamp\n",argv[0]);
    return 1;
  }

  i2c_t   pI2C;
  int     stat;
  byte_t* buf = (byte_t*) malloc(NBYTES);

  // Open i2c device
  if( (stat = i2c_open(&pI2C, DEVNAME)) < 0 )
  {
    fprintf(stderr,
        "%s: Failed to open device: %d %s\n",
        DEVNAME, stat, strerror(errno));
    return 2;
  }

  // Read from address on i2c device
  if( (stat = i2c_read(&pI2C, ADDR, buf, NBYTES)) < 0 )
  {
    fprintf(stderr,
        "Couldn't read device at address 0x%x: %d %s\n",
        ADDR, stat, strerror(errno));
    return 3;
  }

  int i;
  printf("Read %d bytes: ", stat);
  for(i=0; i<NBYTES; ++i)
  {
    printf("%d ", buf[i]);
  }
  printf("\n");

  free (buf);

  return 0;
}
