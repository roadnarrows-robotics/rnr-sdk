////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Library:   libgpio
//
// File:      mmapGpio.c
//
/*! \file
 *
 * $LastChangedDate: 2015-04-09 15:03:13 -0600 (Thu, 09 Apr 2015) $
 * $Rev: 3916 $
 * 
 * \brief GPIO library definitions using the memory mapped device interface.
 *
 * \par Original Code
 * \author Markham Thomas
 * (https://github.com/mlinuxguy/odpygpio)\n
 * version 1.2  October 15, 2013\n
 * Support Odroid-XU. The Python library for odroid-x and x2 boards from
 * hardkernel implements mmap'd GPIO address space for performance.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
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

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/gpio.h"

#if defined(ARCH_odroid)

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#define MAP_SIZE 4096UL           ///< 4k memory map size
#define MAP_MASK (MAP_SIZE - 1)   ///< memory map mask

// Switch the defines for the correct board.
#define ODROIDXU3
#undef ODROIDXU
#undef ODROIDX2

#define EXYNOS4_PA_GPIO1      0x11400000  ///< base address for GPIO registers
#define EXYNOS_5410           0x13400000  ///< base address for GPIO registers
#define EXYNOS_5422           0x13400000  ///< base address for GPIO registers

#if defined(ODROIDXU3)
  #define EXYNOS EXYNOS_5422

#elif defined(ODROIDXU)
  #define EXYNOS EXYNOS_5410

#elif defined(ODROIDX2)
  #define EXYNOS EXYNOS4_PA_GPIO1

#else
  #error "Unknown Odroid version"
#endif

#define GPIO_GPCONREG 4 // subtract 4 from DATA register base to get CON reg
#define GPIO_UPDOWN   4 // add 4 to DATA register base to get UPD register
#define GPIO_DRIVESTR 8 // add 8 to DATA register base to get drive str ctl reg

// example:
// GPF0CON = 0x0180 low/high nibbles are either 0000=input, 0001=output
// GPF0DAT = 0x0184 data bits for input or output
// GPF0UPD = 0x0188 every 2 bits is a GPIO pin 00=up/down disabled,
//                  01=pull down, 10=pull up, 00
// GPF0DRV = 0x018c drive str control reg (2-bits each pin)
//                  00=1x, 10=2x, 01=3x, 11=4x

/*!
 * \brief System GPIO exported number - pin number association structure.
 */
typedef struct
{
  int pin;     ///< external header pin number
  int gpio;    ///< exported gpio number
} gpio_sysfs_t;

/*!
 * \brief GPIO memory mapped offset channel and bit position structure.
 */
typedef struct
{
  int channel;    ///< offset
  int bit;        ///< bit
} gpio_address_t;

#define NOPIN {0, 0}      ///< no mapped pin to GPIO

#if defined(ODROIDXU3)

/*!
 * \brief Associates the CON10 header pin number to sysfs exported GPIO number.
 */
static gpio_sysfs_t GpioSysfsTbl[] =
{
  {4, 173},   {5, 174},   {6, 171},   {7, 192},
  {8, 172},   {9, 191},   {10, 189},  {11, 190},
  {13, 21},   {14, 210},  {15, 18},   {16, 209},
  {17, 22},   {18, 19},   {19, 30},   {20, 28},
  {21, 29},   {22, 31},   {24, 25},   {26, 24}
};

#if 0
/*!
 * \brief Maps the CON10 header GPIO pin to the memory mapped offset and bit.
 *
 * Indexed by pin number.
 */
static gpio_address_t PinAddrTbl[] =
{
  NOPIN,                      // 0

  // 5V0          GND
  NOPIN,          NOPIN,      // 1, 2

  // ADC_0.AIN0   UART_0.RTSN
  //              gpio 173
  NOPIN,          NOPIN,      // 3, 4

  // UART_0.CTSN  UART_O.RXD
  // gpio 174     gpio 171
  NOPIN,          NOPIN,      // 5, 6

  // SPI_1.MOISI  UART_O.TXD
  // gpio 192     gpio 172
  NOPIN,          NOPIN,      // 7, 8

  // SPI_1.MISO   SPI_1.CLK
  // gpio 191     gpio 189
  NOPIN,          NOPIN,      // 9, 10

  // SPI_1.CSN    PWRON
  // gpio 190
  NOPIN,          NOPIN,      // 11, 12

  // XE.INT13     I2C_1.SCL
  // gpio 21      gpio 210
  {0x0c24,5},     NOPIN,      // 13, 14

  // XE.INT10     I2C_1.SDA
  // gpio 18      gpio 209
  {0x0c24,2},     NOPIN,      // 15, 16

  // XE.INT14     XE.INT11
  {0x0c24,6},     {0x0c24,3}, // 17, 18

  // XE.INT22     XE.INT20
  {0x0c44,6},     {0x0c44,4}, // 19, 20

  // XE.INT21     XE.INT23
  {0x0c44,5},     {0x0c44,7}, // 21, 22

  // XE.INT18     XE.INT17
  {0x0c44,2},     {0x0c44,1}, // 23, 24

  // XE.INT15     XE.INT16
  {0x0c24,7},     {0x0c44,0}, // 25, 26

  // XE.INT25     GND
  {0x0c64,1},     NOPIN,      // 27, 28

  // VDD_IO       GND
  NOPIN,          NOPIN,      // 29, 30
};
#endif // 0

/*!
 * \brief Maps exported GPIO number to the memory mapped offset and bit.
 *
 * Indexed by GPIO number.
 */
static gpio_address_t GpioAddrTbl[514];

/*!
 * \brief Fill device memory address offset and bit lookup table.
 *
 * \todo Need to find mapping for GPIO: 23 171 172 173 ...
 */
static void mmapFillLookupTbl()
{
  memset(GpioAddrTbl, 0, sizeof(GpioAddrTbl));

  // GPIO: 18  Pin: 15  Net: XE.INT10
  GpioAddrTbl[18].channel = 0x0c24;
  GpioAddrTbl[18].bit     = 2;

  // GPIO: 19  Pin: 18  Net: XE.INT11
  GpioAddrTbl[19].channel = 0x0c24;
  GpioAddrTbl[19].bit     = 3;

  // GPIO: 21  Pin: 13  Net: XE.INT13
  GpioAddrTbl[21].channel = 0x0c24;
  GpioAddrTbl[21].bit     = 5;

  // GPIO: 22  Pin: 17  Net: XE.INT14
  GpioAddrTbl[22].channel = 0x0c24;
  GpioAddrTbl[22].bit     = 6;

  // GPIO: 24  Pin: 26  Net: XE.INT16
  GpioAddrTbl[24].channel = 0x0c44;
  GpioAddrTbl[24].bit     = 0;

  // GPIO: 25  Pin: 24  Net: XE.INT17
  GpioAddrTbl[25].channel = 0x0c44;
  GpioAddrTbl[25].bit     = 1;

  // GPIO: 28  Pin: 20  Net: XE.INT20
  GpioAddrTbl[28].channel = 0x0c44;
  GpioAddrTbl[28].bit     = 4;

  // GPIO: 29  Pin: 21  Net: XE.INT21
  GpioAddrTbl[29].channel = 0x0c44;
  GpioAddrTbl[29].bit     = 5;

  // GPIO: 30  Pin: 19  Net: XE.INT22
  GpioAddrTbl[30].channel = 0x0c44;
  GpioAddrTbl[30].bit     = 6;

  // GPIO: 31  Pin: 22  Net: XE.INT23
  GpioAddrTbl[31].channel = 0x0c44;
  GpioAddrTbl[31].bit     = 7;
}

#elif defined(ODROIDXU) || defined(ODROIDX2)

/*!
 * \brief Associates the CON10 header pin number to sysfs exported GPIO number.
 */
static gpio_sysfs_t GpioSysfsTbl[] =
{
  {13, 309},   {14, 316},   {15, 306},  {16, 304},
  {17, 310},   {18, 307},   {19, 319},  {20, 317},
  {21, 318},   {22, 320},   {23, 315},  {24, 314},
  {25, 311},   {26, 313},   {27, 323}
};

/*!
 * \brief Maps the CON10 header GPIO pin to the memory mapped offset and bit.
 *
 * Indexed by pin number.
 */
static gpio_address_t GpioAddrTbl[] =
{
  NOPIN,                      // 0

  // 5V0          GND
  NOPIN,          NOPIN,      // 1, 2

  // ADC_0.AIN0   UART_0.RTSN
  NOPIN,          NOPIN,      // 3, 4

  // UART_0.CTSN  UART_O.RXD
  NOPIN,          NOPIN,      // 5, 6

  // SPI_1.MOISI  UART_O.TXD
  NOPIN,          NOPIN,      // 7, 8

  // SPI_1.MISO   SPI_1.CLK
  NOPIN,          NOPIN,      // 9, 10

  // SPI_1.CSN    PWRON
  NOPIN,          NOPIN,      // 11, 12

  // XE.INT13     XE.INT19
  {0x0c24,5},     {0x0c44,3}, // 13, 14

  // XE.INT10     XE.INT8
  {0x0c24,2},     {0x0c24,0}, // 15, 16

  // XE.INT14     XE.INT11
  {0x0c24,6},     {0x0c24,3}, // 17, 18

  // XE.INT22     XE.INT20
  {0x0c44,6},     {0x0c44,4}, // 19, 20

  // XE.INT21     XE.INT23
  {0x0c44,5},     {0x0c44,7}, // 21, 22

  // XE.INT18     XE.INT17
  {0x0c44,2},     {0x0c44,1}, // 23, 24

  // XE.INT15     XE.INT16
  {0x0c24,7},     {0x0c44,0}, // 25, 26

  // XE.INT25     GND
  {0x0c64,1},     NOPIN,      // 27, 28

  // VDD_IO       GND
  NOPIN,          NOPIN,      // 29, 30
};

#endif // defined(ODROIDXU) || defined(ODROIDX2)

static void *MapBase;             ///< the gpio base address

/*!
 * \brief Test if exported gpio number has a known mapped memory area.
 *
 * \param gpio  Exported GPIO number.
 *
 * \return Returns 1 if has map, 0 otherwise.
 */
static int hasmap(int gpio)
{
  if( (gpio < 0) || (gpio >= arraysize(GpioAddrTbl)) )
  {
    return 0;
  }
  else if( (GpioAddrTbl[gpio].channel == 0) && (GpioAddrTbl[gpio].bit == 0) )
  {
    return 0;
  }
  else
  {
    return 1;
  }
}

static const char *ErrStrNoMapFound = "GPIO block has not been memory mapped.";
static const char *ErrStrGpioNoMap  = "No memory mapped address offset found.";


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

int gpioExportedToPin(int gpio)
{
  size_t  i;

  for(i=0; i<arraysize(GpioSysfsTbl); ++i)
  {
    if( GpioSysfsTbl[i].gpio == gpio )
    {
      return GpioSysfsTbl[i].pin;
    }
  }
  return RC_ERROR;
}

int gpioPinToExported(int pin)
{
  size_t  i;

  for(i=0; i<arraysize(GpioSysfsTbl); ++i)
  {
    if( GpioSysfsTbl[i].pin == pin )
    {
      return GpioSysfsTbl[i].gpio;
    }
  }
  return RC_ERROR;
}

int mmapGpioMap()
{
  int   fd;
  off_t target = EXYNOS;
 
  MapBase = 0;

  if( (fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1 )
  {
    LOGSYSERROR("Could not open /dev/mem device.");
    return RC_ERROR;
  } 

  // Map one page
  MapBase = mmap(NULL,
                  MAP_SIZE,
                  PROT_READ | PROT_WRITE,
                  MAP_SHARED,
                  fd,
                  target & (off_t)~MAP_MASK);

  close(fd);

  if( MapBase == (void *) -1 )
  {
    LOGSYSERROR("Memory map failed.");
    return RC_ERROR;
  } 

  mmapFillLookupTbl();

  LOGDIAG3("Memory mapped GPIO setup at address 0x%0lx.", target);

  return OK;
}

int mmapGpioUnmap()
{
  if( MapBase == 0 )
  {
    LOGWARN("%s", ErrStrNoMapFound);
    return OK;
  }
  else if( munmap(MapBase, MAP_SIZE) == -1 )
  {
    LOGSYSERROR("Memory unmap failed.");  
    return RC_ERROR;
  }
  else
  {
    MapBase = 0;
    LOGDIAG3("Memory mapped GPIO torn down.");
    return OK;
  }
}

int mmapGpioSetDirection(int gpio, int dir)
{
  int         channel, bit;
  div_t       div_res;
  byte_t      val;
  byte_t     *base;
  const char *sOp;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;

  base = (MapBase + channel) - GPIO_GPCONREG;

  div_res = div(bit, 2);      // 2 nibbles per byte so divide by 2

  base += div_res.quot;

  // read
  val = *(byte_t *)base;

  switch( dir )
  {
    // set to output
    case GPIO_DIR_OUT:
      sOp = GPIO_DIR_OUT_STR;

      // if remainder then its upper nibble
      if( div_res.rem )
      {
        val &= 0b00011111;  // upper nibble, not always def to zero
        val |= 0b00010000;  // set upper nibble as output
      }
      // otherwise its lower nibble
      else
      {
        val &= 0b11110001;  // not always def to zero on boot
        val |= 0b00000001;  // set lower nibble as output
      }
      break;

    // set to input
    case GPIO_DIR_IN:
      sOp = GPIO_DIR_IN_STR;

      // if remainder then its upper nibble
      if( div_res.rem )
      {
        val &= 0b00001111;  // clear upper nibble to be input
      }
      // otherwise its lower nibble
      else
      {
        val &= 0b11110000;  // clear lower nibble to be input
      }
      break;

    // invalid direction
    default:
      LOGERROR("GPIO %d: Invalid direction. Must be one of: %d(=in) %d(=out).",
        gpio, GPIO_DIR_IN, GPIO_DIR_OUT);
      return RC_ERROR;
  }

  // write
  *(byte_t *)base = val;  

  LOGDIAG3("GPIO %d: Configured for %sput.", gpio, sOp);

  return OK;
}

int mmapGpioSetPull(int gpio, int pull)
{
  int         channel, bit;
  byte_t      val, v, hld;
  byte_t     *base;
  const char *sOp;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;

  base = (MapBase + channel) + GPIO_UPDOWN;

  switch( pull )
  {
    // disable pullup/down
    case GPIO_PULL_DS:
      sOp = GPIO_PULL_DS_STR;
      v   = 0;
      break;

    // pullup enabled
    case GPIO_PULL_UP:
      sOp = GPIO_PULL_UP_STR;
      v   = 0b00000010;
      break;
 
    // pulldown enabled
    case GPIO_PULL_DN:
      sOp = GPIO_PULL_DN_STR;
      v   = 0b00000001;
      break;

    // invalid pull
    default:
      LOGERROR("GPIO %d: Invalid pull. "
              "Must be one of: %d(=disabled) %d(=up) %d(=down).",
        gpio, GPIO_PULL_DS, GPIO_PULL_UP, GPIO_PULL_DN);
      return RC_ERROR;
  }

  if( bit < 4 )
  {
    hld = (byte_t)(v << (bit*2)); // shift the 2 bits to their proper location
  }
  else
  {
    bit = bit - 4;
    hld = (byte_t)(v << (bit*2)); // shift the 2 bits to their proper location
    base++;                       // move up to next byte
  }

  // read
  val = *(byte_t *)base;

  // set
  val |= hld;

  // write
  *(byte_t *)base = val;  

  LOGDIAG3("GPIO %d: Configured for pull %s.", gpio, sOp);

  return OK;
}

int mmapGpioProbe(int gpio, gpio_info_t *p)
{
  int     channel, bit;
  div_t   div_res;
  byte_t  value;
  byte_t  dir;
  byte_t  pull;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;
  div_res = div(bit, 2);              // 2 nibbles per byte so divide by 2

  p->base     = EXYNOS;
  p->gpio     = gpio;
  p->pin      = gpioExportedToPin(gpio);
  p->channel  = channel;
  p->bit      = bit;

  //
  // Value
  //
  value = *(byte_t *)(MapBase + channel);

  // high
  if( value & (1 << bit) )
  {
    p->value = 1;
  }
  // low
  else
  {
    p->value = 0;
  }

  //
  // Direction
  //
  dir = *(byte_t *)(MapBase + channel - GPIO_GPCONREG);
  
  // if remainder then its upper nibble
  if( div_res.rem )
  {
    if( dir & 0b00010000 )
    {
      p->dir = GPIO_DIR_OUT;
    }
    else
    {
      p->dir = GPIO_DIR_IN;
    }
  }
  // otherwise its lower nibble
  else
  {
    if( dir & 0b00000001 )
    {
      p->dir = GPIO_DIR_OUT;
    }
    else
    {
      p->dir = GPIO_DIR_IN;
    }
  }
  
  //
  // Pull state
  //
  if( bit < 4 )
  {
    // read pull state
    pull  = *(byte_t *)(MapBase + channel + GPIO_UPDOWN);
    // shift the 2 bits to their proper location
    pull = (byte_t)(pull >> (bit*2));
  }
  else
  {
    // read pull state
    pull  = *(byte_t *)(MapBase + channel + GPIO_UPDOWN + 1);
    // shift the 2 bits to their proper location
    pull = (byte_t)(pull >> ((bit-4)*2));
  }

  pull &= 0x03;

  // pullup enabled
  if( pull == 0b00000010 )
  {
    p->pull = GPIO_PULL_UP;
  }
  // pulldown enabled
  else if( pull == 0b00000001 )
  {
    p->pull = GPIO_PULL_DN;
  } 
  // disable pullup/down
  else
  {
    p->pull = GPIO_PULL_DS;
  }

  return OK;
}

int mmapGpioRead(int gpio)
{
  int     channel, bit;
  byte_t  input;
  int     value;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;

  input = *(byte_t *)(MapBase + channel);

  if( input & (1 << bit) )
  {
    value = 1;
  }
  else
  {
    value = 0;
  }

  return value;
}

int mmapGpioWrite(int gpio, int value)
{
  int     channel, bit;
  void   *virtAddr;
  byte_t  output;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;

  virtAddr = MapBase + channel;      // offset of the GPIO

  // get the current bits
  output = *(byte_t *)virtAddr;

  if( value )
  {
    output |=  (byte_t)(1 << bit);        // set the bit
  }
  else
  {
    output &= (byte_t)(~(1 << bit));      // clear the bit
  }

  // write the newly set bit out
  *(byte_t *)virtAddr = output;

  return OK;
}
 
int mmapGpioToggle(int gpio, int count)
{
  int     channel, bit;
  void   *virtAddr;
  byte_t  val, vbl, vcl, sbit;
  int     i;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

  channel = GpioAddrTbl[gpio].channel;
  bit     = GpioAddrTbl[gpio].bit;

  virtAddr = MapBase + channel; // offset of the GPIO

  val = *(byte_t *)virtAddr;    // get the current bits

  sbit = (byte_t)(1 << bit);    // our bit to toggle
  sbit &= 0xff;                 // sanity
  vbl = val ^ sbit;             // toggle the bit one way
  vcl = vbl ^ sbit;             // toggle the bit the other way

  // wiggle
  for(i=0; i<count; ++i)
  {
    //val ^= sbit;            // toggle the bit
    *(byte_t *)virtAddr = vbl;  // write the newly changed bit out
    *(byte_t *)virtAddr = vcl;  // write the newly changed bit out
  }

  return OK;
}

int mmapGpioBitBang(int           gpio,
                    byte_t        pattern[],
                    size_t        bitCount,
                    unsigned int  usecIbd)
{
  size_t  byteCount = (bitCount + 7) / 8;
  size_t  i, j, k;
  int     mask, value;

  if( MapBase == NULL )
  {
    LOGERROR("%s", ErrStrNoMapFound);
    return RC_ERROR;
  }

  if( !hasmap(gpio) )
  {
    LOGERROR("GPIO %d: %s", gpio, ErrStrGpioNoMap);
    return RC_ERROR;
  }

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
      if( mmapGpioWrite(gpio, value) < 0 )
      {
        return RC_ERROR;
      }
    }
  }

  return OK;
}

#else   // unsupported target architectures

static const char *ErrStrArchNoMap  = "No memory mapped GPIO support.";

#define NO_SOUP "Architecture %s: %s."


int gpioExportedToPin(int gpio)
{
  return RC_ERROR;
}

int gpioPinToExported(int pin)
{
  return RC_ERROR;
}

int mmapGpioMap()
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioUnmap()
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioSetDirection(int pin, int dir)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioSetPull(int pin, int pull)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioProbe(int pin, gpio_info_t *p)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioRead(int pin)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioWrite(int pin, int value)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioToggle(int pin, int count)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

int mmapGpioBitBang(int           pin,
                    byte_t        pattern[],
                    size_t        bitCount,
                    unsigned int  usecIbd)
{
  LOGERROR(NO_SOUP, ARCH, ErrStrArchNoMap);
  return RC_ERROR;
}

#endif  // architecture
