////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Library:   libgpio
//
// File:      gpio.h
//
/*! \file
 *
 * $LastChangedDate: 2015-04-08 17:22:10 -0600 (Wed, 08 Apr 2015) $
 * $Rev: 3913 $
 *
 * \brief GPIO interface declarations and defines.
 *
 * The memory mapped interface is base on:
 * \author Markham Thomas
 * (https://github.com/mlinuxguy/odpygpio)\n
 * 
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016. RoadNarrows LLC.
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
////////////////////////////////////////////////////////////////////////////////

#ifndef _GPIO_H
#define _GPIO_H

#include <sys/types.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

//
// GPIO direction
//
#define GPIO_DIR_IN       0           ///< input
#define GPIO_DIR_OUT      1           ///< output
#define GPIO_DIR_IN_STR   "in"        ///< input string
#define GPIO_DIR_OUT_STR  "out"       ///< output string

//
// GPIO edge to make select trigger
//
#define GPIO_EDGE_NONE        0           ///< no edge
#define GPIO_EDGE_RISING      1           ///< rising edge
#define GPIO_EDGE_FALLING     2           ///< falling edge
#define GPIO_EDGE_BOTH        3           ///< both edges
#define GPIO_EDGE_NONE_STR    "none"      ///< no edge string
#define GPIO_EDGE_RISING_STR  "rising"    ///< rising edge string
#define GPIO_EDGE_FALLING_STR "falling"   ///< falling edge string
#define GPIO_EDGE_BOTH_STR    "both"      ///< both edges string

//
// GPIO pull ups
//
#define GPIO_PULL_DS      0           ///< disable pullup/down
#define GPIO_PULL_UP      1           ///< enable pullup
#define GPIO_PULL_DN      2           ///< enable pulldown
#define GPIO_PULL_DS_STR  "disabled"  ///< disable pullup/down string
#define GPIO_PULL_UP_STR  "up"        ///< enable pullup string
#define GPIO_PULL_DN_STR  "down"      ///< enable pulldown string

/*!
 * \brief GPIO info structure.
 */
typedef struct
{
  int           gpio;     ///< sysfs exported gpio number
  int           pin;      ///< external header pin number
  int           dir;      ///< gpio direction
  int           edge;     ///< gpio edge type trigger 
  int           pull;     ///< pull state
  int           value;    ///< current value
} gpio_info_t;


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// GPIO access methods using the sysfs system.
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Make GPIO directory name.
 *
 * Director name: /sys/class/gpio/gpio<gpio>
 *
 * The base directory holds key special files such as <em>value</em>,
 * <em>direction</em>, and <em>edge</em>.
 *
 * Method: sysfs
 *
 * \param gpio        The sysfs exported GPIO number.
 * \param [out] buf   Buffer to hold directory name.
 * \param size        Size of buffer in bytes.
 */
extern void gpioMakeDirname(int gpio, char buf[], size_t size);

/*!
 * \brief Export (create) a GPIO interface.
 *
 * The call requires root privaleges.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioExport(int gpio);

/*!
 * \brief Unexport (delete) a GPIO interface.
 *
 * The call requires root privaleges.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioUnexport(int gpio);

/*!
 * \brief Set GPIO signal direction.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param dir   Direction. One of: GPIO_DIR_IN(0) GPIO_DIR_OUT(1)
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioSetDirection(int gpio, int dir);

/*!
 * \brief Set GPIO edge trigger type. 
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param edge  Edge. One of: GPIO_EDGE_NONE(0)    GPIO_EDGE_RISING(1)
 *                            GPIO_EDGE_FALLING(2) GPIO_EDGE_BOTH(3)
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioSetEdge(int gpio, int edge);

/*!
 * \brief Set GPIO pull.
 *
 * Methodsys mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param pull    Pull type. One of:
 *                  GPIO_PULL_DS(0) to disabled 
 *                  GPIO_PULL_UP(1) to pull-up 
 *                  GPIO_PULL_DN(2) to pull-down 
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioSetPull(int gpio, int pull);

/*!
 * \brief Safely probe GPIO parameters.
 *
 * Method: sysfs
 *
 * \param gpio      The sysfs exported GPIO number.
 * \param [out] p   GPIO info.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioProbe(int gpio, gpio_info_t *p);

/*!
 * \brief Read GPIO pin's current value.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return
 * On success the pin value 0 or 1 is returned.
 * Otherwise RC_ERROR(-1) is returned.
 */
extern int gpioRead(int gpio);

/*!
 * \brief Write GPIO value.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param value GPIO pin value. One of: 0 (low) or 1 (high).
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioWrite(int gpio, int value);

/*!
 * \brief Notify on GPIO input value change.
 *
 * This function blocks until 1) value has changed, 2) a timeout has occurred,
 * or 3) an error was encoutered.
 *
 * Method: sysfs
 *
 * \param fd      Open GPIO pin file descriptor.
 * \param timeout Wait timeout. A value of 0.0 is no timeout.
 *
 * \return
 * On success the pin value 0 or 1 is returned.
 * Otherwise RC_ERROR(-1) is returned. Use errno to determine error.
 */
extern int gpioNotify(int fd, double timeout);

/*!
 * \brief Open GPIO pin.
 *
 * Leaving the file descriptor open allows for quick reads or writes. It is
 * the responsibility of the user to close the file descriptor.
 *
 * Method: sysfs
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return On success the open file descriptor is returned.
 * Otherwise RC_ERROR(-1) is returned.
 */
extern int gpioOpen(int gpio);

/*!
 * \brief Close GPIO pin.
 *
 * Method: sysfs
 *
 * \param fd  Open GPIO pin file descriptor.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioClose(int fd);

/*!
 * \brief Quick read GPIO pin's current value.
 *
 * Method: sysfs
 *
 * \param fd  Open GPIO pin file descriptor.
 *
 * \return
 * On success the pin value 0 or 1 is returned.
 * Otherwise RC_ERROR(-1) is returned.
 */
extern int gpioQuickRead(int fd);

/*!
 * \brief Quick write GPIO pin value.
 *
 * Method: sysfs
 *
 * \param fd    Open GPIO pin file descriptor.
 * \param value GPIO pin value. One of: 0 (low) or 1 (high).
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioQuickWrite(int fd, int value);

/*!
 * \brief Bit-bang bits out a GPIO pin.
 *
 * Bits from pattern[0] ... pattern[N] sequentially drive the GPIO output
 * pin. If bit == 0 then the pin is driven low, if bit == 1 then the pin
 * is driven high.
 *
 * The pattern buffer must be at least (bitCount + 7) / 8 bytes long. The bits
 * in pattern[i] are read out in big-endian order b7 b6 ... 
 *
 * Between bit output, an inter-bit delay of usecIbd is enforced. 
 *
 * Method: sysfs
 *
 * \param fd        Open GPIO pin file descriptor.
 * \param pattern   Bit pattern buffer to bang. The bytes are big-endian.
 * \param bitCount  Count of bits to bang.
 * \param usecIbd   Inter-bit delay in microseconds. A 0 value means a fast
 *                  as the architecture can bang.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int gpioBitBang(int           fd,
                       byte_t        pattern[],
                       size_t        bitCount,
                       unsigned int  usecIbd);


#ifdef MMAP_GPIO

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// GPIO access methods using memory mapped I/O.
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief GPIO info structure.
 */
typedef struct
{
  int           gpio;     ///< sysfs exported gpio number
  int           pin;      ///< external header pin number
  off_t         base;     ///< memory mapped base address
  int           channel;  ///< memory mapped gpio channel offset
  int           bit;      //<< memory mapped gpio bit
  int           dir;      ///< gpio direction
  int           edge;     ///< gpio edge type trigger 
  int           pull;     ///< pull state
  int           value;    ///< current value
} mmap_gpio_info_t;

/*!
 * \brief Find the external header pin number from system exported GPIO number.
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return On success returns the associated header pin number.
 * On error, RC_ERROR(-1) is returned.
 */
extern int gpioExportedToPin(int gpio);

/*!
 * \brief Find the system exported GPIO number from external header pin number.
 *
 * \param pin   External CON10 header pin number.
 *
 * \return On success returns the associated exported GPIO number.
 * On error, RC_ERROR(-1) is returned.
 */
extern int gpioPinToExported(int pin);

/*!
 * \brief Map the memory block of GPIO.
 *
 * This call must be called prior to accessing the GPIO via memory.
 *
 * Method: mmap
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioMap();

/*!
 * \brief Unmap the memory block of GPIO.
 *
 * Should be called when finished with memory mapped GPIO.
 *
 * Method: mmap
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioUnmap();

/*!
 * \brief Set GPIO signal direction.
 *
 * Method: mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param dir   Direction. One of: GPIO_DIR_IN(0) GPIO_DIR_OUT(1)
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioSetDirection(int gpio, int dir);

/*!
 * \brief Set GPIO pull.
 *
 * Method: mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param pull  Pull type. One of:
 *                  GPIO_PULL_DS(0) to disabled 
 *                  GPIO_PULL_UP(1) to pull-up 
 *                  GPIO_PULL_DN(2) to pull-down 
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioSetPull(int gpio, int pull);

/*!
 * \brief Safely probe GPIO parameters.
 *
 * Method: mmap
 *
 * \param gpio      The sysfs exported GPIO number.
 * \param [out] p   GPIO info.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioProbe(int gpio, mmap_gpio_info_t *p);

/*!
 * \brief Read GPIO pin's current value.
 *
 * Method: mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 *
 * \return
 * On success the pin value 0 or 1 is returned.
 * Otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioRead(int gpio);

/*!
 * \brief Write GPIO pin value.
 *
 * \note Expect it to take around 800 clocks after setting a output bit for it
 * to show.
 * 
 * Method: mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param value GPIO pin value. One of: 0 (low) or 1 (high).
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioWrite(int gpio, int value);

/*!
 * \brief Toggle GPIO count high-low (low-high) times.
 *
 * Method: mmap
 *
 * \param gpio  The sysfs exported GPIO number.
 * \param count Count toggles.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioToggle(int gpio, int count);

/*!
 * \brief Bit-bang bits out a GPIO pin.
 *
 * Bits from pattern[0] ... pattern[N] sequentially drive the GPIO output
 * pin. If bit == 0 then the pin is driven low, if bit == 1 then the pin
 * is driven high.
 *
 * The pattern buffer must be at least (bitCount + 7) / 8 bytes long. The bits
 * in pattern[i] are read out in big-endian order b7 b6 ... 
 *
 * Between bit output, an inter-bit delay of usecIbd is enforced. 
 *
 * Method: mmap
 *
 * \param gpio      The sysfs exported GPIO number.
 * \param pattern   Bit pattern buffer to bang. The bytes are big-endian.
 * \param bitCount  Count of bits to bang.
 * \param usecIbd   Inter-bit delay in microseconds. A 0 value means a fast
 *                  as the architecture can bang.
 *
 * \return On success OK(0) is returned, otherwise RC_ERROR(-1) is returned.
 */
extern int mmapGpioBitBang(int           gpio,
                           byte_t        pattern[],
                           size_t        bitCount,
                           unsigned int  usecIbd);

#endif // MMAP_GPIO


C_DECLS_END

#endif // _GPIO_H
