////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// File:      spintest.c
//
/*! \file
 *
 * $LastChangedDate: 2011-07-15 14:10:46 -0600 (Fri, 15 Jul 2011) $
 * $Rev: 1139 $
 *
 * \brief Simple continuous spin test of a servo.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011  RoadNarrows LLC.
 * \n All Rights Reserved
 */
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
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/select.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "dynamixel/Dynamixel.h"
#include "dynamixel/libDynamixel.h"

#include "version.h"

#define APP_EC_OK       0
#define APP_EC_USAGE    2
#define APP_EC_EXEC     4


// 
// The command with option and argument values.
//
static char  *Argv0;                            ///< the command
static char  *OptsSerDev    = "/dev/ttyUSB0";   ///< the serial device
static int    OptsBaudRate  = 1000000;          ///< serial baudrate
static int    OptsSpeed     = 200;              ///< servo speed
static int    ArgServoId    = -1;               ///< servo id


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  "id",

  // synopsis
  "Spin dynamixel servo continuously.",

  // long_desc = 
  "The %P test application spins the given servo continuously at the given "
  "speed.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // -d, --device <device>
  {
    "device",                   // long_opt
    'd',                        // short_opt
    required_argument,          // has_arg
    true,                       // has_default
    &OptsSerDev,                // opt_addr
    OptsCvtArgStr,              // fn_cvt
    OptsFmtStr,                 // fn_fmt
    "<device>",                 // arg_name
    "Serial device interface",  // opt desc
  },

  // -b, --baudrate <baud>
  {
    "baudrate",                 // long_opt
    'b',                        // short_opt
    required_argument,          // has_arg
    true,                       // has_default
    &OptsBaudRate,              // opt_addr
    OptsCvtArgInt,              // fn_cvt
    OptsFmtInt,                 // fn_fmt
    "<baud>",                   // arg_name
    "Serial interface baudrate",  // opt desc
  },

  // -s, --speed <speed>
  {
    "speed",                    // long_opt
    's',                        // short_opt
    required_argument,          // has_arg
    true,                       // has_default
    &OptsSpeed,                 // opt_addr
    OptsCvtArgInt,              // fn_cvt
    OptsFmtInt,                 // fn_fmt
    "<speed>",                  // arg_name
    "Speed of servo. <0 = CCW, 0 = stop, >0 = CW.",   // opt desc
  },

  {NULL, }
};

//
// Global Data
//
int           ServoDir  = DYNA_DIR_CW;         
DynaHandle_T *DynaHandle;

bool_t peekc(uint_t msec)
{
   int            fno = fileno(stdin);
   fd_set         fdset;
   uint_t         usec = msec * 1000;
   struct timeval timeout;
   int            rc;

   FD_ZERO(&fdset);
   FD_SET(fno, &fdset);
   
   // timeout (gets munged after each select())
   timeout.tv_sec  = (time_t)(usec / 1000000);
   timeout.tv_usec = (time_t)(usec % 1000000);

   if( (rc = select(fno+1, &fdset, NULL, NULL, &timeout)) > 0 )
   {
     fflush(stdin);
   }

   return rc>0? true: false;
}

/*!
 * \brief Main argument initialization.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInitArgs(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true,
                 &argc, argv);

  if( argc < 1 )
  {
    OptsInvalid(Argv0, "no servo id specified.");
  }
  else
  {
    ArgServoId = atoi(argv[0]);
  }

  if( OptsSpeed < 0 )
  {
    ServoDir = DYNA_DIR_CCW;
    OptsSpeed = -OptsSpeed;
  }
  else
  {
    ServoDir = DYNA_DIR_CW;
  }
}

static int AppDynaInit()
{
  int       rc;
  ushort_t  lim;
  ushort_t  speed;
  ushort_t  torque;
  byte_t    alarms;

  //
  // Create interface
  //
  DynaHandle = dynaCreateInterface(OptsSerDev, OptsBaudRate);

  if( DynaHandle == NULL )
  {
    LOGERROR("failed to create interface.");
    return -APP_EC_EXEC;
  }

  //
  // Ping servo
  //
  if( !dynaPing(DynaHandle, ArgServoId) )
  {
    LOGERROR("failed to ping servo %d.", ArgServoId);
    return -APP_EC_EXEC;
  }

  //
  // Set up continuous mode
  //
  rc = dynaRead16(DynaHandle, ArgServoId, DYNA_ADDR_LIM_CW_LSB, &lim, &alarms);
  if( (rc == DYNA_OK) && (lim != DYNA_CW_POS_CONT_MODE) )
  {
    dynaWrite16(DynaHandle, ArgServoId,
        DYNA_ADDR_LIM_CW_LSB, DYNA_CW_POS_CONT_MODE, &alarms);
    sleep(1); // writing to EEPROM needs a little time - servo is blocked 
  }

  rc = dynaRead16(DynaHandle, ArgServoId, DYNA_ADDR_LIM_CCW_LSB, &lim, &alarms);
  if( (rc == DYNA_OK) && (lim != DYNA_CCW_POS_CONT_MODE) )
  {
    dynaWrite16(DynaHandle, ArgServoId,
        DYNA_ADDR_LIM_CCW_LSB, DYNA_CCW_POS_CONT_MODE, &alarms);
    sleep(1); // writing to EEPROM needs a little time - servo is blocked 
  }

  //
  // Set speed
  //
  speed = OptsSpeed & DYNA_GOAL_SPEED_MAG_MASK;
  if( ServoDir == DYNA_DIR_CW )
  {
    speed |= (ushort_t)DYNA_GOAL_SPEED_DIR_CW;
  }
  else
  {
    speed |= (ushort_t)DYNA_GOAL_SPEED_DIR_CCW;
  }
  dynaWrite16(DynaHandle, ArgServoId, DYNA_ADDR_GOAL_SPEED_LSB, speed,
      &alarms);
  
  // set torque limit
  torque = DYNA_TORQUE_MAX_RAW;
  dynaWrite16(DynaHandle, ArgServoId, DYNA_ADDR_LIM_TORQUE_LSB, torque,
      &alarms);
  
  return APP_EC_OK;
}

static void AppDynaRun()
{
  while(1)
  {
    if( peekc(10) )
    {
      break;
    }
  }
}

static void AppDynaDeinit()
{
  byte_t    alarms;

  // stop servo
  dynaWrite16(DynaHandle, ArgServoId, DYNA_ADDR_GOAL_SPEED_LSB, 0, &alarms);
}

static void AppBanner()
{
  printf("\n");
  printf("Spinning servo %d %s at raw speed of %d.\n",
      ArgServoId,
      (ServoDir==DYNA_DIR_CW? "clockwise": "counter-clockwise"),
      OptsSpeed);
  printf("Press <CR> to stop.\n");

}

/*!
 * \brief Main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  int   rc;

  MainInitArgs(argc, argv);

  if( (rc = AppDynaInit()) == APP_EC_OK )
  {
    AppBanner();
    AppDynaRun();
    AppDynaDeinit();
  }

  return rc >= 0? APP_EC_OK: -rc;
}
