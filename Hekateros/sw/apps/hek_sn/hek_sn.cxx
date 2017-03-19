////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   hek_sn   
//
// File:      hek_sn.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-15 12:45:22 -0700 (Thu, 15 Jan 2015) $
 * $Rev: 3857 $
 *
 * \brief Print Hekateros serial numbers.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "version.h"

using namespace std;

/*!
 * \ingroup apps
 * \defgroup hek_sn hek_sn
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                  ///< the command
static bool_t   OptsAll = false;        ///< all S/Ns


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "",

  // synopsis
  "Print Hekateros serial numbers to stdout.",

  // long_desc = 
  "The %P command prints the read-only serial numbers programmed into "
  "Hekateros to stdout.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -a, --all
  {
    "all",                // long_opt
    'a',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsAll,             // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Show all serial numbers associated with Hekateros."
  },

  {NULL, }
};


/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);
}

static void printSN()
{
  static const string SNFilename("/usr/local/share/misc/.sn");

  string line;

  ifstream snfile(SNFilename.c_str());

  cout << "Serial Number: ";

  if( snfile.is_open() )
  {
    while( getline(snfile, line) )
    {
      cout << line << endl;
    }
    snfile.close();
  }
  else
  {
    cout << "??????" << endl;
  }
}

#define MAP_SIZE        4096UL          ///< memory map page size
#define MAP_MASK        (MAP_SIZE - 1)  ///< memory map mask
#define DEVID_ADDR_HIGH 0x4830a224      ///< device id high address
#define DEVID_ADDR_LOW  0x4830a218      ///< device id low address
#define DEVID_ADDR_STEP 4               ///< device id adress step size

static void printProcIdOvero()
{
  int           fd;
  void          *map_base, *virt_addr; 
  unsigned long read_result;
  off_t         addr;
  unsigned long i;

  printf("Processor Id:  ");

  if( (fd = open("/dev/mem", O_RDONLY | O_SYNC)) == -1 )
  {
    printf("?\n");
    return;
  }

  // map one page
  map_base = mmap(0, MAP_SIZE, PROT_READ, MAP_SHARED, fd,
                    DEVID_ADDR_LOW & ~MAP_MASK);

  if( map_base == (void *)(-1) )
  {
    close(fd);
    printf("??\n");
    return;
  }
 
  // reverse printout 
  for(addr = DEVID_ADDR_HIGH; addr >= DEVID_ADDR_LOW; addr -= DEVID_ADDR_STEP)
  {
    virt_addr = (void *)((unsigned long)map_base + (addr & MAP_MASK));
    read_result = *((unsigned long *)virt_addr);
  
    printf("0x%08lx ", read_result);
  }

  printf("\n");

  munmap(map_base, MAP_SIZE);

  close(fd);
}

#if 0 // RDK
#define USB_IDVENDOR        0x0403
#define USB_IDPRODUCT       0x6001
#define USB_IMANUFACTURER   RoadNarrows
#define USB_IPRODUCT        iProduct
#define USB_ISerial
#define USB_IPRODUCT_ARDUINO Arduino_Pro_Mini_328
#define USB_IPRODUCT_DYNABUS Dynamixel_Bus
#define USB_IPRODUCT_DYNABUS_ALT Dynamixel_Chain

static void printUSBSNs()
{
  system("lsusb -v -d 0403:6001 >/tmp/lsusb.out")
}
#endif // RDK

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  mainInit(argc, argv);

  printSN();

  if( OptsAll )
  {
    printProcIdOvero();
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
