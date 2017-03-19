////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsLoadTest
//
// File:      bsLoadTest.c
//
/*! \file
 *
 * $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
 * $Rev: 581 $
 *
 * \brief BotSense bsProxy server load tester client(s).
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <libgen.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "botsense/bsI2C.h"
#include "botsense/bsI2CMsgs.h"
#include "botsense/bsNull.h"
#include "botsense/bsNullMsgs.h"
#include "botsense/bsSerial.h"
#include "botsense/bsSerialMsgs.h"

#include "version.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Application Exit Codes
//
#define APP_EC_OK       0         ///< success exit code
#define APP_EC_USAGE    2         ///< usage error exit code
#define APP_EC_EXEC     4         ///< execution error exit code

#define LT_MAX_CLIENTS  256       ///< maximum number of load testing clients
#define LT_MAX_VCONN    4         ///< maximum number of virtual conn. / client
#define LT_N_CLIENT_DFT 1         ///< default option load testing client count

#define LT_DEVICES_DFT  "server,null"
                                  ///< default option list of proxied devices
#define LT_DEV_SERVER   0x0001    ///< load test server-terminated requests
#define LT_DEV_I2C      0x0002    ///< load test proxied \h_i2c device requests
#define LT_DEV_NULL     0x0004    ///< load test proxied /dev/null requests
#define LT_DEV_SERIAL   0x0008    ///< load test proxied serial device requests

#define LT_T_MIN        1000      ///< minimum thread sleep time
#define LT_T_MAX        2000000   ///< minimum thread sleep time

//
// The command and command-line options.
//
static char    *Argv0;                              ///< this command basename
static char    *OptsProxyServer = "localhost";      ///< proxy server addr/port
static int      OptsClientCnt   = LT_N_CLIENT_DFT;  ///< number of clients
static char    *OptsDevices     = LT_DEVICES_DFT;   ///< number of clients
static bool_t   OptsFixedDiag   = false;            ///< fixed diagnostics
static char    *OptsDevSerial   = "/dev/ttyS0";     ///< serial device
static char    *OptsDevI2C      = "/dev/i2c-0";     ///< i2c device

//
// Forward declarations.
//
static int OptsCvtArgServerAddr(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
static int OptsCvtArgClientCnt(const char *argv0, const char *sOptName,
                            char *optarg, void *pOptVal);
static int OptsCvtArgDevices(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal);

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  .synopsis =
    "BotSense bsProxy server load tester client.",

  .long_desc = 
    "The %P application creates a multi-client, multi-thread application to "
    "load test both the bsProxy server and the libbotsense client library."
};

/*!
 * \brief Command-line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -p, --proxy=<addr[:port]>
  {
    .long_opt   = "proxy",
    .short_opt  = 'p',
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsProxyServer,
    .fn_cvt     = OptsCvtArgServerAddr,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<addr[:port]>",
    .opt_desc   = "Proxy Server's network address. The format of the address "
                  "can be either a network hostname or a dotted IP address "
                  "number. If port is not specfied, then the default port "
                  "9195 is used."
  },

  // -c, --count=<count>
  {
    .long_opt   = "count",
    .short_opt  = 'c',
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsClientCnt,
    .fn_cvt     = OptsCvtArgClientCnt,
    .fn_fmt     = OptsFmtInt,
    .arg_name   = "<count>",
    .opt_desc   = "Number of client's to create and run."
  },

    // --devices=<list>
  {
    .long_opt   = "devices",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsDevices,
    .fn_cvt     = OptsCvtArgDevices,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<list>",
    .opt_desc   = "List of proxied devices where each client will create "
                  "a thread and load test. Format:\n"
                  "  list:    dev[,dev...]\n"
                  "  dev:     server i2c null serial"
  },


  // --fixeddiag
  {
    .long_opt   = "fixeddiag",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptsFixedDiag,
    .fn_cvt     = OptsCvtArgBool,
    .fn_fmt     = OptsFmtBool,
    .arg_name   = NULL,
    .opt_desc   = "Fix message tracing and logging diagnostics to the values "
                  "set when %P was invoked."
  },

  // --serdev=<dev>
  {
    .long_opt   = "serdev",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsDevSerial,
    .fn_cvt     = OptsCvtArgStr,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<dev>",
    .opt_desc   = "Proxied serial device. The interface module is fixed at "
                  "libbserver_serial."
  },

  // --i2cdev=<dev>
  {
    .long_opt   = "i2cdev",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsDevI2C,
    .fn_cvt     = OptsCvtArgStr,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<dev>",
    .opt_desc   = "Proxied I2C device. The interface module is fixed at "
                  "libbserver_i2c."
  },

  {NULL, }
};

//
// State
//
static char      *ProxyIPAddr   = "localhost";  ///< default bsProxy IP addr
static int        ProxyIPPort   = BSPROXY_LISTEN_PORT_DFT;
                                                ///< default bsProxy port
static BsClient_P Client[LT_MAX_CLIENTS];       ///< array of clients
static uint_t     ProxiedDevices = LT_DEV_SERVER |
                                   LT_DEV_NULL; ///< proxied devices
static union
{
  ulong_t   seed;         ///< the initial seed
  ushort_t  xsubi[3];     ///< successive x_i'th 48-bit value
} RandomSeq;              ///< Random seed


// ...........................................................................
// Execution Functions
// ...........................................................................

/*!
 * \brief Seed random number generator.
 */
static void RandSeed()
{
  RandomSeq.seed = (ulong_t)time(NULL);
  srand48((long)RandomSeq.seed);
}

/*!
 * \brief Generate random integer between [0,n).
 *
 * \param n   Upper value.
 *
 * \return Random unsigned integer.
 */
static uint_t RandN(uint_t n)
{
  double  r;

  r = erand48( RandomSeq.xsubi );

  return (uint_t)(n * r);
}

/*!
 * \brief Sleep for a random \h_usec duration between [tmin, tmax].
 *
 * \param tmin   Minimum sleep value.
 * \param tmax   Maximum sleep value.
 */
static void RandSleep(uint_t tmin, uint_t tmax)
{
  usleep( tmin + RandN(tmax-tmin) );
}

/*!
 * \brief Randomly pick a virtual connection in list.
 *
 * \param pVConnList  Virtual connection list.
 *
 * \return Virtual connection handle.
 */
static BsVConnHnd_T RandVConn(BsVecHandles_T *pVConnList)
{
  if( pVConnList->m_uCount == 0 )
  {
    return (BsVConnHnd_T)BSPROXY_VCONN_UNDEF;
  }
  else
  {
    return pVConnList->m_vecHnd[RandN((uint_t)pVConnList->m_uCount)];
  }
}

/*!
 * \brief Load test proxied \h_i2c device thread.
 *
 * \param pArg    BotSense client (casted as a NULL).
 *
 * \return Returns NULL on thread exit.
 */
static void *LtThreadI2C(void *pArg)
{
  enum ActionOps
  {
    ActionScan      = BsI2CMsgIdReqScan,
    ActionOpenClose = BsI2CMsgIdNumOf
  };

  static uint_t actions[] = 
  {
    ActionOpenClose, ActionScan
  };

  static uint_t uNumActions = (uint_t)arraysize(actions);

  BsClient_P        pClient = (BsClient_P)pArg;
  const char       *sClientName = bsClientAttrGetName(pClient);
  uint_t            uCloseFreq = 50;
  BsVConnHnd_T      hndVConn;
  bool_t            bIsOpen;
  i2c_addr_t        addrs[256];
  uint_t            i;
  int               rc;

  // open proxied device
  hndVConn = bsI2CReqOpen(pClient, OptsDevI2C, false);

  if( hndVConn < 0 )
  {
    BSCLIENT_LOG_ERROR(pClient, hndVConn, "%s", OptsDevI2C);
    return NULL;
  }

  printf("<%s>: i2copen: %s\n", sClientName, OptsDevI2C);

  bIsOpen = true;

  printf("<%s>: i2c load test thread started.\n", sClientName);

  while( true )
  {
    // randomly sleep between minimum and maximum times
    RandSleep(LT_T_MIN, LT_T_MAX);

    // choose random request to send
    switch( actions[RandN(uNumActions)] )
    {
      case ActionOpenClose:
        if( bIsOpen )
        {
          if( RandN(uCloseFreq) == 0 )
          {
            rc = bsI2CReqClose(pClient, hndVConn);
            if( rc == BS_OK )
            {
              printf("<%s>: i2cclose: %s\n", sClientName, OptsDevI2C);
              bIsOpen = false;
            }
            else
            {
              printf("<%s>: i2cclose: failed: rc=%d\n", sClientName, rc);
            }
          }
        }
        else
        {
          // open proxied device
          hndVConn = bsI2CReqOpen(pClient, OptsDevI2C, false);

          if( hndVConn >= 0 )
          {
            printf("<%s>: i2copen: %s\n", sClientName, OptsDevI2C);
            bIsOpen = true;
          }
          else
          {
            printf("<%s>: i2copen: failed: rc=%d\n", sClientName, hndVConn);
          }
        }
        break;
      case ActionScan:
        if( bIsOpen )
        {
          rc = bsI2CReqScan(pClient, hndVConn, addrs, arraysize(addrs));
          if( rc > 0 )
          {
            printf("<%s>: i2cscan: %d addresses:", sClientName, rc);
            for(i=0; i<rc; ++i)
            {
              printf(" 0x%02x", addrs[i]);
            }
            printf(".\n");
          } 
          else
          {
            printf("<%s>: i2cscan: failed: rc=%d\n", sClientName, rc);
          }
        }
        break;
      default:
        break;
    }
  }

  printf("<%s>: i2c load test thread terminated.\n", sClientName);

  return NULL;
}

/*!
 * \brief Start proxied \h_i2c device load test thread.
 *
 * \param pClient BotSense client.
 */
static void ClientStartTestThreadI2C(BsClient_P pClient)
{
  pthread_t thread;

  if( pthread_create(&thread, NULL, LtThreadI2C, (void *)pClient) )
  {
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_NO_EXEC,
        "pthread_create(\"/dev/null\")");
    return;
  }
}

/*!
 * \brief Load test proxied /dev/null device thread.
 *
 * \param pArg    BotSense client (casted as a NULL).
 *
 * \return Returns NULL on thread exit.
 */
static void *LtThreadNull(void *pArg)
{
  enum ActionOps
  {
    ActionWrite     = BsNullMsgIdReqWrite,
    ActionOpenClose = BsNullMsgIdNumOf
  };

  static uint_t actions[] = 
  {
    ActionOpenClose, ActionWrite
  };

  static uint_t uNumActions = (uint_t)arraysize(actions);

  static char *wrPhrase[] = 
  {
    "It just so happens that your friend here is only MOSTLY dead.",
    "Rest well and dream of large women.",
    "Hello. My name is Inigo Montoya. You killed my father. Prepare to die."
  };
  static uint_t uNumWrPhrases = (uint_t)arraysize(wrPhrase);

  BsClient_P        pClient = (BsClient_P)pArg;
  const char       *sClientName = bsClientAttrGetName(pClient);
  uint_t            uCloseFreq = 50;
  BsVConnHnd_T      hndVConn;
  bool_t            bIsOpen;
  byte_t            buf[1024];
  uint_t            i;
  int               rc;

  // open proxied device
  hndVConn = bsNullReqOpen(pClient, false);

  if( hndVConn < 0 )
  {
    BSCLIENT_LOG_ERROR(pClient, hndVConn, "%s", BS_NULL_DEV_NAME);
    return NULL;
  }

  printf("<%s>: nullopen: %s\n", sClientName, BS_NULL_DEV_NAME);

  bIsOpen = true;

  printf("<%s>: devnull load test thread started.\n", sClientName);

  while( true )
  {
    // randomly sleep between minimum and maximum times
    RandSleep(LT_T_MIN, LT_T_MAX);

    // choose random request to send
    switch( actions[RandN(uNumActions)] )
    {
      case ActionOpenClose:
        if( bIsOpen )
        {
          if( RandN(uCloseFreq) == 0 )
          {
            rc = bsNullReqClose(pClient, hndVConn);
            if( rc == BS_OK )
            {
              printf("<%s>: nullclose: %s\n", sClientName, BS_NULL_DEV_NAME);
              bIsOpen = false;
            }
            else
            {
              printf("<%s>: nullclose: failed: rc=%d\n", sClientName, rc);
            }
          }
        }
        else
        {
          // open proxied device
          hndVConn = bsNullReqOpen(pClient, false);

          if( hndVConn >= 0 )
          {
            printf("<%s>: nullopen: %s\n", sClientName, BS_NULL_DEV_NAME);
            bIsOpen = true;
          }
          else
          {
            printf("<%s>: nullopen: failed: rc=%d\n", sClientName, hndVConn);
          }
        }
        break;
      case ActionWrite:
        if( bIsOpen )
        {
          i = RandN(uNumWrPhrases);
          memcpy(buf, wrPhrase[i], strlen(wrPhrase[i]));
          rc = bsNullReqWrite(pClient, hndVConn, buf, strlen(wrPhrase[i]));
          if( rc > 0 )
          {
            printf("<%s>: nullwrite: %d bytes: \"%*s\"\n",
                sClientName, rc, rc, wrPhrase[i]);
          } 
          else
          {
            printf("<%s>: nullwrite: failed: rc=%d\n", sClientName, rc);
          }
        }
        break;
      default:
        break;
    }
  }

  printf("<%s>: devnull load test thread terminated.\n", sClientName);

  return NULL;
}

/*!
 * \brief Start proxied /dev/null device load test thread.
 *
 * \param pClient BotSense client.
 */
static void ClientStartTestThreadNull(BsClient_P pClient)
{
  pthread_t thread;

  if( pthread_create(&thread, NULL, LtThreadNull, (void *)pClient) )
  {
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_NO_EXEC,
        "pthread_create(\"/dev/null\")");
    return;
  }
}

/*!
 * \brief Load test proxied serial device thread.
 *
 * \param pArg    BotSense client (casted as a NULL).
 *
 * \return Returns NULL on thread exit.
 */
static void *LtThreadSerial(void *pArg)
{
  enum ActionOps
  {
    ActionRead      = BsSerialMsgIdReqRead,
    ActionTrans     = BsSerialMsgIdReqTrans,
    ActionWrite     = BsSerialMsgIdReqWrite,
    ActionOpenClose = BsSerialMsgIdNumOf
  };

  static uint_t actions[] = 
  {
    ActionOpenClose, ActionWrite
  };

  static uint_t uNumActions = (uint_t)arraysize(actions);

  static char *wrPhrase[] = 
  {
    "hello",
    "eat me", 
    "This is the end / Beautiful friend / This is the end / "
    "My only friend, the end"
  };
  static uint_t uNumWrPhrases = (uint_t)arraysize(wrPhrase);

  BsClient_P        pClient = (BsClient_P)pArg;
  const char       *sClientName = bsClientAttrGetName(pClient);
  uint_t            uCloseFreq = 50;
  BsVConnHnd_T      hndVConn;
  bool_t            bIsOpen;
  byte_t            buf[1024];
  uint_t            i;
  int               rc;

  // open proxied device
  hndVConn = bsSerialReqOpen(pClient, OptsDevSerial,
                            115200, 8, 'N', 1, false, false, false);

  if( hndVConn < 0 )
  {
    BSCLIENT_LOG_ERROR(pClient, hndVConn, "%s", OptsDevSerial);
    return NULL;
  }

  printf("<%s>: serialopen: %s\n", sClientName, OptsDevSerial);

  bIsOpen     = true;

  printf("<%s>: serial load test thread started.\n", sClientName);

  while( true )
  {
    // randomly sleep between minimum and maximum times
    RandSleep(LT_T_MIN, LT_T_MAX);

    // choose random request to send
    switch( actions[RandN(uNumActions)] )
    {
      case ActionOpenClose:
        if( bIsOpen )
        {
          if( RandN(uCloseFreq) == 0 )
          {
            rc = bsSerialReqClose(pClient, hndVConn);
            if( rc == BS_OK )
            {
              printf("<%s>: serialclose: %s\n", sClientName, OptsDevSerial);
              bIsOpen = false;
            }
            else
            {
              printf("<%s>: serialclose: failed: rc=%d\n", sClientName, rc);
            }
          }
        }
        else
        {
          // open proxied device
          hndVConn = bsSerialReqOpen(pClient, OptsDevSerial,
                            115200, 8, 'N', 1, false, false, false);

          if( hndVConn >= 0 )
          {
            printf("<%s>: serialopen: %s\n", sClientName, OptsDevSerial);
            bIsOpen = true;
          }
          else
          {
            printf("<%s>: serialopen: failed: rc=%d\n", sClientName, hndVConn);
          }
        }
        break;
      case ActionWrite:
        if( bIsOpen )
        {
          i = RandN(uNumWrPhrases);
          memcpy(buf, wrPhrase[i], strlen(wrPhrase[i]));
          rc = bsSerialReqWrite(pClient, hndVConn, buf, strlen(wrPhrase[i]));
          if( rc > 0 )
          {
            printf("<%s>: serialwrite: %d bytes: \"%*s\"\n",
                sClientName, rc, rc, wrPhrase[i]);
          } 
          else
          {
            printf("<%s>: serialwrite: failed: rc=%d\n", sClientName, rc);
          }
        }
        break;
      case ActionRead:
      case ActionTrans:
      default:
        break;
    }
  }

  printf("<%s>: serial load test thread terminated.\n", sClientName);

  return NULL;
}

/*!
 * \brief Start proxied serial device load test thread.
 *
 * \param pClient BotSense client.
 */
static void ClientStartTestThreadSerial(BsClient_P pClient)
{
  pthread_t thread;

  if( pthread_create(&thread, NULL, LtThreadSerial, (void *)pClient) )
  {
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_NO_EXEC,
        "pthread_create(\"serial\")");
    return;
  }
}

/*!
 * \brief Load test server-terminated messages thread.
 *
 * \param pArg    BotSense client (casted as a NULL).
 *
 * \return Returns NULL on thread exit.
 */
static void *LtThreadServer(void *pArg)
{
  enum ActionOps
  {
    ActionLoopback      = BsProxyMsgIdReqLoopback,
    ActionSetLogging    = BsProxyMsgIdReqSetLogging,
    ActionGetVersion    = BsProxyMsgIdReqGetVersion,
    ActionMsgTrace      = BsProxyMsgIdReqMsgTrace,
    ActionGetVConnList  = BsProxyMsgIdReqGetVConnList,
    ActionGetVConnInfo  = BsProxyMsgIdReqGetVConnInfo
  };

  static uint_t actions[] = 
  {
    ActionLoopback,     ActionSetLogging,    ActionGetVersion,
    ActionMsgTrace,     ActionGetVConnList,  ActionGetVConnInfo
  };

  static uint_t uNumActions = (uint_t)arraysize(actions);

  BsClient_P        pClient = (BsClient_P)pArg;
  const char       *sClientName = bsClientAttrGetName(pClient);
  BsVecHandles_T    vconnlist = {0, {0, }};
  BsVConnInfo_T     vconninfo;
  char              buf[1024];
  uint_t            i, j;
  BsVConnHnd_T      hndVConn;
  int               rc;

  printf("<%s>: server load test thread started.\n", sClientName);

  //bsServerReqMsgTrace(pClient, (BsVConnHnd_T)BSPROXY_VCONN_SERVER, true);
 
  while( true )
  {
    // randomly sleep between minimum and maximum times
    RandSleep(LT_T_MIN, LT_T_MAX);

    // choose random request to send
    switch( actions[RandN(uNumActions)] )
    {
      case ActionLoopback:
        sprintf(buf, "%s wants loopback.", sClientName);
        rc = bsServerReqLoopback(pClient, buf);
        if( rc == BS_OK )
        {
          printf("<%s>: loopback: \"%s\"\n", sClientName, buf);
        }
        else
        {
          printf("<%s>: loopback: failed: rc=%d\n", sClientName, rc);
        }
        break;
      case ActionSetLogging:
        if( !OptsFixedDiag )
        {
          i = RandN(5);
          rc = bsServerReqSetLogging(pClient, (int)i);
          if( rc == BS_OK )
          {
            printf("<%s>: setlogging: %d\n", sClientName, (int)i);
          }
          else
          {
            printf("<%s>: setlogging: failed: rc=%d\n", sClientName, rc);
          }
        }
        break;
      case ActionGetVersion:
        rc = bsServerReqGetVersion(pClient, buf, sizeof(buf));
        if( rc == BS_OK )
        {
          printf("<%s>: getversion: \"%s\"\n", sClientName, buf);
        }
        else
        {
          printf("<%s>: getversion: failed: rc=%d\n", sClientName, rc);
        }
        break;
      case ActionMsgTrace:
        if( !OptsFixedDiag )
        {
          i = RandN(LT_MAX_VCONN+1); // n dev handles/client + 1 server handle
          if( i == LT_MAX_VCONN )
          {
            hndVConn = BSPROXY_VCONN_SERVER;
          }
          else
          {
            hndVConn = RandVConn(&vconnlist);
          }
          if( hndVConn != BSPROXY_VCONN_UNDEF )
          {
            j = RandN(2); // do [not] trace
            rc = bsServerReqMsgTrace(pClient, hndVConn, (bool_t)j);
            if( rc == BS_OK )
            {
              printf("<%s>: msgtrace: vconn=%d, trace=%d\n",
                    sClientName, hndVConn, (int)j);
            }
            else
            {
              vconnlist.m_uCount = 0;
              printf("<%s>: msgtrace: failed: rc=%d\n", sClientName, rc);
            }
          }
        }
        break;
      case ActionGetVConnList:
        rc = bsServerReqGetVConnList(pClient, &vconnlist);
        if( rc == BS_OK )
        {
          printf("<%s>: getvconnlist: (%zu)", sClientName, vconnlist.m_uCount);
          for(i=0; i<vconnlist.m_uCount; ++i)
          {
            printf(" %d", vconnlist.m_vecHnd[i]);
          }
          printf("\n");
        }
        else
        {
          vconnlist.m_uCount = 0;
          printf("<%s>: getvconnlist: failed: rc=%d\n", sClientName, rc);
        }
        break;
      case ActionGetVConnInfo:
        hndVConn = RandVConn(&vconnlist);
        if( hndVConn != BSPROXY_VCONN_UNDEF )
        {
          rc = bsServerReqGetVConnInfo(pClient, hndVConn, &vconninfo);
          if( rc == BS_OK )
          {
            printf("<%s>: getvconninfo:\n", sClientName);
            printf("  vconn   = %d\n", vconninfo.m_vconn);
            printf("  rd      = %d\n", vconninfo.m_rd);
            printf("  client  = %s\n", vconninfo.m_client);
            printf("  devuri  = %s\n", vconninfo.m_devuri);
            printf("  moduri  = %s\n", vconninfo.m_moduri);
            printf("  modver  = %s\n", vconninfo.m_modver);
            printf("  moddate = %s\n", vconninfo.m_moddate);
          }
          else
          {
            vconnlist.m_uCount = 0;
            printf("<%s>: getvconninfo: failed: rc=%d\n", sClientName, rc);
          }
        }
        break;
      default:
        break;
    }
  }

  printf("<%s>: server load test thread terminated.\n", sClientName);

  return NULL;
}

/*!
 * \brief Start server-terminated requests load test thread.
 *
 * \param pClient BotSense client.
 */
static void ClientStartTestThreadServer(BsClient_P pClient)
{
  pthread_t thread;

  if( pthread_create(&thread, NULL, LtThreadServer, (void *)pClient) )
  {
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_NO_EXEC,
        "pthread_create(\"server\")");
    return;
  }
}

/*!
 * \brief Get and pretty print all the versions from all proxied entities.
 *
 * The returned BotSense allocatated string is multi-lined.
 *
 * \param i   Test client instance.
 */
static BsClient_P CreateTestClient(int i)
{
  char        buf[256];
  BsClient_P  pClient;
  int         rc;

  sprintf_s(buf, sizeof(buf), "TestClient%d", i);

  pClient = bsClientNew(buf);

  // connect to bsProxy server
  if( (rc = bsServerConnect(pClient, ProxyIPAddr, ProxyIPPort)) < 0 )
  {
    LOGERROR("bsProxy @%s:%d: %s\n", ProxyIPAddr, ProxyIPPort, bsStrError(rc));
  }

  // success
  else
  {
    LOGDIAG1("Created client %s.", bsClientAttrGetName(pClient));
    rc = BS_OK;

    if( ProxiedDevices & LT_DEV_SERVER )
    {
      ClientStartTestThreadServer(pClient);
    }
    if( ProxiedDevices & LT_DEV_I2C )
    {
      ClientStartTestThreadI2C(pClient);
    }
    if( ProxiedDevices & LT_DEV_NULL )
    {
      ClientStartTestThreadNull(pClient);
    }
    if( ProxiedDevices & LT_DEV_SERIAL )
    {
      ClientStartTestThreadSerial(pClient);
    }
  }

  // Error clean up.
  if( rc < 0 )
  {
    bsClientDelete(pClient);
    pClient = NULL;
  }

  return pClient;
}


// ...........................................................................
// Initialization Functions
// ...........................................................................

/*!
 * \brief Convert command-line server option IP address string to 
 * network name/number and port number.
 *
 * \par Option Argument Syntax:
 * proxy:      addr[:port]
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OptsInvalid()
 *
 * \return If returns, then returns BS_OK.
 */
static int OptsCvtArgServerAddr(const char *argv0, const char *sOptName,
                                char *optarg, void *pOptVal)
{
  char  *sSepField;
  char  *sPort;

  ProxyIPAddr = NULL;
  ProxyIPPort = 0;

  ProxyIPAddr = new_strdup(optarg);

  sSepField = strchr(ProxyIPAddr, ':');

  if( sSepField )
  {
    *sSepField    = 0;
    sPort         = sSepField+1;
    ProxyIPPort = (int)atol(sPort);
    if( ProxyIPPort <= 0 )
    {
      OptsInvalid(Argv0, "'%s': Invalid '%s' argument port value.",
          optarg, sOptName);
    }
  }
  else
  {
    ProxyIPPort = BSPROXY_LISTEN_PORT_DFT;
  }

  if( *ProxyIPAddr == 0 )
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' argument address value.",
        optarg, sOptName);
  }

  return BS_OK;
}

/*!
 * \brief Convert command-line client count option.
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OptsInvalid()
 *
 * \return If returns, then returns BS_OK.
 */
static int OptsCvtArgClientCnt(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal)
{
  OptsCvtArgInt(argv0, sOptName, optarg, &OptsClientCnt);

  if( (OptsClientCnt < 0) || (OptsClientCnt > LT_MAX_CLIENTS) )
  {
    OptsInvalid(Argv0, "'%s': Client '%s' argument out-of-range.",
        optarg, sOptName);
  }

  return BS_OK;
}

/*!
 * \brief Convert command-line devices option string to bitmap.
 *
 * \par Option Argument Syntax:
 * list:      dev[,dev...]\n
 * behavior:  server i2c null serial
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \return If returns, then returns BS_OK.
 */
static int OptsCvtArgDevices(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal)
{
  char               *sList;          // pointer to list string
  char               *sDev;           // pointer to device string
  
  ProxiedDevices = 0;

  sList = new_strdup(optarg);

  for(sDev=strtok(sList, ",");
      sDev!=NULL;
      sDev=strtok(NULL, ","))
  {
    if( !strcmp(sDev, "server") )
    {
      ProxiedDevices |= LT_DEV_SERVER;
    }
    else if( !strcmp(sDev, "i2c") )
    {
      ProxiedDevices |= LT_DEV_I2C;
    }
    else if( !strcmp(sDev, "null") )
    {
      ProxiedDevices |= LT_DEV_NULL;
    }
    else if( !strcmp(sDev, "serial") )
    {
      ProxiedDevices |= LT_DEV_SERIAL;
    }
    else
    {
      OptsInvalid(Argv0, "'%s': Option '%s': Unknown device '%s'.",
        optarg, sOptName, sDev);
    }
  }

  delete(sList);

  return BS_OK;
}

/*!
 * \brief Initialize application.
 *
 * Any command-line error immediately terminates application.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \return
 * Returns \ref APP_EC_OK on success, >0 exit code on failure.
 */
static int MainInit(int argc, char *argv[])
{
  // Name of this process.
  Argv0 = basename(argv[0]);

    // 
  // Parse input options.
  //
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  RandSeed();

  return APP_EC_OK;
}

/*!
 * \brief Application main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */

int main(int argc, char *argv[])
{
  int   i;

  // Initialize command.
  MainInit(argc, argv);

  //
  // Create client load tester
  //
  for(i=0; i<OptsClientCnt; ++i)
  {
    Client[i] = CreateTestClient(i);
  }

  //
  // Wait for interrupt.
  //
  while( true )
  {
    sleep(5);
  }

  //
  // Disonnect from server. All opened proxied devices are automatically closed.
  // Then delete client, freeing up all resources.
  //
  for(i=0; i<OptsClientCnt; ++i)
  {
    bsServerDisconnect(Client[i]);
    bsClientDelete(Client[i]);
  }

  return APP_EC_OK;
}
