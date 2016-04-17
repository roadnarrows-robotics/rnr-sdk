////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekUno.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-11 14:17:34 -0600 (Thu, 11 Jul 2013) $
 * $Rev: 3113 $
 *
 * \brief \h_hek Arduino Uno compatible I/O board class implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <string>
#include <sstream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/serdev.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUno.h"

using namespace std;
using namespace hekateros;

HekUno::HekUno()
{
  m_fd        = -1;
  m_uHekHwVer = 0;
  m_nFwVer    = 0;

  m_nFwOpState  = -1;
  m_nFwSeqNum   = 0;
}

HekUno::~HekUno()
{
  close();
}

int HekUno::open(uint_t uHekHwVer, const string& dev, int baud)
{
  m_uHekHwVer = uHekHwVer;

  if( (m_fd = SerDevOpen(dev.c_str(), baud, 8, 'N', 1, 0, 0) ) == -1 )
  {
    LOGERROR("Failed to open Arduino digital I/O board");
    return -HEK_ECODE_SYS;
  }

  SerDevFIFOInputFlush(m_fd);

  // flush any residuals
  cmdNull();

  //SerDevFIFOInputFlush(m_fd);

  LOGDIAG3("Opened Arduino compatible device: %s@%d", dev.c_str(), baud);

  return HEK_OK;
}

int HekUno::close()
{
  if( m_fd >= 0 )
  {
    ::close(m_fd);
    m_fd = -1;
    LOGDIAG3("Closed Arduino compatible device.");
  }
  m_uHekHwVer = 0;
  m_nFwVer    = 0;
}

int HekUno::scan()
{
  LOGDIAG3("Scanning Arduino digital I/O board capabilities.");

  //
  // Read firmware version. Do this a few times to flush out any residual 
  // characters in arduino command pipeline.
  //
  for(int i=0; i<3; ++i)
  {
    cmdReadFwVersion(m_nFwVer);
    cmdReadLimits();
    if( m_nFwVer != 0 )
    {
      break;
    }
  }

  LOGDIAG3("Arduino firmware version %d.", m_nFwVer);

  return HEK_OK;
}

int HekUno::cmdReadFwVersion(int &ver)
{
  char cmd[] = "!v\r";
  char rsp[16];
  int  n;  

  ver = 0;

  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp), 200000);

  if( n > 1 )
  {
    ver = atoi(rsp+1);
    return HEK_OK;
  }
  else
  {
    LOGERROR("Failed to read firmware version.");
    return -HEK_ECODE_NO_EXEC;
  }
}

byte_t HekUno::cmdReadLimits()
{
  char    cmd[] = "!o\r";
  char    rsp[16];
  int     n;  
  byte_t  val = 0;

  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( n > 2 )
  {
    val = unpackHex(rsp+1);
  }
  else
  {
    LOGERROR("Failed to read optical limits.");
    return -HEK_ECODE_NO_EXEC;
  }

//fprintf(stderr, "received %x from uno.\n", val);

  return val;
}

byte_t HekUno::cmdReadAux()
{
  char    cmd[] = "!e\r";
  char    rsp[16];
  int     n;  
  byte_t  val = 0;

  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( n > 2 )
  {
    val = unpackHex(rsp+1);
  }

  return val;
}

int HekUno::cmdReadPin(int id)
{
  char  cmd[16];
  char  rsp[16];
  int   n;  
  int   val;

  sprintf(cmd, "!r %d\r", id);
  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( n > 1 )
  {
    val = atoi(rsp);
  }

  return val;
}

int HekUno::cmdWritePin(int id, int val)
{
  char cmd[16];
  char rsp[16];
  int  n;  

  sprintf(cmd, "!w %d %d\r", id, val? 1: 0);
  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( (n < 1) || (rsp[0] != '@') )
  {
    return -HEK_ECODE_NO_EXEC;
  }
  else
  {
    return HEK_OK;
  }
}

int HekUno::cmdConfigPin(int id, char dir)
{
  char  cmd[16];
  char  rsp[16];
  int   c;
  int   n;  

  if( dir == 'i' )
  {
    c = 0;
  }
  else if( dir == 'o' )
  {
    c = 1;
  }
  else 
  {
    return -HEK_ECODE_BAD_VAL;
  }

  sprintf(cmd, "!c %d %d\r", id, c);
  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( (n < 1) || (rsp[0] != '@') )
  {
    return -HEK_ECODE_NO_EXEC;
  }
  else
  {
    return HEK_OK;
  }
}

int HekUno::cmdSetAlarmLED(int val)
{
  char cmd[16];
  char rsp[16];
  int  n;  

  if( m_uHekHwVer < HEK_VERSION(1, 2, 0) )
  {
    return cmdWritePin(5, val);
  }

  sprintf(cmd, "!l 1 %d\r", val? 1: 0);
  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( (n < 1) || (rsp[0] != '@') )
  {
    return -HEK_ECODE_NO_EXEC;
  }
  else
  {
    return HEK_OK;
  }
}

int HekUno::cmdSetStatusLED(int val)
{
  char cmd[16];
  char rsp[16];
  int  n;  

  if( m_uHekHwVer < HEK_VERSION(1, 2, 0) )
  {
    return HEK_OK;
  }

  sprintf(cmd, "!l 2 %d\r", val? 1: 0);
  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( (n < 1) || (rsp[0] != '@') )
  {
    return -HEK_ECODE_NO_EXEC;
  }
  else
  {
    return HEK_OK;
  }
}

int HekUno::cmdSetHaltedState()
{
  char cmd[] = "!h\r";
  char rsp[16];
  int  n;  

  if( m_uHekHwVer < HEK_VERSION(1, 2, 0) )
  {
    return HEK_OK;
  }

  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp));

  if( (n < 1) || (rsp[0] != '@') )
  {
    return -HEK_ECODE_NO_EXEC;
  }
  else
  {
    return HEK_OK;
  }
}

int HekUno::cmdTestInterface()
{
  char  cmd[] = "!t\r";
  char  rsp[16];
  int   n;  
  int   nOpState;
  int   nSeqNum;

  if( m_nFwVer >= 3 )
  {
    sendCommand(cmd, strlen(cmd));

    n = recvResponse(rsp, sizeof(rsp));

    if( (n < 1) || (rsp[0] != '@') )
    {
      return -HEK_ECODE_NO_EXEC;
    }

    n = sscanf(&rsp[1], "%d %d", &nOpState, &nSeqNum);

    if( n != 2 )
    {
      LOGERROR("Failed test of interface: bad response.");
      return -HEK_ECODE_NO_EXEC;
    }

    if( nOpState != m_nFwOpState )
    {
      fprintf(stderr, "ARDUINO TEST: New OpState=%d\n", m_nFwOpState);
    }

    if( nSeqNum != m_nFwSeqNum )
    {
      fprintf(stderr, "ARDUINO TEST: Lost sync: Sequence number received=%d, "
                    "expected=%d\n", nSeqNum, m_nFwSeqNum);
    }

    m_nFwOpState  = nOpState;
    m_nFwSeqNum   = (nSeqNum + 1) & 0xff;
  }

  return HEK_OK;
}

int HekUno::cmdNull()
{
  char  cmd[] = "\r";
  char  rsp[16];
  int   n;  

  sendCommand(cmd, strlen(cmd));

  n = recvResponse(rsp, sizeof(rsp), 2000000);

  return HEK_OK;
}

int HekUno::sendCommand(char *buf, size_t nBytes, uint_t timeout)
{
  SerDevWrite(m_fd, (byte_t *)buf, nBytes, timeout);
  SerDevFIFOOutputDrain(m_fd);
  return HEK_OK;
}

int HekUno::recvResponse(char buf[], size_t count, uint_t timeout)
{
  char    c;
  ssize_t nBytes=0;

  // grab actual response
  nBytes = SerDevReadLine(m_fd, buf, count, (char *)"\n\r", timeout);

#ifdef MON_DEBUG
  fprintf(stderr, "DBG - received(%zd): ", nBytes);
  for(ssize_t i=0; i<nBytes; ++i)
  {
    fprintf(stderr, "%x ", buf[i]);
  }
  fprintf(stderr, "\n");
#endif // MON_DEBUG

  return (int)nBytes;
}

byte_t HekUno::unpackHex(char buf[])
{
  byte_t  val = 0;

  //
  // Parse 2 character hex.
  //
  for(int i=0; i<2; ++i)
  {
    val *= 16;
    if(buf[i] >= '0' && buf[i] <='9')
    {
      val += buf[i] - '0';
    }

    else if (buf[i] >= 'A' && buf[i] <= 'F')
    {
      val += buf[i] -'A'+10;
    }

    else if (buf[i] >= 'a' && buf[i] <= 'f')
    {
      val += buf[i] -'a'+10;
    }
  }
  
  return val;
}
