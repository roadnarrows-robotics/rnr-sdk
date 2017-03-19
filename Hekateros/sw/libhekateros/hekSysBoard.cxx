////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekSysBoard.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief \h_hek original system board.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSysBoard.h"

using namespace std;
using namespace hekateros;

#if defined(ARCH_overo) || defined(ARCH_linaro)
#define TGT_EMBEDDED    ///< on hekateros target
#else
#undef TGT_EMBEDDED     ///< off hekateros target
#endif


int HekSysBoard::open(const string& dev)
{
  byte_t  val;
  int     rc;

  m_i2c.fd    = -1;
  m_i2c.addr  = HekIOExpI2CAddr;

#ifdef TGT_EMBEDDED

  // open i2c device
  if( i2c_open(&m_i2c, dev.c_str()) < 0 )
  {
    LOGSYSERROR("%s.", dev.c_str());
    return -HEK_ECODE_NO_DEV;
  }

  LOGDIAG3("Hekateros i2c bus hardware initialized successfully.");

  //
  // Set required i/o expansion configuration.
  //
 
  // port 0 polarity
  val = readIOExp(HekIOExpCmdPolarity0);

  if( val != HekIOExpConstPolarity0 )
  {
    if( (rc = writeIOExp(HekIOExpCmdPolarity0, HekIOExpConstPolarity0)) < 0 )
    {
      return rc;
    } 
  }

  // port 0 input/output direction
  val = readIOExp(HekIOExpCmdConfig0);

  if( val != HekIOExpConstConfig0 )
  {
    if( (rc = writeIOExp(HekIOExpCmdConfig0, HekIOExpConstConfig0)) < 0 )
    {
      return rc;
    } 
  }

#endif // TGT_EMBEDDED

  return HEK_OK;
}

int HekSysBoard::close()
{
  if( m_i2c.fd >= 0 )
  {
    i2c_close(&m_i2c);
    m_i2c.fd    = -1;
    m_i2c.addr  = 0;
  }

  return HEK_OK;
}

int HekSysBoard::scan()
{
  return HEK_OK;
}

int HekSysBoard::cmdReadFwVersion(int &ver)
{
  ver = 0;

  return HEK_OK;
}

byte_t HekSysBoard::cmdReadLimits()
{
  byte_t  cmd = HekIOExpCmdInput0;
  byte_t  val = 0;

#ifdef TGT_EMBEDDED
  if(i2c_transfer(&m_i2c, HekIOExpI2CAddr, &cmd, 1, &val, 1) < 0)
  {
    LOGSYSERROR("i2c_transfer: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, cmd);
  }
#endif // TGT_EMBEDDED

  return val;
}

byte_t HekSysBoard::cmdReadAux()
{
  byte_t  cmd = HekIOExpCmdInput1;
  byte_t  val = 0;

#ifdef TGT_EMBEDDED
  if(i2c_transfer(&m_i2c, HekIOExpI2CAddr, &cmd, 1, &val, 1) < 0)
  {
    LOGSYSERROR("i2c_transfer: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, cmd);
  }
#endif // TGT_EMBEDDED

  return val;
}

int HekSysBoard::cmdReadPin(int id)
{
  byte_t  cmd;
  int     shift;
  byte_t  mask;
  byte_t  bits = 0;
  int     val = 0;

  if( id < 8 )
  {
    cmd = HekIOExpCmdInput0;
    shift = id;
  }
  else
  {
    cmd = HekIOExpCmdInput1;
    shift = id - 8;
  }

  mask = 0x01 << shift;

#ifdef TGT_EMBEDDED
  if(i2c_transfer(&m_i2c, HekIOExpI2CAddr, &cmd, 1, &bits, 1) < 0)
  {
    LOGSYSERROR("i2c_transfer: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, cmd);
  }
  else if( bits & mask )
  {
    val = 1;
  }
#endif // TGT_EMBEDDED

  return val;
}

int HekSysBoard::cmdWritePin(int id, int val)
{
  byte_t  cmd;
  int     shift;
  byte_t  mask;
  byte_t  bits;
  byte_t  buf[2];

  if( id < 8 )
  {
    cmd = HekIOExpCmdInput0;
    shift = id;
    bits = cmdReadLimits();
  }
  else
  {
    cmd = HekIOExpCmdInput0;
    shift = id;
    bits = cmdReadAux();
  }

  mask = 0x01 << shift;
  bits = (bits & ~mask) | (val & mask);

  buf[0] = cmd;
  buf[1] = bits;

#ifdef TGT_EMBEDDED
  if(i2c_write(&m_i2c, HekIOExpI2CAddr, buf, 2) < 0)
  {
    LOGSYSERROR("i2c_write: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, cmd);
    return -HEK_ECODE_SYS;
  }
#endif // TGT_EMBEDDED

  return HEK_OK;

}

int HekSysBoard::cmdConfigPin(int id, char dir)
{
  // TODO
  return HEK_OK;
}

int HekSysBoard::cmdSetAlarmLED(int val)
{
  // TODO overio gpio
  return HEK_OK;
}

int HekSysBoard::cmdSetStatusLED(int val)
{
  // not supported
  return HEK_OK;
}

int HekSysBoard::cmdSetHaltingState()
{
  // not supported
  return HEK_OK;
}

byte_t HekSysBoard::readIOExp(byte_t byCmd)
{
  byte_t  byVal = 0;

#ifdef TGT_EMBEDDED
  if(i2c_transfer(&m_i2c, HekIOExpI2CAddr, &byCmd, 1, &byVal, 1) < 0)
  {
    LOGSYSERROR("i2c_transfer: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, byCmd);
  }
#endif // TGT_EMBEDDED

  return byVal;
}

int HekSysBoard::writeIOExp(byte_t byCmd, byte_t byVal)
{
  byte_t  buf[2];

  buf[0] = byCmd;
  buf[1] = byVal;

#ifdef TGT_EMBEDDED
  if(i2c_write(&m_i2c, HekIOExpI2CAddr, buf, 2) < 0)
  {
    LOGSYSERROR("i2c_write: %s: addr=0x%.02x, command=%u.",
        HekI2CDevice, HekIOExpI2CAddr, byCmd);
    return -HEK_ECODE_SYS;
  }
#endif // TGT_EMBEDDED

  return HEK_OK;
}



