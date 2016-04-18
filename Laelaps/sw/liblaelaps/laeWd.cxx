////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeWd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps WatchDog software class implementation.
 *
 * The class provides the interface between the library software and the
 * Arduino sub-processor.
 *
 * The Wd class is safe multi-threading.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
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

#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/Time.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeDb.h"
#include  "Laelaps/laeI2C.h"
#include  "Laelaps/laeWatchDog.h"
#include  "Laelaps/laeWd.h"

using namespace std;
using namespace laelaps;

#undef WD_DBG_ENABLE  ///< debug switch
#ifdef WD_DBG_ENABLE

/*!
 * \brief Debug print response buffer
 * \param cmd Command number.
 * \param buf Buffer to print.
 * \param len Buffer length.
 * \param fmt Format string.
 * \param ... Optional variable arguments for fmt.
 */
#define WD_DBG_BUF(cmd, buf, len, fmt, ...) \
  do \
  { \
    fprintf(stderr, "DBG: %s[%d]: Wd: Cmd=%d " fmt, \
        __FILE__, __LINE__, cmd, ##__VA_ARGS__); \
    for(int i=0;i<len; ++i) { fprintf(stderr, "0x%02x ", buf[i]); } \
    fprintf(stderr, "\n"); \
  } while(0)

#else

/*!
 * \brief Debug print disabled.
 */
#define WD_DBG_BUF(cmd, buf, len, fmt, ...)

#endif // WD_DBG_ENABLE

//
// Times
//
static long TStd = 100;     ///< standard wait time between write_read (usec)
static long TMc  = 250000;  ///< motor controller power up wait time (usec)


//------------------------------------------------------------------------------
// LaeWd Class
//------------------------------------------------------------------------------
  
LaeWd::LaeWd(LaeI2C &i2cBus, uint_t addr) :
    m_i2cBus(i2cBus), m_addrSubProc(addr)
{
  // shadow values
  m_uFwVer              = 0;
  m_bBatteryIsCharging  = false;
  m_fBatteryVoltage     = 0.0;
  m_fJackVoltage        = 0.0;
  m_uBatterySoC         = LaeWdArgBattChargeMax;
  m_uAlarms             = LaeWdArgAlarmNone;
  m_bMotorCtlrEn        = false;
  m_bAuxPortBattEn      = false;
  m_bAuxPort5vEn        = false; 

  RtDb.m_energy.m_bBatteryIsCharging  = m_bBatteryIsCharging;
  RtDb.m_energy.m_fJackVoltage        = m_fJackVoltage;
  RtDb.m_gpio.m_bMotorCtlrEn          = m_bMotorCtlrEn;
  RtDb.m_gpio.m_bAuxPortBattEn        = m_bAuxPortBattEn;
  RtDb.m_gpio.m_bAuxPort5vEn          = m_bAuxPort5vEn;

  pthread_mutex_init(&m_mutex, NULL);
}

LaeWd::~LaeWd()
{
  pthread_mutex_destroy(&m_mutex);
}

void LaeWd::sync()
{
  uint_t  uFwVer;
  double  fJackV, fBattV;

  cmdGetFwVersion(uFwVer);            // must be first
  cmdEnableMotorCtlrs(false);
  cmdEnableAuxPort5V(true);
  cmdEnableAuxPortBatt(true);
  cmdReadVoltages(fJackV, fBattV);
  cmdSetBatterySoC(LaeWdArgBattChargeMax);
  cmdSetAlarms(LaeWdArgAlarmNone);
  cmdResetRgbLed();
  cmdPetTheDog();
}

int LaeWd::configure(const LaeTunes &tunes)
{
  return LAE_OK;
}

int LaeWd::reload(const LaeTunes &tunes)
{
  return LAE_OK;
}

void LaeWd::exec()
{
  uint_t  uBattSoC;
  uint_t  uAlarms;
  double  fJackV, fBattV;
  bool    bMotorCtlrEn, bAuxPort5vEn, bAuxPortBattEn;

  cmdPetTheDog();

  cmdReadVoltages(fJackV, fBattV);
  cmdReadEnables(bMotorCtlrEn, bAuxPort5vEn, bAuxPortBattEn);

  uBattSoC = (uint_t)RtDb.m_energy.m_fBatterySoC;

  if( uBattSoC != m_uBatterySoC )
  {
    cmdSetBatterySoC(uBattSoC);
  }

  uAlarms = determineAlarms();

  if( uAlarms != m_uAlarms )
  {
    cmdSetAlarms(uAlarms);
  }
}

int LaeWd::cmdPetTheDog()
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp;
  bool    bBatteryIsCharging;
  int     n;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdPetDog;

  // version 1
  if( m_uFwVer <= 1 )
  {
    n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

    rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;
  }

  // versions 2+
  else
  {
    lenRsp = LaeWdRspLenPetDog_2;

    rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

    if( rc == LAE_OK )
    {
      bBatteryIsCharging = rsp[0] == LaeWdArgDPinValHigh? true: false;
      m_bBatteryIsCharging = bBatteryIsCharging;
      RtDb.m_energy.m_bBatteryIsCharging = m_bBatteryIsCharging;
    }
  }

  unlock();

  return rc;
}

int LaeWd::cmdGetFwVersion(uint_t &uVerNum)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenGetVersion;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdGetVersion;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    uVerNum = (uint_t)rsp[0];
    m_uFwVer = uVerNum;
    RtDb.m_product.m_uWatchDogFwVer = m_uFwVer;
  }

  unlock();

  return rc;
}

int LaeWd::cmdSetBatterySoC(uint_t uBatterySoC)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  if( uBatterySoC > LaeWdArgBattChargeMax )
  {
    uBatterySoC = LaeWdArgBattChargeMax;
  }

  cmd[lenCmd++] = LaeWdCmdIdSetBattCharge;
  cmd[lenCmd++] = (byte_t)uBatterySoC;

  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  if( n == (int)lenCmd )
  {
    m_uBatterySoC = uBatterySoC;
    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdSetAlarms(uint_t uAlarms)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdSetAlarms;
  cmd[lenCmd++] = (byte_t)((uAlarms >> 8) & 0xff);
  cmd[lenCmd++] = (byte_t)(uAlarms & 0xff);

  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  if( n == (int)lenCmd )
  {
    m_uAlarms = uAlarms;
    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdSetRgbLed(uint_t red, uint_t green, uint_t blue)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdSetRgbLed;
  cmd[lenCmd++] = (byte_t)red;
  cmd[lenCmd++] = (byte_t)green;
  cmd[lenCmd++] = (byte_t)blue;

  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;

  unlock();

  return rc;
}

int LaeWd::cmdResetRgbLed()
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdResetRgbLed;

  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;

  unlock();

  return rc;
}

int LaeWd::cmdConfigDPin(uint_t pin, uint_t dir)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  if( (pin < LaeWdArgDPinNumWMin) || (pin > LaeWdArgDPinNumWMax) )
  {
    LOGERROR("Pin %u: Out-of-range.", pin);
    rc = -LAE_ECODE_BAD_VAL;
  }

  else
  {
    dir = dir > 0? LaeWdArgDPinDirOut: LaeWdArgDPinDirIn;

    cmd[lenCmd++] = LaeWdCmdIdConfigDPin;
    cmd[lenCmd++] = (byte_t)pin;
    cmd[lenCmd++] = (byte_t)dir;

    n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

    rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdReadDPin(uint_t pin, uint_t &val)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenReadDPin;
  int     rc;

  lock();

  if( (pin < LaeWdArgDPinNumMin) || (pin > LaeWdArgDPinNumMax) )
  {
    LOGERROR("Pin %u: Out-of-range.", pin);
    rc = -LAE_ECODE_BAD_VAL;
  }

  else
  {
    cmd[lenCmd++] = LaeWdCmdIdReadDPin;
    cmd[lenCmd++] = (byte_t)pin;

    rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);
  }

  if( rc == LAE_OK )
  {
    if( (uint_t)rsp[0] != pin )
    {
      WD_DBG_BUF(LaeWdCmdIdReadDPin, rsp, n, "pin=%d rsp=", pin);
      LOGERROR("Response pin %u != pin %u.", (uint_t)rsp[0], pin);
      rc = -LAE_ECODE_BAD_VAL;
    }
    else
    {
      val = (uint_t)rsp[1];
    }
  }

  unlock();

  return rc;
}

int LaeWd::cmdWriteDPin(uint_t pin, uint_t val)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  if( (pin < LaeWdArgDPinNumWMin) || (pin > LaeWdArgDPinNumWMax) )
  {
    LOGERROR("Pin %u: Out-of-range.", pin);
    rc = -LAE_ECODE_BAD_VAL;
  }

  else
  {
    val = val > 0? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;

    cmd[lenCmd++] = LaeWdCmdIdWriteDPin;
    cmd[lenCmd++] = (byte_t)pin;
    cmd[lenCmd++] = (byte_t)val;

    n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

    rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdReadAPin(uint_t pin, uint_t &val)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenReadAPin;
  int     rc;

  lock();

  if( (pin < LaeWdArgAInPinNumMin) || (pin > LaeWdArgAInPinNumMax) )
  {
    LOGERROR("Pin %u: Out-of-range.", pin);
    rc = -LAE_ECODE_BAD_VAL;
  }

  else
  {
    cmd[lenCmd++] = LaeWdCmdIdReadAPin;
    cmd[lenCmd++] = (byte_t)pin;

    rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);
  }

  if( rc == LAE_OK )
  {
    if( (uint_t)rsp[0] != pin )
    {
      WD_DBG_BUF(LaeWdCmdIdReadAPin, rsp, n, "pin=%d rsp=", pin);
      LOGERROR("Response pin %u != pin %u.", (uint_t)rsp[0], pin);
      rc = -LAE_ECODE_BAD_VAL;
    }
    else
    {
      val = (uint_t)((((uint_t)rsp[1]) << 8) | (((uint_t)rsp[2]) & 0xff));
    }
  }

  unlock();

  return rc;
}

int LaeWd::cmdWriteAPin(uint_t pin, uint_t val)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  lock();

  if( (pin < LaeWdArgAOutPinNumMin) || (pin > LaeWdArgAOutPinNumMax) )
  {
    LOGERROR("Pin %u: Out-of-range.", pin);
    rc = -LAE_ECODE_BAD_VAL;
  }

  else
  {
    if( val > LaeWdArgAOutPinValMax )
    {
      val = LaeWdArgAOutPinValMax;
    }

    cmd[lenCmd++] = LaeWdCmdIdWriteAPin;
    cmd[lenCmd++] = (byte_t)pin;
    cmd[lenCmd++] = (byte_t)val;

    n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

    rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdEnableMotorCtlrs(bool bEnable)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenEnableMotorCtlrs;
  bool    bPass;
  int     nTries;
  int     nMaxTries = 3;
  int     rc;

  // old version had no hardware control to enable power
  if( m_uFwVer == 0 ) // unknown
  {
    return LAE_OK;
  }
  else if( m_uFwVer == 1 ) // always enabled
  {
    m_bMotorCtlrEn              = true;
    RtDb.m_gpio.m_bMotorCtlrEn  = m_bMotorCtlrEn;
    return LAE_OK;
  }

  lock();

  cmd[lenCmd++] = LaeWdCmdIdEnableMotorCtlrs;
  cmd[lenCmd++] = bEnable? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;

  //
  // Really try to set the motor controller power in state.
  //
  for(nTries = 0, bPass = false; (nTries < nMaxTries) && !bPass; ++nTries)
  {
    rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, 5000);

    if( (rc == LAE_OK) && (rsp[0] == LaeWdArgPass) )
    {
      bPass = true;
    }
    else
    {
      usleep(50);
    }
  }

  // Success.
  if( bPass )
  {
    // give time for motor controllers to power up
    if( bEnable )
    {
      usleep(TMc);

      // disable to enable
      if( !m_bMotorCtlrEn )
      {
        m_timeMotorCtlrs.markNow();
      }
    }
    // enable to disable
    else if( m_bMotorCtlrEn )
    {
      double t = m_timeMotorCtlrs.now();
      LOGDIAG1("Motor controllers uptime: %.4lfs.", t - m_timeMotorCtlrs.t());
    }

    m_bMotorCtlrEn              = bEnable;
    RtDb.m_gpio.m_bMotorCtlrEn  = m_bMotorCtlrEn;
  }

  // Failure.
  else
  {
    LOGERROR("Failed to %s motor controllers.", (bEnable? "enable": "disable"));
    rc = -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeWd::cmdEnableAuxPort5V(bool bEnable)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  // old version had no hardware control to enable power
  if( m_uFwVer < 2 )
  {
    return LAE_OK;
  }
  if( m_uFwVer == 0 ) // unknown
  {
    return LAE_OK;
  }
  else if( m_uFwVer == 1 ) // always enabled
  {
    m_bAuxPort5vEn              = true; 
    RtDb.m_gpio.m_bAuxPort5vEn  = m_bAuxPort5vEn;
    return LAE_OK;
  }


  lock();

  cmd[lenCmd++] = LaeWdCmdIdEnableAuxPort;
  cmd[lenCmd++] = LaeWdArgAuxPort5V;
  cmd[lenCmd++] = bEnable? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;

  // enable/disable aux port
  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;

  if( rc == LAE_OK )
  {
    m_bAuxPort5vEn              = bEnable; 
    RtDb.m_gpio.m_bAuxPort5vEn  = m_bAuxPort5vEn;
  }

  unlock();

  return rc;
}

int LaeWd::cmdEnableAuxPortBatt(bool bEnable)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  size_t  lenCmd = 0;
  int     n;
  int     rc;

  // old version had no hardware control to enable power
  if( m_uFwVer == 0 ) // unknown
  {
    return LAE_OK;
  }
  else if( m_uFwVer == 1 ) // always enabled
  {
    m_bAuxPortBattEn              = true;
    RtDb.m_gpio.m_bAuxPortBattEn  = m_bAuxPortBattEn;
    return LAE_OK;
  }


  lock();

  cmd[lenCmd++] = LaeWdCmdIdEnableAuxPort;
  cmd[lenCmd++] = LaeWdArgAuxPortBatt;
  cmd[lenCmd++] = bEnable? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;

  // enable/disable aux port
  n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd);

  rc = (n == (int)lenCmd)? LAE_OK: -LAE_ECODE_IO;

  if( rc == LAE_OK )
  {
    m_bAuxPortBattEn              = bEnable;
    RtDb.m_gpio.m_bAuxPortBattEn  = m_bAuxPortBattEn;
  }

  unlock();

  return rc;
}

int LaeWd::cmdReadEnables(bool &bMotorCtlrEn,
                          bool &bAuxPort5vEn,
                          bool &bAuxPortBattEn)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenReadEnables;
  int     rc;

  // old version had no hardware enables
  if( m_uFwVer == 0 ) // unknown
  {
    return LAE_OK;
  }

  else if( m_uFwVer == 1 ) // always enabled
  {
    // return values
    bMotorCtlrEn    = true;
    bAuxPort5vEn    = true;
    bAuxPortBattEn  = true;

    // shadow
    m_bMotorCtlrEn    = bMotorCtlrEn;
    m_bAuxPort5vEn    = bAuxPort5vEn;
    m_bAuxPortBattEn  = bAuxPortBattEn;

    // real-time db
    RtDb.m_gpio.m_bMotorCtlrEn    = m_bMotorCtlrEn;
    RtDb.m_gpio.m_bAuxPort5vEn    = m_bAuxPort5vEn;
    RtDb.m_gpio.m_bAuxPortBattEn  = m_bAuxPortBattEn;

    return LAE_OK;
  }

  lock();

  cmd[lenCmd++] = LaeWdCmdIdReadEnables;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    // return values
    bMotorCtlrEn    = rsp[0] > 0? true: false;
    bAuxPort5vEn    = rsp[1] > 0? true: false;
    bAuxPortBattEn  = rsp[2] > 0? true: false;

    // disable to enable
    if( !m_bMotorCtlrEn && bMotorCtlrEn )
    {
      m_timeMotorCtlrs.markNow();
    }
    // enable to disable
    else if( m_bMotorCtlrEn && !bMotorCtlrEn )
    {
      double t = m_timeMotorCtlrs.now();
      LOGDIAG1("Motor controllers uptime: %.4lfs.", t - m_timeMotorCtlrs.t());
    }

    // shadow
    m_bMotorCtlrEn    = bMotorCtlrEn;
    m_bAuxPort5vEn    = bAuxPort5vEn;
    m_bAuxPortBattEn  = bAuxPortBattEn;

    // real-time db
    RtDb.m_gpio.m_bMotorCtlrEn    = m_bMotorCtlrEn;
    RtDb.m_gpio.m_bAuxPort5vEn    = m_bAuxPort5vEn;
    RtDb.m_gpio.m_bAuxPortBattEn  = m_bAuxPortBattEn;
  }

  unlock();

  return rc;
}

int LaeWd::cmdReadVoltages(double &fJackV, double &fBattV)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenReadVolts;
  int     rc;

  // old version had no hardware to sense voltages
  if( m_uFwVer < 2 )
  {
    fJackV = 0.0;
    fBattV = 0.0;
    return LAE_OK;
  }

  lock();

  cmd[lenCmd++] = LaeWdCmdIdReadVolts;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    fJackV = (double)(rsp[0]) * LaeWdArgVScale;
    fBattV = (double)(rsp[1]) * LaeWdArgVScale;
  }

  m_fBatteryVoltage = fBattV;
  m_fJackVoltage    = fJackV;

  RtDb.m_energy.m_fBatteryVoltage = m_fBatteryVoltage;
  RtDb.m_energy.m_fJackVoltage    = m_fJackVoltage;

  unlock();

  return rc;
}

int LaeWd::cmdTest(uint_t &uSeqNum,
                   uint_t &uOpState,
                   uint_t &uAlarms,
                   uint_t &uLedIndex)
{
  byte_t  cmd[LaeWdMaxCmdLen];
  byte_t  rsp[LaeWdMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeWdRspLenTest;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeWdCmdIdTest;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    uSeqNum   = (uint_t)rsp[0];
    uOpState  = (uint_t)rsp[1];
    uAlarms   = (uint_t)((((uint_t)rsp[2]) << 8) | (((uint_t)rsp[3]) & 0xff));
    uLedIndex = (uint_t)rsp[4];
  }

  unlock();

  return rc;
}

uint_t LaeWd::determineAlarms()
{
  uint_t  uAlarms = LaeWdArgAlarmNone;

  //
  // Alarms.
  //
  if( RtDb.m_alarms.m_system.m_uAlarms & LAE_ALARM_BATT )
  {
    uAlarms |= LaeWdArgAlarmBatt;
  }
  if( RtDb.m_alarms.m_system.m_uAlarms & LAE_ALARM_TEMP )
  {
    uAlarms |= LaeWdArgAlarmTemp;
  }
  if( RtDb.m_alarms.m_system.m_uAlarms & LAE_ALARM_ESTOP )
  {
    uAlarms |= LaeWdArgAlarmEStop;
  }
  if( RtDb.m_alarms.m_system.m_uAlarms & ~uAlarms )
  {
    uAlarms |= LaeWdArgAlarmGen;
  }

  //
  // Critical.
  //
  if( RtDb.m_alarms.m_system.m_bIsCritical &&
      (uAlarms & ~((uint_t)LaeWdArgAlarmBatt)) )
  {
    uAlarms |= LaeWdArgAlarmCrit;
  }
  if( RtDb.m_alarms.m_battery.m_bIsCritical )
  {
    uAlarms |= LaeWdArgAlarmBattCrit;
  }

  return uAlarms;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static functions.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeWd::enableMotorCtlrs(void *pArg, bool bEnable)
{
  return ((LaeWd *)pArg)->cmdEnableMotorCtlrs(bEnable);
}