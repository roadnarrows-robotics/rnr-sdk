////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeGpio.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps GPIO class implementations.
 * support functions.
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

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/gpio.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeGpio.h"


using namespace std;
using namespace laelaps;


// -----------------------------------------------------------------------------
// LaeGpio Class
// -----------------------------------------------------------------------------
  
LaeGpio::LaeGpio(const std::string       &strTag,
                 const int                gpio,
                 const LaeGpio::Direction dir) :
    m_gpioTag(strTag), m_gpioNum(gpio), m_gpioDir(dir)
{
  m_gpioVal = LaeGpio::UNKNOWN;
  m_gpioCfg = false;
}

void LaeGpio::sync()
{
  m_gpioCfg = checkConfig();

  if( isConfigured() )
  {
    int value;
    readValue(value);
  }
}

int LaeGpio::writeValue(const int value)
{
  int   rc;
  int   val = value? 1: 0;

  if( gpioWrite(m_gpioNum, val) == OK )
  {
    m_gpioVal = val;
    rc = LAE_OK;

    LOGDIAG3("%s GPIO %d: new value is %d.",
        m_gpioTag.c_str(), m_gpioNum, m_gpioVal);
  }
  else
  {
    LOGERROR("%s GPIO %d: Failed to write %d to pin.",
        m_gpioTag.c_str(), m_gpioNum, value);
    rc = -LAE_ECODE_IO;
  }

  return rc;
}

int LaeGpio::readValue(int &value)
{
  int   val;
  int   rc;

  if( (val = gpioRead(m_gpioNum)) != RC_ERROR )
  {
    value = val;
    m_gpioVal = val;
    rc = LAE_OK;

    LOGDIAG3("%s GPIO %d: current value is %d.",
        m_gpioTag.c_str(), m_gpioNum, m_gpioVal);
  }
  else
  {
    LOGERROR("%s GPIO %d: Failed to read pin.", m_gpioTag.c_str(), m_gpioNum);
    rc = -LAE_ECODE_IO;
  }

  return rc;
}

bool LaeGpio::checkConfig()
{
  char  buf[MAX_PATH];
  int   mode;

  gpioMakeDirname(m_gpioNum, buf, sizeof(buf));

  string strValFileName(buf);
  string strDirFileName(buf);
  
  strValFileName += "/value";
  strDirFileName += "/direction";

  if( m_gpioDir == LaeGpio::INPUT )
  {
    mode = R_OK;
  }
  else
  {
    mode = R_OK|W_OK;
  }

  return access(strValFileName.c_str(), mode) == 0;
}


// -----------------------------------------------------------------------------
// LaeMotorCtlrEnable Class
// -----------------------------------------------------------------------------

void LaeMotorCtlrEnable::sync()
{
  LaeGpio::sync();

  // version 2.0 has fixed power to motor controllers
  if( !isConfigured() )
  {
    m_gpioVal == LaeGpio::HIGH;
  }
}

bool LaeMotorCtlrEnable::enable()
{
  bool  wasEnabled;
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have power enable to motor controllers
  if( isConfigured() )
  {
    wasEnabled = isEnabled();

    writeValue(1);

    // give the motor controllers time to power up
    if( !wasEnabled && isEnabled() )
    {
      usleep(TPowerUp);
    }
  }

  bEnabled = isEnabled();

  if( bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) enabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeMotorCtlrEnable::disable()
{
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have power enable to motor controllers
  if( isConfigured() )
  {
    writeValue(0);
  }

  bEnabled = isEnabled();

  if( !bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) disabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeMotorCtlrEnable::isEnabled()
{
  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  return hasValue() == LaeGpio::HIGH;
}


// -----------------------------------------------------------------------------
// LaeWatchDogReset Class
// -----------------------------------------------------------------------------

void LaeWatchDogReset::sync()
{
  LaeGpio::sync();

  // version 2.0 does not have a reset line
  if( !isConfigured() )
  {
    m_gpioVal == LaeGpio::LOW;
  }
}

void LaeWatchDogReset::reset()
{
  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have reset line to watchdog sub-processor
  if( isConfigured() )
  {
    // transition high to low
    if( writeValue(1) == LAE_OK )
    {
      usleep(TTrans);
      writeValue(0);
      usleep(TTrans);

      if( hasValue() == LaeGpio::LOW )
      {
        usleep(TReboot);
      }
    }
  }
}


// -----------------------------------------------------------------------------
// LaeI2CMuxReset Class
// -----------------------------------------------------------------------------

void LaeI2CMuxReset::sync()
{
  LaeGpio::sync();

  // version 2.0 does not have a reset line
  if( !isConfigured() )
  {
    m_gpioVal == LaeGpio::LOW;
  }
}

void LaeI2CMuxReset::reset()
{
  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have reset line to watchdog sub-processor
  if( isConfigured() )
  {
    // transition high to low
    if( writeValue(1) == LAE_OK )
    {
      usleep(TTrans);
      writeValue(0);
      usleep(TTrans);

      if( hasValue() == LaeGpio::LOW )
      {
        usleep(TReboot);
      }
    }
  }
}


// -----------------------------------------------------------------------------
// LaeAuxBattOutEnable Class
// -----------------------------------------------------------------------------

void LaeAuxBattOutEnable::sync()
{
  LaeGpio::sync();

  // version 2.0 has fixed battery power to top deck
  if( !isConfigured() )
  {
    m_gpioVal == LaeGpio::HIGH;
  }
}

bool LaeAuxBattOutEnable::enable()
{
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have battery power enable to top deck
  if( isConfigured() )
  {
    writeValue(1);
  }

  bEnabled = isEnabled();

  if( bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) enabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeAuxBattOutEnable::disable()
{
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have battery power enable to top deck
  if( isConfigured() )
  {
    writeValue(0);
  }

  bEnabled = isEnabled();

  if( !bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) disabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeAuxBattOutEnable::isEnabled()
{
  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  return hasValue() == LaeGpio::HIGH;
}


// -----------------------------------------------------------------------------
// LaeAux5VOutEnable Class
// -----------------------------------------------------------------------------

void LaeAux5VOutEnable::sync()
{
  LaeGpio::sync();

  // version 2.0 has fixed regulated 5V out to top deck
  if( !isConfigured() )
  {
    m_gpioVal == LaeGpio::HIGH;
  }
}

bool LaeAux5VOutEnable::enable()
{
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have regulated 5V power enable to top deck
  if( isConfigured() )
  {
    writeValue(1);
  }

  bEnabled = isEnabled();

  if( bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) enabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeAux5VOutEnable::disable()
{
  bool  bEnabled;

  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  // versions 2.1+ have regulated 5V power enable to top deck
  if( isConfigured() )
  {
    writeValue(0);
  }

  bEnabled = isEnabled();

  if( !bEnabled )
  {
    LOGDIAG2("%s (GPIO %d) disabled.", m_gpioTag.c_str(), m_gpioNum);
  }

  return bEnabled;
}

bool LaeAux5VOutEnable::isEnabled()
{
  // lazy one-time synchronization
  if( m_gpioVal == LaeGpio::UNKNOWN )
  {
    sync();
  }

  return hasValue() == LaeGpio::HIGH;
}
