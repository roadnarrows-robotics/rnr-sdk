////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeI2C.cxx
//
/*! \file
 *
 * \brief Laelaps I2C class implementation.
 *
 * The I2C class supports safe multi-threading.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#include <unistd.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/i2c.h"
#include "rnr/log.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/laeI2C.h"

using namespace std;
using namespace laelaps;

//------------------------------------------------------------------------------
// I2C Utilities
//------------------------------------------------------------------------------

int laelaps::i2cTryOpen(LaeI2C &i2cBus, uint_t addr)
{
  const char *devNames[] = { LaeDevI2C_0, LaeDevI2C_1, LaeDevI2C_2, NULL };

  int   rc;

  for(int i = 0; devNames[i] != NULL; ++i)
  {
    LOGDIAG2("Try I2C device %s.", devNames[i]);

    if( (rc = i2cBus.open(devNames[i])) == LAE_OK )
    {
      if( i2cBus.check(addr) )
      {
        LOGDIAG2("Address 0x%02x found on I2C device %s.", addr, devNames[i]);
        return LAE_OK;
      }
      else
      {
        LOGDIAG2("No address 0x%02x found on I2C device %s.",
            addr, devNames[i]);
        i2cBus.close();
      }
    }
  }
  rc = -LAE_ECODE_NO_DEV;
}

//------------------------------------------------------------------------------
// LaeI2C Class
//------------------------------------------------------------------------------

/*! \brief No bound \h_i2c bus. */
static i2c_t I2CBusNone = { -1, I2C_ADDR_NONE};

LaeI2C::LaeI2C() :
  m_i2c(I2CBusNone)
{
  pthread_mutex_init(&m_mutex, NULL);
}

LaeI2C::~LaeI2C()
{
  close();
  pthread_mutex_destroy(&m_mutex);
}

int LaeI2C::open(const string &strDevName)
{
  string  strDevNameReal;
  int     rc;

  lock();

  // Get the real device name, not any symbolic links.
  strDevNameReal  = getRealDeviceName(strDevName);

  if( i2c_open(&m_i2c, strDevNameReal.c_str()) < 0 )
  {
    LOGSYSERROR("%s.", strDevNameReal.c_str());
    rc = -LAE_ECODE_NO_DEV;
  }

  else
  {
    m_strDevName = strDevNameReal;

    LOGDIAG2("I2C device %s opened.", m_strDevName.c_str());

    rc = LAE_OK;
  }

  unlock();

  return rc;
}

int LaeI2C::close()
{
  lock();

  if( m_i2c.fd >= 0 )
  {
    i2c_close(&m_i2c);
    LOGDIAG3("I2C device %s closed.", m_strDevName.c_str());
  }

  m_strDevName.clear();
  m_i2c = I2CBusNone;

  unlock();

  return LAE_OK;
}

bool LaeI2C::check(uint_t addr)
{
  return i2c_exists(&m_i2c, (i2c_addr_t)addr) == 1;
}

int LaeI2C::read(uint_t addr, byte_t buf[], size_t len)
{
  int   n;

  lock();

  if( (n = i2c_read(&m_i2c, (i2c_addr_t)addr, buf, (uint_t)len)) < 0 )
  {
    // LOGSYSERROR
    LOGDIAG3("I2C: Slave device address 0x%02x: "
          "Failed to read from endpoint device data.",
          addr);
    n = -LAE_ECODE_IO;
  }

  else if( n != (int)len )
  {
    LOGWARN("I2C: Slave device address 0x%02x: "
          "Only %d/%d bytes read from endpoint device.",
          addr, n, (int)len);
  }

  unlock();

  return n;
}

int LaeI2C::write(uint_t addr, const byte_t buf[], size_t len)
{
  int   n;

  lock();

  if( (n = i2c_write(&m_i2c, (i2c_addr_t)addr, buf, (uint_t)len)) < 0 )
  {
    // LOGSYSERROR
    LOGDIAG3("I2C: Slave device address 0x%02x: "
          "Failed to write to device data.",
          addr);
    n = -LAE_ECODE_IO;
  }

  else if( n != (int)len )
  {
    LOGWARN("I2C: Slave device address 0x%02x: "
          "Only %d/%d bytes written to endpoint device.",
          addr, n, (int)len);
  }

  unlock();

  return n;
}

int LaeI2C::transfer(uint_t addr, const byte_t wbuf[], size_t wlen,
                                  byte_t rbuf[], size_t rlen)
{
  int   rc;

  lock();

  rc = i2c_transfer(&m_i2c, (i2c_addr_t)addr, wbuf, (uint_t)wlen,
                                              rbuf, (uint_t)rlen);

  if( rc < 0 )
  {
    LOGSYSERROR("I2C: Slave device address 0x%02x: "
          "Failed to perform write/read transaction.",
          addr);
    rc = -LAE_ECODE_IO;
  }

  else
  {
    rc = LAE_OK;
  }

  unlock();

  return rc;
}

int LaeI2C::write_read(uint_t addr, const byte_t wbuf[], size_t wlen,
                                   byte_t rbuf[], size_t rlen,
                                   long usec)
{
  int   n;
  int   rc = LAE_OK;

  lock();

  //
  // Write.
  //
  if( wlen > 0 )
  {
    if( (n = i2c_write(&m_i2c, (i2c_addr_t)addr, wbuf, (uint_t)wlen)) < 0 )
    {
      // LOGSYSERROR
      LOGDIAG3("I2C: Slave device address 0x%02x: "
          "Failed to write to device data.",
          addr);
      rc = -LAE_ECODE_IO;
    }

    else if( n != (int)wlen )
    {
      LOGERROR("I2C: Slave device address 0x%02x: "
          "Only %d/%d bytes written to endpoint device.",
          addr, n, (int)wlen);
      rc = -LAE_ECODE_IO;
    }
  }

  //
  // Read.
  //
  if( (rlen > 0) && (rc == LAE_OK) )
  {
    // delay
    if( usec > 0 )
    {
      usleep(usec);
    }

    if( (n = i2c_read(&m_i2c, (i2c_addr_t)addr, rbuf, (uint_t)rlen)) < 0 )
    {
      // LOGSYSERROR
      LOGDIAG3("I2C: Slave device address 0x%02x: "
          "Failed to read from endpoint device data.",
          addr);
      rc = -LAE_ECODE_IO;
    }

    else if( n != (int)rlen )
    {
      LOGERROR("I2C: Slave device address 0x%02x: "
          "Only %d/%d bytes read from endpoint device.",
          addr, n, (int)rlen);
      rc = -LAE_ECODE_IO;
    }
  }

  unlock();

  return rc;
}
