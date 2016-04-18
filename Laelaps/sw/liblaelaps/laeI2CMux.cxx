////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeI2CMux.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps PCA9548A I2C multiplexer switch implementation.
 *
 * The multiplexer allows access to multiple \h_i2c devices including those
 * that have the same fixed address.
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

#include <unistd.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeI2C.h"
#include  "Laelaps/laeI2CMux.h"

using namespace std;
using namespace laelaps;

LaeI2CMux::LaeI2CMux(LaeI2C &i2cBus, uint_t addr) :
    m_i2cBus(i2cBus), m_addrMux(addr)
{
  m_chan = LaeI2CMuxChanNone;

  pthread_mutex_init(&m_mutex, NULL);
}

LaeI2CMux::~LaeI2CMux()
{
  pthread_mutex_destroy(&m_mutex);
}

int LaeI2CMux::read(int chan, uint_t addr, byte_t buf[], size_t len)
{
  int   n;
  int   rc;

  lock();

  if( (rc = setChannel(chan)) == LAE_OK )
  {
    if( (n = m_i2cBus.read(addr, buf, len)) < 0 )
    {
      LOGDIAG3("I2C Mux: Channel %d I2C device address 0x%02x: "
          "Failed to read from endpoint device data.",
          m_chan, addr);
      rc = -LAE_ECODE_IO;
    }

    if( n != (int)len )
    {
      LOGWARN("I2C Mux: Channel %d I2C device address 0x%02x: "
          "Only %d/%d bytes read from endpoint device.",
          m_chan, addr, n, (int)len);
    }
  }

  unlock();

  return rc < 0? rc: n;
}

int LaeI2CMux::write(int chan, uint_t addr, byte_t buf[], size_t len)
{
  int   n;
  int   rc;

  lock();

  if( (rc = setChannel(chan)) == LAE_OK )
  {
    if( (n = m_i2cBus.write(addr, buf, len)) < 0 )
    {
      LOGDIAG3("I2C Mux: Channel %d I2C device address 0x%02x: "
          "Failed to write to device data.",
          m_chan, addr);
      rc = -LAE_ECODE_IO;
    }

    else if( n != (int)len )
    {
      LOGWARN("I2C Mux: Channel %d I2C device address 0x%02x: "
          "Only %d/%d bytes written to endpoint device.",
          m_chan, addr, n, (int)len);
    }
  }

  unlock();

  return rc < 0? rc: n;
}

int LaeI2CMux::transact(int chan, uint_t addr,
                        const byte_t wbuf[], size_t wlen,
                        byte_t rbuf[], size_t rlen,
                        long usec)
{
  int   n;
  int   rc;

  lock();

  if( (rc = setChannel(chan)) == LAE_OK )
  {
    rc = m_i2cBus.write_read(addr, wbuf, wlen, rbuf, rlen, usec);

    if( rc != LAE_OK )
    {
      LOGDIAG3("I2C Mux: Channel %d I2C device address 0x%02x: "
          "Failed to write_read to device data.",
          m_chan, addr);
    }
  }

  unlock();

  return rc < 0? rc: n;
}

int LaeI2CMux::readChannelStates(byte_t &chanBits)
{
  int   rc;

  lock();

  if( m_i2cBus.read(m_addrMux, &chanBits, 1) < 0 )
  {
    LOGSYSERROR("I2C Mux: Address 0x%02x: Failed to read channel states.",
          m_addrMux);
      rc = -LAE_ECODE_IO;
  }
  else
  {
    rc = LAE_OK;
  }

  unlock();

  return rc;
}

int LaeI2CMux::setChannel(int chan)
{
  if( (chan < LaeI2CMuxChanMin) || (chan > LaeI2CMuxChanMax) )
  {
    LOGERROR("I2C Mux: %d channel: Out-of-range.", chan);
    return -LAE_ECODE_BAD_VAL;
  }

  else if( m_chan != chan )
  {
    byte_t  ctl = (byte_t)(1 << chan);    // control register data

    if( m_i2cBus.write(m_addrMux, &ctl, 1) < 0 )
    {
      LOGSYSERROR("I2C Mux: Address 0x%02x: Failed to select channel.",
          m_addrMux);
      return -LAE_ECODE_IO;
    }
    else
    {
      m_chan = chan;
      usleep(100);
    }
  }

  return LAE_OK;
}
