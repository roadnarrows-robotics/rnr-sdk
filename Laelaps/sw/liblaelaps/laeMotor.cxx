////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeMotor.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-10-02 15:17:27 -0600 (Fri, 02 Oct 2015) $
 * $Rev: 4130 $
 *
 * \brief Laelaps motors, encoder, and controllers hardware abstraction
 * support functions.
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

#include <sys/types.h>
#include <unistd.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/gpio.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeMotor.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;

// -----------------------------------------------------------------------------
// LaeMotorCtlrChipSelect Class
// -----------------------------------------------------------------------------
  
LaeMotorCtlrChipSelect::LaeMotorCtlrChipSelect()
{
  m_pinGpio = -1;
  m_fdGpio  = -1;
}

LaeMotorCtlrChipSelect::~LaeMotorCtlrChipSelect()
{
  close();
}

int LaeMotorCtlrChipSelect::open(int pinGpio)
{
  if( (m_fdGpio = gpioOpen(pinGpio)) < 0 )
  {
    LOGERROR("Failed to open motor controllers chip select on GPIO pin %d.",
        pinGpio);

    return -LAE_ECODE_NO_RSRC;
  }

  m_pinGpio = pinGpio;

  return LAE_OK;
}

int LaeMotorCtlrChipSelect::close()
{
  int   rc;

  if( m_fdGpio >= 0 )
  {
    rc = gpioClose(m_fdGpio);
  }

  m_pinGpio = -1;
  m_fdGpio  = -1;

  return rc == OK? LAE_OK: -LAE_ECODE_IO;
}

void LaeMotorCtlrChipSelect::select(int fd, byte_t addrSel)
{
  int   value;

  if( addrSel != m_addrLast )
  {
    value = addrSel == LaeMotorCtlrCsHigh? 1: 0; 
    if( gpioQuickWrite(m_fdGpio, value) < 0 )
    {
      LOGWARN("Failed to select motor controller 0x%02x.", addrSel);
    }
    usleep(100);

    RoboClawChipSelect::select(fd, addrSel);
  }
}
