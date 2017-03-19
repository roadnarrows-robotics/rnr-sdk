////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekSysBoard.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief \h_hek original system board.
 *
 * \note The original system board is deprecated. However a newer version may
 * be resurected at a future time, so keep the code.
 *
 * \author Robin Knight    (robin.knight@roadnarrows.com)
 * \author Daniel Packard  (daniel@roadnarrows.com)
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

#ifndef _HEK_SYS_BOARD_H
#define _HEK_SYS_BOARD_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"

namespace hekateros
{
  class HekSysBoard
  {
  public:

    /*!
     * \brief Default constructor.
     */
    HekSysBoard()
    {
      m_i2c.fd    = -1;
      m_i2c.addr  = 0;
    }
    
    /*!
     * \brief Destructor.
     */
    virtual ~HekSysBoard()
    {
      close();
    }

    /*!
     * \brief Open all interfaces monitoring hardware.
     *
     * \param dev   \h_i2c device.
     *
     * \copydoc doc_return_std
     */
    virtual int open(const std::string& dev=HekI2CDevice);

    /*!
     * \brief Close all interfaces to monitoring hardware.
     *
     * \copydoc doc_return_std
     */
    virtual int close();

    /*!
     * \brief Scan and initialize hardware.
     *
     * \copydoc doc_return_std
     */
    virtual int scan();

    /*!
     * \brief Command to read firmware version.
     *
     * param [out] ver  Version number.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdReadFwVersion(int &ver);

    /*!
     * \brief Command to read limit bit state.
     *
     * \return Returns limit switches state as a bit packed byte.
     */
    virtual byte_t cmdReadLimits();

    /*!
     * \brief Command to read auxilliary bit state.
     *
     * Auxillary bits are usually End Effector GPIO, but allows room for
     * additional bits.
     *
     * \return Returns auxilliary switches state as a bit packed byte.
     */
    virtual byte_t cmdReadAux();

    /*!
     * \brief Command to read one I/O pin.
     *
     * \param id    Pin identifier.
     *
     * \return Return pin state 0 or 1.
     */
    virtual int cmdReadPin(int id);
    
    /*!
     * \brief Command to write value to an I/O pin.
     *
     * \param id    Pin identifier.
     * \param val   Pin state 0 or 1.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdWritePin(int id, int val);

    /*!
     * \brief Command to configure direction of an I/O pin.
     *
     * \param id    Pin identifier.
     * \param dir   Pin direction 'i' or 'o'.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdConfigPin(int id, char dir);

    /*!
     * \brief Command to set Alarm LED.
     *
     * \param val   LED 0==off or 1==on value.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetAlarmLED(int val);
  
    /*!
     * \brief Command to set Status LED.
     *
     * \param val   LED 0==off or 1==on value.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetStatusLED(int val);
  
    /*!
     * \brief Command to set monitoring state to halting.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetHaltingState();

  protected:
    i2c_struct  m_i2c;            ///< i2c bus

    /*!
     * \brief Read I/O expander byte.
     *
     * \param byCmd   I/O expander command.
     *
     * \return Byte value.
     */
    byte_t readIOExp(byte_t byCmd);

    /*!
     * \brief Read I/O expander port 0.
     *
     * All optical limit switches are tied to port 0.
     *
     * \return Port 0 byte value.
     */
    byte_t readIOExpPort0()
    {
      return readIOExp(HekIOExpCmdInput0);
    }

    /*!
     * \brief Read I/O expander port 1.
     *
     * All end effector i/o are tied to port 1.
     *
     * \return Port 1 byte value.
     */
    byte_t readIOExpPort1()
    {
      return readIOExp(HekIOExpCmdInput1);
    }

    /*!
     * \brief Write byte to I/O expander.
     *
     * \param byCmd   I/O expander command.
     * \param Byte value.
     *
     * \copydoc doc_return_std
     */
    int writeIOExp(byte_t byCmd, byte_t byVal);
  }; 

} // namespace hekateros


#endif // _HEK_SYS_BOARD_H
