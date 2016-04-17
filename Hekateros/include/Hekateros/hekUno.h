////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekUno.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-11 10:45:27 -0600 (Thu, 11 Jul 2013) $
 * $Rev: 3111 $
 *
 * \brief \h_hek Arduino Uno compatible I/O board class interface.
 *
 * \author Robin Knight    (robin.knight@roadnarrows.com)
 * \author Daniel Packard  (daniel@roadnarrows.com)
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

#ifndef _HEK_UNO_H
#define _HEK_UNO_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"

namespace hekateros
{
  class HekUno
  {
  public:

    /*!
     * \brief Default constructor.
     */
    HekUno();
    
    /*!
     * \brief Destructor.
     */
    virtual ~HekUno();

    /*!
     * \brief Open all interfaces monitoring hardware.
     *
     * \param uHekHwVer \h_hek hardware version.  
     * \param dev       Arduino compatible device.
     * \param baud      Baud rate.
     *
     * \copydoc doc_return_std
     */
    virtual int open(uint_t             uHekHwVer,
                     const std::string& dev  = HekDevArduino,
                     int                baud = HekBaudRateArduino);

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
     * \brief Command to set monitoring state to halted.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetHaltedState();

    /*!
     * \brief Test serial interface command.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdTestInterface();

    /*!
     * \brief Null command.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdNull();

  protected:
    int         m_fd;         ///< open arduino file descriptor
    uint_t      m_uHekHwVer;  ///< hekateros hardware version
    int         m_nFwVer;     ///< arduino firmware version

    // test serial interface
    int         m_nFwOpState; ///< firmware operational state
    int         m_nFwSeqNum;  ///< expected test sequence number modulo 256 

    /*!
     * \brief Send command to Arduino.
     *
     * \param buf     Buffer of bytes.
     * \param nBytes  Number of bytes in buffer to write.
     * \param timeout Send timeout (usec).
     *
     * \copydoc doc_return_std
     */
    int sendCommand(char *buf, size_t nBytes, uint_t timeout=50000);

    /*!
     * \brief Receive response from Arduino.
     *
     * \param buf     Receiving character buffer.
     * \param count   Maximum number of bytes to read.
     * \param timeout Receive timeout (usec).
     *
     * \copydoc doc_return_std
     */
    int recvResponse(char buf[], size_t count, uint_t timeout=100000);

    /*!
     * \brief Unpack hex characters to byte.
     *
     * Format: hh
     *
     * \param buf     Buffer of hex characters
     *
     * \return Returns unpacked byte.
     */
    byte_t unpackHex(char buf[]);
  }; 

} // namespace hekateros


#endif // _HEK_UNO_H
