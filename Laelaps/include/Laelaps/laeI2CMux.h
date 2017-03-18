////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeI2CMux.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps PCA9548A I2C multiplexer switch interface.
 *
 * The multiplexer allows access to multiple \h_i2c devices including those
 * that have the same fixed address.
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

#ifndef _LAE_I2C_MUX_H
#define _LAE_I2C_MUX_H

#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeI2C.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
#ifndef SWIG
namespace laelaps
{
#endif // SWIG

  //----------------------------------------------------------------------------
  // Constants
  //----------------------------------------------------------------------------

  // 
  // Addresses
  //
  const i2c_addr_t LaeI2CMuxAddrMin  = 0x70; ///< \h_i2c minimum 7-bit address
  const i2c_addr_t LaeI2CMuxAddrMax  = 0x77; ///< \h_i2c maximum 7-bit address
  const i2c_addr_t LaeI2CMuxAddrDft  = 0x70; ///< \h_i2c default 7-bit address

  //
  // I2C slave channel numbers.
  //
  const int     LaeI2CMuxChanMin  = 0;     ///< min channel number
  const int     LaeI2CMuxChanMax  = 7;     ///< min channel number
  const int     LaeI2CMuxChanNone = 0x100; ///< no channel selected

  //
  // Control register.
  //
  const byte_t  LaeI2CMuxCtlRegMask = 0xff; /// control register mask

  //----------------------------------------------------------------------------
  // LaeI2CMux Class
  //----------------------------------------------------------------------------
  
  /*!
   * TI PCA9548A \h_i2c mulitplexer switch class.
   */
  class LaeI2CMux
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param i2cbus  Bound open \h_i2c bus instance.
     * \param addr    I2C Mux address.
     */
    LaeI2CMux(LaeI2C &i2cBus, uint_t addr=LaeI2CMuxAddrDft);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeI2CMux();

    /*! 
     * \brief Read from a multiplexed \h_i2c slave endpoint device.
     *
     * Reads data from a device at the given address on the bound \h_i2c bus.
     *
     * \param chan      Multiplexed channel number.
     * \param addr      Endpoint \h_i2c device's 7/10-bit address.
     * \param [out] buf Pointer to the buffer that will receive the data bytes.
     * \param len       Number of bytes to read.
     *
     * \return
     * On success, returns \h_ge 0 number of bytes read.\n
     * Else returns \h_lt 0 error code.
     */
    virtual int read(int chan, uint_t addr, byte_t buf[], size_t len);

    /*! 
     * \brief Write from an \h_i2c endpoint device.
     *
     * Write data to a device at the given address on the bound \h_i2c bus.
     *
     * \param chan      Multiplexed channel number.
     * \param addr      Endpoint \h_i2c device's 7/10-bit address.
     * \param [int] buf Pointer to the buffer that will be written.
     * \param len       Number of bytes to write.
     *
     * \return
     * On success, returns \h_ge 0 number of bytes read.\n
     * Else returns \h_lt 0 error code.
     */
    virtual int write(int chan, uint_t addr, byte_t buf[], size_t len);

    /*! 
     * \brief Perform a write/read transaction to/from an \h_i2c slave endpoint
     * device.
     *
     * \param chan        Multiplexed channel number.
     * \param addr        Endpoint \h_i2c device's 7/10-bit address.
     * \param [in] wbuf   Pointer to the buffer that contains the data 
     *                    to be written. 
     * \param wlen        Number of bytes to write.
     * \param [out] rbuf  Pointer to the buffer that will receive the data. 
     * \param rlen        Number of bytes to read.
     * \param usec        Delay between write and read operations (usecs).
     *
     * \copydoc doc_return_std
     */
    virtual int transact(int chan, uint_t addr,
                         const byte_t wbuf[], size_t wlen,
                         byte_t rbuf[], size_t rlen,
                         long usec=0);

    /*! 
     * \brief Read the current \h_i2c multiplexer enable state.
     *
     * \param [out] chanBits  Bit map of enable/disable channels.
     *
     * \copydoc doc_return_std
     */
    virtual int readChannelStates(byte_t &chanBits);

    /*!
     * \brief Reset \h_i2c devices on multiplexer bus.
     *
     * \copydoc doc_return_std
     */
    virtual int reset()
    {
      // TODO
      return -LAE_ECODE_INTERNAL;
    }

    /*!
     * \brief Check if a channel is enabled in the channel bit map.
     *
     * \param chan      Multiplexed channel number.
     * \param chanBits  Bit map of enable/disable channels.
     *
     * \return True or false.
     */
    bool isChanEnabled(int chan, byte_t chanBits)
    {
      return (LaeI2CMuxCtlRegMask & ((byte_t)(1 << chan) & chanBits)) != 0;
    }

    /*!
     * \brief Get associated \h_i2c device name.
     *
     * \return Returns device name string. Empty if no device.
     */
    std::string getDevName()
    {
      return m_i2cBus.getDevName();
    }

    /*!
     * \brief Check if bound I2C device is open.
     *
     * \return Returns true or false.
     */
    bool isOpen()
    {
      return m_i2cBus.isOpen();
    }

  protected:
    LaeI2C     &m_i2cBus;     ///< boulnd \h_i2c bus instance
    uint_t      m_addrMux;    ///< \h_i2c multiplexer address
    int         m_chan;       ///< current active channel

    // mutual exclusion
    pthread_mutex_t m_mutex;          ///< mutex
  
    /*!
     * \brief Lock the shared resource.
     *
     * The lock()/unlock() primitives provide for safe multi-threading.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutex);
    }
    
    /*!
     * \brief Unlock the shared resource.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutex);
    }

    /*!
     * \brief Set channel.
     *
     * This function does not call the lock/unlock mutex.
     *
     * \param chan      Multiplexed channel number.
     *
     * \copydoc doc_return_std
     */
    int setChannel(int chan);

  }; // class LaeI2CMux
  
#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_I2C_MUX_H
