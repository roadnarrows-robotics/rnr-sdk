////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeI2C.h
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps I2C class interface.
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

#ifndef _LAE_I2C_H
#define _LAE_I2C_H

#include <sys/types.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/i2c.h"
#include "rnr/log.h"

#include  "Laelaps/laelaps.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
#ifndef SWIG
namespace laelaps
{
#endif // SWIG

  //----------------------------------------------------------------------------
  // LaeI2C Class
  //----------------------------------------------------------------------------
  
  /*!
   * I2C class.
   */
  class LaeI2C
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeI2C();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeI2C();

    /*!
     * \brief Open \h_i2c bus device.
     *
     * \param strDevName    \h_i2c device name.
     *
     * \copydoc doc_return_std
     */
    virtual int open(const std::string &strDevName);

    /*!
     * \brief Close \h_i2c bus device.
     *
     * \copydoc doc_return_std
     */
    virtual int close();

    /*! 
     * \brief Read from a \h_i2c slave endpoint device.
     *
     * Reads data from a device at the given address on the bound \h_i2c bus.
     *
     * \param addr      Endpoint \h_i2c device's 7/10-bit address.
     * \param [out] buf Pointer to the buffer that will receive the data bytes.
     * \param len       Number of bytes to read.
     *
     * \return
     * On success, returns \h_ge 0 number of bytes read.\n
     * Else returns \h_lt 0 error code.
     */
    virtual int read(uint_t addr, byte_t buf[], size_t len);

    /*! 
     * \brief Write to an \h_i2c slave endpoint device.
     *
     * Writes data to a device at the given address on the bound \h_i2c bus.
     *
     * \param addr      Endpoint \h_i2c device's 7/10-bit address.
     * \param [in] buf  Pointer to the buffer that will be written.
     * \param len       Number of bytes to write.
     *
     * \return
     * On success, returns \h_ge 0 number of bytes written.\n
     * Else returns \h_lt 0 error code.
     */
    virtual int write(uint_t addr, const byte_t buf[], size_t len);

    /*! 
     * \brief Perform a write/read transfer to/from an \h_i2c slave endpoint
     * device.
     *
     * An \h_i2c tranfer sends/receives one or more messages before the STOP
     * is issued to terminate the operation. Each message does begin with a
     * START.
     *
     * \note Not all slave devices support this fast transfer protocol. If not,
     * then use the \ref write_read() instead.
     * 
     * \param addr        Endpoint \h_i2c device's 7/10-bit address.
     * \param [in] wbuf   Pointer to the buffer that contains the data 
     *                    to be written. 
     * \param wlen        Number of bytes to write.
     * \param [out] rbuf  Pointer to the buffer that will receive the data. 
     * \param rlen        Number of bytes to read.
     *
     * \copydoc doc_return_std
     */
    virtual int transfer(uint_t addr, const byte_t wbuf[], size_t wlen,
                                      byte_t rbuf[], size_t rlen);

    /*! 
     * \brief Perform a write/read transaction to/from an \h_i2c slave endpoint
     * device.
     *
     * Before each message a START is issued.
     * After each message, a STOP is issued.
     *
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
    virtual int write_read(uint_t addr, const byte_t wbuf[], size_t wlen,
                                        byte_t rbuf[], size_t rlen,
                                        long usec=0);

    /*!
     * \brief Get associated \h_i2c device name.
     *
     * \return Returns device name string. Empty if no device.
     */
    std::string getDevName()
    {
      return m_strDevName;
    }

    /*!
     * \brief Check if device is open.
     *
     * \return Returns true or false.
     */
    bool isOpen()
    {
      return m_i2c.fd >= 0? true: false;
    }

  protected:
    std::string     m_strDevName; ///< \h_i2c device name
    i2c_t           m_i2c;        ///< \h_i2c bus instance
    pthread_mutex_t m_mutex;      ///< mutual exclusion

    /*!
     * \brief Lock the \h_i2c bus.
     *
     * The lock()/unlock() primitives provide safe multi-threading access.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutex);
    }

  
    /*!
     * \brief Unlock the \h_i2c bus.
     *
     * The lock()/unlock() primitives provide safe multi-threading access.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutex);
    }

  };
  
#ifndef SWIG
} // namespace laelaps
#endif // SWIG


#endif // _LAE_I2C_H
