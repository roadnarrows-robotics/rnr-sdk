////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeWd.h
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps WatchDog software class interface.
 *
 * The class provides the interface between the library software and the
 * Arduino sub-processor.
 *
 * The Wd class is multi-threading safe.
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

#ifndef _LAE_WD_H
#define _LAE_WD_H

#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/Time.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"
#include  "Laelaps/laeTune.h"

#include  "Laelaps/laeI2C.h"
#include  "Laelaps/laeWatchDog.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{

  //----------------------------------------------------------------------------
  // LaeWd Class
  //----------------------------------------------------------------------------
  
  /*!
   * WatchDog sub-processor interface class.
   */
  class LaeWd
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param i2cbus  Bound open \h_i2c bus instance.
     * \param addr    Arduino I2C address.
     */
    LaeWd(LaeI2C &i2cBus, uint_t addr=LaeI2CAddrArduino);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeWd();

    /*!
     * \brief Synchronize watchdog state with subprocessor state.
     */
    virtual void sync();

    /*!
     * \brief Configure watchdog.
     *
     * Call after connection is opened.
     *
     * \param tunes   Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeTunes &tunes);
  
    /*!
     * \brief Reload with new configuration.
     *
     * \param tunes   Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int reload(const LaeTunes &tunes);
 
    /*!
     * \brief Execute cycle to pet/read/update Watchdog sub-processor.
     */
    virtual void exec();

    //..........................................................................
    // WatchDog Sub-Processor Member Functions
    //..........................................................................

    /*!
     * \brief Pet the watchdog command.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdPetTheDog();

    /*!
     * \brief Get the firmware version command.
     *
     * \param [out] uVerNum   Firmware version number.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdGetFwVersion(uint_t &uVerNum);

    /*!
     * \brief Set battery's state of charge state command.
     *
     * \param uBatterySoC   Battery state of charge as a percent [0-100].
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetBatterySoC(uint_t uBatterySoC);

    /*!
     * \brief Set (clear) alarms command.
     *
     * \param uAlarms   Bit-or'ed alarm state. 
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetAlarms(uint_t uAlarms);

    /*!
     * \brief Set the LED RGB color command.
     *
     * \param red     Red component [0-255].
     * \param green   Green component [0-255].
     * \param blue    Blue component [0-255].
     *
     * \copydoc doc_return_std
     */
    virtual int cmdSetRgbLed(uint_t red, uint_t green, uint_t blue);

    /*!
     * \brief Reset the LED RGB color to state defaults.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdResetRgbLed();

    /*!
     * \brief Configure a digital pin command.
     *
     * \param pin   Digital pin number.
     * \param dir   Pin direction. 0 == input, 1 == output.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdConfigDPin(uint_t pin, uint_t dir);

    /*!
     * \brief Read the value of a digital pin command.
     *
     * \param pin         Digital pin number.
     * \param [out] val   Digital pin low (0) or high (1) value.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdReadDPin(uint_t pin, uint_t &val);

    /*!
     * \brief Write a value to a digital pin command.
     *
     * \param pin     Digital pin number.
     * \param val     Digital pin low (0) or high (1) value.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdWriteDPin(uint_t pin, uint_t val);

    /*!
     * \brief Read the value of an analog pin command.
     *
     * \param pin         Analog pin number.
     * \param [out] val   Analog 10-bit value [0-1023].
     *
     * \copydoc doc_return_std
     */
    virtual int cmdReadAPin(uint_t pin, uint_t &val);

    /*!
     * \brief Write the value to an analog pin command.
     *
     * \param pin   Analog pin number.
     * \param val   Analog 10-bit value [0-1023].
     *
     * \copydoc doc_return_std
     */
    virtual int cmdWriteAPin(uint_t pin, uint_t val);

    /*!
     * \brief Enable/disable power in to motor controllers.
     *
     * \param bEnable   Disable (false) or enable (true).
     *
     * \copydoc doc_return_std
     */
    virtual int cmdEnableMotorCtlrs(bool bEnable);

    /*!
     * \brief Enable/disable regulated 5 volt auxilliary port power out.
     *
     * \param bEnable   Disable (false) or enable (true).
     *
     * \copydoc doc_return_std
     */
    virtual int cmdEnableAuxPort5V(bool bEnable);

    /*!
     * \brief Enable/disable battery auxilliary port power out.
     *
     * \param bEnable   Disable (false) or enable (true).
     *
     * \copydoc doc_return_std
     */
    virtual int cmdEnableAuxPortBatt(bool bEnable);

    /*!
     * \brief Read enable lines.
     *
     * \param [out] bMotorCtlrEn    Motor controllers enable.
     * \param [out] bAuxPort5vEn    Auxilliary port 5v out enable.
     * \param [out] bAuxPortBattEn  Auxilliary port battery out enable.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdReadEnables(bool &bMotorCtlrEn,
                               bool &bAuxPort5vEn,
                               bool &bAuxPortBattEn);

    /*!
     * \brief Read sensed voltages.
     *
     * \param [out] fJackV    Sensed charging jack input voltage.
     * \param [out] fBattV    Sensed battery output voltage.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdReadVoltages(double &fJackV, double &fBattV);

    /*!
     * \brief Test the firmware state command.
     *
     * \param [out] uSeqNum   This command's sequence number modulo 256.
     * \param [out] uOpState  Firmware's operational state.
     * \param [out] uAlarms   Firmware's view of alarms.
     * \param [out] uLedIndex Firmware's current LED pattern index.
     *
     * \copydoc doc_return_std
     */
    virtual int cmdTest(uint_t &uSeqNum,
                        uint_t &uOpState,
                        uint_t &uAlarms,
                        uint_t &uLedIndex);

    /*!
     * \brief Enable/disable power in to motor controllers.
     *
     * \param pArg      Recast pointer to this class instance.
     * \param bEnable   Disable (false) or enable (true).
     *
     * \copydoc doc_return_std
     */
    static int enableMotorCtlrs(void *pArg, bool bEnable);

  protected:
    // hardware
    LaeI2C   &m_i2cBus;       ///< bound \h_i2c bus instance
    uint_t    m_addrSubProc;  ///< \h_i2c sub-processor address
    uint_t    m_uFwVer;       ///< firmware version number

    // shadow values
    bool      m_bBatteryIsCharging; ///< battery is [not] charging
    double    m_fBatteryVoltage;    ///< sensed battery voltage
    double    m_fJackVoltage;       ///< sensed power supply jack voltage
    uint_t    m_uBatterySoC;        ///< battery state of charge
    uint_t    m_uAlarms;            ///< alarms
    bool      m_bMotorCtlrEn;       ///< motor controller enable
    bool      m_bAuxPortBattEn;     ///< battery auxilliary port enable
    bool      m_bAuxPort5vEn;       ///< 5 volt auxilliary port enable
    
    // uptimes
    rnr::Time m_timeMotorCtlrs;     ///< motor controller up time

    // mutual exclusion
    pthread_mutex_t m_mutex;  ///< mutex
  
    /*!
     * \brief Lock the shared resource.
     *
     * The lock()/unlock() primitives provide a safe threading.
     * registered vServo data.
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
     * \brief Determine watchdog alarm state.
     *
     * The current real-time database alarm state is used for the determination.
     *
     * \return Watchdog alarm state.
     */
    uint_t determineAlarms();
  };
  
} // namespace laelaps


#endif // _LAE_WD_H
