////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeGpio.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps Odroid General Purpose I/O class interfaces.
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

#ifndef _LAE_GPIO_H
#define _LAE_GPIO_H

#include <sys/types.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/gpio.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeSysDev.h"

namespace laelaps
{
  // ---------------------------------------------------------------------------
  // LaeGpio Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps Odroid GPIO base class.
   *
   * \note The configuration of the GPIO is assumed done elsewhere
   * (e.g. at system initialization time).
   */
  class LaeGpio
  {
  public:
    /*!
     * \brief GPIO tri-state value.
     */
    enum TriState
    {
      UNKNOWN = -1,     ///< unknown state value
      LOW     =  0,     ///< low value
      HIGH    =  1      ///< high value
    };

    /*!
     * \brief GPIO direction.
     */
    enum Direction
    {
      INPUT   = GPIO_DIR_IN,  ///< input direction
      OUTPUT  = GPIO_DIR_OUT  ///< output direction
    };

    /*!
     * \brief Default intialization constructor.
     *
     * No file I/O operations are perform.
     *
     * \param strTag  Tag identifier associated with GPIO pin.
     * \param gpio    Exported GPIO number.
     * \param dir     GPIO direction.
     */
    LaeGpio(const std::string        &strTag,
            const int                 gpio,
            const LaeGpio::Direction  dir);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeGpio()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Write value to gpio.
     *
     * \param value   LOW/HIGH tri-state value.
     *
     * \copydoc doc_return_std
     */
    virtual int writeValue(const LaeGpio::TriState value)
    {
      return writeValue((const int)value);
    }

    /*!
     * \brief Write value to gpio.
     *
     * \param value   LOW/HIGH integer value.
     *
     * \copydoc doc_return_std
     */
    virtual int writeValue(const int value);

    /*!
     * \brief Read current value of gpio.
     *
     * \param [out] value   LOW/HIGH integer value.
     *
     * \copydoc doc_return_std
     */
    virtual int readValue(int &value);

    /*!
     * \brief Get the current shadowed gpio value.
     *
     * \return  Returns LaeGpio::TriState value as an integer.
     */
    int hasValue() const
    {
      return m_gpioVal;
    }

    /*!
     * \brief Is the exported GPIO number configured to match this ojbect?
     *
     * Ass Laelaps evolves, the test can distinguish between different platform
     * versions. For example, Laelaps 2.0 does not have a gpio pin dedicated
     * to enabling power to the motor controllers, while subsequent version do.
     *
     * \return Returns true or false.
     */
    bool isConfigured() const
    {
      return m_gpioCfg;
    }

  protected:
    std::string         m_gpioTag;    ///< identifying tag 
    int                 m_gpioNum;    ///< exported GPIO number
    LaeGpio::Direction  m_gpioDir;    ///< GPIO direction
    int                 m_gpioVal;    ///< shadowed value
    bool                m_gpioCfg;    ///< GPIO is [not] configured (correctly)

    /*!
     * \brief Check if the exported GPIO exists in /sys/class and that it has
     * been configured to match this objects configuration.
     *
     * \return Returns true or false.
     */
    bool checkConfig();
  };


  // ---------------------------------------------------------------------------
  // LaeMotorCtlrEnable Class
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Laelaps motor controller power enable class.
   */
  class LaeMotorCtlrEnable : public LaeGpio
  {
  public:
    static const int TPowerUp = 500000;   ///< power-up time (usec)

    /*!
     * \brief Default constructor.
     */
    LaeMotorCtlrEnable() :
      LaeGpio("MotorCtlrEn", LaeGpioMotorCtlrEn, LaeGpio::OUTPUT)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeMotorCtlrEnable()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Enable power to motor controllers.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool enable();

    /*!
     * \brief Disable power to motor controllers.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool disable();

    /*!
     * \brief Test if power to motor controllers is enabled.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool isEnabled();

  protected:
  };


  // ---------------------------------------------------------------------------
  // LaeWatchDogReset Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps watchdog sub-processor reset class.
   */
  class LaeWatchDogReset : public LaeGpio
  {
  public:
    static const int TTrans   =  10000;   ///< signal transition time (usec)
    static const int TReboot  = 500000;   ///< reboot time (usec)

    /*!
     * \brief Default constructor.
     */
    LaeWatchDogReset() :
      LaeGpio("WatchDogReset", LaeGpioWdReset, LaeGpio::OUTPUT)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeWatchDogReset()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Reset the watchdog sub-processor.
     *
     * The reset is caused by a high to low edge trigger.
     */
    void reset();

  protected:
  };


  // ---------------------------------------------------------------------------
  // LaeI2CMuxReset Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps I2C multiplexer reset class.
   */

  // controlled by ThreadRange 
  class LaeI2CMuxReset : public LaeGpio
  {
  public:
    static const int TTrans   =  10000;   ///< signal transition time (usec)
    static const int TReboot  =  10000;   ///< reboot time (usec)

    /*!
     * \brief Default constructor.
     */
    LaeI2CMuxReset() :
      LaeGpio("I2CMuxReset", LaeGpioI2CMuxReset, LaeGpio::OUTPUT)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeI2CMuxReset()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Reset the I2C mulitplex chip.
     *
     * The reset is caused by a high to low edge trigger.
     */
    void reset();

  protected:
  };


  // ---------------------------------------------------------------------------
  // LaeAuxBattOutEnable Class
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Laelaps top deck auxilliary battery power out enable class.
   */
  class LaeAuxBattOutEnable : public LaeGpio
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeAuxBattOutEnable() :
      LaeGpio("AuxBattEn", LaeGpioAuxBattEn, LaeGpio::OUTPUT)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeAuxBattOutEnable()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Enable battery power to top deck.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool enable();

    /*!
     * \brief Disable battery power to top deck.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool disable();

    /*!
     * \brief Test if battery power to top deck is enabled.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool isEnabled();

  protected:
  };


  // ---------------------------------------------------------------------------
  // LaeAux5VOutEnable Class
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Laelaps top deck auxilliary regulated 5V power out enable class.
   */
  class LaeAux5VOutEnable : public LaeGpio
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeAux5VOutEnable() :
      LaeGpio("Aux5VEn", LaeGpioAux5VEn, LaeGpio::OUTPUT)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeAux5VOutEnable()
    {
    }

    /*!
     * \brief Synchronized this with GPIO hardware state.
     */
    virtual void sync();

    /*!
     * \brief Enable regulated 5V power to top deck.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool enable();

    /*!
     * \brief Disable regulated 5V power to top deck.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool disable();

    /*!
     * \brief Test if regulated 5V power to top deck is enabled.
     *
     * \return Returns true if power is enabled, false otherwise.
     */
    bool isEnabled();

  protected:
  };

} // namespace laelaps


#endif // _LAE_GPIO_H
