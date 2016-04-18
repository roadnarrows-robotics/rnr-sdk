////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeDesc.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-15 15:44:49 -0700 (Mon, 15 Feb 2016) $
 * $Rev: 4320 $
 *
 * \brief Laelaps robotic base mobile platform description class interface.
 *
 * The description does not include any payload descriptions.
 * Any applicable tuning parameters override the description.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows
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

#ifndef _LAE_DESC_H
#define _LAE_DESC_H

#include <stdio.h>

#include <string>
#include <map>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeMotor.h"


namespace laelaps
{
  /*!
   * \brief Supported joint/wheel types. Not really used for now.
   */
  enum LaeJointType
  {
    LaeJointTypeUnknown     = 0,  ///< unknown/undefined joint type
    LaeJointTypeFixed,            ///< fixed joint
    LaeJointTypeRevolute,         ///< limited rotation
    LaeJointTypeContinuous,       ///< continuous rotation
    LaeJointTypeRevMimic,         ///< mimic rotation (e.g. fingers)

    LaeJointTypeNumOf       = 4   ///< number of supported joint types
  };

  /*!
   * \brief Joint/wheel location detection types.
   *
   * A joint may have more than one type.
   */
  enum LaeEncType
  {
    LaeEncTypeUnknown     = 0x00,   ///< unknown/undefined encoder type
    LaeEncTypeNone        = 0x01,   ///< no encoder
    LaeEncTypePhys        = 0x02,   ///< physical torque with relative encoder
    LaeEncTypeElec        = 0x04,   ///< limit switch with relative encoder
    LaeEncTypeElecTDC     = 0x08,   ///< top dead center switch with rel. enc.
    LaeEncTypeAbs         = 0x10,   ///< absolute encoder
    LaeEncTypeQuadrature  = 0x20,   ///< quadrature encoder

    LaeEncTypeNumOf       = 6       ///< number of supported location types
  };


  // ---------------------------------------------------------------------------
  // Class LaeDescBase
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic base platform description.
   *
   * A base includes the body, suspension, power supply, and fixed interfaces.
   */
  class LaeDescBase
  {
  public:
    std::string       m_strKey;         ///< base name
    Dim               m_dimRobot;       ///< robot full dimensions(m)
    Dim               m_dimBody;        ///< body dimensions(m)
    double            m_fWheelbase;     ///< wheelbase(m)
    double            m_fWheeltrack;    ///< wheeltrack(m)
    int               m_nNumMotorCtlrs; ///< number of motor controllers
    int               m_nNumMotors;     ///< number of motors

    /*!
     * \brief Default constructor.
     */
    LaeDescBase();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeDescBase();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    LaeDescBase operator=(const LaeDescBase &rhs);

    /*!
     * \brief Clear description.
     */
    void clear();

    /*!
     * \brief Calculate robot dimensions.
     *
     * \param fTireRadius   Tire radius (meters).
     * \param fTireWidth    Tire width (meters).
     */
    void calcDimensions(double fTireRadius, double fTireWidth);

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    virtual void print(int indent = 0);

  }; // class LaeDescBase


  // ---------------------------------------------------------------------------
  // Class LaeDescPowertrain
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic powertrain description.
   *
   * A powertrain includes the motor and drivetrain. The drivetrain is the
   * group of components that deliver power to the driving wheel(s) including
   * encoders, gears, axles, wheels, and tires.
   */
  class LaeDescPowertrain
  {
  public:
    std::string   m_strKey;           ///< powertrain key
    int           m_nMotorId;         ///< unique robot motor id
    int           m_nMotorCtlrId;     ///< unique motor controller id
    int           m_nMotorIndex;      ///< motor controller unique motor index
    LaeJointType  m_eJointType;       ///< powertrain joint type
    LaeEncType    m_eEncType;         ///< encoder type
    double        m_fGearRatio;       ///< motor gear ratio
    int           m_nDir;             ///< normalize cw/ccw direction.

    /*!
     * \brief Default constructor.
     */
    LaeDescPowertrain();

    /*!
     * \brief Initialization constructor.
     */
    LaeDescPowertrain(const std::string &strKey);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeDescPowertrain();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    LaeDescPowertrain operator=(const LaeDescPowertrain &rhs);

    /*!
     * \brief Clear description.
     */
    void clear();

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    virtual void print(int indent = 0);

  }; // class LaeDescPowertrain


  // ---------------------------------------------------------------------------
  // Class LaeDescBattery
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Battery description.
   */
  class LaeDescBattery
  {
  public:
    std::string   m_strKey;         ///< battery key
    std::string   m_strType;        ///< battery type
    std::string   m_strChemistry;   ///< battery chemistry
    int           m_nNumCells;      ///< number of cells
    double        m_fCapacity;      ///< battery capacity(Ah)
    double        m_fMaxV;          ///< battery maximum high charge (V)
    double        m_fMinV;          ///< battery minimum low at cutoff (V)
    double        m_fNomV;          ///< battery nominal voltage (V)

    /*!
     * \brief Default constructor.
     */
    LaeDescBattery();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeDescBattery();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    LaeDescBattery operator=(const LaeDescBattery &rhs);

    /*!
     * \brief Clear description.
     */
    void clear();

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    virtual void print(int indent = 0);

  }; // class LaeDescBattery


  // ---------------------------------------------------------------------------
  // Class LaeDescRangeSensor
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Range sensor description.
   */
  class LaeDescRangeSensor
  {
  public:
    std::string   m_strKey;     ///< range sensor key
    int           m_nChan;      ///< channel number
    double        m_fDir;       ///< sensor direction (radians)
    double        m_fDeadzone;  ///< deadzone (meters)
    std::string   m_strDesc;    ///< short description string

    /*!
     * \brief Default constructor.
     */
    LaeDescRangeSensor();

    /*!
     * \brief Initialization constructor.
     */
    LaeDescRangeSensor(const std::string &strKey);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeDescRangeSensor();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    LaeDescRangeSensor operator=(const LaeDescRangeSensor &rhs);

    /*!
     * \brief Clear description.
     */
    void clear();

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    virtual void print(int indent = 0);

  protected:

    /*!
     * \brief Make sensor description string from description.
     */
    virtual void makeDesc();

  }; // class LaeDescRangeSensor


  // ---------------------------------------------------------------------------
  // Class LaeDescImu
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic built-in IMU description.
   */
  class LaeDescImu
  {
  public:
    std::string       m_strKey;     ///< base key
    std::string       m_strHw;      ///< hardware 
    std::string       m_strFw;      ///< firmware

    /*!
     * \brief Default constructor.
     */
    LaeDescImu();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeDescImu();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    LaeDescImu operator=(const LaeDescImu &rhs);

    /*!
     * \brief Clear description.
     */
    void clear();

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    virtual void print(int indent = 0);

  }; // class LaeDescImu


  // ---------------------------------------------------------------------------
  // Class LaeDesc
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps robotic mobile platform full description class.
   *
   * The description is determined from the XML etc configuration file. From
   * those description components, the compiled description are assigned.
   */
  class LaeDesc
  {
  public: 
    /*! Map of powertrain descriptions type. */
    typedef std::map<std::string, LaeDescPowertrain*> MapDescPowertrain;

    /*! Map of range sensor descriptions type. */
    typedef std::map<std::string, LaeDescRangeSensor*> MapDescRangeSensor;

    static const char* const  KeyRobotBase;       ///< robot base key
    static const char* const  KeyBattery;         ///< internal battery key
    static const char* const  KeyMotorCtlr[];     ///< motor controller keys
    static const char* const  KeyPowertrain[];    ///< powertrain keys
    static const char* const  KeyImu;             ///< built-in IMU keys
    static const char* const  KeyRangeSensor[];   ///< full opt range sensr keys
    static const char* const  KeyRangeSensorStd[]; ///< std opt range sensr keys
    static const char* const  KeyFCam;            ///< front camera keys

    /*!
     * \brief Create pretty motor controller identifier string.
     *
     * \param nCtlrId Unique motor controller id.
     *
     * \return String
     */
    static std::string prettyMotorCtlrName(int nCtlrId);

    /*!
     * \brief Create pretty motor controller identifier string.
     *
     * \param nCtlrId Unique motor controller id.
     * \param addr    Motor controller packet address.
     *
     * \return String
     */
    static std::string prettyMotorCtlrName(int nCtlrId, byte_t addr);

    /*!
     * \brief Create pretty motor/powertrain identifier string.
     *
     * \param nMotorId  Unique motor id.
     *
     * \return String
     */
    static std::string prettyMotorName(int nMotorId);

    /*!
     * \brief Create pretty motor/powertrain identifier string.
     *
     * \param nCtlrId   Unique motor controller id.
     * \param addr      Motor controller packet address.
     * \param nMotorId  Unique motor id.
     *
     * \return String
     */
    static std::string prettyMotorName(int nCtlrId, byte_t addr, int nMotorId);


    // RoadNarrows product identifiers
    bool          m_bIsDescribed;   ///< \h_laelaps is [not] fully described
    int           m_eProdId;        ///< base product id
    std::string   m_strProdFamily;  ///< product family
    std::string   m_strProdModel;   ///< product model
    std::string   m_strProdName;    ///< product name
    std::string   m_strProdBrief;   ///< product brief
    std::string   m_strProdHwVer;   ///< product hardware version string
    uint_t        m_uProdHwVer;     ///< product hardware version number

    // RoadNarrows product descriptions
    LaeDescBase        *m_pDescBase;          ///< base description
    LaeDescBattery     *m_pDescBattery;       ///< internal battery description
    MapDescPowertrain   m_mapDescPowertrain;  ///< powertrain descriptions
    MapDescRangeSensor  m_mapDescRangeSensor; ///< range sensor descriptions
    LaeDescImu         *m_pDescImu;           ///< built-in imu description

    /*!
     * \breif Default constructor.
     */
    LaeDesc();
  
    /*!
     * \breif Destructor.
     */
    virtual ~LaeDesc();
  
    /*!
     * \brief Mark \h_laelaps hardware as fully described.
     *
     * The calling application context determines this state.
     *
     * \copydoc doc_return_std
     */
    int markAsDescribed();

    /*!
     * \brief Clear description to the "unitialized" values.
     */
    void clear();

    /*!
     * \brief Test if required base description is adequately described.
     *
     * \return Returns true or false.
     */
    bool isDescribed() const
    {
      return m_bIsDescribed;
    }

    /*!
     * \brief Get this base description's base product id.
     *
     * \return Returns product id. See \ref LaeProdId.
     */
    int getProdId() const
    {
      return m_eProdId;
    }
  
    /*!
     * \brief Get this base description's name.
     *
     * \return Returns string.
     */
    std::string getProdName() const
    {
      return m_strProdName;
    }
  
    /*!
     * \brief Get this base description's brief.
     *
     * \return Returns string.
     */
    std::string getProdBrief() const
    {
      return m_strProdBrief;
    }
  
    /*!
     * \brief Get this robot's hardware version string.
     *
     * \return Returns string.
     */
    std::string getProdHwVerString() const
    {
      return m_strProdHwVer;
    }
  
    /*!
     * \brief Get this robot's packed hardware version number.
     *
     * \return Number.
     */
    uint_t getProdHwVer() const
    {
      return m_uProdHwVer;
    }
  
    /*!
     * \brief Get the \h_laelaps product name string given the product id.
     *
     * \param eProdId Supported product id. See \ref LaeProdId.
     *
     * \return Returns product name string. An unidentified product id
     * returns "".
     */
    static const char *getProdName(int eProdId);
  
    /*!
     * \brief Get the \h_laelaps product one-line brief description string given
     * the product id.
     *
     * \param eProdId Supported product id. See \ref LaeProdId.
     *
     * \return Returns product description string. An unidentified product id
     * returns "".
     */
    static const char *getProdBrief(int eProdId);

    /*!
     * \brief Print out description to stdout.
     *
     * \param indent  Left indentation.
     */ 
    void print(int indent = 0);

  protected:
    /*!
     * \brief Set version number from parsed version string.
     */
    void setVersion();
  };

} // namespace laelaps

#endif // _LAE_DESC_H
