////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaServo.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Archetype Servo Abstract Base Class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#ifndef _DYNA_SERVO_H
#define _DYNA_SERVO_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaOlio.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaPidSpeed.h"


// ---------------------------------------------------------------------------
// Dynamixel Servo Abstract Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief Dynamixel Servo Abstract Base Class
 *
 * The DynaServo abstract class provides the archetype functions to bootstrap
 * into a specific DynaServo class object.
 */
class DynaServo
{
public:
  /*!
   * \brief Default initialization constructor.
   *
   * \param comm      Dynamixel bus communication instance.
   * \param nServoId  Servo Id.
   * \param uModelNum Servo model number.
   * \param uFwVer    Servo firmware version.
   */
  DynaServo(DynaComm &comm, int nServoId, uint_t uModelNum, uint_t uFwVer) :
              m_comm(comm)
  {
    Init(nServoId, uModelNum, uFwVer);
  }

  /*!
   * \brief Destructor.
   */
  virtual ~DynaServo();

  /*!
   * \brief Archetype constructor to create a new Dynamixel servo instance.
   *
   * The specific DynaServo object created depends on both the servo's model
   * number and firmware version stored in the servo's EEPROM.
   *
   * \param comm        Dynamixel bus communication instance.
   * \param nServoId    Servo id.
   *
   * \return On succes, returns pointer to the allocated DynaServo derived
   * object.\n
   * On failure, NULL is returned.
   */
  static DynaServo *New(DynaComm &comm, int nServoId);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // General Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  /*!
   * \brief Get servo model number.
   *
   * \return Model number.
   */
  virtual uint_t GetModelNumber() const
  {
    return m_cfg.m_uModelNum;
  }

  /*!
   * \brief Get servo firmware version.
   *
   * \return Model number.
   */
  virtual uint_t GetFirmwareVersion() const
  {
    return m_cfg.m_uFwVer;
  }

  /*!
   * \brief Get servo model name string.
   *
   * \return Model name.
   */
  virtual const char *GetModelName() const
  {
    return m_spec.m_sModelName;
  }

  /*!
   * \brief Get servo id.
   *
   * \return Servo id.
   */
  virtual uint_t GetServoId() const
  {
    return m_nServoId;
  }

  /*!
   * \brief Get the servo operational mode.
   *
   * \return Servo mode.
   */
  virtual uint_t GetServoMode() const
  {
    return m_cfg.m_uServoMode;
  }

  /*!
   * \brief Test if servo has 360\h_deg positioning information.
   *
   * \return Returns true or false.
   */
  virtual uint_t Has360PosInfo() const
  {
    return m_spec.m_bHas360Pos;
  }

  /*!
   * \brief Get servo specification.
   *
   * \return Reference to shadowed specification data.
   */
  virtual const DynaServoSpec_T &GetSpecification() const
  {
    return m_spec;
  }

  /*!
   * \brief Get servo configuration.
   *
   * \return Reference to shadowed configuration data.
   */
  virtual const DynaServoCfg_T &GetConfiguration() const
  {
    return m_cfg;
  }

  /*!
   * \brief Get servo state.
   *
   * \return Reference to shadowed state data.
   */
  virtual const DynaServoState_T &GetState() const
  {
    return m_state;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Linking Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Test if this servo is a master.
   *
   * A master is either the master servo in a linked master-slave configuration,
   * or an unlinked servo (a master unto thyself).
   *
   * \return Returns true if a master, false otherwise.
   */
  bool IsMaster() const
  {
    return ((m_link.m_uLinkType == DYNA_LINK_NONE) || 
            (m_link.m_uLinkType == DYNA_LINK_MASTER))? true: false;
  }

  /*!
   * \brief Test if this servo is a linked master.
   *
   * A linked master is the master servo in a linked master-slave configuration.
   *
   * \return Returns true if a linked master, false otherwise.
   */
  bool IsLinkedMaster() const
  {
    return (m_link.m_uLinkType == DYNA_LINK_MASTER)? true: false;
  }

  /*!
   * \brief Test if this servo is unlinked.
   *
   * \return Returns true if unlinked, false otherwise.
   */
  bool IsUnlinked() const
  {
    return (m_link.m_uLinkType == DYNA_LINK_NONE)? true: false;
  }

  /*!
   * \brief Get linked information.
   *
   * \return Reference to linked data.
   */
  virtual const DynaServoLink_T GetLinkInfo() const
  {
    DynaServoLink_T link;

    link.m_uLinkType    = m_link.m_uLinkType;
    link.m_nServoIdMate = m_link.m_pServoMate != NULL?
                              m_link.m_pServoMate->GetServoId(): DYNA_ID_NONE;
    link.m_bRotReversed = m_link.m_bRotReversed;

    return link;
  }

  /*!
   * \brief Link this servo to another.
   *
   * \param uLinkType     This servo's linked type.
   *                      See \ref dyna_servo_link_type.
   * \param pServoMate    This servo's linked mate.
   * \param bRotReversed  The linked servos do [not] rotate in opposite
   *                      directions.
   */
  virtual void Link(uint_t uLinkType, DynaServo *pServoMate, bool bRotReversed)
  {
    m_link.m_uLinkType    = uLinkType;
    m_link.m_pServoMate   = pServoMate;
    m_link.m_bRotReversed = bRotReversed;
  }

  /*!
   * \brief Unlink this servo.
   */
  virtual void Unlink()
  {
    m_link.m_uLinkType    = DYNA_LINK_NONE;
    m_link.m_pServoMate   = NULL;
    m_link.m_bRotReversed = false;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Virtual Odometer Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Get the current virtual odometer value.
   *
   * \return Odometer value.
   */
  int GetOdometer()
  {
    return m_state.m_od.m_nOdometer;
  }

  /*!
   * \brief Test if virtual odometer mapping is enabled.
   *
   * \return Returns true or false.
   */
  int IsOdometerEnabled()
  {
    return m_state.m_od.m_bOdEnabled;
  }

  /*!
   * \brief Get the virtual odometer zero point.
   *
   * \return Servo zero point.
   */
  int GetOdometerZeroPt()
  {
    return m_state.m_od.m_nEncZeroPt;
  }

  /*!
   * \brief Test if the virtual odometer is reversed.
   *
   * \return Returns true if reversed, false otherwise.
   */
  bool IsOdometerReversed()
  {
    return m_state.m_od.m_nOdDir == -1? true: false;
  }

  /*!
   * \brief Calculate the odometer value at the minimum (zero) encoder value.
   *
   * \return Odometer value.
   */
  int CalcOdometerAtEncMin()
  {
    return -m_state.m_od.m_nEncZeroPt * m_state.m_od.m_nOdDir;
  }

  /*!
   * \brief Calculate the odometer value at the maximum encoder value.
   *
   * \return Odometer value.
   */
  int CalcOdometerAtEncMax()
  {
    return (m_spec.m_uRawPosMax - m_state.m_od.m_nEncZeroPt) *
              m_state.m_od.m_nOdDir;
  }

  /*!
   * \brief Convert virtual odometer units to servo encoder units.
   *
   * \param nOdPos  Odometer position.
   *
   * \return Encoder ticks.
   */
  int OdometerToEncoder(int nOdPos)
  {
    return imod(m_state.m_od.m_nEncZeroPt + nOdPos * m_state.m_od.m_nOdDir,
                m_spec.m_uRawPosModulo);
  }

  /*!
   * \brief Calculate serve direction to goal odometer position.
   *
   * \param nOdGoalPos  Odometer goal position.
   *
   * \return 1 or -1
   */
  int CalcSpeedDir(int nOdGoalPos)
  {
    if( GetServoMode() == DYNA_MODE_CONTINUOUS )
    {
      return nOdGoalPos >= m_state.m_od.m_nOdometer?
                             m_state.m_od.m_nOdDir:
                            -m_state.m_od.m_nOdDir;
    }
    else
    {
      return 1;
    }
  }

  /*!
   * \brief Reset the servo's virtual odometer.
   *
   * The odometer mapping is enabled.
   *
   * \param nEncZeroPt  Zero point position in servo raw encoder position units.
   * \param bIsReverse  Do [not] reverse rotation sense.
   *
   * \return Returns the new, odometer position [-OdModulo+1, OdModulo-1] in
   * virtual raw position units.
   */
  virtual int   ResetOdometer(int nEncZeroPt, bool bIsReverse);

  /*!
   * \brief Update the odometer from the current servo position and rotation
   * direction.
   *
   * \param nEncCurPos  Servo encoder current position in raw ticks.
   *
   * \return Returns the new, odometer position [-OdModulo+1, OdModulo-1] in
   * virtual raw position units.
   */
  virtual int   UpdateOdometer(int nEncCurPos);

  /*!
   * \brief Disable odometer mapping.
   *
   * The odometer will always equal the encoder value.
   */
  virtual void  DisableOdometer();


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // State Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  /*!
   * \brief Get the current servo alarms.
   *
   * \return Alarm bits.
   */
  virtual uint_t GetAlarms() const
  {
    return m_state.m_uAlarms;
  }

  /*!
   * \brief Get the current servo position.
   *
   * \return Current position.
   */
  virtual uint_t GetCurPos() const
  {
    return m_state.m_uCurPos;
  }

  /*!
   * \brief Get the current servo speed.
   *
   * \return Current speed.
   */
  virtual int GetCurSpeed() const
  {
    return m_state.m_nCurSpeed;
  }

  /*!
   * \brief Get the goal servo speed.
   *
   * \return Goal speed.
   */
  virtual int GetGoalSpeed() const
  {
    return m_state.m_nGoalSpeed;
  }

  /*!
   * \brief Get the current servo load.
   *
   * \return Current speed.
   */
  virtual int GetCurLoad() const
  {
    return m_state.m_nCurLoad;
  }

  /*!
   * \brief Get the current temperature.
   *
   * \return Current temperature in raw units.
   */
  virtual uint_t GetCurTemp() const
  {
    return m_state.m_uCurTemp;
  }

  /*!
   * \brief Convert raw temperature coding to degrees Celsius.
   *
   * \param uTemp   Raw temperature value.
   *
   * \return Celsius.
   */
  virtual float CvtRawTempToC(uint uTemp) = 0;

  /*!
   * \brief Get the current voltage.
   *
   * \return Current voltage in raw units.
   */
  virtual uint_t GetCurVolt() const
  {
    return m_state.m_uCurVolt;
  }

  /*!
   * \brief Convert raw volts coding to volts.
   *
   * \param uTemp   Raw volts value.
   *
   * \return Volatage.
   */
  virtual float CvtRawVoltToVolts(uint uVolts) = 0;

  /*!
   * \brief Set soft torque thresholds.
   *
   * The thresholds are used as a hystersis to software control servo torque.
   *
   * \param uOverTorqueTh   High threshold where the servo is set in an over
   *                        torque condition.
   * \param uClearTorqueTh  Low threshold where the servo is cleared of an over
   *                        torque condition.
   */
  virtual void SetSoftTorqueThresholds(uint_t uOverTorqueTh,
                                       uint_t uClearTorqueTh)
  {
    m_state.m_uOverTorqueTh   = uOverTorqueTh;
    m_state.m_uClearTorqueTh  = uClearTorqueTh;
    m_state.m_bOverTorqueCond = false;
  }

  /*!
   * \brief Get soft torque thresholds.
   *
   * The thresholds are used as a hystersis to software control servo torque.
   *
   * \param [out] uOverTorqueTh   High threshold where the servo is set in an
   *                              over torque condition.
   * \param [out] uClearTorqueTh  Low threshold where the servo is cleared of
   *                              an over torque condition.
   */
  virtual void GetSoftTorqueThresholds(uint_t &uOverTorqueTh,
                                       uint_t &uClearTorqueTh)
  {
   uOverTorqueTh  = m_state.m_uOverTorqueTh;
   uClearTorqueTh = m_state.m_uClearTorqueTh;
  }

  /*!
   * \brief Set or clear servo in soft over torque condition.
   *
   * \param bNewCond    Servo [not] in over torque condition.
   */
  virtual void SetSoftTorqueOverCond(bool bNewCond)
  {
    m_state.m_bOverTorqueCond = bNewCond;
  }

  /*!
   * \brief Test if servo is in a soft over torque condition.
   *
   * \return Returns current in over torque condition.
   */
  virtual bool HasSoftTorqueOverCond()
  {
    return m_state.m_bOverTorqueCond;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Proxy Agent Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Register servo proxy agent.
   *
   * \param pAgent    Pointer to agent calls.
   * \param pAgentArg Agent supplied Argument passed back to calls.
   */
  virtual void RegisterAgent(DynaAgent_T *pAgent,
                             void        *pAgentArg)
  {
    m_pAgent    = pAgent;
    m_pAgentArg = pAgentArg;
  }

  /*!
   * \brief Unregister servo proxy agent.
   */
  virtual void UnregisterAgent()
  {
    m_pAgent    = NULL;
    m_pAgentArg = NULL;
  }

  /*!
   * \brief Tests if servo has a registered agent.
   * \return Returns true or false.
   */
  virtual bool HasAgent()
  {
    return m_pAgent != NULL? true: false;
  }

  virtual int AgentWriteGoalPos(int nGoalPos) = 0;
  virtual int AgentWriteGoalSpeed(int nGoalSpeed) = 0;
  virtual int AgentWriteGoalSpeedPos(int nGoalSpeed, int nGoalPos) = 0;
  
  
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Abstract Servo Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  virtual int MoveTo(int nGoalPos) = 0;
  virtual int MoveAtSpeedTo(int nGoalSpeed, int nGoalPos) = 0;
  virtual int MoveAtSpeed(int nGoalSpeed) = 0;

  virtual int EStop() = 0;
  virtual int Stop() = 0;
  virtual int Freeze() = 0;
  virtual int Release() = 0;


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Read the servo model number from the servo's EEPROM.
   * 
   * \par Control Table:
   * \ref dyna_servo_memmap_model_num
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   * \param [out] pModelNum Pointer to read servo model number.
   *
   * \copydoc doc_return_std
   */
  static int ReadModelNumber(DynaComm &comm, int nServoId, uint_t *pModelNum);

  /*!
   * \brief Read the servo's firmware version from the servo's EEPROM.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_fwver
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   * \param [out] pFwVer    Pointer to read servo firmware version.
   *
   * \copydoc doc_return_std
   */
  static int ReadFirmwareVersion(DynaComm &comm, int nServoId, uint_t *pFwVer);

  /*!
   * \brief Read the servo's id from the servo's EEPROM.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_id
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   * \param [out] pServoId  Pointer to read servo servo id.
   *
   * \copydoc doc_return_std
   */
  static int ReadServoId(DynaComm &comm, int nServoId, int *pServoId);

  /*!
   * \brief Write the new servo id to the servo's EEPROM.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_id
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   * \param nNewServoId     New servo id.
   *
   * \copydoc doc_return_std
   */
  static int WriteServoId(DynaComm &comm, int nServoId, int nNewServoId);

  /*!
   * \brief Read the servo's baud rate from the servo's EEPROM.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_baud_rate
   *
   * \param comm              Dynamixel bus communication instance.
   * \param nServoId          Servo id.
   * \param [out] pBaudRate   Baud rate.
   *
   * \copydoc doc_return_std
   */
  static int ReadBaudRate(DynaComm &comm, int nServoId, int *pBaudRate);

  /*!
   * \brief Write the new baud rate to the servo's EEPROM.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_baud_rate
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   * \param nNewBaudRate    New baud rate.
   *
   * \copydoc doc_return_std
   */
  static int WriteBaudRate(DynaComm &comm, int nServoId, int nNewBaudRate);

  /*!
   * \brief Ping the given servo.
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   *
   * \return Returns true if the servo responded, else false.
   */
  static bool Ping(DynaComm &comm, int nServoId);

  /*!
   * \brief Reset the given servo back to default values.
   *
   * \warning All configuration data are lost.
   *
   * \param comm            Dynamixel bus communication instance.
   * \param nServoId        Servo id.
   *
   * \copydoc doc_std_return
   */
  static int Reset(DynaComm &comm, int nServoId);

  //
  // Abstract read/write functions.
  //

  virtual int CfgReadRotationLimits(uint_t *pCwLim, uint_t *pCcwLim) = 0;

  virtual int CfgWriteRotationLimits(uint_t uCwLim, uint_t uCcwLim) = 0;

  virtual int CfgReadTemperatureLimit(uint_t *pTempLim) = 0;

  virtual int CfgWriteTemperatureLimit(uint_t uTempLim) = 0;

  virtual int CfgReadVoltageLimits(uint_t *pMinVoltLim,
                                   uint_t *pMaxVoltLim) = 0;

  virtual int CfgWriteVoltageLimits(uint_t uMinVoltLim,
                                    uint_t uMaxVoltLim) = 0;

  virtual int CfgReadMaxTorqueLimit(uint_t *pMaxTorqueLim) = 0;

  virtual int CfgWriteMaxTorqueLimit(uint_t uMaxTorqueLim) = 0;

  virtual int CfgReadAlarmShutdownMask(uint_t *pAlarmMask) = 0;

  virtual int CfgWriteAlarmShutdownMask(uint_t uAlarmMask) = 0;

  virtual int CfgReadServoMode(uint_t *pServoMode) = 0;

  virtual int CfgWriteServoMode(uint_t uCwLim, uint_t uCcwLim) = 0;

  virtual int CfgWriteServoModeContinuous() = 0;

  virtual int ReadTorqueEnable(bool *pState) = 0;

  virtual int WriteTorqueEnable(bool bState) = 0;

  virtual int ReadLed(bool *pState) = 0;

  virtual int WriteLed(bool bState) = 0;

  virtual int ReadControlMethod(DynaServoCtlMethod_T *pCtlMethod) = 0;

  virtual int WriteControlMethod(DynaServoCtlMethod_T &ctlMethod) = 0;

  virtual int ReadGoalPos(int *pGoalPos) = 0;

  virtual int WriteGoalPos(int nGoalPos) = 0;

  virtual int ReadGoalSpeed(int *pGoalSpeed) = 0;

  virtual int WriteGoalSpeed(int nGoalSpeed) = 0;

  virtual int ReadMaxTorqueLimit(uint_t *pMaxTorqueLim) = 0;

  virtual int WriteMaxTorqueLimit(uint_t uMaxTorqueLim) = 0;

  virtual int ReloadMaxTorqueLimit() = 0;

  virtual int ReadCurPos(int *pCurPos) = 0;

  virtual int ReadCurSpeed(int *pCurSpeed) = 0;

  virtual int ReadCurLoad(int *pCurLoad) = 0;

  virtual int ReadDynamics(int *pCurPos, int *pCurSpeed, int *pCurLoad) = 0;

  virtual int ReadHealth(uint_t *pAlarms,
                         int    *pCurLoad,
                         uint_t *pCurVolt,
                         uint_t *pCurTemp) = 0;

  virtual int ReadIsMoving(bool *pState) = 0;

  virtual int  Read(uint_t uAddr, uint_t *pVal) = 0;

  virtual int  Write(uint_t uAddr, uint_t uVal) = 0;

  virtual bool Ping() = 0;

  virtual int  Reset() = 0;

  virtual int SyncData() = 0;

  virtual int SyncCfg() = 0;

  virtual int SyncState() = 0;

  virtual void Dump() = 0;

protected:
  /*! servo linkage */
  typedef struct
  {
    uint_t      m_uLinkType;    ///< linked type \ref dyna_servo_link_type
    DynaServo  *m_pServoMate;   ///< linked servo mate
    bool        m_bRotReversed; ///< do [not] rotate in opposite directions
  } CrossLink_T;

  DynaComm         &m_comm;         ///< attached Dynamixel bus comm. object
  int               m_nServoId;     ///< servo id
  DynaServoSpec_T   m_spec;         ///< servo specification
  DynaServoCfg_T    m_cfg;          ///< servo shadowed EEPROM configuration
  DynaServoState_T  m_state;        ///< servo shadowed RAM state
  CrossLink_T       m_link;         ///< servo cross linkage
  DynaAgent_T      *m_pAgent;       ///< servo agent
  void             *m_pAgentArg;    ///< servo agent callback argument
  int               m_nErrorCode;   ///< class instance errored state

  friend class DynaServoGeneric;
  friend class DynaServoAX12;
  friend class DynaServoEX106P;
  friend class DynaServoMX28;
  friend class DynaServoMX64;
  friend class DynaServoMX106;
  friend class DynaServoRX10;
  friend class DynaServoRX24F;
  friend class DynaServoRX28;
  friend class DynaServoRX64;
  friend class DynaChain;

  /*!
   * \brief Initialize servo class instance.
   *
   * \param comm      Dynamixel bus communication instance.
   * \param nServoId  Servo Id.
   * \param uModelNum Servo model number.
   * \param uFwVer    Servo firmware version.
   */
  void Init(int nServoId, uint_t uModelNum, uint_t uFwVer);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Linking Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  virtual int CalcMatesGoalPos(int nGoalPos, int *pGoalPosMate) = 0;

  virtual int CalcMatesGoalSpeed(int nGoalSpeed) = 0;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Field Packing Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  virtual uint_t PackGoalSpeed(int nGoalSpeed) = 0;

  virtual int UnpackGoalSpeed(uint_t uVal) = 0;

  virtual int UnpackCurSpeed(uint_t uVal) = 0;

  virtual int UnpackCurLoad(uint_t uVal) = 0;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Other Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

 /*!
  * \brief Dump the servo control tabl values to stdout.
  *
  * \param sTblName  Name of the control table.
  * \param tblInfo   Control table info. 
  * \param uSize     Control table info size (number of entries). 
  */
  void DumpCtlTbl(const char             *sTblName,
                  const DynaCtlTblEntry_T tblInfo[],
                  size_t                  uSize);
};


#endif // _DYNA_SERVO_H
