////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaServoEX106P.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief EX-106+ Dynamixel Servo Class Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
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

#ifndef _DYNA_SERVO_EX106P_H
#define _DYNA_SERVO_EX106P_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/EX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"



// ---------------------------------------------------------------------------
// EX-106+ Extended Data Types
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_types
 * \defgroup dyna_lib_types_ex106p EX-106+ Extended Data Types
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_lib_types_ex106p 
 * \brief Dynamixel EX-106+ Driver Mode Configuration Extension Structure.
 */
typedef struct
{
  bool    m_bDriveModeIsMaster; ///< master (slave) servo
  bool    m_bDriveModeIsNormal; ///< normal (reverse) rotation to each other
} DynaEX106PCfgExt_T;

/*!
 * \ingroup dyna_lib_types_ex106p
 * \brief Dynamixel EX-106+ Sensed Electrical Current State Extension Structure.
 */
typedef struct
{
  uint_t  m_uSensedCurrentMilliAmps;   ///< sensed current milli-amperes
  uint_t  m_uSensedCurrentTorqueDir;   ///< sensed applied torque direction
} DynaEX106PStateExt_T;


// ---------------------------------------------------------------------------
// EX-106+ Dynamixel Servo Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief EX-106+ Dynamixel Servo Class.
 *
 * The DynaServoEX106P class provides the specific interface to the EX-106+
 * Dynamixel servos
 */
class DynaServoEX106P : public DynaServoGeneric
{
public:
  /*! modem number */
  static const int DYNA_MODEL_NUM = DYNA_MODEL_NUM_EX106P;

  /*!
   * \brief Bare-bones initialization constructor.
   *
   * May be used be derived classes to avoid undue communication and
   * initializaton overhead.
   *
   * \param comm      Dynamixel bus communication instance.
   * 
   */
  DynaServoEX106P(DynaComm &comm) : DynaServoGeneric(comm)
  {
  }

  /*!
   * \brief Initialization constructor.
   *
   * \param comm      Dynamixel bus communication instance.
   * \param nServoId  Servo Id.
   * \param uModelNum Servo model number.
   * \param uFwVer    Servo firmware version.
   */
  DynaServoEX106P(DynaComm &comm,
                   int       nServoId,
                   uint_t    uModelNum = DYNA_MODEL_NUM,
                   uint_t    uFwVer    = DYNA_FWVER_NA);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaServoEX106P();


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  /*!
   * \brief Get servo extended configuration.
   *
   * \return Reference to shadowed configuration extension data.
   */
  virtual const DynaEX106PCfgExt_T &GetConfigurationExt()
  {
    return m_cfgExt;
  }

  /*!
   * \brief Get servo extended state.
   *
   * \return Reference to shadowed state extension data.
   */
  virtual const DynaEX106PStateExt_T &GetStateExt()
  {
    return m_stateExt;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

 /*!
   * \brief Read from the servo configuration EEPROM the drive mode.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \note Drive mode requires the paired EX-106+'s to be connected via the
   * synchronization cable.
   *
   * \par Control Table:
   * \ref dyna_ex106p_memmap_drive_mode.
   *
   * \param [out] pIsMaster   This servo is [not] the master.
   * \param [out] pIsNormal   Servos [don't] rotate in same directions.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadDriveMode(bool *pIsMaster, bool *pIsNormal);

  /*!
   * \brief Write to the servo configuration EEPROM the new drive mode.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written appropriately.
   *
   * \note Drive mode requires the paired EX-106+'s to be connected via the
   * synchronization cable.
   *
   * \par Control Table:
   * \ref dyna_ex106p_memmap_drive_mode.
   *
   * \param pIsMaster   This servo is [not] the master.
   * \param pIsNormal   Servos [don't] rotate in same directions.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteDriveMode(bool bIsMaster, bool bIsNormal);

  /*!
   * \brief Read from the servo configuration EEPROM the sensed current data.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_ex106p_memmap_drive_mode.
   *
   * \param [out] pMillAmps   Sensed mAmps.
   * \param [out] pTorqueDir  Sensed applied torque direction.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadSensedCurrent(uint_t *pMilliAmps, uint_t *pTorqueDir);

  /*!
   * Read a raw value from the servo EEPROM/RAM control table.
   *
   * \warning
   * The shadowed configuration and state data are not update.\n
   * Linked servos are not kept in sync.
   *
   * \param uAddr       Control table address.
   * \param [out] pVal  Read raw value.
   *
   * \copy doc_return_std
   */
  virtual int  Read(uint_t uAddr, uint_t *pVal);

  /*!
   * Write a raw value to the servo EEPROM/RAM control table.
   *
   * \warning The shadowed configuration and state data are not updated and will
   * hence be out of sync.
   *
   * \warning Any linked master-slave servos may get out of sync and may result
   * in physical damage.
   *
   * \param uAddr   Control table address.
   * \param uVal    Raw value to write.
   *
   * \copy doc_return_std
   */
  virtual int  Write(uint_t uAddr, uint_t uVal);

  /*!
   * \brief Synchronize the shadowed configuration to the servo control table
   * EEPROM configuration.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncCfg();

  /*!
   * \brief Synchronize the shadowed state data to the servo control table RAM
   * state.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncState();

  /*!
   * \brief Dump contents of the servo EEPROM and RAM control tables.
   */
  virtual void Dump();

protected:
  DynaEX106PCfgExt_T    m_cfgExt;     ///< configuration extension data
  DynaEX106PStateExt_T  m_stateExt;   ///< state extension data

  /*!
   * \brief Initialize servo class instance.
   *
   * \param nServoId  Servo Id.
   * \param uFwVer    Servo firmware version.
   */
  void Init(int nServoid, uint_t uFwVer);

  /*!
   * \brief Initialize servo fixed specification data.
   */
  void InitSpec();

  /*!
   * \brief Initialize servo configuration data.
   */
  void InitCfg();

  /*!
   * \brief Initialize servo state data.
   */
  void InitState();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Linking Fuctions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Field Packing Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Pack drive mode.
   *
   * \param bIsMaster   This servo is [not] the master.
   * \param bIsNormal   Servos [don't] rotate in same directions.
   *
   * \return Returns packed value.
   */
  uint_t PackDriveMode(bool bIsMaster, bool bIsNormal);

  /*!
   * \brief Unpack drive mode.
   *
   * \param uVal            Packed field.
   * \param [out] pIsMaster This servo is [not] the master).
   * \param [out] pIsNormal Servos [don't] rotate in same directions.
   *
   * \copydoc doc_return_std
   */
  int UnpackDriveMode(uint_t uVal, bool *pIsMaster, bool *pIsNormal);

  /*!
   * \brief Unpack sensed current.
   *
   * \param uVal              Packed field.
   * \param [out] pMilliAmps  Sensed mAmps.
   * \param [out] pTorqueDir  Sensed torque direction. See \ref dyna_servo_dir.
   *
   * \copydoc doc_return_std
   */
  int UnpackSensedCurrent(uint_t uVal, uint_t *pMilliAmps, uint_t *pTorqueDir);
};


#endif // _DYNA_SERVO_EX106P_H
