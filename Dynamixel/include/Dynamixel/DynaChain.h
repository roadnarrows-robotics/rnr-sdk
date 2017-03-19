////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaChain.h
//
/*! \file
 * $LastChangedDate: 2015-01-12 11:54:07 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3846 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Servo Chain Container Base Class Interface.
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

#ifndef _DYNA_CHAIN_H
#define _DYNA_CHAIN_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"


// ---------------------------------------------------------------------------
// Dynamixel Chain Container Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief Dynamixel Chain Container Base Class.
 *
 * The DynaChain base class provides a coherent and synchronized interface to
 * a set of servos communicating on the same Dynamixel bus.
 */
class DynaChain
{
public:
  /*!
   * \brief Default initialization constructor.
   *
   * The chain is bound to the given Dynamixel bus communication object.
   * No servos are present in the chain until explicitly added.
   * 
   * A background thread may be automatically started (default) to:
   * \li control the positioning of servos configured in continuous mode.
   * \li monitor the dynamics of the servos in the chain.
   * \li monitor the health of the servos in the chain.
   *
   * It is recommended that if any of the servos in the chain are in continuous
   * (wheel) mode and have 360\h_deg positioning data, that the background
   * thread be started.
   *
   * \param comm            Dynamixel bus communication instance.
   */
  DynaChain(DynaComm &comm);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaChain();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * Test if servo is in the chain.
   *
   * \param nServoId  Servo id.
   *
   * \return Returns true if servo is the chain, false otherwise.
   */
  virtual bool HasServo(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) &&
            (nServoId <= DYNA_ID_MAX) &&
            (m_pChain[nServoId] != NULL))?  true: false;
  }

  /*!
   * Get the servo in the chain
   *
   * \param nServoId  Servo id.
   *
   * \return 
   * Returns pointer to the servo if servo is the chain.\n
   * Returns NULL otherwise.
   */
  virtual DynaServo *GetServo(int nServoId)
  {
    return ((nServoId < DYNA_ID_MIN) || (nServoId > DYNA_ID_MAX))?
              NULL: m_pChain[nServoId];
  }

  /*!
   * \brief Get the number of servos currently in chain.
   *
   * \return Number in chain.
   */
  virtual uint_t GetNumberInChain() const
  {
    return m_uNumInChain;
  }

  /*!
   * \brief Software link two unlinked servos in a master-slave configuration.
   *
   * The servos must be of the same Dynamixel model number.
   *
   * \param nServoIdMaster  Master servo id.
   * \param nServoIdSlave   Slave servo id.
   * \param bRotReversed    The linked servos do [not] rotate in opposite
   *                        directions.
   *
   * \copydoc doc_return_std
   */
  virtual int LinkServos(int  nServoIdMaster,
                         int  nServoIdSlave,
                         bool bIsReversed);

  /*!
   * \brief Unlink two software linked servos.
   *
   * \param nServoIdMaster  Master servo id.
   *
   * \copydoc doc_return_std
   */
  virtual int UnlinkServos(int nServoIdMaster);

  /*!
   * \brief Test if the servo is a master.
   *
   * A master is either the master servo in a linked master-slave configuration,
   * or an unlinked servo (a master unto thyself).
   *
   * \param nServoId  Servo id.
   *
   * \return Returns true if a master, false otherwise.
   */
  virtual bool IsMaster(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX) &&
            ((m_links[nServoId].m_uLinkType == DYNA_LINK_MASTER) ||
             (m_links[nServoId].m_uLinkType == DYNA_LINK_NONE)))? true: false;
  }

  /*!
   * \brief Test if this servo is a linked master.
   *
   * A linked master is the master servo in a linked master-slave configuration.
   *
   * \param nServoId  Servo id.
   *
   * \return Returns true if a linked master, false otherwise.
   */
  bool IsLinkedMaster(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX) &&
             (m_links[nServoId].m_uLinkType == DYNA_LINK_MASTER))? true: false;
  }

  /*!
   * \brief Test if this servo is a linked slave.
   *
   * A linked slave is the slave servo in a linked master-slave configuration.
   *
   * \param nServoId  Servo id.
   *
   * \return Returns true if a linked slave, false otherwise.
   */
  bool IsLinkedSlave(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX) &&
             (m_links[nServoId].m_uLinkType == DYNA_LINK_SLAVE))? true: false;
  }

  /*!
   * \brief Test if this servo is unlinked.
   *
   * \param nServoId  Servo id.
   *
   * \return Returns true if unlinked (or non-existent), false otherwise.
   */
  bool IsUnlinked(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX) &&
             (m_links[nServoId].m_uLinkType == DYNA_LINK_NONE))? true: false;
  }

  /*!
   * \brief Get linked mate.
   *
   * \param nServoId  Servo id.
   *
   * \return If the servo is linked, returns its linked mate's servo id.\n
   * Else returns \ref DYNA_ID_NONE.
   */
  int GetLinkedMateId(int nServoId) const
  {
    return ((nServoId >= DYNA_ID_MIN) && (nServoId <= DYNA_ID_MAX))?
             m_links[nServoId].m_nServoIdMate: DYNA_ID_NONE;
  }

  /*!
   * \brief Get the number of servos currently in chain.
   *
   * \return Number of masters in chain.
   */
  virtual uint_t GetNumberOfMastersInChain();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Chain Servo Add/Remove Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Create a new servo and add it to the Dynamixel chain.
   *
   * Communication with servo is checked prior to adding the servo.
   *
   * \note
   * Any existing servo with the same servo id will be replaced by this servo.\n
   * Any existing linked servo configuration will be preserved if possible.
   *
   * \param nServoId      Servo id to add.
   *
   * \copydoc doc_return_std
   */ 
  virtual int AddNewServo(int nServoId);

  /*!
   * \brief Scan and add all discovered and created servos to the Dynamixel
   * chain.
   *
   * \note
   * All existing servos in the chain are removed and deleted prior to the
   * scan.\n
   * Any existing linked servo configuration will be preserved if possible.
   *
   * \return Returns the number of servos added.
   */ 
  virtual int AddNewServosByScan();

  /*!
   * \brief Force create and add a servo to the Dynamixel chain.
   *
   * No communication with servo is attempted, nor any data validation
   * performed.
   *
   * \note
   * Any existing servo with the same servo id will be replaced by this servo.\n
   * Any existing linked servo configuration will be preserved if possible.
   *
   * \param nServoId      Servo id to add. 
   * \param uModelNum     Servo model number.
   *
   * \copydoc doc_return_std
   */ 
  virtual int AddNewServoForced(int nServoId, uint_t uModelNum);

  /*!
   * \brief Remove from chain and delete.
   *
   * \param nServoId  Servo Id to delete.
   *
   * \copydoc doc_return_std
   */
  virtual int RemoveServo(int nServoId);

  /*!
   * \brief Remove all servos from chain and delete.
   *
   * \copydoc doc_return_std
   */
  virtual int RemoveAllServos();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Master and Synchronous Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Move to the goal postition.
   *
   * The move will proceed at the current goal speed. If the servo is in 
   * continuous mode, but supports 360\h_deg positioning data, then a registered
   * external user control function is required.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param nServoId    Servo Id
   * \param uGoalOdPos  Goal positon (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int MoveTo(int nServoId, int nGoalOdPos);

  /*!
   * \brief Synchronous move servos to new goal positions.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions.
   *
   * The moves will proceed at the current goal speeds.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 2-tuples of type
   *                  (int,int) specifying the servo id and the goal end
   *                  position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int vSyncMoveTo(uint_t uCount, ...);

  /*!
   * \brief Synchronous move servos to new goal positions in tuple list.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions.
   *
   * The moves will proceed at the current goal speeds.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param tuplesPos Array of servo id, goal odometer position tuples.
   * \param uCount    Number of tuples.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncMoveTo(DynaPosTuple_T tuplesPos[], uint_t uCount);

  /*!
   * \brief Move at speed to the goal postition.
   *
   * For servos in continuous mode, speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify rotation in the clockwise direction,
   * stop, or rotation in the counterclockwise direction, respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control.
   *
   * If the servo is in continuous mode, but supports 360\h_deg positioning
   * data, then a registered external user control function is required.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO). The direction will be ignored.\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param nServoId    Servo Id
   * \param uGoalSpeed  Goal speed (raw).
   * \param uGoalOdPos  Goal position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int MoveAtSpeedTo(int nServoId, int nGoalSpeed, int nGoalOdPos);

  /*!
   * \brief Synchronous move servos to new goal positions at the given speeds.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions at the appropriate speeds.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 3-tuples of type
   *                  (int,int,int) specifying the servo id, goal speed, and
   *                  the end goal odometer position.
   *
   * \copydoc doc_return_std
   */
  virtual int vSyncMoveAtSpeedTo(uint_t uCount, ...);

  /*!
   * \brief Synchronous move servos to new goal positions at the given speeds.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions at the appropriate speeds.
   *
   * \todo Optimize. Add support for bg task continuous mode sync write, then 
   * control.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param tuplesSpeedPos  Array of servo id, goal speed, and goal odometer
   *                        position 3-tuples.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncMoveAtSpeedTo(DynaSpeedPosTuple_T tuplesSpeedPos[],
                                uint_t              uCount);

  /*!
   * \brief Move at speed.
   *
   * Speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify rotation in the clockwise direction,
   * stop, or rotation in the counterclockwise direction, respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS).
   *
   * \param nServoId      Servo Id
   * \param nGoalSpeed    Goal speed.
   *
   * \copydoc doc_return_std
   */
  virtual int MoveAtSpeed(int nServoId, int nGoalSpeed);

  /*!
   * \brief Synchronous move servos at the given speeds.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * at the appropriate speeds and directions.
   *
   * \par Servo Operational Modes:
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS).
   *
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 2-tuples of type
   *                  (int,int) specifying the servo id and goal speed.
   *
   * \copydoc doc_return_std
   */
  virtual int vSyncMoveAtSpeed(uint_t uCount, ...);

  /*!
   * \brief Synchronous move servos at the given speeds.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * at the appropriate speeds and directions.
   *
   * \par Servo Operational Modes:
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS).
   *
   * \param tuplesSpeed Array of servo id, goal speed 2-tuples.
   * \param uCount      Number of tuples.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncMoveAtSpeed(DynaSpeedTuple_T tuplesSpeed[],
                              uint_t           uCount);

  /*!
   * \brief Emergency stop all servos.
   *
   * All torque is removed.
   *
   * \copydoc doc_return_std
   */
  virtual int EStop();

  /*!
   * \brief Stop all servos.
   * 
   * Current torque settings are kept.
   *
   * \copydoc doc_return_std
   */
  virtual int Stop();

  /*!
   * \brief Freeze all servos at current position.
   *
   * Torque is applied.
   *
   * \copydoc doc_return_std
   */
  virtual int Freeze();

  /*!
   * \brief Release all servos from any applied torque. 
   *
   * Servos are free to rotate.
   *
   * \copydoc doc_return_std
   */
  virtual int Release();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Master and Synchronous Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Synchronous write to all of the servos in the chain to
   * enable/disable applied torque.
   *
   * If the torque state is false (off), no power is applied to the servo,
   * allowing it to be free rotated by any external force.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_torque_en.\n
   *
   * \param bState          Torque enabled (true) or disabled (off) state.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncWriteTorqueEnable(bool bState);

  /*!
   * \brief Synchronous write new goal positions for servos.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_goal_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 2-tuples of type
   *                  (int,int) specifying the servo id and the goal end
   *                  position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int vSyncWriteGoalPos(uint_t uCount, ...);

  /*!
   * \brief Synchronous write new goal positions for servos.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * to the appropriate end positions.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_goal_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param tuplesPos Array of servo id, goal odometer position tuples.
   * \param uCount    Number of tuples.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncWriteGoalPos(DynaPosTuple_T tuplesPos[], uint_t uCount);

  /*!
   * \brief Synchronous write new goal speed for for servos.
   *
   * For servos in continuous mode, speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify rotation in the clockwise direction,
   * stop, or rotation in the counterclockwise direction, respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * at the appropriate speeds and directions.
   *
   * \param uCount    Number of tuples.
   * \param ...       A variable argument list of uCount 2-tuples of type
   *                  (int,int) specifying the servo id and the goal speed.
   *
   * \copydoc doc_return_std
   */
  virtual int vSyncWriteGoalSpeed(uint_t uCount, ...);

  /*!
   * \brief Synchronous write new goal speed for servos.
   *
   * If a servo is in continuous mode, rotation will occur indefinitely
   * in the direction specified and at the given speed.
   *
   * If the servo is in servo mode, rotation will occur until the goal position
   * is achieved.
   *
   * If any servo is a linked master, the linked pair will moved synchronously
   * at the appropriate speeds and directions.
   *
   * \par Control Table:
   * \ref dyna_servo_memmap_goal_speed.\n
   * \ref dyna_servo_speed.\n
   * \ref dyna_servo_dir.
   *
   * \param tuplesSpeed Array of servo id,speed 2-tuples.
   * \param uCount      Number of tuples.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncWriteGoalSpeed(DynaSpeedTuple_T tuplesSpeed[], uint_t uCount);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Iterators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  /*!
   * \brief Start iteration over of entire servo chain.
   *
   * \param pIter   Pointer to iterator.
   *
   * \return 
   * On success, returns the starting servo id.\n
   * On end of chain or error, \ref DYNA_ID_NONE is returned.
   */
  virtual int IterStart(int *pIter);

  /*!
   * \brief Next iteration over of entire servo chain.
   *
   * \param pIter   Pointer to iterator.
   *
   * \return 
   * On success, returns the next servo id.\n
   * On end of chain or error, \ref DYNA_ID_NONE is returned.
   */
  virtual int IterNext(int *pIter);

  /*!
   * \brief Start iteration master servos over of entire servo chain.
   *
   * \param pIter   Pointer to iterator.
   *
   * \return 
   * On success, returns the starting servo id.\n
   * On end of chain or error, \ref DYNA_ID_NONE is returned.
   */
  virtual int IterStartMaster(int *pIter);

  /*!
   * \brief Next iteration of master servos over of entire servo chain.
   *
   * \param pIter   Pointer to iterator.
   *
   * \return 
   * On success, returns the next servo id.\n
   * On end of chain or error, \ref DYNA_ID_NONE is returned.
   */
  virtual int IterNextMaster(int *pIter);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Operators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Subscript operator.
   *
   * \param nServoId  Servo id.
   *
   * \return 
   * On success, returns pointer to servo object in chain.\n
   * On failure, return NULL.
   */
  const DynaServo *operator[](int nServoId);

protected:
  DynaComm          &m_comm;                  ///< bus communication instance
  DynaServo        *m_pChain[DYNA_ID_NUMOF];  ///< chain of servos
  int               m_nIIdx[DYNA_ID_NUMOF];   ///< indirect index
  DynaServoLink_T   m_links[DYNA_ID_NUMOF];   ///< servo linked info
  uint_t            m_uNumInChain;            ///< number of servos in chain
  uint_t            m_uNumMastersInChain;     ///< num of master servos in chain

  /*!
   * \brief Initialize chain class instance.
   */
  void Init();

  /*!
   * \brief Create new dervied DyanServo object and insert into chain.
   *
   * \param nServoId  Servo Id to delete.
   *
   * \copydoc doc_return_std
   */
  virtual int ChainEntryNew(int nServoId);

  /*!
   * \brief Remove servo from chain and delete.
   *
   * \param nServoId  Servo Id to delete.
   *
   * \copydoc doc_return_std
   */
  virtual int ChainEntryDelete(int nServoId);

  /*!
   * \brief Audit servo links and repair or delete.
   *
   * \copydoc doc_return_std
   */
  virtual void AuditLinks();

  /*!
   * \brief Set local link data for the given servo.
   *
   * \param nServiId        Servo id.
   * \param uLinkType       This servo's link type.
   * \param nServoIdMate    Mate servo id.
   * \param bRotReversed    The linked servos do [not] rotate in opposite
   *                        directions.
   */
  virtual void SetLinkData(int    nServoId,
                           uint_t uLinkType,
                           int    nServoIdMate,
                           bool   bRotReversed);

  /*!
   * \brief Clear local link data for the given servo.
   */
  virtual void ClearLinkData(int nServoId);
};


#endif // _DYNA_CHAIN_H
