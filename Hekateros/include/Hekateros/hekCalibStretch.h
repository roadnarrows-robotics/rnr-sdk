////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekCalibStretch.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-11 14:47:16 -0700 (Wed, 11 Feb 2015) $
 * $Rev: 3867 $
 *
 * \brief HekCalibStretch - Hekateros calibration by stretching class interface.
 *
 * Yawn. Gosh if feels good to stretch! Now where is my coffee?
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
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

#ifndef _HEK_CALIB_STRETCH_H
#define _HEK_CALIB_STRETCH_H

#include "Hekateros/hekateros.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekCalib.h"


namespace hekateros
{
  //
  // Forward declaraions.
  //
  class HekRobot;

  /*!
   * \brief Hekateros robotic manipulator calibration by stretching class.
   *
   * The hekateros will do a choreographed sequence of moves to find the limits
   * or top-dead-centers for all joints.
   *
   * \note It is best if the user places the arm in the upright position and
   * with not obstructions in the workspace prior to executing sequence.
   *
   * \par Move Sequence:
   * \li End effector open and close limits. (torque determines this range).
   * \li Wrist rotation top dead center.
   * \li Wrist pitch clockwise and counter-clockwise limits.
   * \li Elbow clockwise and counter-clockwise limits.
   * \li Shoulder clockwise and counter-clockwise limits.
   * \li Base rotation top dead center.
   */
  class HekCalibStretch : public HekCalib
  {
  public: 

    /*!
     * \breif Default initialization constructor.
     *
     * \param robot   Instance of \h_hek robot.
     */
    HekCalibStretch(HekRobot &robot) : HekCalib(robot)
    {
    }
  
    /*!
     * \breif Destructor.
     */
    virtual ~HekCalibStretch()
    {
    }

    /*!
     * \brief Calibrate the \h_hek's robotic arm.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrate();

  protected:
    /*!
     * \brief Calibrate a continuously rotating joint by top-dead-center
     * electronic limits.
     *
     * The joint has optical limit switch(es) at defined location, typically
     * at 0\h_deg and/or, at 180\h_deg. A small occlusion band triggers
     * the (occludes) the switch(es).
     *
     * The joint is rotated until the electronic limits are triggered to find
     * top-dead-center.
     *
     * When the joint calibration moves are finished, the joint is repositioned
     * at its new home position.
     *
     * \par Heuristic:
     * light = optical switch unoccluded (closed)\n
     * dark  = optical switch occluded (open)
     *  -# If starting in the light
     *    -# Rotate a small number of degrees in minimum direction.
     *    -# If light, rotate joint in the oposite direction 360\h_deg.
     *    -# If light, return with failure.
     *  -# Started in or moved to the dark.
     *    -# Now find an edge of occlusion band (dark to light).
     *    -# Move back to find opposite edge.
     *    -# Calculated center of band from both edge positions and rotate
     *       there.
     *    -# Rotate to 0\h_deg.
     *    -# Reset odometer.
     *    -# Return with success.
     *
     * \param joint   Robotic joint to calibrate.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrateJointTopDeadCenter(HekRobotJoint &joint);

    /*!
     * \brief Calibrate a rovolute joint by electronic limits.
     *
     * The joint has an electronic limit switch and a occlusion band that marks
     * the minimum and maximum rotation limits.
     *
     * The joint is rotated until the electronic limit are triggered to find
     * these min,max limits.
     *
     * When the joint calibration moves are finished, the joint is repositioned
     * at its new home position.
     *
     * \par Heuristic:
     * light = optical switch unoccluded (closed)\n
     * dark  = optical switch occluded (open)
     *  -# If starting in the dark.
     *    -# Get the park position from spec to determine best guess direction.
     *    -# Try to rotate \e X degrees to find the light, stopping either when
     *     first light is detected, torqued limit is reached,
     *     or have rotated \e X\h_deg.
     *    -# If in the light, goto 4.
     *    -# Else if over torqued (probably at physical limit).
     *      -# Reverse rotate \e X degrees to find the light, stopping either
     *        when first light is detected, torqued limit is reached,
     *        or have rotated \e X\h_deg.
     *      -# If in the light, goto 4.
     *    -# Else cannot get out of the dark, return with failure.
     *  -# Else starting in the light.
     *      -# Rotate in minimum direction \e Y degrees to find the dark,
     *         stopping either when first dark is detected,
     *         torqued limit is reached,
     *          or have rotated \e Y\h_deg.
     *      -# If in the dark, goto 4.
     *      -# Else probably obstructed.
     *        -# While less than \e MAX tries.
     *          -# Rotate in maximum direction \e Y degrees to find the dark,
     *            stopping either when first dark is detected,
     *            torqued limit is reached,
     *              or have rotated \e Y\h_deg.
     *          -# If in the dark, goto 4.
     *          -# Else if have helpful parent joint (e.g. shoulder for elbow).
     *            -# Rotate parent joint position \e Z degrees.
     *            -# If no movement return with failure.
     *          -# Else break loop.
     *  -# Return with failure.
     *  -# Final Stage.
     *    -# Fine tune to find edge precisely.
     *    -# While less than \e MAX tries.
     *      -# Rotate to 0\h_deg.
     *      -# If reached 0\h_deg.
     *          -# Reset odometer.
     *          -# Return with success.
     *      -# Else if have helpful parent joint.
     *        -# Rotate parent joint position \e Z degrees.
     *        -# If no movement return with failure.
     *      -# Else break loop.
     *    -# Return with failure
     *
     * \param joint   Robotic joint to calibrate.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrateJointByLimits(HekRobotJoint &joint);

    /*!
     * \brief Perform final calibration by limits calculations and movements.
     *
     * \param joint           Robotic joint.
     * \param byOptMask       Joint optical limit bit mask;
     * \param nEdge           Which edge triggered limit switch. The minimum,
     *                        unknown, or maximum leading edge is specified as
     *                        \h_lt 0, 0, or \h_gt 0, respectively.
     *
     * \copydoc doc_return_std
     */
    virtual int calibByLimitsFinal(HekRobotJoint &joint,
                                   byte_t         byOptMask,
                                   int            nEdge);

    /*!
     * \brief Calibrate joint by torque limits.
     *
     * The joint is rotated to its minimum and maximum physical end
     * points. When the torque reaches a software defined limit, an endpoint
     * is considered reached.
     *
     * From the joint specification and detected limits, the home position
     * at 0\h_deg is fine tuned.
     *
     * When the joint calibration moves are finished, the joint is repositioned
     * at its new home position.
     *
     * \par Heuristic:
     *  -# Rotate in minimum direction until torque limit reached.
     *  -# Mark position.
     *  -# Rotate in maximum direction until torque limit reached.
     *  -# Mark position.
     *  -# Calculate 0\h_deg based on expected min,max positions and detected
     *  min,max positions.
     *  -# Rotate to 0\h_deg.
     *  -# Reset odomter.
     *
     * \param joint   Robotic joint to calibrate.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrateJointByTorqueLimits(HekRobotJoint &joint);

    /*!
     * \brief Calibrate joint by trust.
     *
     * The joint is assumed to be already position at zero degrees by the user.
     *
     * \param joint   Robotic joint to calibrate.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrateJointByTrust(HekRobotJoint &joint);
    
    /*!
     * \brief Fine tune joint to top dead center of optical limit.
     *
     * The position of the joint must be in the dark, occluded region of a band
     * prior to calling this function.
     *
     * \par TDC Fine Tuning:
     * \verbatim
     *  fine tune step           occlusion band
     *                       _____________________
     *   0 (start)           |     *             |
     *                       |     *             |
     *   1 lit_edge_0    * <------ *             |
     *                   *   |                   |
     *   2 dark_edge_0   * ----> *               |
     *                       |   *               |
     *   3 lit_edge_1        |   * ----------------> *
     *                       |                   |   *
     *   4 center            |         * <---------- *
     *                       |___________________|
     *
     * \endverbatim
     *
     * \param joint           Robotic joint.
     * \param byOptMask       Joint optical limit bit mask;
     * \param nEdge           Which edge triggered limit switch. The minimum,
     *                        unknown, or maximum leading edge is specified as
     *                        \h_lt 0, 0, or \h_gt 0, respectively.
     *                        <b>Not used</b>
     * \param [out] fPosition Difference of determined uncalibrated joint
     *                        position at TDC and the designed TDC (radians).
     *
     * \copydoc doc_return_std
     */
    virtual int fineTuneTDC(HekRobotJoint &joint,
                            byte_t         byOptMask,
                            int            nEdge,
                            double        &fPosition);

    
    /*!
     * \brief Fine tune joint to optical limit edge.
     *
     * \par Limit Fine Tuning:
     * \verbatim
     *  fine tune step           occlusion band
     *                       _____________________
     *   0 (start)           |     *             |
     *                       |     *             |
     *   1 lit_edge      * <------ *             |
     *                   *   |                   |
     *   2 dark_edge     * ----> *               |
     *                       |___________________|
     *
     * Or:
     *
     *  fine tune step           occlusion band
     *                       _____________________
     *   0 start         *   |                   |
     *                   *   |                   |
     *   2 dark_edge     * ----> *               |
     *                       |___________________|
     * \endverbatim
     *
     * \param joint           Robotic joint.
     * \param byOptMask       Joint optical limit bit mask;
     * \param nEdge           Which edge triggered limit switch. The minimum,
     *                        unknown, or maximum leading edge is specified as
     *                        \h_lt 0, 0, or \h_gt 0, respectively.
     * \param [out] fDeltaPos Delta position (radians) from current to zero
     *                        point as determined by the optical limit.
     *
     * \copydoc doc_return_std
     */
    virtual int fineTuneLimit(HekRobotJoint &joint,
                              byte_t         byOptMask,
                              int            nEdge,
                              double        &fDeltaPos);

    /*!
     * \brief Calculate two joints' gear ratios.
     *
     * \param strJointName1   First joint.
     * \apram strJointName2   Second joint.
     *
     * \return  joint1 gear ratio to joint 2 gear ratio.
     */
    double calcJointRatios(const std::string &strJointName1,
                           const std::string &strJointName2);


    /*!
     * \brief Move joint until unoccluded optical limit position is detected.
     *
     * Since moving a given joint during calibration may result in large 
     * trajectory arcs, other joints may be reposition to minimize this effects.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName  Name of joint.
     * \param nDir          Direction indicator.
     * \param fJointGoalPos Joint goal position (radians).
     * \param fJointGoalVel Joint goal velocity (radians/second).
     * \param byMask        Mask of limits to check.
     *
     * \copydoc doc_return_std
     */
    virtual int multiMoveToLight(const std::string &strJointName,
                                 int                nDir,
                                 double             fJointGoalPos,
                                 double             fJointGoalVel,
                                 byte_t             byMask);

    /*!
     * \brief Move joint until occluded optical limit position is detected.
     *
     * Since moving a given joint during calibration may result in large 
     * trajectory arcs, other joints may be reposition to minimize this effects.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName  Name of joint.
     * \param nDir          Direction indicator.
     * \param fJointGoalPos Joint goal position (radians).
     * \param fJointGoalVel Joint goal velocity (radians/second).
     * \param byMask        Mask of limits to check.
     *
     * \copydoc doc_return_std
     */
    virtual int multiMoveToDark(const std::string &strJointName,
                                 int                nDir,
                                 double             fJointGoalPos,
                                 double             fJointGoalVel,
                                 byte_t             byMask);

    /*!
     * \brief Get the parent joint (and link) that could aid in calibration.
     *
     * Only a few joints serve this function.
     *
     * \param nChildServoId   Child joint's master servo id.
     *
     * \return If parenet exists, returns pointer to parent joint.
     * Otherwise returns NULL.
     */
    virtual HekRobotJoint *getHelpfulParentJoint(int nChildServoId);
  };

} // namespace hekateros

#endif // _HEK_CALIB_STRETCH_H
