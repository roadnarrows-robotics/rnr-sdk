////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd4L.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-10-06 15:13:20 -0600 (Mon, 06 Oct 2014) $
 * $Rev: 3773 $
 *
 * \brief Hekateros 4 DoF long robotic arm static specification.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
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

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekProdArm4L.h"

using namespace std;
using namespace hekateros;


/*!
 * \brief Specification of links.
 *
 * \par Data:
 * link_name,\n
 * length(mm)
 *
 * TODO
 */
const HekSpecLink_T hekateros::HekProdArm4LSpecLinks[HekProdArm4LNumLinks] =
{
  // fixed footprint base to base
  { "base_fixed",
    0.0
  },

  // upper arm to forearm
  { "upper_arm",
    406.27
  },

  // forearm to wrist
  { "forearm",
    401.6
  },

  // wrist to end effector zero point
  { "tool_zero",
    0.0
  }
};


//------------------------------------------------------------------------------
// 4L Arm, v1.1
//------------------------------------------------------------------------------

/*!
 * \brief Specification of joints.
 *
 * \par Data:
 * joint_name,\n
 * master_servo_id, slave_servo_id, joint_type, gear_ratio,\n
 * min_phy_limit(deg), max_phy_limit(deg), limit_types,\n
 * {
 *  {io_bit_0,
 *    min_edge_pos(deg), min_black_pos(deg),
 *    center_pos(deg),
 *    max_black_pos(deg), max_edge_pos(deg)
 *  },
 *  {io_bit_1,
 *    min_edge_pos(deg), min_black_pos(deg),
 *    center_pos(deg),
 *    max_black_pos(deg), max_edge_pos(deg)
 *  },
 * calib_pos(deg), balanced_pos(deg), park_pos(deg)\n
 * parent_link_idx, child_link_index
 */
const HekSpecJoint_T
                  hekateros::HekProdArm4LSpecJoints_1_1[HekProdArm4LDoF] =
{
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -77.0, 120.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -72.0, -77.0, 13.5, 104.0, 99.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    0, 1
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -135.0, 0.0, 135.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    1, 2
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -114.0, 114.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -90.0, -95.0, 0.0, 95.0, 90.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 30.0, 
    2, 3
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -9.25, -3.75, 0.0, 3.75, 9.25 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    2, 3
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
              hekateros::HekProdArm4LSpecServos_1_1[HekProdArm4LNumServos] =
{
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  60.0},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, 60.0},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  60.0},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, 60.0},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  60.0}
};
