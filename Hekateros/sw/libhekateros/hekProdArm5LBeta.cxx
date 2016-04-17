////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd5LBeta.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros 5 DoF long beta robotic arm static specification.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows
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

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekProdArm5LBeta.h"


using namespace std;
using namespace hekateros;

/*!
 * \brief Specification of links.
 *
 * \par Data:
 * name,\n
 * length(mm)
 *
 * TODO
 */
const HekSpecLink_T
                hekateros::HekProdArm5LBetaSpecLinks[HekProdArm5LBetaNumLinks] =
{
  // fixed footprint base to base
  { "base_fixed",
    0.0
  },

  // base to upper arm
  { "base",
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
                  hekateros::HekProdArm5LBetaSpecJoints[HekProdArm5LBetaDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.35,
    0.0, 0.0, HekLimitTypeNone,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -80.0, 122.0, HekLimitTypeNone,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -80.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -130.0, 130.0, HekLimitTypePhys,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -110.0, 110.0, HekLimitTypePhys,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0,
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeNone,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    3, 4
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
            hekateros::HekProdArm5LBetaSpecServos[HekProdArm5LBetaNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, 80.0},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  80.0},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, 80.0},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  80.0},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, 80.0},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  80.0}
};
