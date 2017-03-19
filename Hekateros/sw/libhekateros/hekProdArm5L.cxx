////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd5L.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-12-10 10:43:51 -0700 (Thu, 10 Dec 2015) $
 * $Rev: 4239 $
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
#include "Hekateros/hekTune.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekProdArm5L.h"

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
const HekSpecLink_T hekateros::HekProdArm5LSpecLinks[HekProdArm5LNumLinks] =
{
  // fixed footprint base to base
  { "base_fixed",
    0.0
  },

  // fixed footprint base to base
  { "base_rot",
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
//  5L Arm v1.1
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
                    hekateros::HekProdArm5LSpecJoints_1_1[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.1,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -77.0, 120.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -72.0, -77.0, 13.5, 104.0, 99.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -135.0, 0.0, 135.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -114.0, 114.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -90.0, -95.0, 0.0, 95.0, 90.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -9.25, -3.75, 0.0, 3.75, 9.25 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_1_1[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};


//------------------------------------------------------------------------------
// 5L Arm v1.2
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
                    hekateros::HekProdArm5LSpecJoints_1_2[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.0,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -77.0, 120.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -72.0, -77.0, 13.5, 104.0, 99.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -135.0, 0.0, 135.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -114.0, 114.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -90.0, -95.0, 0.0, 95.0, 90.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -9.25, -3.75, 0.0, 3.75, 9.25 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_1_2[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};


//------------------------------------------------------------------------------
// 5L Arm v1.3
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
                    hekateros::HekProdArm5LSpecJoints_1_3[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.0,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -80.0, 115.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -69.85, -72.85, 17.5, 115.0, 112.85 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -134.0, 0.0, 134.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -110.0, 110.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -90.0, -93.0, 0.0, 93.0, 90.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -9.25, -6.25, 0.0, 6.25, 9.25 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_1_3[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};


//------------------------------------------------------------------------------
// 5L Arm v1.3.5  Special SIUE Hybrid Arm
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
                    hekateros::HekProdArm5LSpecJoints_1_3_5[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 40.0/17.0,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -80.0, 115.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -69.85, -72.85, 17.5, 115.0, 112.85 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -134.0, 0.0, 134.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -115.9, 115.9, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -112.5, -115.5, 0.0, 115.5, 112.5 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -15.0, -12.0, 0.0, 12.0, 15.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_1_3_5[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};


//------------------------------------------------------------------------------
// 5L Arm v1.4
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
                    hekateros::HekProdArm5LSpecJoints_1_4[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.0,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -80.0, 115.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -69.85, -72.85, 17.5, 115.0, 112.85 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -134.0, 0.0, 134.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -115.9, 115.9, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -112.5, -115.5, 0.0, 115.5, 112.5 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -15.0, -12.0, 0.0, 12.0, 15.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_1_4[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};


//------------------------------------------------------------------------------
// 5L Arm v2.0
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
                    hekateros::HekProdArm5LSpecJoints_2_0[HekProdArm5LDoF] =
{
  { "base_rot",
    HekServoIdBase, DYNA_ID_NONE, HekJointTypeContinuous, 2.0,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0Base0,    -3.0,  -1.0,   0.0,   1.0,   3.0 },
      { HekIOExpPort0Base180, 177.0, 179.0, 180.0, 181.0, 183.0 }
    },
    0.0, 0.0, 0.0,
    0, 1
  },
  { "shoulder",
    HekServoIdShoulderL, HekServoIdShoulderR, HekJointTypeRevolute, 2.0,
    -80.0, 115.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Shoulder, -69.85, -72.85, 17.5, 115.0, 112.85 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, -30.0, -75.0,
    1, 2
  },
  { "elbow",
    HekServoIdElbow, DYNA_ID_NONE, HekJointTypeRevolute, 4.0,
    -135.0, 135.0, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0Elbow, -131.0, -134.0, 0.0, 134.0, 131.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 120.0, 130.0,
    2, 3
  },
  { "wrist_pitch",
    HekServoIdWristPitch, DYNA_ID_NONE, HekJointTypeRevolute, 1.5,
    -115.9, 115.9, HekLimitTypePhys|HekLimitTypeElec,
    {
      { HekIOExpPort0WristPitch, -112.5, -115.5, 0.0, 115.5, 112.5 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 50.0, 
    3, 4
  },
  { "wrist_rot",
    HekServoIdWristRot, DYNA_ID_NONE, HekJointTypeContinuous, 1.5,
    0.0, 0.0, HekLimitTypeElecTDC,
    {
      { HekIOExpPort0WristRot0, -15.0, -12.0, 0.0, 12.0, 15.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    0.0, 0.0, 0.0,
    4, 5
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdArm5LSpecServos_2_0[HekProdArm5LNumServos] =
{
  {HekServoIdBase,        true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdShoulderL,   true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdShoulderR,   false,  true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdElbow,       true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
  {HekServoIdWristPitch,  true,   true, DYNA_DIR_CCW, HekTuneOverTorqueThDft},
  {HekServoIdWristRot,    true,   true, DYNA_DIR_CW,  HekTuneOverTorqueThDft},
};
