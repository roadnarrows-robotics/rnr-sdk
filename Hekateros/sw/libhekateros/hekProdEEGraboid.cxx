////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProdEEGraboid.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-02-10 13:39:03 -0700 (Tue, 10 Feb 2015) $
 * $Rev: 3866 $
 *
 * \brief Hekateros 1 DoF Graboid end effector family.
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
#include "Hekateros/hekProdEEGraboid.h"


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
                hekateros::HekProdEEGraboidSpecLinks[HekProdEEGraboidNumLinks] =
{
  // end effector mount point
  { "gear_assembly",
    0.0
  },

  // left finger
  { "finger_L",
    0.0
  },

  // right finger
  { "finger_R",
    406.27
  }
};


//------------------------------------------------------------------------------
//  Graboid End Effector v1.0
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
              hekateros::HekProdEEGraboidSpecJoints_1_0[HekProdEEGraboidDoF] =
{
  { "grip",
    HekServoIdGraboid, DYNA_ID_NONE, HekJointTypeRevMimic, 2.0,
    0.0, 60.0, HekLimitTypePhys,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    5.0, 20.0, 20.0,
    0, 1
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir_to_max toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdEEGraboidSpecServos_1_0[HekProdEEGraboidNumServos] =
{
  // AX-12 version
  {HekServoIdGraboid,   true, false, DYNA_DIR_CW, 60.0}
};


//------------------------------------------------------------------------------
//  Graboid End Effector v1.1
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
              hekateros::HekProdEEGraboidSpecJoints_1_1[HekProdEEGraboidDoF] =
{
  { "grip",
    HekServoIdGraboid, DYNA_ID_NONE, HekJointTypeRevMimic, 2.0,
    0.0, 60.0, HekLimitTypePhys,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    5.0, 20.0, 20.0,
    0, 1
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir_to_max toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdEEGraboidSpecServos_1_1[HekProdEEGraboidNumServos] =
{
  // MX-28 version
  {HekServoIdGraboid,   true, true, DYNA_DIR_CW, HekTuneEEOverTorqueThDft}
};


//------------------------------------------------------------------------------
//  Graboid End Effector v1.2
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
              hekateros::HekProdEEGraboidSpecJoints_1_2[HekProdEEGraboidDoF] =
{
  { "grip",
    HekServoIdGraboid, DYNA_ID_NONE, HekJointTypeRevMimic, 4.0,
    0.0, 60.0, HekLimitTypePhys,
    {
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 },
      { HekIOExpUnassigned, 0.0, 0.0, 0.0, 0.0, 0.0 }
    },
    5.0, 20.0, 20.0,
    0, 1
  }
};

/*!
 * \brief Specification of servos.
 *
 * \par Data:
 * servo_id, is_master, is_continuous, dir_to_max toque_limit(% of max)
 */
const HekSpecServo_T
        hekateros::HekProdEEGraboidSpecServos_1_2[HekProdEEGraboidNumServos] =
{
  // MX-28 version
  {HekServoIdGraboid,   true, true, DYNA_DIR_CW, HekTuneEEOverTorqueThDft}
};
