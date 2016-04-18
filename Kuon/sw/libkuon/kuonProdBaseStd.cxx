////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonProdBaseStd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon standard size robotic mobile platform base static specification.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014.  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Kuon/RS160DControl.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonProdBaseStd.h"

using namespace std;
using namespace kuon;


/*!
 * \brief Default specification of motors, wheels, and tires.
 *
 * \par Data:
 * name, motor_id, motor_ctlr_id, motor_index, gear_ratio, tire_radius(mm), dir
 */
const KuonSpecMotor_T kuon::KuonProdBaseStdSpecMotors[KuonProdBaseStdNumMotors]=
{
  { "left_front",
      KuonMotorIdLF, KuonMotorCtlrId0, RS160D_MOTOR_LEFT_ID,
      1.0,  200.0,  KuonMotorDirCcw
  },
  { "right_front",
      KuonMotorIdRF, KuonMotorCtlrId0, RS160D_MOTOR_RIGHT_ID,
      1.0,  200.0,  KuonMotorDirCw
  },
  { "left_rear",
      KuonMotorIdLR, KuonMotorCtlrId1, RS160D_MOTOR_LEFT_ID,
      1.0,  200.0,  KuonMotorDirCcw
  },
  { "right_rear",
      KuonMotorIdRR, KuonMotorCtlrId1, RS160D_MOTOR_RIGHT_ID,
      1.0,  200.0,  KuonMotorDirCw
  },
};
