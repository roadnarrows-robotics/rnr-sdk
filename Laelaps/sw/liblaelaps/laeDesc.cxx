////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeDesc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-15 15:44:49 -0700 (Mon, 15 Feb 2016) $
 * $Rev: 4320 $
 *
 * \brief Laelaps robotic base mobile platform description class implementation.
 *
 * The base description does not include any payload descriptions.
 * Any applicable tuning parameters override the description.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2015-2016.  RoadNarrows LLC.
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

#include <stdio.h>
#include <math.h>

#include <string>
#include <sstream>
#include <iomanip>
#include <locale>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeDb.h"

using namespace std;
using namespace laelaps;

// -----------------------------------------------------------------------------
// Fixed Data
// -----------------------------------------------------------------------------

//
// Dimensions.
//

/*!
 * \brief Laelaps body dimensions (W x H x L).
 */
static const Dim LaeDimBody(0.250, 0.080, 0.350);

/*!
 * \brief Front bumper dimensions (W x H x L).
 */
static const Dim LaeDimFrontBumper(0.025, 0.038, 0.190);

/*!
 * \brief Center of wheel shaft offset from body (dW x dH x dL).
 */
static const Dim LaeDimWheelShaftOffset(0.005, -0.015, -0.015);


// -----------------------------------------------------------------------------
// Class LaeDescBase
// -----------------------------------------------------------------------------

LaeDescBase::LaeDescBase() :
  m_strKey(LaeDesc::KeyRobotBase),
  m_dimBody(LaeDimBody)
{
  calcDimensions(LaeTuneTireRadiusDft, LaeTuneTireWidthDft);

  m_nNumMotorCtlrs    = LaeNumMotorCtlrs;
  m_nNumMotors        = LaeMotorsNumOf;
}

LaeDescBase::~LaeDescBase()
{
}

LaeDescBase LaeDescBase::operator=(const LaeDescBase &rhs)
{
  m_strKey            = rhs.m_strKey;
  m_dimRobot          = rhs.m_dimRobot;
  m_dimBody           = rhs.m_dimBody;
  m_fWheelbase        = rhs.m_fWheelbase;
  m_fWheeltrack       = rhs.m_fWheeltrack;
  m_nNumMotorCtlrs    = rhs.m_nNumMotorCtlrs;
  m_nNumMotors        = rhs.m_nNumMotors;

  return *this;
}

void LaeDescBase::clear()
{
  m_strKey.clear();
  m_dimRobot.clear();
  m_dimBody.clear();
  m_fWheelbase        = 0.0;
  m_fWheeltrack       = 0.0;
  m_nNumMotorCtlrs    = 0;
  m_nNumMotors        = 0;
}

void LaeDescBase::calcDimensions(double fTireRadius, double fTireWidth)
{
  //
  // Robot outer dimensions.
  //
  m_dimRobot.m_width  = m_dimBody.m_width +
                          2.0 * LaeDimWheelShaftOffset.m_width +
                          2.0 * fTireWidth;

  m_dimRobot.m_height = m_dimBody.m_height +
                          LaeDimWheelShaftOffset.m_height +
                          fTireRadius;

  m_dimRobot.m_length = m_dimBody.m_length + 
                          LaeDimFrontBumper.m_width;

  //
  // Wheelbase is measured from the centers of rotation of the front and rear
  // axes.
  //
  m_fWheelbase        = m_dimBody.m_length +
                          2.0 * LaeDimWheelShaftOffset.m_length;

  //
  // Wheeltrack is measured from the centers of width between the left and right
  // tires.
  //
  m_fWheeltrack       = m_dimBody.m_width +
                          2.0 * LaeDimWheelShaftOffset.m_width +
                          fTireWidth;
}

void LaeDescBase::print(int indent)
{
  printf("%*sBase Description =\n", indent, "");
  printf("%*s{\n", indent, "");

  printf("%*sKey = %s\n", indent+2, "", m_strKey.c_str());

  printf("%*sOuter Dimensions =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sWidth  = %.3lf\n", indent+4, "", m_dimRobot.m_width);
  printf("%*sHeight = %.3lf\n", indent+4, "", m_dimRobot.m_height);
  printf("%*sLength = %.3lf\n", indent+4, "", m_dimRobot.m_length);
  printf("%*s}\n", indent+2, "");

  printf("%*sBody Dimensions =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sWidth  = %.3lf\n", indent+4, "", m_dimBody.m_width);
  printf("%*sHeight = %.3lf\n", indent+4, "", m_dimBody.m_height);
  printf("%*sLength = %.3lf\n", indent+4, "", m_dimBody.m_length);
  printf("%*s}\n", indent+2, "");

  printf("%*sWheelbase              = %.3lf\n", indent+2, "", m_fWheelbase);
  printf("%*sWheeltrack             = %.3lf\n", indent+2, "", m_fWheeltrack);
  printf("%*sNumber of Motors Ctlrs = %d\n", indent+2, "", m_nNumMotorCtlrs);
  printf("%*sNumber of Motors       = %d\n", indent+2, "", m_nNumMotors);

  printf("%*s}\n", indent, "");
}


// -----------------------------------------------------------------------------
// Class LaeDescPowertrain
// -----------------------------------------------------------------------------
 
LaeDescPowertrain::LaeDescPowertrain()
{
  clear();
}

LaeDescPowertrain::LaeDescPowertrain(const string &strKey)
{
  m_eJointType  = LaeJointTypeContinuous;
  m_eEncType    = LaeEncTypeQuadrature;
  m_fGearRatio  = LaeMotorGearRatio;
  m_nDir        = LaeMotorDirNormal;

  if( strKey ==  LaeKeyLeftFront )
  {
    m_nMotorId      = LaeMotorIdLF;
    m_nMotorCtlrId  = LaeMotorCtlrIdFront;
    m_nMotorIndex   = LaeMotorLeft;
  }
  else if( strKey == LaeKeyRightFront )
  {
    m_nMotorId      = LaeMotorIdRF;
    m_nMotorCtlrId  = LaeMotorCtlrIdFront;
    m_nMotorIndex   = LaeMotorRight;
  }
  else if( strKey == LaeKeyLeftRear )
  {
    m_nMotorId      = LaeMotorIdLR;
    m_nMotorCtlrId  = LaeMotorCtlrIdRear;
    m_nMotorIndex   = LaeMotorLeft;
  }
  else if( strKey == LaeKeyRightRear )
  {
    m_nMotorId      = LaeMotorIdRR;
    m_nMotorCtlrId  = LaeMotorCtlrIdRear;
    m_nMotorIndex   = LaeMotorRight;
  }
  else
  {
    LOGWARN("Powertrain %s: Unknown.", strKey.c_str());
    clear();
  }

  m_strKey = strKey;
}

LaeDescPowertrain::~LaeDescPowertrain()
{
}

LaeDescPowertrain LaeDescPowertrain::operator=(const LaeDescPowertrain &rhs)
{
  m_strKey        = rhs.m_strKey;
  m_nMotorId      = rhs.m_nMotorId;
  m_nMotorCtlrId  = rhs.m_nMotorCtlrId;
  m_nMotorIndex   = rhs.m_nMotorIndex;
  m_eJointType    = rhs.m_eJointType;
  m_eEncType      = rhs.m_eEncType;
  m_fGearRatio    = rhs.m_fGearRatio;
  m_nDir          = rhs.m_nDir;

  return *this;
}
 
void LaeDescPowertrain::clear()
{
  m_strKey.clear();
  m_nMotorId      = LaeMotorIdNone;
  m_nMotorCtlrId  = LaeMotorCtlrIdNone;
  m_nMotorIndex   = LaeMotorLeft;
  m_eJointType    = LaeJointTypeContinuous;
  m_eEncType      = LaeEncTypeUnknown;
  m_fGearRatio    = 1.0;
  m_nDir          = LaeMotorDirNormal;
}

void LaeDescPowertrain::print(int indent)
{
  printf("%*sPowertrain[%s] Description =\n", indent, "", m_strKey.c_str());
  printf("%*s{\n", indent, "");
  printf("%*sKey           = %s\n", indent+2, "", m_strKey.c_str());
  printf("%*sMotor Id      = %d\n", indent+2, "", m_nMotorId);
  printf("%*sMotor Ctlr Id = %d\n", indent+2, "", m_nMotorCtlrId);
  printf("%*sMotor Index   = %d\n", indent+2, "", m_nMotorIndex);
  printf("%*sJoint Type    = %d\n", indent+2, "", m_eJointType);
  printf("%*sEncoder Type  = 0x%02x\n", indent+2, "", m_eEncType);
  printf("%*sDirection     = %d\n", indent+2, "", m_nDir);
  printf("%*s}\n", indent, "");
}


// -----------------------------------------------------------------------------
// Class LaeDescBattery
// -----------------------------------------------------------------------------

LaeDescBattery::LaeDescBattery() :
    m_strKey(LaeDesc::KeyBattery),
    m_strType(LaeTuneBattType),
    m_strChemistry(LaeTuneBattChem)
{
  m_nNumCells = LaeTuneBattCells;
  m_fCapacity = LaeTuneBattCapAh;
  m_fMaxV     = LaeTuneBattMaxVDft;
  m_fMinV     = LaeTuneBattMinVDft;
  m_fNomV     = LaeTuneBattNominalV;
}

LaeDescBattery::~LaeDescBattery()
{
}

LaeDescBattery LaeDescBattery::operator=(const LaeDescBattery &rhs)
{
  m_strKey        = rhs.m_strKey;
  m_strType       = rhs.m_strType;
  m_strChemistry  = rhs.m_strChemistry;
  m_nNumCells     = rhs.m_nNumCells;
  m_fCapacity     = rhs.m_fCapacity;
  m_fMaxV         = rhs.m_fMaxV;
  m_fMinV         = rhs.m_fMinV;
  m_fNomV         = rhs.m_fNomV;

  return *this;
}

void LaeDescBattery::clear()
{
  m_strKey.clear();
  m_strType.clear();
  m_strChemistry.clear();
  m_nNumCells    = 0;
  m_fCapacity    = 0.0;
  m_fMaxV        = 0.0;
  m_fMinV        = 0.0;
  m_fNomV        = 0.0;
}

void LaeDescBattery::print(int indent)
{
  printf("%*sBattery Description =\n", indent, "");
  printf("%*s{\n", indent, "");
  printf("%*sKey         = %s\n", indent+2, "", m_strKey.c_str());
  printf("%*sType        = %s\n", indent+2, "", m_strType.c_str());
  printf("%*sChemistry   = %s\n", indent+2, "", m_strChemistry.c_str());
  printf("%*sCapacity Ah = %.2lf\n", indent+2, "", m_fCapacity);
  printf("%*sCells       = %d\n", indent+2, "", m_nNumCells);
  printf("%*sMax V       = %.2lf\n", indent+2, "", m_fMaxV);
  printf("%*sNominal V   = %.2lf\n", indent+2, "", m_fNomV);
  printf("%*sMin V       = %.2lf\n", indent+2, "", m_fMinV);
  printf("%*s}\n", indent, "");
}


// -----------------------------------------------------------------------------
// Class LaeDescRangeSensor
// -----------------------------------------------------------------------------
 
LaeDescRangeSensor::LaeDescRangeSensor()
{
  clear();
}

LaeDescRangeSensor::LaeDescRangeSensor(const string &strKey)
{
  if( strKey ==  "front" )
  {
    m_nChan     = ToFSensor0Chan;
    m_fDir      = ToFSensor0Dir;
    m_fDeadzone = ToFSensor0Deadzone;
  }
  else if( strKey == "left_front" )
  {
    m_nChan     = ToFSensor1Chan;
    m_fDir      = ToFSensor1Dir;
    m_fDeadzone = ToFSensor1Deadzone;
  }
  else if( strKey == "left" )
  {
    m_nChan     = ToFSensor2Chan;
    m_fDir      = ToFSensor2Dir;
    m_fDeadzone = ToFSensor2Deadzone;
  }
  else if( strKey == "left_rear" )
  {
    m_nChan     = ToFSensor3Chan;
    m_fDir      = ToFSensor3Dir;
    m_fDeadzone = ToFSensor3Deadzone;
  }
  else if( strKey == "rear" )
  {
    m_nChan     = ToFSensor4Chan;
    m_fDir      = ToFSensor4Dir;
    m_fDeadzone = ToFSensor4Deadzone;
  }
  else if( strKey == "right_rear" )
  {
    m_nChan     = ToFSensor5Chan;
    m_fDir      = ToFSensor5Dir;
    m_fDeadzone = ToFSensor5Deadzone;
  }
  else if( strKey == "right" )
  {
    m_nChan     = ToFSensor6Chan;
    m_fDir      = ToFSensor6Dir;
    m_fDeadzone = ToFSensor6Deadzone;
  }
  else if( strKey == "right_front" )
  {
    m_nChan     = ToFSensor7Chan;
    m_fDir      = ToFSensor7Dir;
    m_fDeadzone = ToFSensor7Deadzone;
  }
  else
  {
    LOGWARN("Range Sensor %s: Unknown.", strKey.c_str());
    clear();
  }

  makeDesc();

  m_strKey = strKey;
}

LaeDescRangeSensor::~LaeDescRangeSensor()
{
}

LaeDescRangeSensor LaeDescRangeSensor::operator=(const LaeDescRangeSensor &rhs)
{
  m_strKey    = rhs.m_strKey;
  m_nChan     = rhs.m_nChan;
  m_fDir      = rhs.m_fDir;
  m_fDeadzone = rhs.m_fDeadzone;

  return *this;
}
 
void LaeDescRangeSensor::clear()
{
  m_strKey.clear();
  m_nChan     = -1;
  m_fDir      = 0.0;
  m_fDeadzone = 0.0;
}

void LaeDescRangeSensor::print(int indent)
{
  printf("%*sRange Sensor[%s] Description =\n", indent, "", m_strKey.c_str());
  printf("%*s{\n", indent, "");
  printf("%*sKey            = %s\n", indent+2, "", m_strKey.c_str());
  printf("%*sChannel Number = %d\n", indent+2, "", m_nChan);
  printf("%*sDirection      = %.1lf\n", indent+2, "", radToDeg(m_fDir));
  printf("%*sDeadZone       = %.3lf\n", indent+2, "", m_fDeadzone);
  printf("%*s}\n", indent, "");
}

void LaeDescRangeSensor::makeDesc()
{
  stringstream  ss;

  if( m_nChan == -1 )
  {
    m_strDesc = "Unknown sensor";
    return;
  }

  ss  << "Range sensor at "
      << m_strKey << " "
      << radToDeg(m_fDir) << " degrees";

  m_strDesc = ss.str();
}


// -----------------------------------------------------------------------------
// Class LaeDescImu
// -----------------------------------------------------------------------------

LaeDescImu::LaeDescImu() :
  m_strKey(LaeDesc::KeyImu),
  m_strHw("Naze32"),
  m_strFw("CleanFlight")
{
}

LaeDescImu::~LaeDescImu()
{
}

LaeDescImu LaeDescImu::operator=(const LaeDescImu &rhs)
{
  m_strKey  = rhs.m_strKey;
  m_strHw   = rhs.m_strHw;
  m_strFw   = rhs.m_strFw;

  return *this;
}

void LaeDescImu::clear()
{
  m_strKey.clear();
  m_strHw.clear();
  m_strFw.clear();
}

void LaeDescImu::print(int indent)
{
  printf("%*sIMU Description =\n", indent, "");
  printf("%*s{\n", indent, "");
  printf("%*sKey      = %s\n", indent+2, "", m_strKey.c_str());
  printf("%*sHardware = %s\n", indent+2, "", m_strHw.c_str());
  printf("%*sFirmware = %s\n", indent+2, "", m_strFw.c_str());
  printf("%*s}\n", indent, "");
}


// -----------------------------------------------------------------------------
// Class LaeDesc
// -----------------------------------------------------------------------------

const char* const LaeDesc::KeyRobotBase = "base";

const char* const LaeDesc::KeyBattery = "battery";

const char* const LaeDesc::KeyMotorCtlr[] =
{
  LaeKeyFront, LaeKeyRear
};

const char* const LaeDesc::KeyPowertrain[] =
{
  LaeKeyLeftFront, LaeKeyRightFront,
  LaeKeyLeftRear,  LaeKeyRightRear
};

const char* const LaeDesc::KeyImu = "imu";

const char* const LaeDesc::KeyRangeSensor[] =
{
  "front", "left_front", "left", "left_rear",
  "rear", "right_rear", "right", "right_front"
};

const char* const LaeDesc::KeyRangeSensorStd[] =
{
  "front", "left_front", "right_front"
};

const char* const LaeDesc::KeyFCam = "fcam";

std::string LaeDesc::prettyMotorCtlrName(int nCtlrId)
{
  std::locale       loc;
  std::stringstream ss;
  std::string       str;

  ss << LaeDesc::KeyMotorCtlr[nCtlrId] << " motor controller";
  str = ss.str();

  str[0] = std::toupper(str[0], loc);

  return str;
}

std::string LaeDesc::prettyMotorCtlrName(int nCtlrId, byte_t addr)
{
  std::locale       loc;
  std::stringstream ss;
  std::string       str;

  ss << LaeDesc::KeyMotorCtlr[nCtlrId] << " motor controller (0x"
      << std::hex << std::setfill('0') << std::setw(2) << (unsigned)addr
      << ")";
  str = ss.str();

  str[0] = std::toupper(str[0], loc);

  return str;
}

std::string LaeDesc::prettyMotorName(int nMotorId)
{
  std::locale       loc;
  std::stringstream ss;
  std::string       str;
  size_t            n;

  ss << LaeDesc::KeyPowertrain[nMotorId] << " motor (id=" << nMotorId << ")";
  str = ss.str();

  str[0] = std::toupper(str[0], loc);
  if( (n = str.find_first_of('_')) != str.npos )
  {
    str.replace(n, 1, " ");
  }

  return str;
}

std::string LaeDesc::prettyMotorName(int nCtlrId, byte_t addr, int nMotorId)
{
  std::stringstream ss;

  ss << LaeDesc::prettyMotorCtlrName(nCtlrId, addr) << ": "
     << LaeDesc::prettyMotorName(nMotorId);

  return ss.str();
}

LaeDesc::LaeDesc() :
  m_strProdFamily(LaeProdFamily)
{
  m_bIsDescribed  = false;
  m_eProdId       = LaeProdIdUnknown;
  m_uProdHwVer    = 0;

  m_pDescBase     = NULL;
  m_pDescBattery  = NULL;
  m_pDescImu      = NULL;
}

LaeDesc::~LaeDesc()
{
  clear();
}

int LaeDesc::markAsDescribed()
{
  const char *sKey;
  int         i;
  int         rc;

  setVersion();

  switch( m_eProdId )
  {
    case LaeProdIdStd:
    case LaeProdIdLarge:
      if( m_uProdHwVer < LAE_VERSION(2, 2, 0) )
      {
        m_pDescBase = new LaeDescBase();
        m_pDescBattery = new LaeDescBattery();
        for(i = 0; i < LaeMotorsNumOf; ++i)
        {
          sKey = KeyPowertrain[i];
          m_mapDescPowertrain[sKey] = new LaeDescPowertrain(sKey);
        }
        for(i = 0; i < ToFSensorStdNumOf; ++i)
        {
          sKey = KeyRangeSensorStd[i];
          m_mapDescRangeSensor[sKey] = new LaeDescRangeSensor(sKey);
        }
        m_pDescImu = new LaeDescImu();

        RtDb.m_product.m_eProdId    = m_eProdId;
        RtDb.m_product.m_uProdHwVer = m_uProdHwVer;

        rc = LAE_OK;
      }
      else
      {
        LOGERROR("Version %s: Laelaps robot version unsupported.",
            m_strProdHwVer.c_str());
        rc = -LAE_ECODE_BAD_OP;
      }
      break;

    case LaeProdIdUnknown:
      LOGERROR("Laelaps robot description is undefined.");
      rc = -LAE_ECODE_BAD_OP;
      break;
    default:
      LOGERROR("ProdId %d: Unknown Laelaps robot product id.", m_eProdId);
      rc = -LAE_ECODE_BAD_OP;
      break;
  }

  if( rc == LAE_OK )
  {
    m_bIsDescribed = true;
  }

  return rc;
}

void LaeDesc::clear()
{
  m_bIsDescribed  = false;
  m_eProdId       = LaeProdIdUnknown;
  m_strProdModel.clear();
  m_strProdName.clear();
  m_strProdBrief.clear();
  m_strProdHwVer.clear();
  m_uProdHwVer    = 0;

  if( m_pDescBase != NULL )
  {
    delete m_pDescBase;
    m_pDescBase = NULL;
  }

  if( m_pDescBattery != NULL )
  {
    delete m_pDescBattery;
    m_pDescBattery = NULL;
  }

  for(MapDescPowertrain::iterator iter = m_mapDescPowertrain.begin();
      iter != m_mapDescPowertrain.end();
      ++iter)
  {
    delete iter->second;
  }
  m_mapDescPowertrain.clear();

  for(MapDescRangeSensor::iterator iter = m_mapDescRangeSensor.begin();
      iter != m_mapDescRangeSensor.end();
      ++iter)
  {
    delete iter->second;
  }
  m_mapDescRangeSensor.clear();

  if( m_pDescImu != NULL )
  {
    delete m_pDescImu;
    m_pDescImu = NULL;
  }
}

const char *LaeDesc::getProdName(int eProdId)
{
  switch( eProdId )
  {
    case LaeProdIdStd:
      return "Laelaps-Standard";
    case LaeProdIdLarge:
      return "Laelaps-Large";
    default:
      return "";
  }
}

const char *LaeDesc::getProdBrief(int eProdId)
{
  switch( eProdId )
  {
    case LaeProdIdStd:
      return "RoadNarrows Laelaps Standard robotic mobile platform";
    case LaeProdIdLarge:
      return "RoadNarrows Laelaps Large robotic mobile platform";
    default:
      return "";
  }
}

void LaeDesc::print(int indent)
{
  printf("%*sRobot Description =\n", indent, "");
  printf("%*s{\n", indent, "");

  printf("%*sProduct Id         = %d\n", indent+2, "", m_eProdId);
  printf("%*sProduct Family     = %s\n", indent+2, "", m_strProdFamily.c_str());
  printf("%*sProduct Model      = %s\n", indent+2, "", m_strProdModel.c_str());
  printf("%*sProduct Full Name  = %s\n", indent+2, "", m_strProdName.c_str());
  printf("%*sProduct Brief      = %s\n", indent+2, "", m_strProdBrief.c_str());
  printf("%*sHardware Version   = %s\n", indent+2, "", m_strProdHwVer.c_str());
  printf("%*sHardware Version   = 0x%08x\n", indent+2, "", m_uProdHwVer);

  if( m_pDescBase != NULL )
  {
    m_pDescBase->print(indent+2);
  }

  if( m_pDescBattery != NULL )
  {
    m_pDescBattery->print(indent+2);
  }

  for(MapDescPowertrain::iterator iter = m_mapDescPowertrain.begin();
      iter != m_mapDescPowertrain.end();
      ++iter)
  {
    iter->second->print(indent+2);
  }

  for(MapDescRangeSensor::iterator iter = m_mapDescRangeSensor.begin();
      iter != m_mapDescRangeSensor.end();
      ++iter)
  {
    iter->second->print(indent+2);
  }

  if( m_pDescImu != NULL )
  {
    m_pDescImu->print(indent+2);
  }

  printf("%*s}\n", indent, "");
}

void LaeDesc::setVersion()
{
  m_uProdHwVer = strToVersion(m_strProdHwVer);
}
