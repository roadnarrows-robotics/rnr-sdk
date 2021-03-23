////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeTune.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-03-01 16:54:32 -0700 (Tue, 01 Mar 2016) $
 * $Rev: 4333 $
 *
 * \brief Laelaps tuning implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDesc.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/RoboClaw.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"
#include "Laelaps/laeVL6180.h"
#include "Laelaps/laeImu.h"
#include "Laelaps/laeCams.h"

// control and application interface
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"
#include "Laelaps/laeKin.h"
#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadImu.h"
#include "Laelaps/laeThreadKin.h"
#include "Laelaps/laeThreadRange.h"

using namespace std;
using namespace laelaps;


//------------------------------------------------------------------------------
// Laelaps Tuning Defaults, Limits, and Fixed Values
//------------------------------------------------------------------------------

namespace laelaps
{
  // threads
  const double LaeTuneThreadImuHzDft = LaeThreadImu::ThreadImuHzDft;
  const double LaeTuneThreadKinHzDft = LaeThreadKin::ThreadKinHzDft;
  const double LaeTuneThreadRangeHzDft=LaeThreadRange::ThreadRangeHzDft;
  const double LaeTuneThreadHzMin    = LaeThread::ThreadMinHz;

  // watchdog timeout
  const double LaeTuneWdTimeoutDft = (double)LaeWdTimeoutDft / 1000.0;
  const double LaeTuneWdTimeoutMin = (double)LaeWdTimeoutMin / 1000.0;
  const double LaeTuneWdTimeoutMax = (double)LaeWdTimeoutMax / 1000.0;
 
  // velocity limits
  const double LaeTuneVelDerateDft = 100.0;
  const double LaeTuneVelDerateMin = 10.0;
  const double LaeTuneVelDerateMax = 100.0;

  // navigation trajectory
  const LaeNorm LaeTuneTrajNormDft   = LaeNormLinf;
  const double LaeTuneTrajEpsilonDft = 0.001;
  const double LaeTuneTrajEpsilonMin = 0.0;

  // battery
  const char* const LaeTuneBattType      = "lithium-ion polymer";
  const char* const LaeTuneBattChem      = "NMC";
  const double      LaeTuneBattCapAh     = 10.0;
  const int         LaeTuneBattCells     = 3;
  const double      LaeTuneBattMaxVMax   = 12.9;
  const double      LaeTuneBattMaxVDft   = 12.6;
  const double      LaeTuneBattNominalV  = 11.1;
  const double      LaeTuneBattMinVDft   = 9.0;
  const double      LaeTuneBattMinVMin   = 8.1;

  // velocity pid
  const double LaeTuneVelPidKpDft  = 2500.0;
  const double LaeTuneVelPidKiDft  =  500.0;
  const double LaeTuneVelPidKdDft  =  150.0;
  const double LaeTuneVelPidKMin   =    0.0;

  // tires
  const double LaeTuneTireRadiusDft  = 0.60;
  const double LaeTuneTireWidthDft   = 0.65;
  const double LaeTuneTireDimMin     = 0.01;

  // VL6180 Range Sensor
  const int    LaeTuneVL6180TofOffsetDft    = VL6180X_FACTORY_DFT;
  const int    LaeTuneVL6180TofOffsetMin    = VL6180X_RANGE_OFFSET_MIN;
  const int    LaeTuneVL6180TofOffsetMax    = VL6180X_RANGE_OFFSET_MAX;
  const int    LaeTuneVL6180TofXTalkDft     = VL6180X_FACTORY_DFT;
  const int    LaeTuneVL6180TofXTalkMin     = VL6180X_RANGE_XTALK_MIN;
  const int    LaeTuneVL6180TofXTalkMax     = VL6180X_RANGE_XTALK_MAX;
  const double LaeTuneVL6180AlsGainDft      = VL6180X_AMBIENT_GAIN_MIN;
  const double LaeTuneVL6180AlsGainMin      = VL6180X_AMBIENT_GAIN_MIN;
  const double LaeTuneVL6180AlsGainMax      = VL6180X_AMBIENT_GAIN_MAX;
  const int    LaeTuneVL6180AlsIntPeriodDft = VL6180X_AMBIENT_INT_T_REC;
  const int    LaeTuneVL6180AlsIntPeriodMin = VL6180X_AMBIENT_INT_T_MIN;
  const int    LaeTuneVL6180AlsIntPeriodMax = VL6180X_AMBIENT_INT_T_MAX;

} // namespace laelaps



//------------------------------------------------------------------------------
// LaeTunesBattery Class
//------------------------------------------------------------------------------
  
LaeTunesBattery::LaeTunesBattery() :
  m_strType(LaeTuneBattType), m_strChem(LaeTuneBattChem)
{
  m_fCapAh    = LaeTuneBattCapAh;
  m_nCells    = LaeTuneBattCells;
  m_fMaxV     = LaeTuneBattMaxVDft;
  m_fNominalV = LaeTuneBattNominalV;
  m_fMinV     = LaeTuneBattMinVDft;
}

LaeTunesBattery &LaeTunesBattery::operator=(const LaeTunesBattery &rhs)
{
  m_strType   = rhs.m_strType;
  m_strChem   = rhs.m_strChem;
  m_fCapAh    = rhs.m_fCapAh;
  m_nCells    = rhs.m_nCells;
  m_fMaxV     = rhs.m_fMaxV;
  m_fNominalV = rhs.m_fNominalV;
  m_fMinV     = rhs.m_fMinV;

  return *this;
}

void LaeTunesBattery::print(int indent)
{
  printf("%*sBattery =\n", indent, "");
  printf("%*s{\n", indent, "");
  printf("%*sType        = %s\n", indent+2, "", m_strType.c_str());
  printf("%*sChemistry   = %s\n", indent+2, "", m_strChem.c_str());
  printf("%*sCapacity Ah = %.2lf\n", indent+2, "", m_fCapAh);
  printf("%*sCells       = %d\n", indent+2, "", m_nCells);
  printf("%*sMax V       = %.2lf\n", indent+2, "", m_fMaxV);
  printf("%*sNominal V   = %.2lf\n", indent+2, "", m_fNominalV);
  printf("%*sMin V       = %.2lf\n", indent+2, "", m_fMinV);
  printf("%*s}\n", indent, "");
}


//------------------------------------------------------------------------------
// LaeTunesPowertrain Class
//------------------------------------------------------------------------------
  
LaeTunesPowertrain::LaeTunesPowertrain()
{
  m_fVelPidKp   = LaeTuneVelPidKpDft;   // velocity PID proportional constant
  m_fVelPidKi   = LaeTuneVelPidKiDft;   // velocity PID integral constant
  m_fVelPidKd   = LaeTuneVelPidKdDft;   // velocity PID differential constant
  m_fTireRadius = LaeTuneTireRadiusDft; // tire radius (meters)
  m_fTireWidth  = LaeTuneTireWidthDft;  // tire width (meters)
}

LaeTunesPowertrain &LaeTunesPowertrain::operator=(const LaeTunesPowertrain &rhs)
{
  m_fVelPidKp   = rhs.m_fVelPidKp;
  m_fVelPidKi   = rhs.m_fVelPidKi;
  m_fVelPidKd   = rhs.m_fVelPidKd;
  m_fTireRadius = rhs.m_fTireRadius;
  m_fTireWidth  = rhs.m_fTireWidth;

  return *this;
}

void LaeTunesPowertrain::print(const string &strKey, int indent)
{
  printf("%*sPowertrains[%s] =\n", indent, "", strKey.c_str());
  printf("%*s{\n", indent, "");

  printf("%*sVelocity PID =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sKp = %.4lf\n", indent+4, "", m_fVelPidKp);
  printf("%*sKi = %.4lf\n", indent+4, "", m_fVelPidKi);
  printf("%*sKd = %.4lf\n", indent+4, "", m_fVelPidKd);
  printf("%*s}\n", indent+2, "");

  printf("%*sTires =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sRadius = %.3f\n", indent+4, "", m_fTireRadius);
  printf("%*sWidth  = %.3f\n", indent+4, "", m_fTireWidth);
  printf("%*s}\n", indent+2, "");

  printf("%*s}\n", indent, "");
}


//------------------------------------------------------------------------------
// LaeTunesVL6180 Class
//------------------------------------------------------------------------------
  
LaeTunesVL6180::LaeTunesVL6180()
{
  m_nTofOffset    = LaeTuneVL6180TofOffsetDft;    // ToF part-to-part offset
  m_nTofCrossTalk = LaeTuneVL6180TofXTalkDft;     // ToF cross-talk compensation
  m_fAlsGain      = LaeTuneVL6180AlsGainDft;      // ALS analog gain
  m_nAlsIntPeriod = LaeTuneVL6180AlsIntPeriodDft; // ALS integration period
}

LaeTunesVL6180 &LaeTunesVL6180::operator=(const LaeTunesVL6180 &rhs)
{
  m_nTofOffset    = rhs.m_nTofOffset;
  m_nTofCrossTalk = rhs.m_nTofCrossTalk;
  m_fAlsGain      = rhs.m_fAlsGain;
  m_nAlsIntPeriod = rhs.m_nAlsIntPeriod;

  return *this;
}

void LaeTunesVL6180::print(const string &strKey, int indent)
{
  printf("%*sVL6180[%s] =\n", indent, "", strKey.c_str());
  printf("%*s{\n", indent, "");

  printf("%*sToF Offset             = ", indent+2, "");
  if( m_nTofOffset == VL6180X_FACTORY_DFT )
  {
    printf("factory\n");
  }
  else
  {
    printf("%d\n", m_nTofOffset);
  }

  printf("%*sToF Cross-Talk         = ", indent+2, "");
  if( m_nTofCrossTalk == VL6180X_FACTORY_DFT )
  {
    printf("factory\n");
  }
  else
  {
    printf("%d\n", m_nTofCrossTalk);
  }
  printf("%*sALS Gain               = %.1lf\n", indent+2, "", m_fAlsGain);
  printf("%*sALS Integration Period = %d\n", indent+2, "", m_nAlsIntPeriod);

  printf("%*s}\n", indent, "");
}


//------------------------------------------------------------------------------
// LaeTunes Class
//------------------------------------------------------------------------------
  
LaeTunes::LaeTunes()
{
  LaeTunesPowertrain  dftPowertrain;  // powertrain pair tuning defaults
  LaeTunesVL6180      dftVL6180;      // range sensor tuning defaults
  size_t              i;              // working index

  m_fImuHz              = LaeTuneThreadImuHzDft;
  m_fKinematicsHz       = LaeTuneThreadKinHzDft;
  m_fRangeHz            = LaeTuneThreadRangeHzDft;
  m_fWatchDogTimeout    = LaeTuneWdTimeoutDft;
  m_fVelDerate          = LaeTuneVelDerateDft / 100.0;
  m_eTrajNorm           = LaeTuneTrajNormDft;
  m_fTrajEpsilon        = degToRad(LaeTuneTrajEpsilonDft);

  for(i = 0; i < LaeNumMotorCtlrs; ++i)
  {
    m_mapPtp[LaeDesc::KeyMotorCtlr[i]] = dftPowertrain;
  }

  for(i = 0; i < ToFSensorMaxNumOf; ++i)
  {
    m_mapVL6180[LaeDesc::KeyRangeSensorMax[i]] = dftVL6180;
  }
}

double LaeTunes::getImuHz() const
{
  return m_fImuHz;
}

double LaeTunes::getKinematicsHz() const
{
  return m_fKinematicsHz;
}

double LaeTunes::getRangeHz() const
{
  return m_fRangeHz;
}

double LaeTunes::getWatchDogTimeout() const
{
  return m_fWatchDogTimeout;
}

double LaeTunes::getVelocityDerate() const
{
  return m_fVelDerate;
}

void LaeTunes::getTrajectoryParams(LaeNorm &eNorm, double &fEpsilon) const
{
  eNorm     = m_eTrajNorm;
  fEpsilon  = m_fTrajEpsilon;
}

void LaeTunes::getBattOpRangeParams(double &fBattMinV, double &fBattMaxV) const
{
  fBattMinV = m_battery.m_fMinV;
  fBattMaxV = m_battery.m_fMaxV;
}

void LaeTunes::getVelPidKParams(const string &strName,
                                double       &fKp,
                                double       &fKi,
                                double       &fKd) const
{
  string    strKey;
  LaeTunesMapPtp::const_iterator pos;

  strKey = mapToPtp(strName);

  if( (pos = m_mapPtp.find(strKey)) != m_mapPtp.end() )
  {
    fKp = pos->second.m_fVelPidKp;
    fKi = pos->second.m_fVelPidKi;
    fKd = pos->second.m_fVelPidKd;
  }
  else
  {
    LaeTunesPowertrain dft;   // default parameters

    fKp = dft.m_fVelPidKp;
    fKi = dft.m_fVelPidKi;
    fKd = dft.m_fVelPidKd;
  }
}

void LaeTunes::getTireDimParams(const string &strName,
                                double       &fTireRadius,
                                double       &fTireWidth) const
{
  string    strKey;
  LaeTunesMapPtp::const_iterator pos;

  strKey = mapToPtp(strName);

  if( (pos = m_mapPtp.find(strKey)) != m_mapPtp.end() )
  {
    fTireRadius = pos->second.m_fTireRadius;
    fTireWidth  = pos->second.m_fTireWidth;
  }
  else
  {
    LaeTunesPowertrain dft;   // default parameters

    fTireRadius = dft.m_fTireRadius;
    fTireWidth  = dft.m_fTireWidth;
  }
}

string LaeTunes::mapToPtp(const string &strKey) const
{
  if( (strKey == LaeKeyFront) || (strKey == LaeKeyRear) )
  {
    return strKey;
  }
  else if( (strKey == LaeKeyLeftFront) || (strKey == LaeKeyRightFront) )
  {
    return LaeKeyFront;
  }
  else
  {
    return LaeKeyRear;
  }
}

void LaeTunes::getVL6180Params(const std::string &strName,
                               int     &nTofOffset,
                               int     &nTofCrossTalk,
                               double  &fAlsGain,
                               int     &nAlsIntPeriod) const
{
  LaeTunesMapVL6180::const_iterator pos;

  if( (pos = m_mapVL6180.find(strName)) != m_mapVL6180.end() )
  {
    nTofOffset    = pos->second.m_nTofOffset;
    nTofCrossTalk = pos->second.m_nTofCrossTalk;
    fAlsGain      = pos->second.m_fAlsGain;
    nAlsIntPeriod = pos->second.m_nAlsIntPeriod;
  }
  else
  {
    LaeTunesVL6180 dft;     // default parameters

    nTofOffset    = dft.m_nTofOffset;
    nTofCrossTalk = dft.m_nTofCrossTalk;
    fAlsGain      = dft.m_fAlsGain;
    nAlsIntPeriod = dft.m_nAlsIntPeriod;
  }
}

void LaeTunes::print(int indent)
{
  LaeTunesMapPtp::iterator      iPtp;
  LaeTunesMapVL6180::iterator   iSensor;

  printf("%*sGlobals =\n", indent, "");
  printf("%*s{\n", indent, "");

  printf("%*sThreads =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sIMU Thread Hertz        = %.1lf\n", indent+4, "", m_fImuHz);
  printf("%*sKinematics Thread Hertz = %.1lf\n", indent+4, "",
      m_fKinematicsHz);
  printf("%*sRange Thread Hertz      = %.1lf\n", indent+4, "", m_fRangeHz);
  printf("%*s}\n", indent+2, "");

  printf("%*sTrajectory =\n", indent+2, "");
  printf("%*s{\n", indent+2, "");
  printf("%*sNorm    = %d\n", indent+4, "", m_eTrajNorm);
  printf("%*sEpsilon = %lf\n", indent+4, "", radToDeg(m_fTrajEpsilon));
  printf("%*s}\n", indent+2, "");

  printf("%*sVelocity Derate = %.1lf\n", indent+2, "", m_fVelDerate * 100.0);
  printf("%*s}\n", indent, "");

  m_battery.print(indent);

  for(iPtp = m_mapPtp.begin(); iPtp != m_mapPtp.end(); ++iPtp)
  {
    iPtp->second.print(iPtp->first, indent);
  }

  for(iSensor = m_mapVL6180.begin(); iSensor != m_mapVL6180.end(); ++iSensor)
  {
    iSensor->second.print(iSensor->first, indent);
  }
}
