////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeBatt.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-03-02 11:18:30 -0700 (Wed, 02 Mar 2016) $
 * $Rev: 4337 $
 *
 * \brief Laelaps battery management and energy monitoring class implementation.
 *
 * A class instance runs under the control of the WatchDog thread.
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

#include <sys/types.h>
#include <unistd.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeDb.h"
#include "Laelaps/laeBatt.h"


using namespace std;
using namespace laelaps;

// -----------------------------------------------------------------------------
// Private
// -----------------------------------------------------------------------------

//
// Logic Power Constraints
//
// TODO: All non-odroid components need to be measured and verified.
//
static const double OdroidXU3MinAmps    = 1.0;    // from website
static const double OdroidXU3Volts      = 1.8;    // from website
static const double OdroidXU4MinAmps    = 1.0;    // from xu3 website 
static const double OdroidXU4Volts      = 1.8;    // from website
static const double RoboClaw2x30AAmps   = 0.03;   // from manual
static const double RoboClaw2x30AVolts  = 5.0;    // could be 3.3V
static const double Naze32Amps          = 0.50;   // usb limit ?
static const double Naxe32Volts         = 5.0;    // could be 3.3V
static const double BattChargerAmps     = 0.0;    // ?
static const double BattChargerVolts    = 0.0;    // ?
static const double ArduinoAmps         = 0.01;   // ?
static const double ArduinoVolts        = 5.0;    // ?
static const double OtherAmps           = 0.01;   // ?
static const double OtherVolts          = 5.0;    // ?


/*!
 * \brief Lithium-Ion State of Charge - Cell Voltage table.
 *
 * Tables associating the State of Charge for a single cell and sensed voltage
 * V for the given specific discharge current C.
 *
 * C is the normalized value: BatterCapacity(Ah)/discharge(A). For example,
 * for a 10Ah battery:
 *  0.5C = 5A
 *    1C = 10A
 *    2C = 20A
 *
 * Sources:
 * \li http://web.mit.edu/evt/summary_battery_specifications.pdf
 * \li http://www.electricrcaircraftguy.com/2013_01_01_archive.html
 * \li http://epg.eng.ox.ac.uk/content/electric-vehicles-using-physics-based-battery-models-improved-estimation-state-charge
 */

/*!
 * \brief State of Charge - Battery Voltage entry.
 */
struct LiIonCellSoCVEntry
{
  double m_fSoC;    ///< state of charge (%)
  double m_fV;      ///< sensed battery voltage (V)
};

/*!
 * \brief Specific Current - State of Charge/Battery Voltage table entry.
 */
struct LiIonCellCSoCEntry
{
  double              m_fC;       ///< specific normalized current
  LiIonCellSoCVEntry *m_pTbl;     ///< SoC - V table
  size_t              m_sizeTbl;  ///< number of entries
};


/*!
 * \brief SoC-V table for an 0C unloaded single cell.
 */
static LiIonCellSoCVEntry LiIonCellSoCVTbl_0C[] =
{
  {  0.0, 3.00}, { 10.0, 3.62}, { 20.0, 3.70}, { 30.0, 3.74}, { 40.0, 3.78},
  { 50.0, 3.84}, { 60.0, 3.88}, { 70.0, 3.93}, { 80.0, 3.97}, { 90.0, 4.05},
  {100.0, 4.20}
};

/*!
 * \brief SoC-V table for an 1C single cell.
 */
static LiIonCellSoCVEntry LiIonCellSoCVTbl_1C[] =
{
  {  0.0, 2.00}, { 10.0, 3.50}, { 20.0, 3.55}, { 30.0, 3.60}, { 40.0, 3.65},
  { 50.0, 3.70}, { 60.0, 3.75}, { 70.0, 3.78}, { 80.0, 3.80}, { 90.0, 3.90},
  {100.0, 4.05}
};

/*!
 * \brief SoC-V table for an 2C single cell.
 */
static LiIonCellSoCVEntry LiIonCellSoCVTbl_2C[] =
{
  {  0.0, 1.00}, { 10.0, 2.90}, { 20.0, 3.45}, { 30.0, 3.55}, { 40.0, 3.60},
  { 50.0, 3.65}, { 60.0, 3.70}, { 70.0, 3.75}, { 80.0, 3.76}, { 90.0, 3.80},
  {100.0, 4.00}
};

/*!
 * \brief SoC-V table for an 3C single cell.
 */
static LiIonCellSoCVEntry LiIonCellSoCVTbl_3C[] =
{
  {  0.0, 0.10}, { 10.0, 0.50}, { 20.0, 3.30}, { 30.0, 3.45}, { 40.0, 3.51},
  { 50.0, 3.60}, { 60.0, 3.65}, { 70.0, 3.70}, { 80.0, 3.74}, { 90.0, 3.78},
  {100.0, 4.00}
};

/*!
 * \brief C - SoC-V table of tables.
 */
static LiIonCellCSoCEntry LiIonCellSoCTbls[] = 
{
  {0.0, LiIonCellSoCVTbl_0C, arraysize(LiIonCellSoCVTbl_0C)},
  {1.0, LiIonCellSoCVTbl_1C, arraysize(LiIonCellSoCVTbl_1C)},
  {2.0, LiIonCellSoCVTbl_2C, arraysize(LiIonCellSoCVTbl_2C)},
  {3.0, LiIonCellSoCVTbl_3C, arraysize(LiIonCellSoCVTbl_3C)}
};

/*!
 * \brief Linear interpolate.
 *
 * \param x   X input in [x0, x1].
 * \param x0  Lower domain value.
 * \param x1  Upper domain value.
 * \param y0  Lower range value.
 * \param y1  Upper range value.
 *
 * \return Interpolated Y output in [y0, y1].
 */
static double linearInterp(double x, double x0, double x1, double y0, double y1)
{
  return y0 + (x - x0)/(x1 - x0) * (y1 - y0);
}

/*!
 * \brief Lookup State of Charge, given the battery voltage.
 *
 * Linear interpolation is performed as necessary.
 *
 * \param fV    Input battery voltage.
 * \param tbl   SoC - V table.
 * \param n     Number of table entries.
 *
 * \return Interpolated SoC.
 */
static double lookupSoC(double fV, LiIonCellSoCVEntry tbl[], size_t n)
{
  size_t  i;

  // table list voltage for single cell
  fV /= (double)LaeTuneBattCells; 

  // search accending table for nearest entries
  for(i = 0; i < n; ++i)
  {
    if( tbl[i].m_fV >= fV )
    {
      break;
    }
  }

  // lower boundary case
  if( i == 0 )
  {
    return tbl[i].m_fSoC;
  }
  // upper boundary case
  else if( i == n )
  {
    return tbl[i-1].m_fSoC;
  }
  // interpolate
  else
  {
    return linearInterp(fV, tbl[i-1].m_fV,    tbl[i].m_fV,
                            tbl[i-1].m_fSoC,  tbl[i].m_fSoC);
  }
}


// -----------------------------------------------------------------------------
// LaeBattery Class
// -----------------------------------------------------------------------------

LaeBattery::LaeBattery()
{
  m_bIsCharging       = false;
  m_fBatteryVoltage   = 0.0;
  m_fBatterySoC       = 0.0;
  m_fMotorAmps        = 0.0;
  m_fMotorWatts       = 0.0;
  m_fLogicWatts       = 0.0;
  m_fTotalAmps        = 0.0;
  m_fTotalWatts       = 0.0;
}

LaeBattery::~LaeBattery()
{
}

void LaeBattery::calcMotorEnergyState()
{
  int     nCtlr, nMotor;
  double  fVolts, fAmps, fWatts;

  m_fBatteryVoltage = 0.0;
  m_fMotorAmps      = 0.0;
  m_fMotorWatts     = 0.0;

  //
  // Batteries and motors
  //
  for(nCtlr=0; nCtlr<LaeNumMotorCtlrs; ++nCtlr)
  {
    fVolts = RtDb.m_motorctlr[nCtlr].m_fBatteryVoltage;
    for(nMotor=0; nMotor<LaeNumMotorsPerCtlr; ++nMotor)
    {
      fAmps   = RtDb.m_motorctlr[nCtlr].m_fMotorCurrent[nMotor];
      fWatts  = fAmps * fVolts;

      m_fMotorAmps  += fAmps;
      m_fMotorWatts += fWatts;
    }
    m_fBatteryVoltage += fVolts;
  }

  //
  // For original 2.0 hardware, the only sources to sense battery voltages were
  // from the motor controllers. New in v2.1+, there is a direct voltage sense
  // to the WatchDog processor. This change was needed because the motor
  // controllers' power enable line can be disabled independent of battery
  // state.
  // 
  if( RtDb.m_product.m_uProdHwVer >= LAE_VERSION(2, 1, 0) )
  {
    m_fBatteryVoltage = RtDb.m_energy.m_fBatteryVoltage;
  }
  else
  {
    // average
    m_fBatteryVoltage /= (double)LaeNumMotorCtlrs;
  }
}

void LaeBattery::calcLogicEnergyState()
{
  //
  // TODO See https://github.com/hardkernel/EnergyMonitor
  //
  m_fLogicAmps  = 0.5;
  m_fLogicWatts = m_fLogicAmps * 1.8;
}

double LaeBattery::estimateBatteryStateOfCharge()
{
  double  fC;
  size_t  n, i;
  double  fSoC0, fSoC1;

  // normalize specific current
  fC = m_fTotalAmps / LaeTuneBattCapAh;

  n = arraysize(LiIonCellSoCTbls);

  // look for C's nearest entries
  for(i = 0; i < n; ++i)
  {
    if( LiIonCellSoCTbls[i].m_fC >= fC )
    {
      break;
    }
  }

  // lower boundary case - single interpolation
  if( i == 0 )
  {
    m_fBatterySoC = lookupSoC(m_fBatteryVoltage, LiIonCellSoCTbls[i].m_pTbl,
                                        LiIonCellSoCTbls[i].m_sizeTbl);
  }
  // upper boundary case - single interpolation
  else if( i == n )
  {
    m_fBatterySoC = lookupSoC(m_fBatteryVoltage, LiIonCellSoCTbls[i-1].m_pTbl,
                                        LiIonCellSoCTbls[i-1].m_sizeTbl);
  }
  // double interpolation
  else
  {
    fSoC0 = lookupSoC(m_fBatteryVoltage, LiIonCellSoCTbls[i-1].m_pTbl,
                                        LiIonCellSoCTbls[i-1].m_sizeTbl);
    fSoC1 = lookupSoC(m_fBatteryVoltage, LiIonCellSoCTbls[i].m_pTbl,
                                        LiIonCellSoCTbls[i].m_sizeTbl);
    m_fBatterySoC = linearInterp(fC,
                          LiIonCellSoCTbls[i-1].m_fC, LiIonCellSoCTbls[i].m_fC,
                          fSoC0, fSoC1);
  }

  return m_fBatterySoC;
}

void LaeBattery::update()
{
  m_bIsCharging     = RtDb.m_energy.m_bBatteryIsCharging;

  // calculate the amps, watts, etc. used by all motors
  calcMotorEnergyState();
  
  // calculate the amps, watts, etc. used by all logic circuitry
  calcLogicEnergyState();
  
  //
  // Totals
  //
  m_fTotalAmps  = m_fMotorAmps + m_fLogicAmps;
  m_fTotalWatts = m_fMotorWatts + m_fLogicWatts;

  //
  // Estimate battery state of charge.
  //
  estimateBatteryStateOfCharge();

  //
  // Export key data to database
  //
  if( RtDb.m_product.m_uProdHwVer < LAE_VERSION(2, 1, 0) )
  {
    // producer
    RtDb.m_energy.m_fBatteryVoltage = m_fBatteryVoltage;
  }
  RtDb.m_energy.m_fBatterySoC     = m_fBatterySoC;
  RtDb.m_energy.m_fTotalCurrent   = m_fTotalAmps;
  RtDb.m_energy.m_fTotalPower     = m_fTotalWatts;

  //fprintf(stderr, "DBG: battV=%lf, battSoC=%lf totCurr=%lf, totPwr=%lf\n",
  //  RtDb.m_energy.m_fBatteryVoltage, RtDb.m_energy.m_fBatterySoC,
  //  RtDb.m_energy.m_fTotalCurrent, RtDb.m_energy.m_fTotalPower);
}
