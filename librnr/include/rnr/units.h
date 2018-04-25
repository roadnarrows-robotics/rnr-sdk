////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Scientific units.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/units.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2010-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_UNITS_H
#define _RNR_UNITS_H

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

//-----------------------------------------------------------------------------
// Defines and Data Types
//-----------------------------------------------------------------------------

/*!
 * \brief Unit Types
 */
typedef enum
{
  units_undef,      ///< undefined units

  // raw
  units_raw,        ///< raw, native device units

  // relative
  units_percent,    ///< percent % of maximum raw value
  units_permil,     ///< permil \h_permil of maximum raw value
  units_norm,       ///< normalized units (continuous from -1 to 1)

  // linear distance
  units_um,         ///< micrometers
  units_mm,         ///< millimeters
  units_cm,         ///< centimeters
  units_m,          ///< meters

  // angular distance
  units_radians,    ///< radians
  units_degrees,    ///< degrees

  // linear velocity
  units_um_per_s,   ///< micrometers/s
  units_mm_per_s,   ///< millimeters/s
  units_cm_per_s,   ///< centimeters/s
  units_m_per_s,    ///< meters/s

  // angular velocity
  units_rad_per_s,  ///< radians/s
  units_deg_per_s,  ///< degrees/s
  units_rpm,        ///< revolutions per minute

  // linear acceleration
  units_g,          ///< 1 earth g = 9.80665m/s<sup>2</sup>
  units_m_per_s2,   ///< meters/s<sup>2</sup>

  // force
  units_kgf,        ///< kilogram-force: 1kgf = 9.80665 N
  units_gf,         ///< gram-force: 1gf = 0.001kgf
  units_newton,     ///< newton N = 1 kg x m / s<sup>2</sup>
  units_dyne,       ///< dyne: 1 dyne = N<sup>-5</sup>
  
  // electrical
  units_volt,         ///< volt
  units_amp,          ///< ampheres
  units_ohm,          ///< ohms
  units_conductance,  ///< mho
  units_compacitance, ///< farad

  // temperature
  units_t_f,          ///< Fahrenheit 
  units_t_c,          ///< Celsius
  units_t_k,          ///< Kelvin
  
  // energy
  
  units_numof         ///< number of units
} units_t;


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

extern const char *units_shortname(units_t u);


C_DECLS_END


#endif // _RNR_UNITS_H
