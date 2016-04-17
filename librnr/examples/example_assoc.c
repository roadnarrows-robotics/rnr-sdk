////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_assoc.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Example of a librnr simple associative map operations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2007-2010  RoadNarrows LLC.
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"


/*!
 * \brief Major and minor solar body ids (partial, of course).
 */
typedef enum 
{
  unknown,

  Sun,
  Mercury,
  Venus,
  Earth, Moon,
  Mars, Phobos, Deimos,
  Ceres,
  Jupiter, Io, Europa, Ganymede, Callisto,
  Saturn, Titan,
  Uranus, Titania,
  Neptune, Triton,
  Pluto, Charon,
  Eris,

  NumBodies
} BodyId_T;

/*!
 * \brief Some solar body facts.
 */
typedef struct
{
  const char     *m_sName;            ///< name of body
  double          m_fDiam;            ///< diameter of body (km)
  double          m_fOrbitalAvgDist;  ///< average orbital distance (km)
  double          m_fOribtalPeriod;   ///< orbital revolution period (days)
  BodyId_T        m_eOrbits;          ///< body this body orbits around
} Factoids_T;

//
// Define planet id <--> planet facts associative map
//
#define AMAP_NAME   Planet            ///< associate map namespace
#define AMAP_XTYPE  BodyId_T          ///< x data type
#define AMAP_YTYPE  Factoids_T*       ///< y data type
#include "rnr/assoc.h"                ///< make the magic

/*!
 * \brief Solar body name-id pairs (partial).
 */
Nvp_T SolarBodyNames[] =
{
  {"unknown",   unknown}, // default
  {"Sun",       Sun},
  {"Mercury",   Mercury},
  {"Venus",     Venus},
  {"Earth",     Earth},
  {"Moon",      Moon},
  {"Mars",      Mars},
  {"Phobos",    Phobos},
  {"Deimos",    Deimos},
  {"Jupiter",   Jupiter},
  {"Io",        Io},
  {"Europa",    Europa},
  {"Ganymede",  Ganymede},
  {"Callisto",  Callisto},
  {"Saturn",    Saturn},
  {"Uranus",    Uranus},
  {"Neptune",   Neptune},
  {"Ceres",     Ceres},
  {"Pluto",     Pluto}
};

/*!
 * \brief the solar body factoids
 */
AssocMapPlanetPoint_T  SolarBodyFacts[NumBodies];

size_t BodyCount = 0;     ///< number of bodies in facts table above.

/*!
 * \brief Add a new body to our db, normalizing units
 *
 * \param eThisBodyId   Solar body id.
 * \param diam          Body diameter.
 * \param units_diam    Diameter units.
 * \param dist          Average orbital distance from the Sun.
 * \param units_dist    Distance units.
 * \param period        Average orbital period about parent body.
 * \param units_period  Period units.
 * \param eOrbitsId     Parent body.
 */
static void AddNewBody(BodyId_T eThisBodyId,
                      double diam, char *units_diam,
                      double dist, char *units_dist,
                      double period, char *units_period,
                      BodyId_T eOrbitsId)
{
  Factoids_T  *pOid = NEW(Factoids_T);
  const char  *sName;

  sName = NvpVal2Name(SolarBodyNames,
                      arraysize(SolarBodyNames),
                      (int)eThisBodyId);

  if( sName == NULL )
  {
    fprintf(stderr, "Error: %d: unknown body type", eThisBodyId);
    return;
  }

  // normalize diameter to km units
  if( !strcmp(units_diam, "miles") )
  {
    diam *= 1.609344;
  }

  // normalize distance to km units
  if( !strcmp(units_dist, "miles") )
  {
    dist *= 1.609344;
  }

  // normalize period to days
  if( !strcmp(units_period, "years") )
  {
    period *= 365.25;
  }

  // initialize factoid
  pOid->m_sName             = sName;
  pOid->m_fDiam             = diam;
  pOid->m_fOrbitalAvgDist   = dist;
  pOid->m_fOribtalPeriod    = period;
  pOid->m_eOrbits           = eOrbitsId;

  // insert into table
  SolarBodyFacts[BodyCount].x = eThisBodyId;
  SolarBodyFacts[BodyCount].y = pOid;

  BodyCount++;
}

/*!
 * \brief Print solar body facts.
 *
 * \param pOid  Pointer to facts.
 */
void PrintBody(Factoids_T *pOid)
{
  printf("name:     %s\n", pOid->m_sName);
  printf("diameter: %e km\n", pOid->m_fDiam);
  printf("distance: %e km\n", pOid->m_fOrbitalAvgDist);
  printf("period:   %f days\n", pOid->m_fOribtalPeriod);
  printf("orbits:   %s\n", NvpVal2Name(SolarBodyNames,
                                       arraysize(SolarBodyNames),
                                       (int)pOid->m_eOrbits));
}

/*!
 * \brief X comparator callback.
 *
 * \param x1  Solar body id x1.
 * \param x2  Solar body id x2.
 *
 * \returns \h_lt 0, 0, or \h_gt 0 if body x1 is less than, equal to, or greater
 * than x2, respectively.
 */
int CmpIds(const BodyId_T x1, const BodyId_T x2)
{
  return (int)(x1 - x2);
}

/*!
 * \brief Y comparator callback.
 *
 * \param y1  Solar body factoids y1.
 * \param y2  Solar body factoids y2.
 *
 * \returns \h_lt 0, 0, or \h_gt 0 if y1 is less than, equal to, or greater
 * than y2, respectively.
 */
int CmpOids(const Factoids_T *y1, const Factoids_T *y2)
{
  return strcmp(y1->m_sName, y2->m_sName);
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  AssocMapPlanetMapper_T   map;                   // associative x<-->y mapper
  AssocMapPlanetPoint_T    dft = {unknown, NULL}; // default "body"
  Factoids_T              *pOid;                  // working factoid
  Factoids_T               findoid;               // search factoid

  // add some factioids
  AddNewBody(Mercury, 4878.0, "km", 46.0E6, "km", 0.24, "years", Sun);
  AddNewBody(Moon, 3476.0, "km", 384400.0, "km", 27.322, "days", Earth);
  AddNewBody(Deimos, 8.0, "km", 23460.0, "km", 1.263, "days", Mars);
  AddNewBody(Phobos, 28.0, "km", 9270.0, "km", 0.319, "days", Mars);
  AddNewBody(Callisto, 4800.0, "km",  1883000.0, "km", 16.689, "days", Jupiter);
  AddNewBody(Europa, 3126.0, "km", 670900.0, "km", 3.551, "days", Jupiter);
  AddNewBody(Jupiter, 88736.0, "miles",  817.0E6, "km", 12.0, "years", Sun);
  AddNewBody(Earth, 7926.0, "miles", 91.0E6, "miles", 1.0, "years", Sun);
  
  // initialize mapper
  map.m_tblAssocMap = SolarBodyFacts;
  map.m_tblSize     = BodyCount;
  map.m_opXCmp      = CmpIds;
  map.m_opYCmp      = CmpOids;
  map.m_pMapDft     = &dft;

  //
  // print some data
  //

  if( (pOid = AssocMapPlanetXtoY(&map, Moon)) != NULL )
  {
    PrintBody(pOid);
    printf("\n");
  }
  if( (pOid = AssocMapPlanetXtoY(&map, Mercury)) != NULL )
  {
    PrintBody(pOid);
    printf("\n");
  }
  if( (pOid = AssocMapPlanetXtoY(&map, Earth)) != NULL )
  {
    PrintBody(pOid);
    printf("\n");
  }
  if( (pOid = AssocMapPlanetXtoY(&map, Callisto)) != NULL )
  {
    PrintBody(pOid);
    printf("\n");
  }
  if( (pOid = AssocMapPlanetXtoY(&map, Eris)) != NULL )   // negative
  {
    PrintBody(pOid);
    printf("\n");
  }

  // what was ceres' id? negative
  findoid.m_sName = "Ceres";    // note: only searches on name in this example
  printf("ceres id: %u\n", AssocMapPlanetYtoX(&map, &findoid));

  // an Jupiter's id?
  findoid.m_sName = "Jupiter";
  printf("jupiter id: %u\n", AssocMapPlanetYtoX(&map, &findoid));

  return 0;
}
