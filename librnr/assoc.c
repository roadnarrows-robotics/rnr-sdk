////////////////////////////////////////////////////////////////////////////////
/*!
 * \file
 *
 * \brief Simple associvative map operator definitions.
 *
 * These associative maps are composed of tables (vectors) of discrete (x, y)
 * points where the x and y may be of any data types.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{assoc.c}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/dliststr.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/assoc.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the value associated with the name.
 *
 * The table search terminates at the first match or at the end of 
 * nTblEntries entries.
 *
 * \param tbl[]         Name-Value Pair table
 * \param nTblEntries   Number of name-value pair table entries
 * \param sName         Name to in table to find.
 *
 * \return 
 * The associated value on success, or the first (default) value in the 
 * table if the no name match is found.
 */
int NvpName2Val(Nvp_T tbl[], size_t nTblEntries, const char *sName)
{
  size_t i;

  CHKEXPR_ULONG(nTblEntries, (nTblEntries > 0), 0);

  for(i=0; i<nTblEntries; ++i)
  {
    if( !strcmp(tbl[i].m_sName, sName) )
    {
      return tbl[i].m_iVal;
    }
  }
  return tbl[0].m_iVal;
}

/*!
 * \brief Get the name associated with the value.
 *
 * The table search terminates at the first match or at the end of 
 * nTblEntries entries.
 *
 * \param tbl[]         Name-Value Pair table
 * \param nTblEntries   Number of name-value pair table entries
 * \param iVal          Value in table to find.
 *
 * \return 
 * The associated name on success, or the first (default) value in the 
 * table if the no value match is found.
 */
const char *NvpVal2Name(Nvp_T tbl[], size_t nTblEntries, int iVal)
{
  size_t i;

  CHKEXPR_ULONG(nTblEntries, (nTblEntries > 0), 0);

  for(i=0; i<nTblEntries; ++i)
  {
    if( tbl[i].m_iVal == iVal )
    {
      return tbl[i].m_sName;
    }
  }
  return tbl[0].m_sName;
}

/*!
 * \brief Get the y value associated with the given x value.
 *
 * The table search terminates at the first match or at the end of 
 * the table.
 *
 * \param pMapper    Associative map mapper container.
 * \param px         Pointer to X value in table to find.
 *
 * \return 
 * The associated y value on success, or the default y value if no
 * x value is found.
 */
void *AssocMapVoidXtoY(AssocMapVoidMapper_T *pMapper, void *px)
{
  size_t i;

  CHKPTR(pMapper, NULL);
  CHKPTR(pMapper->m_tblAssocMap, NULL);
  CHKPTR(pMapper->m_pMapDft, NULL);

  if( pMapper->m_opXCmp == NULL )
  {
    return pMapper->m_pMapDft->y;
  }

  for(i=0; i<pMapper->m_tblSize; ++i)
  {
    if( pMapper->m_opXCmp(pMapper->m_tblAssocMap[i].x, px) == 0 )
    {
      return pMapper->m_tblAssocMap[i].y;
    }
  }
  return pMapper->m_pMapDft->y;
}

/*!
 * \brief Get the x value associated with the given y value.
 *
 * The table search terminates at the first match or at the end of 
 * the table.
 *
 * \param pMapper    Associative map mapper container.
 * \param py         Pointer to Y value in table to find.
 *
 * \return 
 * The associated x value on success, or the default x value if no
 * y value is found.
 */
void *AssocMapVoidYtoX(AssocMapVoidMapper_T *pMapper, void *py)
{
  size_t i;

  CHKPTR(pMapper, NULL);
  CHKPTR(pMapper->m_tblAssocMap, NULL);
  CHKPTR(pMapper->m_pMapDft, NULL);

  if( pMapper->m_opYCmp == NULL )
  {
    return pMapper->m_pMapDft->x;
  }

  for(i=0; i<pMapper->m_tblSize; ++i)
  {
    if( pMapper->m_opYCmp(pMapper->m_tblAssocMap[i].y, py) == 0 )
    {
      return pMapper->m_tblAssocMap[i].x;
    }
  }
  return pMapper->m_pMapDft->x;
}
