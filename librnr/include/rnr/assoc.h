////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Simple associative map data and operator declarations.
 *
 * These associative maps are composed of tables (vectors) of discrete (x, y)
 * points where x and y may be of any data types.
 *
 * The specific Nvp_T Name-Value Pair associative map point provides a
 * char* \<--\> int mapping.
 *
 * The more general associative mapping is provided by the
 * AssocMap\<<em>x</em>\> declarations.
 *
 * The general associative maps are configured to support a "poor man's" C
 * version of class inheritance and templating.
 *
 * The "base" set of void* data types and functions are 
 * AssocMapVoid\<<em>x</em>\> named definitions with data types returning
 * void*. These definitions are defined in librnr.
 *
 * A "derived" associative map can be defined using the following three
 * \#define's (usually in a header file). Follow these \#defines with the 
 * inclusion of this header file (assoc.h):\n
 * <dl>
 * <dt> \ref AMAP_NAME </dt>
 *  <dd> 
 *    The associative map derived namespace name. All data types and
 *    function definitions are named:   
 *      AssocMap\<\ref AMAP_NAME\>\<<em>x</em>\>
 *  </dd>
 * <dt> \ref AMAP_XTYPE </dt>
 *  <dd>
 *    The derived x data type.
 *  </dd>
 * <dt> \ref AMAP_YTYPE </dt>
 *  <dd>
 *    The the derived y data type.
 *  </dd>
 * </dl>
 *
 * Multiple assoc. maps derived types and the base type may be used in the
 * same C source.
 *
 * \sa 
 * \ref example_assoc under "Related Pages" for an example.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/assoc.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
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
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_ASSOC_H
#include <sys/types.h>
#endif // _RNR_ASSOC_H

//-----------------------------------------------------------------------------
// Base and Derived Associative Controls
//-----------------------------------------------------------------------------
/*!
 * Define the associative namespace (must be literal).
 *
 * If \ref AMAP_NAME is defined then a new set of assoc. map data types and
 * functions will be defined, prefaced by AssocMap\<\ref AMAP_NAME\>.
 *
 * If \ref AMAP_NAME is not defined, then the base set prefaced by
 * AssocMapVoid data types and functions will be used (already defined in 
 * librnr).
 */
#ifndef AMAP_NAME
#define AMAP_NAME Void                ///< base assoc. map name
#undef  _AMAP_DERIVED                 ///< not a "derived" assoc. map
#else
#define _AMAP_DERIVED                 ///< derived assoc. map
#endif
#define _AMAP_NAME AMAP_NAME          ///< assoc. map name

/*!
 * Define the derived associative map x and y data types (must be literal).
 *
 * \ref AMAP_XTYPE defines the x data type. If not defined, then void* is used.
 *
 * \ref AMAP_YTYPE defines the y data type. If not defined, then void* is used.
 */
#ifndef AMAP_XTYPE
#define AMAP_XTYPE   void *           ///< default assoc. map x data type
#endif
#ifndef AMAP_YTYPE
#define AMAP_YTYPE   void  *          ///< default assoc. map y data type
#endif
#define _AMAP_XTYPE   AMAP_XTYPE      ///< assoc. map x data type
#define _AMAP_YTYPE   AMAP_YTYPE      ///< assoc. map y data type

/*!
 * Useful string concatenation macros.
 */
#undef _CONCAT
#undef _CONCAT_
#define _CONCAT_(x, y)  x ## y            ///< build concatenation operator
#define _CONCAT(x, y)   _CONCAT_(x, y)    ///< now concatenate

/*!
 * \brief AssocMap (base or derived) definition preface
 */
#undef _AMAP_DEFPREFACE
#define _AMAP_DEFPREFACE   _CONCAT(AssocMap, _AMAP_NAME)

/*!
 * \brief AssocMap (base or derived) definition name
 */
#undef _AMAP_DEF
#define _AMAP_DEF(name) _CONCAT(_AMAP_DEFPREFACE, name)


//-----------------------------------------------------------------------------
// Base Data Types
//-----------------------------------------------------------------------------

#ifndef _RNR_ASSOC_H

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * Simple Name-Value Pair (Nvp) associative map entry structure. 
 *
 * \par Association:
 * name <--> value where name is a string and value is an integer.
 *
 * \note
 * Although there are no restrictions to either the names or the values,
 * be careful about shadowing. The library searches always return the
 * first match, hiding any potential subsequent matches.
 */
typedef struct
{
  const char  *m_sName;   ///< null-terminated string name
  int          m_iVal;    ///< associated integer value
} Nvp_T;


/*!
 * Base associative map data point structure type.
 *
 * \par Association:
 * x <--> y where x,y are of types void*.
 *
 * \note
 * Although there are no restrictions to either the names or the values,
 * be careful about shadowing. The library searches always return the
 * first match, hiding any potential subsequent matches.
 */
typedef struct
{
  void *x;    ///< the x value associates
  void *y;    ///< with this y value (and vice versa).
} AssocMapVoidPoint_T;

/*!
 * \brief base associative map comparator function type
 */
typedef int (*AssocMapVoidCmp_T)(void *t1, void *t2);

/*!
 * Base associative mapper structure type.
 *
 * The structure holds an associative map plus operators on that map.
 */
typedef struct
{
  AssocMapVoidPoint_T  *m_tblAssocMap;    ///< associative map table
  size_t                m_tblSize;        ///< number of entries in table
  AssocMapVoidCmp_T     m_opXCmp;         ///< return 0 if x1 == x2
  AssocMapVoidCmp_T     m_opYCmp;         ///< return 0 if y1 == y2
  AssocMapVoidPoint_T  *m_pMapDft;        ///< default x, y point 
} AssocMapVoidMapper_T;

C_DECLS_END

#endif // _RNR_ASSOC_H


//-----------------------------------------------------------------------------
// Derived Data Types
//-----------------------------------------------------------------------------

#ifdef _AMAP_DERIVED

C_DECLS_BEGIN

/*!
 * Derived associative map data point structure type.
 *
 * \par Association:
 * x <--> y where x is of type AMAP_XTYPE and y is of type AMAP_YTYPE.
 *
 * \par Name:
 * AssocMap\<\ref AMAP_NAME\>Point_T
 */
typedef struct
{
  _AMAP_XTYPE x;    ///< the x value associates
  _AMAP_YTYPE y;    ///< with this y value (and vice versa).
} _AMAP_DEF(Point_T);

/*!
 * \brief Derived x and y comparator callback function types. 
 *
 * \par Name:
 * AssocMap\<\ref AMAP_NAME\>XCmp_T()\n
 * AssocMap\<\ref AMAP_NAME\>YCmp_T()
 */
typedef int (*_AMAP_DEF(XCmp_T))(const _AMAP_XTYPE, const _AMAP_XTYPE);
typedef int (*_AMAP_DEF(YCmp_T))(const _AMAP_YTYPE, const _AMAP_YTYPE);

/*!
 * Derived associative mapper structure type.
 *
 * The structure holds an associative map plus operators on that map.
 *
 * \par Name:
 * AssocMap\<\ref AMAP_NAME\>Mapper_T\n
 */
typedef struct
{
  _AMAP_DEF(Point_T)   *m_tblAssocMap;    ///< associative map table
  size_t                m_tblSize;        ///< number of entries in table
  _AMAP_DEF(XCmp_T)     m_opXCmp;         ///< return 0 if x1 == x2
  _AMAP_DEF(YCmp_T)     m_opYCmp;         ///< return 0 if y1 == y2
  _AMAP_DEF(Point_T)   *m_pMapDft;        ///< default x, y point 
} _AMAP_DEF(Mapper_T);

C_DECLS_END

#endif // _AMAP_DERIVED


//-----------------------------------------------------------------------------
// Base Prototypes
//-----------------------------------------------------------------------------

#ifndef _RNR_ASSOC_H

C_DECLS_BEGIN

//
// Prototypes
//
extern int NvpName2Val(Nvp_T tbl[], size_t nTblEntries, const char *sName);

extern const char *NvpVal2Name(Nvp_T tbl[], size_t nTblEntries, int iVal);

extern void *AssocMapVoidXtoY(AssocMapVoidMapper_T *pMapper, void *px);

extern void *AssocMapVoidYtoX(AssocMapVoidMapper_T *pMapper, void *py);

C_DECLS_END

#endif // _RNR_ASSOC_H


//-----------------------------------------------------------------------------
// Derived Extensions
//-----------------------------------------------------------------------------

#ifdef _AMAP_DERIVED

C_DECLS_BEGIN

/*!
 * \brief Derived: Get the y value associated with the given x value.
 *
 * The table search terminates at the first match or at the end of 
 * the table.
 *
 * \par Name:
 * AssocMap\<\ref AMAP_NAME\>XtoY()
 *
 * \param pMapper    Derived associative map mapper container.
 * \param x          Derived x value in table to find.
 *
 * \return 
 * The associated derived y value on success, or the default y value if no
 * x value is found.
 */
static inline _AMAP_YTYPE _AMAP_DEF(XtoY)(_AMAP_DEF(Mapper_T) *pMapper,
                                          _AMAP_XTYPE x)
{
  return (_AMAP_YTYPE)AssocMapVoidXtoY((AssocMapVoidMapper_T *)pMapper,
                                       (void *)x);
}

/*!
 * \brief Derived: Get the x value associated with the given y value.
 *
 * The table search terminates at the first match or at the end of 
 * the table.
 *
 * \par Name:
 * AssocMap\<\ref AMAP_NAME\>YtoX()
 *
 * \param pMapper    Derived associative map mapper container.
 * \param y          Derived y value in table to find.
 *
 * \return 
 * The associated derived x value on success, or the default x value if no
 * y value is found.
 */
static inline _AMAP_XTYPE _AMAP_DEF(YtoX)(_AMAP_DEF(Mapper_T) *pMapper,
                                          _AMAP_YTYPE y)
{
  return (_AMAP_XTYPE)AssocMapVoidYtoX((AssocMapVoidMapper_T *)pMapper,
                                       (void *)y);
}

C_DECLS_END

#endif // _AMAP_DERIVED


//-----------------------------------------------------------------------------
// Fix Up Defines
//-----------------------------------------------------------------------------

/*!
 * Undefine macros to allow redefinition.
 */
#undef AMAP_NAME
#undef AMAP_XTYPE
#undef AMAP_YTYPE
#undef _AMAP_DERIVED
#undef _AMAP_NAME
#undef _AMAP_XTYPE
#undef _AMAP_YTYPE

#define _RNR_ASSOC_H    ///< include base declaration only once
