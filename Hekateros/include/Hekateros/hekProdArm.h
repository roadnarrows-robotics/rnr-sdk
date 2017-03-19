////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProdArm.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Aggregagte of supported Hekateros robotic arms static specifications.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
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

#ifndef _HEK_PROD_ARM_H
#define _HEK_PROD_ARM_H

#include "rnr/rnrconfig.h"

#include "Hekateros/hekateros.h"

#define HEK_ARM_FAMILY  0x01  ///< only 1 family of arm products so define here

#include "Hekateros/hekProdArm5LBeta.h"
#include "Hekateros/hekProdArm4S.h"
#include "Hekateros/hekProdArm4L.h"
#include "Hekateros/hekProdArm5L.h"
#include "Hekateros/hekProdArm5S.h"

#endif // _HEK_PROD_ARM_H
