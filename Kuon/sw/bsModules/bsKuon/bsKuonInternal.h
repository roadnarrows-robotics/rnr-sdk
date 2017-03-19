////////////////////////////////////////////////////////////////////////////////
//
// Package:   kuon
//
// Module:    bsKuon
//
// Library:   libbsserver_kuon
//            libbsclient_kuon
//
// File:      bsKuonInternal.h
//
/*! \file
 *
 * $LastChangedDate: 2012-06-12 14:57:23 -0600 (Tue, 12 Jun 2012) $
 * $Rev: 2041 $
 *
 * \brief RoadNarrows Kuon robot top-level header file.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _BSKUON_INTERNAL_H
#define _BSKUON_INTERNAL_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"


#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG


// ---------------------------------------------------------------------------
// Defines and Types
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

#ifndef SWIG

extern int bsKuonBgThreadStart(KuonImu_T *pImu);

extern void bsKuonBgThreadStop();

extern void bsKuonBgThreadJoin();


#endif // SWIG


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _BSKUON_INTERNAL_H
