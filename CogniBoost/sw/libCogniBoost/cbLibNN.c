////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibNN.c
//
/*! \file
 *
 * $LastChangedDate: 2011-09-30 13:30:51 -0600 (Fri, 30 Sep 2011) $
 * $Rev: 1327 $
 *
 * \brief CogniBoost Neural Network interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/CogniBoostProto.h"
#include "CogniBoost/CogniBoostMsgs.h"
#include "CogniBoost/libCogniBoost.h"

#include "cbLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------


// returns nid or -error
extern int cbNNTrain(cbHnd_T hnd, uint_t uCategory, cbNNPattern_T *pPattern);
extern int cbNNCategorize(cbHnd_T hnd, cbNNPattern_T *pPattern,
    cbClassification_T classification[], size_t size);
extern int cbNNForget(cbHnd_T hnd);
// returns count or -error
extern int cbNNGetCount(cbHnd_T hnd);
extern int cbNNUploadTrainingSet(cbHnd_T hnd, cbNNSet_T *pNNSet);
extern int cbNNDownloadTrainingSet(cbHnd_T hnd, cbNNSet_T *pNNSet);
