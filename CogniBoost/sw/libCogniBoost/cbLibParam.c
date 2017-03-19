////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibParam.c
//
/*! \file
 *
 * $LastChangedDate: 2011-09-30 13:30:51 -0600 (Fri, 30 Sep 2011) $
 * $Rev: 1327 $
 *
 * \brief CogniBoost parameters interface.
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

// reconnect at new baudrate
extern int cbParamSetBaudRate(cbHnd_T hnd, uint_t uBaudRate);
extern int cbParamGetBaudRate(cbHnd_T hnd, uint_t *pBaudRate);

extern int cbParamSetAutoRestore(cbHnd_T hnd, bool_t bEnable);
extern int cbParamGetAutoRestore(cbHnd_T hnd, bool_t *pEnable);

extern int cbParamSetAutoSleep(cbHnd_T hnd, uint_t uMsec);
extern int cbParamGetAutoSleep(cbHnd_T hnd, uint_t *pMsec);

extern int cbParamSetLedBling(cbHnd_T hnd, cbLedBlingParams_T *pLedBling);
extern int cbParamGetLedBling(cbHnd_T hnd, cbLedBlingParams_T *pLedBling);



extern int cbParamSetCMClassifier(cbHnd_T hnd, uint_t uClassifier);
extern int cbParamGetCMClassifier(cbHnd_T hnd, uint_t *pClassifier);

extern int cbParamSetCMContext(cbHnd_T hnd, uint_t uContext);
extern int cbParamGetCMContext(cbHnd_T hnd, uint_t *pContext);

extern int cbParamSetCMNorm(cbHnd_T hnd, uint_t uNorm);
extern int cbParamGetCMNorm(cbHnd_T hnd, uint_t *pNorm);

extern int cbParamSetCMMinIF(cbHnd_T hnd, uint_t uMinIF);
extern int cbParamGetCMMinIF(cbHnd_T hnd, uint_t *pMinIF);

extern int cbParamSetCMMaxIF(cbHnd_T hnd, uint_t uMaxIF);
extern int cbParamGetCMMaxIF(cbHnd_T hnd, uint_t *pMaxIF);

extern int cbParamSetCMMaxClassified(cbHnd_T hnd, uint_t uMaxClassified);
extern int cbParamGetCMMaxClassified(cbHnd_T hnd, uint_t *pMaxClassified);

extern int cbParamSetNNLabel(cbHnd_T hnd, char *sLabel);
extern int cbParamGetNNLabel(cbHnd_T hnd, char buf[], size_t size);

extern int cbParamSetAll(cbHnd_T hnd, cbCogniBoostParams_T *pParams);
extern int cbParamGetAll(cbHnd_T hnd, cbCogniBoostParams_T *pParams);

