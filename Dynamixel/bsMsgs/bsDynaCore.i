/******************************************************************************
 *
 * Package:   BotSense
 *
 * File:      bsDynaCore.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief BotSense Dynamixel python swig interface core definitions file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 *   (C) 2012-2015.  RoadNarrows LLC.
 *   (http://www.roadnarrows.com)
 *   All Rights Reserved
 */

/*
 * @EulaBegin@
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
 *
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 * @EulaEnd@
 ******************************************************************************/

%module bsDynaCore
%{
#include "botsense/BotSense.h"
#include "botsense/bsDyna.h"
%}

%begin
%{
/*! \file
 *  \brief Swig generated null wrapper c file.
 */
%}

/* 
 * Required RNR C Types
 */
typedef unsigned char byte_t;
typedef unsigned short ushort_t;
typedef unsigned int uint_t;
typedef unsigned long ulong_t;
typedef int bool_t;
/*typedef int bool;*/

/*
 * Required BotSense C Types
 */
typedef struct _bsClientStruct *BsClient_P;
typedef int BsVConnHnd_T;

/*
 * Dynamixel Simple Types
 */
typedef int units_t;

%include "carrays.i"
%include "cpointer.i"

/* the swigged interface */
%include "Dynamixel/Dynamixel.h"
%include "botsense/bsDyna.h"

%array_functions(uint_t, uintArray);
%array_functions(bool_t, boolArray);
%pointer_functions(int, intp);
%pointer_functions(uint_t, uintp);

/*
 * Higher-level python interface to the BotSense Dynamixel robot C library.
 */
%pythoncode
%{

"""
BotSense Dynamixel Python Core Interface.
"""

## \file 
## \package BotSense.Dynamixel.bsDynaCore
##
## \brief BotSense swigged Dynamixel Python core interface module.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2012-2015.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

%}
