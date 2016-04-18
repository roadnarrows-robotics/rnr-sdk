/******************************************************************************
 *
 * Package:   Laelaps
 *
 * File:      RoboClawMsgs.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief RoboClaw motor controller messaging python swig definitions file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 *   (C) 2016.  RoadNarrows LLC.
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
 * @EulaEnd@
 *
 ******************************************************************************/

%module RoboClawMsgs
%{
#define SWIG
#include "rnr/rnrconfig.h"
#include "Laelaps/RoboClaw.h"
#undef SWIG
%}

%begin
%{
/*! \file
 *  \brief Swig generated RoboClaw messaging wrapper c file.
 */
%}

/* 
 * Required RNR C types
 */
typedef unsigned char byte_t;
typedef unsigned short ushort_t;
typedef unsigned int uint_t;
typedef unsigned long ulong_t;
typedef int bool_t;

#define static 
#define const %constant

%include "Laelaps/RoboClaw.h"

#undef static
#undef const

%include "carrays.i"
%include "cpointer.i"

%inline
%{
%}

/*
 * Higher-level python interface to the core C library.
 */
%pythoncode
%{

"""
RoadNarrows Robotics Laelaps WatchDog Subprocessor Messages.
"""

## \file 
## \package LaeLaeps.WatchDogMsgs
##
## \brief RoadNarrows Robotics Swigged RoboClaw Motor Controller Messaging
##  Python Module.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

%}
