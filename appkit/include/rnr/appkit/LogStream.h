////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      LogStream.h
//
/*! \file
 *
 * \brief Logging facitlities built on librnr log.h macros to support C++
 * output insertion streaming.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par License:
 * MIT
 */
/*
 * @EulaBegin@
 * 
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
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_LOG_STREAM_H
#define _RNR_LOG_STREAM_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <sstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

/*!
 * \brief User diagnostic stream logging.
 * \param level Logging user level.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGUSER_STREAM(level, args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGUSER(level, "%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Diagnostic level 5 stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGDIAG5_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGDIAG5("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Diagnostic level 4 stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGDIAG4_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGDIAG4("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Diagnostic level 3 stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGDIAG3_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGDIAG3("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Diagnostic level 2 stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGDIAG2_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGDIAG2("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Diagnostic level 1 stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGDIAG1_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGDIAG1("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Warning stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGWARN_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGWARN("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief Error stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGERROR_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGERROR("%s", ss.str().c_str()); \
  } \
  while(0)

/*!
 * \brief System Error stream logging.
 * \param args  Stream arguments arg [<< arg [<< ...]].
 */
#define LOGSYSERROR_STREAM(args) \
  do \
  { \
    stringstream ss; \
    ss << args; \
    LOGSYSERROR("%s", ss.str().c_str()); \
  } \
  while(0)


#endif // _RNR_LOG_STREAM_H
