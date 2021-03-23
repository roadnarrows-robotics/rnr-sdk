/******************************************************************************
 *
 * Package:   BotSense
 *
 * File:      BotSenseCore.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief Core BotSense python swig interface definitions file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 *   (C) 2010.  RoadNarrows LLC.
 *   (http://www.roadnarrows.com)
 *   All Rights Reserved
 */

/*
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
 ******************************************************************************/

%module BotSenseCore
%{
#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
/* RDK add mod i/f here */
%}

%begin
%{
/*! \file
 *  \brief Swig generated core wrapper c file.
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

%include "carrays.i"
%include "cpointer.i"

// suppress 'Warning 451: Setting a const char * variable may leak memory.'
#pragma SWIG nowarn=451

%include "botsense/BotSense.h"
%include "botsense/libBotSense.h"

%array_functions(byte_t, byteArray);
%array_functions(BsVConnHnd_T, hndVConnArray);
%pointer_functions(uint_t, uintp);

%immutable BsClientAppInfo_T::app_name;
%immutable BsClientAppInfo_T::brief;
%immutable BsClientAppInfo_T::version;
%immutable BsClientAppInfo_T::date;
%immutable BsClientAppInfo_T::maintainer;
%immutable BsClientAppInfo_T::license;

%extend BsClientAppInfo_T
{
  BsClientAppInfo_T(const char *app_name, const char *brief,
                  const char *version, const char *date,
                  const char *maintainer, const char *license,
                  const char *(*getMsgName)(BsClient_P, BsVConnHnd_T, uint_t ))

  {
    assert(app_name!=NULL);
    assert(brief!=NULL);
    assert(version!=NULL);
    assert(date!=NULL);
    assert(maintainer!=NULL);
    assert(license!=NULL);

    struct {const char *s; size_t n;} m[] =
    {
      {app_name, },
      {brief, },
      {version, },
      {date, },
      {maintainer, },
      {license, },
      {NULL, }
    };
    size_t n = 0;
    int i;
    for(i=0; m[i].s!=NULL; ++i)
    {
      m[i].n = n;
      n += strlen(m[i].s) + 1;
    }

    // single malloc() so that a single free() will work
    BsClientAppInfo_T *p = malloc(sizeof(BsClientAppInfo_T) + n);

    // put data in malloc'd block after struct
    void *q = (void *)&p[1];
    for(i=0; m[i].s!=NULL; ++i)
    {
      strcpy(q+m[i].n, m[i].s);
    }
    i = 0;
    p->app_name = q+m[i++].n;
    p->brief = q+m[i++].n;
    p->version = q+m[i++].n;
    p->date = q+m[i++].n;
    p->maintainer = q+m[i++].n;
    p->license = q+m[i++].n;
    p->fnGetMsgName = getMsgName;
    return p;
  }
}

%immutable BsClientConnState_T::m_sServerHostName;

%extend BsClientConnState_T
{
  BsClientConnState_T(bool_t isConnected, const char *serverHostName)
  {
    assert(serverHostName!=NULL);
    // single malloc() so that a single free() will work
    BsClientConnState_T *p = malloc(sizeof(BsClientConnState_T)
                                    + strlen(serverHostName) + 1);
    // put data in malloc'd block after struct
    char *buf = (void *)&p[1];
    strcpy(buf, serverHostName);
    // assign struct
    p->m_bIsConnected = isConnected;
    p->m_sServerHostName = buf;
    return p;
  }
}

/*
 * Higher-level python interface to the core C library.
 */
%pythoncode
%{

"""
BotSense Core Python Inline Extensions and Wrappers.
"""

## \file 
## \package BotSense.BotSenseCore
##
## \brief BotSense Swigged Core Python Interface Module.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2012.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

%}
