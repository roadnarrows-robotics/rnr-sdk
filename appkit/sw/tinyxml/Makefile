################################################################################
#
# ./sw/tinyxml/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-02-18 12:42:09 -0700 (Mon, 18 Feb 2013) $
$Rev: 2691 $

\brief
Make tinyxml in RoadNarrows rnmake system.

RN Make System Specific Makefile

\author: @AUTHOR@ <@EMAIL@>

\par Copyright:
  (C) @DATE@ @OWNER@
  (@WEBSITE@)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, @OWNER@, OR ANY MEMBERS/EMPLOYEES
# OF @OWNER@ OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND @OWNER@ SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS       = rnr_tinyxml

# Distribution Shared Libraries
DIST_SHLIBS       = rnr_tinyxml

# Source Files
rnr_tinyxml.SRC.CPP = tinyxml.cpp tinyxmlparser.cpp tinyxmlerror.cpp tinystr.cpp

# Linked Libraries 
rnr_tinyxml.LIBS	= stdc++

#------------------------------------------------------------------------------
# Test Programs

# uncomment to build test program
# LOC_PGMS =xmltest

rnr_xmltest.LIBS		= tinyxml

# C++ Source Files
xmltest.SRC.CPP	= xmltest.cpp

#------------------------------------------------------------------------------
# Optional Variables 
#

TINYXML_DEBUG          := NO

# TINYXML_USE_STL can be used to turn on STL support. NO, then STL
# will not be used. YES will include the STL files.
TINYXML_USE_STL := NO

TINYXML_DEBUG_CFLAGS     := -Wall -Wno-format -g -DDEBUG
TINYXML_RELEASE_CFLAGS   := -O3

TINYXML_DEBUG_CPPFLAGS   := ${TINYXML_DEBUG_CFLAGS} 
TINYXML_RELEASE_CPPFLAGS := ${TINYXML_RELEASE_CFLAGS}

TINYXML_DEBUG_LDFLAGS    := -g
TINYXML_RELEASE_LDFLAGS  :=

ifeq (YES, ${TINYXML_USE_STL})
  DEFS := -DTIXML_USE_STL
else
  DEFS :=
endif

ifeq (YES, ${TINYXML_DEBUG})
   EXTRA_CFLAGS       := ${TINYXML_DEBUG_CFLAGS}
   EXTRA_CPPFLAGS     := ${TINYXML_DEBUG_CPPFLAGS}
   EXTRA_LDFLAGS      := ${TINYXML_DEBUG_LDFLAGS}
else
   EXTRA_CFLAGS       := ${TINYXML_RELEASE_CFLAGS}
   EXTRA_CPPFLAGS     := ${TINYXML_RELEASE_CPPFLAGS}
   EXTRA_LDFLAGS      := ${TINYXML_RELEASE_LDFLAGS}
endif

EXTRA_CFLAGS   += ${DEFS}
EXTRA_CXXFLAGS += ${DEFS}

EXTRA_TGT_ALL_POST  = extra-make-post

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

.PHONY: extra-make-post
extra-make-post: $(pkgroot)/include/rnr/tinyxml/tinyxml.h $(pkgroot)/include/rnr/tinyxml/tinystr.h

$(pkgroot)/include/rnr/tinyxml/tinyxml.h: tinyxml.h
	$(CP) $(<) $(@)
 
$(pkgroot)/include/rnr/tinyxml/tinystr.h: tinystr.h
	$(CP) $(<) $(@)
