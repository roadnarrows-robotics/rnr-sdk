################################################################################
#
# ./sw/tinyxml/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_tinyxml library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/tinyxml/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2013-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_tinyxml

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_tinyxml

# Target library subdirectory
rnr_tinyxml.SUBDIR = rnr

# Source Files
rnr_tinyxml.SRC.CPP = tinyxml.cpp tinyxmlparser.cpp tinyxmlerror.cpp tinystr.cpp

# Linked Libraries 
rnr_tinyxml.LIBS	= stdc++


#------------------------------------------------------------------------------
# Test Programs

# uncomment to build test program
# RNMAKE_TEST_PGMS = xmltest

rnr_xmltest.LIBS = tinyxml

# C++ Source Files
xmltest.SRC.CPP	= xmltest.cpp


#------------------------------------------------------------------------------
# Extras

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

.PHONY: extra-make-post
extra-make-post: 	$(RNMAKE_PKG_ROOT)/include/rnr/tinyxml/tinyxml.h \
									$(RNMAKE_PKG_ROOT)/include/rnr/tinyxml/tinystr.h

$(RNMAKE_PKG_ROOT)/include/rnr/tinyxml/tinyxml.h: tinyxml.h
	$(CP) $(<) $(@)
 
$(RNMAKE_PKG_ROOT)/include/rnr/tinyxml/tinystr.h: tinystr.h
	$(CP) $(<) $(@)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
