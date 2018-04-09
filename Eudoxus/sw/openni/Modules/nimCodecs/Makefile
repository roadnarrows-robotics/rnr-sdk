################################################################################
#
# ./sw/openni/Modules/nimCodecs/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The OpenNI codecs modules makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/openni/Modules/nimCodecs/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ../../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Dynamic Shared Libraries
RNMAKE_DIST_DLLIBS = nimCodecs

# Source Files
nimCodecs.SRC.CPP = $(shell ls *.cpp) 

# Linked Libraries 
nimCodecs.LIBS	= EuOpenNI JPEG stdc++

nimCodecs.LIBDEPS	= EuOpenNI JPEG


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libnimCodes\""

EXTRA_CXXFLAGS = -fvisibility=hidden

EXTRA_INCDIRS = ..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
