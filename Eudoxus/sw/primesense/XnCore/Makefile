################################################################################
#
# ./sw/primesense/XnCore/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The PrimeSense libXnCore library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/primesense/XnCore/Makefile}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS       = XnCore

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS       = XnCore

# Source Files
XnCore.SRC.CPP = $(shell ls *.cpp) 

# Linked Libraries 
XnCore.LIBS	= EuOpenNI JPEG stdc++

XnCore.LIBDEP	= EuOpenNI JPEG


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DXN_CORE_EXPORTS -DLOG -DLOGMOD="\"libXnCore\""

EXTRA_CXXFLAGS = -fvisibility=hidden

EXTRA_INCDIRS = ..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
