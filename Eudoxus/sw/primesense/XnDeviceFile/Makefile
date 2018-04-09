################################################################################
#
# ./sw/primesense/XnDeviceFile/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The PrimeSense libXnDeviceFile library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/primesense/XnDeviceFile/Makefile}

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
RNMAKE_DIST_STLIBS       = XnDeviceFile

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS       = XnDeviceFile

# Source Files
XnDeviceFile.SRC.CPP = $(shell ls *.cpp) 

# Linked Libraries 
XnDeviceFile.LIBS	= XnCore XnFormats XnDDK EuOpenNI JPEG stdc++

XnDeviceFile.LIBDEP	= XnCore XnFormats XnDDK EuOpenNI JPEG

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DXN_DEVICE_EXPORTS -DLOG -DLOGMOD="\"libXnDeviceFile\""

EXTRA_CXXFLAGS = -fvisibility=hidden

EXTRA_INCDIRS = ..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
