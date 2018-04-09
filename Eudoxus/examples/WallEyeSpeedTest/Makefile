################################################################################
#
# ./examples/WallEyeSpeedTest/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The WallEyeSpeedTest example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{examples/WallEyeSpeedTest/X/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = WallEyeSpeedTest 

# Libraries to Link With
WallEyeSpeedTest.LIBS = \
	OpenNI \
	TinyXml \
	JPEG \
	Eudoxus \
	rnr \
	stdc++ \
	usb-1.0 \
	dl \
	m \
	pthread 

WallEyeSpeedTest.LIBDEPS = TinyXml JPEG OpenNI

# Source Files
WallEyeSpeedTest.SRC.CPP = WallEyeSpeedTest.cpp ofsensor.cpp


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"WallEyeSpeedTest\""

#EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
