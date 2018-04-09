################################################################################
#
# ./sw/primesense/Utils/XnSensorServer/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The PrimeSense XnSensorServer application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/primesense/Utils/XnSensorServer/Makefile}

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
# Programs

RNMAKE_DIST_PGMS = XnSensorServer 

# Libraries to Link With
XnSensorServer.LIBS =	\
	XnCore \
	XnDDK \
	XnFormats \
	XnDeviceSensorV2 \
	OpenNI \
	JPEG \
	TinyXml \
	stdc++ \
	pthread 

XnSensorServer.LIBDEPS = \
	XnCore XnDDK XnFormats XnDeviceSensorV2 OpenNI JPEG TinyXml

# Source Files
XnSensorServer.SRC.CPP = SensorServer.cpp


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"XnSensorServer\""

EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
