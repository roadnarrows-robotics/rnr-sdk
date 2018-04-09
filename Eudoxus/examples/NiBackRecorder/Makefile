################################################################################
#
# ./examples/NiBackRecorder/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The NiBackRecord example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{examples/NiBackRecorder/Makefile}

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

RNMAKE_DIST_PGMS = NiBackRecorder 

# Libraries to Link With
NiBackRecorder.LIBS = \
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

NiBackRecorder.LIBDEPS = TinyXml JPEG OpenNI

# Source Files
NiBackRecorder.SRC.CPP = NiBackRecorder.cpp


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"NiBackRecorder\""

#EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
