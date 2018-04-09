################################################################################
#
# ./examples/NiViewer/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The NiViewer example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{examples/NiViewer/Makefile}

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

RNMAKE_DIST_PGMS = NiViewer 

# Libraries to Link With
NiViewer.LIBS = \
	OpenNI \
	TinyXml \
	JPEG \
	Eudoxus \
	rnr \
	stdc++ \
	glut \
	GL \
	usb-1.0 \
	dl \
	m \
	pthread 

NiViewer.LIBDEPS = TinyXml JPEG OpenNI

# C++ Source Files
NiViewer.SRC.CPP = $(shell ls *.cpp) 


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"NiViewer\""

#EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk



ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
