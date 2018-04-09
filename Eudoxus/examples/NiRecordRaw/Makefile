################################################################################
#
# ./examples/NiRecordRaw/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The NiRecordRaw example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{examples/NiRecordRaw/Makefile}

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

RNMAKE_DIST_PGMS = NiRecordRaw 

# Libraries to Link With
NiRecordRaw.LIBS = \
	OpenNI \
	TinyXml \
	Eudoxus \
	rnr \
	stdc++ \
	usb-1.0 \
	dl \
	m \
	pthread 

NiRecordRaw.LIBDEPS = TinyXml OpenNI

# Source Files
NiRecordRaw.SRC.CPP = NiRecordRaw.cpp


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"NiRecordRaw\""

EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
