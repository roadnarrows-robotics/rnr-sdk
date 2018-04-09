################################################################################
#
# ./sw/openni/Utils/niLicense/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The OpenNI licensing utility application makefile.

DEPRECATED

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/openni/Utils/niLicense/Makefile}

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

RNMAKE_DIST_PGMS = niLicense 

# Libraries to Link With
niLicense.LIBS =	\
	EuOpenNI \
	TinyXml \
	stdc++ \
	usb-1.0 \
	dl \
	m \
	pthread 

niLicense.LIBDEPS = TinyXml EuOpenNI

# Source Files
niLicense.SRC.CPP = niLicense.cpp


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"niLicense\""

EXTRA_INCDIRS = ../..


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
