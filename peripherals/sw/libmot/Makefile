################################################################################
#
# ./sw/libmot/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_mot library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/libmot/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Jessica Trujillo,jessica@roadnarrows.com}
\pkgauthor{Maurice Woods III,maurice@roadnarrows.com}

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
# Subdirectories

RNMAKE_SUBDIRS = ut

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_mot 

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_mot 

# Target library subdirectory
rnr_mot.SUBDIR = rnr

# Source Files
rnr_mot.SRC.CXX = MotRoboteqSmall.cxx
							#MotDummy.cxx

# Linked Libraries 
rnr_mot.LIBS	= rnr_serial rnr stdc++ usb-1.0

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_mot\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
