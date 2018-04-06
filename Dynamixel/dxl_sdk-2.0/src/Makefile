################################################################################
#
# ./dxl_sdk-2.0/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Robotis Dynamixel SDK 2.0 library libdxl2 makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dxl_sdk-2.0/src/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = dxl2

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = dxl2

# Target library subdirectory
dxl2.SUBDIR = rnr

# Source Files
dxl2.SRC.CPP 	=	bulkread.cpp dynamixel.cpp serialport.cpp


#------------------------------------------------------------------------------
# Extras

EXTRA_INCDIRS = ../include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
