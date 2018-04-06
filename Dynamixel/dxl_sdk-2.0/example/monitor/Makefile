################################################################################
#
# ./dxl_sdk-2.0/example/monitor/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Robotis Dynamixel SDK 2.0 monitor example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dxl_sdk-2.0/example/monitor/Makefile}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Programs

RNMAKE_LOC_PGMS = monitor 

# Libraries to Link With
monitor.LIBS      = dxl2 stdc++
monitor.LIBDEPS   = dxl2

# C++ Source Files
monitor.SRC.CPP = main.cpp


#------------------------------------------------------------------------------
# Extras

EXTRA_INCDIRS = ../../include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
