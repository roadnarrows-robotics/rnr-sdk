################################################################################
#
# ./dxl_sdk-2.0/example/BulkRead/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Robotis Dynamixel SDK 2.0 bulk_read example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dxl_sdk-2.0/example/BulkRead/Makefile}

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Programs

RNMAKE_LOC_PGMS = bulk_read 

# Libraries to Link With
bulk_read.LIBS      = dxl2 stdc++
bulk_read.LIBDEPS   = dxl2

# C++ Source Files
bulk_read.SRC.CPP = main.cpp


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
