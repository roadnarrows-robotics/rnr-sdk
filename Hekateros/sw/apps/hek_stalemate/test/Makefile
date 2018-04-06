################################################################################
#
# ./sw/apps/hek_stalemate/test/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The hek_stalemate unit test makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/hek_stalemate/test/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_TEST_PGMS = utbotplanner 

# Libraries to Link With
utbotplanner.LIBS  = \
	opencv_core \
	opencv_highgui \
	opencv_imgproc \
	opencv_calib3d \
	jpeg \
	Dynamixel \
	rnrwin \
	rnr \
	stdc++

utbotplanner.LIBDEPS   = rnrwin

# Source Files
utbotplanner.SRC.CXX = utbotplanner.cxx ../SMBotPlanner.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTStaleMate\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
