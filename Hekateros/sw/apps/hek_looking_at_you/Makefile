################################################################################
#
# ./sw/apps/hek_looking_at_you/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The hek_looking_at_you application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/hek_looking_at_you/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

#RNMAKE_DIST_PGMS = hek_looking_at_you 

# Libraries to Link With
hek_looking_at_you.LIBS  = \
	opencv_core \
	opencv_highgui \
	opencv_imgproc \
	jpeg \
	rnrwin-gtk \
	rnr \
	stdc++

hek_looking_at_you.LIBDEPS   = Dynamixel

# C++ Source Files
hek_looking_at_you.SRC.CXX = ilay.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_looking_at_you\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
