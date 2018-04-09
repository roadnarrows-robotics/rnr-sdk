################################################################################
#
# ./sw/apps/laelaps_tof/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The laelaps_tof application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_laelaps Small Mobile Robot Package

\pkgfile{sw/apps/laelaps_tofX/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2015-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_DIST_PGMS = laelaps_tof 

# Libraries to Link With
laelaps_tof.LIBS = laelaps rnr_i2c rnr stdc++

laelaps_tof.LIBDEPS  = laelaps

# Source Files
laelaps_tof.SRC.CXX = laelaps_tof.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"laelaps_tof\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
