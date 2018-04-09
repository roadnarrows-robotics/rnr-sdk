################################################################################
#
# ./sw/apps/laelaps_diag/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The laelaps_diag application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_laelaps Small Mobile Robot Package

\pkgfile{sw/apps/laelaps_diag/Makefile}

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

RNMAKE_DIST_PGMS = laelaps_diag 

# Libraries to Link With
laelaps_diag.LIBS = laelaps rnr_i2c rnr_gpio rnr_appkit rnr stdc++

laelaps_diag.LIBDEPS  = laelaps

# Source Files
laelaps_diag.SRC.CXX = \
	diagBatt.cxx \
	diagCam.cxx \
	diagCpu.cxx \
	diagImu.cxx \
	diagMotors.cxx \
	diagProd.cxx \
	diagToF.cxx \
	diagWatchDog.cxx \
	laelaps_diag.cxx \
	laelaps_utils.cxx


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"laelaps_diag\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
