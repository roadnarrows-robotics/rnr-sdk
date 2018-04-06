################################################################################
#
# ./sw/librnr_winqt/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_winqt library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_winqt/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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
RNMAKE_DIST_STLIBS = rnr_winqt

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_winqt

# Target library subdirectory
rnr_bpio.SUBDIR = rnr

# Source Files
rnr_winqt.SRC.CXX =

# Linked Libraries 
rnr_winqt.LIBS =	\
	opencv_core opencv_highgui opencv_imgproc \
	jpeg \
	rnr \
	pthread \
	stdc++


#------------------------------------------------------------------------------
# Extras Variables 

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_winqt\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
