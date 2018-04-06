################################################################################
#
# ./cmds/gpioprobe/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The gpioprobe command makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_gpio Package

\pkgfile{cmds/gpioprobe/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = gpioprobe 

# Libraries to Link With
gpioprobe.LIBS = rnr_gpio rnr stdc++

gpioprobe.LIBDEPS = rnr_gpio

# C Source Files
gpioprobe.SRC.CXX = gpioprobe.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gpioprobe\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
