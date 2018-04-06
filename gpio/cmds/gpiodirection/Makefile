################################################################################
#
# ./cmds/gpiodirection/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The gpiodirection command makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_gpio Package

\pkgfile{X/Makefile}

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

RNMAKE_DIST_PGMS = gpiodirection 

# Libraries to Link With
gpiodirection.LIBS = rnr_gpio rnr stdc++

gpiodirection.LIBDEPS = rnr_gpio

# C Source Files
gpiodirection.SRC.CXX = gpiodirection.cxx


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gpiodirection\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
