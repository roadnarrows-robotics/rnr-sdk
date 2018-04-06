################################################################################
#
# ./sw/apps/arduinodiag/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The arduinodiag application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/arduinodiag/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2014-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_DIST_PGMS = arduinodiag 

# Libraries to Link With
arduinodiag.LIBS = rnr_serial rnr stdc++

arduinodiag.LIBDEPS = 

# C++ Source Files
arduinodiag.SRC.CXX = arduinodiag.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"arduinodiag\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
