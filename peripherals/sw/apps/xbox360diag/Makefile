################################################################################
#
# ./sw/apps/xbox360diag/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The xbox360diag application makefile.

The xbox360diag provide Xbox360 dignostics.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/apps/xbox360diag/Makefile}

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

RNMAKE_DIST_PGMS = xbox360diag

# Library dependencies
xbox360diag.LIBS = rnr_hid rnr stdc++
xbox360diag.LIBDEPS = rnr_hid

# C++ Source Files
xbox360diag.SRC.CXX = xbox360diag.cxx

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"xbox360diag\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
