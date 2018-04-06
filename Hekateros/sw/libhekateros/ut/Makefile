################################################################################
#
# ./sw/libhekateros/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libhekateros library unit test makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/libhekateros/ut/Makefile}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

# Distribution Static Libraries
RNMAKE_TEST_PGMS = ut-libhekateros

# Linked Libraries 
ut-libhekateros.LIBS = \
	gtest hekateros \
	pthread stdc++

ut-libhekateros.LIBDEPS	= gtest hekateros

# C++ Source Files
ut-libhekateros.SRC.CXX = \
	ut-libhekateros-main.cxx \
	ut-hekrobot.cxx


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibhekateros\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
