################################################################################
#
# ./sw/libmot/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_mot library unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/libmot/ut/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
\pkgauthor{Jessica Trujillo,jessica@roadnarrows.com}

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
RNMAKE_TEST_PGMS = ut-libmot

# Linked Libraries 
ut-libmot.LIBS		  = gtest mot rnr pthread stdc++
ut-libmot.LIBDEPS		= mot

# C++ Source Files
ut-libmot.SRC.CXX 	= ut-libmot-main.cxx \
											ut-roboteq-small.cxx
											#ut-MotDummy.cxx

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibmot\""

EXTRA_INCDIRS += /opt/include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
