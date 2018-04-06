################################################################################
#
# ./sw/libhid/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_hid library unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/libhid/ut/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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

# Distribution Static Libraries
RNMAKE_TEST_PGMS = ut-libhid

# Linked Libraries 
ut-libhid.LIBS		  = gtest hid rnr pthread stdc++
ut-libhid.LIBDEPS		= gtest hid

# C++ Source Files
ut-libhid.SRC.CXX 	= ut-libhid-main.cxx \
											ut-keyboard.cxx \
											ut-HIDXbox360.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibhid\""

EXTRA_INCDIRS += /opt/include


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
