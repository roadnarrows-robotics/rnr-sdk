################################################################################
#
# ./sw/libEudoxus/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libEudoxus library unit test makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/libEudoxus/ut/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_TEST_PGMS = utEudoxus

utEudoxus.LIBS  			= Eudoxus rnr boost_system stdc++
utEudoxus.LIBDEPS  		= Eudoxus

# C++ Source Files
utEudoxus.SRC.CXX 	 	= utEudoxus.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibEudoxus\""

#RNMAKE_PKG_SYS_INCDIRS += \
	$(RNMAKE_SYS_PREFIX)/include/gstreamer-0.10 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(RNMAKE_SYS_PREFIX)/include/glib-2.0 \
	$(RNMAKE_SYS_PREFIX)/include/libxml2 \
	$(RNMAKE_SYS_PREFIX)/include/eigen3 \
	$(RNMAKE_SYS_PREFIX)/include/vtk-5.8


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
