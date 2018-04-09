################################################################################
#
# ./sw/apps/euStitch/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_eudoxus aeuStitch application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/apps/euStitch/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com}

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

RNMAKE_DIST_PGMS = euStitch

# Libraries to Link With
euStitch.LIBS = \
	Eudoxus \
	rnr \
	gstbase-1.0 \
	gobject-2.0 \
	glib-2.0 \
	pcl_common \
	pcl_io \
	pcl_filters \
	pcl_kdtree \
	pcl_search \
	pcl_visualization \
	boost_system \
	espeak \
	pthread \
	m \
	stdc++

euStitch.LIBDEPS   = Eudoxus

# Source Files
euStitch.SRC.CXX = euStitch.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"euStitch\"" \

EXTRA_INCDIR_GLIBLIB=$(RNMAKE_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

EXTRA_INCDIRS = \
	../include \
	$(RNMAKE_SYS_PREFIX)/include/gstreamer-1.0 \
	$(RNMAKE_SYS_PREFIX)/include/glib-2.0 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(RNMAKE_SYS_PREFIX)/include/libxml2 \
	$(RNMAKE_SYS_PREFIX)/include/eigen3 \
	$(RNMAKE_SYS_PREFIX)/include/pcl-1.7 \
	$(RNMAKE_SYS_PREFIX)/include/vtk-5.8


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
