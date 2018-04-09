################################################################################
#
# ./sw/gst/pcsviewersink/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libpcsviewersink library makefile.

GStreamer Point Cloud Stream viewer sink plug-in libraries.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/gst/pcsviewersink/Makefile}

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
# Libraries

# Distribution Dynamic Linked Libraries
RNMAKE_DIST_DLLIBS = gstpcsviewersink

# Target library subdirectory
gstpcsviewersink.SUBDIR	=	gst

# Source Files
gstpcsviewersink.SRC.CXX	=	gstpcsviewersink.cxx gstpcsviewer.cxx

# Libraries to link with
gstpcsviewersink.LIBS = \
	Eudoxus \
	rnr \
	gstbase-1.0 \
	gobject-2.0 \
	glib-2.0 \
	pcl_common \
	pcl_io \
	pcl_visualization \
	stdc++

gstpcsviewersink.LIBDEPS = Eudoxus


#------------------------------------------------------------------------------
# Extras
#

EXTRA_INCDIR_GLIBLIB=$(RNMAKE_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gstpcsviewersink\""

EXTRA_INCDIRS = \
	../include

RNMAKE_PKG_SYS_INCDIRS += \
	$(RNMAKE_SYS_PREFIX)/include/gstreamer-1.0 \
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
