################################################################################
#
# ./sw/gst/rntemplate/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libgstrnnamefilter plug-in library makefile.

RoadNarrows GStreamer filter template plug-in.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/gst/rntemplate/Makefile}

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
RNMAKE_DIST_DLLIBS = gstrnnamefilter

# Target library subdirectory
gstrnnamefilter.SUBDIR = gst

# Source Files
gstrnnamefilter.SRC.C	=	gstrnnamefilter.c

# Libraries to link with
gstrnnamefilter.LIBS		= OpenNI TinyXml JPEG Eudoxus rnr stdc++
gstrnnamefilter.LIBDEPS = OpenNI TinyXml JPEG Eudoxus


#------------------------------------------------------------------------------
# Extras

EXTRA_INCDIR_GLIBLIB=$(RNMAKE_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gstonisrc\""

EXTRA_INCDIRS = \
	../include \
	$(RNMAKE_SYS_PREFIX)/include/gstreamer-1.0 \
	$(RNMAKE_SYS_PREFIX)/include/glib-2.0 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(RNMAKE_SYS_PREFIX)/include/libxml2


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
