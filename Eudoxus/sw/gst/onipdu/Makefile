################################################################################
#
# ./sw/gst/onipdu/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libonipdu plug-in libraries makefile.

GStreamer OpenNI Protocol Data Unit encoder/decoder plug-in libraries.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/gst/onipdu/Makefile}

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
RNMAKE_DIST_DLLIBS = gstonipduenc gstonipdudec

# Target library subdirectory
gstonipduenc.SUBDIR	=	gst
gstonipdudec.SUBDIR	=	gst

# Source Files
gstonipduenc.SRC.CXX	=	gstonipduenc.cxx gstonipdu.cxx
gstonipdudec.SRC.CXX	=	gstonipdudec.cxx gstonipdu.cxx

# Libraries to link with
gstonipduenc.LIBS gstonipdudec.LIBS	= \
	OpenNI TinyXml JPEG Eudoxus rnr z gstbase-1.0 stdc++

gstonipduenc.LIBDEPS gstonipdudec.LIBDEPS = OpenNI TinyXml JPEG Eudoxus


#------------------------------------------------------------------------------
# Extras

EXTRA_INCDIR_GLIBLIB=$(RNMAKE_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gstonipdu\""

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
