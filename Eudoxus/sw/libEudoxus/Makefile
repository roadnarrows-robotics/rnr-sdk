################################################################################
#
# ./sw/libEudoxus/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libEudoxus library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{sw/libEudoxus/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS 	= ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = Eudoxus

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = Eudoxus

# Source Files
Eudoxus.SRC.CXX	=	\
	euClientUdp.cxx \
	euEnv.cxx \
	euError.cxx \
	euIo.cxx \
	euOni.cxx \
	euPcd.cxx \
	euPcs.cxx \
	euUtils.cxx


# Libraries to link with
Eudoxus.LIBS = \
	rnr \
	TinyXml \
	JPEG \
  $(PCL_LIBS_BASE) \
  $(ONI_LIBS) \
	boost_system \
	$(GST_LIBS_BASE) \
	$(GST_LIBS_VID) \
	$(GLIB_LIBS_BASE) \
	m \
	stdc++

#	gstinterfaces-1.0 \
Eudoxus.LIBDEPS = #OpenNi


#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libEudoxus\""

#PKG_SYS_INCDIRS += \
#	$(PKG_SYS_PREFIX)/include/libxml2 \
#	$(PKG_SYS_PREFIX)/include/eigen3 \
#	$(PKG_SYS_PREFIX)/include/vtk-5.8


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
