################################################################################
#
# ./sw/kuon_teleop/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief \h_kuon kuon_teleop application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/kuon_teleop/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = kuon_teleop 

# Libraries to Link With
kuon_teleop.LIBS = \
	bsclient_kuon \
	botsense \
	rnr_netmsgs \
	rnr_serial \
	rnr_hid \
	RS160D \
	rnr \
	usb-1.0 \
	stdc++

bsKuonControl.LIBDEPS = \
	bsclient_kuon \
	RS160D

# C Source Files
kuon_teleop.SRC.CXX = kuon_teleop.cxx


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"kuon_teleop\""

EXTRA_LD_LIBDIRS = $(DISTDIR_LIB)/$(PKG_LIB_PLUGIN_SUBDIR)


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
