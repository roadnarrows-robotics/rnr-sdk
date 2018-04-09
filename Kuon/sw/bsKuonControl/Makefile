################################################################################
#
# ./sw/bsKuonControl/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief BotSense \h_kuon control application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/bsKuonControl/Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = bsKuonControl 

# Libraries to Link With
bsKuonControl.LIBS = 	bsclient_kuon \
											botsense \
											rnr_netmsgs \
											rnr \
											rnr_serial \
											RS160D \
											USBcontroller \
											usb-1.0

bsKuonControl.LIBDEPS = bsclient_kuon \
												RS160D \
												USBcontroller

# C Source Files
bsKuonControl.SRC.C = bsKuonControl.c


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsKuonControl\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
