################################################################################
#
# ./sw/bsMouseBotControl/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Botsense MouseBot control application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/bsMouseBotControl/Makefile}

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

RNMAKE_DIST_PGMS = bsMouseBotControl 

# Libraries to Link With
bsMouseBotControl.LIBS = 	bsclient_mousebot \
													botsense \
													rnr_netmsgs \
													rnr_serial \
													rnr \
													RS160D \
													USBcontroller \
													usb-1.0

bsMouseBotControl.LIBDEPS = bsclient_mousebot \
													botsense \
													RS160D \
													USBcontroller

# C Source Files
bsMouseBotControl.SRC.C = bsMouseBotControl.c


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsMouseBotControl\""
EXTRA_LD_LIBDIRS = $(DISTDIR_LIB)/$(PKG_LIB_PLUGIN_SUBDIR)


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
