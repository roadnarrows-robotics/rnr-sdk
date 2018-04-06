################################################################################
#
# ./libbserver_dyna/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libbsserver_dyna library makefile.

The BotSense bsProxy server dll Dynamixel library.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{libbserver_dynaX/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Dynamically Linked Libraries
RNMAKE_DIST_DLLIBS 	= bsserver_dyna 

# Target library subdirectory
bsserver_dyna.SUBDIR = botsense

# Libraries to link dll with
bsserver_dyna.LIBS	= \
	rnr_dynamixel bsclient_dyna botsense rnr_netmsgs rnr pthread stdc++

# Source Files
bsserver_dyna.SRC.C 	=	bsDynaMsgs.c
bsserver_dyna.SRC.CXX =	bsDynaServer.cxx


#------------------------------------------------------------------------------
# Extras

# install botsense library directory
EXTRA_LD_LIBDIRS = $(libdir)/botsense

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libbsserver_dyna\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
