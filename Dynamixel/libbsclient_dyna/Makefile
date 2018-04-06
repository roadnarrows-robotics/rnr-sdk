################################################################################
#
# ./libbsclient_dyna/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libbsclient_dyna library makefile.

The BotSense client interface between the Dynamixels and the bsProxy.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{libbsclient_dyna/Makefile}

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

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = bsclient_dyna

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = bsclient_dyna

# Target library subdirectory
bsclient_dyna.SUBDIR = botsense

# Libraries to link dll with
bsclient_dyna.LIBS	= rnr_netmsgs rnr pthread stdc++

# Source Files
bsclient_dyna.SRC.C 	=	bsDynaMsgs.c
bsclient_dyna.SRC.CXX =	bsDynaClient.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libbsclient_dyna\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
