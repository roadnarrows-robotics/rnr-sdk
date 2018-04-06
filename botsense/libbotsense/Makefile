################################################################################
#
# ./libbotsense/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libbotsense library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{libbotsense/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_DIST_STLIBS = botsense

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = botsense

# Target library subdirectory
botsense.SUBDIR = botsense

# Source Files
botsense.SRC.C =	bsProxyMsgs.c \
									bsLibClient.c \
									bsLibError.c \
									bsLibServer.c \
									bsLibUtils.c

# Linked Libraries
botsense.LIBS	= rnr_netmsgs rnr


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libbotsense\""

EXTRA_TGT_ALL 		= extra-make
EXTRA_TGT_DEPS 		= extra-make
EXTRA_TGT_CLEAN 	= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsProxyMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/bsPython/modules/BotSense
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include netmsgs rules
include $(rnmake)/Rules.netmsgs.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
