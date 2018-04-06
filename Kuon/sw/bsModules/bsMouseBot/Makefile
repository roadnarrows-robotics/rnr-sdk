################################################################################
#
# ./sw/bsModules/bsMouseBot/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The BotSense bsProxy server dll and client library for the MouseBot
robot makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/bsModules/bsMouseBot/Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = bsclient_mousebot

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = bsclient_mousebot

# Dynamically Linked Libraries
RNMAKE_DIST_DLLIBS = bsserver_mousebot 

# Target library subdirectories 
bsclient_mousebot.SUBDIR = botsense
bsserver_mousebot.SUBDIR = botsense

# Libraries to link dll with
bsserver_mousebot.LIBS = qik

# Source Files
bsclient_mousebot.SRC.C =	bsMouseBotClient.c bsMouseBotMsgs.c
bsserver_mousebot.SRC.C =	bsMouseBotServer.c bsMouseBotMsgs.c


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = bsModMouseBot

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModMouseBot.HDRS.H =	botsense/bsMouseBot.h \
       			        		botsense/bsMouseBotMsgs.h


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsMouseBot\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST  = extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsMouseBotMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/MouseBot
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

SWIG_FILES 				= bsMouseBotCore.i
SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/MouseBot
SWIG_EXTMOD_LIBS 	= -lbsclient_mousebot -lbotsense -lnetmsgs -li2c -lrnr

# Include Swig Rules Makefile
#include $(RNMAKE_ROOT)/Rules.swig.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
#extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean 
#swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk

# Include netmsgs rules
include $(RNMAKE_ROOT)/Rules.netmsgs.mk

# Include swig rules
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
