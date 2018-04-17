################################################################################
#
# ./bsModules/bsSerial/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief \h_botsense serial modules makefile.

\h_botsense bsProxy server dll and bs client library for raw RS-232 serial
devices.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{bsModules/bsSerial/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = bsclient_serial

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = bsclient_serial

# Dynamically Linked Libraries
RNMAKE_DIST_DLLIBS = bsserver_serial 

# Target library subdirectory
bsclient_serial.SUBDIR = botsense/plugins
bsserver_serial.SUBDIR = botsense/plugins

# Source Files
bsclient_serial.SRC.C	=	bsSerialClient.c bsSerialMsgs.c
bsserver_serial.SRC.C	=	bsSerialServer.c bsSerialMsgs.c

bsclient_serial.LIBS 	=	rnr_netmsgs rnr
bsserver_serial.LIBS 	=	rnr_serial rnr_netmsgs rnr


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = bsModSerial

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModSerial.HDRS.H =	botsense/bsSerial.h \
											botsense/bsSerialMsgs.h


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsSerial\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsSerialMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/bsPython/modules/BotSense
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

SWIG_FILES 				= bsSerial.i
SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/bsPython/modules/BotSense
SWIG_EXTMOD_LIBS 	= -lbsclient_serial -lbotsense -lrnr_netmsgs -lrnr

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include netmsgs rules
include $(RNMAKE_ROOT)/Rules.netmsgs.mk

# include swig rules
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
