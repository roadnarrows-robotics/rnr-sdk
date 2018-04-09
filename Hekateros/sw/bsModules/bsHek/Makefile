################################################################################
#
# ./sw/bsModules/bsHek/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The BotSense bsHek modules makefile.

The BotSense bsProxy server dll and bs client library for the Hekateros robot.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/bsModules/bsHek/Makefile}

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

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = bsclient_hek

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = bsclient_hek

# Dynamically Linked Libraries
RNMAKE_DIST_DLLIBS = bsserver_hek 

# Target library subdirectory
bsclient_hek.SUBDIR = botsense
bsserver_hek.SUBDIR = botsense

# Libraries to link dll with
bsclient_hek.LIBS	= rnr_dynamixel botsense rnr_netmsgs rnr pthread stdc++
bsserver_hek.LIBS	= \
	hekateros bsclient_hek rnr_dynamixel botsense rnr_netmsgs rnr pthread stdc++

# Source Files
bsclient_hek.SRC.C 	 =	bsHekMsgs.c
bsclient_hek.SRC.CXX =	bsHekClient.cxx
bsserver_hek.SRC.C 	 =	bsHekMsgs.c
bsserver_hek.SRC.CXX =	bsHekServer.cxx


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= bsModHek

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModHek.HDRS.H=	\
	botsense/bsHek.h \
	botsense/bsHekMsgs.h


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsHek\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsHekMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/sw/pyModules/modules/Hekateros
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

# Swig Variables
SWIG_FILES 				= bsHekCore.i
SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/sw/pyModules/modules/Hekateros
SWIG_EXTMOD_LIBS 	= -lbsclient_hek -lbotsense -lrnr_netmsgs -lrnr_i2c -lrnr

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean swig-clean
	

#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk

# include netmsgs rules
include $(RNMAKE_ROOT)/Rules.netmsgs.mk

# include swig rules
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
