################################################################################
#
# ./sw/bsModules/bsKuon/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief BotSense \h_kuon bsProxy server dll and bs client library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/bsModules/bsKuon/Makefile}

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
RNMAKE_DIST_STLIBS = bsclient_kuon

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = bsclient_kuon

# Dynamically Linked Libraries
RNMAKE_DIST_DLLIBS = bsserver_kuon 

# Target libraries subdirectory
bsclient_kuon.SUBDIR = botsense
bsserver_kuon.SUBDIR = botsense

# Libraries to link dll with
bsserver_kuon.LIBS = RS160D MPU imu rnr_i2c rnr_serial rnr pthread

# Source Files
bsclient_kuon.SRC.C     =	bsKuonClient.c bsKuonMsgs.c
bsserver_kuon.SRC.CXX   =	bsKuonServer.cxx
bsserver_kuon.SRC.C   	=	bsKuonBgThread.c bsKuonMsgs.c


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= bsModKuon

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModKuon.HDRS.H	=	botsense/bsKuon.h \
       			        botsense/bsKuonMsgs.h


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsKuon\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST  = extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsKuonMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/Kuon
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

# Swig Variables
SWIG_FILES 				= bsKuonCore.i
SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/Kuon
SWIG_EXTMOD_LIBS 	= -lbsclient_kuon -lbotsense -lnetmsgs -li2c -lrnr

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

# include netmsgs rules
include $(RNMAKE_ROOT)/Rules.netmsgs.mk

# include swig rules
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
