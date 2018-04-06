################################################################################
#
# ./CogniBoostMsgs/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The BotSense \h_cogniboost message definitions makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_cogniboost Package

\pkgfile{CogniBoostMsgs/Makefile}

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = CogniBoost

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
CogniBoost.HDRS.H	= CogniBoost/CogniBoostMsgs.h


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"cb\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= CogniBoostMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/CogniBoost
NETMSGS_C_DIR     = $(RNMAKE_PKG_ROOT)/sw/libCogniBoost
NETMSGS_CFLAGS		= --incprefix=CogniBoost
#NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/CogniBoost
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

#SWIG_FILES 				= bsCogniBoostCore.i
#SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/pyModules/modules/BotSense/CogniBoost
#SWIG_EXTMOD_LIBS 	= -lbsclient_cogniboost -lbotsense -lnetmsgs -li2c -lrnr

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all
	@touch $(DEPSFILE)

.PHONY: extra-clean
extra-clean: netmsgs-clean swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include netmsgs rules
include $(rnmake)/Rules.netmsgs.mk

# include swig rules
include $(rnmake)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
