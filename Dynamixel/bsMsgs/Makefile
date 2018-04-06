################################################################################
#
# ./bsMsgs/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Dynamixel BotSense messages makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{bsMsgs/Makefile}

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
# Extras

# install botsense library directory
EXTRA_LD_LIBDIRS = $(libdir)/botsense

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

# NetMsgs Variables
NETMSGS_XML_FILES	= bsDynaMsgs.xml
NETMSGS_H_DIR     = $(RNMAKE_PKG_ROOT)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(RNMAKE_PKG_ROOT)/bsPython/modules/BotSense/Dynamixel
NETMSGS_SHARE_DIR	= $(RNMAKE_PKG_ROOT)/share/msgs

SWIG_FILES 				= bsDynaCore.i
SWIG_EXTMOD_DIR 	= $(RNMAKE_PKG_ROOT)/bsPython/modules/BotSense/Dynamixel
SWIG_EXTMOD_LIBS 	= -lbotsense -lrnr_netmsgs -lrnr

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
include $(RNMAKE_ROOT)/Rules.netmsgs.mk

# include swig rules
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
