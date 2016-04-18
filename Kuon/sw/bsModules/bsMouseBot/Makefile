################################################################################
#
# ./bsModules/bsMouseBot/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-04-25 18:46:22 -0600 (Wed, 25 Apr 2012) $
$Rev: 1896 $

\brief
Make bsProxy server dll and bs client library for the MouseBot robot.

RN Make System Specific Makefile

\author: Robin Knight   (robin.knight@roadnarrows.com)
\author: Rob Shiely     (rob@roadnarrows.com)
\author: Daniel Packard (daniel@roadnarrows.com)

\par Copyright:
  (C) 2012.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../../

#------------------------------------------------------------------------------
# Libraries

LIB_SUBDIR = $(PKG_LIB_PLUGIN_SUBDIR)

# Distribution Static Libraries
DIST_STLIBS 			= bsclient_mousebot

# Distribution Shared Libraries
DIST_SHLIBS 			= bsclient_mousebot

# Dynamically Linked Libraries
DIST_DLLIBS 			= bsserver_mousebot 

# Libraries to link dll with
bsserver_mousebot.LIBS	= qik

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
DIST_HDRS	= bsModMouseBot

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModMouseBot.HDRS.H	=	botsense/bsMouseBot.h \
       			        		botsense/bsMouseBotMsgs.h

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsMouseBot\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST  = extra-make-post
EXTRA_TGT_DEPS 			= extra-deps
EXTRA_TGT_CLEAN 		= extra-clean

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

# NetMsgs Variables
NETMSGS_XML_FILES	= bsMouseBotMsgs.xml
NETMSGS_H_DIR     = $(pkgroot)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
#NETMSGS_PY_DIR		= $(pkgroot)/bsPython/modules/BotSense/khepera3
NETMSGS_SHARE_DIR	= $(pkgroot)/share/msgs

# Include NetMsgs Rules Makefile
include $(rnmake)/Rules.netmsgs.mk

#SWIG_FILES 				= bsMouseBotCore.i
#SWIG_EXTMOD_DIR 	= $(pkgroot)/bsPython/modules/BotSense/khepera3
#SWIG_EXTMOD_LIBS 	= -lbsclient_mousebot -lbotsense -lnetmsgs -li2c -lrnr

# Include Swig Rules Makefile
#include $(rnmake)/Rules.swig.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
#extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean 
#swig-clean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
