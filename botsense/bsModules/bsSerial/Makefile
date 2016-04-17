################################################################################
#
# ./bsModules/bsSerial/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2010-09-22 18:08:19 -0600 (Wed, 22 Sep 2010) $
$Rev: 595 $

\brief
Make bsProxy server dll and bs client library for raw RS-232 serial devices.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2010.  RoadNarrows LLC.
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
pkgroot		= ../..

#------------------------------------------------------------------------------
# Libraries

# push libraries to botsense subdirectory
LIB_SUBDIR = $(PKG_LIB_PLUGIN_SUBDIR)

# Distribution Static Libraries
DIST_STLIBS 			= bsclient_serial

# Distribution Shared Libraries
DIST_SHLIBS 			= bsclient_serial

# Dynamically Linked Libraries
DIST_DLLIBS 			= bsserver_serial 

# Source Files
bsclient_serial.SRC.C 	=	bsSerialClient.c bsSerialMsgs.c
bsserver_serial.SRC.C 	=	bsSerialServer.c bsSerialMsgs.c

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= bsModSerial

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
bsModSerial.HDRS.H	=	botsense/bsSerial.h \
											botsense/bsSerialMsgs.h

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsSerial\""

EXTRA_TGT_ALL 			= extra-make
EXTRA_TGT_ALL_POST	= extra-make-post
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
NETMSGS_XML_FILES	= bsSerialMsgs.xml
NETMSGS_H_DIR     = $(pkgroot)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(pkgroot)/bsPython/modules/BotSense
NETMSGS_SHARE_DIR	= $(pkgroot)/share/msgs

# Include NetMsgs Rules Makefile
include $(rnmake)/Rules.netmsgs.mk

SWIG_FILES 				= bsSerial.i
SWIG_EXTMOD_DIR 	= $(pkgroot)/bsPython/modules/BotSense
SWIG_EXTMOD_LIBS 	= -lbsclient_serial -lbotsense -lnetmsgs -lrnr

# Include Swig Rules Makefile
include $(rnmake)/Rules.swig.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean swig-clean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
