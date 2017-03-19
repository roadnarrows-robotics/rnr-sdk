################################################################################
#
# ./bsModules/bsK3/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-06-14 16:08:57 -0600 (Tue, 14 Jun 2011) $ 
$Rev: 1000 $ 

\brief Make CogniBoost Message definitions 

RN Make System Specific Makefile

\author: Robin Knight   (robin.knight@roadnarrows.com)
\author: Daniel Packard (daniel@roadnarrows.com)

##! \par Copyright
##!   \h_copy 2011-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ..

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= CogniBoost

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
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"cb\""

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
NETMSGS_XML_FILES	= CogniBoostMsgs.xml
NETMSGS_H_DIR     = $(pkgroot)/include/CogniBoost
NETMSGS_C_DIR     = $(pkgroot)/sw/libCogniBoost
NETMSGS_CFLAGS		= --incprefix=CogniBoost
#NETMSGS_PY_DIR		= $(pkgroot)/bsPython/modules/BotSense/khepera3
NETMSGS_SHARE_DIR	= $(pkgroot)/share/msgs

# Include NetMsgs Rules Makefile
include $(rnmake)/Rules.netmsgs.mk

#SWIG_FILES 				= bsK3Core.i
#SWIG_EXTMOD_DIR 	= $(pkgroot)/bsPython/modules/BotSense/khepera3
#SWIG_EXTMOD_LIBS 	= -lbsclient_k3 -lbotsense -lnetmsgs -li2c -lrnr

# Include Swig Rules Makefile
#include $(rnmake)/Rules.swig.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-make-post
extra-make-post: 
#extra-make-post: swig-all

.PHONY: extra-deps
extra-deps: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean 
#extra-clean: netmsgs-clean swig-clean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
