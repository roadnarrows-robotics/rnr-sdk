################################################################################
#
# ./libbotsense/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-12-06 13:19:13 -0700 (Thu, 06 Dec 2012) $
$Rev: 2568 $

\brief Make libbotsense library.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2009-2017. RoadNarrows LLC.\n
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
# Libraries

# Distribution Static Libraries
DIST_STLIBS 			= botsense

# Distribution Shared Libraries
DIST_SHLIBS 			= botsense

# Source Files
botsense.SRC.C 	=	bsProxyMsgs.c \
									bsLibClient.c \
									bsLibError.c \
									bsLibServer.c \
									bsLibUtils.c

# Linked Libraries
botsense.LIBS		= rnr_netmsgs rnr

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libbotsense\""

EXTRA_TGT_ALL 		= extra-make
EXTRA_TGT_DEPS 		= extra-make
EXTRA_TGT_CLEAN 	= extra-clean

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
NETMSGS_XML_FILES	= bsProxyMsgs.xml
NETMSGS_H_DIR     = $(pkgroot)/include/botsense
NETMSGS_CFLAGS		= --incprefix=botsense
NETMSGS_PY_DIR		= $(pkgroot)/bsPython/modules/BotSense
NETMSGS_SHARE_DIR	= $(pkgroot)/share/msgs

# Include NetMsgs Rules Makefile
include $(rnmake)/Rules.netmsgs.mk

.PHONY: extra-make
extra-make: netmsgs-all

.PHONY: extra-clean
extra-clean: netmsgs-clean


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
