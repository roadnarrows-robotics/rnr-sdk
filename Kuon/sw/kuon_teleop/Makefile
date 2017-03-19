################################################################################
#
# ./examples/kuon_teleop/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-02-28 16:08:51 -0700 (Thu, 28 Feb 2013) $
$Rev: 2718 $

\brief Make the kuon_teleop application.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

##! \copyright
##!   \h_copy 2013-2017. RoadNarrows LLC.\n
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
pkgroot		= ../..

#------------------------------------------------------------------------------
# Programs

DIST_PGMS = kuon_teleop 


# Libraries to Link With
kuon_teleop.LIBS = \
	bsclient_kuon \
	botsense \
	rnr_netmsgs \
	rnr_serial \
	rnr_hid \
	RS160D \
	rnr \
	usb-1.0 \
	stdc++

bsKuonControl.LIBDEPS = \
	bsclient_kuon \
	RS160D

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
kuon_teleop.SRC.CXX = kuon_teleop.cxx

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"kuon_teleop\""

EXTRA_LD_LIBDIRS = $(DISTDIR_LIB)/$(PKG_LIB_PLUGIN_SUBDIR)


#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
