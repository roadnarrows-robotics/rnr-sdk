################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2010-03-25 08:03:07 -0600 (Thu, 25 Mar 2010) $
$Rev: 309 $

\brief Make RoadNarrows peripherals package.

RN Make System Specific Makefile

\author: Robin Knight   (robin.knight@roadnarrows.com)
\author: Daniel Packard (daniel@roadnarrows.com)
\author: Rob Shiely     (rob@roadnarrows.com)

\par Copyright:
  (C) 2012-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

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
pkgroot		= .


#------------------------------------------------------------------------------
# Subdirectories

#
# Subdirectories to Build
#
SUBDIRS = sw

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= peripherals

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
peripherals.HDRS.H = \
	rnr/usbext.h \
	rnr/hid/HID.h \
	rnr/hid/HIDXbox360.h \
	rnr/imu/yost.h\
  rnr/mot/Mot.h\
	rnr/mot/MotRoboteq.h\
	rnr/mot/MotRoboteqSmall.h

#	econ/econ32.h \
	econ/econ50.h \
	econ/econ_error.h \
	econ/econ_fn_res_prototype.h \
	econ/econ_typedef.h \

#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_TGT_ALL_POST = all-post

#
# Document SubPackage Name
# 
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf


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
# Extras

PKGDIR_ETC					:= $(pkgroot)/etc

# Post make all target
.PHONY: all-post
all-post: etc-cp

# Copy etc files to dist architures. Somewhat of a hack. The copy preserves
# links, but also copies .svn files. So clean up after copy is required.
.PHONY: etc-cp
etc-cp:
	@printf "\n"
	@printf "$(color_tgt_file)Copying etc files to dist.$(color_end)\n"
	@cp -rp $(PKGDIR_ETC) $(DIST_ARCH)
	@find $(DIST_ARCH)/etc -name '*.svn' -print | xargs rm -fr


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
