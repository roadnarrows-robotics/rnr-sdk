
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-11-20 15:52:58 -0700 (Fri, 20 Nov 2015) $
$Rev: 4213 $

\brief Make Eudoxus package.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

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
# Optional variables

# do [not] include botsense subpackage in package
ifeq "$(findstring $(has_botsense),n no 0 false)" ""
	has_botsense := yes
else
	has_botsense := no
endif

#------------------------------------------------------------------------------
# Subdirectories

SUBDIRS		= sw examples

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
#DIST_HDRS	= ni primesense eudoxus

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
#ni.HDRS.H	=	$(shell cd include; ls ni/*.h ni/Linux-*/*.h) 

#primesense.HDRS.H	=	$(shell cd include; ls primesense/*.h primesense/XnEE/*.h) 
#eudoxus.HDRS.H	=	$(shell cd include; ls Eudoxus/*.h) 

#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR				= sw examples

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_TGT_ALL_POST      = all-extra-post
EXTRA_TGT_INSTALL       = install-extra


#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extras

PKGDIR_SHARE        := $(pkgroot)/share
PKGDIR_SHARE_MSGS   := $(pkgroot)/share/msgs
DISTDIR_SHARE_MSGS  := $(DISTDIR_SHARE)/msgs
PKG_SHARE_FILES     := $(wildcard $(PKGDIR_SHARE)/*.xml) \
                       $(wildcard $(PKGDIR_SHARE_MSGS)/*.xml)
DIST_SHARE_FILES    := $(patsubst $(PKGDIR_SHARE)%,$(DISTDIR_SHARE)%,\
                       $(PKG_SHARE_FILES))

# Target: all-post
# Make share and etc files
.PHONY: all-extra-post
all-extra-post:
	@$(rnmake)/utils/doinstall.sh -s 664 $(pkgroot)/share $(realpath $(pkgroot))/$(DISTDIR_SHARE)
	@$(rnmake)/utils/doinstall.sh -s 664 $(pkgroot)/etc $(realpath $(pkgroot))/$(DISTDIR_ETC)

#
# Install share files from distribution
.PHONY: install-extra
install-extra:
	@echo ""
	@echo "Installing system share files to $(sharedir)"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)
	@echo "Installing system etc files to $(sysconfdir)"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_ETC) $(sysconfdir)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
