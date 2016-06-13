################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-12-06 13:19:13 -0700 (Thu, 06 Dec 2012) $
$Rev: 2568 $

\brief Make BotSense package.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2007-2016.  RoadNarrows LLC.
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

# Subdirectories to Build
SUBDIRS = libbotsense bsProxy bsModules bsPython examples

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= botsense libbotsense

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
botsense.HDRS.H	=	botsense/BotSense.h \
									botsense/libBotSense.h \
									botsense/bsProxyModIF.h

libbotsense.HDRS.H	=	botsense/bsProxyMsgs.h

#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_TGT_DOC       		= doc-python
EXTRA_TGT_ALL_POST			= all-extra
EXTRA_TGT_CLEAN     		= clean-python
EXTRA_TGT_DISTCLEAN 		= distclean-python

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

DISTDIR_SHARE_MSGS 	= $(DISTDIR_SHARE)/msgs
PKGDIR_SHARE				= $(pkgroot)/share
PKGDIR_SHARE_MSGS		= $(pkgroot)/share/msgs
PKG_SHARE_FILES 		= $(PKGDIR_SHARE)/botsense-conf.xml \
											$(PKGDIR_SHARE)/botsense.dtd \
											$(wildcard $(PKGDIR_SHARE_MSGS)/*.xml)
DIST_SHARE_FILES = $(patsubst $(PKGDIR_SHARE)%,$(DISTDIR_SHARE)%,$(PKG_SHARE_FILES))
 
# Target: all-extra
# Install botsense configuration and message definition files
all-extra: share-mkdirs $(DIST_SHARE_FILES)

# Configuration xml rule 
$(DISTDIR_SHARE)/%.xml : $(pkgroot)/share/%.xml
	@echo ""
	@echo "    $(<)"
	$(CP) $(<) $(@)

# Configuration document type definition rule 
$(DISTDIR_SHARE)/%.dtd : $(pkgroot)/share/%.dtd
	@echo ""
	@echo "    $(<)"
	$(CP) $(<) $(@)

#  Message xml definitions rule 
$(DISTDIR_SHARE_MSGS)/%.xml : $(pkgroot)/share/msgs/%.xml
	@echo ""
	@echo "    $(<)"
	$(CP) $(<) $(@)

.PHONY: share-mkdirs
share-mkdirs:
	@test -d $(DISTDIR_SHARE_MSGS) || $(MKDIR) $(DISTDIR_SHARE_MSGS)

.PHONY: doc-python clean-python distclean-python
doc-python clean-python distclean-python:
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C bsPython $(@)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
