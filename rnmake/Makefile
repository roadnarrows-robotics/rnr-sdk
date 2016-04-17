################################################################################
#
# libserial/Makefile
#
# $LastChangedDate$
# $Rev$
#
# Description:
#  	RN Make System Specific Makefile
# 
#   Make rnmake system documentation and tarballs. Nothing else to make.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2009.  RoadNarrows LLC.
# All Rights Reserved
#
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
# Environment

# REQUIRED Package Root Directory
export pkgroot   = .

# Default RoadNarrows make system base directory
ifndef rnmake
export rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Define to not make c/c++ dependencies.
export nodeps = 1

# Sub[Package] Doxygen Configuration File and Styles
export DOXY_CONF_FILE   = $(pkgroot)/make/doxy.conf

# Include package specificiation
include $(pkgroot)/make/Pkg.mk

#------------------------------------------------------------------------------
# Overide Rules.mk Targets

# Simple make distro directory structure
.PHONY: all
all:
	$(call rnbanner)
	@$(MAKE) -s -f $(rnmake)/Rules.mk mkdistdirs
	$(call rnfooter)

# Only install the documentation
.PHONY: install
install:
	$(call rnbanner)
	@$(MAKE) -s -f $(rnmake)/Rules.mk rel
	@$(MAKE) -s -f $(rnmake)/Rules.mk install-docs
	$(call rnfooter)

# Make documentation
.PHONY: documents
documents:
	$(call rnbanner)
	@$(MAKE) -s -f $(rnmake)/Rules.mk docs-src-gen
	$(call rnfooter)

# Only make the documentation and source tarballs
.PHONY: tarballs
tarballs:
	$(call rnbanner)
	@$(MAKE) -s -f $(rnmake)/Rules.mk tarball-doc
	@$(MAKE) -s -f $(rnmake)/Rules.mk tarball-src
	$(call rnfooter)

# Documents tarball
.PHONY: tarball-doc
tarball-doc:
	@$(MAKE) -s -f $(rnmake)/Rules.mk tarball-doc

# Source tarball
.PHONY: tarball-src
tarball-src:
	@$(MAKE) -s -f $(rnmake)/Rules.mk tarball-src

clean distclean clobber:
	$(call rnbanner)
	@$(MAKE) -s -f $(rnmake)/Rules.mk $(@)
	$(call rnfooter)

# Pretty Print Major Banner
# rnbanner <target>
define rnbanner
	@if [ "$(MAKELEVEL)" = "0" ]; then \
	echo "============================================";\
	echo "Package:       $(PKG_FULL_NAME)";\
	echo "Package Root:  $(rnmake)";\
	echo "Directory:     $(CURDIR)";\
	echo "Target:        $(1)";\
	echo "Start:         `date`";\
	echo "============================================";\
	fi
endef

# Pretty Print Footer
define rnfooter
	@if [ "$(MAKELEVEL)" = "0" ]; then \
	echo "";\
	echo "            ###";\
	echo "Finished: `date`";\
	echo "            ###";\
	fi
endef
