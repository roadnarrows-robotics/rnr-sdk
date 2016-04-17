################################################################################
#
# ./Makefile
#
# Version:
# 	$LastChangedDate: 2010-01-21 16:05:01 -0700 (Thu, 21 Jan 2010) $
# 	$Rev: 233 $
#
# Description:
#  	RN Make System Specific Makefile
#
#   Make Fusion package.
#
# Author: Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2005-2010.  RoadNarrows LLC.
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
# Required

# Package Root Directory
pkgroot		= .

#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf


#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_TGT_ALL 				= all-python
EXTRA_TGT_DOC 				= doc-python
EXTRA_TGT_CLEAN 			= clean-python
EXTRA_TGT_DISTCLEAN 	= distclean-python

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

.PHONY: all-python
all-python:
	$(PYTHON) setup.py install --prefix=$(DIST_ARCH)

.PHONY: doc-python
doc-python:
	$(PYTHON) $(rnmake)/utils/pydocmk.py -d $(DISTDIR_DOC)/pydoc setup

.PHONY: clean-python
clean-python:
	@echo "removing compiled python files"
	@$(FIND) modules \( \
		-name '*.svn*' -or -wholename '.svn' -or \
		-wholename '*.deps*' -or -wholename '.deps' -or \
		-wholename '*obj*' -or -wholename 'obj' -or -wholename '*.o' -or \
		-wholename '*.out' -or -wholename '*.log' \
	\) -prune -or -print | \
 	$(GREP) -e '.py[co]' | \
	$(XARGS) $(RM) 

.PHONY: distclean-python
distclean-python:
	$(RM) $(pkgroot)/build
