
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-11-20 15:52:58 -0700 (Fri, 20 Nov 2015) $
$Rev: 4213 $

\brief Make the Eudoxus package.

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

EXTRA_TGT_ALL_POST      = all-post
#RDK EXTRA_TGT_INSTALL       = install-extra


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

#
# Odroid Ubuntu versions of Laelaps
# 
ifeq "$(arch)" "odroid"
	ETC_PROF_D_EUD = share/etc/profile.d/eudoxus_odroid.sh 

	ETC_FILES	=	share/etc/init.d/* \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-eudoxus-config.cmake

#
# Off-target host computers
#
else
	ETC_FILES	= share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-eudoxus-config.cmake

endif

PKGDIR_SHARE        := $(pkgroot)/share

#
# Template to copy files from source to destination directory.
#
# Usage: copyfiles_template src_files dst_dir
#
define copyfiles_template
	@if [ "$(1)" != "" ]; \
	then \
		b=$(notdir $(2)); \
		flist="$(wildcard $(addprefix $(pkgroot)/,$(1)))"; \
		for f in $${flist}; \
		do \
			tgt=$(2)$${f##*$$b}; \
			if [ ! -f "$${tgt}" -o "$${f}" -nt "$${tgt}" ]; \
			then \
				d=$$(dirname $${tgt}); \
				test -d $$d || $(MKDIR) $$d; \
				printf "$(color_tgt_file) $${tgt}$(color_end)\n"; \
				$(CP) $$f $${tgt}; \
			fi; \
		done; \
	fi
endef

#
# Template to copy file from source to destination.
#
# Usage: cp_template src_file dst_file
#
define cp_template
	@if [ "$(1)" != "" ]; \
	then \
		if [ ! -f "$(2)" -o "$(1)" -nt "$(2)" ]; \
		then \
			d=$$(dirname $(2)); \
			test -d $$d || $(MKDIR) $$d; \
			printf "$(color_tgt_file) $(2)$(color_end)\n"; \
			$(CP) $(1) $(2); \
		fi; \
	fi
endef

# post 'all' make target
.PHONY: all-post
all-post: show-banner etc-cp-files lib-cp-files share-cp-files

.PHONY: show-banner
show-banner:
	$(call dirbanner,.)
	@printf "Copying Configuration Files\n"

.PHONY: etc-cp-files
etc-cp-files:
	@printf "\n"
	@printf "$(color_tgt_lib)Copying etc files to $(DISTDIR_ETC)$(color_end)\n"
	$(call cp_template,$(ETC_PROF_D_EUD),$(DISTDIR_ETC)/profile.d/eudoxus.sh)
	$(call copyfiles_template,$(ETC_FILES),$(DISTDIR_ETC))

.PHONY: lib-cp-files
lib-cp-files:
	@printf "\n"
	@printf "$(color_tgt_lib)Copying lib files to $(DISTDIR_LIB)$(color_end)\n"
	$(call copyfiles_template,$(LIB_FILES),$(DISTDIR_LIB))

.PHONY: share-cp-files
share-cp-files:
	@printf "\n"
	@printf "$(color_tgt_lib)Copying share directory to $(DISTDIR_SHARE)$(color_end)\n"
	@cp -rp $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
