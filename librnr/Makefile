################################################################################
#
# ./Makefile
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
$Rev: 3823 $

\brief RoadNarrows Robotics Common Library 1 Makefile.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \par Copyright
##!   \h_copy 2005-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= .


#------------------------------------------------------------------------------
# Libraries

SUBDIRS						= examples

# Distribution Static Libraries
DIST_STLIBS 			= rnr

# Distribution Shared Libraries
DIST_SHLIBS 			= rnr

# Source Files
rnr.SRC.C =	assoc.c \
						char.c \
						checksum.c \
						color.c \
						config.c \
						dlistvoid.c \
						hash.c \
						log.c \
						new.c \
						opts.c \
						path.c \
						shm.c \
						simplebuf.c\
						sock.c \
						sockset.c \
						units.c \
						uri.c \
						win.c

rnr.LIBS  = pthread rt


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= rnr

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(pkgroot)/include. Otherwise, is it really a deliverable header?
#
rnr.HDRS.H =	rnr/assoc.h \
							rnr/char.h \
							rnr/checksum.h \
							rnr/color.h \
							rnr/config.h \
							rnr/dlist.h \
							rnr/dliststr.h \
							rnr/dlistvoid.h \
							rnr/hash.h \
							rnr/install.h \
							rnr/log.h \
							rnr/new.h \
							rnr/opts.h \
							rnr/path.h \
							rnr/pkg.h \
							rnr/rnrconfig.h \
							rnr/shm.h \
							rnr/simplebuf.h \
							rnr/sock.h \
							rnr/sockset.h \
							rnr/units.h \
							rnr/uri.h

#------------------------------------------------------------------------------
# Documentation

# Document SubPackage Name
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr\""

# Uncomment to discover specific compiler warnings switches 
#EXTRA_CFLAGS = -fdiagnostics-show-option

EXTRA_TGT_ALL_POST = all-post 


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

ETC_FILES	=	share/etc/*.xml \
						share/etc/ld.so.conf.d/*.conf \
						etc/udev/rules.d/*

LIB_FILES = share/lib/cmake/rnr/rnr-sdk-config.cmake

PKGDIR_SHARE := $(pkgroot)/share

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
	$(call cp_template,$(ETC_PROF_D_LAE),$(DISTDIR_ETC)/profile.d/laelaps.sh)
	$(call copyfiles_template,$(ETC_FILES),$(DISTDIR_ETC))

.PHONY: lib-cp-files
lib-cp-files:
	@printf "\n"
	@printf "$(color_tgt_lib)Copying lib files to $(DISTDIR_LIB)$(color_end)\n"
	$(call copyfiles_template,$(LIB_FILES),$(DISTROOT_LIB))

.PHONY: share-cp-files
share-cp-files:
	@printf "\n"
	@printf "$(color_tgt_lib)Copying share directory to $(DISTDIR_SHARE)$(color_end)\n"
	@cp -rp $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr



ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
