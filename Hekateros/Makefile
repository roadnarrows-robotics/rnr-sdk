################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-03-10 14:19:39 -0700 (Thu, 10 Mar 2016) $
$Rev: 4348 $

\brief Make Hekateros package.

RN Make System Specific Makefile

\author: RoadNarrows LLC (support@roadnarrows.com)

\par Copyright:
	(C) 2010-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
	All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
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
# Subdirectories

SUBDIRS		= sw examples

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= hekateros

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
hekateros.HDRS.H	=	 $(shell cd include; ls Hekateros/*.h)


#------------------------------------------------------------------------------
# Documentation 

# Document SubPackage Name
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
DOXY_CONF_FILE		= $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Optional Variables 
#

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

#
# Overo Linaro versions of Hekateros
#
ifeq "$(arch)" "linaro"
	ETC_PROF_D_HEK = share/etc/profile.d/hekateros_linaro.sh 

	ETC_FILES	=	share/etc/profile.d/linaro.sh \
							share/etc/hekateros/hek_tune.conf \
							share/etc/hekateros/*.xml \
							share/etc/init.d/* \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES =	share/lib/cmake/rnr/rnr-hekateros-config.cmake
else

#
# Odroid Ubuntu versions of Hekateros
# 
ifeq "$(arch)" "odroid"
	ETC_PROF_D_HEK = share/etc/profile.d/hekateros_odroid.sh 

	ETC_FILES	=	share/etc/hekateros/hek_tune.conf \
							share/etc/hekateros/*.xml \
							share/etc/init.d/* \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-hekateros-config.cmake

#
# Off-target host computers
#
else
	ETC_FILES	=	share/etc/hekateros/*.xml \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-hekateros-config.cmake

endif
endif

PKGDIR_SHARE					:= $(pkgroot)/share

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
	$(call cp_template,$(ETC_PROF_D_HEK),$(DISTDIR_ETC)/profile.d/hekateros.sh)
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