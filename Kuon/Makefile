################################################################################
#
# ./Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-05-31 17:31:11 -0600 (Sun, 31 May 2015) $
$Rev: 4004 $

\brief Make Kuon package.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)
\author: Rob Shiely (rob@roadnarrows.com)

\par Copyright:
	(C) 2010-2016.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
	All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= .

#------------------------------------------------------------------------------
# Subdirectories

#SUBDIRS		= fw sw
SUBDIRS		= sw 

# examples  

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
DIST_HDRS	= kuon

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
kuon.HDRS.H	=	\
  Kuon/kuon.h \
	Kuon/kuonDesc.h \
	Kuon/kuonDescBase.h \
	Kuon/kuonJoint.h \
	Kuon/kuonProdBase.h \
	Kuon/kuonProdBaseStd.h \
	Kuon/kuonRobot.h \
	Kuon/kuonSpec.h \
	Kuon/kuonStatus.h \
	Kuon/kuonTraj.h \
	Kuon/kuonUtils.h \
	Kuon/kuonXmlCfg.h \
  Kuon/RS160DControl.h

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

#
# Overo Linaro versions of Kuon
#
ifeq "$(arch)" "linaro"
	ETC_PROF_D_KUON = share/etc/profile.d/kuon_linaro.sh 

	ETC_FILES	=	share/etc/profile.d/linaro.sh \
							share/etc/kuon/kuon_tune.conf \
							share/etc/kuon/*.xml \
							share/etc/init.d/* \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES =	share/lib/cmake/rnr/rnr-kuon-config.cmake
else

#
# Odroid Ubuntu versions of Kuon
# 
ifeq "$(arch)" "odroid"
	ETC_PROF_D_KUON = share/etc/profile.d/kuon_odroid.sh 

	ETC_FILES	=	share/etc/kuon/kuon_tune.conf \
							share/etc/kuon/*.xml \
							share/etc/init.d/* \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-kuon-config.cmake

#
# Off-target host computers
#
else
	ETC_FILES	=	share/etc/kuon/*.xml \
							share/etc/ld.so.conf.d/*.conf \
							etc/udev/rules.d/*

	LIB_FILES = share/lib/cmake/rnr/rnr-kuon-config.cmake

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
	$(call cp_template,$(ETC_PROF_D_KUON),$(DISTDIR_ETC)/profile.d/kuon.sh)
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
