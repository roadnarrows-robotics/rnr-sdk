################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_hek package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= .

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = sw examples


#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = hekateros

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

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE		= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST = all-post

# Overo Linaro versions of Hekateros
ETC_PROF_D_HEK.linaro = share/etc/profile.d/hekateros_linaro.sh 
ETC_FILES.linaro 			=	share/etc/profile.d/linaro.sh \
												share/etc/init.d/*

# Odroid Ubuntu versions of Hekateros
ETC_PROF_D_HEK.odroid = share/etc/profile.d/hekateros_odroid.sh 
ETC_FILES.odroid 			=	share/etc/init.d/*

# profile.d source and destination shell script
ETC_PROF_D_HEK_SRC = $(ETC_PROF_D_HEK.$(RNMAKE_ARCH))
ETC_PROF_D_HEK_DST = $(DISTDIR_ETC)/profile.d/hekateros.sh

# On-target and off-target common etc files
ETC_FILES	=	share/etc/hekateros/*.conf \
						share/etc/hekateros/*.xml \
						share/etc/ld.so.conf.d/*.conf \
						etc/udev/rules.d/*.rules

LIB_FILES = share/lib/cmake/rnr/rnr-hekateros-config.cmake

PKGDIR_SHARE = $(RNMAKE_PKG_ROOT)/share

.PHONY: all-post
all-post: show-banner cp-etc-files cp-lib-files cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-etc-files
cp-etc-files:
	$(call printGoalDesc,$(@),Copying etc files to $(DISTDIR_ETC))
	$(call copySrcDirRel,$(ETC_FILES) $(ETC_FILES.$(RNMAKE_ARCH)),$(DISTDIR_ETC))
	$(if $(ETC_PROF_D_HEK_SRC),\
		$(call copySrcDst,$(ETC_PROF_D_HEK_SRC),$(ETC_PROF_D_HEK_DST)),)

.PHONY: cp-lib-files
cp-lib-files:
	$(call printGoalDesc,$(@),Copying extra lib files to $(DISTDIR_LIB))
	$(call copySrcDirRel,$(LIB_FILES),$(DISTDIR_LIB))

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share directory to $(DISTDIR_SHARE))
	@$(CP_R) $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
