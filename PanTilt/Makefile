################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_pan_tilt package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_pan_tilt Robotic Platform Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2015-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS		= sw


#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
#RNMAKE_DIST_HDRS	= pan_tilt

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
pan_tilt.HDRS.H	=	 $(shell cd include; ls PanTilt/*.h)


#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE		= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST = all-post

ETC_FILES	=	share/etc/pan_tilt/pan_tilt_tune.conf \
						share/etc/pan_tilt/*.xml \
						share/etc/profile.d/pan_tilt.sh \
						share/etc/init.d/* \
						share/etc/ld.so.conf.d/*.conf \
						share/etc/udev/rules.d/*.rules \
						etc/udev/rules.d/*

LIB_FILES =	share/lib/cmake/rnr/rnr-pantilt-config.cmake

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
