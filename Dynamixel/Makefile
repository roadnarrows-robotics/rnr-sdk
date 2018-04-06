################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_dynamixel package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS = \
	libdxl \
	bsMsgs \
	libbsclient_dyna \
	libDynamixel \
	libbsserver_dyna \
	dynashell \
	examples

#	dxl_sdk-2.0


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
DIST_DYN_HDRS	= Dynamixel
DIST_DXL_HDRS	= dxl
DIST_BS_HDRS  = botsense

RNMAKE_DIST_HDRS = $(DIST_DYN_HDRS) $(DIST_DXL_HDRS) $(DIST_BS_HDRS)

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
$(DIST_DYN_HDRS).HDRS.H	=	$(shell cd include; ls $(DIST_DYN_HDRS)/*.h)
$(DIST_DXL_HDRS).HDRS.H =	\
	$(shell cd include; ls $(DIST_DYN_HDRS)/$(DIST_DXL_HDRS)/*.h)
$(DIST_BS_HDRS).HDRS.H = botsense/bsDyna.h botsense/bsDynaMsgs.h


#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE		= $(RNMAKE_PKG_ROOT)/make/doxy.conf

#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST  = all-post

PKGDIR_SHARE = $(RNMAKE_PKG_ROOT)/share

# post 'all' make target
.PHONY: all-post
all-post: show-banner cp-etc-files cp-lib-files cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-etc-files
cp-etc-files:

.PHONY: cp-lib-files
cp-lib-files:

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share directory to $(DISTDIR_SHARE))
	@$(CP_R) $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
