################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Top-level \h_kuon package makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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

RNMAKE_SUBDIRS = sw 

# examples  


#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = kuon

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

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(RNMAKE_PKG_ROOT)/make/doxy.conf

#------------------------------------------------------------------------------
# Extras
#

EXTRA_TGT_ALL_POST = all-post 

# Overo Linaro versions of Kuon
ETC_PROF_D_KUON.linaro 	=	share/etc/profile.d/kuon_linaro.sh \
ETC_FILES.linaro 				= share/etc/profile.d/linary.sh \
													share/etc/init.d/*

# Odroid Ubuntu versions of Kuon
ETC_PROF_D_KUON.odroid 	=	share/etc/profile.d/kuon_odroid.sh \
ETC_FILES.odroid 				= share/etc/init.d/*

# profile.d source and destination shell script
ETC_PROF_D_KUON_SRC = $(ETC_PROF_D_KUON.$(RNMAKE_ARCH))
ETC_PROF_D_KUON_DST = $(DISTDIR_ETC)/profile.d/kuon.sh

# On-target Kuon and off-target host computers
ETC_FILES	=	share/etc/kuon/kuon.conf \
						share/etc/kuon/*.xml \
						share/etc/ld.so.conf.d/*.conf \
						etc/udev/rules.d/*

LIB_FILES = share/lib/cmake/rnr/rnr-kuon-config.cmake

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
		$(if $(ETC_PROF_D_KUON_SRC),\
		$(call copySrcDst,$(ETC_PROF_D_KUON_SRC),$(ETC_PROF_D_KUON_DST)),)

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

# include extras (must include after top-level)
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
