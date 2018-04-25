################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Top-level rntools makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics Tools to Manage Projects, Packages, and the Business.

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2008-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

nodeps = 1

#------------------------------------------------------------------------------
# Tools

EXTRA_TGT_ALL_POST = all-post 

SCRIPTS = \
	cppstrings \
	rncp_to_repostaging \
	rnmake_all \
	rnmake_deb_all \
	rnmake_fastinstall \
	rnmake_lsarch \
	rnmake_versions \
	rnmon_dd \
	rnros_git_pull_all \
	rnros_git_status \
	rnsdk_ls_installed \
	rnupdate_copyright \
	rnupdate_eula \
	sgrep

ETC_FILES	=	

LIB_FILES = 

PKGDIR_SHARE := $(RNMAKE_PKG_ROOT)/share

# post 'all' make target
.PHONY: all-post
all-post: show-banner cp-script-files cp-etc-files cp-lib-files cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-script-files
cp-script-files:
	$(call printGoalDesc,$(@),Copying script files to $(DISTDIR_BIN))
	$(call copySrcDir,$(SCRIPTS),$(DISTDIR_BIN))

.PHONY: cp-etc-files
cp-etc-files:
	$(call printGoalDesc,$(@),Copying etc files to $(DISTDIR_ETC))
	$(call copySrcDirRel,$(ETC_FILES),$(DISTDIR_ETC))

.PHONY: cp-lib-files
cp-lib-files:
	$(call printGoalDesc,$(@),Copying extra lib files to $(DISTDIR_LIB))
	$(call copySrcDirRel,$(LIB_FILES),$(DISTDIR_LIB))

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share directory to $(DISTDIR_SHARE))
	@$(CP_R) $(PKGDIR_SHARE)/* $(DISTDIR_SHARE)
	@find $(DISTDIR_SHARE) -name '*.svn' -print | xargs rm -fr

.PHONY: uninstall
uninstall:
	@for s in $(SCRIPTS); \
	do \
		scr="$(bindir)/$${s}"; \
		if [ -e "$${scr}" ]; \
		then \
			$(RM) "$${scr}"; \
		fi; \
	done


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras (must include after top-level)
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
