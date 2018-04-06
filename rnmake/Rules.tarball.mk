################################################################################
#
# Rules.tarball.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Make Debian repo packages for a package.

This file is automatically included by \ref Rules.mk when one or more of the
Debian make goals are specified.

\pkgsynopsis
RN Make System

\pkgfile{Rules.tarball.mk}

\pkgauthor{Daniel Packard,daniel@roadnarrows.com}
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_RULES_TARBALL_MK = 1

# binary tarball stem (basename without any extensions)
TARBALL_BIN_STEM = $(RNMAKE_PKG_FULL_NAME)-$(RNMAKE_ARCH)

# tarball file basenames for source, documentation, and binary
TARBALL_SRC_NAME = $(RNMAKE_PKG_FULL_NAME)-src.tar.gz
TARBALL_DOC_NAME = $(RNMAKE_PKG_FULL_NAME)-doc.tar.gz
TARBALL_BIN_NAME = $(TARBALL_BIN_STEM)-$(RNMAKE_ARCH).tar.gz

# binary tarball temporary staging directory
DISTDIR_TMP_TARBALL_BIN = $(DISTDIR_TMP)/$(TARBALL_BIN_STEM)


# make all tarball archives
.PHONY: tarballs
tarballs: pkgbanner tarball-bin tarball-doc tarball-src
	$(footer)

# make documentation tarball archive
.PHONY: tarball-doc
tarball-doc: 
	$(printTgtGoal)
	$(if $(call isDir,$(DISTDIR_DOC)),,\
			    					$(error No documentation - Try 'make documents' first.))
	@cd $(DIST_ARCH)/doc; \
	$(TAR) $(DISTDIR_REPO)/$(TARBALL_DOC_NAME) $(RNMAKE_PKG_FULL_NAME)-doc
	$(footer)

# make source tarball archive
DISTDIR_DOC     = $(DIST_ARCH)/doc/$(RNMAKE_PKG_FULL_NAME)-doc
.PHONY: tarball-src
tarball-src: 
	$(printTgtGoal)
	@$(RM) $(DISTDIR_SRC)
	@test -d $(DISTDIR_SRC) || $(MKDIR) $(DISTDIR_SRC)
	@$(RNMAKE_ROOT)/utils/tarball-src-filter.sh $(RNMAKE_PKG_ROOT) | \
	while read src; \
	do \
		$(RNMAKE_ROOT)/utils/cppath.sh $$src $(DISTDIR_SRC); \
	done;
	@cd $(DIST_ARCH)/src; \
	$(TAR) $(DISTDIR_REPO)/$(TARBALL_SRC_NAME) $(RNMAKE_PKG_FULL_NAME)
	$(footer)

# make binary tarball archive
.PHONY: tarball-bin
tarball-bin:
	$(printTgtGoal)
	$(if $(call isDir,$(DIST_ARCH)),,$(error Nothing made - Try 'make' first.))
	@test -d $(DISTDIR_TMP_TARBALL_BIN) || $(MKDIR) $(DISTDIR_TMP_TARBALL_BIN)
	@cd $(DIST_ARCH); \
	$(FIND) bin lib include etc share -print | \
	while read src; \
	do \
		if [ -f $$src ]; \
		then \
			$(RNMAKE_ROOT)/utils/cppath.sh $$src $(DISTDIR_TMP_TARBALL_BIN); \
		fi; \
	done;
	@cd $(DIST_ARCH)/tmp; \
	$(TAR) $(DISTDIR_REPO)/$(TARBALL_BIN_NAME) $(TARBALL_BIN_STEM)
	$(footer)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
