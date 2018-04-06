################################################################################
#
# Rules.dpkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Make Debian repo packages for a package.

This file is automatically included by \ref Rules.mk when one or more of the
Debian make goals are specified.

\pkgsynopsis
RN Make System

\pkgfile{Rules.dpkg.mk}

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

_RULES_DPKG_MK = 1

# Debian package install prefix
RNMAKE_DEB_PREFIX ?= /usr/local

# Debian configuration directories for development, source, and documentation
DEB_CONF_DEV = $(RNMAKE_PKG_ROOT)/make/deb-dev
DEB_CONF_SRC = $(RNMAKE_PKG_ROOT)/make/deb-src
DEB_CONF_DOC = $(RNMAKE_PKG_ROOT)/make/deb-doc

# Debian package file basenames for development, source, and documentation
DEB_PKG_DEV_NAME = $(RNMAKE_PKG)-dev-$(RNMAKE_PKG_VERSION_DOTTED)
DEB_PKG_SRC_NAME = $(RNMAKE_PKG)-src-$(RNMAKE_PKG_VERSION_DOTTED)
DEB_PKG_DOC_NAME = $(RNMAKE_PKG)-doc-$(RNMAKE_PKG_VERSION_DOTTED)

# temporary staging directories
DISTDIR_TMP_DEB	    = $(DIST_ARCH)/tmp/deb
DISTDIR_TMP_DEB_DEV = $(DISTDIR_TMP_DEB)/$(DEB_PKG_DEV_NAME)
DISTDIR_TMP_DEB_SRC = $(DISTDIR_TMP_DEB)/$(DEB_PKG_SRC_NAME)
DISTDIR_TMP_DEB_DOC = $(DISTDIR_TMP_DEB)/$(DEB_PKG_DOC_NAME)

# include tarball rules if not already included
ifeq ($(_RULES_TARBALL_MK),)
  $(eval include $(RNMAKE_ROOT)/Rules.tarball.mk)
endif

# include document rules if not already included
ifeq ($(_RULES_DOC_MK),)
  $(eval include $(RNMAKE_ROOT)/Rules.doc.mk)
endif

.PHONY: deb-pkgs
deb-pkgs: pkgbanner echo-deb-pkgs deb-pkg-dev deb-pkg-src deb-pkg-doc
	$(footer)

.PHONY: echo-deb-pkgs
echo-deb-pkgs:
	$(call printEchoTgtGoalDesc,Make all Debian packages)

.PHONY: deb-pkg-dev
deb-pkg-dev: pkgbanner all echo-deb-pkg-dev
	$(if $(call isDir, $(DEB_CONF_DEV)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_DEV) \
			-d $(DIST_ARCH) \
			-t $(DISTDIR_TMP_DEB_DEV) \
			-n $(DEB_PKG_DEV_NAME) \
			-p $(RNMAKE_DEB_PREFIX) \
			-v $(RNMAKE_PKG_VERSION_DOTTED) \
			-y pkgtype-dev \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_DEV). Skipping $(DEB_PKG_DEV_NAME)))
	$(footer)
	
.PHONY: echo-deb-pkg-dev
echo-deb-pkg-dev:
	$(call printEchoTgtGoalDesc,Making Debian development package)

.PHONY: deb-pkg-src
deb-pkg-src: pkgbanner tarball-src echo-deb-pkg-src
	$(if $(call isDir, $(DEB_CONF_SRC)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_SRC) \
			-d $(DIST_ARCH) \
			-t $(DISTDIR_TMP_DEB_SRC) \
			-n $(DEB_PKG_SRC_NAME) \
			-p $(RNMAKE_DEB_PREFIX) \
			-v $(RNMAKE_PKG_VERSION_DOTTED) \
			-y pkgtype-src \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_SRC). Skipping $(DEB_PKG_SRC_NAME)))
	$(footer)

.PHONY: echo-deb-pkg-src
echo-deb-pkg-src:
	$(call printEchoTgtGoalDesc,Making Debian source package)

.PHONY: deb-pkg-doc
deb-pkg-doc: pkgbanner documents echo-deb-pkg-doc
	$(if $(call isDir, $(DEB_CONF_DOC)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_DOC) \
			-d $(DIST_ARCH) \
			-t $(DISTDIR_TMP_DEB_DOC) \
			-n $(DEB_PKG_DOC_NAME) \
			-p $(RNMAKE_DEB_PREFIX) \
			-v $(RNMAKE_PKG_VERSION_DOTTED) \
			-y pkgtype-doc \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_DOC). Skipping $(DEB_PKG_DOC_NAME)))
	$(footer)

.PHONY: echo-deb-pkg-doc
echo-deb-pkg-doc:
	$(call printEchoTgtGoalDesc,Making Debian documentaion package)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
