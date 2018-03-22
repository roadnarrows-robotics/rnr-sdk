################################################################################
#
# Package: 	RN Make System
#
# File:			Rules.dpkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Make Debian packages for package.

Include this file into each local make file (usually at the bottom).
Only one library or program target is supported per make file, unlike the
standard Rules.mk file.\n

$LastChangedDate: 2014-07-18 10:10:13 -0600 (Fri, 18 Jul 2014) $
$Rev: 3726 $

\sa Rules.mk for more details of standard make targets.

\author Robin Knight 		(robin.knight@roadnarrows.com)
\author Daniel Packard 	(daniel@roadnarrows.com)

\par Copyright:
(C) 2009-2014.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
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
################################################################################

DEB_PREFIX   = /usr/local

DEB_CONF_DEV = $(RNMAKE_PKG_ROOT)/make/deb-dev
DEB_CONF_SRC = $(RNMAKE_PKG_ROOT)/make/deb-src
DEB_CONF_DOC = $(RNMAKE_PKG_ROOT)/make/deb-doc

DEB_DEV_PKG_NAME = $(PKG)-dev-$(PKG_VERSION_DOTTED)
DEB_SRC_PKG_NAME = $(PKG)-src-$(PKG_VERSION_DOTTED)
DEB_DOC_PKG_NAME = $(PKG)-doc-$(PKG_VERSION_DOTTED)

DEB_DISTDIR_TMP_DEV = $(DISTDIR_TMP_DEB)/$(DEB_DEV_PKG_NAME)
DEB_DISTDIR_TMP_SRC = $(DISTDIR_TMP_DEB)/$(DEB_SRC_PKG_NAME)
DEB_DISTDIR_TMP_DOC = $(DISTDIR_TMP_DEB)/$(DEB_DOC_PKG_NAME)

.PHONY: deb-pkgs
deb-pkgs: pkgbanner echo-deb-pkgs deb-pkg-dev deb-pkg-src deb-pkg-doc
	$(footer)

.PHONY: echo-deb-pkgs
echo-deb-pkgs:
	$(call fnEchoGoalDesc,Make all Debian packages)

.PHONY: deb-pkg-dev
deb-pkg-dev: pkgbanner all echo-deb-pkg-dev
	$(if $(call isdir, $(DEB_CONF_DEV)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_DEV) \
			-d $(DIST_ARCH) \
			-t $(DEB_DISTDIR_TMP_DEV) \
			-n $(DEB_DEV_PKG_NAME) \
			-p $(DEB_PREFIX) \
			-v $(PKG_VERSION_DOTTED) \
			-y pkgtype-dev \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_DEV). Skipping $(DEB_DEV_PKG_NAME)))
	$(footer)
	
.PHONY: echo-deb-pkg-dev
echo-deb-pkg-dev:
	$(call fnEchoGoalDesc,Making Debian development package)

.PHONY: deb-pkg-src
deb-pkg-src: pkgbanner tarball-src echo-deb-pkg-src
	$(if $(call isdir, $(DEB_CONF_SRC)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_SRC) \
			-d $(DIST_ARCH) \
			-t $(DEB_DISTDIR_TMP_SRC) \
			-n $(DEB_SRC_PKG_NAME) \
			-p $(DEB_PREFIX) \
			-v $(PKG_VERSION_DOTTED) \
			-y pkgtype-src \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_SRC). Skipping $(DEB_SRC_PKG_NAME)))
	$(footer)

.PHONY: echo-deb-pkg-src
echo-deb-pkg-src:
	$(call fnEchoGoalDesc,Making Debian source package)

.PHONY: deb-pkg-doc
deb-pkg-doc: pkgbanner documents echo-deb-pkg-doc
	$(if $(call isdir, $(DEB_CONF_DOC)),\
		$(shell $(RNMAKE_ROOT)/utils/dpkg-helper.sh \
			-a $(RNMAKE_ARCH) \
			-c $(DEB_CONF_DOC) \
			-d $(DIST_ARCH) \
			-t $(DEB_DISTDIR_TMP_DOC) \
			-n $(DEB_DOC_PKG_NAME) \
			-p $(DEB_PREFIX) \
			-v $(PKG_VERSION_DOTTED) \
			-y pkgtype-doc \
			1>&2 \
	 	),\
	$(info Debian conf directory not found: $(DEB_CONF_DOC). Skipping $(DEB_DOC_PKG_NAME)))
	$(footer)

.PHONY: echo-deb-pkg-doc
echo-deb-pkg-doc:
	$(call fnEchoGoalDesc,Making Debian documentaion package)


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
