################################################################################
#
# Makefile
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Make the RoadNarrows Make System. Cool.

Make the RN Make System documentation and repo packages. Nothing else to make.

\pkgsynopsis
(Meta) RN Make System

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

#------------------------------------------------------------------------------
# Prelims

# this makefile is last, get the directory part for rnmake root
RNMAKE_ROOT = $(realpath .)

# package root
export RNMAKE_PKG_ROOT = $(RNMAKE_ROOT)

# Define to not require c/c++ dependencies.
export nodeps = 1

# Sub[Package] Doxygen Configuration File and Styles
export RNMAKE_DOXY_CONF_FILE  = $(RNMAKE_ROOT)/make/doxy.conf

# Include standard collection of functions, etc
include $(RNMAKE_ROOT)/Std.mk

# Include envirionment
include $(RNMAKE_ROOT)/Env.mk

# Include package specificiation
include $(RNMAKE_ROOT)/make/Pkg.mk


#------------------------------------------------------------------------------
# Overide Rules.mk Targets

# Simple make distro directory structure
.PHONY: all
all: rnbanner
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk mkdistdirs
	$(footer)

# Only install the documentation
.PHONY: install
install: rnbanner
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk rel
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk install-docs
	$(footer)

# Only make the documentation and source tarballs
.PHONY: tarballs
tarballs: rnbanner
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk tarball-doc
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk tarball-src
	$(footer)

# Only make the documentation and source Debian packages
.PHONY: deb-pkgs
deb-pkgs: rnbanner
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk deb-pkg-doc deb-pkg-src
	$(footer)

# Documents tarball
.PHONY: tarball-doc
tarball-doc:
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk tarball-doc

# Source tarball
.PHONY: tarball-src
tarball-src:
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk tarball-src

.PHONY: distclean clobber
clean distclean clobber: rnbanner
	@$(MAKE) -s -f $(RNMAKE_ROOT)/Rules.mk $(@)
	$(footer)

.PHONY: rnbanner
rnbanner:
	$(call printPkgBanner,$(RNMAKE_PKG_FULL_NAME),any,$(MAKECMDGOALS))

# $(footer)
# 	Conditionally print footer.
footer = $(call printFooter,$(@),$(lasword,$(MAKECMDGOALS)))
