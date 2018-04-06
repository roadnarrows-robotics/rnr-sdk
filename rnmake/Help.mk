################################################################################
#
# Help.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Prints help message(s) for Rules.mk.

\pkgsynopsis
RN Make System

\pkgfile{Help.mk}

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

_HELP_MK = 1

$(info rnmake - RoadNarrows Make System Help)

archlist = $(subst $(rnmake)/Arch/Arch.,,$(basename $(wildcard $(rnmake)/Arch/Arch.*.mk)))

.PHONY:   help
help: help-usage help-arch help-targets help-install help-tarballs \
			help-dpkg help-other help-help

help-usage:
	@echo " + Synopsis ($@)"
	@echo "usage: make [arch=<arch>] [color=<scheme>] [MAKEARGS] [target]"
	@echo "        <arch>   - <rnmake>/arch/Arch.<arch>.mk architecture make file."
	@echo "        <scheme> - Color scheme. Default: default, off=no color, ..."
	@echo "        <target> - rnmake target goal."

help-arch:
	@echo ""
	@echo " + Supported Target Platform Architectures ($@)"
	@echo $(sort $(archlist))

help-targets:
	@echo ""
	@echo " + Standard Targets ($@)"
	@echo "all            - (default) makes the distribution [sub]package(s)"
	@echo "subdirs        - makes all subdirectories from current directory"
	@echo "<subdir>       - makes subdirectory <subdir> from current directory"
	@echo "libs           - makes all libraries in current directory"
	@echo "<lib>          - makes library <lib> in current directory"
	@echo "pgms           - makes all programs in current directory"
	@echo "<pgm>          - makes program <pgm> in current directory"
	@echo "documents      - makes documentation"
	@echo "install        - installs the distribution"
	@echo "tarballs       - makes package binary, source, and documentation tarballs"
	@echo "deps           - makes source dependencies"
	@echo "clean          - deletes generated intermediate files" 
	@echo "distclean      - cleans plus deletes distribution and local made files"

help-install:
	@echo ""
	@echo " + Install Specific Targets ($@)"
	@echo "install          - installs package distribution files"
	@echo "install-bin      - installs package distribution executables"
	@echo "install-lib      - installs package distribution libraries"
	@echo "install-includes - installs package distribution API headers"
	@echo "install-docs     - installs package distribution documentation"
	@echo "install-share    - installs package distribution system share files"
	@echo "install-etc      - installs package distribution configuration files"

help-tarballs:
	@echo ""
	@echo " + Tarball Specific Targets ($@)"
	@echo "tarballs       - makes package binary, source, and documentation tarballs"
	@echo "tarball-bin    - makes package executables tarball"
	@echo "tarball-doc    - makes documentation tarball"
	@echo "tarball-src    - makes source tarball"

help-dpkg:
	@echo ""
	@echo " + Debian Package Specific Targets ($@)"
	@echo "deb-pkgs       - makes all debian packages for an architecture"
	@echo "deb-pkg-dev    - makes debian development package"
	@echo "deb-pkg-src    - makes debian source package"
	@echo "deb-pkg-doc    - makes debian documentation package"

help-other:
	@echo ""
	@echo " + Other Targets ($@)"
	@echo "<src>.o        - makes object from <src>.{c|cxx}"
	@echo "<src>.ii       - makes post CPP processed source from <src>.{c|cxx}"

help-help:
	@echo ""
	@echo " + Help Targets ($@)"
	@echo "help           - prints full rnmake help"
	@echo "help-usage     - prints rnmake usage"
	@echo "help-arch      - prints list of rnmake supported architectures"
	@echo "help-targets   - prints rnmake standard targets"
	@echo "help-install   - prints rnmake install specific targets"
	@echo "help-tarballs  - prints rnmake tarball specific targets"
	@echo "help-other     - prints rnmake make other targets (debugging)"
	@echo "help-help      - prints this help text"

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
