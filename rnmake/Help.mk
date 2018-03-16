################################################################################
#
# Package: 	RN Make System 
#
# File:			Help.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Prints help message(s) for Rules.mk

$LastChangedDate: 2012-11-05 11:05:42 -0700 (Mon, 05 Nov 2012) $
$Rev: 2507 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2009.  RoadNarrows LLC.
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
