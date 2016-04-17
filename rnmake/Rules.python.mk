################################################################################
#
# Package: 	RN Make System 
#
# File:			Rules.python.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file for make python modules using python's standard 
setup.py file.

Include this file into each local make file (usually at the bottom).\n

\par Key Variables:
	\li PYTHON_MOD_DIR 	  - python module's directory (default: ./modules)

$LastChangedDate: 2012-11-05 11:05:42 -0700 (Mon, 05 Nov 2012) $
$Rev: 2507 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2010.  RoadNarrows LLC.
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

ifeq "$(PYTHON_ENABLED)" "y"

PYTHON_MOD_DIR 	 ?= ./modules

SETUP_BUILD_DIR		= ./build
PYTHON_BUILD_DIR	= $(SETUP_BUILD_DIR).$(ARCH)

define unlink_build
	@if [ -e $(SETUP_BUILD_DIR) ]; \
	then \
		$(UNLINK) $(SETUP_BUILD_DIR); \
	fi
endef

# Target:	python-all (default)
#
# Use python's standard distutils to build and install python modules. The 
# "build - install" sequence is equivalent to the rnmake "all" target with
# the "install" installing into the package dist/dist.<arch> directory.
#
# Note that the distutils supports where to build the intermediates but does
# not support the concept of source location for installing to target location,
# but rather always installs from the build directory. Hence the linking high
# jynx.
#
.PHONY: python-all
python-all:
	@printf "\n"
	@printf "$(color_tgt_file)     $(PYTHON) setup.py $(color_end)\n"
	$(call unlink_build)
	$(PYTHON) setup.py build --build-base=$(PYTHON_BUILD_DIR)
	$(SYMLINK) $(PYTHON_BUILD_DIR) $(SETUP_BUILD_DIR)
	$(PYTHON) setup.py install --prefix=$(DIST_ARCH)

# Target: make python source documentation
.PHONY: python-doc
python-doc:
	@printf "Making python source documentation\n"
	@$(PYTHON) $(rnmake)/utils/pydocmk.py -d $(DISTDIR_DOC)/pydoc setup

# Target:	python-clean
.PHONY: python-clean
python-clean:
	@printf "Removing byte-compiled python files\n"
	@$(FIND) $(PYTHON_MOD_DIR) \( \
		-name '*.svn*' -or -wholename '.svn' -or \
		-wholename '*.deps*' -or -wholename '.deps' -or \
		-wholename '*obj*' -or -wholename 'obj' -or -wholename '*.o' -or \
		-wholename '*.out' -or -wholename '*.log' \
	\) -prune -or -print | \
	$(GREP) -e '.py[co]' | \
	$(XARGS) $(RM) 

.PHONY: python-distclean
python-distclean:
	$(call unlink_build)
	$(RM) $(PYTHON_BUILD_DIR)

else

python-all python-doc python-clean python-distclean:
	@printf "python not enabled - ignoring\n"

endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
