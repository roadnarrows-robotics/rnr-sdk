################################################################################
#
# Rules.python.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file to make python modules using standard setup.py
python file.

Include this file in each appropriate local make file (usually near the bottom).

\par Key RNMAKE Variables:
	\li RNMAKE_PYTHON_ENABLED 	  - python is [not] enabled for this architecture.
	\li PYTHON_MOD_DIR 	  - python module directory (default: ./modules)

\pkgsynopsis
RN Make System

\pkgfile{Rules.python.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_RULES_PYTHON_MK = 1

ifeq ($(strip $(RNMAKE_PYTHON_ENABLED)),y)

PYTHON_MOD_DIR 	 ?= ./modules

SETUP_BUILD_DIR		= ./build
PYTHON_BUILD_DIR	= $(SETUP_BUILD_DIR).$(RNMAKE_ARCH)

define unlink_build
	@if [ -e $(SETUP_BUILD_DIR) ]; \
	then \
		$(UNLINK) $(SETUP_BUILD_DIR); \
	fi
endef

# Target:	python-all (default)
#
# Use python's standard distutils to build and install python modules. The 
# "build - install" sequence is equivalent to the RNMAKE "all" target with
# the "install" installing into the package dist/dist.<arch> directory.
#
# Note that the distutils supports where to build the intermediates but does
# not support the concept of source location for installing to target location,
# but rather always installs from the build directory. Hence the linking high
# jynx.
#
.PHONY: python-all
python-all:
	$(call printGoalDesc,$(PYTHON) setup.py)
	$(call unlink_build)
	$(PYTHON) setup.py build --build-base=$(PYTHON_BUILD_DIR)
	$(SYMLINK) $(PYTHON_BUILD_DIR) $(SETUP_BUILD_DIR)
	$(PYTHON) setup.py install --prefix=$(DIST_ARCH)

# Target: make python source documentation
.PHONY: python-doc
python-doc:
	$(call printGoalDesc,$(@),Making python source documentation)
	@test -d "$(DISTDIR_DOC)/pydoc" || $(MKDIR) $(DISTDIR_DOC)/pydoc
	@$(PYTHON) $(RNMAKE_ROOT)/utils/pydocmk.py \
		--docroot=$(DISTDIR_DOC)/pydoc \
		--vpath=$(DIST_VPATH_LIB) \
		setup

# Target:	python-clean
.PHONY: python-clean
python-clean:
	$(call printGoalDesc,$(@),Removing byte-compiled python files)
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
	$(call printGoalDesc,$(@),Removing intermediate $(PYTHON_BUILD_DIR) directory)
	$(call unlink_build)
	@$(RM) $(PYTHON_BUILD_DIR)

else

python-all python-doc python-clean python-distclean:
	@printf "python not enabled - ignoring\n"

endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
