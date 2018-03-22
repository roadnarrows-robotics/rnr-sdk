################################################################################
#
# Package: 	RN Make System
#
# File:			Rules.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Master file for defining rules for make targets, variables, and macros.

Include this file into each local make file (usually at the bottom).

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2018.  RoadNarrows LLC.
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

export _RULES_MK = 1

#------------------------------------------------------------------------------
# Prelims

# this makefile is last in the list (must call before any includes from this)
RNMAKE_ROOT = $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# default goal
.DEFAULT_GOAL := all

# list of command-line goals
GOAL_LIST = $(MAKECMDGOALS)

# add default if empty
ifeq "$(GOAL_LIST)" ""
  GOAL_LIST = $(.DEFAULT_GOAL)
endif

# set first, last, and major goals
FIRST_GOAL	= $(firstword $(GOAL_LIST))
LAST_GOAL 	= $(lastword $(GOAL_LIST))

# list of goals with subdirectory traversals
GOALS_WITH_SUBDIRS = 

# Find goal patterns in command-line goal list. Returns whitespace separated
# matched goals or empty string.
#
# Usage: $(call fnFindGoals,goalpattern...)
define fnFindGoals
$(filter $(1),$(GOAL_LIST))
endef

# Conditionally include makefile if one of the goal patterns matches the
# command-line goal list.
#
# Usage: $(call fnGoalInclude,goalpattern...,makefile)
define fnGoalInclude
$(if $(call fnFindGoals,$(1)),$(eval include $(2)))
endef


#------------------------------------------------------------------------------
# Environment (Env.mk)
#
# Parse rnmake specific command-line and environment variables.

include $(RNMAKE_ROOT)/Env.mk


#------------------------------------------------------------------------------
# Print help (Help.mk)
#
# Check if any of the make goals contain help goals. If true, include the
# help makefile, which defines the help[-<subhelp>] rules.

$(call fnGoalInclude,help help-%,$(RNMAKE_ROOT)/Help.mk)


#------------------------------------------------------------------------------
# Compile and run unit tests (Rules.test.mk)
#
# Check if any of the make goals contain test goals. If true, include the
# test makefile, which defines the [run-]test rules.

$(call fnGoalInclude,test run-test,$(RNMAKE_ROOT)/Rules.test.mk)


#------------------------------------------------------------------------------
# Debian package builds (Ruls.dpkg.mk)
#
# Check if any of the make goals contain a debian package goals. If true,
# include debian package makefile, which defines the deb-pkgs and deb-pkg-<type>
# rules.

$(call fnGoalInclude,deb-pkgs deb-pkg-%,$(RNMAKE_ROOT)/Rules.dpkg.mk)


#------------------------------------------------------------------------------
# Product Makefile (Optional)

ifdef RNMAKE_PROD_MKFILE
  # optionally include (no error if not found)
  -include $(RNMAKE_PROD_MKFILE)
endif


#------------------------------------------------------------------------------
# Package Makefile (required)
#

ifeq ($(RNMAKE_PKG_ROOT),)
  $(error 'RNMAKE_PKG_ROOT': Not defined in including Makefile)
endif

_x := $(realpath $(RNMAKE_PKG_ROOT))

ifeq ($(_x),)
	$(error 'RNMAKE_PKG_ROOT=$(RNMAKE_PKG_MKFILE)': Not a directory)
endif

RNMAKE_PKG_ROOT := $(_x)

RNMAKE_PKG_MKFILE = $(realpath $(RNMAKE_PKG_ROOT)/make/Pkg.mk)

ifeq ($(RNMAKE_PKG_MKFILE),)
	$(error 'RNMAKE_PKG_MKFILE=$(RNMAKE_PKG_ROOT)/make/Pkg.mk': No such file)
endif

# required
include $(RNMAKE_PKG_MKFILE)


# -------------------------------------------------------------------------
# Architecture Dependent Definitions

# Standard rnmake rules do not support the following targets.
ifneq "$(findstring $(RNMAKE_ARCH_TAG),atmega16)" ""
  $(error Rules.mk does not support $(arch) rules)
endif

# Architecture make file name
RNMAKE_ARCH_MKFILE = $(RNMAKE_ROOT)/Arch/Arch.$(RNMAKE_ARCH_TAG).mk

# Test if file exist (empty string returned if not).
ifeq ($(realpath $(RNMAKE_ARCH_MKFILE)),)
	$(error 'RNMAKE_ARCH_MKFILE=$(RNMAKE_ARCH_MKFILE)': Unknown architecture: See $(RNMAKE_ROOT)/Arch)
endif

# Include the architecture make file.
include $(RNMAKE_ARCH_MKFILE)

# Included architecture makefile must define RNMAKE_ARCH which defines the
# real architecture.
#
ifeq ($(RNMAKE_ARCH),)
  $(error 'RNMAKE_ARCH': not defined in including arhitecture makefile)
endif


#------------------------------------------------------------------------------
# Include helper make files (Cmds.mk, Colors.mk)
#
# Can conditionally define macros by architecuture definitions included above.
#

# basic host commands
include $(RNMAKE_ROOT)/Cmds.mk

# color schemes
ifneq "$(color)" "off"
include $(RNMAKE_ROOT)/Colors.mk
endif


#------------------------------------------------------------------------------
# Install Directories - Override as necessary in including Makefile.
#
# Note: These are traditional configuration names - keep the naming convention.

prefix := $(RNMAKE_INSTALL_PREFIX)
ifeq ($(prefix),)
  prefix := $(RNMAKE_INSTALL_XPREFIX)/$(RNMAKE_ARCH)
endif

exec_prefix 	?= $(prefix)
bindir     		?= $(exec_prefix)/bin
sbindir 			?= $(exec_prefix)/sbin
libexecdir 		?= $(exec_prefix)/libexec
sysconfdir 		?= $(prefix)/etc
localstatedir ?= $(prefix)/var
libdir 				?= $(exec_prefix)/lib
includedir 		?= $(prefix)/include
sharedir 			?= $(prefix)/share
infodir 			?= $(prefix)/info
docdir 				?= $(prefix)/share/doc
mandir 				?= $(prefix)/man


#------------------------------------------------------------------------------
# Distribution Directories (Architecture Dependent)
# Notes:
# 	Documents are architecture independent

DIST_ROOT				= $(RNMAKE_PKG_ROOT)/dist
DIST_ARCH       = $(DIST_ROOT)/dist.$(RNMAKE_ARCH)

# Product overrides - some tarballs contain files from all packages
ifdef PROD_FULL_NAME
DIST_NAME_BIN	      = $(PROD_FULL_NAME)
else
DIST_NAME_BIN	      = $(PKG_FULL_NAME)
endif

# Distributions Directories
DISTDIR_BIN     = $(DIST_ARCH)/bin

DISTROOT_LIB 		= $(DIST_ARCH)/lib
ifdef LIB_SUBDIR
DIST_VPATH_LIB 	= $(DISTROOT_LIB):$(DISTROOT_LIB)/$(LIB_SUBDIR)
DIST_LD_LIBDIRS = $(DISTROOT_LIB) $(DISTROOT_LIB)/$(LIB_SUBDIR)
DISTDIR_LIB		  = $(DISTROOT_LIB)/$(LIB_SUBDIR)
else
DIST_VPATH_LIB	= $(DISTROOT_LIB)
DIST_LD_LIBDIRS = $(DISTROOT_LIB)
DISTDIR_LIB     = $(DISTROOT_LIB)
endif

DISTDIR_INCLUDE = $(DIST_ARCH)/include
DISTDIR_ETC     = $(DIST_ARCH)/etc
DISTDIR_MAN     = $(DIST_ARCH)/man
DISTDIR_SHARE   = $(DIST_ARCH)/share/$(PKG_FULL_NAME)
DISTDIR_DOC     = $(DIST_ARCH)/doc/$(PKG_FULL_NAME)-doc
DISTDIR_SRC     = $(DIST_ARCH)/src/$(PKG_FULL_NAME)
DISTDIR_REPO    = $(DIST_ARCH)/repo
DISTDIR_TMP     = $(DIST_ARCH)/tmp/$(DIST_NAME_BIN)-$(RNMAKE_ARCH)
DISTDIR_TMP_DEB	= $(DIST_ARCH)/tmp/deb
DISTDIR_LIST    = $(DISTDIR_BIN) \
                  $(DISTDIR_INCLUDE) \
                  $(DISTDIR_LIB) \
                  $(DISTDIR_ETC) \
                  $(DISTDIR_SHARE) \
                  $(DISTDIR_DOC) \
                  $(DISTDIR_SRC) \
                  $(DISTDIR_REPO) \
                  $(DISTDIR_TMP) \
                  $(DISTDIR_TMP_DEB) \
                  $(DISTDIR_MAN)

# documentation subdirectories
DIST_SRCDOC					= srcdoc
DISTDIR_DOC_SRC			= $(DISTDIR_DOC)/$(DIST_SRCDOC)
DISTDIR_DOC_SRC_IMG	= $(DISTDIR_DOC_SRC)/images

# tar ball files - source, documentation, binary
DIST_TARBALL_SRC		= $(PKG_FULL_NAME)-src.tar.gz
DIST_TARBALL_DOC		= $(PKG_FULL_NAME)-doc.tar.gz
DIST_TARBALL_BIN		= $(DIST_NAME_BIN)-$(RNMAKE_ARCH).tar.gz


#------------------------------------------------------------------------------
# Local Directories (Architecture Dependent)
#
ifndef LOCDIR_ROOT
LOCDIR_ROOT			= $(RNMAKE_PKG_ROOT)/loc
endif
LOCDIR_BIN			= $(LOCDIR_ROOT)/bin.$(RNMAKE_ARCH)
LOCDIR_LIB			= $(LOCDIR_ROOT)/lib.$(RNMAKE_ARCH)
LOCDIR_INCLUDE	= $(RNMAKE_PKG_ROOT)/include
LOCDIR_LIST			= $(LOCDIR_BIN) \
									$(LOCDIR_LIB)

LOC_VPATH_LIB		= $(LOCDIR_LIB)
LOC_LD_LIBDIRS 	= $(LOCDIR_LIB)

# Architecture dependent local include directory 
# Note: Since developers can add files to this directory, never delete it.
ARCH_INCDIR			= $(LOCDIR_INCLUDE)/arch/arch.$(RNMAKE_ARCH)

# Object Directory
OBJDIR					= obj/obj.$(RNMAKE_ARCH)

# Dependencies Directory
DEPSDIR			= .deps

# Dependencies File
DEPSFILE				= $(DEPSDIR)/deps.$(RNMAKE_ARCH)


#------------------------------------------------------------------------------
# VPATH Search Path 
LIBS_VPATH = $(LOC_VAPATH_LIB):$(DIST_VPATH_LIB)
vpath %.a  $(LIBS_VPATH)
vpath %.so $(LIBS_VPATH)


#------------------------------------------------------------------------------
# Build Flags
# Merge Architecture, Package and Parent Makefile variables into build flags.
#

# Include Flags
EXTRA_INCLUDES		= $(addprefix -I,$(EXTRA_INCDIRS))
EXTRA_SYS_INCLUDES= $(addprefix -I,$(EXTRA_SYS_INCDIRS))
PKG_INCLUDES			= $(addprefix -I,$(PKG_INCDIRS))
PROD_INCLUDES			= $(addprefix -I,$(PROD_INCDIRS))
ARCH_INCLUDES			= $(addprefix -I,$(ARCH_INCDIRS))
PKG_SYS_INCLUDES	= $(addprefix -I,$(PKG_SYS_INCDIRS))
DIST_INCLUDES			= -I$(DISTDIR_INCLUDE)
INCLUDES					= -I. \
										$(EXTRA_INCLUDES) \
										$(PKG_INCLUDES) \
										$(PROD_INCLUDES) \
										$(ARCH_INCLUDES) \
										$(DIST_INCLUDES) \
										-I$(includedir) \
										$(EXTRA_SYS_INCLUDES) \
										$(PKG_SYS_INCLUDES)

# CPP Flags
override CPPFLAGS		:= $(EXTRA_CPPFLAGS) \
											$(PKG_CPPFLAGS) \
											$(ARCH_CPPFLAGS) \
											-DARCH_$(RNMAKE_ARCH) \
											-DARCH="\"$(RNMAKE_ARCH)\"" \
											$(CPPFLAGS)

# C Flags
override CFLAGS			:= $(EXTRA_CFLAGS) $(PKG_CFLAGS) $(CFLAGS)

# CXX Flags
override CXXFLAGS		:= $(EXTRA_CXXFLAGS) $(PKG_CXXFLAGS) $(CXXFLAGS)

# Library Path Flags
EXTRA_LD_LIBPATHS	= $(addprefix -L,$(EXTRA_LD_LIBDIRS))
PKG_LD_LIBPATHS		= $(addprefix -L,$(PKG_LD_LIBDIRS))
LOC_LD_LIBPATHS		=	$(addprefix -L,$(LOC_LD_LIBDIRS))
DIST_LD_LIBPATHS	= $(addprefix -L,$(DIST_LD_LIBDIRS))
LD_LIBPATHS			 := $(EXTRA_LD_LIBPATHS) \
									 	 $(PKG_LD_LIBPATHS) \
									 	 $(LOC_LD_LIBPATHS) \
										 $(DIST_LD_LIBPATHS) \
										-L$(libdir) \
										-L$(libdir)/rnr \
										 $(LD_LIBPATHS)

# DHP/RDK libdir/rnr doesn't really belong here :-( please fix.

# External Libraries
LD_LIBS						:= $(EXTRA_LD_LIBS) $(PKG_LD_LIBS) $(LD_LIBS)

LDFLAGS     			:= $(EXTRA_LDFLAGS) $(PKG_LDFLAGS) $(LDFLAGS)

# default link-loader is c compiler - override if using C++
ifeq "$(LANG)" "C++"
LD = $(LD_CXX)
endif

# default link-loader is c compiler - override if using CUDA
ifeq "$(LANG)" "CUDA"
LD = $(LD_CUDA)
endif

#------------------------------------------------------------------------------
# Build Target Names
# Construct build target files and set from parent makefile.
#

# Preferred Library Type for Distribution
LIB_TYPE				:=	$(LIB_TYPE)

# Complete list of core libraries
STLIBS					= $(LOC_STLIBS) $(DIST_STLIBS)
SHLIBS					= $(DIST_SHLIBS)
DLLIBS					= $(DIST_DLLIBS)

# Add test targets
ifeq "$(RNMAKE_TEST)" "true"
LOC_PGMS += $(TEST_PGMS)
endif

# Complete list of core programs
PGMS						= $(LOC_PGMS) $(DIST_PGMS)


# Release files
REL_FILES				= $(PKG_REL_FILES) $(EXTRA_REL_FILES)

# Share make targets
SHARE_TGT			= $(PKG_TGT_SHARE) $(EXTRA_TGT_SHARE)

# Etc make targets
ETC_TGT				= $(PKG_TGT_ETC) $(EXTRA_TGT_ETC)

#
# Fully Qualified target names as opposed to the tight, core names 
# used in the enclosing makefiles.

# Fully Qualified Static Library Name(s) from Core Name(s)
# Usage: fq_stlib_names dir,libs
fq_stlib_names 	=	$(addprefix $(1)/$(STLIB_PREFIX),\
									$(addsuffix $(STLIB_SUFFIX),$(2)))

# Fully Qualified Shared Library Name(s) from Core Name(s)
fq_shlib_names 	=	$(addprefix $(1)/$(SHLIB_PREFIX),\
									$(addsuffix $(SHLIB_SUFFIX),$(2)))

# Fully Qualified Dynamically Linked Library Name(s) from Core Name(s)
fq_dllib_names 	=	$(addprefix $(1)/$(DLLIB_PREFIX),\
									$(addsuffix $(DLLIB_SUFFIX),$(2)))

# Core Static Library Name(s) from Fully Qualified Name(s)
co_stlib_name		= $(patsubst %$(STLIB_SUFFIX),%, \
									$(patsubst $(1)/$(STLIB_PREFIX)%,%,$(2)))

# Fully Qualified Static Libraries
FQ_STLIBS				= $(call fq_stlib_names,$(LOCDIR_LIB),$(LOC_STLIBS)) \
									$(call fq_stlib_names,$(DISTDIR_LIB),$(DIST_STLIBS))

# Fully Qualified Shared Libraries
FQ_SHLIBS				= $(call fq_shlib_names,$(DISTDIR_LIB),$(DIST_SHLIBS))

# Fully Qualified Dynamically Linked Libraries
FQ_DLLIBS				= $(call fq_dllib_names,$(DISTDIR_LIB),$(DIST_DLLIBS))

# Fully Qualified Program Name(s) from Core Name(s)
fq_pgm_names 		=	$(addprefix $(1)/$(PGM_PREFIX),\
									$(addsuffix $(PGM_SUFFIX),$(2)))

# Fully Qualified Programs
FQ_PGMS					= $(call fq_pgm_names,$(LOCDIR_BIN),$(LOC_PGMS)) \
									$(call fq_pgm_names,$(DISTDIR_BIN),$(DIST_PGMS))

# Release Files
FQ_REL_FILES 		= $(addprefix $(DISTDIR_DOC)/,$(REL_FILES))

# Auto-Generated Header Files
AUTOHDRS				= $(addprefix $(LOCDIR_INCLUDE)/,version.h)

# DEPRECATED		$(addprefix $(ARCH_INCDIR)/,install.h)

#------------------------------------------------------------------------------
# Target Specific Variables
#

# Shared library compiled objects need special CFLAGS (e.g. -fPIC)
$(FQ_SHLIBS): CFLAGS += $(SHLIB_CFLAGS)

# Dynamically Linked library compiled objects need special CFLAGS
$(FQ_DLLIBS): CFLAGS += $(DLLIB_CFLAGS)

#------------------------------------------------------------------------------
# Common Support Functions and Macros
#

# GNU Make has no boolean functions (why???), fake it.
neq = $(filter-out $(1),$(2))
eq  = $(if $(call neq,$(1),$(2)),,1)

# Returns "1" if given entity exists.
isfile 	= $(shell  if [ -f $(1) ]; then echo 1; fi)
isdir 	= $(shell  if [ -d $(1) ]; then echo 1; fi)

# Major Version sanity check 
sanity_chk_major_ver = \
	$(if $($(1)),\
		$(if $(call neq,$(RULES_RN_MAKE_VER_MAJOR),$($(1))),\
			$(error Error: $(1): major version mismatch: $($(1)) != $(RULES_RN_MAKE_VER_MAJOR))),\
		$(error Error: $(1): not set))

# Minor Version sanity check 
sanity_chk_minor_ver = \
	$(if $($(1)),\
		$(if $(call neq,$(RULES_RN_MAKE_VER_MINOR),$($(1))),\
			$(warning Warning: $(1): major version mismatch: $($(1)) != $(RULES_RN_MAKE_VER_MINOR))),\
		$(warning Warning: $(1): not set))

# Generate list of objects from sources given the core target name
objs_from_src = $(addprefix $(OBJDIR)/,$(subst .c,.o,$($(1).SRC.C))) \
 								$(addprefix $(OBJDIR)/,$(subst .cxx,.o,$($(1).SRC.CXX))) \
 								$(addprefix $(OBJDIR)/,$(subst .cpp,.o,$($(1).SRC.CPP))) \
 								$(addprefix $(OBJDIR)/,$(subst .cu,.o,$($(1).SRC.CU)))

# Make obj/obj-<RNMAKE_ARCH> in current directory
mkobjdir = \
	@test -d "$(OBJDIR)" || $(MKDIR) $(OBJDIR); \
	test -d "$(dir $(1))" || test -z "$(dir $(1))" || $(MKDIR) "$(dir $(1))"


########################### Explicit Rules #####################################

# -------------------------------------------------------------------------
# Target:	all (default)
# Desc: 	Front end for making the [sub]package(s) (libraries, programs, tools,
# 				documents, etc).
# Notes: 	There are two version:
# 					1) only done once on the first invocation and 
# 					2) for all other invocations.
.PHONY: all
ifeq ($(MAKELEVEL),0)
all: pkgbanner once-all $(EXTRA_TGT_ALL) pkg subdirs-all $(EXTRA_TGT_ALL_POST)
	$(footer)
else
all: $(EXTRA_TGT_ALL) pkg subdirs-all $(EXTRA_TGT_ALL_POST)
endif

.PHONY: once-all
once-all: mkdistdirs mklocdirs autohdrs 


# -------------------------------------------------------------------------
# Target:	pkg
# Desc: 	Makes the distribution [sub]package(s) (libraries, programs, tools, 
# 				documents, etc).
.PHONY: pkg
pkg: libs pgms rel share


# -------------------------------------------------------------------------
# Target:	libs
# Desc: 	Makes all libraries in current directory.

# Make all libraries
.PHONY: libs
libs: stlibs shlibs dllibs

# Make all static libraries
.PHONY: stlibs
stlibs: $(FQ_STLIBS)

# Make all shared libraries
.PHONY: shlibs
shlibs: $(FQ_SHLIBS)

# Make all dll libraries
.PHONY: dllibs
dllibs: $(FQ_DLLIBS)

# Make specific distribution static librarary
.PHONY: $(DIST_STLIBS)
$(DIST_STLIBS): $(call fq_stlib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific distribution shared librarary
.PHONY: $(DIST_SHLIBS)
$(DIST_SHLIBS): $(call fq_shlib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific distribution dll librarary
.PHONY: $(DIST_DLLIBS)
$(DIST_DLLIBS): $(call fq_dllib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific local static librarary
.PHONY: $(LOC_STLIBS)
$(LOC_STLIBS): $(call fq_stlib_names,$(LOCDIR_LIB),$(GOAL_LIST))

# Template to build a static library including all necessary prerequisites
define STLIBtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).FQ_LIB = $(call fq_stlib_names,$(2),$(1))
 OUTDIR = $$(dir $$($(1).FQ_LIB))
 $$($(1).FQ_LIB): $$($(1).OBJS)
	@printf "\n"
	@printf "$(color_tgt_lib)     $$@$(color_end)\n"
	@test -d $$(OUTDIR) || $(MKDIR) $$(OUTDIR)
	$$(STLIB_LD) $$(STLIB_LD_FLAGS) $$(STLIB_LD_EXTRAS) $$@  $$($(1).OBJS)
	$$(RANLIB) $$@
endef

# Template to build a shared library including all necessary prerequisites
define SHLIBtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).LIBS := $(addprefix -l, $($(1).LIBS))
 $(1).FQ_LIB = $(call fq_shlib_names,$(2),$(1))
 OUTDIR = $$(dir $$($(1).FQ_LIB))
 $$($(1).FQ_LIB): $$($(1).OBJS)
	@printf "\n"
	@printf "$(color_tgt_lib)     $$@$(color_end)\n"
	@test -d $$(OUTDIR) || $(MKDIR) $$(OUTDIR)
	$$(SHLIB_LD) $$(SHLIB_LD_FLAGS) $$(SHLIB_LD_EXTRAS) -o $$@  $$($(1).OBJS) $$(LD_LIBPATHS) $$($(1).LIBS) $$(LD_LIBS)
endef

# Template to build a dynamically linke library including all necessary
# prerequisites
define DLLIBtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).LIBS := $(addprefix -l, $($(1).LIBS))
 $(1).FQ_LIB = $(call fq_dllib_names,$(2),$(1))
 OUTDIR = $$(dir $$($(1).FQ_LIB))
 $$($(1).FQ_LIB): $$($(1).OBJS)
	@printf "\n"
	@printf "$(color_tgt_lib)     $$@$(color_end)\n"
	@test -d $$(OUTDIR) || $(MKDIR) $$(OUTDIR)
	$$(DLLIB_LD) $$(DLLIB_LD_FLAGS) $$(DLLIB_LD_EXTRAS) $$($(1).OBJS) $$(LD_LIBPATHS) $$($(1).LIBS) $$(LD_LIBS) -o $$@
endef

# For each library target, evaluate (i.e make) template.
$(foreach lib,$(LOC_STLIBS),$(eval $(call STLIBtemplate,$(lib),$(LOCDIR_LIB))))
$(foreach lib,$(DIST_STLIBS),$(eval $(call STLIBtemplate,$(lib),$(DISTDIR_LIB))))
$(foreach lib,$(DIST_SHLIBS),$(eval $(call SHLIBtemplate,$(lib),$(DISTDIR_LIB))))
$(foreach lib,$(DIST_DLLIBS),$(eval $(call DLLIBtemplate,$(lib),$(DISTDIR_LIB))))

# -------------------------------------------------------------------------
# Target:	pgms
# Desc: 	Makes all programs in current directory.

# Make all programs
.PHONY: pgms
pgms: $(FQ_PGMS)

# Make specific local program
.PHONY: $(LOC_PGMS)
$(LOC_PGMS): $(call fq_pgm_names,$(LOCDIR_BIN),$(GOAL_LIST))

# Make specific distribution program
.PHONY: $(DIST_PGMS)
$(DIST_PGMS): $(call fq_pgm_names,$(DISTDIR_BIN),$(GOAL_LIST))

# Template to build a program including all necessary prerequisites
define PGMtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).LIBDEPS  = $(shell $(RNMAKE_ROOT)/utils/libdeps.sh $(LIBS_VPATH) $($(1).LIBDEPS))
 $(1).LIBS := $(addprefix -l, $($(1).LIBS))
 $(1).FQ_PGM = $(call fq_pgm_names,$(2),$(1))
 $$($(1).FQ_PGM): $$($(1).OBJS) $$($(1).LIBDEPS)
	@printf "\n"
	@printf "$(color_tgt_pgm)     $$@$(color_end)\n"
	$$(LD) $$(LDFLAGS) $$(LD_LIBPATHS) $$($(1).OBJS) $$($(1).LIBS) $$(LD_LIBS) -o $$@
endef

libdeps = $(shell for lib in $(1); do echo lib$${lib}.a; done)

# For each program target, evaluate (i.e make) template.
$(foreach prog,$(LOC_PGMS),$(eval $(call PGMtemplate,$(prog),$(LOCDIR_BIN))))
$(foreach prog,$(DIST_PGMS),$(eval $(call PGMtemplate,$(prog),$(DISTDIR_BIN))))

## RDK investigate for linking programs
#	$(LINK.o) $^ $(LDLIBS) -o $@

# -------------------------------------------------------------------------
# Target:	autohdrs
# Desc: 	Makes auto-generated header files.
# Notes:	Auto-headers are only made at top level.
#  	      May make a shell script to do this.
autohdrs: $(AUTOHDRS)

# verion.h auto-generated header
$(LOCDIR_INCLUDE)/version.h: $(RNMAKE_PKG_MKFILE)
	$(printgoal)
	@test -d $(LOCDIR_INCLUDE) || $(MKDIR) $(LOCDIR_INCLUDE)
	@$(MAKE) -f $(RNMAKE_ROOT)/version_h.mk -s \
		RNMAKE_PKG_ROOT=$(RNMAKE_PKG_ROOT) \
		version_h=$@ \
		pkg_mk=$(RNMAKE_PKG_MKFILE)

# install.h auto-generated header DEPRECATED
$(ARCH_INCDIR)/install.h: $(RNMAKE_ARCH_MKFILE)
	@test -d $(ARCH_INCDIR) || $(MKDIR) $(ARCH_INCDIR)
	@$(MAKE) -f $(RNMAKE_ROOT)/install_h.mk -s \
	 	RNMAKE_PKG_ROOT=$(RNMAKE_PKG_ROOT) install_h=$@ \
		arch=$(RNMAKE_ARCH) \
		bindir=$(bindir) \
		sbindir=$(sbindir) \
		libdir=$(libdir) \
		includedir=$(includedir) \
		sysconfdir=$(sysconfdir) \
		docdir=$(docdir) \
		mandir=$(mandir) \
		infodir=$(infodir)


# -------------------------------------------------------------------------
# Target:	hdrs
# Desc: 	Makes interface header files

# List of all header tags
HDR_TAG_LIST = $(addsuffix .HDRS.H,$(DIST_HDRS))
#$(info HDR_TAG_LIST: $(HDR_TAG_LIST))

# Complete list of headers
PREREQ_HDRS = $(foreach tag,$(HDR_TAG_LIST),$($(tag)))
#$(info PREREQ_HDRS: $(PREREQ_HDRS))

# Make all distribution headers
.PHONY: hdrs
hdrs: echo-hdrs $(PREREQ_HDRS)

.PHONY: echo-hdrs
echo-hdrs:
	$(call fnGoalDesc,$(DISTDIR_INCLUDE),Copying tagged interfaces headers from $(RNMAKE_PKG_ROOT)/include.)

# Copy newer headers to distribution include [sub]directory
$(PREREQ_HDRS):
	@for h in $(@); \
		do\
			src=$(RNMAKE_PKG_ROOT)/include/$$h; \
			dst=$(DISTDIR_INCLUDE)/$$h; \
			hdir=$$(dirname $$dst); \
			if [ ! -f $$dst -o $$src -nt $$dst ]; \
			then \
				test -d $$hdir || $(MKDIR) $$hdir; \
				$(CP) $$src $$dst; \
			fi; \
		done

# -------------------------------------------------------------------------
# Target:	rel
# Desc: 	Makes all release files
# Notes:	Release files are only made at top level
.PHONY: rel
ifeq "$(RNMAKE_PKG_ROOT)" "."
rel: $(FQ_REL_FILES)
else
rel:
endif

.PHONY: $(DISTDIR_DOC)/VERSION.txt
$(DISTDIR_DOC)/VERSION.txt:
	@echo ""
	@echo "     $@"
	@echo "$(PKG) v$(PKG_VERSION_DOTTED)"  > $@
	@echo "Copyright (C) $(PKG_VERSION_DATE) RoadNarrows LLC" >> $@
	@echo "" >> $@
	@echo "Compiled: `date`" >> $@

$(DISTDIR_DOC)/README.md: README.md
	@echo ""
	@echo "     $@"
	-$(CP) README.md $@

$(DISTDIR_DOC)/README.txt: README.txt
	@echo ""
	@echo "     $@"
	-$(CP) README.txt $@

$(DISTDIR_DOC)/README.xml: README.xml
	@echo ""
	@echo "     $@"
	-$(CP) README.xml $@

# -------------------------------------------------------------------------
# Target:	share
# Desc: 	Makes all share files
# Notes:	Share files are only made at top level
.PHONY: share
ifeq "$(RNMAKE_PKG_ROOT)" "."
share: $(SHARE_TGT)
else
share:
endif

# -------------------------------------------------------------------------
# Target:	etc (not used yet)
# Desc: 	Makes all system configuration files
# Notes:	Etc files are only made at top level
.PHONY: etc
ifeq "$(RNMAKE_PKG_ROOT)" "."
etc: $(ETC_TGT)
else
etc:
endif

# -------------------------------------------------------------------------
# Target: documents
# Desc:   Recursively make subdirectories.
.PHONY: documents
documents: pkgbanner echo-documents docs-clean docs-src-gen docs-pub-gen \
						$(EXTRA_TGT_DOC)
	$(footer)

.PHONY: echo-documents
echo-documents:
	$(call fnEchoGoalDesc,Making documentation)

DOXY_VER=1.8.11

# documentation generator from source files
ifndef HTML_HEADER
HTML_HEADER     = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy_header.html
endif

ifndef HTML_FOOTER
HTML_FOOTER     = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy_footer.html
endif

ifndef HTML_STYLESHEET
HTML_STYLESHEET = $(RNMAKE_ROOT)/doxy/$(DOXY_VER)/rn_doxy.css
endif

ifndef DOXY_IMAGES
DOXY_IMAGES = $(RNMAKE_ROOT)/doxy/rn_images
endif

ifndef RN_DOXY_CONF_FILE
RN_DOXY_CONF_PATH = $(RNMAKE_ROOT)/doxy
RN_DOXY_CONF_FILE = $(RN_DOXY_CONF_PATH)/rn_doxy.conf
endif

docs-clean:
	$(printgoal)
	$(RM) $(DISTDIR_DOC)

# generate doxygen source documetiona
docs-src-gen:
	$(printgoal)
	@if [ "$(DOXY_CONF_FILE)" ]; \
	then \
		echo "Making doxygen source documentation"; \
		test -d $(DISTDIR_DOC) || $(MKDIR) $(DISTDIR_DOC); \
		test -d $(DISTDIR_DOC_SRC_IMG) || $(MKDIR) $(DISTDIR_DOC_SRC_IMG); \
		$(CP) -r $(DOXY_IMAGES)/* $(DISTDIR_DOC_SRC_IMG)/.; \
		$(CP) -r $(HTML_STYLESHEET) $(DISTDIR_DOC_SRC)/.; \
		(cat $(DOXY_CONF_FILE); \
		 echo "PROJECT_NUMBER=$(PKG_VERSION_DOTTED)"; \
		 echo "HTML_HEADER=$(HTML_HEADER)"; \
		 echo "HTML_FOOTER=$(HTML_FOOTER)"; \
		 echo "OUTPUT_DIRECTORY=$(DISTDIR_DOC)"; \
		 echo "HTML_OUTPUT=$(DIST_SRCDOC)"; \
		 echo "PROJECT_LOGO=$(DISTDIR_DOC_SRC_IMG)/RNLogo.png"; \
		 echo "@INCLUDE_PATH=$(RN_DOXY_CONF_PATH)"; \
		 echo "@INCLUDE=$(RN_DOXY_CONF_FILE)"; \
		) | doxygen - >$(RNMAKE_PKG_ROOT)/doxy.out.log 2>$(RNMAKE_PKG_ROOT)/doxy.err.log; \
	fi
	$(footer)

# This utility scrip is no longer used after doxygen 1.7. DEPRECATED
#		$(RNMAKE_ROOT)/utils/doxyindex.sh \
#							-t "$(PKG) v$(PKG_VERSION_DOTTED)" \
#							-h $(HTML_HEADER) \
#							>$(DISTDIR_DOC)/$(DIST_SRCDOC)/index.html; \

# generate published documentation
docs-pub-gen:
	$(printgoal)
	@pubdstdir="$(DISTDIR_DOC)/papers"; \
	pubsrcdir="$(RNMAKE_PKG_ROOT)/docs/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi; \
	pubsrcdir="$(RNMAKE_PKG_ROOT)/3rdparty/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi
	$(footer)


# -------------------------------------------------------------------------
# Target:	mkdistdirs
# Desc: 	Make Distribution Directories 
.PHONY: mkdistdirs $(DISTDIR_LIST)
mkdistdirs: echomkdistdirs $(DISTDIR_LIST)

.PHONY: echomkdistdirs
echomkdistdirs:
ifeq ($(realpath $(DIST_ARCH)),)
	$(call fnGoalDesc,\
		$(patsubst $(RNMAKE_PKG_ROOT)/%,%,$(DIST_ARCH)),Making distribution directories.)
endif

$(DISTDIR_LIST):
	@test -d ${@} || $(MKDIR) ${@}


# -------------------------------------------------------------------------
# Target:	mklocdirs
# Desc: 	Make Local Directories 
.PHONY: mklocdirs $(LOCDIR_LIST) $(PKGDIR_INCLUDE)
mklocdirs: echolocdirs $(LOCDIR_LIST) $(PKGDIR_INCLUDE)

.PHONY: echolocdirs
echolocdirs:
ifeq ($(realpath $(LOCDIR_BIN)),)
	$(call fnGoalDesc,\
		$(patsubst $(RNMAKE_PKG_ROOT)/%,%,$(LOCDIR_ROOT)),Making local directories.)
endif

$(LOCDIR_LIST) $(PKGDIR_INCLUDE):
	@test -d ${@} || $(MKDIR) ${@}


# -------------------------------------------------------------------------
# Target:	install
# Desc: 	Install the distribution
.PHONY: install
install: pkgbanner all echo-install $(EXTRA_TGT_INSTALL) install-bin \
					install-lib install-includes install-docs install-share install-etc \
					$(EXTRA_TGT_INSTALL_POST)
	$(footer)

.PHONY: echo-install
echo-install:
	$(call fnEchoGoalDesc,Installing package $(PKG_FULL_NAME))

# DEPRECATED
instest:
	@if [ ! -f $(DISTDIR_DOC)/VERSION.txt ]; \
	then \
		echo "Error: The $(PKG) package has not been built. Try \"make\" first."; \
		echo "Install aborted."; \
		exit 4; \
	fi

# install bin
install-bin:
	$(printgoal)
	@printf "Installing executables to $(bindir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 755 $(DISTDIR_BIN) $(bindir)
	$(call POSTINStemplate,$(STRIP_EXE),$(DISTDIR_BIN),$(bindir))

# install lib
install-lib:
	$(printgoal)
	@printf "Installing libraries to $(libdir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 755 $(DISTROOT_LIB) $(libdir)
	$(call POSTINStemplate,$(STRIP_LIB),$(DISTROOT_LIB),$(libdir))

# install includes
install-includes: hdrs
	$(printgoal)
	@printf "Installing includes to $(includedir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 664 $(DISTDIR_INCLUDE) $(includedir)

# install documentation
install-docs: documents
	$(printgoal)
	@printf "Installing documents to $(docdir)/$(PKG_FULL_NAME)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh -s 664 $(DISTDIR_DOC) $(docdir)/$(PKG_FULL_NAME)

# install share files
install-share:
	$(printgoal)
	@printf "Installing system share files to $(sharedir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)/$(PKG_FULL_NAME)
	@if [ ! -e $(sharedir)/$(PKG) ]; \
	then \
		$(SYMLINK) $(sharedir)/$(PKG_FULL_NAME) $(sharedir)/$(PKG); \
	elif [ -L $(sharedir)/$(PKG) ]; \
	then \
		$(UNLINK) $(sharedir)/$(PKG); \
		$(SYMLINK) $(sharedir)/$(PKG_FULL_NAME) $(sharedir)/$(PKG); \
	fi

# install etc
install-etc:
	$(printgoal)
	@printf "Installing system configuration to $(sysconfdir)\n"
	@printf "\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 664 $(DISTDIR_ETC) $(sysconfdir)

# Post-Install directory component template
# Usage: POSTINStemplate postprocess source_dir dest_dir
define POSTINStemplate
	@cd $(2); \
	srclist=$$($(FIND) . -type f); \
	for src in $$srclist; \
	do \
		dst="$(3)/$${src##./}"; \
		-$(1) $$dst 2>/dev/null || echo "  $$dst"; \
	done;
endef


# -------------------------------------------------------------------------
# Target:	tarballs
# Desc: 	Makes package tar balls
# -------------------------------------------------------------------------
.PHONY: tarballs
tarballs: pkgbanner tarball-bin tarball-doc tarball-src
	$(footer)

.PHONY: tarball-doc
tarball-doc: 
	$(printgoal)
	$(if $(call isdir,$(DISTDIR_DOC)),,\
			    					$(error No documentation - Try 'make documents' first.))
	@cd $(DIST_ARCH)/doc; \
	$(TAR) $(DISTDIR_REPO)/$(DIST_TARBALL_DOC) $(PKG_FULL_NAME)-doc
	$(footer)

.PHONY: tarball-src
tarball-src: 
	$(printgoal)
	@$(RM) $(DISTDIR_SRC)
	@test -d $(DISTDIR_SRC) || $(MKDIR) $(DISTDIR_SRC)
	@$(RNMAKE_ROOT)/utils/tarball-src-filter.sh $(RNMAKE_PKG_ROOT) | \
	while read src; \
	do \
		$(RNMAKE_ROOT)/utils/cppath.sh $$src $(DISTDIR_SRC); \
	done;
	@cd $(DIST_ARCH)/src; \
	$(TAR) $(DISTDIR_REPO)/$(DIST_TARBALL_SRC) $(PKG_FULL_NAME)
	$(footer)

.PHONY: tarball-bin
tarball-bin:
	$(printgoal)
	$(if $(call isdir,$(DIST_ARCH)),,$(error Nothing made - Try 'make' first.))
	@test -d $(DISTDIR_TMP) || $(MKDIR) $(DISTDIR_TMP)
	@cd $(DIST_ARCH); \
	$(FIND) bin lib include etc share -print | \
	while read src; \
	do \
		if [ -f $$src ]; \
		then \
			$(RNMAKE_ROOT)/utils/cppath.sh $$src tmp/$(DIST_NAME_BIN)-$(RNMAKE_ARCH); \
		fi; \
	done;
	@cd $(DIST_ARCH)/tmp; \
	$(TAR) $(DISTDIR_REPO)/$(DIST_TARBALL_BIN) $(DIST_NAME_BIN)-$(RNMAKE_ARCH)
	$(footer)


# -------------------------------------------------------------------------
# Target:	deps
# Desc: 	Makes dependencies
# -------------------------------------------------------------------------
.PHONY: deps
deps: pkgbanner echo-deps autohdrs mkdepsdir $(EXTRA_TGT_DEPS) hdrdeps libdeps \
			subdirs-deps
	$(footer)

.PHONY: echo-deps
echo-deps:
	$(call fnEchoGoalDesc,Making dependencies for $(CURDIR))

.PHONY: hdrdeps
hdrdeps: 
	@$(call fnGoalDesc,$(@),Making C/C++ dependencies)
	@echo $(call hdrdeps_sh,$(STLIBS) $(SHLIBS) $(DLLIBS) $(PGMS))

hdrdeps_sh = \
	$(shell $(RNMAKE_ROOT)/utils/hdrdeps.sh \
		-c "$(MAKEDEPS)" \
		-f $(DEPSFILE) \
		-o $(OBJDIR) \
		-d "$(CPPFLAGS)" \
		$(INCLUDES) \
		$(foreach f,\
			$(sort 	$(addsuffix .SRC.C,$(1)) \
							$(addsuffix .SRC.CXX,$(1)) \
							$(addsuffix .SRC.CPP,$(1))),\
					$($(f))))

libdeps:

.PHONY: mkdepsdir
mkdepsdir:
	@test -d $(DEPSDIR) || $(MKDIR) $(DEPSDIR)


# -------------------------------------------------------------------------
# Target:	clean
# Desc: 	Deletes generated intermediate files
# -------------------------------------------------------------------------
.PHONY: clean
clean: pkgbanner echo-clean do-clean $(EXTRA_TGT_CLEAN) subdirs-clean
	$(footer)

.PHONY: echo-clean
echo-clean:
	$(call fnEchoGoalDesc,)

.PHONY: do-clean
do-clean:
	@echo "Cleaning $(CURDIR)"
	$(RM) *.o *.ii *.c~ *.cxx~ .h~ *.pyc *.pyo $(LOC_PGMS) $(DIST_PGMS) a.out doxy.*.log
	$(RM) $(OBJDIR)


# -------------------------------------------------------------------------
# Target:	distclean (clobber)
# Desc: 	Cleans plus deletes distribution
# -------------------------------------------------------------------------
.PHONY: distclean clobber
distclean clobber: pkgbanner clean $(EXTRA_TGT_DISTCLEAN)
	$(call fnGoalDesc,$(@),Clobbering distribution $(CURDIR))
	$(RM) $(DIST_ARCH)
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-$(RNMAKE_ARCH).tar.gz
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-doc.tar.gz
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-src.tar.gz
	$(RM) $(LOCDIR_LIST)
	$(RM) $(AUTOHDRS)
	$(RM) $(DEPSFILE)
	$(footer)


# -------------------------------------------------------------------------
# Target:	subdirs
# Desc: 	Recursively make subdirectories.
# Notes:	Any two or more goals that traverse the subdirectory tree need
# 				separate subdirectory targets. Otherwise, only the first goal
# 				will traverse.
# -------------------------------------------------------------------------

# Subdirectory call
.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)

# Make all sub-directories with all command-line goals
$(SUBDIRS):
	$(call fnDirBanner,$(@),$(GOAL_LIST))
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C $(@) $(GOAL_LIST)
	@printf "    $(color_dir_banner)~~$(color_end)\n"

#
# Template to build subdirectories by goal rules. Since rnmake traverses the
# command-line goals depth first, and GNU make will only execute a rule once,
# this template builds unique subdirectory rules.
#
# Built Rules:
#   subdirs-<goal>: $(SUBDIRS.<goal>)
#   $(SUBDIRS.<goal>:
#   	<recipe>
#
# Usage: $(call SUBDIRtemplate,goal)
#
define SUBDIRtemplate
SUBDIRS.$(1) = $(addsuffix .$(1),$(SUBDIRS))

subdirs-$(1): $$(SUBDIRS.$(1))

$$(SUBDIRS.$(1)):
	$$(call fnDirBanner,$$(basename $$(@)),$(1))
	@$$(MAKE) $$(EXTRA_MAKE_FLAGS) -C $$(basename $$(@)) $(1)
	@printf "    $(color_dir_banner)~~$(color_end)\n"
endef

# all goals with subdirectory traversal prerequisite
GOALS_WITH_SUBDIRS += deps all clean

# build make rules for goal-specific subdirectories
$(foreach goal,$(GOALS_WITH_SUBDIRS),$(eval $(call SUBDIRtemplate,$(goal))))


# -------------------------------------------------------------------------
# Pretty Print Support.
#
# Defines canned sequences, goal recipes, and goal prerequisite targets.
# -------------------------------------------------------------------------

# Print directory banner canned sequence.
#
# Usage: $(call fnDirBanner,dir,goal)
#
define fnDirBanner =
	@if [ "$(1)" != "" ]; then \
		subdirname="$(patsubst $(dir $(RNMAKE_PKG_ROOT))%,%,$(CURDIR)/$(1))";\
	else \
		subdirname=$(notdir $(RNMAKE_PKG_ROOT));\
	fi; \
	printf "\n";\
	printf "$(color_dir_banner)$(normline)\n";\
	printf "Directory: $$subdirname\n";\
	printf "Goal:      $(2)\n";\
	printf "$(normline)$(color_end)\n";\
	fi
endef

# Print goal banner canned sequence.
#
# Usage: $(call fnGoalBanner,goal)
#
define fnGoalBanner =
	@if [ "$(1)" != "" ]; then \
		printf "\n     $(color_tgt_file)$(1)$(color_end)\n";\
	fi
endef

# Print goal with description canned sequence.
#
# Usage: $(call fnGoalDesc,goal,desc)
#
define fnGoalDesc =
	$(call fnGoalBanner,$(1));
	$(if $(2),@printf "$(2)\n",)
endef

# Silly print goal with description canned sequence.
# The goal is determined by stripping off the 'echo-' prefix from the current
# target $(@).
#
# Usage: $(call fnEchoGoalDesc,desc)
#
define fnEchoGoalDesc =
	$(call fnGoalDesc,$(patsubst echo-%,%,$(@)),$(1))
endef

# Print footer canned sequence.
#
# Usage: $(call fnFooter,goal)
#
define fnFooter =
	@if [ "$(MAKELEVEL)" = "0" -a "$(1)" = "$(LAST_GOAL)" ]; then \
	printf "\n";\
	printf "$(color_pkg_banner)              ###\n";\
	printf "Finished: `date`\n";\
	printf "              ###$(color_end)\n";\
	fi
endef

# Recipe pretty prints for current goal.
printgoal				= $(call fnGoalBanner,$(@))
footer 					= $(call fnFooter,$(@))

# Pretty print make banner.
.PHONY: pkgbanner
pkgbanner:
	@if [ "$(MAKELEVEL)" = "0" ]; then \
	printf "$(color_pkg_banner)$(boldline)\n";\
	printf "Package:       $(PKG_FULL_NAME)\n";\
	printf "Package Root:  $(RNMAKE_PKG_ROOT)\n";\
	printf "Architecture:  $(RNMAKE_ARCH)\n";\
	printf "Directory:     $(CURDIR)\n";\
	printf "Goal(s):       $(GOAL_LIST)\n";\
	printf "Start:         `date`\n";\
	printf "$(boldline)$(color_end)\n";\
	fi

eqline := \
================================================================================
dotline := \
................................................................................
tildeline := \
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
underline := \
________________________________________________________________________________

boldline := $(eqline)
normline := $(tildeline)

# -------------------------------------------------------------------------
# Conditionally include any dependency file only specific targets
#
# File: .deps/deps.$(RNMAKE_ARCH)

# Include dependency file or error canned sequence.
# If the override variable 'nodeps' is not empty, then no action is performed.
define fnDepsInclude
	$(if $(nodeps),,\
		$(if $(call isfile,$(DEPSFILE)),\
			$(eval include $(DEPSFILE)),\
			$(error No dependencies file - Try 'make deps' first)))
endef

# Include dependency file for specific targets only.
$(if $(call fnFindGoals,all test),$(call fnDepsInclude),)


# -------------------------------------------------------------------------
# force some targets to always make
force: ;


########################### Pattern Rules #####################################

# C Rule: <name>.c -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.c
	$(call mkobjdir,$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C++ Rule: <name>.cxx -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.cxx
	$(call mkobjdir,$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C++ Rule: <name>.cpp -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.cpp
	$(call mkobjdir,$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# CUDA Rule: <name>.cu -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.cu
	$(call mkobjdir,$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CUDA) $(CUDAFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# Compile a single c file. (Nice for debugging)
%.o : %.c force
	$(call mkobjdir,$(OBJDIR)/$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single cxx file. (Nice for debugging)
%.o : %.cxx force
	$(call mkobjdir,$(OBJDIR)/$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single cpp file. (Nice for debugging)
%.o : %.cpp force
	$(call mkobjdir,$(OBJDIR)/$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single cuda file. (Nice for debugging)
%.o : %.cu force
	$(call mkobjdir,$(@))
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CUDA) $(CUDAFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.c -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
# Historically the output suffix is .i but this can interfere with swig 
# interface files that also have the .i suffix. So .ii will be used until 
# I find another "standard".
%.ii : %.c force
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CC) $(CFLAGS_CPP_ONLY) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.cpp -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
%.ii : %.cxx force
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS_CPP_ONLY) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.cpp -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
%.ii : %.cpp force
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CXX) $(CXXFLAGS_CPP_ONLY) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.cu -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
%.ii : %.cu force
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CUDA) $(CUDAFLAGS_CPP_ONLY) $(CUDAFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

null:
	@echo "null me"


# -------------------------------------------------------------------------
# default error rule (doesn't work yet)

#%::
#	@echo "$(@): Unknown target. See 'make help' for help."


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
