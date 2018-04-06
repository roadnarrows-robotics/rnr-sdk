################################################################################
#
# Rules.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Master file for defining rules for make targets, variables, and macros.

Include this file in each local make file (usually near the bottom).

\pkgsynopsis
RN Make System

\pkgfile{Rules.mk}

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

_RULES_MK = 1

#------------------------------------------------------------------------------
# Prelims

# this makefile is last in the list (must call before any includes from this)
RNMAKE_ROOT = $(realpath $(dir $(lastword $(MAKEFILE_LIST))))

# version of these makefile rules
RNMAKE_RULES_VER_MAJOR := 3
RNMAKE_RULES_VER_MINOR := 0

# default goal
.DEFAULT_GOAL := all

# list of command-line goals
GOAL_LIST = $(MAKECMDGOALS)

# add default if empty
ifeq "$(GOAL_LIST)" ""
  GOAL_LIST = $(.DEFAULT_GOAL)
endif

# save first and last goals
FIRST_GOAL	= $(firstword $(GOAL_LIST))
LAST_GOAL 	= $(lastword $(GOAL_LIST))

# list of goals with subdirectory traversals
GOALS_WITH_SUBDIRS = 


#------------------------------------------------------------------------------
# Environment (Env.mk)
#
# Parse rnmake specific command-line and environment variables.

ifeq ($(_ENV_MK),)
include $(RNMAKE_ROOT)/Env.mk
endif

#------------------------------------------------------------------------------
# Standard collection of RN Make System functions and varibbles (Std.mk)

ifeq ($(_STD_MK),)
include $(RNMAKE_ROOT)/Std.mk
endif

#------------------------------------------------------------------------------
# Print help (Help.mk)
#
# Check if any of the make goals contain help goals. If true, include the
# help makefile, which defines the help[-<subhelp>] rules.

$(call includeIfGoals,help help-%,$(RNMAKE_ROOT)/Help.mk)

#------------------------------------------------------------------------------
# Compile and run unit tests (Rules.test.mk)
#
# Check if any of the make goals contain test goals. If true, include the
# test makefile, which defines the [run-]test rules.

$(call includeIfGoals,test run-test,$(RNMAKE_ROOT)/Rules.test.mk)

#------------------------------------------------------------------------------
# Documentation makes (Rules.doc.mk)
#
# Check if any of the make goals contain documentation goals. If true, include
# the doc package makefile, which defines the documents and docs-<type> rules.

$(call includeIfGoals,install install-docs documents docs-%,\
	$(RNMAKE_ROOT)/Rules.doc.mk)

#------------------------------------------------------------------------------
# Tarball package repo builds (Rules.tarball.mk)
#
# Check if any of the make goals contain tarball goals. If true, include the
# tarball package makefile, which defines the tarballs and tarball-<type> rules.

$(call includeIfGoals,tarballs tarball-%,$(RNMAKE_ROOT)/Rules.tarball.mk)

#------------------------------------------------------------------------------
# Debian package repo builds (Rules.dpkg.mk)
#
# Check if any of the make goals contain debian package goals. If true,
# include the debian package makefile, which defines the deb-pkgs and
# deb-pkg-<type> rules.

$(call includeIfGoals,deb-pkgs deb-pkg-%,$(RNMAKE_ROOT)/Rules.dpkg.mk)


# -------------------------------------------------------------------------
# Architecture Dependent Definitions

# Standard rnmake rules do not support the following targets.
ifneq "$(findstring $(RNMAKE_ARCH_TAG),atmega16)" ""
  $(error Rules.mk does not support $(arch) rules)
endif

# architecture makefile name
RNMAKE_ARCH_MKFILE = $(call findReqFile,\
  $(RNMAKE_ROOT)/Arch/Arch.$(RNMAKE_ARCH_TAG).mk,\
  See $(RNMAKE_ROOT)/Arch for available architectures)

# Include the architecture make file.
ifeq ($(_ARCH_$(RNMAKE_ARCH_TAG)_MK),)
  include $(RNMAKE_ARCH_MKFILE)
endif

# Included architecture makefile must define RNMAKE_ARCH which defines the
# real architecture.
ifeq ($(RNMAKE_ARCH),)
  $(error 'RNMAKE_ARCH': Not defined in including arhitecture makefile)
endif

#------------------------------------------------------------------------------
# Product Makefile (Optional)

ifdef RNMAKE_PROD_MKFILE
  ifeq ($(_PROD_MK),)
    # optionally include (no error if not found)
    -include $(RNMAKE_PROD_MKFILE)
  endif
endif

#------------------------------------------------------------------------------
# Package Master Makefile (required)
#

# must be defined in including makefile
ifeq ($(RNMAKE_PKG_ROOT),)
  $(error 'RNMAKE_PKG_ROOT': Not defined in including Makefile)
endif

# package makefile
RNMAKE_PKG_MKFILE = $(call findReqFile,$(RNMAKE_PKG_ROOT)/make/Pkg.mk,)

# package root absolute path
RNMAKE_PKG_ROOT := $(realpath $(RNMAKE_PKG_ROOT))

# required
ifeq ($(_PKG_MK),)
include $(RNMAKE_PKG_MKFILE)
endif

# Included package master makefile must define RNMAKE_PKG which defines the
# package name.
ifeq ($(RNMAKE_PKG),)
  $(error 'RNMAKE_PKG': Not defined in including arhitecture makefile)
endif


#------------------------------------------------------------------------------
# Include helper make files (Cmds.mk, Colors.mk)
#
# Can conditionally define macros by architecuture definitions included above.
#

# basic host commands
ifeq ($(_CMDS_MK),)
include $(RNMAKE_ROOT)/Cmds.mk
endif

# color schemes
ifeq ($(_COLORS_MK),)
ifneq "$(color)" "off"
include $(RNMAKE_ROOT)/Colors.mk
endif
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

DIST_ROOT	= $(RNMAKE_PKG_ROOT)/dist
DIST_ARCH = $(DIST_ROOT)/dist.$(RNMAKE_ARCH)

# Distributions Directories
DISTDIR_BIN     = $(DIST_ARCH)/bin
DISTDIR_LIB 		= $(DIST_ARCH)/lib

DISTDIR_INCLUDE = $(DIST_ARCH)/include
DISTDIR_ETC     = $(DIST_ARCH)/etc
DISTDIR_MAN     = $(DIST_ARCH)/man
DISTDIR_SHARE   = $(DIST_ARCH)/share/$(RNMAKE_PKG_FULL_NAME)
DISTDIR_DOC     = $(DIST_ARCH)/doc/$(RNMAKE_PKG_FULL_NAME)-doc
DISTDIR_SRC     = $(DIST_ARCH)/src/$(RNMAKE_PKG_FULL_NAME)
DISTDIR_REPO    = $(DIST_ARCH)/repo
DISTDIR_TMP     = $(DIST_ARCH)/tmp
DISTDIR_LIST    = $(DISTDIR_BIN) \
                  $(DISTDIR_INCLUDE) \
                  $(DISTDIR_LIB) \
                  $(DISTDIR_ETC) \
                  $(DISTDIR_SHARE) \
                  $(DISTDIR_DOC) \
                  $(DISTDIR_SRC) \
                  $(DISTDIR_REPO) \
                  $(DISTDIR_TMP) \
                  $(DISTDIR_MAN)

# documentation subdirectories
DIST_SRCDOC					= srcdoc
DISTDIR_DOC_SRC			= $(DISTDIR_DOC)/$(DIST_SRCDOC)
DISTDIR_DOC_SRC_IMG	= $(DISTDIR_DOC_SRC)/images

# distribution linker loader library directories
DIST_LD_LIBDIRS = $(DISTDIR_LIB) \
									$(addprefix $(DISTDIR_LIB)/,$(RNMAKE_PKG_LIB_SUBDIRS))

# virtual library path
DIST_VPATH_LIB = $(call makePath,$(DIST_LD_LIBDIRS))


#------------------------------------------------------------------------------
# Local Directories (Architecture Dependent)
#
RNMAKE_LOCDIR_ROOT ?= $(RNMAKE_PKG_ROOT)/loc

LOCDIR_BIN			= $(RNMAKE_LOCDIR_ROOT)/bin.$(RNMAKE_ARCH)
LOCDIR_LIB			= $(RNMAKE_LOCDIR_ROOT)/lib.$(RNMAKE_ARCH)
LOCDIR_INCLUDE	= $(RNMAKE_LOCDIR_ROOT)/include.$(RNMAKE_ARCH)
LOCDIR_LIST			= $(LOCDIR_BIN) \
									$(LOCDIR_LIB) \
									$(LOCDIR_INCLUDE)

LOC_VPATH_LIB		= $(LOCDIR_LIB)
LOC_LD_LIBDIRS 	= $(LOCDIR_LIB)

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
EXTRA_INCLUDES				= $(addprefix -I,$(EXTRA_INCDIRS))
EXTRA_SYS_INCLUDES		= $(addprefix -I,$(EXTRA_SYS_INCDIRS))
PKG_INCLUDES					= $(addprefix -I,$(RNMAKE_PKG_INCDIRS))
PROD_INCLUDES					= $(addprefix -I,$(RNMAKE_PROD_INCDIRS))
RNMAKE_ARCH_INCLUDES 	= $(addprefix -I,$(ARCH_INCDIRS))
PKG_SYS_INCLUDES			= $(addprefix -I,$(RNMAKE_PKG_SYS_INCDIRS))
DIST_INCLUDES					= -I$(DISTDIR_INCLUDE)
LOC_INCLUDES					= -I$(LOCDIR_INCLUDE)

INCLUDES = -I. \
						$(EXTRA_INCLUDES) \
						$(PKG_INCLUDES) \
						$(PROD_INCLUDES) \
						$(ARCH_INCLUDES) \
						$(DIST_INCLUDES) \
						$(LOC_INCLUDES) \
						-I$(includedir) \
						$(EXTRA_SYS_INCLUDES) \
						$(RNMAKE_PKG_SYS_INCLUDES)

# CPP Flags
override CPPFLAGS	:= 	$(EXTRA_CPPFLAGS) \
											$(RNMAKE_PKG_CPPFLAGS) \
											$(RNMAKE_ARCH_CPPFLAGS) \
											-DARCH_$(RNMAKE_ARCH) \
											-DARCH="\"$(RNMAKE_ARCH)\"" \
											$(CPPFLAGS)

# C Flags
override CFLAGS	:=	$(EXTRA_CFLAGS) \
										$(RNMAKE_PKG_CFLAGS) \
										$(RNMAKE_ARCH_CFLAGS)\
									 	$(CFLAGS)

# CXX Flags
override CXXFLAGS	:=	$(EXTRA_CXXFLAGS) \
											$(RNMAKE_PKG_CXXFLAGS) \
											$(RNMAKE_ARCH_CXXFLAGS) \
											$(CXXFLAGS)

# Library Path Flags
EXTRA_LD_LIBPATHS	= $(addprefix -L,$(EXTRA_LD_LIBDIRS))
PKG_LD_LIBPATHS		= $(addprefix -L,$(RNMAKE_PKG_LD_LIBDIRS))
LOC_LD_LIBPATHS		=	$(addprefix -L,$(LOC_LD_LIBDIRS))
DIST_LD_LIBPATHS	= $(addprefix -L,$(DIST_LD_LIBDIRS))
LD_LIBPATHS			 := $(EXTRA_LD_LIBPATHS) \
									 	 $(RNMAKE_PKG_LD_LIBPATHS) \
									 	 $(LOC_LD_LIBPATHS) \
										 $(DIST_LD_LIBPATHS) \
										-L$(libdir) \
										-L$(libdir)/rnr \
										 $(LD_LIBPATHS)

# DHP/RDK libdir/rnr doesn't really belong here :-( please fix.

# External Libraries
LD_LIBS						:= $(EXTRA_LD_LIBS) $(PRNMAKE_KG_LD_LIBS) $(LD_LIBS)

LDFLAGS     			:= $(EXTRA_LDFLAGS) $(RNMAKE_PKG_LDFLAGS) $(LDFLAGS)

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
LIB_TYPE :=	$(LIB_TYPE)

# Complete list of core libraries
STLIBS = $(RNMAKE_LOC_STLIBS) $(RNMAKE_DIST_STLIBS)
SHLIBS = $(RNMAKE_DIST_SHLIBS)
DLLIBS = $(RNMAKE_DIST_DLLIBS)

# Complete list of core programs
PGMS = $(RNMAKE_LOC_PGMS) $(RNMAKE_DIST_PGMS)

# Release files
REL_FILES	= $(RNMAKE_PKG_REL_FILES) $(EXTRA_REL_FILES)

# Share make targets
SHARE_TGT	= $(RNMAKE_PKG_TGT_SHARE) $(EXTRA_TGT_SHARE)

# Etc make targets
ETC_TGT = $(RNMAKE_PKG_TGT_ETC) $(EXTRA_TGT_ETC)

# $(call fq_lib_names,dir,lib...,libprefix,libsuffix)
# 	Make fully qualified library name(s) from core name(s).
fq_lib_names = \
$(foreach lib,$(2),$(if $($(lib).SUBDIR),\
	$(addprefix $(1)/$($(lib).SUBDIR)/$(3),$(addsuffix $(4),$(lib))),\
	$(addprefix $(1)/$(3),$(addsuffix $(4),$(lib)))))

# $(call fq_loc_stlib_name,dir,libs)
# 	Make fully qualified local static library name(s) from core name(s). Note
# 	that local libraries do not support library subdirecties.
fq_loc_stlib_names 	=	$(addprefix $(1)/$(STLIB_PREFIX),\
											$(addsuffix $(STLIB_SUFFIX),$(2)))

# $(call fq_stlib_names,dir,lib...)
# 	Make fully qualified static library name(s) from core name(s).
# 	Example:
# 		panda.SUBDIR = fu
# 		fqlibs = $(call fq_stlib_names,my/lib,kung panda)
#
#			Sets 'fqlibs' to 'my/libs/libkung.a my/libs/fu/libpanda.a'
fq_stlib_names = $(call fq_lib_names,$(1),$(2),$(STLIB_PREFIX),$(STLIB_SUFFIX))

# $(call fq_shlib_names,dir,lib...)
# 	Make fully qualified shared library name(s) from core name(s).
# 	Example:
# 		lex.SUBDIR = villian
# 		fqlibs = $(call fq_shlib_names,/home/lib,supergirl lex)
#
#			Sets 'fqlibs' to '/home/lib/libsupergirl.so /home/lib/villian/liblex.so'
fq_shlib_names = $(call fq_lib_names,$(1),$(2),$(SHLIB_PREFIX),$(SHLIB_SUFFIX))

# $(call fq_dllib_names,dir,lib...)
# 	Make fully qualified dynamically linked library name(s) from core name(s).
# 	Example:
# 		falcon9.SUBDIR = spacex
# 		newshepard.SUBDIR = blueorigin
# 		fqlibs = $(call fq_dllib_names,lib,falcon9 newshepard)
#
#		Sets 'fqlibs' to '/lib/spacex/libfalcon9.so lib/blueorign/libnewshepard.so'
fq_dllib_names = \
  $(call fq_lib_names,$(1),$(2),$(DLLIB_PREFIX),$(DLLIB_SUFFIX))

# Fully Qualified Static Libraries
FQ_STLIBS	= $(call fq_loc_stlib_names,$(LOCDIR_LIB),$(RNMAKE_LOC_STLIBS)) \
						$(call fq_stlib_names,$(DISTDIR_LIB),$(RNMAKE_DIST_STLIBS))

# Fully Qualified Shared Libraries
FQ_SHLIBS	= $(call fq_shlib_names,$(DISTDIR_LIB),$(RNMAKE_DIST_SHLIBS))

# Fully Qualified Dynamically Linked Libraries
FQ_DLLIBS	= $(call fq_dllib_names,$(DISTDIR_LIB),$(RNMAKE_DIST_DLLIBS))

# Fully Qualified Program Name(s) from Core Name(s)
fq_pgm_names = $(addprefix $(1)/$(PGM_PREFIX),\
									$(addsuffix $(PGM_SUFFIX),$(2)))

# Fully Qualified Programs
FQ_PGMS	= $(call fq_pgm_names,$(LOCDIR_BIN),$(RNMAKE_LOC_PGMS)) \
					$(call fq_pgm_names,$(DISTDIR_BIN),$(RNMAKE_DIST_PGMS))

# Release Files
FQ_REL_FILES = $(addprefix $(DISTDIR_DOC)/,$(REL_FILES))

# Auto-Generated header files directory
AUTO_INCDIR = $(firstword $(RNMAKE_PKG_INCDIRS))
ifeq ($(AUTO_INCDIR),)
  AUTO_INCDIR = $(LOCDIR_INCLUDE)
endif

# Auto-Generated Header Files
AUTO_VERSION_H = $(AUTO_INCDIR)/version.h
AUTO_INSTALL_H = $(AUTO_INCDIR)/install-$(RNMAKE_ARCH).h

AUTOHDRS	= $(AUTO_VERSION_H)

# DEPRECATED $(AUTO_INSTALL_H)


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
all: pkgbanner check-deps once-all $(EXTRA_TGT_ALL) pkg subdirs-all $(EXTRA_TGT_ALL_POST)
	$(footer)
else
all: check-deps $(EXTRA_TGT_ALL) pkg subdirs-all $(EXTRA_TGT_ALL_POST)
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
.PHONY: $(RNMAKE_DIST_STLIBS)
$(RNMAKE_DIST_STLIBS): $(call fq_stlib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific distribution shared librarary
.PHONY: $(RNMAKE_DIST_SHLIBS)
$(RNMAKE_DIST_SHLIBS): $(call fq_shlib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific distribution dll librarary
.PHONY: $(RNMAKE_DIST_DLLIBS)
$(RNMAKE_DIST_DLLIBS): $(call fq_dllib_names,$(DISTDIR_LIB),$(GOAL_LIST))

# Make specific local static librarary
.PHONY: $(RNMAKE_LOC_STLIBS)
$(RNMAKE_LOC_STLIBS): $(call fq_stlib_names,$(LOCDIR_LIB),$(GOAL_LIST))

# $(call STLIBtemplate,lib,libdir)
# Template to build a static library including all necessary prerequisites
define STLIBtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).FQ_LIB = $(call fq_stlib_names,$(2),$(1))
 OUTDIR = $$(dir $$($(1).FQ_LIB))
 $$($(1).FQ_LIB): $$($(1).OBJS)
	@printf "\n"
	@printf "$(color_tgt_lib)     $$@$(color_end)\n"
	@test -d "$$(OUTDIR)" || $(MKDIR) $$(OUTDIR)
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
	@test -d "$$(OUTDIR)" || $(MKDIR) $$(OUTDIR)
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
	@test -d "$$(OUTDIR)" || $(MKDIR) $$(OUTDIR)
	$$(DLLIB_LD) $$(DLLIB_LD_FLAGS) $$(DLLIB_LD_EXTRAS) $$($(1).OBJS) $$(LD_LIBPATHS) $$($(1).LIBS) $$(LD_LIBS) -o $$@
endef

# For each library target, evaluate (i.e make) the template.
$(foreach lib,$(RNMAKE_LOC_STLIBS),\
  $(eval $(call STLIBtemplate,$(lib),$(LOCDIR_LIB))))

$(foreach lib,$(RNMAKE_DIST_STLIBS),\
	$(eval $(call STLIBtemplate,$(lib),$(DISTDIR_LIB))))

$(foreach lib,$(RNMAKE_DIST_SHLIBS),\
	$(eval $(call SHLIBtemplate,$(lib),$(DISTDIR_LIB))))

$(foreach lib,$(RNMAKE_DIST_DLLIBS),\
	$(eval $(call DLLIBtemplate,$(lib),$(DISTDIR_LIB))))

# -------------------------------------------------------------------------
# Target:	pgms
# Desc: 	Makes all programs in current directory.

# Make all programs
.PHONY: pgms
pgms: $(FQ_PGMS)

# Make specific local program
.PHONY: $(RNMAKE_LOC_PGMS)
$(RNMAKE_LOC_PGMS): $(call fq_pgm_names,$(LOCDIR_BIN),$(GOAL_LIST))

# Make specific distribution program
.PHONY: $(RNMAKE_DIST_PGMS)
$(RNMAKE_DIST_PGMS): $(call fq_pgm_names,$(DISTDIR_BIN),$(GOAL_LIST))

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
$(foreach prog,$(RNMAKE_LOC_PGMS),\
  $(eval $(call PGMtemplate,$(prog),$(LOCDIR_BIN))))

$(foreach prog,$(RNMAKE_DIST_PGMS),\
	$(eval $(call PGMtemplate,$(prog),$(DISTDIR_BIN))))

## RDK investigate for linking programs
#	$(LINK.o) $^ $(LDLIBS) -o $@

# -------------------------------------------------------------------------
# Target:	autohdrs
# Desc: 	Makes auto-generated header files.
# Notes:	Auto-headers are only made at top level.
#  	      May make a shell script to do this.
autohdrs: $(AUTOHDRS)

# verion.h auto-generated header
$(AUTO_VERSION_H): $(RNMAKE_PKG_MKFILE)
	$(printTgtGoal)
	@test -d "$(AUTO_INCDIR)" || $(MKDIR) $(AUTO_INCDIR)
	@$(MAKE) -f $(RNMAKE_ROOT)/version_h.mk -s \
		RNMAKE_PKG_ROOT=$(RNMAKE_PKG_ROOT) \
		version_h=$(@) \
		pkg_mk=$(RNMAKE_PKG_MKFILE) \
		autogen

# install.h auto-generated header DEPRECATED
$(AUTO_INSTALL_H): $(RNMAKE_ARCH_MKFILE)
	@test -d "$(AUTO_INCDIR)" || $(MKDIR) $(AUTO_INCDIR)
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
HDR_TAG_LIST = $(addsuffix .HDRS.H,$(RNMAKE_DIST_HDRS))
#$(info HDR_TAG_LIST: $(HDR_TAG_LIST))

# Complete list of headers
PREREQ_HDRS = $(foreach tag,$(HDR_TAG_LIST),$($(tag)))
#$(info PREREQ_HDRS: $(PREREQ_HDRS))

# Make all distribution headers
.PHONY: hdrs
hdrs: echo-hdrs $(PREREQ_HDRS)

.PHONY: echo-hdrs
echo-hdrs:
	$(call printGoalDesc,$(DISTDIR_INCLUDE),\
		Copying tagged interfaces headers from $(RNMAKE_PKG_ROOT)/include.)

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
	@echo "$(RNMAKE_PKG) v$(RNMAKE_PKG_VERSION_DOTTED)"  > $@
	@echo "Copyright (C) $(RNMAKE_PKG_VERSION_DATE) RoadNarrows LLC" >> $@
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
# Target:	mkdistdirs
# Desc: 	Make Distribution Directories 
.PHONY: mkdistdirs $(DISTDIR_LIST)
mkdistdirs: echo-mkdistdirs $(DISTDIR_LIST)

.PHONY: echo-mkdistdirs
echo-mkdistdirs:
ifeq ($(realpath $(DIST_ARCH)),)
	$(call printGoalDesc,\
		$(patsubst $(RNMAKE_PKG_ROOT)/%,%,$(DIST_ARCH)),\
		Making distribution directories.)
endif

$(DISTDIR_LIST):
	@test -d ${@} || $(MKDIR) ${@}


# -------------------------------------------------------------------------
# Target:	mklocdirs
# Desc: 	Make Local Directories 
.PHONY: mklocdirs $(LOCDIR_LIST)
mklocdirs: echo-mklocdirs $(LOCDIR_LIST)

.PHONY: echo-mklocdirs
echo-mklocdirs:
ifeq ($(realpath $(LOCDIR_BIN)),)
	$(call printGoalDesc,\
		$(patsubst $(RNMAKE_PKG_ROOT)/%,%,$(RNMAKE_LOCDIR_ROOT)),\
		Making local directories.)
endif

$(LOCDIR_LIST):
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
	$(call printEchoTgtGoalDesc,Installing package $(RNMAKE_PKG_FULL_NAME))

# DEPRECATED
instest:
	@if [ ! -f $(DISTDIR_DOC)/VERSION.txt ]; \
	then \
		echo "Error: The $(RNMAKE_PKG) package has not been built. Try \"make\" first."; \
		echo "Install aborted."; \
		exit 4; \
	fi

# install bin
install-bin:
	$(printTgtGoal)
	@printf "Installing executables to $(bindir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 755 $(DISTDIR_BIN) $(bindir)
	$(call POSTINStemplate,$(STRIP_EXE),$(DISTDIR_BIN),$(bindir))

# install lib
install-lib:
	$(printTgtGoal)
	@printf "Installing libraries to $(libdir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 755 $(DISTDIR_LIB) $(libdir)
	$(call POSTINStemplate,$(STRIP_LIB),$(DISTDIR_LIB),$(libdir))

# install includes
install-includes: hdrs
	$(printTgtGoal)
	@printf "Installing includes to $(includedir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh 664 $(DISTDIR_INCLUDE) $(includedir)

# install documentation
install-docs: documents
	$(printTgtGoal)
	@printf "Installing documents to $(docdir)/$(RNMAKE_PKG_FULL_NAME)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh -s 664 $(DISTDIR_DOC) $(docdir)/$(RNMAKE_PKG_FULL_NAME)

# install share files
install-share:
	$(printTgtGoal)
	@printf "Installing system share files to $(sharedir)\n"
	@$(RNMAKE_ROOT)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)/$(RNMAKE_PKG_FULL_NAME)
	@if [ ! -e $(sharedir)/$(RNMAKE_PKG) ]; \
	then \
		$(SYMLINK) $(sharedir)/$(RNMAKE_PKG_FULL_NAME) $(sharedir)/$(RNMAKE_PKG); \
	elif [ -L $(sharedir)/$(RNMAKE_PKG) ]; \
	then \
		$(UNLINK) $(sharedir)/$(RNMAKE_PKG); \
		$(SYMLINK) $(sharedir)/$(RNMAKE_PKG_FULL_NAME) $(sharedir)/$(RNMAKE_PKG); \
	fi

# install etc
install-etc:
	$(printTgtGoal)
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
# Target:	deps
# Desc: 	Makes dependencies
# File: 	.deps/deps.$(RNMAKE_ARCH)
# -------------------------------------------------------------------------
.PHONY: deps
deps: pkgbanner echo-deps autohdrs mkdepsdir $(EXTRA_TGT_DEPS) hdrdeps libdeps \
			subdirs-deps
	$(footer)

.PHONY: echo-deps
echo-deps:
	$(call printEchoTgtGoalDesc,Making dependencies for $(CURDIR))

.PHONY: hdrdeps
hdrdeps: 
	@$(call printGoalDesc,$(@),Making C/C++ dependencies)
	@echo $(call hdrdeps_sh,$(STLIBS) $(SHLIBS) $(DLLIBS) $(PGMS))

hdrdeps_sh = \
	$(shell $(RNMAKE_ROOT)/utils/hdrdeps.sh \
		-c "$(RNMAKE_MAKEDEPS)" \
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

# Check if deps file exist or error.
# If the override variable 'nodeps' is not empty, then no check is performed.
define checkDeps
	$(if $(nodeps),,\
		$(if $(realpath $(DEPSFILE)),,\
			$(error No dependencies file - Try 'make deps' first)))
endef

# Include dependency file or error canned sequence.
# If the override variable 'nodeps' is not empty, then no action is performed.
define includeDeps
	$(if $(nodeps),,\
		$(if $(call isFile,$(DEPSFILE)),\
			$(eval include $(DEPSFILE)),\
	$(error No dependencies file - Try 'make deps' first)))
endef

# Conditionally include any dependency file for specific targets only.
$(if $(call findGoals,all test),$(call includeDeps),)

# Check deps target
.PHONY: check-deps
check-deps:
	$(call checkDeps)


# -------------------------------------------------------------------------
# Target:	clean
# Desc: 	Deletes generated intermediate files
# -------------------------------------------------------------------------
.PHONY: clean
clean: pkgbanner echo-clean do-clean $(EXTRA_TGT_CLEAN) subdirs-clean
	$(footer)

.PHONY: echo-clean
echo-clean:
	$(call printEchoTgtGoalDesc,)

.PHONY: do-clean
do-clean:
	@echo "Cleaning $(CURDIR)"
	$(RM) *.o *.ii *.c~ *.cxx~ .h~ *.pyc *.pyo \
		$(RNMAKE_LOC_PGMS) $(RNMAKE_DIST_PGMS) a.out doxy.*.log
	$(RM) $(OBJDIR)


# -------------------------------------------------------------------------
# Target:	distclean (clobber)
# Desc: 	Cleans plus deletes distribution
# -------------------------------------------------------------------------
.PHONY: distclean clobber
distclean clobber: pkgbanner clean $(EXTRA_TGT_DISTCLEAN)
	$(call printGoalDesc,$(@),Clobbering distribution $(CURDIR))
	$(RM) $(DIST_ARCH)
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-$(RNMAKE_ARCH).tar.gz
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-doc.tar.gz
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-src.tar.gz
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
.PHONY: subdirs $(RNMAKE_SUBDIRS)
subdirs: $(RNMAKE_SUBDIRS)

# Make all sub-directories with all command-line goals
$(RNMAKE_SUBDIRS):
	$(call printDirBanner,$(@),$(GOAL_LIST))
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C $(@) $(GOAL_LIST)
	@printf "    $(color_dir_banner)~~$(color_end)\n"

#
# Template to build subdirectories by goal rules. Since rnmake traverses the
# command-line goals depth first, and GNU make will only execute a rule once,
# this template builds unique subdirectory rules.
#
# Built Rules:
#   subdirs-<goal>: $(RNMAKE_SUBDIRS.<goal>)
#   $(RNMAKE_SUBDIRS.<goal>:
#   	<recipe>
#
# Usage: $(call SUBDIRtemplate,goal)
#
define SUBDIRtemplate
RNMAKE_SUBDIRS.$(1) = $(addsuffix .$(1),$(RNMAKE_SUBDIRS))

subdirs-$(1): $$(RNMAKE_SUBDIRS.$(1))

$$(RNMAKE_SUBDIRS.$(1)):
	$$(call printDirBanner,$$(basename $$(@)),$(1))
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

# Pretty print make banner.
.PHONY: pkgbanner
pkgbanner:
	$(call printPkgBanner,$(RNMAKE_PKG_FULL_NAME),$(RNMAKE_ARCH),$(GOAL_LIST))

# $(footer)
# 	Conditionally print footer.
footer = $(call printFooter,$(@),$(LAST_GOAL))


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
