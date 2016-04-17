################################################################################
#
# Package: 	RN Make System
#
# File:			Rules.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Master file for defining rules for make targets, variables,
       and macros.

Include this file into each local make file (usually at the bottom).\n

The rnmake system inclusion order:
\termblock
\term 1.
  \termdata Makefile \termdata \h_leftdblarrow
  \termdata \$(rnmake)/Rules.mk
  \termdata Local make file includes toplevel rnmake rules file.
\endterm
\term 2.
  \termdata Rules.mk \termdata \h_leftdblarrow
  \termdata \$(pkgroot)/../make/Prod.mk
  \termdata Optional local product make file.\n
            Override location by defining <em>prod_mk</em>.
\endterm
\term 3.
  \termdata Rules.mk \termdata \h_leftdblarrow
  \termdata \$(pkgroot)/make/Pkg.mk
  \termdata Required local package make file.\n
            Override location by defining <em>pkg_mk</em>.
\endterm
\term 4.
  \termdata Rules.mk \termdata \h_leftdblarrow
  \termdata \$(rnmake)/Arch/Arch.<em>arch</em>.mk
  \termdata Appropriate architecture make file defining target
            native/cross-compile tools chains and libraries.\n
						Defined on the command-line as arch=<em>arch</em>\n
            Default: <b>RNMAKE_ARCH_DFT</b> if defined, else "x86_64"
\endterm
\term 5.
  \termdata Rules.mk \termdata \h_leftdblarrow
  \termdata \ref Cmds.mk "\$(rnmake)/Cmds.mk"
  \termdata Build host basic support commands.
\endterm
\endtermblock

\par Usage:
	make [arch=<em>arch</em>] [color=scheme] [<em>target</em> ...]\n
  See 'make help' for more details.
          
\todo
1. Add make check target

$LastChangedDate: 2016-02-03 12:51:32 -0700 (Wed, 03 Feb 2016) $
$Rev: 4301 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2010.  RoadNarrows LLC.
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

#------------------------------------------------------------------------------
# Prelims
ifeq "$(MAKECMDGOALS)" ""
	GOAL	= all
else
	GOAL = $(firstword $(MAKECMDGOALS))
endif

#------------------------------------------------------------------------------
# Print Help and Quit
# Note: First make goal is tested for the substring 'help*'. If matched, then
# include the help make file which should catch the help[-<subhelp>] target.
ifeq "$(findstring help,$(GOAL))" "help"
ifndef rnmake
rnmake = $(dir $(lastword $(MAKEFILE_LIST)))
endif
include $(rnmake)/Help.mk
endif

# Default command-line architecture, if not specified
ifndef arch
ifdef RNMAKE_ARCH_DFT
	  arch=$(RNMAKE_ARCH_DFT)
else
    arch = x86_64
endif
endif

#------------------------------------------------------------------------------
# Compile and run unit tests.
# Note: First make goal is tested for the substring 'test*'. If matched, then
# include the test make file which should catch the test[-<subtest>] target.
include $(rnmake)/Test.mk



#------------------------------------------------------------------------------
# Tweaks
# 
# See also the standard prefix, ... parameters

#
# pkgroot		- specifies package root directory
#
# Including Makefile must define package root prior to including Rules.mk
#
ifndef pkgroot
$(error Error: pkgroot: not defined in including Makefile)
endif

# topdir 		- specifies the product/project top directory.
#
# For RoadNarrows developers, the default translates to /prj
#
ifndef topdir
	topdir = $(realpath $(pkgroot)/../..)
endif

# rnmake 		- specifies the	RN make system base directory
#
ifndef rnmake
	rnmake = $(realpath $(pkgroot)/../rnmake)
endif

# distroot 	- specifies the root directory for the intermediary distribution
# 						files
#
ifndef distroot
	distroot = $(pkgroot)/dist
endif


#------------------------------------------------------------------------------
# Product Makefile (Optional)
#
ifdef prod_mk
PROD_MKFILE = $(prod_mk)
else
PROD_MKFILE = $(pkgroot)/../make/Prod.mk
endif

# optionally include
-include $(PROD_MKFILE)



#------------------------------------------------------------------------------
# Package Makefile
#
ifdef pkg_mk
PKG_MKFILE = $(pkg_mk)
else
PKG_MKFILE = $(pkgroot)/make/Pkg.mk
endif

include $(PKG_MKFILE)

ifndef PKG
$(error Error: PKG: not defined: specify in Pkg.mk)
endif

# Package root absolute path name
ifndef PKG_ROOT
export PKG_ROOT := $(realpath $(CURDIR)/$(pkgroot))
endif


# -------------------------------------------------------------------------
# Architecture Dependent Definitions

# Stanard rules does not support the following targets.
ifneq "$(findstring $(arch),atmega16)" ""
$(error Error: Rules.mk does not support $(arch) rules)
endif

# architecture make file name
ARCH_MKFILE = $(rnmake)/Arch/Arch.$(arch).mk

# check to see if architecture file exists
hasArch := $(shell if [ -f $(ARCH_MKFILE) ]; then echo "true"; fi)
ifndef hasArch
$(error Error: Unknown architecture: $(arch). See $(rnmake)/Arch/Arch.<arch>.mk)
endif

# supported architectures - default is all architectures
# RDK No way to exit submake without aborting calling make 
#ifdef ARCH_SUPPORTED
#supArch := $(findstring $(arch),$(ARCH_SUPPORTED))
#ifndef supArch
#$(shell $(MAKE) -f $(rnmake)/null.mk --stop arch=$(arch) null 1>&2)
#endif
#endif

# include the architecture make file
include $(ARCH_MKFILE)

# Included architecture makefile must define ARCH which "overrides" the
# command-line arch in the rules targets.
#
ifndef ARCH
$(error Error: ARCH: not defined in including Makefile)
endif


#------------------------------------------------------------------------------
# Include helper make files
# Can conditionally define macros by architecuture definitions included
# above.
#

# basic host commands
include $(rnmake)/Cmds.mk

# color schemes
ifneq "$(color)" "off"
include $(rnmake)/Colors.mk
endif


#------------------------------------------------------------------------------
# Install Directories - Override as necessary in including Makefile.
# See Also: Arch.<arch>.mk.
# Note: These are traditional configuration names - keep the naming convention.
#
ifndef prefix
prefix					= /usr/local
endif

ifndef exec_prefix
exec_prefix     = $(prefix)
endif

ifndef bindir
bindir          = $(exec_prefix)/bin
endif

ifndef sbindir
sbindir         = $(exec_prefix)/sbin
endif

ifndef libexecdir
libexecdir      = $(exec_prefix)/libexec
endif

ifndef sysconfdir
sysconfdir      = $(prefix)/etc
endif

ifndef localstatedir
localstatedir   = $(prefix)/var
endif

ifndef libdir
libdir          = $(exec_prefix)/lib
endif

ifndef includedir
includedir      = $(prefix)/include
endif

ifndef sharedir
sharedir        = $(prefix)/share
endif

ifndef infodir
infodir         = $(prefix)/info
endif

ifndef docdir
docdir          = $(prefix)/share/doc
endif

ifndef mandir
mandir          = $(prefix)/man
endif


#------------------------------------------------------------------------------
# Distribution Directories (Architecture Dependent)
# Notes:
# 	Documents are architecture independent
#
DIST_ROOT				= $(distroot)
DIST_ARCH       = $(DIST_ROOT)/dist.$(ARCH)

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
DISTDIR_TMP     = $(DIST_ARCH)/tmp/$(DIST_NAME_BIN)-$(ARCH)
DISTDIR_TMP_DEB	= $(DIST_ARCH)/tmp/deb
DISTDIR_LIST    = $(DISTDIR_BIN) \
                  $(DISTDIR_INCLUDE) \
                  $(DISTDIR_LIB) \
                  $(DISTDIR_ETC) \
                  $(DISTDIR_SHARE) \
                  $(DISTDIR_DOC) \
                  $(DISTDIR_SRC) \
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
DIST_TARBALL_BIN		= $(DIST_NAME_BIN)-$(ARCH).tar.gz


#------------------------------------------------------------------------------
# Local Directories (Architecture Dependent)
#
ifndef LOCDIR_ROOT
LOCDIR_ROOT			= $(pkgroot)/loc
endif
LOCDIR_BIN			= $(LOCDIR_ROOT)/bin.$(ARCH)
LOCDIR_LIB			= $(LOCDIR_ROOT)/lib.$(ARCH)
LOCDIR_INCLUDE	= $(pkgroot)/include
LOCDIR_LIST			= $(LOCDIR_BIN) \
									$(LOCDIR_LIB)

LOC_VPATH_LIB		= $(LOCDIR_LIB)
LOC_LD_LIBDIRS 	= $(LOCDIR_LIB)

# Architecture dependent local include directory 
# Note: Since developers can add files to this directory, never delete it.
ARCH_INCDIR			= $(LOCDIR_INCLUDE)/arch/arch.$(ARCH)

# Object Directory
OBJDIR					= obj/obj.$(ARCH)

# Dependencies Directory
DEPSDIR			= .deps

# Dependencies File
DEPSFILE				= $(DEPSDIR)/deps.$(ARCH)


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
											-DARCH_$(ARCH) \
											-DARCH="\"$(ARCH)\"" \
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
ifeq "$(TEST)" "true"
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
AUTOHDRS				= $(addprefix $(LOCDIR_INCLUDE)/,version.h) \
								  $(addprefix $(ARCH_INCDIR)/,install.h)

#------------------------------------------------------------------------------
# Target Specific Variables
#

CURGOAL 	= $(GOAL)
all: 				CURGOAL := all
deps: 			CURGOAL := deps
install: 		CURGOAL := install
clean: 			CURGOAL := clean
distclean: 	CURGOAL := distclean
clobber: 		CURGOAL := clobber

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

# Make obj/obj-<ARCH> in current directory
mkobjdir = \
	@test -d "$(OBJDIR)" || $(MKDIR) $(OBJDIR); \
	test -d "$(dir $(1))" || test -z "$(dir $(1))" || $(MKDIR) "$(dir $(1))"


########################### Explicit Rules #####################################

# -------------------------------------------------------------------------
# Target:	all (default)
# Desc: 	Front end to making the [sub]package(s) (libraries, programs, tools,
# 				documents, etc).
# Notes: 	There are two version:
# 					1) only done once on the first invocation and 
# 					2) for all other invocations.
.PHONY: all onceall
ifndef ONCE
export ONCE = 1
all: onceall $(EXTRA_TGT_ALL) pkg $(EXTRA_TGT_ALL_POST) footer
else
all: $(EXTRA_TGT_ALL) pkg $(EXTRA_TGT_ALL_POST)
endif

onceall: mkdistdirs mklocdirs pkgbanner autohdrs 

# -------------------------------------------------------------------------
# Target:	pkg
# Desc: 	Makes the distribution [sub]package(s) (libraries, programs, tools, 
# 				documents, etc).
.PHONY: pkg
pkg: libs pgms rel share subdirs
# RDK pkg: libs pgms hdrs rel share subdirs


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
$(DIST_STLIBS): $(call fq_stlib_names,$(DISTDIR_LIB),$(MAKECMDGOALS))

# Make specific distribution shared librarary
.PHONY: $(DIST_SHLIBS)
$(DIST_SHLIBS): $(call fq_shlib_names,$(DISTDIR_LIB),$(MAKECMDGOALS))

# Make specific distribution dll librarary
.PHONY: $(DIST_DLLIBS)
$(DIST_DLLIBS): $(call fq_dllib_names,$(DISTDIR_LIB),$(MAKECMDGOALS))

# Make specific local static librarary
.PHONY: $(LOC_STLIBS)
$(LOC_STLIBS): $(call fq_stlib_names,$(LOCDIR_LIB),$(MAKECMDGOALS))

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
$(LOC_PGMS): $(call fq_pgm_names,$(LOCDIR_BIN),$(MAKECMDGOALS))

# Make specific distribution program
.PHONY: $(DIST_PGMS)
$(DIST_PGMS): $(call fq_pgm_names,$(DISTDIR_BIN),$(MAKECMDGOALS))

# Template to build a program including all necessary prerequisites
define PGMtemplate
 $(1).OBJS  = $(call objs_from_src,$(1))
 $(1).LIBDEPS  = $(shell $(rnmake)/utils/libdeps.sh $(LIBS_VPATH) $($(1).LIBDEPS))
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

$(LOCDIR_INCLUDE)/version.h: $(PKG_MKFILE)
	@test -d $(LOCDIR_INCLUDE) || $(MKDIR) $(LOCDIR_INCLUDE)
	@$(MAKE) -f $(rnmake)/version_h.mk -s pkgroot=$(pkgroot) version_h=$@ \
								pkg_mk=$(PKG_MKFILE)

$(ARCH_INCDIR)/install.h: $(ARCH_MKFILE)
	@test -d $(ARCH_INCDIR) || $(MKDIR) $(ARCH_INCDIR)
	@$(MAKE) -f $(rnmake)/install_h.mk -s pkgroot=$(pkgroot) install_h=$@ \
								arch=$(ARCH) \
								bindir=$(bindir) \
								sbindir=$(sbindir) \
								libdir=$(libdir) \
								includedir=$(includedir) \
								sysconfdir=$(sysconfdir) \
								docdir=$(docdir) \
								mandir=$(mandir) \
								infodir=$(infodir)

# Dummies
$(PROD_MKFILE):
$(PKG_MKFILE):
$(ARCH_MKFILE):

# -------------------------------------------------------------------------
# Target:	hdrs
# Desc: 	Makes interface header files

# List of all header tags
HDR_TAG_LIST = $(addsuffix .HDRS.H,$(DIST_HDRS))
#$(warning HDR_TAG_LIST: $(HDR_TAG_LIST))

# Complete list of headers
PREREQ_HDRS = $(foreach tag,$(HDR_TAG_LIST),$($(tag)))
#$(warning PREREQ_HDRS: $(PREREQ_HDRS))

# Make all distribution headers
.PHONY: hdrs
hdrs: $(PREREQ_HDRS)

# Copy newer headers to distribution include [sub]directory
$(PREREQ_HDRS):
	@for h in $(@); \
		do\
			src=$(pkgroot)/include/$$h; \
			dst=$(DISTDIR_INCLUDE)/$$h; \
			hdir=$$(dirname $$dst); \
			if [ ! -f $$dst -o $$src -nt $$dst ]; \
			then \
				echo "     $$dst"; \
				test -d $$hdir || $(MKDIR) $$hdir; \
				$(CP) $$src $$dst; \
			fi; \
		done

# -------------------------------------------------------------------------
# Target:	rel
# Desc: 	Makes all release files
# Notes:	Release files are only made at top level
.PHONY: rel
ifeq "$(pkgroot)" "."
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
ifeq "$(pkgroot)" "."
share: $(SHARE_TGT)
else
share:
endif

# -------------------------------------------------------------------------
# Target:	etc (not used yet)
# Desc: 	Makes all system configuration files
# Notes:	Etc files are only made at top level
.PHONY: etc
ifeq "$(pkgroot)" "."
etc: $(ETC_TGT)
else
etc:
endif

# -------------------------------------------------------------------------
# Target: documents
# Desc:   Recursively make subdirectories.
.PHONY: documents
documents: docs-clean docs-src-gen docs-pub-gen $(EXTRA_TGT_DOC)

# documentation generator from source files
ifndef HTML_HEADER
HTML_HEADER     = $(rnmake)/doxy/rn_doxy_header.html
endif

ifndef HTML_FOOTER
HTML_FOOTER     = $(rnmake)/doxy/rn_doxy_footer.html
endif

ifndef HTML_STYLESHEET
HTML_STYLESHEET = $(rnmake)/doxy/rn_doxy.css
endif

ifndef DOXY_IMAGES
DOXY_IMAGES = $(rnmake)/doxy/rn_images
endif

docs-clean:
	@$(RM) $(DISTDIR_DOC)

# generate doxygen source documetiona
docs-src-gen:
	@if [ "$(DOXY_CONF_FILE)" ]; \
	then \
		echo ""; \
		echo "Making source documentation"; \
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
		) | doxygen - >$(pkgroot)/doxy.out.log 2>$(pkgroot)/doxy.err.log; \
	fi

# This utility scrip is no longer used after doxygen 1.7
#		$(rnmake)/utils/doxyindex.sh \
#							-t "$(PKG) v$(PKG_VERSION_DOTTED)" \
#							-h $(HTML_HEADER) \
#							>$(DISTDIR_DOC)/$(DIST_SRCDOC)/index.html; \

# generate published documentation
docs-pub-gen:
	@pubdstdir="$(DISTDIR_DOC)/papers"; \
	pubsrcdir="$(pkgroot)/docs/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi; \
	pubsrcdir="$(pkgroot)/3rdparty/published"; \
	unset flist; \
	test -d $${pubsrcdir} && flist=$$(ls $${pubsrcdir}); \
	if [ "$${flist}" != "" ]; \
	then \
		test -d $${pubdstdir} || $(MKDIR) $${pubdstdir}; \
		$(CP) -r $${pubsrcdir}/* $${pubdstdir}/. 2>/dev/null; \
	fi


# -------------------------------------------------------------------------
# Target:	mkdistdirs
# Desc: 	Make Distribution Directories 
.PHONY: mkdistdirs $(DISTDIR_LIST)
mkdistdirs: $(DISTDIR_LIST)

$(DISTDIR_LIST):
	@$(MKDIR) $@;

# -------------------------------------------------------------------------
# Target:	mklocdirs
# Desc: 	Make Local Directories 
.PHONY: mklocdirs $(LOCDIR_LIST) $(PKGDIR_INCLUDE)
mklocdirs: $(LOCDIR_LIST) $(PKGDIR_INCLUDE)

$(LOCDIR_LIST) $(PKGDIR_INCLUDE):
	@$(MKDIR) $@;

# -------------------------------------------------------------------------
# Target:	install
# Desc: 	Install the distribution
.PHONY: install
install: 	pkgbanner all $(EXTRA_TGT_INSTALL) install-bin install-lib \
					install-includes install-docs install-share install-etc \
					$(EXTRA_TGT_INSTALL_POST) footer

instest:
	@if [ ! -f $(DISTDIR_DOC)/VERSION.txt ]; \
	then \
		echo "Error: The $(PKG) package has not been built. Try \"make\" first."; \
		echo "Install aborted."; \
		exit 4; \
	fi

# install bin
install-bin:
	@printf "\n"
	@printf "$(color_tgt_file)Installing executables to $(bindir)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh 755 $(DISTDIR_BIN) $(bindir)
	$(call POSTINStemplate,$(STRIP_EXE),$(DISTDIR_BIN),$(bindir))

# install lib
install-lib:
	@printf "\n"
	@printf "$(color_tgt_file)Installing libraries to $(libdir)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh 755 $(DISTROOT_LIB) $(libdir)
	$(call POSTINStemplate,$(STRIP_LIB),$(DISTROOT_LIB),$(libdir))

# install includes
install-includes: hdrs
	@printf "\n"
	@printf "$(color_tgt_file)Installing includes to $(includedir)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh 664 $(DISTDIR_INCLUDE) $(includedir)

# install documentation
install-docs: documents
	@printf "\n"
	@printf "$(color_tgt_file)Installing documents to $(docdir)/$(PKG_FULL_NAME)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_DOC) $(docdir)/$(PKG_FULL_NAME)

# install share files
install-share:
	@printf "\n"
	@printf "$(color_tgt_file)Installing system share files to $(sharedir)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)/$(PKG_FULL_NAME)
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
	@printf "\n"
	@printf "$(color_tgt_file)Installing system configuration to $(sysconfdir)$(color_end)"
	@printf "\n"
	@$(rnmake)/utils/doinstall.sh 664 $(DISTDIR_ETC) $(sysconfdir)

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

.PHONY: tarball-doc
tarball-doc: 
	$(if $(call isdir,$(DISTDIR_DOC)),,\
			    					$(error No documentation - Try 'make documents' first.))
	@cd $(DIST_ARCH)/doc; \
	$(TAR) ../../$(DIST_TARBALL_DOC) $(PKG_FULL_NAME)-doc

.PHONY: tarball-src
tarball-src: 
	@$(RM) $(DISTDIR_SRC)
	@test -d $(DISTDIR_SRC) || $(MKDIR) $(DISTDIR_SRC)
	@$(rnmake)/utils/tarball-src-filter.sh $(pkgroot) | \
	while read src; \
	do \
		$(rnmake)/utils/cppath.sh $$src $(DISTDIR_SRC); \
	done;
	@cd $(DIST_ARCH)/src; \
	$(TAR) ../../$(DIST_TARBALL_SRC) $(PKG_FULL_NAME)

.PHONY: tarball-bin
tarball-bin:
	$(if $(call isdir,$(DIST_ARCH)),,$(error Nothing made - Try 'make' first.))
	@test -d $(DISTDIR_TMP) || $(MKDIR) $(DISTDIR_TMP)
	@cd $(DIST_ARCH); \
	$(FIND) bin lib include etc share -print | \
	while read src; \
	do \
		if [ -f $$src ]; \
		then \
			$(rnmake)/utils/cppath.sh $$src tmp/$(DIST_NAME_BIN)-$(ARCH); \
		fi; \
	done;
	@cd $(DIST_ARCH)/tmp; \
	$(TAR) ../../$(DIST_TARBALL_BIN) $(DIST_NAME_BIN)-$(ARCH)

# -------------------------------------------------------------------------
# Target:	deps
# Desc: 	Makes dependencies
# -------------------------------------------------------------------------
.PHONY: deps
deps: pkgbanner autohdrs mkdepsdir $(EXTRA_TGT_DEPS) hdrdeps libdeps subdirs

.PHONY: hdrdeps
hdrdeps: 
	@echo "Making dependencies for $(CURDIR)"
	@echo $(call hdrdeps_sh,$(STLIBS) $(SHLIBS) $(DLLIBS) $(PGMS))

hdrdeps_sh = \
	$(shell $(rnmake)/utils/hdrdeps.sh \
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
.PHONY: clean do-clean
clean: pkgbanner do-clean $(EXTRA_TGT_CLEAN) subdirs 

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
	@echo "\nClobbering distribution $(CURDIR)"
	$(RM) $(DIST_ARCH)
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-$(ARCH).tar.gz
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-doc.tar.gz
	$(RM) $(DIST_ROOT)/$(PKG_FULL_NAME)-src.tar.gz
	$(RM) $(LOCDIR_LIST)
	$(RM) $(AUTOHDRS)
	$(RM) $(DEPSFILE)

# -------------------------------------------------------------------------
# Target:	subdirs
# Desc: 	Recursively make subdirectories.
# -------------------------------------------------------------------------
.PHONY: subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

$(SUBDIRS):
	$(call dirbanner,$(@))
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C $(@) $(CURGOAL)
	@echo 

# -------------------------------------------------------------------------
# Pretty Print Support "Targets"
# -------------------------------------------------------------------------

# Directory Banner Template
define dirbanner
	@if [ "$(1)" != "" ]; then \
	subdirnam="$(patsubst $(dir $(PKG_ROOT))%,%,$(CURDIR)/$(1))";\
	printf "\n";\
	printf "$(color_dir_banner)$(dashline)\n";\
	printf "Directory: $$subdirnam\n";\
	printf "Target:    $(CURGOAL)\n";\
	printf "$(dashline)$(color_end)\n";\
	fi
endef

# Pretty Print Major Banner
.PHONY: pkgbanner
pkgbanner:
	@if [ "$(MAKELEVEL)" = "0" -a "$(CURGOAL)" = "$(GOAL)" ]; then \
	printf "$(color_pkg_banner)$(boldline)\n";\
	printf "Package:       $(PKG_FULL_NAME)\n";\
	printf "Package Root:  $(PKG_ROOT)\n";\
	printf "Architecture:  $(ARCH)\n";\
	printf "Directory:     $(CURDIR)\n";\
	printf "Target:        $(CURGOAL)\n";\
	printf "Start:         `date`\n";\
	printf "$(boldline)$(color_end)\n";\
	fi

# Pretty Print Footer
.PHONY: footer
footer:
	@if [ "$(MAKELEVEL)" = "0" -a "$(CURGOAL)" = "$(GOAL)" ]; then \
	echo "";\
	echo "            ###";\
	echo "Finished: `date`";\
	echo "            ###";\
	fi

dashline := \
________________________________________________________________________________

boldline := $(dashline)\n$(dashline)

# -------------------------------------------------------------------------
# Include any dependency file only for the given CURGOAL targets
# \todo dependency file should only be included if not nodeps
#
ifneq "$(findstring $(CURGOAL),all test)" ""

# check if dependencies have been made unless nodeps is defined
chkdeps = $(if $(or $(nodeps),$(call isfile,$(1))),,\
		$(error No dependencies file - Try 'make deps' first.))

$(call chkdeps,$(DEPSFILE))

include $(DEPSFILE)
endif


# force some targets to always make
force: ;

# dpkg make rules
include $(rnmake)/Rules.dpkg.mk


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
