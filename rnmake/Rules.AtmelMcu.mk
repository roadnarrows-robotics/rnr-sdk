################################################################################
#
# Rules.AtmelMcu.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Alternative rules file to support Atmel16 Microcontroller Units.

Include this file in each appropriate local make file (usually near the bottom).
Only one library or program target is supported per make file, unlike the
standard Rules.mk file.

\sa Rules.mk for more details on standard make targets.

\pkgsynopsis
RN Make System

\pkgfile{Rules.AtmelMcu.mk}

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

_RULES_ATMEL_MCU_MK = 1

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
# Product Makefile (Optional)

ifdef RNMAKE_PROD_MKFILE
  ifeq ($(_PROD_MK),)
    # optionally include (no error if not found)
    -include $(RNMAKE_PROD_MKFILE)
  endif
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
DIST_NAME_BIN	      = $(RNMAKE_PKG_FULL_NAME)
endif

# Distributions Directories
DISTDIR_BIN     = $(DIST_ARCH)/bin
DISTDIR_LIB     = $(DIST_ARCH)/lib
DISTDIR_INCLUDE = $(DIST_ARCH)/include
DISTDIR_ETC     = $(DIST_ARCH)/etc
DISTDIR_MAN     = $(DIST_ARCH)/man
DISTDIR_SHARE   = $(DIST_ARCH)/share/$(RNMAKE_PKG_FULL_NAME)
DISTDIR_DOC     = $(DIST_ARCH)/doc/$(RNMAKE_PKG_FULL_NAME)-doc
DISTDIR_SRC     = $(DIST_ARCH)/src/$(RNMAKE_PKG_FULL_NAME)
DISTDIR_TMP     = $(DIST_ARCH)/tmp/$(DIST_NAME_BIN)-$(ARCH)
DISTDIR_LIST    = $(DISTDIR_BIN) \
                  $(DISTDIR_INCLUDE) \
                  $(DISTDIR_LIB) \
                  $(DISTDIR_ETC) \
                  $(DISTDIR_SHARE) \
                  $(DISTDIR_DOC) \
                  $(DISTDIR_SRC) \
                  $(DISTDIR_TMP) \
                  $(DISTDIR_MAN)

# documentation subdirectories
DIST_SRCDOC					= srcdoc
DISTDIR_DOC_SRC			= $(DISTDIR_DOC)/$(DIST_SRCDOC)
DISTDIR_DOC_SRC_IMG	= $(DISTDIR_DOC_SRC)/images

# tar ball files - source, documentation, binary
DIST_TARBALL_SRC		= $(RNMAKE_PKG_FULL_NAME)-src.tar.gz
DIST_TARBALL_DOC		= $(RNMAKE_PKG_FULL_NAME)-doc.tar.gz
DIST_TARBALL_BIN		= $(DIST_NAME_BIN)-$(ARCH).tar.gz

# Object Directory
OBJDIR					= obj/obj.$(ARCH)

# Dependencies Directory
DEPSDIR			= .deps

# Dependencies File
DEPSFILE				= $(DEPSDIR)/deps.$(ARCH)


#------------------------------------------------------------------------------
# Build Flags
# Merge Architecture, Package and Parent Makefile variables into build flags.
#

# Include Flags
EXTRA_INCLUDES		= $(addprefix -I,$(EXTRA_INCDIRS))
EXTRA_SYS_INCLUDES= $(addprefix -I,$(EXTRA_SYS_INCDIRS))
PKG_INCLUDES			= $(addprefix -I,$(RNMAKE_PKG_INCDIRS))
PROD_INCLUDES			= $(addprefix -I,$(RNMAKE_PROD_INCDIRS))
ARCH_INCLUDES			= $(addprefix -I,$(RNMAKE_ARCH_INCDIRS))
PKG_SYS_INCLUDES	= $(addprefix -I,$(RNMAKE_PKG_SYS_INCDIRS))
DIST_INCLUDES			= -I$(DISTDIR_INCLUDE)
INCLUDES					= -I. \
										$(EXTRA_INCLUDES) \
										$(RNMAKE_PKG_INCLUDES) \
										$(PROD_INCLUDES) \
										$(ARCH_INCLUDES) \
										$(DIST_INCLUDES) \
										-I$(includedir) \
										$(EXTRA_SYS_INCLUDES) \
										$(RNMAKE_PKG_SYS_INCLUDES)

# CPP Flags
CPPFLAGS					= $(EXTRA_CPPFLAGS) \
										$(RNMAKE_PKG_CPPFLAGS) \
										$(RNMAKE_ARCH_CPPFLAGS) \
										-DARCH_$(ARCH) \
										-DARCH=\"$(ARCH)\"

# Assembler Flags
ASFLAGS						= $(EXTRA_ASFLAGS) $(RNMAKE_PKG_ASFLAGS) $(ARCH_ASFLAGS)

# C Flags
CFLAGS						= $(EXTRA_CFLAGS) $(RNMAKE_PKG_CFLAGS) $(RNMAKE_ARCH_CFLAGS)

# CXX Flags
CXXFLAGS					= $(EXTRA_CXXFLAGS) $(RNMAKE_PKG_CXXFLAGS) $(RNMAKE_ARCH_CXXFLAGS)

# Library Path Flags
EXTRA_LD_LIBPATHS	= $(addprefix -L,$(EXTRA_LD_LIBDIRS))
PKG_LD_LIBPATHS		= $(addprefix -L,$(RNMAKE_PKG_LD_LIBDIRS))
DIST_LD_LIBPATHS	= -L$(DISTDIR_LIB)
LD_LIBPATHS			  = $(EXTRA_LD_LIBPATHS) \
									 	 $(RNMAKE_PKG_LD_LIBPATHS) \
										 $(DIST_LD_LIBPATHS) \
										-L$(libdir) \
										 $(ARCH_LD_LIBPATHS)

# External Libraries
LD_LIBS						= $(EXTRA_LD_LIBS) $(RNMAKE_PKG_LD_LIBS) $(ARCH_LD_LIBS)

LDFLAGS     			= $(EXTRA_LDFLAGS) $(RNMAKE_PKG_LDFLAGS) $(ARCH_LDFLAGS)

# default link-loader is c compiler - override if using C++
ifeq "$(LANG)" "C++"
LD = $(LD_CXX)
endif

#------------------------------------------------------------------------------
# Build Target Names
# Construct build target files and set from parent makefile.
#

# build either a library or a program, but not both
ifdef TGT_LIB
TARGET = $(TGT_LIB)
BUILD		= lib
else
TARGET = $(TGT_PGM)
BUILD		= pgm
endif

# Generate list of obejcts from sources
OBJS = $(addprefix $(OBJDIR)/,$($(TARGET).SRC.C:.c=.o) \
															$($(TARGET).SRC.S:.S=.o))

# Output listing files
LSTS = $($(TARGET).SRC.C:.c=.lst) $($(TARGET).SRC.S:.S=.lst)

# Release files
REL_FILES				= $(RNMAKE_PKG_REL_FILES) $(EXTRA_REL_FILES)

# Release Files
FQ_REL_FILES 		= $(addprefix $(DISTDIR_DOC)/,$(REL_FILES))


#------------------------------------------------------------------------------
# Target Specific Variables
#

CURGOAL 	= $(GOAL)
all: 					CURGOAL := all
deps: 				CURGOAL := deps
install: 			CURGOAL := install
install-fw: 	CURGOAL := install
clean: 				CURGOAL := clean
distclean: 		CURGOAL := distclean
distclean-fw: CURGOAL := distclean
clobber: 			CURGOAL := clobber


#------------------------------------------------------------------------------
# Common Support Functions and Macros
#

# GNU Make has no boolean functions (why???), fake it.
neq = $(filter-out $(1),$(2))
eq  = $(if $(call neq,$(1),$(2)),,1)

# Returns "1" if given entity exists.
isFile 	= $(shell  if [ -f $(1) ]; then echo 1; fi)
isDir 	= $(shell  if [ -d $(1) ]; then echo 1; fi)


# Make obj/obj-<ARCH> in current directory
mkobjdir = @test -d $(OBJDIR) || $(MKDIR) $(OBJDIR)


########################### Explicit Rules #####################################

# -------------------------------------------------------------------------
# Target:	all (default)
# Desc: 	Front end to making the [sub]package(s) (libraries, programs, tools,
# 				documents, etc).
# Notes: 	There are two version:
# 					1) only done once on the first invocation and 
# 					2) for all other invocations.
.PHONY: all
all: mkdistdirs pkgbanner $(EXTRA_TGT_ALL) pkg footer

# -------------------------------------------------------------------------
# Target:	pkg
# Desc: 	Makes the distribution [sub]package(s) (libraries, programs, tools, 
# 				documents, etc).
.PHONY: pkg
pkg: $(BUILD) hdrs rel subdirs


# -------------------------------------------------------------------------
# Target:	lib (future)
# Desc: 	Makes all libraries in current directory.

# Make all libraries
.PHONY: lib
lib: stlib

# Make all static libraries
.PHONY: stlibs
stlib:


# -------------------------------------------------------------------------
# Target:	pgm
# Desc: 	Makes all firmware programs (images) in current directory.

# Make the program
.PHONY: pgm $(TGT_PGM)
pgm $(TGT_PGM): elf hex eep lss sym cppgm

# copy program to distribution directory
.PHONY: cppgm
cppgm:
	@$(CP) $(TARGET).hex $(DISTDIR_BIN)/.

# -------------------------------------------------------------------------
#  Target: elf hex eep lss sym
#  Desc:	 Make an intermediary file.

.PHONY: elf hex eep lss sym

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss 
sym: $(TARGET).sym

# -------------------------------------------------------------------------
#  Target: AVRDUDE
#  Desc:	 Rules to program and debug using avrdude.

# The Command
AVRDUDE = avrdude

# Programming Hardware:
#
# One of:
# 	alf avr910 avrisp bascom bsd dt006 pavr picoweb pony-stk200 sp12 stk200
# 	stk500 avrispmkII
#
# Type: 'avrdude -c ?' to get a full listing.
#
# Override as necessary.
#
AVRDUDE_PROGRAMMER ?= avrispmkII

# Communication Port
#
# com1 = serial port. Use lpt1 to connect to parallel port.
#
# Override as necessary.
#
#AVRDUDE_PORT = com1    # programmer connected to serial device

# USB Communication Port
#
# If only one AVRISP MKII is connected, the the default 'usb' port suffices.
# Otherwise, a portion of the the serial number must be specified. Below are the
# serial numbers for the two in-house ISPs. The serial number can be found on
# the bottom of the ISP.
#
AVRDUDE_PORT ?= usb
#AVRDUDE_PORT = usb:58:68    # usb port with avrispmkII serialno
#AVRDUDE_PORT = usb:82:82    # usb port with avrispmkII serialno

# Trick to map FORMAT to avrdude file format field
AVRDUDE_FLASH_FMT =	$(basename $(filter %.$(FORMAT),s.srec r.binary i.ihex))
AVRDUDE_FLASH_FMT ?= i

# Write Flash Memory Operation 
AVRDUDE_WRITE_FLASH 	= -U flash:w:$(TARGET).hex:$(AVRDUDE_FLASH_FMT)

# Write EEPROM Memory Operation 
AVRDUDE_WRITE_EEPROM	= -U eeprom:w:$(TARGET).eep

# Write Fuse High Byte Memory Operation 
AVRDUDE_WRITE_HFUSE		= -U hfuse:w:$(FUSE_HIGH):m

# Write Fuse High Byte Memory Operation 
AVRDUDE_WRITE_LFUSE		= -U lfuse:w:$(FUSE_LOW):m

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_FLAGS_ERASE_COUNTER = -y

# Uncomment the following if you do not wish a verification to be
# performed after programming the device.
#AVRDUDE_FLAGS_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
AVRDUDE_FLAGS_VERBOSE = -v

# Communication flags 
AVRDUDE_FLAGS_COM = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)

# AVRDUDE common flags
AVRDUDE_FLAGS = $(AVRDUDE_FLAGS_COM) \
								$(AVRDUDE_FLAGS_NO_VERIFY) \
								$(AVRDUDE_FLAGS_VERBOSE) \
								$(AVRDUDE_FLAGS_ERASE_COUNTER)

# Program device flash
.PHONY: flash program
flash program: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

# Program device EEPROM
.PHONY: eeprom
eeprom: $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_EEPROM)

# Program the device fuse high byte.
.PHONY: hfuse
-hfuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_HFUSE)

# Program the device fuse low byte.
.PHONY: lfuse
lfuse:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_LFUSE)

# Program the device fuses.
.PHONY: fuses
fuses:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_HFUSE) \
															$(AVRDUDE_WRITE_LFUSE)

# Program the whole enchilada. 
.PHONY: program-all
program-all: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) \
															$(AVRDUDE_WRITE_EEPROM) \
															$(AVRDUDE_WRITE_HFUSE) \
															$(AVRDUDE_WRITE_LFUSE)

# AVRDUD command-line interface interactive mode
avrdude:
	$(AVRDUDE) $(AVRDUDE_FLAGS) -t


# -------------------------------------------------------------------------
# Target:	hdrs
# Desc: 	Makes interface header files

# List of all header tags
HDR_TAG_LIST = $(addsuffix .HDRS.H,$(RNMAKE_DIST_HDRS))

# Complete list of headers
PREREQ_HDRS = $(foreach tag,$(HDR_TAG_LIST),$($(tag)))

# Make all distribution headers
.PHONY: hdrs
hdrs: $(PREREQ_HDRS)

# Copy newer headers to distribution include [sub]directory
$(PREREQ_HDRS):
	@for h in $(@); \
		do\
			src=$(RNMAKE_PKG_ROOT)/include/$$h; \
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
# Target: documents
# Desc:   Make documentation
.PHONY: documents
documents: docs-src-gen

# documentation generator from source files
ifndef HTML_HEADER
HTML_HEADER     = $(rnmake)/doxy/rnr_doxy_header.html
endif

ifndef HTML_FOOTER
HTML_FOOTER     = $(rnmake)/doxy/rnr_doxy_footer.html
endif

ifndef HTML_STYLESHEET
HTML_STYLESHEET = $(rnmake)/doxy/rnr_doxy.css
endif

ifndef DOXY_IMAGES
DOXY_IMAGES = $(rnmake)/doxy/rnr_images
endif

docs-src-gen:
	@if [ "$(RNMAKE_DOXY_CONF_FILE)" ]; \
	then \
		echo ""; \
		echo "Making source documentation"; \
		test -d $(DISTDIR_DOC) || $(MKDIR) $(DISTDIR_DOC); \
		test -d $(DISTDIR_DOC_SRC_IMG) || $(MKDIR) $(DISTDIR_DOC_SRC_IMG); \
		$(CP) -p $(DOXY_IMAGES)/* $(DISTDIR_DOC_SRC_IMG)/.; \
		(cat $(RNMAKE_DOXY_CONF_FILE); \
		 echo "PROJECT_NUMBER=$(RNMAKE_PKG_VERSION_DOTTED)"; \
		 echo "HTML_HEADER=$(HTML_HEADER)"; \
		 echo "HTML_FOOTER=$(HTML_FOOTER)"; \
		 echo "HTML_STYLESHEET=$(HTML_STYLESHEET)"; \
		 echo "EXAMPLE_PATH=$(RNMAKE_PKG_ROOT)/examples"; \
		 echo "OUTPUT_DIRECTORY=$(DISTDIR_DOC)"; \
		 echo "HTML_OUTPUT=$(DIST_SRCDOC)"; \
		) | doxygen - >$(RNMAKE_PKG_ROOT)/doxy.out.log 2>$(RNMAKE_PKG_ROOT)/doxy.err.log; \
		$(rnmake)/utils/doxyindex.sh \
							-t "$(RNMAKE_PKG) v$(RNMAKE_PKG_VERSION_DOTTED)" \
							-h $(HTML_HEADER) \
							>$(DISTDIR_DOC)/$(DIST_SRCDOC)/index.html; \
	fi


# -------------------------------------------------------------------------
# Target:	mkdistdirs
# Desc: 	Make Distribution Directories 
.PHONY: mkdistdirs $(DISTDIR_LIST)
mkdistdirs: $(DISTDIR_LIST)

$(DISTDIR_LIST):
	@$(MKDIR) $@;

# -------------------------------------------------------------------------
# Target:	install
# Desc: 	Install the distribution
.PHONY: install install-fw
install install-fw: pkgbanner all $(EXTRA_TGT_INSTALL) install-bin install-lib \
										install-includes install-docs install-share install-etc \
										footer

# install bin
install-bin:
	@echo ""
	@echo "Installing executables to $(bindir)"
	@$(rnmake)/utils/doinstall.sh 755 $(DISTDIR_BIN) $(bindir)
	$(call POSTINStemplate,$(STRIP_EXE),$(DISTDIR_BIN),$(bindir))

# install lib
install-lib:
	@echo ""
	@echo "Installing libraries to $(libdir)"
	@$(rnmake)/utils/doinstall.sh 755 $(DISTDIR_LIB) $(libdir)
	$(call POSTINStemplate,$(STRIP_LIB),$(DISTDIR_LIB),$(libdir))

# install includes
install-includes:
	@echo ""
	@echo "Installing includes to $(includedir)"
	@$(rnmake)/utils/doinstall.sh 664 $(DISTDIR_INCLUDE) $(includedir)

# install documentation
install-docs: documents
	@echo ""
	@echo "Installing documents to $(docdir)/$(RNMAKE_PKG_FULL_NAME)"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_DOC) $(docdir)/$(RNMAKE_PKG_FULL_NAME)

# install share
install-share:
	@echo ""
	@echo "Installing system share files to $(sharedir)"
	@$(rnmake)/utils/doinstall.sh -s 664 $(DISTDIR_SHARE) $(sharedir)/$(RNMAKE_PKG_FULL_NAME)

# install etc
install-etc:
	@echo ""
	@echo "Installing system configuration to $(sysconfdir)"
	@$(rnmake)/utils/doinstall.sh 664 $(DISTDIR_ETC) $(sysconfdir)

# Post-Install directory component template
# Usage: POSTINStemplate postprocess source_dir dest_dir
define POSTINStemplate
	@cd $(2); \
	srclist=$$($(FIND) . -type f); \
	for src in $$srclist; \
	do \
	  dst="$(3)/$${src##./}"; \
		echo "  $$dst"; \
		$(1) $$dst; \
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
	$(if $(call isDir,$(DISTDIR_DOC)),,\
			    					$(error No documentation - Try 'make documents' first))
	@cd $(DIST_ARCH)/doc; \
	$(TAR) ../../$(DIST_TARBALL_DOC) $(RNMAKE_PKG_FULL_NAME)-doc

.PHONY: tarball-src
tarball-src: 
	@test -d $(DISTDIR_SRC) || $(MKDIR) $(DISTDIR_SRC)
	@$(FIND) $(RNMAKE_PKG_ROOT) \( \
		-wholename '$(RNMAKE_PKG_ROOT)/dist' -or -wholename dist -or \
		-wholename '$(RNMAKE_PKG_ROOT)/loc' -or -wholename loc -or \
		-name '*.svn*' -or -wholename '.svn' -or \
		-wholename '*.deps*' -or -wholename '.deps' -or \
		-wholename '*obj*' -or -wholename 'obj' -or -wholename '*.o' -or \
		-wholename '*.out' -or -wholename '*.log' -or \
		-wholename '*.pyc' -or -wholename '*.pyo' \
	  \) -prune -or -print | \
	while read src; \
	do \
		$(rnmake)/utils/cppath.sh $$src $(DISTDIR_SRC); \
	done;
	@cd $(DIST_ARCH)/src; \
	$(TAR) ../../$(DIST_TARBALL_SRC) $(RNMAKE_PKG_FULL_NAME)

.PHONY: tarball-bin
tarball-bin:
	$(if $(call isDir,$(DIST_ARCH)),,$(error Nothing made - Try 'make' first))
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
deps: pkgbanner mkdepsdir $(EXTRA_TGT_DEPS) hdrdeps libdeps subdirs

.PHONY: hdrdeps
hdrdeps: 
	@echo "Making dependencies for $(CURDIR)"
	@echo $(call hdrdeps_sh,$(TGT_LIB) $(TGT_PGM))

hdrdeps_sh = \
	$(shell $(rnmake)/utils/hdrdeps.sh \
		-c "$(RNMAKE_MAKEDEPS)" \
		-f $(DEPSFILE) \
		-o $(OBJDIR) \
		-d "$(CPPFLAGS)" \
		$(INCLUDES) \
		$(foreach f,\
			$(sort $(addsuffix .SRC.C,$(1)) $(addsuffix .SRC.CXX,$(1))),\
			$($(f))))

libdeps:

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
	$(RM) *.o *.ii *.c~ *.cxx~ .h~ $(RNMAKE_DIST_PGMS) a.out doxy.*.log
	$(RM) *.hex *.eep *.cof *.elf *map *.a90 *.sym *.lnk *.lss
	$(RM) $(LSTS)
	$(RM) $(OBJDIR)

# -------------------------------------------------------------------------
# Target:	distclean (clobber)
# Desc: 	Cleans plus deletes distribution
# -------------------------------------------------------------------------
.PHONY: distclean clobber distclean-fw
distclean clobber distclean-fw: pkgbanner clean $(EXTRA_TGT_DISTCLEAN)
	@echo "\nClobbering distribution $(CURDIR)"
	$(RM) $(DIST_ARCH)
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-$(ARCH).tar.gz
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-doc.tar.gz
	$(RM) $(DIST_ROOT)/$(RNMAKE_PKG_FULL_NAME)-src.tar.gz
	$(RM) $(DEPSFILE)

# -------------------------------------------------------------------------
# Target:	subdirs
# Desc: 	Recursively make subdirectories.
# -------------------------------------------------------------------------
.PHONY: subdirs $(RNMAKE_SUBDIRS)
subdirs: $(RNMAKE_SUBDIRS)

$(RNMAKE_SUBDIRS):
	$(call dirbanner,$(@))
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C $(@) $(CURGOAL)

# -------------------------------------------------------------------------
# Pretty Print Support "Targets"
# -------------------------------------------------------------------------

# Directory Banner Template
define dirbanner
	@if [ "$(1)" != "" ]; then \
	subdirnam="$(patsubst $(dir $(RNMAKE_PKG_ROOT))%,%,$(CURDIR)/$(1))";\
	echo ""; \
	echo "$(dashline)";\
	echo "Directory: $$subdirnam";\
	echo "Target:    $(CURGOAL)";\
	echo "$(dashline)";\
	fi
endef

# Pretty Print Major Banner
.PHONY: pkgbanner
pkgbanner:
	@if [ "$(MAKELEVEL)" = "0" -a "$(CURGOAL)" = "$(GOAL)" ]; then \
	echo "$(boldline)";\
	echo "Package:       $(RNMAKE_PKG_FULL_NAME)";\
	echo "Package Root:  $(RNMAKE_PKG_ROOT)";\
	echo "Architecture:  $(ARCH)";\
	echo "Directory:     $(CURDIR)";\
	echo "Target:        $(CURGOAL)";\
	echo "Start:         `date`";\
	echo "$(boldline)";\
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
 "-----------------------------------------------------------------------------"
boldline := \
 "============================================================================="

# Display size of file.
HEXSIZE_CMD 	= $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE_CMD		= $(SIZE) -A $(TARGET).elf

sizes:
	$(call reportsizes)

define reportsizes
	@echo 
	@if [ -f $(TARGET).elf ]; \
	then \
		echo "$(TARGET).elf Sizes:"; \
		$(ELFSIZE_CMD); \
	fi
	@if [ -f $(TARGET).hex ]; \
	then \
		echo "$(TARGET).hex Sizes:"; \
		$(HEXSIZE_CMD); \
	fi
endef

# -------------------------------------------------------------------------
# Include any dependency file
# \todo dependency file should only be included if not nodeps
#
ifeq "$(CURGOAL)" "all"
$(call chkdeps)
include $(DEPSFILE)
endif

# check if dependencies have been made unless nodeps is defined
chkdeps:
	$(if $(or $(nodeps),$(call isFile,$(DEPSFILE))),,\
			    					$(error No dependencies file - Try 'make deps' first))


# force some targets to always make
force: ;


########################### Pattern Rules #####################################

# Define Information Messages
MSG_COFF 							= Converting to AVR COFF:
MSG_EXTENDED_COFF 		= Converting to AVR Extended COFF:
MSG_FLASH 						= Creating load file for Flash:
MSG_EEPROM 						= Creating load file for EEPROM:
MSG_EXTENDED_LISTING	= Creating Extended Listing:
MSG_SYMBOL_TABLE 			= Creating Symbol Table:
MSG_LINKING 					= Linking:


# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT=$(OBJCOPY) --debugging \
--change-section-address .data-0x800000 \
--change-section-address .bss-0x800000 \
--change-section-address .noinit-0x800000 \
--change-section-address .eeprom-0x810000 

coff: $(TARGET).elf
	@echo
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof

extcoff: $(TARGET).elf
	@echo
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof

# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O $(FORMAT) -R .eeprom $< $@
	@echo "$(TARGET).hex Sizes:"
	$(HEXSIZE_CMD)

%.eep: %.elf
	@echo
	@echo $(MSG_EEPROM) $@
	-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 -O $(FORMAT) $< $@

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@

#LDX = -L/opt/pkg/avr32/avr32-gnu-toolchain-linux_x86_64/lib/gcc/avr32/4.4.3 \
#			-L/opt/avr32/lib/ucr3
#LDY = /opt/avr32/lib/ucr3/crt0.o
LDY = c
LDX = -L/usr/lib/avr/lib

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJS)
%.elf: $(OBJS)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(CFLAGS) $(OBJS) --output $@ $(LDFLAGS)
	@echo "$(TARGET).elf Sizes:"
	$(ELFSIZE_CMD)

# Assembler Rule: <name>.S -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.S
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	@echo
	$(AS) $(ASFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C Rule: <name>.c -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.c
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C++ Rule: <name>.cxx -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.cxx
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C++ Rule: <name>.cpp -> $(OBJDIR)/<name>.o
$(OBJDIR)/%.o : %.cpp
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# Compile a single S file. (Nice for debugging)
%.o : %.S force
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(AS) $(ASFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single c file. (Nice for debugging)
%.o : %.c force
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CC) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single cxx file. (Nice for debugging)
%.o : %.cxx force
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# Compile a single cpp file. (Nice for debugging)
%.o : %.cpp force
	$(mkobjdir)
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(OBJDIR)/$(@) -c $(<)

# C PreProcess Rule: <name>.c -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
# Historically the output suffix is .i but this can interfere with swig 
# interface files that also have the .i suffix. So .ii will be used until 
# I find another "standard".
%.ii : %.c force
	@echo ""
	@echo "     $(<)"
	$(CC) $(CFLAGS_CPP_ONLY) $(CFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.cpp -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
%.ii : %.cxx force
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS_CPP_ONLY) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

# C PreProcess Rule: <name>.cpp -> <name>.ii
# Generate c preprocessor output (useful to debug compiling errors)
%.ii : %.cpp force
	@echo ""
	@echo "     $(<)"
	$(CXX) $(CXXFLAGS_CPP_ONLY) $(CXXFLAGS) $(CPPFLAGS) $(INCLUDES) -o $(@) -c $(<)

null:
	@echo "null me"


# -------------------------------------------------------------------------
# default error rule (doesn't work yet)

#%::
#	@echo "$(@): Unknown target. See 'make help' for help."


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
