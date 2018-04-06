################################################################################
#
# Env.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RN Make System rnmake command-line and environment variables helper
makefile.

\pkgsynopsis
RN Make System

\pkgfile{Env.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_ENV_MK = 1

# ------------------------------------------------------------------------------
# RNMAKE_ROOT
# 	What:									Specifies root path to rnmake package.
# 	Environment variable: RNMAKE_ROOT
# 	Make override:				make rnmake=<path> ...
# 	Default:							no
# 	Required:							yes
# ------------------------------------------------------------------------------

# 'make rnmake=<path> ...' or RNMAKE_ROOT
rnmake ?= $(RNMAKE_ROOT)

# must be defined and non-empty
ifeq ($(rnmake),)
  $(error 'RNMAKE_ROOT' environment variable not specified)
endif

# make absolute filename (empty string returned if directory does not exist)
RNMAKE_ROOT := $(realpath $(rnmake))

# required
ifeq ($(RNMAKE_ROOT),)
  $(error 'RNMAKE_ROOT=$(rnmake)': No such directory)
endif

rnmake := $(RNMAKE_ROOT)


# ------------------------------------------------------------------------------
# RNMAKE_ARCH_TAG
# 	What:									Determines which architecture makefile to include.
# 													Arch.$(RNMAKE_ARCH_TAG).mk
# 	Environment variable: RNMAKE_ARCH_DFT
# 	Make override:				make arch=<arch> ...
# 	Default:							x86_64
# 	Required:							no
# ------------------------------------------------------------------------------

# 'make arch=<arch> ...' or RNMAKE_ARCH_DFT
arch ?= $(RNMAKE_ARCH_DFT)

# default
ifeq ($(arch),)
  arch = x86_64
  $(info 'arch=$(arch)' default default used.)
endif

RNMAKE_ARCH_TAG := $(arch)


# ------------------------------------------------------------------------------
# RNMAKE_INSTALL_XPREFIX
# 	What:									Cross-install prefix.
# 												Actual packages are installed to
# 	                      	$(RNMAKE_INSTALL_XPREFIX)/$(RNMAKE_ARCH)/
# 	Environment variable: RNMAKE_INSTALL_XPREFIX
# 	Make override:				make xprefix=<path> ...
# 	Default:							$(HOME)/xinstall
# 	Required:							no
# ------------------------------------------------------------------------------

# 'make xprefix=<path> ...' or RNMAKE_INSTALL_XPREFIX
xprefix ?= $(RNMAKE_INSTALL_XPREFIX)

# default
ifeq ($(xprefix),)
  xprefix = $(HOME)/xinstall
  $(info 'RNMAKE_INSTALL_XPREFIX=$(xprefix)' default used.)
endif

# make absolute path (does not have to exist)
xprefix := $(abspath $(xprefix))

RNMAKE_INSTALL_XPREFIX := $(xprefix)


# ------------------------------------------------------------------------------
# RNMAKE_INSTALL_PREFIX
# 	What:									Install prefix. Overrides RNMAKE_INSTALL_XPREFIX.
# 												Packages are installed to:
# 													$(RNMAKE_INSTALL_PREFIX)/
# 	Environment variable: RNMAKE_INSTALL_PREFIX
# 	Make override:				make prefix=_path_ ...
# 	Default:							
# 	Required:							no
# ------------------------------------------------------------------------------

# 'make prefix=<path> ...' or RNMAKE_INSTALL_PREFIX
prefix ?= $(RNMAKE_INSTALL_PREFIX)

# make absolute path (does not have to exist)
prefix := $(abspath $(prefix))

RNMAKE_INSTALL_PREFIX := $(prefix)


# ------------------------------------------------------------------------------
# Export to sub-makes
#
export RNMAKE_ROOT
export RNMAKE_ARCH_TAG
export RNMAKE_INSTALL_XPREFIX
export RNMAKE_INSTALL_PREFIX

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
