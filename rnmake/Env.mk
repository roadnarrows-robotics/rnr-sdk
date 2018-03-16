# //////////////////////////////////////////////////////////////////////////////
# 
# Package:	RN Make System
#
# File:			Env.mk
#
ifdef RNMAKE_DOXY
/*!
 * \file
 *
 * \brief RN Make System rnmake command-line and environment variables helper
 * makefile.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2018. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 * 
 * \cond RNMAKE_DOXY
 */
endif
# 
# //////////////////////////////////////////////////////////////////////////////

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
# 	Default:							/usr/local
# 	Required:							no
# ------------------------------------------------------------------------------

# 'make prefix=<path> ...' or RNMAKE_INSTALL_PREFIX
prefix ?= $(RNMAKE_INSTALL_PREFIX)

# default
ifeq ($(prefix),)
  prefix = /usr/local
endif

# make absolute path (does not have to exist)
prefix := $(abspath $(prefix))

RNMAKE_INSTALL_PREFIX := $(prefix)


# ------------------------------------------------------------------------------
# Export to sub-makes
#
#export rnmake arch xprefix prefix
export RNMAKE_ROOT
export RNMAKE_ARCH_TAG
export RNMAKE_INSTALL_XPREFIX
export RNMAKE_INSTALL_PREFIX
