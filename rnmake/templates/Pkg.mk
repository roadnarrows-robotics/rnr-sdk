################################################################################
#
# Package: 	RN Make System 
#
# File:			Pkg.mk
#
# Version: 2.0
#
# Description:
#  	Package Template Makefile.
#
# Author(s): Robin Knight (robin.knight@roadnarrows.com)
#
# Copyright (C) 2005-2007.  RoadNarrows LLC.
# All Rights Reserved
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

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef pkgroot
$(error Error: pkgroot not defined in including makefile)
endif

#
# The Package Definition
#
PKG 								= packagename
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 0
PKG_VERSION_RELEASE = 0
PKG_VERSION_DATE    = 2007
PKG_AUTHORS					= "Robin Knight & Kim Wheeler-Smith"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= \
"This is free software; see the source for copying conditions.  There is NO\n" \
"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."

# Dotted full version number
PKG_VERSION_DOTTED	= $(PKG_VERSION_MAJOR).$(PKG_VERSION_MINOR).$(PKG_VERSION_RELEASE)

# Concatenated full version number
PKG_VERSION_CAT    	= $(PKG_VERSION_MAJOR)$(PKG_VERSION_MINOR)$(PKG_VERSION_RELEASE)

# Package full name
PKG_FULL_NAME				= $(PKG)-$(PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# Package Local Working Directory
#
# Local components are not distributed.
#
# Uncomment and define to override where package local bin and libraries
# are placed. This can be usefull if serveral packages use a global
# local area rather than one local area per package.
#
# Default: $(pkgroot)/loc/
#
#LOCDIR_ROOT				=

# Package Include Directories
PKG_INCDIRS 				= $(pkgroot)/include

# Package System Include Directories
PKG_SYS_INCDIRS			=

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			= 

# Release Files (docs)
PKG_REL_FILES				= VERSION README

# CPP flags
PKG_CPPFLAGS				=

# C flags
PKG_CFLAGS					=

# CXX flags
PKG_CXXFLAGS				=

# Link flags
PKG_LDFLAGS					=
