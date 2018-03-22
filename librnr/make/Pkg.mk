# //////////////////////////////////////////////////////////////////////////////
#
ifdef RNMAKE_DOXY
/*!
 * \file
 * 
 * \brief The RoadNarrows Robotics \h_librnr package makefile.
 *
 * An rnmake system package specific makefile.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgfile{make/Pkg.mk}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2018. RoadNarrows LLC.\n
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

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef RNMAKE_PKG_ROOT
  $(error 'RNMAKE_PKG_ROOT' Not defined in including makefile)
endif

# The Package Definition
PKG 								= librnr
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 14
PKG_VERSION_RELEASE = 3
PKG_VERSION_DATE    = 2017
PKG_AUTHORS					= "Robin Knight, Kim Wheeler, Daniel Packard"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= \
"See the README and EULA files for any copyright and licensing information."

# Dotted full version number
PKG_VERSION_DOTTED	= $(PKG_VERSION_MAJOR).$(PKG_VERSION_MINOR).$(PKG_VERSION_RELEASE)

# Concatenated full version number
PKG_VERSION_CAT    	= $(PKG_VERSION_MAJOR)$(PKG_VERSION_MINOR)$(PKG_VERSION_RELEASE)

# Package full name
PKG_FULL_NAME				= $(PKG)-$(PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# Package Include Directories
PKG_INCDIRS 				= $(RNMAKE_PKG_ROOT)/include

# Package System Include Directories
PKG_SYS_INCDIRS			=

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			= 

# Release Files (docs)
PKG_REL_FILES				= VERSION.txt README.md

# CPP flags
PKG_CPPFLAGS				=

# C flags
PKG_CFLAGS					=

# CXX flags
PKG_CXXFLAGS				=

# Link flags
PKG_LDFLAGS					=

# package wide lib subdir
LIB_SUBDIR					= rnr

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
