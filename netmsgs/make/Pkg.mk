################################################################################
#
# Pkg.mk

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-12-11 14:28:44 -0700 (Tue, 11 Dec 2012) $
$Rev: 2574 $

\brief
RoadNarrows Robotics Package Makefile.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2010-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

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

# The Package Definition
PKG 								= netmsgs
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 2
PKG_VERSION_RELEASE = 0
PKG_VERSION_DATE    = 2016
PKG_AUTHORS					= "Robin Knight"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= \
"This is free software; see the source for copying conditions. There is NO\\n \
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."

# Dotted full version number
PKG_VERSION_DOTTED	= $(PKG_VERSION_MAJOR).$(PKG_VERSION_MINOR).$(PKG_VERSION_RELEASE)

# Concatenated full version number
PKG_VERSION_CAT    	= $(PKG_VERSION_MAJOR)$(PKG_VERSION_MINOR)$(PKG_VERSION_RELEASE)

# Package full name
PKG_FULL_NAME				= $(PKG)-$(PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# Package Include Directories
PKG_INCDIRS 				= $(pkgroot)/include

# Package System Include Directories
PKG_SYS_INCDIRS			=

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			= 

#------------------------------------------------------------------------------
# Release Files 

# Release Files (docs)
PKG_REL_FILES				= VERSION.txt README.md


#------------------------------------------------------------------------------
# Flags
#
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

# Doxy Parameters

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
