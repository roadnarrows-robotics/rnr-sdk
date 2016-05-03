################################################################################
#
# Pkg.mk
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-04-07 14:52:56 -0600 (Thu, 07 Apr 2016) $
$Rev: 4378 $

\brief Package Laelaps Configuration Makefile.

RN Make System Specific Makefile

\author: Robin Knight 	(robin.knight@roadnarrows.com)
\author: Daniel Packard	(daniel@roadnarrows.com)

\par Copyright:
  (C) 2013-2016. RoadNarrows LLC.
  (http://roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# 
# @EulaBegin@
# 
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# 
# @EulaEnd@
# 
################################################################################

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef pkgroot
$(error Error: pkgroot not defined in including makefile)
endif

# The Package Definition
PKG 								= Laelaps
PKG_VERSION_MAJOR   = 2
PKG_VERSION_MINOR   = 1
PKG_VERSION_RELEASE = 5
PKG_VERSION_DATE    = 2016
PKG_AUTHORS					= "Robin Knight"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= "(C) 2016 RoadNarrows LLC. All rights reserved."

# Dotted full version number
PKG_VERSION_DOTTED	= $(PKG_VERSION_MAJOR).$(PKG_VERSION_MINOR).$(PKG_VERSION_RELEASE)

# Concatenated full version number
PKG_VERSION_CAT    	= $(PKG_VERSION_MAJOR)$(PKG_VERSION_MINOR)$(PKG_VERSION_RELEASE)

# Package full name
PKG_FULL_NAME				= $(PKG)-$(PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

prefix_root = /prj/xinstall

# Package Include Directories
PKG_INCDIRS 		= $(pkgroot)/include

# Package System Include Directories
PKG_SYS_INCDIRS			+= /opt/include

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			+= $(OPT_PREFIX)/lib $(libdir)/botsense

# subdirectory under prefix/lib for server modules
PKG_LIB_PLUGIN_SUBDIR = 

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


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
