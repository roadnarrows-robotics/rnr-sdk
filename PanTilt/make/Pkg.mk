################################################################################
#
# Pkg.mk

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-06-01 09:43:21 -0600 (Mon, 01 Jun 2015) $
$Rev: 4005 $

\brief RoadNarrows Robotics PanTilt Package Makefile.

RN Make System Specific Makefile

\author Robin Knight (robin.knight@roadnarrows.com)
	
\par Copyright:
  (C) 2015.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# 
# @EulaBegin@
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
PKG 								= PanTilt
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 2
PKG_VERSION_RELEASE = 0
PKG_VERSION_DATE    = 2015
PKG_AUTHORS					= "RoadNarrows"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= "(C) 2015 RoadNarrows LLC. All rights reserved."


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
PKG_SYS_INCDIRS			+= $(OPT_PREFIX)/include

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			+= $(OPT_PREFIX)/lib $(libdir)/botsense

# subdirectory under prefix/lib for server modules
PKG_LIB_PLUGIN_SUBDIR = 

#------------------------------------------------------------------------------
# Release Files 

# Release Files (docs)
PKG_REL_FILES				= VERSION.txt README.xml


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
