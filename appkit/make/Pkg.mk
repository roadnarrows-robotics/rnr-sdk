################################################################################
#
# Pkg.mk
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-03-18 09:53:37 -0600 (Fri, 18 Mar 2016) $
$Rev: 4352 $

\brief Package appkit Configuration Makefile.

RN Make System Specific Makefile

\author: Robin Knight 	(robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2013-2016. RoadNarrows LLC.
  (http://roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# 
################################################################################

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef pkgroot
$(error Error: pkgroot not defined in including makefile)
endif

# The Package Definition
PKG 								= appkit
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 2
PKG_VERSION_RELEASE = 1
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

ifdef RNMAKE_ARCH_DFT
	  arch=$(RNMAKE_ARCH_DFT)
endif

prefix_root = /prj/xinstall

# Package Include Directories
PKG_INCDIRS 		= $(pkgroot)/include

# Package System Include Directories
PKG_SYS_INCDIRS	+= $(OPT_PREFIX)/include

# Link Library Extra Library Directories (exluding local library)
ifeq "$(arch)" "linaro"
PKG_LD_LIBDIRS 			+= $(ROS_LD_LIBDIR)
endif

PKG_LD_LIBDIRS 			+= $(OPT_PREFIX)/lib

#PKG_LD_LIBDIRS 			+= $(DISTDIR_LIB)/lib/rnr $(OPT_PREFIX)/lib

# subdirectory under prefix/lib for server modules
LIB_SUBDIR = rnr

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
