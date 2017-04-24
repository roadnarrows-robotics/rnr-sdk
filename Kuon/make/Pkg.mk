################################################################################
#
# Pkg.mk

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-06-01 09:50:14 -0600 (Mon, 01 Jun 2015) $
$Rev: 4007 $

\brief RoadNarrows Robotics Kuon Package Makefile.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)
\author Daniel Packard (daniel@roadnarrows.com)
	

##! \copyright
##!   \h_copy 2010-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
# @EulaBegin@
# @EulaEnd@
#
################################################################################

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef pkgroot
$(error Error: pkgroot not defined in including makefile)
endif

# The Package Definition
PKG 								= Kuon
PKG_VERSION_MAJOR   = 1
PKG_VERSION_MINOR   = 1
PKG_VERSION_RELEASE = 2
PKG_VERSION_DATE    = 2017
PKG_AUTHORS					= "Robin Knight, Rob Shiely, and Daniel Packard"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= \
"This is copyrighted software; see the source for copying conditions. There is NO\
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

prefix_root = /prj/xinstall

# Package Include Directories
PKG_SYS_INCDIRS 		= $(pkgroot)/include

ifeq "$(arch)" "overo"
# Package System Include Directories
PKG_SYS_INCDIRS			+= /opt/xinstall/overo/include

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			+= /opt/xinstall/overo/lib

else

# Package System Include Directories
PKG_SYS_INCDIRS			+= /opt/include

# Link Library Extra Library Directories (exluding local library)
PKG_LD_LIBDIRS 			+= /opt/lib

endif

# subdirectory under prefix/lib for server modules
PKG_LIB_PLUGIN_SUBDIR = botsense



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
