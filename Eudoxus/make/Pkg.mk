################################################################################
#
# Pkg.mk

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
$Rev: 4263 $

\brief RoadNarrows Robotics Eudoxus Package Makefile.

RN Make System Specific Makefile

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2012-2016  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

# Prevent mutliple inclusion
PKG_MK						= 1

ifndef pkgroot
$(error Error: pkgroot not defined in including makefile)
endif

# The Package Definition
PKG 								= Eudoxus
PKG_VERSION_MAJOR   = 3
PKG_VERSION_MINOR   = 0
PKG_VERSION_RELEASE = 0
PKG_VERSION_DATE    = 2016
PKG_AUTHORS					= "RoadNarrows"
PKG_OWNERS					= "RoadNarrows LLC"
PKG_DISCLAIMER			= \
"This is free software; see the source for copying conditions. There is NO\\n \
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."

# Dotted full version number
PKG_VERSION_DOTTED	= \
	$(PKG_VERSION_MAJOR).$(PKG_VERSION_MINOR).$(PKG_VERSION_RELEASE)

# Concatenated full version number
PKG_VERSION_CAT    	= \
	$(PKG_VERSION_MAJOR)$(PKG_VERSION_MINOR)$(PKG_VERSION_RELEASE)

# Package full name
PKG_FULL_NAME				= $(PKG)-$(PKG_VERSION_DOTTED)

#------------------------------------------------------------------------------
# Optional Variables and Tweaks

ifndef arch
ifdef RNMAKE_ARCH_DFT
    arch=$(RNMAKE_ARCH_DFT)
else
    arch = x86_64
endif
endif

prefix_root = /prj/xinstall

# Package Include Directories
PKG_INCDIRS = $(pkgroot)/include \
							$(pkgroot)/sw/External/LibJPEG \
							$(pkgroot)/sw/External/TinyXml

#PKG_INCDIRS = $(pkgroot)/include \
#							$(pkgroot)/include/ni \
#							$(pkgroot)/include/primesense

# overo linaro
ifeq "$(arch)" "linaro"

PKG_SYS_PREFIX = /usr
PKG_OPT_PREFIX = /opt
EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/arm-linux-gnueabihf/glib-2.0/include
GST_VER = 0.10

# cross-compile environment for old angstrom overo
else
ifeq "$(arch)" "overo"

PKG_SYS_PREFIX = $(OE_TMP)/sysroots/armv7a-angstrom-linux-gnueabi/usr
PKG_OPT_PREFIX = /opt/xinstall/overo
EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include
GST_VER = 0.10

# odroid
else
ifeq "$(arch)" "odroid"

PKG_SYS_PREFIX = /usr
PKG_OPT_PREFIX = /opt
EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/arm-linux-gnueabihf/glib-2.0/include
PCL_VER = 1.7
GST_VER = 1.0

# off target
else

PKG_SYS_PREFIX = /usr
PKG_OPT_PREFIX = /opt
EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include
PCL_VER = 1.7
GST_VER = 1.0

endif
endif
endif

ifdef PCL_VER
PKG_SYS_INCDIRS += $(PKG_SYS_PREFIX)/include/pcl-$(PCL_VER)
PKG_SYS_INCDIRS += $(PKG_SYS_PREFIX)/include/ni
endif

PKG_SYS_INCDIRS += $(PKG_SYS_PREFIX)/include/gstreamer-$(GST_VER)

#EXTRA_LD_LIBDIRS = /usr/local/lib/rnr

#------------------------------------------------------------------------------
# Release Files 

# Release Files (docs)
PKG_REL_FILES				= VERSION.txt README.md


#------------------------------------------------------------------------------
# Flags
#
# CPP flags
PKG_CPPFLAGS				= -DXN_EXPORTS

# C flags
PKG_CFLAGS					=

# CXX flags
PKG_CXXFLAGS				= -Wno-deprecated-declarations \
											-Wno-deprecated

# Link flags
PKG_LDFLAGS					=


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
