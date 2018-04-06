################################################################################
#
# ./sw/libhid/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_hid library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/libhid/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com} 

\pkgcopyright{2013-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS       = rnr_hid 

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS       = rnr_hid 

# Target library subdirectory
rnr_hid.SUBDIR = rnr

# Source Files
rnr_hid.SRC.CXX = HIDXbox360.cxx 

# Linked Libraries 
rnr_hid.LIBS	= rnr stdc++ usb-1.0


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_hid\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
