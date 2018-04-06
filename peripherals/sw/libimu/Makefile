################################################################################
#
# ./sw/libimu/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_imu library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Peripherals C++ Package

\pkgfile{sw/libimu/Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com}
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_imu

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_imu

# Target library subdirectory
rnr_imu.SUBDIR = rnr

# Source Files
rnr_imu.SRC.CXX = yost.cxx

# Linked Libraries 
rnr_imu.LIBS = rnr stdc++ usb-1.0 


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_imu\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
