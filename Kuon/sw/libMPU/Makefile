################################################################################
#
# ./sw/libMPU/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libMPU library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/libMPU/Makefile}

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
RNMAKE_PKG_ROOT = ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = MPU

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = MPU

# Source Files
MPU.SRC.C = MPU6050.c 

# Linked Libraries 
MPU.LIBS	= rnr_i2c rnr m


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libMPU\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
