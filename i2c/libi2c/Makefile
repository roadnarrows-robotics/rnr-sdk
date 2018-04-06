################################################################################
#
# ./libi2c/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libi2c library makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{libi2c/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2007-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS 			= rnr_i2c

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS 			= rnr_i2c

# Target library subdirectory
rnr_i2c.SUBDIR = rnr

# Source Files
rnr_i2c.SRC.C	= i2ccom.c smbus.c


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_i2c\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
