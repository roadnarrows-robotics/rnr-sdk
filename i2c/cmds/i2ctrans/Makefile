################################################################################
#
# ./cmds/i2ctrans/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The i2ctrans command makefile.

The i2trans commands performs a write-read transaction on a device connected
to an \h_i2c Bus.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{cmds/i2ctrans/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = i2ctrans

# Libraries to Link With
i2ctrans.LIBS    = rnr_i2c rnr
i2ctrans.LIBDEPS = rnr_i2c 


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2ctrans.SRC.C = i2ctrans.c


#------------------------------------------------------------------------------
# Extras
#

# Sub[Package] Extra Include Directories
EXTRA_INCDIRS			= $(RNMAKE_PKG_ROOT)/cmds/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2ctrans\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
