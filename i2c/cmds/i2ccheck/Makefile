################################################################################
#
# ./cmds/i2ccheck/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The i2ccheck command makefile.

The i2ccheck command checks if an \h_i2c device with the specified address
exists on the \h_i2c Bus.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{cmds/i2ccheck/Makefile}

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

RNMAKE_DIST_PGMS = i2ccheck

# Libraries to Link With
i2ccheck.LIBS    = rnr_i2c rnr
i2ccheck.LIBDEPS = rnr_i2c


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2ccheck.SRC.C = i2ccheck.c


#------------------------------------------------------------------------------
# Extras
#

# Sub[Package] Extra Include Directories
EXTRA_INCDIRS	= $(RNMAKE_PKG_ROOT)/cmds/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2ccheck\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
