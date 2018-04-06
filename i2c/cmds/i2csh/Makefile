################################################################################
#
# ./cmds/i2csh/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libi2c library makefile.
\brief The i2csh makefile.

The i2csh command provides an interactive shell to an \h_i2c Bus.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{cmds/i2csh/Makefile}

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

RNMAKE_DIST_PGMS = i2csh

# Libraries to Link With
i2csh.LIBS    = rnr_i2c rnr
i2csh.LIBDEPS = rnr_i2c


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
i2csh.SRC.C = i2csh.c


#------------------------------------------------------------------------------
# Extras
#

# Sub[Package] Extra Include Directories
EXTRA_INCDIRS			= $(RNMAKE_PKG_ROOT)/cmds/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2csh\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
