################################################################################
#
# ./libgpio/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_gpio libgpio library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_gpio Package

\pkgfile{libgpio/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2015-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_DIST_STLIBS = rnr_gpio

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_gpio

# Target library subdirectory
rnr_gpio.SUBDIR = rnr

# Source Files
rnr_gpio.SRC.C = gpio.c mmapGpio.c

# Libraries to link with
rnr_gpio.LIBS	= rnr


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
# -DMMAP_GPIO to enable memory mapped gpio TODO
EXTRA_CPPFLAGS = -DLOG -DLOGMOD=\"librnr_gpio\"


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
