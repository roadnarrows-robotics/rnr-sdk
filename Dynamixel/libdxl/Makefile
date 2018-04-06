################################################################################
#
# ./libdxl/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_dxl library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{libdxl/Makefile}

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
RNMAKE_DIST_STLIBS = rnr_dxl

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_dxl

# Target library subdirectory
rnr_dxl.SUBDIR = rnr

# Source Files
rnr_dxl.SRC.CXX	=	\
	dxlhal.cxx \
	dynamixel.cxx

# Libraries to link with
rnr_dxl.LIBS = rnr_serial rnr


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_dxl\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
