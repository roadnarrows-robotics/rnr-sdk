################################################################################
#
# ./dxl_sdk-1.01/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Robotis dynamixel SDK library libdxl makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dxl_sdk-1.01/Makefile}

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
dxl.SRC.C 	=	dxl_hal.c dynamixel.c

# Linked Libraries

# put the source virtually here (cwd)
VPATH = src


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL_POST = cp-dxl-hdrs

H_FILES = include/*.h

DISTDIR_DXL_INC = $(DISTDIR_INCLUDE)/Dynamixel/dxl

.PHONY: cp-dxl-hdrs
cp-dxl-hdrs:
	$(call printGoalDesc,$(@),Copying dxl library headers to $(DISTDIR_DXL_INC))
	$(call copySrcDirRel,$(H_FILES),$(DISTDIR_DXL_INC))


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
