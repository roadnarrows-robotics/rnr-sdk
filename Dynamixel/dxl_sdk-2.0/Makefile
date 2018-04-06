################################################################################
#
# ./dxl_sdk-2.0/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Robotis Dynamixel SDK 2.0 library and examples makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{dxl_sdk-2.0/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = src example


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
