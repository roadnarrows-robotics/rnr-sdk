################################################################################
#
# ./bsPython/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The BotSense \h_dynamixel python module extensions makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{bsPython/src/Makefile}

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
# Extras

EXTRA_TGT_ALL 	= all-extmod
EXTRA_TGT_CLEAN	= clean-extmod

SWIG_FILES 				= 
SWIG_EXTMOD_DIR 	= ../modules/rnr
SWIG_EXTMOD_LIBS 	= -lrnr_dynamixel -l dxl2 -lrnr

all-extmod: swig-all

# Clean extension module intermediates
clean-extmod: swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefiles

# Include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# Include swig rules (must be after top-level rules)
include $(RNMAKE_ROOT)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
