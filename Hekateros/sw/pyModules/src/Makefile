################################################################################
#
# ./sw/pyModules/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_hek python modules makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/pyModules/src/Makefile}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL 	= all-extmod
EXTRA_TGT_CLEAN	= clean-extmod

SWIG_FILES 				= 
SWIG_EXTMOD_DIR 	= ../modules/Hekateros
SWIG_EXTMOD_LIBS 	= -lhekateros -lrnr

all-extmod: swig-all

# Clean extension module intermediates
clean-extmod: swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include swig rules
include $(rnmake)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
