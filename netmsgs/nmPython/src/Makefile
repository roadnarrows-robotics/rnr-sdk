################################################################################
#
# ./nmPython/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_netmsgs python module extensions makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Network Messaging Package

\pkgfile{nmPython/src/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
#

EXTRA_TGT_ALL 	= all-extmod
#EXTRA_TGT_DEPS 	= all-extmod
EXTRA_TGT_CLEAN	= clean-extmod


#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

SWIG_FILES 				= NetMsgsCore.i
SWIG_EXTMOD_DIR 	= ../modules/NetMsgs
SWIG_EXTMOD_LIBS 	= -lrnr_netmsgs -lrnr

all-extmod: swig-all

# Clean extension module intermediates
clean-extmod: swig-clean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# Include top-levle rules
include $(RNMAKE_ROOT)/Rules.mk

# Include swig rules (must be after top-level)
include $(rnmake)/Rules.swig.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
