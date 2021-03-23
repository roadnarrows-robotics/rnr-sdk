################################################################################
#
# ./bsPython/src/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief \h_botsense python module extensions makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{bsPython/src/Makefile}

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
EXTRA_TGT_CLEAN	= clean-extmod

SWIG_FILES 				= BotSenseCore.i
SWIG_EXTMOD_DIR 	= ../modules/BotSense
SWIG_EXTMOD_LIBS 	= -lbotsense -lrnr_netmsgs -lrnr

EXTRA_CFLAGS += $(SWIG_CFLAGS)
EXTRA_LDFLAGS += $(SWIG_LDFLAGS)

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
