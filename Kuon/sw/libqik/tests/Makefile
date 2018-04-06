################################################################################
#
# ./sw/libqik/tests/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Library libqik unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/libqik/tests/Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com} 

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
# Programs

# Distribution Programs
RNMAKE_TEST_PGMS = QikMove

# Linked Libraries 
TEST_PGMS_LIBS = rnr qik serial
										
QikMove.LIBS = $(TEST_PGMS_LIBS)

# C++ Source Files
QikMove.SRC.C	= QikMove.c


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"utlibqik\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
