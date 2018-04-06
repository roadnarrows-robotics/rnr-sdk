################################################################################
#
# ./examples/spintest/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The spintest example makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{examples/spintest/Makefile}

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
# Programs

RNMAKE_DIST_PGMS = spintest 

READLINE_LIBS     = readline ncurses
READLINE_CPPFLAGS = -DHAVE_READLINE

# Libraries to Link With
spintest.LIBS    = rnr_dynamixel rnr_dxl rnr $(READLINE_LIBS) m
spintest.LIBDEPS = rnr_dynamixel rnr_dxl

# C Source Files
spintest.SRC.C = spintest.c


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DHAVE_READLINE -DLOG -DLOGMOD="\"spintest\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
