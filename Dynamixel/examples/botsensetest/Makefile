################################################################################
#
# ./examples/botsensetest/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The botsensetest example application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{examples/botsensetest/Makefile}

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

RNMAKE_DIST_PGMS = botsensetest 

READLINE_LIBS     = readline ncurses pthread
READLINE_CPPFLAGS = -DHAVE_READLINE

# Target library subdirectory
botsense.SUBDIR = rnr

# Libraries to Link With
botsensetest.LIBS = \
	rnr_dynamixel rnr_dxl bsclient_dyna rnr_netmsgs botsense rnr \
	stdc++ $(READLINE_LIBS) gsl gslcblas m 

botsensetest.LIBDEPS   = rnr_dynamixel rnr_dxl bsclient_dyna

# Source Files
botsensetest.SRC.CXX = botsensetest.cxx 


#------------------------------------------------------------------------------
# Extras

# install botsense library directory
EXTRA_LD_LIBDIRS = $(libdir)/botsense

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DHAVE_READLINE -DLOG -DLOGMOD="\"botsensetest\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
