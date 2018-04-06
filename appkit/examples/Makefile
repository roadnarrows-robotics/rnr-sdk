################################################################################
#
# ./examples/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_appkit package examples makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{examples/Makefile}

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
# Programs

RNMAKE_DIST_PGMS = rnr_eg_cli

# Libraries to Link With
rnr_eg_cli.LIBS     = rnr_appkit rnr_tinyxml rnr stdc++
rnr_eg_cli.LIBDEPS	= rnr_appkit rnr_tinyxml

# Sources
rnr_eg_cli.SRC.CXX = rnr_eg_cli.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"rnr_eg\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
