################################################################################
#
# ./sw/liblaelaps/ut/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The liblaelaps library unit test makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_laelaps Small Mobile Robot Package

\pkgfile{sw/liblaelaps/ut/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2015-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_TEST_PGMS = utRobot utThread utXmlCfg utXmlTunes

COMMON_LIBS  = gtest laelaps rnr_appkit rnr pthread stdc++

# Libraries to Link With
utRobot.LIBS 		= $(COMMON_LIBS)
utThread.LIBS 	= $(COMMON_LIBS)
utXmlCfg.LIBS 	= $(COMMON_LIBS)
utXmlTunes.LIBS = $(COMMON_LIBS)

utRobot.LIBDEPS  		= laelaps
utThread.LIBDEPS  	= laelaps
utXmlCfg.LIBDEPS 		= laelaps
utXmlTunes.LIBDEPS 	= laelaps

# Source Files
utRobot.SRC.CXX 	 	= utRobot.cxx
utThread.SRC.CXX 	 	= utThread.cxx
utXmlCfg.SRC.CXX 	 	= utXmlCfg.cxx
utXmlTunes.SRC.CXX 	= utXmlTunes.cxx


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTliblaelaps\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
