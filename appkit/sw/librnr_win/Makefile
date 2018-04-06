################################################################################
#
# ./sw/librnr_win/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_win library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_win/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_win

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_win

# Target library subdirectory
rnr_win.SUBDIR = rnr

# Source Files
rnr_win.SRC.CXX  = \
	Win.cxx \
	WinCvIoI.cxx \
	WinCvMouse.cxx \
	WinMenu.cxx \
	SessionWin.cxx \
	StateWin.cxx

# Library Package Dependencies
rnr_win.LIBDEPSS =

# Linked Libraries 
rnr_win.LIBS =	rnr_appkit rnr stdc++


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_win\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
