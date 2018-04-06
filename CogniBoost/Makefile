################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_cogniboost package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_cogniboost Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Brent Wilkins,brent@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= .

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = CogniBoostMsgs fw sw examples


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = CogniBoost

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
#       $(topdir)/include. Otherwise, is it really a deliverable header?
#
CogniBoost.HDRS.H = \
	CogniBoost/CogniBoost.h \
	CogniBoost/CogniBoostMsgs.h \
	CogniBoost/libCogniBoost.h


#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE = $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
