################################################################################
#
# ./sw/librnr_appkit/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_appkit library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/librnr_appkit/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_appkit

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_appkit

# Target library subdirectory
rnr_appkit.SUBDIR = rnr

# Source Files
rnr_appkit.SRC.CXX = \
	CmdAddOns.cxx \
	CmdArgDef.cxx \
	CmdCore.cxx \
	CmdDef.cxx \
	CmdExtArg.cxx \
	CmdFormDef.cxx \
	CommandLine.cxx \
	IOManip.cxx \
	LogBook.cxx \
	Random.cxx \
	ReadLine.cxx \
	RegEx.cxx \
	Session.cxx \
	State.cxx \
	StateKb.cxx \
	StateMach.cxx \
	StringTheory.cxx \
	Thread.cxx \
	Time.cxx \
	Token.cxx \
	Xml.cxx

rnr_appkit.LIBS    = rnr_tinyxml rnr readline
rnr_appkit.LIBDEPS = rnr_tinyxml rnr readline


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_appkit\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
