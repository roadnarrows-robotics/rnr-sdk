################################################################################
#
# ./sw/libkuon/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libkuon library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/libkuon/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2014-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

#RNMAKE_SUBDIRS =  ut

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = kuon

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = kuon

# Source Files
kuon.SRC.CXX = \
	kuonDesc.cxx \
	kuonDescBase.cxx \
	kuonJoint.cxx \
	kuonProdBaseStd.cxx \
	kuonRobot.cxx \
	kuonSpec.cxx \
	kuonStatus.cxx \
	kuonUtils.cxx \
	kuonXmlCfg.cxx \

# Linked Libraries 
kuon.LIBS	= \
	RS160D \
	rnr_appkit \
	rnr_serial \
	rnr_tinyxml \
	rnr \
	stdc++


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libkuon\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
