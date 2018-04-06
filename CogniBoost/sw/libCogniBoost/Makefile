################################################################################
#
# ./sw/libCogniBoost/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libCogniboost library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_cogniboost Package

\pkgfile{sw/libCogniBoost/Makefile}

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
RNMAKE_DIST_STLIBS = CogniBoost

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = CogniBoost

# Source Files
CogniBoost.SRC.C = \
			cbLibCM.c \
			cbLibConn.c \
			cbLibDev.c \
			cbLibError.c \
			cbLibUtils.c \
			CogniBoostMsgs.c


# Linked Libraries 
CogniBoost.LIBS	= rnr_netmsgs rnr_serial rnr 

#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libCogniBoost\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
