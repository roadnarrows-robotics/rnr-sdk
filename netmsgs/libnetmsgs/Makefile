################################################################################
#
# ./libnetmsgs/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libnetmsgs library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Network Messaging Package

\pkgfile{libnetmsgs/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_netmsgs

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_netmsgs

# Target library subdirectory
rnr_netmsgs.SUBDIR = rnr

# Source Files
rnr_netmsgs.SRC.C =	nmLibInternal.c \
										nmLibPack.c \
 										nmLibPackFlat.c \
										nmLibPackITV.c \
										nmLibUtils.c

# Linked libraries
rnr_netmsgs.LIBS = rnr


#------------------------------------------------------------------------------
# Extras
#

EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_netmsgs\"" -DNMLIB_DEBUG_ENABLE


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
