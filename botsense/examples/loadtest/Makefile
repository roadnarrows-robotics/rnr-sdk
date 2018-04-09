################################################################################
#
# ./examples/loadtest/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The bsProxy server load tester example makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{examples/loadtest/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_LOC_PGMS = bsLoadTest 

# Libraries to Link With
bsLoadTest.LIBS = bsclient_i2c \
									bsclient_null \
									bsclient_serial \
									botsense \
									rnr_netmsgs rnr pthread

bsLoadTest.LIBDEPS = 	bsclient_i2c \
											bsclient_null \
											bsclient_serial \
											botsense


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
bsLoadTest.SRC.C = bsLoadTest.c


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"bsLoadTest\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
