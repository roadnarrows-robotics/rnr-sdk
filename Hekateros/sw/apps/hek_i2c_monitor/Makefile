################################################################################
#
# ./sw/apps/hek_i2c_monitor/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The hek_i2c_monitor application makefile.

GFS-Monitor and hekateros limits applications.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@RoadNarrows.com)

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/apps/hek_i2c_monitor/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Brent Wilkins,Brent@RoadNarrows.com}

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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = i2c-monitor hek_limits

# Libraries to Link With
i2c-monitor.LIBS = rnr_i2c rnr

hek_limits.LIBS = rnr_i2c rnr

# C Source Files
i2c-monitor.SRC.C = i2c-monitor.c
hek_limits.SRC.C = hek_limits.c


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
i2c-monitor::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"i2c-monitor\""
hek_limits::EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"hek_limits\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
