################################################################################
#
# ./sw/liblaelaps/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The liblaelaps library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_laelaps Small Mobile Robot Package

\pkgfile{sw/liblaelaps/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = laelaps

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = laelaps

# Source Files
laelaps.SRC.CXX  = \
	laeAlarms.cxx \
	laeBatt.cxx \
	laeDb.cxx \
	laeDesc.cxx \
	laeGpio.cxx \
	laeI2C.cxx \
	laeI2CMux.cxx \
	laeImu.cxx \
	laeKin.cxx \
	laeMotor.cxx \
	laePlatform.cxx \
	laePowertrain.cxx \
	laeReports.cxx \
	laeRobot.cxx \
	laeThread.cxx \
	laeThreadAsync.cxx \
	laeThreadImu.cxx \
	laeThreadKin.cxx \
	laeThreadRange.cxx \
	laeThreadWd.cxx \
	laeTraj.cxx \
	laeTune.cxx \
	laeUtils.cxx \
	laeVL6180.cxx \
	laeWd.cxx \
	laeXmlCfg.cxx \
	laeXmlTune.cxx \
	RoboClaw.cxx

# Linked Libraries 
laelaps.LIBS	= \
	botsense \
	rnr_dynamixel \
	rnr_netmsgs \
	rnr_gpio \
	rnr_i2c \
	rnr_serial \
	rnr_appkit \
	rnr_tinyxml \
	rnr \
	pthread \
	stdc++


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"liblaelaps\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

