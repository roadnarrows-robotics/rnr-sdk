################################################################################
#
# ./sw/libhekateros/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The libhekateros makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_hek Robotic Arm Package

\pkgfile{sw/libhekateros/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Rob Shiely,rob@roadnarrows.com} 

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

RNMAKE_SUBDIRS =  ut


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = hekateros

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = hekateros

# Source Files
hekateros.SRC.CXX = \
	hekCalib.cxx \
	hekCalibStretch.cxx \
	hekDesc.cxx \
	hekDescArm.cxx \
	hekDescEE.cxx \
	hekJoint.cxx \
	hekKin.cxx \
	hekKinJoint.cxx \
	hekMonitor.cxx \
	hekPid.cxx \
	hekProdArm4L.cxx \
	hekProdArm5L.cxx \
	hekProdArm5LBeta.cxx \
	hekProdEEGraboid.cxx \
	hekRobot.cxx \
	hekSpec.cxx \
	hekState.cxx \
	hekSysBoard.cxx \
	hekTune.cxx \
	hekUno.cxx \
	hekUtils.cxx \
	hekXmlCfg.cxx \
	hekXmlTune.cxx

# Linked Libraries 
hekateros.LIBS	= \
	rnr_dynamixel \
	bsclient_dyna \
	botsense \
	rnr_netmsgs \
	rnr_i2c \
	rnr_appkit \
	rnr_serial \
	rnr_tinyxml \
	rnr \
	stdc++


#------------------------------------------------------------------------------
# Extras

# install botsense library directory
EXTRA_LD_LIBDIRS = $(libdir)/botsense

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libhekateros\"" -DLOGMOD_COLOR=LOG_COLOR_BLUE


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
