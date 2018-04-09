################################################################################
#
# ./libDynamixel/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The librnr_dynamixel library makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_dynamixel Package

\pkgfile{libDynamixel/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

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
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = rnr_dynamixel

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = rnr_dynamixel

# Target library subdirectory
rnr_dynamixel.SUBDIR = rnr

# Source Files
rnr_dynamixel.SRC.CXX	=	\
	DynaBgThread.cxx \
	DynaChain.cxx \
	DynaComm.cxx \
	DynaCommBotSense.cxx \
	DynaCommSerial.cxx \
	DynaError.cxx \
	DynaPid.cxx \
	DynaPidPos.cxx \
	DynaPidSpeed.cxx \
	DynaServo.cxx \
	DynaServoGeneric.cxx \
	DynaServoAX12.cxx \
	DynaServoEX106P.cxx \
	DynaServoMX12W.cxx \
	DynaServoMX28.cxx \
	DynaServoMX64.cxx \
	DynaServoMX106.cxx \
	DynaServoRX10.cxx \
	DynaServoRX24F.cxx \
	DynaServoRX28.cxx \
	DynaServoRX64.cxx

# Libraries to link with
rnr_dynamixel.LIBS = \
	rnr_dxl \
	bsclient_dyna \
	botsense \
	rnr_netmsgs \
	rnr_serial \
	rnr_gpio \
	rnr \
	pthread rt

rnr_dynamixel.LIBDEPS = rnr_dxl


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_dynamixel\""


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
