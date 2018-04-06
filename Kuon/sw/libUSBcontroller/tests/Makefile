################################################################################
#
# ./sw/libUSBcontroller/tests/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Library libUSBcontroller unit tests makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{sw/libUSBcontroller/tests/Makefile}

\pkgauthor{Rob Shiely,rob@roadnarrows.com} 
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
RNMAKE_PKG_ROOT	= ../../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Programs

# Distribution Programs
RNMAKE_TEST_PGMS = utcontroller

# Linked Libraries 
TEST_PGMS_LIBS = rnr stdc++ usb-1.0 USBcontroller
										
utcontroller.LIBS	= $(TEST_PGMS_LIBS)

# C++ Source Files
utcontroller.SRC.C	= utcontroller.c


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibUSBcontroller\""


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
