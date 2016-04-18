################################################################################
#
# ./sw/libWMSuite/tests/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-10-27 14:47:17 -0600 (Thu, 27 Oct 2011) $
$Rev: 1438 $

\brief
Library libQik unit tests.

RN Make System Specific Makefile

\author: Rob Shiely 	(rob@roadnarrows.com) 

\par Copyright:
  (C) 2012.  University of Northern Colorado
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../..

#------------------------------------------------------------------------------
# Programs

# Distribution Programs
LOC_PGMS = QikMove

# Linked Libraries 
LOC_PGMS_LIBS				=  rnr  qik serial
										
QikMove.LIBS		= $(LOC_PGMS_LIBS)

#------------------------------------------------------------------------------
# Sources

# C++ Source Files
QikMove.SRC.C	= QikMove.c

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libQik\""

#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
