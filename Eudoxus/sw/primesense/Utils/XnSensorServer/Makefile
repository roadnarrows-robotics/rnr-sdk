################################################################################
#
# ./sw/primesense/Utils/XnSensorServer/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-07-12 16:22:51 -0600 (Thu, 12 Jul 2012) $  
$Rev: 2083 $ 

\brief
Make XnSensorServor application.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

\par Copyright:
  (C) 2012.  RoadNarrows Intelligent Systems.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# RoadNarrows Robotics End User License Agreement
#
# This open source file provided by RoadNarrows Intelligent Systems is covered
# under the GNU Lesser General Public License (LGPLv3).
#
# See http://www.opensource.org/licenses/lgpl-3.0.html for the complete
# specification of the license
#
# See the Licenses sections under http://www.rnisys.com/support/robosight
# for the complete definitions and specifications of RoadNarrows Intelligent
# Systems LGPLv3 software and license agreement.
#
# Warranties:
# In no event shall the authors, RoadNarrows Intelligent Systems or
# members/employees of RoadNarrows Intelligent Systems or distributors of this
# software be liable to any party for direct, indirect, special, incidental,
# or consequential damages arising out of the use of this software and its
# documentation, even if the authors or any of the above parties have been
# advised of the possibility of such damage.
#
# The authors and RoadNarrows Intelligent Systems specifically disclaim any
# warranties, including, but not limited to, the implied warranties of
# merchantability and fitness for a particular purpose. The software provided
# hereunder is on an “as-is” basis, and the authors and distributors have no
# obligation to provide maintenance, support, updates, enhancements, or
# modifications.
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../../..


#------------------------------------------------------------------------------
# Programs

DIST_PGMS = XnSensorServer 

# Libraries to Link With
XnSensorServer.LIBS =	\
	XnCore \
	XnDDK \
	XnFormats \
	XnDeviceSensorV2 \
	OpenNI \
	JPEG \
	TinyXml \
	stdc++ \
	pthread 

XnSensorServer.LIBDEPS = \
	XnCore XnDDK XnFormats XnDeviceSensorV2 OpenNI JPEG TinyXml


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
XnSensorServer.SRC.CPP = SensorServer.cpp

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"XnSensorServer\""

EXTRA_INCDIRS = ../..

ifeq "$(arch)" "overo"
else
endif

#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
