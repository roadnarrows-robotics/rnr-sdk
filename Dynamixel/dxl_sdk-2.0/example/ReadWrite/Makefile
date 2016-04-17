################################################################################
#
# ./dxl_sdk-2.0/example/ReadWrite/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
$Rev: 3845 $

\brief Make Robotis Dynamixel SDK 2.0 read_write example.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

\par Copyright:
  (C) 2013.  RoadNarrows LLC
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

#
# RoadNarrows Intelligent Systesm End User License Agreement
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
pkgroot		= ../../..

#------------------------------------------------------------------------------
# Programs

LOC_PGMS = read_write 

# Libraries to Link With
read_write.LIBS      = dxl2 stdc++
read_write.LIBDEPS   = dxl2

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
read_write.SRC.CPP = main.cpp

#------------------------------------------------------------------------------
# Optional Variables 
#

#------------------------------------------------------------------------------
# Make Includes
#
EXTRA_INCDIRS = ../../include


# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
