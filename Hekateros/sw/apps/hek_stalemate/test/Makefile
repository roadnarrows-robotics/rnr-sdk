################################################################################
#
# ./sw/StaleMate/test/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2011-07-25 17:32:46 -0600 (Mon, 25 Jul 2011) $  
$Rev: 1154 $ 

\brief
Make StaleMate unit tests.

RN Make System Specific Makefile

\author: Robin Knight 	(robin@rnisys.com)
\author: Daniel Packard (daniel@roadnarrows.com)

##! \copyright
##!   \h_copy 2011-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

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
pkgroot		= ../../..


#------------------------------------------------------------------------------
# Programs

LOC_PGMS         = utbotplanner 

# Libraries to Link With
utbotplanner.LIBS  = \
	opencv_core \
	opencv_highgui \
	opencv_imgproc \
	opencv_calib3d \
	jpeg \
	Dynamixel \
	rnrwin \
	rnr \
	stdc++

utbotplanner.LIBDEPS   = rnrwin

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
utbotplanner.SRC.CXX = utbotplanner.cxx \
											 ../SMBotPlanner.cxx

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTStaleMate\""

ifeq "$(arch)" "overo"
EXTRA_INCDIRS  = /opt/xc/overo/include/gtk-2.0 \
								 /opt/xc/overo/lib/gtk-2.0/include \
								 /opt/xc/overo/include/pango-1.0 \
								 /opt/xc/overo/include/cairo \
								 /opt/xc/overo/include/glib-2.0 \
								 /opt/xc/overo/lib/glib-2.0/include \
								 /opt/xc/overo/include/atk-1.0 \
								 ../ \
								 ../../../include
else
EXTRA_INCDIRS  = /usr/include/gtk-2.0 \
								 /usr/lib/gtk-2.0/include \
								 /usr/include/pango-1.0 \
								 /usr/include/cairo \
								 /usr/include/glib-2.0 \
								 /usr/lib/glib-2.0/include \
								 /usr/include/atk-1.0
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
