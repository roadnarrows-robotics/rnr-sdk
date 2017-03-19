################################################################################
#
# ./sw/apps/euStitch/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
$Rev: 4267 $

\brief
Make Eudoxus stitch application.

RN Make System Specific Makefile

\author: Robin Knight   (robin@roadnarrows.com)
\author: Rob Shiely     (rob@roadnarrows.com)

##! \copyright
##!   \h_copy 2012-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../..

#------------------------------------------------------------------------------
# Target Specific

ifndef arch
ifdef RNMAKE_ARCH_DFT
    arch=$(RNMAKE_ARCH_DFT)
else
    arch = x86_64
endif
endif

ifeq "$(arch)" "linaro"
	DIST_PGMS = 
else
ifeq "$(arch)" "overo"
	DIST_PGMS =
else
	DIST_PGMS = euStitch
endif # not overo
endif # not linaro


#------------------------------------------------------------------------------
# Programs

ifneq "$(arch)" "overo"
endif

# Libraries to Link With
euStitch.LIBS = \
	Eudoxus \
	rnr \
	gstbase-1.0 \
	gobject-2.0 \
	glib-2.0 \
	pcl_common \
	pcl_io \
	pcl_filters \
	pcl_kdtree \
	pcl_search \
	pcl_visualization \
	boost_system \
	espeak \
	pthread \
	m \
	stdc++

euStitch.LIBDEPS   = Eudoxus


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
euStitch.SRC.CXX = euStitch.cxx

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"euStitch\"" \

EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

EXTRA_INCDIRS = \
	../include \
	$(PKG_SYS_PREFIX)/include/gstreamer-1.0 \
	$(PKG_SYS_PREFIX)/include/glib-2.0 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(PKG_SYS_PREFIX)/include/libxml2 \
	$(PKG_SYS_PREFIX)/include/eigen3 \
	$(PKG_SYS_PREFIX)/include/pcl-1.7 \
	$(PKG_SYS_PREFIX)/include/vtk-5.8

EXTRA_LD_LIBDIRS = /opt/lib

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
