################################################################################
#
# ./examples/botsensetest/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2012-05-12 14:04:56 -0600 (Sat, 12 May 2012) $
$Rev: 1985 $

\brief Make botsensetest example.

RN Make System Specific Makefile

\author: Robin Knight (robin@rnisys.com)

\par Copyright:
  (C) 2011-2016.  RoadNarrows Intelligent Systems.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

#------------------------------------------------------------------------------
# Programs

DIST_PGMS         = botsensetest 

READLINE_LIBS     = readline ncurses pthread
READLINE_CPPFLAGS = -DHAVE_READLINE

# botsense modules are located in a sub-directory of lib
LIB_SUBDIR = $(PKG_LIB_PLUGIN_SUBDIR) $(libdir)/botsense

# Libraries to Link With
botsensetest.LIBS = \
	rnr_dynamixel rnr_dxl bsclient_dyna rnr_netmsgs botsense rnr \
	stdc++ $(READLINE_LIBS) gsl gslcblas m 

botsensetest.LIBDEPS   = rnr_dynamixel rnr_dxl bsclient_dyna

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
botsensetest.SRC.CXX = \
	botsensetest.cxx 

#------------------------------------------------------------------------------
# Optional Variables 
#

# Link Library Extra Library Directories (exluding local library)
EXTRA_LD_LIBDIRS = $(DISTROOT_LIB)/rnr

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DHAVE_READLINE -DLOG -DLOGMOD="\"botsensetest\""

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
