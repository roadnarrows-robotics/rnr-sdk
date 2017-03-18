################################################################################
#
# ./sw/librnr_cam/ut/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-07-13 14:28:26 -0600 (Sat, 13 Jul 2013) $
$Rev: 3128 $

\brief Library librnr_cam unit tests.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)
\author: Daniel Packard (daniel@roadnarrows.com) 

##! \par Copyright
##!   \h_copy 2011-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../..

#------------------------------------------------------------------------------
# Programs

# Distribution Static Libraries
TEST_PGMS = ut-librnr_cam

# Linked Libraries 
ut-librnr_cam.LIBS = \
	gtest rnr_appkit rnr_cam rnr_win rnr_wingtk rnr \
  $(OPENCV_COM_LIBS) \
	$(GTK_LIB) \
	pthread stdc++

ut-librnr_cam.LIBDEPS	= gtest rnr_appkit rnr_cam rnr_win rnrwin_gtk

#------------------------------------------------------------------------------
# Sources

# C++ Source Files
ut-librnr_cam.SRC.CXX = \
	ut-librnr_cam-main.cxx \
	ut-Camera.cxx

#------------------------------------------------------------------------------
# Optional Variables 
#

# Includes
EXTRA_SYS_INCDIRS	+= \
	$(GTK_INCDIR) \
	$(GTK_LIB_INCDIR) \
	$(GDK_PIXBUF_INCDIR) \
	$(PANGO_INCDIR) \
	$(CAIRO_INCDIR) \
	$(ATK_INCDIR) \
	$(GST_INCDIR) \
	$(LIBXML2_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)

# library subdir
LIB_SUBDIR = rnr

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"UTlibrnr_cam\""

EXTRA_INCDIRS += /opt/include

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
