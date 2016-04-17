################################################################################
#
# ./sw/gst/pcsviewersink/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
$Rev: 4263 $

\brief
Make GStreamer Point Cloud Stream viewer sink plug-in libraries.


RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
  (C) 2012-2015.  RoadNarrows LLC.
  (http://www.roadnarrows.com)
  All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../../..

#------------------------------------------------------------------------------
# Libraries

LIB_SUBDIR = gst

# Distribution Dynamic Linked Libraries
DIST_DLLIBS = gstpcsviewersink

# Source Files
gstpcsviewersink.SRC.CXX	=	gstpcsviewersink.cxx gstpcsviewer.cxx

# Libraries to link with
gstpcsviewersink.LIBS = \
	Eudoxus \
	rnr \
	gstbase-1.0 \
	gobject-2.0 \
	glib-2.0 \
	pcl_common \
	pcl_io \
	pcl_visualization \
	stdc++

gstpcsviewersink.LIBDEPS = Eudoxus


#------------------------------------------------------------------------------
# Interface Headers


#------------------------------------------------------------------------------
# Optional Variables 
#

EXTRA_INCDIR_GLIBLIB=$(PKG_SYS_PREFIX)/lib/$(arch)-linux-gnu/glib-2.0/include

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gstpcsviewersink\""

EXTRA_INCDIRS = \
	../include

PKG_SYS_INCDIRS += \
	$(PKG_SYS_PREFIX)/include/gstreamer-1.0 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(PKG_SYS_PREFIX)/include/glib-2.0 \
	$(PKG_SYS_PREFIX)/include/libxml2 \
	$(PKG_SYS_PREFIX)/include/eigen3 \
	$(PKG_SYS_PREFIX)/include/vtk-5.8


#------------------------------------------------------------------------------
# Make Includes
#

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
