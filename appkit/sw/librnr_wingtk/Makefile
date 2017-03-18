################################################################################
#
# ./sw/librnr_wingtk/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-09-19 15:57:54 -0600 (Thu, 19 Sep 2013) $
$Rev: 3318 $

\brief
Make the librnr_wingtk library.

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

#
# @EulaBegin@
# 
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
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ../..

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS       = rnr_wingtk

# Distribution Shared Libraries
DIST_SHLIBS       = rnr_wingtk

# library subdir
LIB_SUBDIR = rnr

# Source Files
rnr_wingtk.SRC.CXX  = \
	WinGtk.cxx \
	WinGtkMenu.cxx

#	WinGtkIoI.cxx \
#	WinGtkMenu.cxx \
#	WinGtkMouse.cxx \
#	WinGtkMsg.cxx \
#	WinGtkUtil.cxx 

# Library Package Dependencies
rnr_wingtk.LIBDEPS =	rnr_win

# Linked Libraries 
rnr_wingtk.LIBS =	\
	rnr_win rnr \
	$(OPENCV_COM_LIBS) \
  $(GTK_LIB) $(GDK_LIB) $(GDK_PIXBUF_LIB) $(ATK_LIB) $(PANGO_LIBS) \
  $(CAIRO_LIB) $(FONT_LIBS) $(GOBJ_LIBS) \
  $(GST_LIB_IF) $(GLIB_LIB) \
	stdc++

#opencv_core opencv_highgui opencv_imgproc \
#									jpeg \
#									rnr \
#									stdc++ \
#									pthread \
#									gtk-x11-2.0 gdk-x11-2.0 atk-1.0 gio-2.0 \
#									pangoft2-1.0 gdk_pixbuf-2.0 m pangocairo-1.0 \
#									cairo pango-1.0 \
#									freetype fontconfig \
#									gobject-2.0 gmodule-2.0 gthread-2.0 rt glib-2.0

#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_wingtk\""

EXTRA_INCDIRS = \
	$(GTK_INCDIR) \
	$(PANGO_INCDIR) \
	$(CAIRO_INCDIR) \
	$(ATK_INCDIR) \
	$(GDK_PIXBUF_INCDIR) \
	$(GTK_LIB_INCDIR) \
	$(GLIB_INCDIR) \
	$(GLIB_LIB_INCDIR)

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
