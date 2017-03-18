
#
# ./sw/librnr_cam/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2013-07-15 11:45:50 -0600 (Mon, 15 Jul 2013) $
$Rev: 3131 $

\brief Make the librnr_cam library.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \par Copyright
##!   \h_copy 2012-2017. RoadNarrows LLC.\n
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

SUBDIRS =  ut

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS = rnr_cam

# Distribution Shared Libraries
DIST_SHLIBS = rnr_cam

# library subdir
LIB_SUBDIR = rnr

# Source Files
rnr_cam.SRC.CXX = \
	Camera.cxx \
	CameraCv.cxx \
	CameraEcon32.cxx \
	CameraEcon50.cxx \
	CameraGst.cxx

rnr_cam.LIBS  = \
	tinyxml rnr_appkit rnr_win rnr_wingtk rnr_winqt rnr \
	$(OPENCV_COM_LIBS) \
	$(GTK_LIB) $(GDK_LIB) $(GDK_PIXBUF_LIB) $(ATK_LIB) $(PANGO_LIBS) \
	$(CAIRO_LIB) $(FONT_LIBS) $(GOBJ_LIBS) \
	$(GST_LIB_IF) $(GLIB_LIB)

rnr_cam.LIBDEPS  = rnr_tinyxml rnr_appkit rnr_win rnr_wingtk rnr_winqt

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

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"librnr_cam\""

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
