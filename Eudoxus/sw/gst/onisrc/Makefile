################################################################################
#
# ./sw/gst/libgstonisrc/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
$Rev: 4263 $

\brief
Make GStreamer libgstonisrc plug-in library.

RN Make System Specific Makefile

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2012-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

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
# Target Specific

ifndef arch
ifdef RNMAKE_ARCH_DFT
    arch=$(RNMAKE_ARCH_DFT)
else
    arch = x86_64
endif
endif

#------------------------------------------------------------------------------
# Libraries

LIB_SUBDIR = gst

# Distribution Dynamic Linked Libraries
DIST_DLLIBS = gstonisrc

# Source Files
gstonisrc.SRC.CXX	=	gstonisrc.cxx gstoni.cxx

# Libraries to link with
gstonisrc.LIBS		= OpenNI TinyXml JPEG Eudoxus rnr gstbase-1.0 stdc++
gstonisrc.LIBDEPS = OpenNI TinyXml JPEG Eudoxus


#------------------------------------------------------------------------------
# Interface Headers


#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"gstonisrc\""

EXTRA_INCDIRS = \
	../include \

PKG_SYS_INCDIRS += \
	$(PKG_SYS_PREFIX)/include/gstreamer-1.0 \
	$(EXTRA_INCDIR_GLIBLIB) \
	$(PKG_SYS_PREFIX)/include/glib-2.0 \
	$(PKG_SYS_PREFIX)/include/libxml2 \
	$(PKG_SYS_PREFIX)/include/eigen3 


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
