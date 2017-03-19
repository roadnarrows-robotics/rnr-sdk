################################################################################
#
# ./sw/libEudoxus/Makefile

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
$Rev: 4263 $

\brief
Make libEudoxus library.

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
pkgroot		= ../..

SUBDIRS 	= ut

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
	PCLLIBS = 
else
ifeq "$(arch)" "overo"
	PCLLIBS = 
else
	PCLLIBS = pcl_common pcl_io
endif # overo
endif # linaro

#------------------------------------------------------------------------------
# Libraries

# Distribution Static Libraries
DIST_STLIBS 			= Eudoxus

# Distribution Shared Libraries
DIST_SHLIBS 			= Eudoxus

# Source Files
Eudoxus.SRC.CXX	=	\
	euClientUdp.cxx \
	euEnv.cxx \
	euError.cxx \
	euIo.cxx \
	euOni.cxx \
	euPcd.cxx \
	euPcs.cxx \
	euUtils.cxx


# Libraries to link with
Eudoxus.LIBS = \
	rnr \
	TinyXml \
	JPEG \
  $(PCLLIBS) \
	OpenNI \
	boost_system \
	gstreamer-1.0 \
	gstbase-1.0 \
	gstvideo-1.0 \
	glib-2.0 \
	gobject-2.0 \
	m \
	stdc++

#	gstinterfaces-1.0 \
Eudoxus.LIBDEPS = #OpenNi


#------------------------------------------------------------------------------
# Interface Headers


#------------------------------------------------------------------------------
# Optional Variables 
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"libEudoxus\""

PKG_SYS_INCDIRS += \
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
