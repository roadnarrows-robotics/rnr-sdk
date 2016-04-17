################################################################################
#
# Package: RN Make System 
# File:    Arch.armdroid.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System Architecture Makefile

\par Architecture:
Android armeabi-v7a (I.E. TI OMAP3 32-bit Gumstix processors)

\par Build Host:
Cross-Compile

\par Tool-Chain:
arm-linux-androideabi-4.6-*

$LastChangedDate: 2013-02-19 08:26:11 -0700 (Tue, 19 Feb 2013) $
$Rev: 2694 $

\author Brent Wilkins (brent@roadnarrows.com)

\par Copyright:
(C) 2009-2013  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
# Description:
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
################################################################################

# Prevent mutliple inclusion
ARCH_MK							= 1

# This architecture (required)
ARCH         				= armdroid
ARCH_FQNAME         = arm-android-4.6

#
# RoadNarrows Install Prefix (override as necessary)
#
ifndef prefix
	ifdef prefix_root
		prefix          = $(prefix_root)/$(ARCH)
	else
		prefix					= $(topdir)/xinstall/$(ARCH)
	endif
endif

# Architecture Include Directories
ARCH_INCDIRS       	= \
	/opt/pkg/android-ndk-r8d/platforms/android-9/arch-arm/usr/include \
	/opt/pkg/android-ndk-r8d/sources/cxx-stl/gnu-libstdc++/4.6/include

# Architecture CPP Flags
ARCH_CPPFLAGS       =

# Cross compiler tool chain prefix
CROSS_COMPILE       = arm-linux-androideabi-

# Build Support Commands
AR                  = $(CROSS_COMPILE)ar
RANLIB              = $(CROSS_COMPILE)ranlib
STRIP_LIB						= $(CROSS_COMPILE)strip --strip-debug
STRIP_EXE						= $(CROSS_COMPILE)strip --strip-all

# C
CC                  = $(CROSS_COMPILE)gcc
CFLAGS_CODEGEN			= -fPIC
CFLAGS_DEBUG        = -g
CFLAGS_OPTIMIZE     = -O2
CFLAGS_WARNING      = -Wall -Wno-implicit-int
CFLAGS_CPP_ONLY     = -E
CFLAGS_DEPS_ONLY    = -M
CFLAGS              = $(CFLAGS_CODEGEN) \
											$(CFLAGS_DEBUG) \
                      $(CFLAGS_OPTIMIZE) \
                      $(CFLAGS_WARNING)

# C++
CXX                 = $(CROSS_COMPILE)g++
CFLAGS_CODEGEN			= -fPIC
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = $(CFLAGS_CODEGEN) \
											$(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)

# LD (linker)
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
LDFLAGS             = $(COMMON_LDFLAGS) # -Wl,--export-dynamic
LD_LIBPATHS         =
LD_LIBS             =

# Static Libs
STLIB_LD            = ${AR} cr
STLIB_PREFIX        = lib
STLIB_SUFFIX        = .a

# Shared Libs
SHLIB_LD            = $(CC) -shared
SHLIB_PREFIX        = lib
SHLIB_SUFFIX        = .so
SHLIB_LD_EXTRAS     = 
SHLIB_LD_FLAGS      = $(COMMON_LDFLAGS)
SHLIB_LD_LIBS       = ${LIBS}
SHLIB_CFLAGS        = -fPIC

# Dynamically Linked Libraries
DLLIB_LD            = $(CC) -shared
DLLIB_PREFIX        = lib
DLLIB_SUFFIX        = .so
DLLIB_LD_NOSTART		= -nostartfiles
DLLIB_LD_EXTRAS     = 
DLLIB_LD_FLAGS      = $(COMMON_LDFLAGS)
DLLIB_LD_LIBS       = ${LIBS}
DLLIB_CFLAGS        = -fPIC
DLLIB_APP_CFLAGS    = -rdynamic -fPIC
DLLIB               = dl

# Make C/CXX Dependencies Command
MAKEDEPS						= $(CC) $(CFLAGS_DEPS_ONLY)

# Python
PYTHON_ENABLED	=	n

# SWIG: Simplified Wrapper and Interface Generator command
SWIG_ENABLED	=	n
SWIG_CFLAGS 	=	$(CFLAGS_CODEGEN) \
								-pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions
SWIG_INCLUDES	= -I/usr/include/python2.6
SWIG_LDFLAGS 	=	


#
# System and Optional Packages
#

SYS_PREFIX = $(OE_TMP)/sysroots/armv7a-angstrom-linux-gnueabi/usr
OPT_PREFIX = /opt/xinstall/$(ARCH)

# OpenCV
OPENCV_COM_LIBS   = opencv_core opencv_imgproc opencv_highgui

# GTK (and related)
GTK_INCDIR        = $(SYS_PREFIX)/include/gtk-2.0
PANGO_INCDIR      = $(SYS_PREFIX)/include/pango-1.0
CAIRO_INCDIR      = $(SYS_PREFIX)/include/cairo
ATK_INCDIR        = $(SYS_PREFIX)/include/atk-1.0
GDK_PIXBUF_INCDIR = $(SYS_PREFIX)/include/gdk-pixbuf
GTK_LIB_INCDIR		= $(SYS_PREFIX)/lib/gtk-2.0/include
GTK_LIB					  = gtk-x11-2.0
GDK_LIB						= gdk-x11-2.0
GDK_PIXBUF_LIB		= gdk_pixbuf-2.0
ATK_LIB						= atk-1.0
PANGO_LIBS				= pango-1.0 pangoft2-1.0 pangocairo-1.0
CAIRO_LIB					= cairo
FONT_LIBS					= freetype fontconfig
GOBJ_LIBS					= gobject-2.0 gmodule-2.0 gthread-2.0 gio-2.0 rt

# GStreamer
GST_INCDIR        = $(SYS_PREFIX)/include/gstreamer-0.10
GST_LIB_IF				= gstinterfaces-0.10

# GLib
GLIB_INCDIR			  = $(SYS_PREFIX)/include/glib-2.0
GLIB_LIB_INCDIR		=	$(SYS_PREFIX)/lib/glib-2.0/include
GLIB_LIB					= glib-2.0

# XML
LIBXML2_INCDIR		= $(SYS_PREFIX)/include/libxml2

# File System
SYSFS_INCDIR			=	$(SYS_PREFIX)/include/sysfs




# Posix Thread Library:
PTHREADLIB          = -lpthread 
PTHREADLIB_INCPATH  = 
PTHREADLIB_LIBPATH  = 
PTHREADLIB_CPPFLAGS =

# Jpeg
JPEGINCPATH         = 
JPEGLIBPATH         = 
JPEGLIB             =  -ljpeg 

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
