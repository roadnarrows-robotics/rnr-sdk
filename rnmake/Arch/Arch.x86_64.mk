################################################################################
#
# Package: RN Make System 
# File:    Arch.x86_64.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System Architecture Makefile

\par Architecture:
Linux (Posix) AMD 64-bit.

\par Build Host:
Native or Cross-Compile

$LastChangedDate: 2013-02-19 08:26:11 -0700 (Tue, 19 Feb 2013) $
$Rev: 2694 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2005-2018.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
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
RNMAKE_ARCH         = x86_64
RNMAKE_ARCH_FQNAME  = x86_64-linux-gnu

# Architecture Include Directories
ARCH_INCDIRS       	=

# Architecture CPP Flags
ARCH_CPPFLAGS       =

# Build Support Commands
AR                  = ar
RANLIB              = ranlib
STRIP_LIB						= strip --strip-debug
STRIP_EXE						= strip --strip-all

# C
CC                  = gcc
CFLAGS_CODEGEN			= -fPIC -m64
CFLAGS_DEBUG        = -g
CFLAGS_OPTIMIZE     = -O2
CFLAGS_WARNING      = -Wall -Wconversion -Wno-implicit-int
CFLAGS_CPP_ONLY     = -E
CFLAGS_DEPS_ONLY    = -M
CFLAGS              = $(CFLAGS_CODEGEN) \
											$(CFLAGS_DEBUG) \
                      $(CFLAGS_OPTIMIZE) \
                      $(CFLAGS_WARNING)

# C++
CXX                 = c++
CXXFLAGS_CODEGEN		= -fPIC -m64
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = $(CXXFLAGS_CODEGEN) \
											$(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)

# CUDA
CUDA                = nvcc
CUDACFLAGS_CODEGEN  = 
CUDAFLAGS_DEBUG     = -g
CUDAFLAGS_OPTIMIZE  = -O2
CUDAFLAGS_CPP_ONLY  = -E
CUDAFLAGS           = $(CUDAFLAGS_CODEGEN) \
											$(CUDAFLAGS_DEBUG) \
                      $(CUDAFLAGS_OPTIMIZE) \
                      $(CUDAFLAGS_WARNING)

# LD (linker)
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD_CUDA             = $(CUDA)
LD									= $(LD_CC)
LDFLAGS             = # -Wl,--export-dynamic
LD_LIBPATHS         =
LD_LIBS             =

# Static Libraries
STLIB_LD            = ${AR} cr
STLIB_PREFIX        = lib
STLIB_SUFFIX        = .a

# Shared Libraries
SHLIB_LD            = $(CC) -shared
SHLIB_PREFIX        = lib
SHLIB_SUFFIX        = .so
SHLIB_LD_EXTRAS     =
SHLIB_LD_FLAGS      =
SHLIB_LD_LIBS       = ${LIBS}
SHLIB_CFLAGS        = -fPIC

# Dynamically Linked Libraries
DLLIB_LD            = $(CC) -shared -m64
DLLIB_PREFIX        = lib
DLLIB_SUFFIX        = .so
DLLIB_LD_NOSTART		= -nostartfiles
DLLIB_LD_EXTRAS     =
DLLIB_LD_FLAGS      =
DLLIB_LD_LIBS       = ${LIBS}
DLLIB_CFLAGS        = -fPIC
DLLIB_APP_CFLAGS    = -rdynamic -fPIC
DLLIB               = dl

# Make C/CXX Dependencies Command
MAKEDEPS						= $(CC) $(CFLAGS_DEPS_ONLY)

# Python
PYTHON_ENABLED	=	y

# SWIG: Simplified Wrapper and Interface Generator command
SWIG_ENABLED	=	y
SWIG_CFLAGS 	=	$(CFLAGS_CODEGEN) \
								-pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions
SWIG_INCLUDES	= 
SWIG_LDFLAGS 	=	


#
# System and Optional Packages
#

SYS_PREFIX = /usr
OPT_PREFIX = /opt/xinstall/$(RNMAKE_ARCH)

# OpenCV
OPENCV_COM_LIBS   = opencv_core opencv_imgproc opencv_highgui

# GTK (and related)
GTK_INCDIR        = $(SYS_PREFIX)/include/gtk-2.0
PANGO_INCDIR      = $(SYS_PREFIX)/include/pango-1.0
CAIRO_INCDIR      = $(SYS_PREFIX)/include/cairo
ATK_INCDIR        = $(SYS_PREFIX)/include/atk-1.0
GDK_PIXBUF_INCDIR = $(SYS_PREFIX)/include/gdk-pixbuf-2.0
GTK_LIB_INCDIR		= $(SYS_PREFIX)/lib/$(RNMAKE_ARCH)-linux-gnu/gtk-2.0/include
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
GLIB_LIB_INCDIR		=	$(SYS_PREFIX)/lib/$(RNMAKE_ARCH)-linux-gnu/glib-2.0/include 
GLIB_LIB					= glib-2.0

# XML
LIBXML2_INCDIR		= $(SYS_PREFIX)/include/libxml2

# File System
SYSFS_INCDIR			=	$(SYS_PREFIX)/include/sysfs

# X11 
XCFLAGS             =  -I/usr/X11R6/include
XLDFLAGS            = 
XMINC               = 
XLIBPATH            =  -L/usr/X11R6/lib64
XLIB                =  -lSM -lICE -lX11
XTLIB               = -lXt
XMLIB               = 
XEXTRALIBS          = 

# Posix Thread Library:
PTHREADLIB          = -lpthread 
PTHREADLIB_INCPATH  = 
PTHREADLIB_LIBPATH  = 
PTHREADLIB_CPPFLAGS =

# Zlib
ZLIB                =    -lz 
ZLIBINCPATH         = 
ZLIBLIBPATH         = 

# Jpeg
JPEGINCPATH         = 
JPEGLIBPATH         = 
JPEGLIB             =  -ljpeg 

# Tiff
TIFFINCPATH         = 
TIFFLIBPATH         = 
TIFFLIB             =  -ltiff 

# Tcl/Tk 
TCLINCDIR           = 
TKINCDIR            = 
TCLTKLIBPATH        = 
TCLTKLIBS           =  -ltk -lm  -ltcl -lm


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
