################################################################################
#
# Arch/Arch.armdroid.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

\par Architecture:
Android armeabi-v7a (I.E. TI OMAP3 32-bit Gumstix processors)

\par Build Host:
Cross-Compiler

\par Tool-Chain:
arm-linux-androideabi-4.6-*

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.armang.mk}

\pkgauthor{Brent Wilkins,brent@roadnarrows.com}

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_ARCH_ARM_DROID_MK = 1

# This architecture (required)
RNMAKE_ARCH         = armdroid
RNMAKE_ARCH_FQNAME	= arm-android-4.6


#------------------------------------------------------------------------------
# Tool Chain
#------------------------------------------------------------------------------

# Architecture specific include directories
RNMAKE_ARCH_INCDIRS = \
	/opt/pkg/android-ndk-r8d/platforms/android-9/arch-arm/usr/include \
	/opt/pkg/android-ndk-r8d/sources/cxx-stl/gnu-libstdc++/4.6/include

# Architecture specific CPP, C, and C++ Flags
RNMAKE_ARCH_CPPFLAGS =
RNMAKE_ARCH_CFLAGS   =
RNMAKE_ARCH_CXXFLAGS =

# Cross compiler tool chain prefix
CROSS_COMPILE       = arm-linux-androideabi-

# Build Support Commands
AR                  = $(CROSS_COMPILE)ar
RANLIB              = $(CROSS_COMPILE)ranlib
STRIP_LIB						= $(CROSS_COMPILE)strip --strip-debug
STRIP_EXE						= $(CROSS_COMPILE)strip --strip-all


#------------------------------------------------------------------------------
# C Compiler and Options
#------------------------------------------------------------------------------
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

# Make C/CXX Dependencies Command
RNMAKE_MAKEDEPS	= $(CC) $(CFLAGS_DEPS_ONLY)


#------------------------------------------------------------------------------
# C++ Compiler and Options
#------------------------------------------------------------------------------
CXX                 = $(CROSS_COMPILE)g++
CFLAGS_CODEGEN			= -fPIC
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = $(CFLAGS_CODEGEN) \
											$(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
LDFLAGS             = $(COMMON_LDFLAGS) # -Wl,--export-dynamic
LD_LIBPATHS         =
LD_LIBS             =


#------------------------------------------------------------------------------
# Library Archiver/Linker and Options
#------------------------------------------------------------------------------

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


#------------------------------------------------------------------------------
# System and Optional Packages
#------------------------------------------------------------------------------

# Key directories
RNMAKE_SYS_PREFIX 			= $(OE_TMP)/sysroots/armv7a-angstrom-linux-gnueabi/usr
RNMAKE_OPT_PREFIX 			= /opt/xinstall/$(RNMAKE_ARCH)
RNMAKE_SYS_ARCH_LIBDIR 	= $(RNMAKE_SYS_PREFIX)/lib
RNMAKE_SYSFS						=	/sys

# Python
RNMAKE_PYTHON_ENABLED	=	n

# SWIG - Simplified Wrapper and Interface Generator command
RNMAKE_SWIG_ENABLED	=	y
SWIG_CFLAGS 				=	$(CFLAGS_CODEGEN) \
											-pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions
SWIG_INCLUDES				=
SWIG_LDFLAGS 				=	

# OpenCV
RNMAKE_OPENCV_ENABLED =	y
OPENCV_COM_LIBS   		= opencv_core opencv_imgproc opencv_highgui

# PCL
RNMAKE_PCL_ENABLED 	=	n

# GTK (and related)
RNMAKE_GTK_ENABLED 	= y
GTK_VER							= 2.0
GDK_VER							= 2.0
PANGO_VER						= 1.0
ATK_VER							= 1.0
GTK_INCDIR        	= $(RNMAKE_SYS_PREFIX)/include/gtk-$(GTK_VER)
PANGO_INCDIR      	= $(RNMAKE_SYS_PREFIX)/include/pango-$(PANGO_VER)
CAIRO_INCDIR      	= $(RNMAKE_SYS_PREFIX)/include/cairo
ATK_INCDIR        	= $(RNMAKE_SYS_PREFIX)/include/atk-$(ATK_VER)
GDK_PIXBUF_INCDIR 	= $(RNMAKE_SYS_PREFIX)/include/gdk-pixbuf
GTK_LIB_INCDIR			= $(RNMAKE_SYS_ARCH_LIBDIR)/gtk-$(GTK_VER)/include
GTK_LIB					  	= gtk-x11-$(GTK_VER)
GDK_LIB							= gdk-x11-$(GDK_VER)
GDK_PIXBUF_LIB			= gdk_pixbuf-$(GDK_VER)
ATK_LIB							= atk-$(ATK_VER)
PANGO_LIBS					= pango-$(PANGO_VER) \
											pangoft2-$(PANGO_VER) \
											pangocairo-$(PANGO_VER)
CAIRO_LIB						= cairo
FONT_LIBS						= freetype fontconfig
GOBJ_LIBS						= gobject-2.0 gmodule-2.0 gthread-2.0 gio-2.0 rt

# GStreamer
RNMAKE_GST_ENABLED 	= y
GST_VER							= 0.10
GST_INCDIR        	= $(RNMAKE_SYS_PREFIX)/include/gstreamer-0.$(GST_VER)
GST_LIB_INCDIR			= $(RNMAKE_SYS_ARCH_LIBDIR)/$(GST)/include
GST_LIB_IF					= gstinterfaces-$(GST_VER)

# GLib
RNMAKE_GLIB_ENABLED	=	y
GLIB_VER						= 2.0
GLIB_INCDIR			  	= $(RNMAKE_SYS_PREFIX)/include/glib-$(GLIB_VER)
GLIB_LIB_INCDIR			=	$(RNMAKE_SYS_ARCH_LIBDIR)/glib-$(GLIB_VER)/include
GLIB_LIB						= glib-$(GLIB_VER)

# XML
LIBXML2_INCDIR		= $(RNMAKE_SYS_PREFIX)/include/libxml2

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
