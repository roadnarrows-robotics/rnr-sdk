################################################################################
#
# Arch/Arch.x86_64.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

\par Architecture:
Linux (Posix) AMD and Intell compatible 64-bit.

\par Build Host:
Native or cross-compile with appropriate flags.

\par Tool-Chain:
gcc

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.x86_64.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_ARCH_X86_64_MK = 1

# This architecture (required)
RNMAKE_ARCH         = x86_64
RNMAKE_ARCH_FQNAME  = x86_64-linux-gnu


#------------------------------------------------------------------------------
# Tool Chain
#------------------------------------------------------------------------------

# Architecture specific include directories
RNMAKE_ARCH_INCDIRS =

# Architecture specific CPP, C, and C++ Flags
RNMAKE_ARCH_CPPFLAGS =
RNMAKE_ARCH_CFLAGS   =
RNMAKE_ARCH_CXXFLAGS =

# Build Support Commands
AR                  = ar
RANLIB              = ranlib
STRIP_LIB						= strip --strip-debug
STRIP_EXE						= strip --strip-all


#------------------------------------------------------------------------------
# C Compiler and Options
#------------------------------------------------------------------------------
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

# Make C/CXX Dependencies Command
RNMAKE_MAKEDEPS	= $(CC) $(CFLAGS_DEPS_ONLY)


#------------------------------------------------------------------------------
# C++ Compiler and Options
#------------------------------------------------------------------------------
CXX                 = c++
CXXFLAGS_CODEGEN		= -fPIC -m64
CXXFLAGS_STD				= -std=c++17
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = $(CXXFLAGS_CODEGEN) \
											$(CXXFLAGS_STD) \
											$(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)


#------------------------------------------------------------------------------
# CUDA and Options
#------------------------------------------------------------------------------
RNMAKE_CUDA_ENABLED = y
CUDA                = nvcc
CUDACFLAGS_CODEGEN  = 
CUDAFLAGS_DEBUG     = -g
CUDAFLAGS_OPTIMIZE  = -O2
CUDAFLAGS_CPP_ONLY  = -E
CUDAFLAGS           = $(CUDAFLAGS_CODEGEN) \
											$(CUDAFLAGS_DEBUG) \
                      $(CUDAFLAGS_OPTIMIZE) \
                      $(CUDAFLAGS_WARNING)


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD_CUDA             = $(CUDA)
LD									= $(LD_CC)
LDFLAGS             = # -Wl,--export-dynamic
LD_LIBPATHS         =
LD_LIBS             =


#------------------------------------------------------------------------------
# Library Archiver/Linker and Options
#------------------------------------------------------------------------------

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


#------------------------------------------------------------------------------
# System and Optional Packages
#------------------------------------------------------------------------------

# Key directories
RNMAKE_SYS_PREFIX 			= /usr
RNMAKE_OPT_PREFIX 			= /opt/xinstall/$(RNMAKE_ARCH)
RNMAKE_SYS_ARCH_LIBDIR 	= $(RNMAKE_SYS_PREFIX)/lib/$(RNMAKE_ARCH_FQNAME)
RNMAKE_SYSFS						=	/sys

# Python
RNMAKE_PYTHON_ENABLED	=	y

# SWIG - Simplified Wrapper and Interface Generator command
RNMAKE_SWIG_ENABLED	=	y
SWIG_CFLAGS_WARNING = -Wno-sign-conversion
SWIG_CFLAGS_LD      = -Wl,-O1 -Wl,-Bsymbolic-functions
SWIG_CFLAGS 				=	$(CFLAGS_CODEGEN) -pthread -shared \
											$(SWIG_CFLAGS_WARNING) \
											$(SWIG_CFLAGS_LD)
SWIG_INCLUDES				= 
SWIG_LDFLAGS 				=	

# OpenCV
RNMAKE_OPENCV_ENABLED =	y
OPENCV_COM_LIBS   		= opencv_core opencv_imgproc opencv_highgui

# PCL
RNMAKE_PCL_ENABLED 	=	y
PCL_VER 						= 1.7
PCL_INCDIR					= $(RNMAKE_SYS_PREFIX)/include/pcl-$(PCL_VER)
ONI_INCDIR					= $(RNMAKE_SYS_PREFIX)/include/ni

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
GDK_PIXBUF_INCDIR 	= $(RNMAKE_SYS_PREFIX)/include/gdk-pixbuf-$(GDK_VER)
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
GST_VER							= 1.0
GST									= gstreamer-$(GST_VER)
GST_INCDIR        	= $(RNMAKE_SYS_PREFIX)/include/$(GST)
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


# X11 
RNMAKE_X11_ENABLED	=	y
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
