################################################################################
#
# Arch/Arch.cygwin-nat.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

Makefile tested on Cygwin v1.7.5 with GCC 4.3.

\par Architecture:
Cygwin 'OS' on Windows 32-bit platforms.

\par Build Host:
Native

\par Tool-Chain:
gcc

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.cygwin-nat.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_ARCH_CYGWIN_NAT_MK = 1

# This architecture (required)
# Note: These names should be identical those in Arch.cygwin.mk
RNMAKE_ARCH         = cygwin
RNMAKE_ARCH_FQNAME	= i686-pc-cygwin


#------------------------------------------------------------------------------
# Tool Chain
#------------------------------------------------------------------------------

# Architecture Include Directories
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
CFLAGS_CODEGEN			= -m32
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
CXX                 = g++
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = -m32 $(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD_CC              	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
# -Wl,--export-dynamic
LDFLAGS             = -m32
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
SHLIB_LD            = $(CC) -shared -m32
SHLIB_PREFIX        = lib
SHLIB_SUFFIX        = .dll
SHLIB_LD_EXTRAS     =
SHLIB_LD_FLAGS      =
SHLIB_LD_LIBS       = ${LIBS}
SHLIB_CFLAGS        = 

# Dynamically Linked Libraries
DLLIB_LD            = $(CC) -shared -m32
DLLIB_PREFIX        = lib
DLLIB_SUFFIX        = .dll
DLLIB_LD_NOSTART		= 
DLLIB_LD_EXTRAS     =
DLLIB_LD_FLAGS      =
DLLIB_LD_LIBS       = ${LIBS}
DLLIB_CFLAGS        = 
DLLIB_APP_CFLAGS    = -rdynamic
DLLIB               = dl


#------------------------------------------------------------------------------
# System and Optional Packages
#------------------------------------------------------------------------------

# Python
RNMAKE_PYTHON_ENABLED	=	y

# SWIG - Simplified Wrapper and Interface Generator command
RNMAKE_SWIG_ENABLED	=	y
SWIG_CFLAGS 				=	$(CFLAGS_CODEGEN) \
											-pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions
SWIG_INCLUDES				= 
SWIG_LDFLAGS 				=	

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
