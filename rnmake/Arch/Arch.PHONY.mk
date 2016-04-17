################################################################################
#
# Package: RN Make System 
# File:    Arch.PHONY.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System Architecture Makefile

\par Architecture:
Phony test architecture (really i386)

\par Build Host:
Native or Cross-Compile

$LastChangedDate: 2012-07-17 11:59:42 -0600 (Tue, 17 Jul 2012) $
$Rev: 2091 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2010-2011.  RoadNarrows LLC.
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
ARCH         				= phony
ARCH_FQNAME         = phony-test-arch

# RoadNarrows Install Prefix (override as necessary)
ifndef prefix
	ifdef prefix_root
		prefix          = $(prefix_root)/$(ARCH)
	else
		prefix					= $(topdir)/xinstall/$(ARCH)
	endif
endif

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
CFLAGS_CODEGEN			= -fPIC -m32
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
CXX                 = g++
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = -m32 $(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING)

# LD (linker)
LD_CC              	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
# -Wl,--export-dynamic
LDFLAGS             = -m32
LD_LIBPATHS         = 
LD_LIBS             =

# Static Libs
STLIB_LD            = ${AR} cr
STLIB_PREFIX        = lib
STLIB_SUFFIX        = .a

# Shared Libs
SHLIB_LD            = $(CC) -shared -m32
SHLIB_PREFIX        = lib
SHLIB_SUFFIX        = .so
SHLIB_LD_EXTRAS     =
SHLIB_LD_FLAGS      =
SHLIB_LD_LIBS       = ${LIBS}
SHLIB_CFLAGS        = -fPIC

# Dynamically Linked Libraries
DLLIB_LD            = $(CC) -shared -m32
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
