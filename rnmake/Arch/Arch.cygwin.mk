################################################################################
#
# Arch/Arch.cygwin.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

\par Architecture:
Cygwin 'OS' on Windows 32-bit platforms.

\par Build Host:
Cross-Compiler

\par Tool-Chain:
i686-pc-cygwin-

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.cygwin.mk}

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

_ARCH_CYGWIN_MK = 1

# This architecture (required)
# Note: These names should be identical those in Arch.cygwin-nat.mk
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

# Cross compiler tool chain prefix
CROSS_COMPILE       = i686-pc-cygwin-

# Locations
CYGWIN_DIR					= /opt/xcc/cygwin
CYGWIN_DIR_INC			= $(CYGWIN_DIR)/usr/include

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
CFLAGS_WARNING      = -Wall -Wconversion -Wno-implicit-int
CFLAGS_CPP_ONLY     = -E
CFLAGS_DEPS_ONLY    = -M
CFLAGS              = $(CFLAGS_CODEGEN) \
											$(CFLAGS_DEBUG) \
                      $(CFLAGS_OPTIMIZE) \
                      $(CFLAGS_WARNING) \
											-I$(CYGWIN_DIR_INC) -I$(CYGWIN_DIR_INC)/w32api

# Make C/CXX Dependencies Command
RNMAKE_MAKEDEPS	= $(CC) $(CFLAGS_DEPS_ONLY)


#------------------------------------------------------------------------------
# C++ Compiler and Options
#------------------------------------------------------------------------------
CXX                 = $(CROSS_COMPILE)c++
CXXFLAGS_DEBUG      = -g
CXXFLAGS_OPTIMIZE   = -O2
CXXFLAGS_CPP_ONLY   = -E
CXXFLAGS            = $(CXXFLAGS_DEBUG) \
                      $(CXXFLAGS_OPTIMIZE) \
                      $(CXXFLAGS_WARNING) \
											-I$(CYGWIN_DIR_INC) -I$(CYGWIN_DIR_INC)/w32api


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
LDFLAGS             = # -Wl,--export-dynamic
LD_LIBPATHS         = -L$(CYGWIN_DIR)/usr/lib -L$(CYGWIN_DIR)/usr/lib/w32api
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

# Posix Thread Library
PTHREADLIB          = -lpthread 
PTHREADLIB_INCPATH  = 
PTHREADLIB_LIBPATH  = 
PTHREADLIB_CPPFLAGS =

# Jpeg Library
JPEGINCPATH         = 
JPEGLIBPATH         = 
JPEGLIB             =  -ljpeg 

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
