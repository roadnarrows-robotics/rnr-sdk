################################################################################
#
# Arch/Arch.armpxa.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Make System architecture makefile.

\par Architecture:
Familiar (Posix) Linux XScale PXA 255+ 32-bit Arm.

\par Build Host:
Cross-Compiler

\par Tool-Chain:
arm-xscale-linux-gnu-*

\pkgsynopsis
RN Make System

\pkgfile{Arch/Arch.armpxa.mk}

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

_ARCH_ARM_PXA_MK = 1

# This architecture (required)
RNMAKE_ARCH         = armpxa
RNMAKE_ARCH_FQNAME	= arm-pxa-familiar-linux-gnu


#------------------------------------------------------------------------------
# Tool Chain
#------------------------------------------------------------------------------

# Architecture specific include directories
RNMAKE_ARCH_INCDIRS =

# Architecture specific CPP, C, and C++ Flags
RNMAKE_ARCH_CPPFLAGS =
RNMAKE_ARCH_CFLAGS   =
RNMAKE_ARCH_CXXFLAGS =

# Cross compiler tool chain prefix
CROSS_COMPILE       = arm-xscale-linux-gnu-

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
                      $(CFLAGS_WARNING)

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
                      $(CXXFLAGS_WARNING)


#------------------------------------------------------------------------------
# Linker and Options
#------------------------------------------------------------------------------
LD_CC             	= $(CC)
LD_CXX              = $(CXX)
LD									= $(LD_CC)
LDFLAGS             = # -Wl,--export-dynamic
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
RNMAKE_PYTHON_ENABLED	=	n

# SWIG - Simplified Wrapper and Interface Generator command
RNMAKE_SWIG_ENABLED	=	n

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
