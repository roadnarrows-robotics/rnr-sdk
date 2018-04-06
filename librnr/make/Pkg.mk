################################################################################
#
# make/Pkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief The RoadNarrows Robotics \h_librnr package master makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics Common Library 1

\pkgfile{make/Pkg.mk}

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

_PKG_MK = 1

ifndef RNMAKE_PKG_ROOT
  $(error 'RNMAKE_PKG_ROOT' Not defined in including makefile)
endif

# The Package Definition
RNMAKE_PKG 								 = librnr
RNMAKE_PKG_VERSION_MAJOR   = 1
RNMAKE_PKG_VERSION_MINOR   = 14
RNMAKE_PKG_VERSION_RELEASE = 4
RNMAKE_PKG_VERSION_DATE    = 2018
RNMAKE_PKG_AUTHORS				 = "Robin Knight, Kim Wheeler, Daniel Packard"
RNMAKE_PKG_OWNERS					 = "RoadNarrows LLC"
RNMAKE_PKG_DISCLAIMER			 = \
"See the README and EULA files for any copyright and licensing information."

# Dotted full version number
RNMAKE_PKG_VERSION_DOTTED	= $(RNMAKE_PKG_VERSION_MAJOR).$(RNMAKE_PKG_VERSION_MINOR).$(RNMAKE_PKG_VERSION_RELEASE)

# Concatenated full version number
RNMAKE_PKG_VERSION_CAT = $(RNMAKE_PKG_VERSION_MAJOR)$(RNMAKE_PKG_VERSION_MINOR)$(RNMAKE_PKG_VERSION_RELEASE)

# Package full name
RNMAKE_PKG_FULL_NAME = $(RNMAKE_PKG)-$(RNMAKE_PKG_VERSION_DOTTED)


#------------------------------------------------------------------------------
# Optional Variables and Tweaks

# Package Include Directories
RNMAKE_PKG_INCDIRS = $(RNMAKE_PKG_ROOT)/include

# Package System Include Directories
RNMAKE_PKG_SYS_INCDIRS =

# Package Library Subdirectories
RNMAKE_PKG_LIB_SUBDIRS	= rnr

# Link Library Extra Library Directories (exluding local library)
RNMAKE_PKG_LD_LIBDIRS = 

# Release Files (docs)
RNMAKE_PKG_REL_FILES = VERSION.txt README.md

# CPP flags
RNMAKE_PKG_CPPFLAGS	=

# C flags
RNMAKE_PKG_CFLAGS =

# CXX flags
RNMAKE_PKG_CXXFLAGS	=

# Link flags
RNMAKE_PKG_LDFLAGS =

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
