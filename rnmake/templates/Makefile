################################################################################
#
# templates/Makefile
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief General [Sub]Pacakge Makefile Template.

Version 3.0

$(RNMAKE_PKG_ROOT)[/subdirs]/Makefile

Copy this file to the package [sub]directory and redefine.
Any define that is not required can be left blank or deleted.

Typically there are 3 distinct flavors used under a package:
- Subdirectory makefile to simply redirect to subdirectories
- Library subdirectory to make libraries
- Program subdirectory to make applications

\pkgsynopsis
RN Make System

\pkgfile{templates/Makefile}

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

#------------------------------------------------------------------------------
# Required

#
# Package Root Directory
#
# Required in each Makefile
#
# Examples: .  .. ../..
#
pkgroot		= .

#
# Top Build Directory
#
# Required in each Makefile
#
topdir 		= $(realpath $(pkgroot)/..)

# Path to RoadNarrows Make System
#
# Package Top Directory.
#
# Required in each Makefile
#
# Examples: /prj/rnmake  $(topdir)/rnmake
#
rnmake		= $(topdir)/rnmake


#------------------------------------------------------------------------------
# Subdirectories

#
# Subdirectories to Build
#
# The order is important. For example, if subdir1 builds an local executable
# needed by subdir3, then subdir1 must appear before subdir3 in the list. 
#
RNMAKE_SUBDIRS = 	subdir1 \
									subdir2 \
									subdir3


#------------------------------------------------------------------------------
# Libraries

#
# Distribution Static Libraries
#
# List of distribution static library tags to make. Distribution components get
# (cross) installed on the system.
#
# Format: <tag> ...
# 					where the fully qualified <tag> depends on the architecture, but
# 					typically gets converted to lib<tag>.a
#
# Note: A library tag can be listed in both the static and shared library tag
# 			list. In this case, both versions are built.
#
RNMAKE_DIST_STLIBS = mylib1 mylib2

#
# Distribution Shared Libraries
#
# List of distribution shared library tags to make. Distribution components get
# (cross) installed on the system.
#
# Format: <tag> ...
# 					where the fully qualified <tag> depends on the architecture, but
# 					typically gets converted to lib<tag>.so
#
# Note: A library tag can be listed in both the static and shared library tag
# 			list. In this case, both versions are built.
#
RNMAKE_DIST_SHLIBS 			= mylib1 

#
# Local Static Libraries (i.e. not in distribution)
#
# List of local static library tags to make. Local components get built, but
# not installed. Local components are place under the directory $(topdir)/loc/.
#
# Format: <tag> ...
# 					where the fully qualified <tag> depends on the architecture, but
# 					typically gets converted to lib<tag>.a
#
# Example:
# 	A set of common files, say in $(topdir)/common, used by various other
# 	package components. By building $(topdir)/loc/lib/libcommon.a, other
# 	programs, etc and used the common files.
#
# Note: Local Shared Libraries are not allowed since installed package 
# 			commponents would require the shared library.
#
RNMAKE_LOC_STLIBS 				= myloclib1


#------------------------------------------------------------------------------
# Programs

RNMAKE_DIST_PGMS = pgm1

#
# Local Program Executables (i.e. not in distribution)
#
# List of local program executable tags to make. Local components get built, but
# not installed. Local components are place under the directory $(topdir)/loc/.
#
# Format: <tag> ...
# 					where the fully qualified <tag> depends on the architecture, but
# 					typically gets converted to <tag> for linux, <tag>.exe for windows.
#
# Example:
# 	A tool hoehoehoe is used by other package components to complete the 
# 	install process.
#
RNMAKE_LOC_PGMS = hoehoehoe

# Libraries to Link With
pgm1.LIBS       = pthread rnr

# Library Dependencies
pgm1.LIBDEPS    = rnr


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
# Source files for libraries and programs.
#
# Format: <tag>.SRC.C
# 					where <tag> is an item from one of the DIST_<x> tag lists for
# 					libraries and programs.
#
mylib1.SRC.C = o.c say.c can.c u.c c.c

#
# C++ Source Files
#
# Source files for libraries and programs.
#
# Format: <tag>.SRC.CXX
# 					where <tag> is an item from one of the DIST_<x> tag lists for
# 					libraries and programs.
#
mylib2.SRC.CXX = file3.cxx file4.cxx

# Local library example
myloclib1.SRC.C = locfile1.c

# Program example
pgm1.SRC.C      = lions.c tigers.c and.c bears.c

# Local program example
pgm2.SRC.C      = what.c a.c joke.c


#------------------------------------------------------------------------------
# Target Special

# Define this...
TGT_SPECIAL			= lookatme

# ...to have Rules.mk call this on make [all]
lookatme:
	@echo "I am so cool"

# Define this...
TGT_INS_SPECIAL	= nohands

# ...to have Rules.mk call this on make install
nohands:


#------------------------------------------------------------------------------
# Optional Variables 
#

#
# Sub[Package] Language (used to facilitate linking)
#
# One of: "C" "C++"
#
# Default is "C"
#
LANG 							= "C"

# Sub[Package] Extra Include Directories
EXTRA_INCDIRS			= /myenv/include /yourenv/include

# Sub[Package] Extra System Include Directories
EXTRA_SYS_INCDIRS	=

# Sub[Package] Extra C PreProcessor Flags
EXTRA_CPPFLAGS		=

# Sub[Package] Extra C Flags
EXTRA_CFLAGS			=

# Sub[Package] Extra CXX Flags
EXTRA_CXXFLAGS		=

# Sub[Package] Extra Library Paths (-L<path> ...)
EXTRA_LD_LIBDIRS	= /myenv/lib

# Sub[Package] Extra External Libraries
EXTRA_LD_LIBS			= -lmylib

# Sub[Package] Extra Link Flags
EXTRA_LDFLAGS			=

# Sub[Package] Extra Release Files (docs)
EXTRA_REL_FILES		=

# Sub[Package] extra 'all' default target 
EXTRA_TGT_ALL = mything

# Sub[Package] extra 'intall' target 
EXTRA_TGT_INSTALL = yourthing

#
# Document SubPackage Name
# 
# Optional document subdirectory where [sub]package documentation will be
# placed.
#
# Installed location: $(docdir)/$(RNMAKE_PKG_FULL_NAME)/$(DOC_SUBDIR)
#
# Example: /usr/share/doc/pkg-1.3.0/mydocdir/
#
DOC_SUBDIR				=

# Sub[Package] Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE		= $(topdir)/make/doxy-mypgm.conf

#------------------------------------------------------------------------------
# The RNMake Rules
#

# Include Rules Makefile
include $(rnmake)/Rules.mk


#------------------------------------------------------------------------------
# Extra Targets (after Rules.mk)
#

mything:
	@echo "do my default thing"


yourthing:
	@echo "do your intall thing"
