################################################################################
#
# make/Pkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief RoadNarrows Robotics \h_eudoxus package master makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics \h_eudoxus 3D Scanner Package

\pkgfile{make/Pkg.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2012-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG 								 = Eudoxus
RNMAKE_PKG_VERSION_MAJOR   = 3
RNMAKE_PKG_VERSION_MINOR   = 1
RNMAKE_PKG_VERSION_RELEASE = 2
RNMAKE_PKG_VERSION_DATE    = 2018
RNMAKE_PKG_AUTHORS				 = "Robin Knight"
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
RNMAKE_PKG_INCDIRS = 	$(RNMAKE_PKG_ROOT)/include \
											$(RNMAKE_PKG_ROOT)/sw/External/LibJPEG \
											$(RNMAKE_PKG_ROOT)/sw/External/TinyXml

# System and Third Party Include Directories
RNMAKE_PKG_SYS_INCDIRS 	= $(GLIB_INCDIR) \
												 	$(GLIB_LIB_INCDIR) \
													$(PCL_INCDIR) \
													$(ONI_INCDIR) \
													$(GST_INCDIR) \
													$(GST_LIB_INCDIR)

# Package Library Subdirectories
RNMAKE_PKG_LIB_SUBDIRS = = gst

# Installed Packages Library Subdirectories
RNMAKE_PKG_LIB_INS_SUBDIRS = rnr

# External System and Third Party Library Directories
RNMAKE_PKG_LD_SYS_LIBDIRS = $(RNMAKE_OPT_PREFIX)/lib

# Release Files (docs)
RNMAKE_PKG_REL_FILES = VERSION.txt README.md

# CPP flags
RNMAKE_PKG_CPPFLAGS	= -DXN_EXPORTS

# C flags
RNMAKE_PKG_CFLAGS	=

# CXX flags
RNMAKE_PKG_CXXFLAGS	= -Wno-deprecated-declarations \
											-Wno-deprecated

# Link flags
RNMAKE_PKG_LDFLAGS=

ifeq ($(strip $(RNMAKE_PCL_ENABLED)),y)
	PCL_LIBS_BASE = pcl_common pcl_io
	ONI_LIBS 			= OpenNI
endif

ifeq ($(strip $(RNMAKE_GST_ENABLED)),y)
	GST_LIBS_BASE = $(GST) gstbase-$(GST_VER)
	GST_LIBS_VID  = gstvideo-$(GST_VER)
endif

ifeq ($(strip $(RNMAKE_GLIB_ENABLED)),y)
	GLIB_LIBS_BASE = $(GLIB_LIB) gobject-2.0
endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
