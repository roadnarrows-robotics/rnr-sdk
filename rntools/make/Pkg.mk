################################################################################
#
# make/Pkg.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief The RoadNarrows Robotics rntools package master makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows Robotics Tools to Manage Projects, Packages, and the Business.

\pkgfile{make/Pkg.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG 								 = rntools
RNMAKE_PKG_VERSION_MAJOR   = 2
RNMAKE_PKG_VERSION_MINOR   = 0
RNMAKE_PKG_VERSION_RELEASE = 0
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

# Release Files (docs)
RNMAKE_PKG_REL_FILES = README.md


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
