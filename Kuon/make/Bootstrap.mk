################################################################################
#
# ./make/Bootstrap.mk
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Bootstrap the RN Make System location and define any simple
package-specific environment.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_kuon Large Mobile Robot Package

\pkgfile{make/Bootstrap.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

_BOOTSTRAP_MK = 1

ifndef RNMAKE_PKG_ROOT
  $(error 'RNMAKE_PKG_ROOT' Not defined in including makefile)
endif

#------------------------------------------------------------------------------
# Required

# 'make rnmake=<path> ...' or RNMAKE_ROOT
rnmake ?= $(RNMAKE_ROOT)

# must be defined and non-empty
ifeq ($(rnmake),)
  $(error 'RNMAKE_ROOT' environment variable not specified)
endif

# make absolute root name (empty string returned if directory does not exist)
RNMAKE_ROOT := $(realpath $(rnmake))

# directory required
ifeq ($(RNMAKE_ROOT),)
  $(error 'RNMAKE_ROOT=$(rnmake)': No such directory)
endif

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
