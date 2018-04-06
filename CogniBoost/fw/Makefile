################################################################################
#
# ./fw/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_cogniboost package firmware makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_cogniboost Package

\pkgfile{X/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 
\pkgauthor{Daniel Packard,daniel@roadnarrows.com} 
\pkgauthor{Brent Wilkins,brent@roadnarrows.com} 

\pkgcopyright{2011-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

 * \cond RNMAKE_DOXY
 */
endif
# 
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
RNMAKE_PKG_ROOT	= ..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS = # Diag


#------------------------------------------------------------------------------
# Extras

EXTRA_MAKE_FLAGS = -f Makefile.atmega32uc3.mk


# -------------------------------------------------------------------------
# Target:	Last Resort Default - Kinda
# Desc: 	Recursively make subdirectories with target(s).
# -------------------------------------------------------------------------
install-fw distclean-fw:
	@for fwdir in $(RNMAKE_SUBDIRS); \
	do \
		$(MAKE) $(EXTRA_MAKE_FLAGS) -C $${fwdir} $(@); \
	done


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
