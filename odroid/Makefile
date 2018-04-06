################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The odroid package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Odroid Processor Package.

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2016-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
RNMAKE_PKG_ROOT	= .

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk


#------------------------------------------------------------------------------
# Subdirectories

RNMAKE_SUBDIRS =  sw

#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS = odroid

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
#       $(topdir)/include. Otherwise, is it really a deliverable header?
#
$(RNMAKE_DIST_HDRS).HDRS.H = $(shell cd include; ls odroid/*.h)


#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE    = $(pkgroot)/make/doxy.conf

#------------------------------------------------------------------------------
# Extras

#EXTRA_TGT_ALL_POST = all-post 

# post 'all' make target
.PHONY: all-post
all-post: show-banner cp-etc-files lcp-ib-files scp-hare-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
