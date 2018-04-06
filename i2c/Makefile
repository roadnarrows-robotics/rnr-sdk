################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Top-level \h_i2c makefile.

An rnmake system package specific makefile.

\pkgsynopsis
RoadNarrows \h_i2c Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2005-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS = libi2c cmds pyModules


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= i2c

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
i2c.HDRS.H = \
	rnr/i2c-dev.h \
	rnr/i2c.h \
	rnr/smbus.h


#------------------------------------------------------------------------------
# Documents
#

# Sub[Package] Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Include RNMAKE top-level rules makefile

include $(RNMAKE_ROOT)/Rules.mk



ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif

