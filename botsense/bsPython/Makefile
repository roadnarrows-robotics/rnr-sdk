################################################################################
#
# ./bsPython/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_botsense python modules, extension modules, and scripts makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{bsPython/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS = src


#------------------------------------------------------------------------------
# Extras
#

EXTRA_TGT_ALL_POST 		= all-python-post
EXTRA_TGT_DOC 				= doc-python
EXTRA_TGT_CLEAN 			= clean-python
EXTRA_TGT_DISTCLEAN 	= distclean-python

.PHONY: all-python-post
all-python-post: python-all

.PHONY: doc-python
doc-python: python-doc

.PHONY: clean-python
clean-python: python-clean

.PHONY: distclean-python
distclean-python: python-distclean


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include python rules
include $(rnmake)/Rules.python.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
