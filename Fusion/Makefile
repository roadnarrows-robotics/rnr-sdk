################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_fusion package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_fusion Python Demonstration Package

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
# Documentation 

# Sub[Package] Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras

EXTRA_TGT_ALL 				= all-python
EXTRA_TGT_DOC 				= doc-python
EXTRA_TGT_CLEAN 			= clean-python
EXTRA_TGT_DISTCLEAN 	= distclean-python

#------------------------------------------------------------------------------
# Extras

.PHONY: all-python
all-python: python-all

.PHONY: doc-python
doc-python: python-doc

.PHONY: clean-python
clean-python: python-clean

.PHONY: distclean-python
distclean-python: python-distclean
	$(RM) $(pkgroot)/build


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include python rules
include $(RNMAKE_ROOT)/Rules.python.mk

