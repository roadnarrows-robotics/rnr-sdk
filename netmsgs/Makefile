################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The top-level \h_netmsgs package makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Network Messaging Package

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2009-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS= libnetmsgs nmPython

#------------------------------------------------------------------------------
# Distribution Header List Tags

# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= netmsgs

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
netmsgs.HDRS.H	=	rnr/netmsgs.h

#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras
#

EXTRA_TGT_ALL_POST	= all-post
EXTRA_TGT_DOC 			= doc-python
EXTRA_TGT_CLEAN			= clean-python
EXTRA_TGT_DISTCLEAN	= distclean-python

SHARE_FILES = share/netmsgs.dtd

# Install botsense configuration and message definition files
.PHONY: all-post
all-post: show-banner cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share files to $(DISTDIR_SHARE))
	$(call copySrcDirRel,$(SHARE_FILES),$(DISTDIR_SHARE))

.PHONY: doc-python clean-python distclean-python
doc-python clean-python distclean-python:
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C nmPython $(@)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras (after top-level)
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
