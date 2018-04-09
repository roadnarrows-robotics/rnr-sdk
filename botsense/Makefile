################################################################################
#
# ./Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief \h_botsense package top-level makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2007-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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

RNMAKE_SUBDIRS = libbotsense bsProxy bsModules bsPython examples


#------------------------------------------------------------------------------
# Interface Headers

#
# Distribution Header List Tags
#
# List of tags to lists of header files slated to be distributed and installed.
#
RNMAKE_DIST_HDRS	= botsense libbotsense

#
# Distribution Headers
#
# Format: <tag>.HDRS.H = [<subdirs>/]<hdr>.h ...
#
# Note: Any header file that requires distribution is expected to be in 
# 			$(topdir)/include. Otherwise, is it really a deliverable header?
#
botsense.HDRS.H	=	botsense/BotSense.h \
									botsense/libBotSense.h \
									botsense/bsProxyModIF.h

libbotsense.HDRS.H =	botsense/bsProxyMsgs.h


#------------------------------------------------------------------------------
# Documentation 

# Doxygen Configuration File
RNMAKE_DOXY_CONF_FILE	= $(RNMAKE_PKG_ROOT)/make/doxy.conf


#------------------------------------------------------------------------------
# Extras
#

EXTRA_TGT_DOC       = doc-python
EXTRA_TGT_ALL_POST	= all-post
EXTRA_TGT_CLEAN     = clean-python
EXTRA_TGT_DISTCLEAN	= distclean-python

SHARE_FILES = share/*.xml share/*.dtd share/msgs/*.xml share/msgs/*.md
 
# Post 'all' make target
.PHONY: all-post
all-post: show-banner cp-share-files

.PHONY: show-banner
show-banner:
	$(call printDirBanner,,all-post)
	@printf "Pre-install Configuration\n"

.PHONY: cp-share-files
cp-share-files:
	$(call printGoalDesc,$(@),Copying share files to $(DISTDIR_SHARE))
	$(call copySrcDirRel,$(SHARE_FILES),$(DISTDIR_SHARE),share)

.PHONY: doc-python clean-python distclean-python
doc-python clean-python distclean-python:
	@$(MAKE) $(EXTRA_MAKE_FLAGS) -C bsPython $(@)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
