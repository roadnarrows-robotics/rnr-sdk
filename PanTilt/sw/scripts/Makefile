################################################################################
#
# ./sw/scripts/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The \h_pan_tilt executable scripts makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_pan_tilt Robotic Platform Package

\pkgfile{sw/scripts/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

#------------------------------------------------------------------------------
# Extras 

EXTRA_TGT_ALL = all-scripts

SCRIPTS = 

.PHONY: all-scripts
all-scripts: cp-script-files

.PHONY: echo-all-scripts
echo-all-scripts:
	$(call printEchoTgtGoalDesc,Copying scripts to $(DISTDIR_BIN))

.PHONY: cp-script-files
cp-script-files:
	$(call printGoalDesc,$(@),Copying script files to $(DISTDIR_BIN))
	$(call copySrcDir,$(SCRIPTS),$(DISTDIR_BIN))


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

# include extras (must include after top-level)
include $(RNMAKE_ROOT)/Extras.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
