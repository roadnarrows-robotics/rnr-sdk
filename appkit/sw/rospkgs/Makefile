################################################################################
#
# ./sw/rospkgs/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief The ROS packages makefile.

DEPRECATED

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics Application Kit

\pkgfile{sw/rospkgs/Makefile}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com} 

\pkgcopyright{2013-2018,RoadNarrows LLC,http://www.roadnarrows.com}

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
# Libraries

# Distribution Static Libraries
RNMAKE_DIST_STLIBS = 

# Distribution Shared Libraries
RNMAKE_DIST_SHLIBS = 


#------------------------------------------------------------------------------
# Extras

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"appkit-rospkg\""

#EXTRA_TGT_DEPS = extra-deps
#EXTRA_TGT_CLEAN = extra-clean

.PHONY: extra-deps
extra-deps:
	@test -h $(BSMSGS_C_FILE) || $(SYMLINK) $(BSMSGS_SRC_DIR)/$(BSMSGS_C_FILE) .

.PHONY: extra-clean
extra-clean: 
	@test ! -h $(BSMSGS_C_FILE) || $(UNLINK) $(BSMSGS_C_FILE)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
