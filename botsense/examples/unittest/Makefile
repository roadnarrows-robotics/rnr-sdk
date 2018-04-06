################################################################################
#
# ./examples/unittest/Makefile
#
ifdef RNMAKE_DOXY
/*!
\file

\brief Python unit test application makefile.

RN Make System Specific Makefile

\pkgsynopsis
RoadNarrows Robotics \h_botsense Client-Server Proxied Services Framework

\pkgfile{examples/unittest/Makefile}

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
RNMAKE_PKG_ROOT	= ../..

# Bootstrap package within RN Make System
include $(RNMAKE_PKG_ROOT)/make/Bootstrap.mk

# command line override
ifndef encoding
	encoding=itv
endif
ifndef endian
	endian=big
endif

XML_OVERRIDES = encoding=$(encoding),endian=$(endian)


#------------------------------------------------------------------------------
# Programs

RNMAKE_TEST_PGMS = nmExample

# Libraries to Link With
nmExample.LIBS    = netmsgs rnr
nmExample.LIBDEPS = netmsgs


#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
nmExample.SRC.C 	= nmExample.c AstroMsgs.c


#------------------------------------------------------------------------------
# Extras
#

# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"nmExample\""

EXTRA_TGT_ALL 		= xml-make
EXTRA_TGT_DEPS 		= xml-make
EXTRA_TGT_CLEAN 	= xml-clean

# undef to use installed version
NETMSGSGENPATH	= ../NetMsgs/scripts/

# the command
NETMSGSGEN			= netmsgsgen

XML_H_DIR				= .
XML_IN_FILES		= AstroMsgs.xml
XML_OUT_H_FILES	= $(addprefix $(XML_H_DIR)/,\
									  $(addsuffix .h,$(basename $(XML_IN_FILES))))
XML_OUT_C_FILES = $(addsuffix .c,$(basename $(XML_IN_FILES)))

define XML_TO_H
	$(addprefix $(XML_H_DIR)/,$(addsuffix .h,$(basename $(1))))
endef

define XML_TO_C
	$(addsuffix .c,$(basename $(1)))
endef

.PHONY: xml-make
xml-make: $(XML_OUT_H_FILES)

# Application specific message h and c files
$(XML_H_DIR)/%.h : %.xml
	@echo ""
	@echo "    $(<)"
	$(NETMSGSGENPATH)$(NETMSGSGEN) --lang=c --overrides="$(XML_OVERRIDES)" \
		--xml=$(<) $(@) $(call XML_TO_C,$(<))

.PHONY: xml-h-clean
xml-clean:
	$(RM) $(XML_OUT_H_FILES)
	$(RM) $(XML_OUT_C_FILES)


#------------------------------------------------------------------------------
# Include RNMAKE rules makefile(s)

# include top-level rules
include $(RNMAKE_ROOT)/Rules.mk


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
