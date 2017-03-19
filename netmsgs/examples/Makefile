################################################################################
#
# ./examples/Makefile
#

ifdef RNMAKE_DOXY
/*!
\file

$LastChangedDate: 2010-09-22 12:57:22 -0600 (Wed, 22 Sep 2010) $
$Rev: 592 $

\brief
Make NetMsgs example applications.

RN Make System Specific Makefile

\par Usage:
make [RNMAKE_OPTS] [encoding=val] [endian=val] target

\note
Make netmsgs example only after the netmsgs package is installed. There
are circular dependencies not easily resolved.

\author: Robin Knight (robin.knight@roadnarrows.com)

##! \copyright
##!   \h_copy 2009-2017. RoadNarrows LLC.\n
##!   http://www.roadnarrows.com\n
##!   All Rights Reserved

\cond RNMAKE_DOXY
 */
endif

# @EulaBegin@
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# @EulaEnd@
#
################################################################################

#------------------------------------------------------------------------------
# Required

# Package Root Directory
pkgroot		= ..

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

LOC_PGMS         = nmExample

# Libraries to Link With
nmExample.LIBS      = rnr_netmsgs rnr
nmExample.LIBDEPS   = rnr_netmsgs

#------------------------------------------------------------------------------
# Sources

#
# C Source Files
#
nmExample.SRC.C 	= nmExample.c AstroMsgs.c

#------------------------------------------------------------------------------
# Optional Variables 
#


# Subpackage C PreProcessor Flags
EXTRA_CPPFLAGS = -DLOG -DLOGMOD="\"nmExample\""

EXTRA_TGT_ALL 		= xml-make
EXTRA_TGT_DEPS 		= xml-make
EXTRA_TGT_CLEAN 	= xml-clean


#------------------------------------------------------------------------------
# Make Includes

# Default RoadNarrows make system base directory
ifndef rnmake
rnmake := $(realpath $(pkgroot)/../rnmake)
endif

# Include Rules Makefile
include $(rnmake)/Rules.mk

#------------------------------------------------------------------------------
# Extra make targets (place after Rules.mk)

# undef to use installed version
NETMSGSGENPATH	= $(pkgroot)/nmPython/scripts/

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


ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
