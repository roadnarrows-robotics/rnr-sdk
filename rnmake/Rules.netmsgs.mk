################################################################################
#
# Package: 	RN Make System 
#
# File:			Rules.netmsgs.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file to make RoadNarrows NetMsgs generated file(s).

Include this file into each local make file (usually at the bottom).\n

\par Key Variables:
	\li NETMSGS_XML_FILES	- list of RN NetMsgs XML specification files.
	\li NETMSGS_H_DIR			- generated .h files output directory.
													default: . (current directory)
	\li NETMSGS_C_DIR			- generated .c files output directory.
													default: . (current directory)
	\li NETMSGS_CFLAGS		- additional netmsgs c flags.
	\li NETMSGS_PY_DIR		- generated .py files output directory.
													default: . (current directory)
	\li NETMSGS_PYFLAGS		- additional netmsgs python flags.
	\li NETMSGS_SHARE_DIR	- share directory to copy XML files.
													default: $(pkgroot)/share

$LastChangedDate: 2012-11-05 11:05:42 -0700 (Mon, 05 Nov 2012) $
$Rev: 2507 $

\author Robin Knight (robin.knight@roadnarrows.com)

\par Copyright:
(C) 2010.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved

\cond RNMAKE_DOXY
 */
endif
#
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
#
################################################################################

# NetMsgs source generator command
NETMSGSGEN					= netmsgsgen

#
# Files
#
NETMSGS_BASES				= $(basename $(NETMSGS_XML_FILES))

NETMSGS_H_DIR      ?= .
NETMSGS_H_FILES 		= $(addprefix $(NETMSGS_H_DIR)/,\
											$(addsuffix .h,$(NETMSGS_BASES)))

NETMSGS_C_DIR      ?= .
NETMSGS_C_FILES 		= $(addprefix $(NETMSGS_C_DIR)/,\
											$(addsuffix .c,$(NETMSGS_BASES)))

NETMSGS_PY_DIR		 ?= .
NETMSGS_PY_FILES 		= $(addprefix $(NETMSGS_PY_DIR)/,\
											$(addsuffix .py,$(NETMSGS_BASES)))

NETMSGS_SHARE_DIR	 ?= $(pkgroot)/share
NETMSGS_SHARE_FILES	= $(addprefix $(NETMSGS_SHARE_DIR)/,$(NETMSGS_XML_FILES))

define xml2c
	$(addprefix $(NETMSGS_C_DIR)/,$(addsuffix .c,$(basename $(1))))
endef

.PHONY: netmsgs-all netmsgs-all-c netmsgs-all-py netmsgs-all-share
netmsgs-all: netmsgs-all-c netmsgs-all-py netmsgs-all-share

netmsgs-all-c: 	$(NETMSGS_H_FILES)

netmsgs-all-py: $(NETMSGS_PY_FILES)

netmsgs-all-share: $(NETMSGS_SHARE_FILES)

.PHONY: netmsgs-clean
netmsgs-clean:
	$(RM) $(NETMSGS_H_FILES)
	$(RM) $(NETMSGS_C_FILES)
	$(RM) $(NETMSGS_PY_FILES)
	$(RM) $(NETMSGS_SHARE_FILES)

# Generate NetMsgs C .h and .c message files from XML
$(NETMSGS_H_DIR)/%.h : %.xml
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(NETMSGSGEN) --lang=c --xml=$(<) $(NETMSGS_CFLAGS) $(@) $(call xml2c,$(<))

# Generate NetMsgs Python .py files from XML
$(NETMSGS_PY_DIR)/%.py : %.xml
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(NETMSGSGEN) --lang=python --xml=$(<) $(NETMSGS_PYFLAGS) $(@)

# Copy NetMsgs XML specification files to share
$(NETMSGS_SHARE_DIR)/%.xml : %.xml
	@printf "\n"
	@printf "$(color_tgt_file)     $(<)$(color_end)\n"
	$(CP) $(<) $(@)

ifdef RNMAKE_DOXY
/*! \endcond RNMAKE_DOXY */
endif
