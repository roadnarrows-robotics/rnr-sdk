################################################################################
#
# Rules.netmsgs.mk
#
ifdef RNMAKE_DOXY
/*! 
\file 

\brief Special rules file to make RoadNarrows NetMsgs generated file(s).

Include this file in each local make file (usually near the bottom).

\par Key RNMAKE Variables:
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
													default: $(RNMAKE_PKG_ROOT)/share

\pkgsynopsis
RN Make System

\pkgfile{Rules.netmsgs.mk}

\pkgauthor{Robin Knight,robin.knight@roadnarrows.com}

\pkgcopyright{2010-2018,RoadNarrows LLC,http://www.roadnarrows.com}

\license{MIT}

\EulaBegin
\EulaEnd

\cond RNMAKE_DOXY
 */
endif
#
################################################################################

_RULES_NETMSGS_MK = 1

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

NETMSGS_SHARE_DIR	 ?= $(RNMAKE_PKG_ROOT)/share
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
